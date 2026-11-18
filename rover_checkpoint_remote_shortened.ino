// Serial Monitor (9600 baud) commands:
// d = drive 28 in   f = forward 5 in   b = back up 5 in   l = turn left 90   r = turn right 90
// s = sensor sweep  i = IR beacon watch   x = stop
// IR remote: up = f   down = b   left = l   right = r   1 = s   OK = x

#include <Servo.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

// Pins
const byte LEFT_MOTOR_PINS[4]  = {4, 5, 6, 7};
const byte RIGHT_MOTOR_PINS[4] = {A0, A1, A2, A3};
const byte SERVO_PIN           = 9;
const byte ULTRASONIC_TRIG_PIN = A4;
const byte ULTRASONIC_ECHO_PIN = A5;
const byte IR_RECEIVER_PIN     = 3;

// Motion tuning
const unsigned int STEP_INTERVAL_US = 1500; // 600 to 2000; raise if motors stall

const float STEPS_PER_INCH   = 507.0;
const float STEPS_PER_90_DEG = 1175.0;

const int LEFT_MOTOR_FORWARD  = +1; // flip sign if wheel runs backward
const int RIGHT_MOTOR_FORWARD = -1;

// Sensing tuning
const byte SERVO_ANGLE_LEFT  = 170;
const byte SERVO_ANGLE_AHEAD = 90;
const byte SERVO_ANGLE_RIGHT = 10;

const unsigned int  SERVO_SETTLE_MS  = 500;
const unsigned int  PING_GAP_MS      = 60;
const unsigned long ECHO_TIMEOUT_US  = 25000;
const unsigned int  TRIGGER_PULSE_US = 12;
const float         US_PER_CM        = 58.0;
const float         NO_ECHO_CM       = 400.0;

const unsigned int IR_LISTEN_MS  = 250;
const unsigned int IR_MIN_PULSES = 5;

// IR remote buttons (NEC command codes; unused buttons print their code to the Serial Monitor)
const byte BUTTON_UP    = 0x46;
const byte BUTTON_DOWN  = 0x15;
const byte BUTTON_LEFT  = 0x44;
const byte BUTTON_RIGHT = 0x43;
const byte BUTTON_OK    = 0x40;
const byte BUTTON_1     = 0x16;
const byte NO_BUTTON    = 0xFF;

// IR remote decoding (NEC timing between falling edges, in microseconds)
const unsigned int NEC_START_MIN_US = 12000;
const unsigned int NEC_START_MAX_US = 15000;
const unsigned int NEC_ZERO_MIN_US  = 800;
const unsigned int NEC_ZERO_MAX_US  = 1500;
const unsigned int NEC_ONE_MIN_US   = 1800;
const unsigned int NEC_ONE_MAX_US   = 2700;
const byte         NEC_BITS         = 32;

// Test settings
const float CHECKPOINT_DRIVE_IN         = 28.0;
const float SHORT_DRIVE_IN              = 5.0;   // forward/backward distance for f, b and the remote arrows
const bool  AUTO_MOTION_DEMO            = false; // true = run d, l, r by itself after power-on
const unsigned int  AUTO_START_DELAY_MS = 5000;
const unsigned int  AUTO_PAUSE_MS       = 2000;
const unsigned long SERIAL_BAUD         = 9600;

// ============================================================================
// GLOBAL STATE: directions, state machine, shared ISR variables
// ============================================================================

const byte LOOK_LEFT  = 0;
const byte LOOK_AHEAD = 1;
const byte LOOK_RIGHT = 2;

const byte SCAN_ORDER[3] = {LOOK_LEFT, LOOK_RIGHT, LOOK_AHEAD};

enum TestState {
  STATE_IDLE,
  STATE_MOVING,
  STATE_AIM_SENSOR,
  STATE_MEASURE,
  STATE_IR_WATCH
};
TestState currentState = STATE_IDLE;

// Half step coil sequence for the 28BYJ-48 stepper motors
const byte COIL_PATTERN[8][4] = {
  {1, 0, 0, 0},
  {1, 1, 0, 0},
  {0, 1, 0, 0},
  {0, 1, 1, 0},
  {0, 0, 1, 0},
  {0, 0, 1, 1},
  {0, 0, 0, 1},
  {1, 0, 0, 1}
};

const byte TIMER2_COMPARE_VALUE = (STEP_INTERVAL_US / 8) - 1;

// Shared with ISRs
volatile long          stepsRemaining   = 0;
volatile int8_t        leftPatternMove  = 0;
volatile int8_t        rightPatternMove = 0;
volatile byte          leftPatternRow   = 0;
volatile byte          rightPatternRow  = 0;
volatile unsigned int  irPulseCount     = 0;
volatile unsigned long irLastEdgeUs     = 0;
volatile byte          irBitCount       = NEC_BITS;
volatile unsigned long irRawCode        = 0;
volatile byte          remoteButton     = NO_BUTTON;

Servo sensorServo;

// Sweep tracking
float         wallDistanceCm[3] = {0, 0, 0};
byte          scanStep          = 0;
float         pingSamples[3]    = {0, 0, 0};
byte          pingSampleCount   = 0;
unsigned long aimStartMs        = 0;
unsigned long lastPingMs        = 0;

// IR watch and auto demo tracking
unsigned long irWindowStartMs = 0;
byte          autoDemoStep    = 0;
unsigned long autoWaitStartMs = 0;
unsigned int  autoWaitMs      = 0;

// ============================================================================
// MOTION: stepping the motors with Timer2, driving and turning
// ============================================================================

// Turns off all motor coils
void releaseMotorCoils(){
  for(byte wire = 0; wire < 4; wire++){
    digitalWrite(LEFT_MOTOR_PINS[wire], LOW);
    digitalWrite(RIGHT_MOTOR_PINS[wire], LOW);
  }
}

// Timer2 ISR: advances both motors one step while a move is active
ISR(TIMER2_COMPA_vect){
  if(stepsRemaining <= 0){
    return;
  }

  leftPatternRow  = (leftPatternRow  + leftPatternMove)  & 7;
  rightPatternRow = (rightPatternRow + rightPatternMove) & 7;

  for(byte wire = 0; wire < 4; wire++){
    digitalWrite(LEFT_MOTOR_PINS[wire],  COIL_PATTERN[leftPatternRow][wire]);
    digitalWrite(RIGHT_MOTOR_PINS[wire], COIL_PATTERN[rightPatternRow][wire]);
  }

  stepsRemaining--;
  if(stepsRemaining == 0){
    releaseMotorCoils();
  }
}

// Sets up motor pins and starts the Timer2 step interrupt
void setupMotion(){
  for(byte wire = 0; wire < 4; wire++){
    pinMode(LEFT_MOTOR_PINS[wire], OUTPUT);
    pinMode(RIGHT_MOTOR_PINS[wire], OUTPUT);
  }
  releaseMotorCoils();

  noInterrupts();
  TCCR2A = 0;
  TCCR2B = 0;
  TCNT2  = 0;
  TCCR2A |= (1 << WGM21);                  // CTC mode
  TCCR2B |= (1 << CS22) | (1 << CS20);     // prescaler 128 -> 8 us per tick
  OCR2A   = TIMER2_COMPARE_VALUE;
  TIMSK2 |= (1 << OCIE2A);
  interrupts();
}

// Starts a move of N steps with each wheel +1 forward or -1 backward
void startMotion(long steps, int leftWheelDirection, int rightWheelDirection){
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    leftPatternMove  = leftWheelDirection  * LEFT_MOTOR_FORWARD;
    rightPatternMove = rightWheelDirection * RIGHT_MOTOR_FORWARD;
    stepsRemaining   = steps;
  }
}

// Starts driving straight forward a number of inches
void startDrive(float inches){
  startMotion((long)(inches * STEPS_PER_INCH), +1, +1);
}

// Starts driving straight backward a number of inches
void startDriveBackward(float inches){
  startMotion((long)(inches * STEPS_PER_INCH), -1, -1);
}

// Starts an in-place turn; positive = right, negative = left
void startPivot(float degreesToRight){
  long steps = (long)(abs(degreesToRight) * STEPS_PER_90_DEG / 90.0);
  if(degreesToRight > 0){
    startMotion(steps, +1, -1);
  }else{
    startMotion(steps, -1, +1);
  }
}

// Returns true when the current move is done
bool motionFinished(){
  long stepsLeft;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    stepsLeft = stepsRemaining;
  }
  return stepsLeft <= 0;
}

// Cancels the current move and turns the motors off
void stopMotion(){
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    stepsRemaining = 0;
  }
  releaseMotorCoils();
}

// ============================================================================
// SENSING: servo, ultrasonic distance, IR beacon
// ============================================================================

// Pin 3 ISR: counts IR pulses for the beacon and decodes NEC remote buttons
void irReceiverEdge(){
  irPulseCount++;

  unsigned long nowUs = micros();
  unsigned long gapUs = nowUs - irLastEdgeUs;
  irLastEdgeUs = nowUs;

  if(gapUs >= NEC_START_MIN_US && gapUs <= NEC_START_MAX_US){
    irBitCount = 0;
    irRawCode = 0;
    return;
  }
  if(irBitCount >= NEC_BITS){
    return;
  }

  irRawCode >>= 1;                         // NEC sends the lowest bit first
  if(gapUs >= NEC_ONE_MIN_US && gapUs <= NEC_ONE_MAX_US){
    irRawCode |= 0x80000000UL;
  }else if(gapUs < NEC_ZERO_MIN_US || gapUs > NEC_ZERO_MAX_US){
    irBitCount = NEC_BITS;                 // bad timing: drop this frame
    return;
  }
  irBitCount++;

  if(irBitCount == NEC_BITS){
    byte command        = (irRawCode >> 16) & 0xFF;
    byte commandInverse = (irRawCode >> 24) & 0xFF;
    if((byte)~command == commandInverse){  // NEC sends the command twice, once inverted
      remoteButton = command;
    }
  }
}

// Returns the last remote button pressed (NO_BUTTON if none) and clears it
byte takeRemoteButton(){
  byte button;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    button = remoteButton;
    remoteButton = NO_BUTTON;
  }
  return button;
}

// Returns IR pulses since the last call and resets the count
unsigned int takeIrPulseCount(){
  unsigned int pulses;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    pulses = irPulseCount;
    irPulseCount = 0;
  }
  return pulses;
}

// Points the servo left, ahead, or right
void pointSensor(byte lookDirection){
  if(lookDirection == LOOK_LEFT){
    sensorServo.write(SERVO_ANGLE_LEFT);
  }else if(lookDirection == LOOK_RIGHT){
    sensorServo.write(SERVO_ANGLE_RIGHT);
  }else{
    sensorServo.write(SERVO_ANGLE_AHEAD);
  }
}

// Sets up ultrasonic pins, the servo, and the IR interrupt
void setupSensing(){
  pinMode(ULTRASONIC_TRIG_PIN, OUTPUT);
  pinMode(ULTRASONIC_ECHO_PIN, INPUT);
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);

  sensorServo.attach(SERVO_PIN);
  pointSensor(LOOK_AHEAD);

  pinMode(IR_RECEIVER_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(IR_RECEIVER_PIN), irReceiverEdge, FALLING);
}

// Takes one ultrasonic reading in cm; NO_ECHO_CM if nothing returns
float pingDistanceCm(){
  digitalWrite(ULTRASONIC_TRIG_PIN, HIGH);
  unsigned long pulseStartUs = micros();
  while(micros() - pulseStartUs < TRIGGER_PULSE_US){
  }
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);

  unsigned long echoTimeUs = pulseIn(ULTRASONIC_ECHO_PIN, HIGH, ECHO_TIMEOUT_US);
  if(echoTimeUs == 0){
    return NO_ECHO_CM;
  }
  return echoTimeUs / US_PER_CM;
}

// Returns the median of three values
float middleOfThree(float a, float b, float c){
  if((a <= b && b <= c) || (c <= b && b <= a)){
    return b;
  }
  if((b <= a && a <= c) || (c <= a && a <= b)){
    return a;
  }
  return c;
}

// ============================================================================
// TEST CONTROL: menu and command handling
// ============================================================================

// Prints the command menu
void printMenu(){
  Serial.println();
  Serial.println("MAKE SURE THE BATTERY SWITCH IS TURNED ON!!!");
  Serial.println("ROVER CHECKPOINT TESTS - type a letter and press Enter:");
  Serial.println("  d = drive 28 inches     f = forward 5 inches  b = back up 5 inches");
  Serial.println("  l = turn left 90        r = turn right 90");
  Serial.println("  s = sensor sweep        i = IR beacon watch   x = stop");
  Serial.println("Remote: up = f  down = b  left = l  right = r  1 = s  OK = x");
}

// Turns a remote button into its command letter; 0 for buttons not used here
char remoteButtonToCommand(byte button){
  switch (button){
    case BUTTON_UP:    return 'f';
    case BUTTON_DOWN:  return 'b';
    case BUTTON_LEFT:  return 'l';
    case BUTTON_RIGHT: return 'r';
    case BUTTON_1:     return 's';
    case BUTTON_OK:    return 'x';
    default:           return 0;
  }
}

// Aims the servo for the current sweep step and waits for it
void aimForCurrentScanStep(){
  pointSensor(SCAN_ORDER[scanStep]);
  aimStartMs = millis();
  currentState = STATE_AIM_SENSOR;
}

// Stops motors, centers the servo, and waits for a command
void goIdle(){
  stopMotion();
  pointSensor(LOOK_AHEAD);
  currentState = STATE_IDLE;
}

// Starts the test for one command letter
void runCommand(char command){
  switch (command){
    case 'd':
      Serial.println("Driving 28 inches...");
      startDrive(CHECKPOINT_DRIVE_IN);
      currentState = STATE_MOVING;
      break;
    case 'f':
      Serial.println("Forward 5 inches...");
      startDrive(SHORT_DRIVE_IN);
      currentState = STATE_MOVING;
      break;
    case 'b':
      Serial.println("Backing up 5 inches...");
      startDriveBackward(SHORT_DRIVE_IN);
      currentState = STATE_MOVING;
      break;
    case 'l':
      Serial.println("Turning left 90 degrees...");
      startPivot(-90);
      currentState = STATE_MOVING;
      break;
    case 'r':
      Serial.println("Turning right 90 degrees...");
      startPivot(+90);
      currentState = STATE_MOVING;
      break;
    case 's':
      Serial.println("Sensor sweep (send x to stop):");
      stopMotion();
      scanStep = 0;
      aimForCurrentScanStep();
      break;
    case 'i':
      Serial.println("IR beacon watch (send x to stop):");
      stopMotion();
      pointSensor(LOOK_AHEAD);
      takeIrPulseCount();
      irWindowStartMs = millis();
      currentState = STATE_IR_WATCH;
      break;
    case 'x':
      Serial.println("Stopped.");
      autoDemoStep = 0;
      goIdle();
      printMenu();
      break;
    default:
      break;
  }
}

// ============================================================================
// SETUP AND MAIN LOOP
// ============================================================================

// Initializes everything and shows the menu; schedules the auto demo if enabled
void setup(){
  Serial.begin(SERIAL_BAUD);
  setupMotion();
  setupSensing();
  printMenu();

  if(AUTO_MOTION_DEMO){
    Serial.println("Automatic demo starts in 5 seconds.");
    autoDemoStep = 1;
    autoWaitStartMs = millis();
    autoWaitMs = AUTO_START_DELAY_MS;
  }
}

// State machine: handles typed commands and remote buttons, then one non blocking piece of the current test
void loop(){
  unsigned long nowMs = millis();

  if(Serial.available() > 0){
    runCommand(Serial.read());
  }

  byte button = takeRemoteButton();
  if(button != NO_BUTTON){
    char command = remoteButtonToCommand(button);
    if(command != 0){
      runCommand(command);
    }else{
      Serial.print("Unused remote button, code 0x");
      Serial.println(button, HEX);
    }
  }

  switch (currentState){

    case STATE_IDLE:
      if(autoDemoStep > 0 && nowMs - autoWaitStartMs >= autoWaitMs){
        if(autoDemoStep == 1){
          runCommand('d');
        }else if(autoDemoStep == 2){
          runCommand('l');
        }else{
          runCommand('r');
        }
        autoDemoStep++;
        if(autoDemoStep > 3){
          autoDemoStep = 0;
        }
      }
      break;

    case STATE_MOVING:
      if(motionFinished()){
        Serial.println("Move complete.");
        autoWaitStartMs = nowMs;
        autoWaitMs = AUTO_PAUSE_MS;
        currentState = STATE_IDLE;
      }
      break;

    case STATE_AIM_SENSOR:
      if(nowMs - aimStartMs >= SERVO_SETTLE_MS){
        pingSampleCount = 0;
        lastPingMs = nowMs - PING_GAP_MS;
        currentState = STATE_MEASURE;
      }
      break;

    case STATE_MEASURE:
      if(pingSampleCount < 3 && nowMs - lastPingMs >= PING_GAP_MS){
        pingSamples[pingSampleCount] = pingDistanceCm();
        pingSampleCount++;
        lastPingMs = millis();
      }
      if(pingSampleCount == 3){
        wallDistanceCm[SCAN_ORDER[scanStep]] =
            middleOfThree(pingSamples[0], pingSamples[1], pingSamples[2]);
        scanStep++;
        if(scanStep >= 3){
          Serial.print("Distance (cm)   left: ");
          Serial.print(wallDistanceCm[LOOK_LEFT], 1);
          Serial.print("   ahead: ");
          Serial.print(wallDistanceCm[LOOK_AHEAD], 1);
          Serial.print("   right: ");
          Serial.println(wallDistanceCm[LOOK_RIGHT], 1);
          scanStep = 0;
        }
        aimForCurrentScanStep();
      }
      break;

    case STATE_IR_WATCH:
      if(nowMs - irWindowStartMs >= IR_LISTEN_MS){
        unsigned int pulses = takeIrPulseCount();
        if(pulses >= IR_MIN_PULSES){
          Serial.print("IR beacon: SEEN      (");
        }else{
          Serial.print("IR beacon: not seen  (");
        }
        Serial.print(pulses);
        Serial.println(" pulses)");
        irWindowStartMs = nowMs;
      }
      break;
  }
}