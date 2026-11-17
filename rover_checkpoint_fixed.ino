/*
  ============================================================================
  ROVER CHECKPOINT TESTS  -  EEL 4730 Embedded Programming Group Project
  ============================================================================

  WHAT THIS PROGRAM DOES (plain English)
  --------------------------------------
  This is the test program for the Wednesday Nov 18 checkpoint. It does NOT
  run the maze. It lets you show each required item one at a time, and it is
  also the tool for calibrating the robot before the real maze program.

  HOW TO USE IT
  -------------
    1. Upload this program and leave the USB cable plugged in.
    2. Open the Serial Monitor (Tools > Serial Monitor) and set it to 9600 baud.
    3. Type one letter in the box at the top and press Enter:

         d   Drive straight 28 inches, then stop.
         l   Turn 90 degrees LEFT in place.
         r   Turn 90 degrees RIGHT in place.
         s   Sensor sweep: point the sensor left, right, and ahead, and
             print the distance in each direction. Repeats until stopped.
         i   IR watch: print whether the IR receiver sees the beacon.
             Repeats until stopped.
         x   Stop whatever is happening.

  The robot does not move until you send a command, so it is safe on a desk.

  No USB cable long enough for the 28 inch drive? Set AUTO_MOTION_DEMO to
  true (Section 1). The robot then waits 5 seconds after power-on and does
  the drive, the left turn, and the right turn by itself on battery power.

  CALIBRATING (do this before running the maze program)
  -----------------------------------------------------
    - Send d. Measure how far the robot really drove. Then
        new STEPS_PER_INCH = old STEPS_PER_INCH x 28 / (inches actually driven)
    - Send l or r. If the robot turns less than 90 degrees, raise
      STEPS_PER_90_DEG. If it turns more, lower it.
    - If the motors hum but do not turn, raise STEP_INTERVAL_US.
    - Copy the final numbers into the same settings in rover_maze.ino.

  This program follows the same rules as the maze program: a state machine
  with a switch/case in loop(), no delay() after setup(), a Timer2 interrupt
  that steps the motors, and a pin 3 interrupt that counts IR pulses.

  Libraries used: Servo (Arduino), avr/io.h, avr/interrupt.h, util/atomic.h.

  BEFORE YOU TURN THIS IN: replace the placeholder names everywhere they
  appear:   [MOTION OWNER]   [SENSING OWNER]   [NAVIGATION OWNER]
  ============================================================================
*/

#include <Servo.h>          // Arduino library that controls the servo motor
#include <avr/io.h>         // AVR library: names of the Timer2 registers
#include <avr/interrupt.h>  // AVR library: lets us write interrupt routines
#include <util/atomic.h>    // AVR library: safe sharing of values with interrupts


// ============================================================================
// SECTION 1: SETTINGS (pins and tuning values)
// Keep these identical to the same settings in rover_maze.ino.
// ============================================================================

// ---- Pins (from the wiring table in the project handout) -------------------
const byte LEFT_MOTOR_PINS[4]  = {4, 5, 6, 7};        // left driver IN1..IN4
const byte RIGHT_MOTOR_PINS[4] = {A0, A1, A2, A3};    // right driver IN1..IN4
const byte SERVO_PIN           = 9;                   // servo signal wire
const byte ULTRASONIC_TRIG_PIN = A4;                  // sends the sound ping
const byte ULTRASONIC_ECHO_PIN = A5;                  // hears the echo
const byte IR_RECEIVER_PIN     = 3;                   // IR receiver (interrupt INT1)

// ---- Motion tuning ---------------------------------------------------------
// Time between motor steps in microseconds. Smaller = faster. If the motors
// hum and do not turn, make this LARGER. Allowed range: 600 to 2000.
const unsigned int STEP_INTERVAL_US = 1500;

const float STEPS_PER_INCH   = 507.0;    // motor steps to roll forward one inch
const float STEPS_PER_90_DEG = 1175;   // motor steps (per wheel) to spin 90 degrees

// Which way each motor spins to roll the robot FORWARD (+1 or -1). If a
// wheel rolls backward during the drive test, flip that wheel's sign.
const int LEFT_MOTOR_FORWARD  = +1;
const int RIGHT_MOTOR_FORWARD = -1;

// ---- Sensing tuning --------------------------------------------------------
// Servo angles (degrees). If left and right are swapped, swap 170 and 10.
const byte SERVO_ANGLE_LEFT  = 170;
const byte SERVO_ANGLE_AHEAD = 90;
const byte SERVO_ANGLE_RIGHT = 10;

const unsigned int  SERVO_SETTLE_MS  = 500;    // wait for the servo to finish swinging
const unsigned int  PING_GAP_MS      = 60;     // rest between pings so old echoes fade
const unsigned long ECHO_TIMEOUT_US  = 25000;  // give up on an echo after this long
const unsigned int  TRIGGER_PULSE_US = 12;     // length of the "send a ping" signal
const float         US_PER_CM        = 58.0;   // echo microseconds per centimeter
const float         NO_ECHO_CM       = 400.0;  // value used when no echo comes back

const unsigned int IR_LISTEN_MS  = 250;  // how long to listen for the beacon each time
const unsigned int IR_MIN_PULSES = 5;    // fewer pulses than this is treated as noise

// ---- Test settings ---------------------------------------------------------
const float CHECKPOINT_DRIVE_IN = 28.0;          // distance for the drive test
const bool  AUTO_MOTION_DEMO    = false;         // true = run d, l, r alone after power-on
const unsigned int  AUTO_START_DELAY_MS = 5000;  // wait before the automatic demo
const unsigned int  AUTO_PAUSE_MS       = 2000;  // pause between automatic moves
const unsigned long SERIAL_BAUD         = 9600;  // Serial Monitor speed


// ============================================================================
// SECTION 2: MEMORY (variables)
// ============================================================================

// The three directions the sensor can look.
const byte LOOK_LEFT  = 0;
const byte LOOK_AHEAD = 1;
const byte LOOK_RIGHT = 2;

// Order the sensor sweep looks around in.
const byte SCAN_ORDER[3] = {LOOK_LEFT, LOOK_RIGHT, LOOK_AHEAD};

// The STATES of the state machine: the one thing the robot is doing right now.
enum TestState {
  STATE_IDLE,         // waiting for a command
  STATE_MOVING,       // carrying out a drive or a turn
  STATE_AIM_SENSOR,   // sensor sweep: waiting for the servo to finish moving
  STATE_MEASURE,      // sensor sweep: taking distance readings
  STATE_IR_WATCH      // listening for the IR beacon and reporting
};
TestState currentState = STATE_IDLE;

// The order in which each motor's four wires are switched on to make it turn
// (called "half-stepping": 8 small steps per cycle). 1 = on, 0 = off.
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

// Timer2 counts in ticks of 8 microseconds. This is the tick count that
// produces one interrupt every STEP_INTERVAL_US.
const byte TIMER2_COMPARE_VALUE = (STEP_INTERVAL_US / 8) - 1;

// Variables shared with the interrupt routines ("volatile" = can change at
// any moment, behind the main program's back).
volatile long          stepsRemaining   = 0;  // steps left in the current move
volatile int8_t        leftPatternMove  = 0;  // +1 or -1: direction through COIL_PATTERN
volatile int8_t        rightPatternMove = 0;
volatile byte          leftPatternRow   = 0;  // current row of COIL_PATTERN, left motor
volatile byte          rightPatternRow  = 0;  // current row of COIL_PATTERN, right motor
volatile unsigned int  irPulseCount     = 0;  // IR pulses counted so far

Servo sensorServo;   // the servo that swivels the sensors

// Bookkeeping for the sensor sweep.
float         wallDistanceCm[3] = {0, 0, 0};  // one result per direction
byte          scanStep          = 0;          // which entry of SCAN_ORDER we are on
float         pingSamples[3]    = {0, 0, 0};  // three readings, to pick the middle one
byte          pingSampleCount   = 0;
unsigned long aimStartMs        = 0;          // when the servo started moving
unsigned long lastPingMs        = 0;          // when the last ping was sent

// Bookkeeping for the IR watch and the automatic demo.
unsigned long irWindowStartMs = 0;  // when the current listening period began
byte          autoDemoStep    = 0;  // 0 = not running; 1, 2, 3 = which move is next
unsigned long autoWaitStartMs = 0;  // when the current pause began
unsigned int  autoWaitMs      = 0;  // how long the current pause lasts


// ============================================================================
// SECTION 3: MOTION
// ============================================================================

/*
  releaseMotorCoils
  What it does: Switches off all eight motor wires so the motors stop drawing
                battery power while the robot is standing still.
  Written by:   [MOTION OWNER]
*/
void releaseMotorCoils() {
  for (byte wire = 0; wire < 4; wire++) {
    digitalWrite(LEFT_MOTOR_PINS[wire], LOW);
    digitalWrite(RIGHT_MOTOR_PINS[wire], LOW);
  }
}

/*
  Timer2 interrupt routine (runs automatically every STEP_INTERVAL_US)
  What it does: If a move is in progress, advances BOTH motors by one step,
                counts that step, and switches the motors off when the move
                is complete.
  Written by:   [MOTION OWNER]
*/
ISR(TIMER2_COMPA_vect) {
  if (stepsRemaining <= 0) {
    return;                       // no move in progress: nothing to do
  }

  // Move each motor to the next (or previous) row of the coil pattern.
  // "& 7" wraps the row number around so it always stays between 0 and 7.
  leftPatternRow  = (leftPatternRow  + leftPatternMove)  & 7;
  rightPatternRow = (rightPatternRow + rightPatternMove) & 7;

  for (byte wire = 0; wire < 4; wire++) {
    digitalWrite(LEFT_MOTOR_PINS[wire],  COIL_PATTERN[leftPatternRow][wire]);
    digitalWrite(RIGHT_MOTOR_PINS[wire], COIL_PATTERN[rightPatternRow][wire]);
  }

  stepsRemaining--;
  if (stepsRemaining == 0) {
    releaseMotorCoils();          // move finished
  }
}

/*
  setupMotion
  What it does: Prepares the motor pins and starts Timer2 so that it calls the
                interrupt routine above once every STEP_INTERVAL_US.
  Written by:   [MOTION OWNER]
*/
void setupMotion() {
  for (byte wire = 0; wire < 4; wire++) {
    pinMode(LEFT_MOTOR_PINS[wire], OUTPUT);
    pinMode(RIGHT_MOTOR_PINS[wire], OUTPUT);
  }
  releaseMotorCoils();

  noInterrupts();                          // pause interrupts while changing the timer
  TCCR2A = 0;                              // clear the old Timer2 settings
  TCCR2B = 0;
  TCNT2  = 0;                              // start counting from zero
  TCCR2A |= (1 << WGM21);                  // mode: count up to a target, then restart
  TCCR2B |= (1 << CS22) | (1 << CS20);     // speed: one tick every 8 microseconds
  OCR2A   = TIMER2_COMPARE_VALUE;          // the target tick count
  TIMSK2 |= (1 << OCIE2A);                 // run the interrupt each time the target is hit
  interrupts();
}

/*
  startMotion
  What it does: Begins a move of the given number of steps. Each wheel
                direction is +1 (roll forward) or -1 (roll backward). Returns
                immediately; the Timer2 interrupt does the stepping.
  Written by:   [MOTION OWNER]
*/
void startMotion(long steps, int leftWheelDirection, int rightWheelDirection) {
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    leftPatternMove  = leftWheelDirection  * LEFT_MOTOR_FORWARD;
    rightPatternMove = rightWheelDirection * RIGHT_MOTOR_FORWARD;
    stepsRemaining   = steps;
  }
}

/*
  startDrive
  What it does: Begins driving straight forward the given number of inches.
  Written by:   [MOTION OWNER]
*/
void startDrive(float inches) {
  startMotion((long)(inches * STEPS_PER_INCH), +1, +1);
}

/*
  startPivot
  What it does: Begins spinning the robot in place. Positive degrees turn
                RIGHT, negative degrees turn LEFT.
  Written by:   [MOTION OWNER]
*/
void startPivot(float degreesToRight) {
  long steps = (long)(abs(degreesToRight) * STEPS_PER_90_DEG / 90.0);
  if (degreesToRight > 0) {
    startMotion(steps, +1, -1);   // right turn: left wheel forward, right wheel back
  } else {
    startMotion(steps, -1, +1);   // left turn: left wheel back, right wheel forward
  }
}

/*
  motionFinished
  What it does: Answers "has the current move finished?" (true or false).
  Written by:   [MOTION OWNER]
*/
bool motionFinished() {
  long stepsLeft;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    stepsLeft = stepsRemaining;
  }
  return stepsLeft <= 0;
}

/*
  stopMotion
  What it does: Cancels the current move right now and switches the motors off.
  Written by:   [MOTION OWNER]
*/
void stopMotion() {
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    stepsRemaining = 0;
  }
  releaseMotorCoils();
}


// ============================================================================
// SECTION 4: SENSING
// ============================================================================

/*
  IR interrupt routine (runs automatically on every pulse from the IR receiver)
  What it does: Adds one to the pulse counter. That is all, so it stays fast.
  Written by:   [SENSING OWNER]
*/
void countIrPulse() {
  irPulseCount++;
}

/*
  takeIrPulseCount
  What it does: Reports how many IR pulses have arrived since the last time
                this function was called, then resets the counter to zero.
  Written by:   [SENSING OWNER]
*/
unsigned int takeIrPulseCount() {
  unsigned int pulses;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    pulses = irPulseCount;
    irPulseCount = 0;
  }
  return pulses;
}

/*
  pointSensor
  What it does: Swivels the servo so the distance sensor and IR receiver face
                LOOK_LEFT, LOOK_AHEAD, or LOOK_RIGHT.
  Written by:   [SENSING OWNER]
*/
void pointSensor(byte lookDirection) {
  if (lookDirection == LOOK_LEFT) {
    sensorServo.write(SERVO_ANGLE_LEFT);
  } else if (lookDirection == LOOK_RIGHT) {
    sensorServo.write(SERVO_ANGLE_RIGHT);
  } else {
    sensorServo.write(SERVO_ANGLE_AHEAD);
  }
}

/*
  setupSensing
  What it does: Prepares the ultrasonic sensor pins, attaches the servo and
                points it ahead, and turns on the pin 3 interrupt that counts
                IR beacon pulses.
  Written by:   [SENSING OWNER]
*/
void setupSensing() {
  pinMode(ULTRASONIC_TRIG_PIN, OUTPUT);
  pinMode(ULTRASONIC_ECHO_PIN, INPUT);
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);

  sensorServo.attach(SERVO_PIN);
  pointSensor(LOOK_AHEAD);

  pinMode(IR_RECEIVER_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(IR_RECEIVER_PIN), countIrPulse, FALLING);
}

/*
  pingDistanceCm
  What it does: Takes ONE distance reading. Sends a short sound ping, times
                the echo, and converts that time to centimeters. Returns
                NO_ECHO_CM ("very far away") if no echo comes back in time.
  Written by:   [SENSING OWNER]
*/
float pingDistanceCm() {
  // Send the ping: hold the trigger pin HIGH for a few microseconds.
  digitalWrite(ULTRASONIC_TRIG_PIN, HIGH);
  unsigned long pulseStartUs = micros();
  while (micros() - pulseStartUs < TRIGGER_PULSE_US) {
    // wait
  }
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);

  // Time the echo. The timeout keeps this from waiting a full second.
  unsigned long echoTimeUs = pulseIn(ULTRASONIC_ECHO_PIN, HIGH, ECHO_TIMEOUT_US);
  if (echoTimeUs == 0) {
    return NO_ECHO_CM;
  }
  return echoTimeUs / US_PER_CM;
}

/*
  middleOfThree
  What it does: Returns the middle value of three numbers (the median), which
                throws away a single freak reading.
  Written by:   [SENSING OWNER]
*/
float middleOfThree(float a, float b, float c) {
  if ((a <= b && b <= c) || (c <= b && b <= a)) {
    return b;
  }
  if ((b <= a && a <= c) || (c <= a && a <= b)) {
    return a;
  }
  return c;
}


// ============================================================================
// SECTION 5: TEST CONTROL
// ============================================================================

/*
  printMenu
  What it does: Prints the list of commands to the Serial Monitor.
  Written by:   [NAVIGATION OWNER]
*/
void printMenu() {
  Serial.println();
  Serial.println("ROVER CHECKPOINT TESTS - type a letter and press Enter:");
  Serial.println("  d = drive 28 inches     l = turn left 90     r = turn right 90");
  Serial.println("  s = sensor sweep        i = IR beacon watch  x = stop");
}

/*
  aimForCurrentScanStep
  What it does: Points the sensor in the direction for the current step of
                the sweep and switches to the "waiting for servo" state.
  Written by:   [NAVIGATION OWNER]
*/
void aimForCurrentScanStep() {
  pointSensor(SCAN_ORDER[scanStep]);
  aimStartMs = millis();
  currentState = STATE_AIM_SENSOR;
}

/*
  goIdle
  What it does: Stops the motors, points the sensor ahead, and waits for the
                next command.
  Written by:   [NAVIGATION OWNER]
*/
void goIdle() {
  stopMotion();
  pointSensor(LOOK_AHEAD);
  currentState = STATE_IDLE;
}

/*
  runCommand
  What it does: Starts the test that matches one command letter.
  Written by:   [NAVIGATION OWNER]
*/
void runCommand(char command) {
  switch (command) {
    case 'd':
      Serial.println("Driving 28 inches...");
      startDrive(CHECKPOINT_DRIVE_IN);
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
      takeIrPulseCount();                 // throw away old pulses
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
      break;                              // ignore Enter key and unknown letters
  }
}


// ============================================================================
// SECTION 6: setup() AND loop()
// ============================================================================

/*
  setup
  What it does: Runs once at power-on. Gets every part of the robot ready and
                shows the menu. If AUTO_MOTION_DEMO is on, schedules the
                automatic drive and turns.
  Written by:   [NAVIGATION OWNER]
*/
void setup() {
  Serial.begin(SERIAL_BAUD);
  setupMotion();
  setupSensing();
  printMenu();

  if (AUTO_MOTION_DEMO) {
    Serial.println("Automatic demo starts in 5 seconds.");
    autoDemoStep = 1;
    autoWaitStartMs = millis();
    autoWaitMs = AUTO_START_DELAY_MS;
  }
}

/*
  loop
  What it does: Runs over and over. Each pass checks for a typed command,
                then does one small piece of the current state's job. It
                never waits in place.
  Written by:   [NAVIGATION OWNER]
*/
void loop() {
  unsigned long nowMs = millis();

  // A typed command can interrupt whatever is happening.
  if (Serial.available() > 0) {
    runCommand(Serial.read());
  }

  switch (currentState) {

    // Waiting for a command. Also runs the automatic demo, one move at a time.
    case STATE_IDLE:
      if (autoDemoStep > 0 && nowMs - autoWaitStartMs >= autoWaitMs) {
        if (autoDemoStep == 1) {
          runCommand('d');
        } else if (autoDemoStep == 2) {
          runCommand('l');
        } else {
          runCommand('r');
        }
        autoDemoStep++;
        if (autoDemoStep > 3) {
          autoDemoStep = 0;               // all three moves have been started
        }
      }
      break;

    // A drive or turn is in progress. Report when it ends.
    case STATE_MOVING:
      if (motionFinished()) {
        Serial.println("Move complete.");
        autoWaitStartMs = nowMs;          // start the pause before the next auto move
        autoWaitMs = AUTO_PAUSE_MS;
        currentState = STATE_IDLE;
      }
      break;

    // Sensor sweep: waiting for the servo to finish swinging.
    case STATE_AIM_SENSOR:
      if (nowMs - aimStartMs >= SERVO_SETTLE_MS) {
        pingSampleCount = 0;
        lastPingMs = nowMs - PING_GAP_MS; // allow the first ping right away
        currentState = STATE_MEASURE;
      }
      break;

    // Sensor sweep: take three readings, keep the middle one, move on.
    case STATE_MEASURE:
      if (pingSampleCount < 3 && nowMs - lastPingMs >= PING_GAP_MS) {
        pingSamples[pingSampleCount] = pingDistanceCm();
        pingSampleCount++;
        lastPingMs = millis();
      }
      if (pingSampleCount == 3) {
        wallDistanceCm[SCAN_ORDER[scanStep]] =
            middleOfThree(pingSamples[0], pingSamples[1], pingSamples[2]);
        scanStep++;
        if (scanStep >= 3) {
          // Looked in all three directions: print one line and start over.
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

    // IR watch: every IR_LISTEN_MS, report whether the beacon was seen.
    case STATE_IR_WATCH:
      if (nowMs - irWindowStartMs >= IR_LISTEN_MS) {
        unsigned int pulses = takeIrPulseCount();
        if (pulses >= IR_MIN_PULSES) {
          Serial.print("IR beacon: SEEN      (");
        } else {
          Serial.print("IR beacon: not seen  (");
        }
        Serial.print(pulses);
        Serial.println(" pulses)");
        irWindowStartMs = nowMs;
      }
      break;
  }
}
