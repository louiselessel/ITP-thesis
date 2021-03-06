/* Example sketch to control a 28BYJ-48 stepper motor
  with ULN2003 driver board, AccelStepper and Arduino UNO:
  continuous rotation. More info: https://www.makerguides.com

  AccelStepper reference:
  https://www.airspayce.com/mikem/arduino/AccelStepper/classAccelStepper.html#a73bdecf1273d98d8c5fbcb764cabeea5
  https://www.airspayce.com/mikem/arduino/AccelStepper/classMultiStepper.html

  // Calculations:
  // In half step mode (8), one revolution takes 4096 steps
  // so 500 steps/sec results in roughly 7 rpm.
  // 500*60s = 30,000 steps in 1 minute
  // 30,000 /4096 = 7.32 rpm
*/



/*
   IMPORTANT !!!!!!!!!!!!!!!!!
   plug in motor first. Then Arduino. To make sure the zero position is set acurately!
*/





// Include the AccelStepper library:
#include <AccelStepper.h>

// Motor pin definitions:

// Stepper - main
#define motorPin0  2      // IN1 on the ULN2003 driver // note: doesn't work when starting from pin 0 and 1
#define motorPin1  3      // IN2 on the ULN2003 driver
#define motorPin2  4      // IN3 on the ULN2003 driver
#define motorPin3  5      // IN4 on the ULN2003 driver

// Stepper 1 - arm
#define motorPin4  6      // IN1 to pin 3
#define motorPin5  7      // IN2 to pin 4
#define motorPin6  8      // IN3 to pin 5
#define motorPin7  9      // IN4 to pin 6

// Stepper 2 - base
#define motorPin8  10      // IN1 on the ULN2003 driver
#define motorPin9  11      // IN2 on the ULN2003 driver
#define motorPin10  12     // IN3 on the ULN2003 driver
#define motorPin11  13     // IN4 on the ULN2003 driver

// Define the AccelStepper interface type; 4 wire motor in half step mode use 8:
#define MotorInterfaceType 8
//FULL4WIRE = 4, HALF4WIRE = 8

// Initialize with pin sequence IN1-IN3-IN2-IN4 for using the AccelStepper library with 28BYJ-48 stepper motor:
AccelStepper stepper = AccelStepper(MotorInterfaceType, motorPin0, motorPin2, motorPin1, motorPin3);
AccelStepper stepper1 = AccelStepper(MotorInterfaceType, motorPin4, motorPin6, motorPin5, motorPin7);
AccelStepper stepper2 = AccelStepper(MotorInterfaceType, motorPin8, motorPin10, motorPin9, motorPin11);

AccelStepper s[3];

// Globals

const int SWITCHPIN = 0;  // the number of the pushbutton pin
int buttonState0 = 0;
int buttonState1 = 0;
bool initialize = true;
bool initialize1 = true;


void setup() {
  // switch
  // declare pin to be an input:
  pinMode(A4, INPUT_PULLUP);
  pinMode(A5, INPUT_PULLUP);

  // setup steppers
  s[0] = stepper;
  // Set the maximum steps per second:
  // Speeds of more than 1000 steps per second can be unreliable.
  s[0].setMaxSpeed(1000);
  s[0].setSpeed(500);
  // Set the maximum acceleration in steps per second^2:
  s[0].setAcceleration(200);


  s[1] = stepper1;
  s[1].setMaxSpeed(1000);
  s[1].setSpeed(500);
  s[1].setAcceleration(200);

  s[2] = stepper2;
  s[2].setMaxSpeed(1000);
  s[2].setSpeed(500);
  s[2].setAcceleration(200);


  Serial.begin(9600);
  Serial.println("Started---------");

  /*
    s[0].moveTo(-1000);
    s[0].runToPosition();


    s[1].moveTo(1000);
    s[1].runToPosition();
  */
}



void loop() {

  // TO DO
  // zero position always same -
  // attach switch -
  // check out multistepper library for running several motors at the same time -
  // program arm motor -
  // change design to equip leaves
  // program leaves
  // presence sensor
  // change setting of zero points to potentiometers
  


  // switches check
  buttonState0 = digitalRead(A4);
  //Serial.print(" read_0: ");
  //Serial.println(buttonState0);

  buttonState1 = digitalRead(A5);
  //Serial.print(" read_1: ");
  //Serial.println(buttonState1);
  //Serial.println(" ");



  // Initialize
  if (initialize == true) {
    initToZeroPos_main(0);
  }
  else {
    if (initialize1 == true) {
      initToZeroPos_extended(1);
    }
    else {
      sadArm(1);
      sadArm(0);
    }
  }





  /* library notes */
  /* accel + can run simultaneously, but is dependent on loop, so println will harm speed */
  // run() - needs a targetpos:   stepper.moveTo(4096);
  /* expensive calls - harm run! */
  //Serial.println(stepper.currentPosition());
  //Serial.println("speed: " + String(stepper.speed()));
  /* can be used to check did the stepper stop at position? */
  //Serial.println(stepper.isRunning ());

  /* constant + can run simultaneously */
  // runSpeed()
  // runSpeedToPosition()

  /* accel + blocks */
  // runToNewPosition() // to target
  // runToPosition()

  /*
    Serial.println("Accel+");
    mAccel(stepsToMove);
    delay(1000);

    Serial.println("Accel-");
    mAccel(0);
    delay(1000);

    Serial.println("Constant");
    mConstant(4096, 500);
    // Set the current position to 0:
    stepper.setCurrentPosition(0);
    delay(1000);
  */


  //Serial.println(sA[0].isRunning ());
  /*
    //s[2].setSpeed(500);
    s[2].moveTo(4096);
    s[2].run();


    s[2].setSpeed(500);
    s[2].runSpeed();
  */

  /*
    int stepsToMove = calcSteps (0.5);
    Serial.println("Accel down");
    mAccel(-stepsToMove, 2);
    delay(1000);

    Serial.println("Accel to 0");
    mAccel(0, 2);
    delay(1000);
  */
}



void sadArm(int _stepper) {
  // move rev
  int stepsToMove = calcSteps (0.2);
  Serial.println("Accel down");
  mAccel(stepsToMove, _stepper);
  delay(1000);

  Serial.println("Accel to 0");
  mAccel(0, _stepper);
  delay(1000);
}


void initToZeroPos_main(int _stepper) {
  //turn clockwise until hit switch
  s[_stepper].setSpeed(500);

  if (buttonState0 == 0) {
    Serial.println("Switch Hit - main");
    s[_stepper].setCurrentPosition(0);
    initialize = false;
    //Serial.println(s[_stepper].currentPosition());
  } else {
    //Serial.println(s[_stepper].currentPosition());
    s[_stepper].run();
  }
}

void initToZeroPos_extended(int _stepper) {
  //turn clockwise until hit switch
  s[_stepper].setSpeed(-500);

  if (buttonState1 == 0) {
    Serial.println("Switch Hit - extended");
    s[_stepper].setCurrentPosition(0);
    initialize1 = false;
    //Serial.println(s[_stepper].currentPosition());
  } else {
    //Serial.println(s[_stepper].currentPosition());
    s[_stepper].run();
  }
}


void mRunUntil_switchHit (int _stepper) {
  s[_stepper].setSpeed(500);

  if (buttonState0 == 0) {
    Serial.println("Button");
    s[_stepper].setCurrentPosition(0);
  } else {
    Serial.println(s[_stepper].currentPosition());
    s[_stepper].run();
  }
}


void mAccel(int targetPos, int _stepper) {
  // Set target position:
  s[_stepper].moveTo(targetPos);

  // Run to position with set speed and acceleration:
  s[_stepper].runToPosition();

  Serial.println("speed: " + String(s[_stepper].speed()));
}


void mConstant(int _steps, int _speed, int _stepper) {
  while (s[_stepper].currentPosition() != _steps) {
    s[_stepper].setSpeed(_speed);
    s[_stepper].runSpeed();
  }
  Serial.println("speed: " + String(s[_stepper].speed()));
}

int calcSteps(float rev) { // Ex. 0.2 = 20% rev, 1 = 100% (one rev), 2 = 200% (2 rev)
  int steps = 0;
  steps = int(4096 * rev);
  return steps;
}



/********** HELPERS */

// move 1 rev
//int stepsToMove = calcSteps (1);
// move 2 rev
//int stepsToMove = calcSteps (2);
// move 1/2 rev
//int stepsToMove = calcSteps (0.5);



/********** CODES */

/*
  // From const speed example

  void loop() {

  //In half step mode, one revolution takes 4096 steps
  //so 500 steps/sec results in roughly 7 rpm.
  // 500*60s = 30,000 steps in 1 minute
  // 30,000 /4096 = 7.32 rpm

  // Set the speed of the motor in steps per second:
  // can be put in setup
  stepper.setSpeed(500);
  // Step the motor with constant speed as set by setSpeed():
  stepper.runSpeed();
  }
*/


/*
  // From Accel-decel example

  void loop() {
  // Set target position:
  stepper.moveTo(8192);
  // Run to position with set speed and acceleration:
  stepper.runToPosition();

  delay(1000);

  // Move back to original position:
  stepper.moveTo(0);
  // Run to position with set speed and acceleration:
  stepper.runToPosition();

  delay(1000);
  }
*/

/*
  // From Speed-dir-rev example

  void loop() {
  // Set the current position to 0:
  stepper.setCurrentPosition(0);

  // Run the motor forward at 500 steps/second until the motor reaches 4096 steps (1 revolution):
  while (stepper.currentPosition() != 4096) {
    stepper.setSpeed(500);
    stepper.runSpeed();
  }
  delay(1000);

  // Reset the position to 0:
  stepper.setCurrentPosition(0);

  // Run the motor backwards at 1000 steps/second until the motor reaches -4096 steps (1 revolution):
  while (stepper.currentPosition() != -4096) {
    stepper.setSpeed(-1000);
    stepper.runSpeed();
  }

  delay(1000);

  // Reset the position to 0:
  stepper.setCurrentPosition(0);
  // Run the motor forward at 1000 steps/second until the motor reaches 8192 steps (2 revolutions):
  while (stepper.currentPosition() != 8192) {
    stepper.setSpeed(1000);
    stepper.runSpeed();
  }

  delay(3000);
  }
*/
