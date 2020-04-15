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
AccelStepper stepper2 = AccelStepper(MotorInterfaceType, motorPin0, motorPin2, motorPin1, motorPin3);
AccelStepper stepper1 = AccelStepper(MotorInterfaceType, motorPin4, motorPin6, motorPin5, motorPin7);
AccelStepper stepper = AccelStepper(MotorInterfaceType, motorPin8, motorPin10, motorPin9, motorPin11);


// Globals

const int SWITCHPIN = 0;  // the number of the pushbutton pin
int buttonState0 = 0;
int buttonState1 = 0;
bool initialize = true;
bool initialize1 = true;


void setup() {
  // switch
  // declare pin to be an input:
  pinMode(0, INPUT_PULLUP);
  pinMode(A5, INPUT_PULLUP);

  // Set the maximum steps per second:
  // Speeds of more than 1000 steps per second can be unreliable.
  stepper.setMaxSpeed(1000);
  stepper.setSpeed(500);
  // Set the maximum acceleration in steps per second^2:
  stepper.setAcceleration(200);

  stepper1.setMaxSpeed(1000);
  stepper1.setSpeed(500);
  stepper1.setAcceleration(200);


  Serial.begin(9600);
}

// my code
void loop() {

  // TO DO
  // zero position always same -
  // attach switch -
  // check out multistepper library for running several motors at the same time -
  // program arm motor - 
  // program leaves


  // switch check
  buttonState0 = digitalRead(0);
  Serial.print(" read_0: ");
  Serial.println(buttonState0);



  buttonState1 = digitalRead(A5);
  Serial.print(" read_1: ");
  Serial.println(buttonState1);
  Serial.println(" ");

  /*
    //stepper.setSpeed(500);
    stepper.moveTo(4096);
    stepper.run();


    stepper1.setSpeed(500);
    stepper1.runSpeed();
  */


  if (initialize == true) {
    initToZeroPos();
  }
  else {
    sadArm();
  }

  if (initialize1 == true) {
    initToZeroPosArm();
  }
  else {
    //sadArm();
    Serial.println("______");
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
}

//arm
void sadArm() {
  // move 20% rev
  int stepsToMove = calcSteps (0.4);
  Serial.println("Accel down");
  mAccel(-stepsToMove);
  delay(1000);

  Serial.println("Accel to 0");
  mAccel(0);
  delay(1000);
}

void test() {
  // move 20% rev
  int stepsToMove = calcSteps (0.2);
  Serial.println("Accel+");
  mAccel(stepsToMove);
  delay(1000);

  Serial.println("Accel-");
  mAccel(0);
  delay(1000);
}


void initToZeroPos() {
  //turn clockwise until hit switch
  stepper.setSpeed(500);

  if (buttonState0 == 0) {
    Serial.println("Switch Hit - arm");
    stepper.setCurrentPosition(0);
    initialize = false;
    //Serial.println(stepper.currentPosition());
  } else {
    //Serial.println(stepper.currentPosition());
    stepper.run();
  }
}

void initToZeroPosArm() {
  //turn clockwise until hit switch
  stepper1.setSpeed(-500);

  if (buttonState1 == 0) {
    Serial.println("Switch Hit - arm");
    stepper1.setCurrentPosition(0);
    initialize1 = false;
    //Serial.println(stepper1.currentPosition());
  } else {
    //Serial.println(stepper1.currentPosition());
    stepper1.run();
  }
}


void mRunUntil_switchHit () {
  stepper.setSpeed(500);

  if (buttonState0 == 0) {
    Serial.println("Button");
    stepper.setCurrentPosition(0);
  } else {
    Serial.println(stepper.currentPosition());
    stepper.run();
  }
}


void mAccel(int targetPos) {
  // Set target position:
  stepper.moveTo(targetPos);

  // Run to position with set speed and acceleration:
  stepper.runToPosition();

  Serial.println("speed: " + String(stepper.speed()));
}

void mConstant(int _steps, int _speed) {
  while (stepper.currentPosition() != _steps) {
    stepper.setSpeed(_speed);
    stepper.runSpeed();
  }
  Serial.println("speed: " + String(stepper.speed()));
}

int calcSteps(float rev) {
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
