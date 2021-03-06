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

  Neopixels library
  https://github.com/adafruit/Adafruit_NeoPixel

  Human_Presence_Sensor-AK9753 from Sparkfun or Groove
  http://wiki.seeedstudio.com/Grove-Human_Presence_Sensor-AK9753/
*/

/*
   IMPORTANT !!!!!!!!!!!!!!!!!
   Always set zero positions on buttons when installation starts.
   Listen - hold 0 and 1 a bit at a time, until it starts grinding. 2 moves both forward.

   Check pos by quick press on 2
*/


#include <Adafruit_NeoPixel.h>
#include <AccelStepper.h>
#include <Wire.h>
#include "Grove_Human_Presence_Sensor.h"
#include <ColorConverter.h>

// LED Neopixels
#define PIN A3         //defining the PWM pin
#define N_LEDS 3      //number of LED units on the strip

// Motor pins:
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


// Initialize LED neopixels
Adafruit_NeoPixel strip = Adafruit_NeoPixel(N_LEDS, PIN); //declared neopixel object
ColorConverter converter;


// Initialize sensor
AK9753 movementSensor;


/* Globals */
// LED
int r = 255;
int g = 255;
int b = 255;

int h = 50;         // hue
int sat = 20;        // saturation
int i = 100;        // intensity
int change = 1;              // increment to change hue by
RGBColor color;

// Movement Sensor
// You need to adjust these sensitivities lower if you want to detect more far
// but it will introduce more error detection
float sensitivity_presence = 1.0;
float sensitivity_movement = 4.0; //
int detect_interval = 30; //milliseconds
PresenceDetector detector(movementSensor, sensitivity_presence, sensitivity_movement, detect_interval);
uint32_t last_time;
int sensorCheckTime = 100;

// Switches
const int init_switch0 = A0;
const int init_switch1 = A1;
const int init_switch2 = A2;

int buttonState0 = 0;
int buttonState1 = 0;
int buttonState2 = 0;
bool initialize = true;
bool initialize1 = true;

// Motors
bool init0 = false;
bool init1 = false;

// Logic
int health = 100;
int healthVal = 1;
uint32_t recoveryTime = 30000;
uint32_t last_time_hurt;
bool mThreshold = false;
int ledBri = 255;

int printVar = 0;

// Modes
bool perkup = false;
bool healthy = false;


/* -------------------------------------------------------------------- */

void setup() {
  Serial.begin(9600);
  Wire.begin();

  // setup switches
  pinMode(init_switch0, INPUT_PULLUP);
  pinMode(init_switch1, INPUT_PULLUP);
  pinMode(init_switch2, INPUT_PULLUP);

  // setup LED
  // Initialize the NeoPixel library.
  strip.begin();    // initialize pixel strip
  strip.clear();    // turn all LEDs off
  strip.show();     // update strip

  // setup movement sensor
  if (movementSensor.initialize() == false) {
    Serial.println("Device not found. Check wiring.");
    while (1);
  }
  last_time = millis();

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
  s[1].setSpeed(50);
  s[1].setAcceleration(10);

  s[2] = stepper2;
  s[2].setMaxSpeed(1000);
  s[2].setSpeed(500);
  s[2].setAcceleration(200);

  init0 = false; // ----------- these should be false when motor is ON, set true for debug.
  init1 = false;

  // set initial positions - make sure to adjust on buttons after startup
  // until the branch is straight
  s[0].setCurrentPosition(0);
  s[1].setCurrentPosition(0);


  Serial.println("Started, set pos 0 for M0 and M1, adjust on buttons as needed.po");

  /*
      // inital motor positions (DEBUG - GO FORWARD)
      s[1].moveTo(1000);
      s[1].runToPosition();

      s[0].moveTo(-1000);
      s[0].runToPosition();
  */
}


bool sensorPrint = true;

void loop() {

  /* INITALIZATION */
  // switches check (1 is default, 0 is hit)
  buttonState0 = digitalRead(init_switch0);
  buttonState1 = digitalRead(init_switch1);
  buttonState2 = digitalRead(init_switch2);
  //printSwitches();

  // intitalize motor positions
  initializeMotorsManually();

  uint32_t now = millis();

  /* SENSOR CHECK */
  // Check movement sensor
  // only check if buttons have been used to initialize motors!
  if (init0 && init1) {
    detector.loop();

    // check if time interval has passed
    if (now - last_time > sensorCheckTime) {
      uint8_t m = detector.getMovement();  //read movement state will clear it
      sensorCheck(m);
      last_time = now;
    }
  }

  /* LIGHT CONTROL */

  // LED update
  // create a single color from hue, sat, intensity:
  color = converter.HSItoRGB(h, sat, i);

  // loop over all the pixels:
  for (int pixel = 0; pixel < N_LEDS; pixel++) {
    strip.setPixelColor(pixel, color.red, color.green, color.blue);    // set the color for this pixel
    //strip.show();   // update the strip
    delay(10);
  }
  strip.show();

  // increment hue to fade from red (0) to reddish orange (15) and back:
  if (now - last_time > 100) {
    h = h + change;
    if (h < 50 || h > 100) {
      change = -change;
    }
  }

  delay(1);

  /* MOVEMENT CONTROL */

  // hurtful movement detectedws
  if (mThreshold == true) {
    Serial.println("hurting ...");
    Serial.println(s[1].currentPosition());
    // decrease health
    health = constrain(health -= healthVal, 0, 100);
    // start timer
    last_time_hurt = now;
    // show hurt
    hurtResponse();
    // reset threshold detector
    mThreshold = false;

    perkup = true;
  }


  if (init0 && init1) {
    // if enough seconds have pasts, start perking back up
    if (now - last_time_hurt > recoveryTime) {
      if (perkup) {
        //Serial.println("perking ...");
        perkUp();
        // if reached 0
        if (s[1].currentPosition() == long(0)) {
          Serial.println("perked up");
          perkup = false;
          // done perking up, start healing
          //healingMode();
        }
      }
    }
  }


}

// MODES -------------------------------------------------------------


void perkUp() {
  // slowly move branch up to 0 steps - dependant on acceleration
  s[1].moveTo(0);
  s[1].run();

  Serial.println(s[1].currentPosition());

  // calculated so steps add up to 'how far is back up' in the amount of recoveryTime
  // motor values corresponding to amount of overall health.
  ledBri = constrain(ledBri += 1, 0, 255);
  strip.setBrightness(ledBri);
}

void hurtResponse() {
  Serial.println("HURT");
  // dim lights
  ledBri = strip.getBrightness();
  
  while (ledBri > 50) {
    strip.setBrightness(ledBri);
    delay(40);
    strip.show();
    ledBri--;
  }

  // stop other movement
  perkup = false;
  s[1].stop();
  s[1].setAcceleration(10);

  // move down until position is reached         // Adjust timing - acceleration 6 seconds to drop
  s[1].moveTo(1800);
  s[1].runToPosition();
  //s[1].moveTo(0);
  //s[1].runToPosition();

  Serial.println(s[1].currentPosition());
  Serial.println("HURT DONE");
  s[1].setAcceleration(3);

  // retract leaves
}





void healingMode() {
  // increase health slowly
  health = constrain(health += healthVal, 0, 100);
  // increase lights slowly

  // lights bright
  ledBri = 255;
  strip.setBrightness(ledBri);
}






// -------------------------------------------------------------

void sensorCheck(uint8_t m) {
  // printout (plotter) or not
  if (sensorPrint) {
    // PLOTTER
    // movement 1-3 (up - down) in blue
    Serial.print(detector.getDerivativeOfDiff13());
    Serial.print(" ");

    //plot a pulse 1-3 - red spikes
    if (m & MOVEMENT_FROM_1_TO_3) {
      Serial.print("20 ");
      //mThreshold = true;
    } else if (m & MOVEMENT_FROM_3_TO_1) {
      Serial.print("-20 ");
      //mThreshold = true;
    } else {
      Serial.print("0 ");
    }

    // movement 2-4 (left - right) in green
    Serial.print(detector.getDerivativeOfDiff24());
    Serial.print(" ");

    //plot a pulse for 2-4 - orange spikes
    if (m & MOVEMENT_FROM_2_TO_4) {
      Serial.println("20 ");
      mThreshold = true;
    } else if (m & MOVEMENT_FROM_4_TO_2) {
      Serial.println("-20 ");
      mThreshold = true;
    } else {
      Serial.println("0 ");
    }
  } else {
    //plot a pulse 1-3 - red spikes
    if (m & MOVEMENT_FROM_1_TO_3) {
      //mThreshold = true;
    } else if (m & MOVEMENT_FROM_3_TO_1) {
      //mThreshold = true;
    } else {
    }

    //plot a pulse for 2-4 - orange spikes
    if (m & MOVEMENT_FROM_2_TO_4) {
      mThreshold = true;
    } else if (m & MOVEMENT_FROM_4_TO_2) {
      mThreshold = true;
    } else {
    }
  }
}

// -------------------------------------------------------------

void initializeMotorsManually() {

  // base
  // run while button is down
  if (buttonState0 == 0) {
    while (buttonState0 == 0) {
      //Serial.println("...setting 0 pos M1");
      s[0].setSpeed(500);
      s[0].run();
      buttonState0 = digitalRead(init_switch0);
    }
    // init start pos on release
    s[0].setCurrentPosition(0);
    Serial.println(" set pos 0, M0 ____________________________");
  }

  // branch
  if (buttonState1 == 0) {
    // run motor 1 (branch)
    while (buttonState1 == 0) {
      //Serial.println("...setting 0 pos M1");
      s[1].setSpeed(-500);
      s[1].run();
      buttonState1 = digitalRead(init_switch1);
    }
    // init start pos on release
    s[1].setCurrentPosition(0);
    Serial.println(" set pos 0, M1 _____________________________");
  }

  // base + branch run forward
  if (buttonState2 == 0) {

    while (buttonState2 == 0) {
      s[0].setSpeed(-500);
      s[0].run();
      //s[1].setSpeed(500);
      //s[1].run();
      buttonState2 = digitalRead(init_switch2);
    }


    Serial.println("Sensor enabled, after button 0 and 1 adjusted motors");
    init0 = true;
    init1 = true;

    Serial.println("positions: ");
    Serial.println(s[0].currentPosition());
    Serial.println(s[1].currentPosition());
    Serial.print("printVar: ");
    Serial.println(printVar);
  }
}


// OLD -------------------------------------------------------------

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


// blocks
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

void printSwitches() {
  Serial.print(" read_0: ");
  Serial.println(buttonState0);
  Serial.print(" read_1: ");
  Serial.println(buttonState1);
  Serial.print(" read_2: ");
  Serial.println(buttonState2);
  Serial.println(" ");
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
