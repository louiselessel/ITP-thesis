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





/* NOT IN USE */

// from loop
  /*
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
  */

/*
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
*/
