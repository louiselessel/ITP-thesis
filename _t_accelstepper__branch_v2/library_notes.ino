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
