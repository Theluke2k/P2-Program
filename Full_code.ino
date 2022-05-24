/* ----- PINS ----- */
//input from encoders
#define encoderINA 13
#define encoderINB 12

//controllerA
#define controllerINA A7 //Input from the PI controller, thus outout of PI circuit.
#define capResetA 51     //Capacitor reset pin for PI controller.
#define controllerOUTA 5 //Output to PI controller.

//controllerB - same pin description as controllerA.
#define controllerINB A6
#define capResetB 53
#define controllerOUTB 6

//motorA
#define in1 31 //To determine direction of the motor with motordriver.
#define in2 33 //To determine direction of the motor with motordriver.
#define enA 2  //Enable pin for motor. Controls speed of motor with PWM.

//motorB - same pin description as motorA
#define in3 35
#define in4 37
#define enB 3

//EMC32
#define EMC32Pin 49  //Input from EMC32 simulated with button.

//LightsensorA
#define AS2 24
#define AS3 26
#define AsensorOut 4

//LightsensorB skal rettes
#define BS2 25
#define BS3 27
#define BsensorOut 23


/* ----- GLOBAL VARIABLES ----- */
//PLACEMENT
double p = 35;        //Length of the probe holder platform.
double c = 1050;      //Distance between the two pulleys.
double yOffset = 120; //Offset distance from the platform to the probe head.

//POSITION
double currentX;  //Current X position of the probe
double currentY;  //Current Y position of the probe

double nextX;     //Next X position of the probe
double nextY;     //Next Y position of the probe

double currentA;  //Current length of cord A
double currentB;  //Current length of cord B

double nextA;     //Next length of the cord A
double nextB;     //Next length of the cord B

//ACTIVATORS
bool prevEncoderA;    //Used to track the last value of the encoder A signal
bool prevEncoderB;    //Used to track the last value of the encoder B signal

//DIRECTIONS
int dirA = 1;   //The current direction of motor A
int dirB = 2;   //The current direction of motor B

//ERROS
double errorA;  //The current error of cord A
double errorB;  //The current error of cord B

//AVG VALUES
int valuesA[100]; //Array used to average the output of the PI controller A.
int avgA = 0;     //The current average value of the output of PI controller A.

int valuesB[100]; //Array used to average the output of the PI controller B.
int avgB = 0;     //The current average value of the output of PI controller B.



//COLOR SENSOR CALIBRATION VALUES
//COLOR SENSOR A
int redMinA = 57;     //Red minimum value
int redMaxA = 395;    //Red maximum value
int blueMinA = 54;    //Blue minimum value
int blueMaxA = 312;   //Blue maximum value

//COLOR SENSOR B
int redMinB = 41;     //Red minimum value
int redMaxB = 318;    //Red maximum value
int blueMinB = 40;    //Blue minimum value
int blueMaxB = 252;   //Blue maximum value

void setup() {
  Serial.begin(115200);

  //MotorA
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enA, OUTPUT);

  //MotorB
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(enB, OUTPUT);

  //ENCODERS
  pinMode(encoderINA, INPUT);
  pinMode(encoderINB, INPUT);

  //Capacitors
  pinMode(capResetA, OUTPUT);
  pinMode(capResetB, OUTPUT);

  //Color sensor A
  pinMode(AsensorOut, INPUT);
  pinMode(AS2, OUTPUT);
  pinMode(AS3, OUTPUT);

  //Color sensor B
  pinMode(BsensorOut, INPUT);
  pinMode(BS2, OUTPUT);
  pinMode(BS3, OUTPUT);

  //EMC32 button signal
  pinMode(EMC32Pin, INPUT);


  /*
     These if else statements handle if the encoders starts on a 1
     to make sure the error is not immediatly and falsely changed.
  */
  if (digitalRead(encoderINA)) {
    prevEncoderA = true;
  }
  else {
    prevEncoderA = false;
  }

  if (digitalRead(encoderINB)) {
    prevEncoderB = true;
  }
  else {
    prevEncoderB = false;
  }

  //Start the main function.
  Main();
}

void loop() {
  steerMotor(); //To steer the probe with putty, if main is not run
}

/*
   The main function controls the flow of the
   calibration process on a high level.

   Operation:
   It starts by calling the ResetModule() function to begin
   the reset process. Afterwards it starts the
   CalibrationProcess() function to begin the
   calibration process.
*/
void Main() {
  Serial.println("STARTING RESET");
  ResetModule();                            //Start reset process.
  Serial.println("RESET COMPLETE");
  Serial.println("");
  delay(1000);
  Serial.println("STARTING CALIBRATION");
  delay(2000);
  CalibrationProcess(currentX, currentY);   //Start calibration process.
}


/*
   This function executes the reset process of the
   probe.

   Operation:
   It start by initializing some color sensor
   values and thereafter enters a while loop that is
   only broken if both of the color sensors see the
   color black. If a color sensor sees red, the
   corresponding motor changes direction. The color
   sensor values are constantly updated for each loop.
*/
void ResetModule() {
  //Print something awesome
  Serial.println("Reset in progres...");

  //Initialize raw color values
  int redPWA = 0;   //current value of red
  int bluePWA = 0;
  int redPWB = 0;
  int bluePWB = 0;

  //Initialize mapped color values
  int redValueA;
  int blueValueA;
  int redValueB;
  int blueValueB;

  /* ----- MOTOR A ----- */
  // Read Red value
  redPWA = getRedPWA();
  // Map to value from 0-255
  redValueA = map(redPWA, redMinA, redMaxA, 255, 0);

  // Read Blue value
  bluePWA = getBluePWA();
  // Map to value from 0-255
  blueValueA = map(bluePWA, blueMinA, blueMaxA, 255, 0);

  /* ----- MOTOR B ----- */
  // Read Red value
  redPWB = getRedPWB();
  // Map to value from 0-255
  redValueB = map(redPWB, redMinB, redMaxB, 255, 0);

  // Read Blue value
  bluePWB = getBluePWB();
  // Map to value from 0-255
  blueValueB = map(bluePWB, blueMinB, blueMaxB, 255, 0);

  //Activators used to break the loop when the system has found the reset point
  bool resetA = false;
  bool resetB = false;

  while (true) // pull motorA to red
  {
    /* ----- MOTOR A ----- */
    // Read Red value
    redPWA = getRedPWA();
    // Map to value from 0-255
    redValueA = map(redPWA, redMinA, redMaxA, 255, 0);

    // Read Blue value
    bluePWA = getBluePWA();
    // Map to value from 0-255
    blueValueA = map(bluePWA, blueMinA, blueMaxA, 255, 0);

    /* ----- MOTOR B ----- */
    // Read Red value
    redPWB = getRedPWB();
    // Map to value from 0-255
    redValueB = map(redPWB, redMinB, redMaxB, 255, 0);

    // Read Blue value
    bluePWB = getBluePWB();
    // Map to value from 0-255
    blueValueB = map(bluePWB, blueMinB, blueMaxB, 255, 0);

    /* ----- MOTOR A -----
      The values for red and black were found by testing
      the color output of redValue and blueValue, the red
      or black tape were placed in front of the sensor
    */
    if ((redValueA > 200) && (blueValueA < 170)) { //One way to find red
      invertDirA();           //Invert direction if it sees red
      driveMotorA(180, dirA); //Drive motor
      delay(1000);            //Delay so it can get away from red tape
    }
    else if ((redValueA < 200) && (blueValueA < 200) ) {  //Find black
      stopMotorA(); //Stop motor if it sees black
      resetA = true;
    }
    else if (redValueA - 30 > blueValueA) {       //Antoher way to find red
      invertDirA();           //Invert direction if it sees red
      driveMotorA(180, dirA); //Drive motor
      delay(1000);            //Delay so it can get away from red tape
    }
    else {                    //If no color is found, drive the motor in the current direction.
      driveMotorA(180, dirA);
      resetA = false;
    }

    /* ----- MOTOR B -----
      Same comments apply from motor A.
    */
    if ((redValueB > 200) && (blueValueB < 170)) {
      invertDirB();
      driveMotorB(180, dirB);
      resetB = false;
      delay(1000);
    }
    else if ((redValueB < 210) && (blueValueB < 210) ) {
      stopMotorB();
      resetB = true;
    }
    else if (redValueB - 30 > blueValueB) {
      invertDirB();
      driveMotorB(180, dirB);
      resetB = false;
      delay(1000);
    }
    else {
      driveMotorB(180, dirB);
      resetB = false;
    }

    //If both color sensors see black: break the while loop.
    if ((resetA == true) && (resetB == true)) {
      break;
    }

  }
  //Stop motors to make sure they dont move
  stopMotorA();
  stopMotorB();

  //Update the current X and Y positions of the probe.
  currentX = 275;
  currentY = 304;
}

/*
   This function controls the full calibration process.

   Operation:
   First it enters a while loop that only exits when all 4
   points in the UFA have been reached. It then sets the
   coordinates of the next setpoint of the probe and waits
   for the EMC32 signal to begin moving the probe. The
   motorTo100()and motorTo0() functions are used to move
   the probe to the desired position.
*/
void CalibrationProcess(double startX, double startY) {
  //The current position of the probe
  currentX = startX;
  currentY = startY;

  //Number of calibrated points
  int pointsCalibrated = 1;

  //While all points havn't been reached
  while (pointsCalibrated < 4) {
    //Print awesome statement
    Serial.println("Waiting for EMC32 signal...");

    //Wait for EMC32 signal
    waitForEMC32();

    //Determine the setpoint of the probe.
    if (pointsCalibrated == 0) {
      nextX = 275;
      nextY = 304;
    }
    else if (pointsCalibrated == 1) {
      nextX = 275;
      nextY = 804;
    }
    else if (pointsCalibrated == 2) {
      nextX = 775;
      nextY = 804;
    }
    else if (pointsCalibrated == 3) {
      nextX = 775;
      nextY = 304;
    }

    //Print nice things
    Serial.println("(" + String(currentX) + ", " + String(currentY) + ") -> (" + String(nextX) + ", " + String(nextY) + ")");
    delay(1000);
    Serial.println("Moving withing 100mm...");

    motorTo100();       //Use motorTo100() to move probe withing 100mm of the setpoint

    Serial.println("DONE");
    delay(2000);
    Serial.println("Moving to 0...");

    motorTo0();         //Use motorTo0() to move probe withing 0mm of the setpoint

    Serial.println("POINT FOUND");
    Serial.println("");

    pointsCalibrated++; //Increment number of calibrated points by 1.
  }

  Serial.println("CALIBRATION COMPLETE");
}

/*
   This function moves the probe within 100mm of the setpoint.

   Operation:
   The current length of the cords are calculated with the
   placement() function based on the current coordinates of the
   probe. The error is then calculated with the getError()
   functions. The program then enters a while loop than can
   only be broken if the error of both cords have been within
   98-102mm in at least one second.
*/
void motorTo100() {
  //Get lengths of the cords
  currentA = placement(currentX, currentY, "A");
  currentB = placement(currentX, currentY, "B");

  //Get the current errors
  double errorA = getErrorA();
  double errorB = getErrorB();

  //Print info
  Serial.println("Error A: " + String(errorA) + " | Error B: " + String(errorB));

  //Initialize timer variables
  double timerA;
  double timerB;

  while (true) {
    //Read from the encoder to update "currentA"
    encoderReadA();

    //Read from the encoder to update "currentB"
    encoderReadB();

    //Update both error values
    errorA = getErrorA();
    errorB = getErrorB();

    /* ----- MOTOR A | error over 0 ----- */
    if (errorA > 0) {           //ErrorA is over 0
      if (errorA > 150) {         //ErrorA is over 150
        dirA = 2;                   //Direction: slack
        driveMotorA(200, dirA);     //Drive fast
      }
      else if (errorA > 102) {    //Error is over 101
        dirA = 2;                   //Direction: slack
        driveMotorA(80, dirA);      //Drive slow
      }
      else if (errorA < 98) {     //Error is under 98
        dirA = 1;                   //Direction: pull
        driveMotorA(80, dirA);      //Drive slow
      }
      else {                      //If error is between 98 and 102
        stopMotorA();               //Stop motor
      }
    }

    /* ----- MOTOR A | error under 0 ----- */
    if (errorA < 0) {           //Error is under 0
      if (errorA < -150) {        //Error is under -150
        dirA = 1;                   //Direction: pull
        driveMotorA(200, dirA);     //Drive fast
      }
      else if (errorA < -102) {   //Error under -101
        dirA = 1;                   //Direction: pull
        driveMotorA(80, dirA);      //Drive slow
      }
      else if (errorA > -98) {    //Error is over -98
        dirA = 2;                   //Direction: slack
        driveMotorA(80, dirA);      //Drive slow
      }
      else {                      //If error is between -98 and -102
        stopMotorA();               //Stop motor
      }
    }

    /* ----- MOTOR B | error over 0 -----
      Same comments apply as to motor A | error over 0
    */
    if (errorB > 0) {
      if (errorB > 150) {
        dirB = 2;
        driveMotorB(200, dirB);
      }
      else if (errorB > 102) {
        dirB = 2;
        driveMotorB(80, dirB);
      }
      else if (errorB < 98) {
        dirB = 1;
        driveMotorB(80, dirB);
      }
      else {
        stopMotorB();
      }
    }

    /* ----- MOTOR B | error under 0 -----
      Same comments apply as to motor A | error under 0
    */

    if (errorB < 0) {
      if (errorB < -150) {
        dirB = 1;
        driveMotorB(200, dirB);
      }
      else if (errorB < -102) {
        dirB = 1;
        driveMotorB(80, dirB);
      }
      else if (errorB > -98) {
        dirB = 2;
        driveMotorB(80, dirB);
      }
      else {
        stopMotorB();
      }
    }

    /*
       If the absolute value of error A is over 102
       or under 98, the timerA variable is updated
       with the current time.
    */
    if ((abs(errorA) > 102) || (abs(errorA) < 98)) {
      timerA = millis();
    }

    /*
       If the absolute value of error B is over 102
       or under 98, the timerA variable is updated
       with the current time.
    */
    if ((abs(errorB) > 102) || (abs(errorB) < 98)) {
      timerB = millis();
    }

    /*
       If the timer variables are added by 1000
       milliseconds, and are still under the current
       time, it means that the error has been in the
       correct interval for at least 1 second, and
       the while loop breaks.
    */
    if ((timerA + 1000 < millis()) && (timerB + 1000 < millis())) {
      //Serial.println("MILLIS: " + String(millis()) + " | TIMERA: " + String(timerA) + " | TIMERB: " + String(timerB));
      break;
    }
  }
  //Stop the motors to make sure they dont move.
  stopMotorA();
  stopMotorB();
}

/*
   This function moves the probe within 0mm of the setpoint
   by using the PI controller.

   Operation:
   It starts by reading the encoders to update the current
   length of the cord. The error is then updated and the
   capacitors in the PI controller are discharged to make sure
   they aren't charged. The program enters a while loop that only breaks
   when the absolute value of both of the errors are under 1.
   The sendError() functions sends the voltage representation
   of the current error to the PI circuits with a PWM signal.
   The output of the PI controllers are then read by the
   controllerRead() functions and saved to variables. The
   motors are then driven with the speed that the controllerReader()
   functions returned. Lastly, if the direction of the motors are
   controlled by the sign of the error.
*/
void motorTo0() {
  //Read encoder values to update current cord lengths
  encoderReadA();
  encoderReadB();

  //Update the current error
  errorA = getErrorA();
  errorB = getErrorB();

  //Reset capacitors in PI controller circuit.
  resetCapA();
  resetCapB();

  while (true) {
    //Update current cord lengths
    encoderReadA();
    encoderReadB();

    //Update the current error
    errorA = getErrorA();
    errorB = getErrorB();

    /* ----- MOTOR A ----- */
    //Set the direction of the motor based on the sign of the error
    if (errorA > 0) {
      dirA = 2;
    }
    else if (errorA < 0) {
      dirA = 1;
    }

    //Send the voltage representation of the error to the PI controller
    sendErrorA(errorA);

    //Read the output from the PI controller
    int PWMtoMotorA = controllerReaderA();

    //Drive the motor with the speed that the controller calculated
    driveMotorA(PWMtoMotorA, dirA);


    /* ----- MOTOR B -----
      Same comments apply as to the MOTOR A section
    */
    //Set direction
    if (errorB > 0) {
      dirB = 2;
    }
    else if (errorB < 0) {
      dirB = 1;
    }

    //Send error
    sendErrorB(errorB);

    //Read error
    int PWMtoMotorB = controllerReaderB();

    //Drive motor
    driveMotorB(PWMtoMotorB, dirB);


    /*
       If the absolute value of both of the errors are
       under 1, the while loop breaks and the setpoint
       is reached.
    */
    if ((abs(errorA) < 1) && (abs(errorB) < 1)) {
      stopMotorA();
      stopMotorB();
      break;
    }
  }
  //Stop the motors to make sure they dont move
  stopMotorA();
  stopMotorB();

  //Reset the capacitors
  resetCapA();
  resetCapB();

  //Update the current placement of the probe
  currentX = nextX;
  currentY = nextY;

  //Empty measurement arrays
  emptyArrA();
  emptyArrB();
}

/*
   Function that resets the capacitor in PI controller A
*/
void resetCapA() {
  digitalWrite(capResetA, HIGH);
  delay(300);
  digitalWrite(capResetA, LOW);
}

/*
   Function that resets the capacitor in PI controller B
*/
void resetCapB() {
  digitalWrite(capResetB, HIGH);
  delay(300);
  digitalWrite(capResetB, LOW);
}

/*
   Function that sends the current error A to the
   PI controller A circuit as a PWM signal.
*/
void sendErrorA(double errorPWM) {
  //Map the 0-100mm error A to a value from 0-255
  double errorVal = map(abs(errorPWM), 0, 102, 0, 255);

  //Send the PWM signal to PI controller A
  analogWrite(controllerOUTA, errorVal);
}

/*
   Function that sends the current error B to the
   PI controller B circuit as a PWM signal.
*/
void sendErrorB(double errorPWM) {
  //Map the 0-100mm error A to a value from 0-255
  double errorVal = map(abs(errorPWM), 0, 102, 0, 255);

  //Send the PWM signal to PI controller B
  analogWrite(controllerOUTB, errorVal);
}

/*
   Function that reads the output of PI controller A
   circuit and returns the average output value based on
   100 measurements.
*/
double controllerReaderA() {
  //Read the output voltage of PI controller A
  double val = analogRead(controllerINA);

  //Map the output voltage to a PWM value
  double valMappedToPWM = map(val, 0.0, 1023.0, 0.0, 255);

  //Insert the value in an array that gets the average output
  avgMotorOutA(valMappedToPWM);

  //Return the calculated average output
  return avgA;
}

/*
   Function that reads the output of PI controller B
   circuit and returns the average output value based on
   100 measurements.
*/
double controllerReaderB() {
  //Read the output voltage of PI controller B
  double val = analogRead(controllerINB);

  //Map the output voltage to a PWM value
  double valMappedToPWM = map(val, 0.0, 1023.0, 0.0, 255);

  //Insert the value in an array that gets the average output
  avgMotorOutB(valMappedToPWM);

  //Return the calculated average output
  return avgB;
}

/*
   Function that insert a value into an array and
   calculates the average value of all the values
   in the array. When the array is full, the oldest
   value gets replaced with the new. The function is
   written for error A.
*/
void avgMotorOutA(int error) {
  //Resets the average value to 0 in case it isn't already.
  avgA = 0;

  //Initialize copy array to move values in real array.
  int copyArr[100];

  //Copy all the values from the "valuesA" array into the "copyArr" array.
  memcpy(copyArr, valuesA, sizeof(valuesA));

  /*
     For each value in the "valuesA" array (except index [99]),
     copy the old value in copyArr,, into the index under the last.
     This is the same as shifting all values in the "valuesA" array
     one index down. Or (to visualize) shifting all the values one
     index to the left.
  */
  for (int i = 0; i < 99; i++) {
    valuesA[i] = copyArr[i + 1];
  }

  //Insert the newest value into index 99 of the array
  valuesA[99] = error;

  //Add all the values in the aray together.
  for (int i = 0; i < 100; i++) {
    avgA += valuesA[i];
  }

  //Divide by the number of values to get the average value.
  avgA = avgA / 100;
}

/*
   Function that insert a value into an array and
   calculates the average value of all the values
   in the array. When the array is full, the oldest
   value gets replaced with the new. The function is
   written for error B.

   The same comments apply as for the avgMotorOutA() function.
*/
void avgMotorOutB(int error) {
  //Resets average value
  avgB = 0;

  //Initialize copy array
  int copyArr[100];

  //Insert values of real array into copy array
  memcpy(copyArr, valuesB, sizeof(valuesB));

  //Shift array values one index down
  for (int i = 0; i < 99; i++) {
    valuesB[i] = copyArr[i + 1];
  }

  //Insert newest value in index 99
  valuesB[99] = error;

  //Add all values together
  for (int i = 0; i < 100; i++) {
    avgB += valuesB[i];
  }

  //Find average value
  avgB = avgB / 100;
}

/*
   Function to set all of the values in the "valuesA"
   array to 0. The values need to be reset so that the
   next point that uses the PI controller doesn't use
   old ouput values from the last point.
*/
void emptyArrA() {
  for (int i = 0; i < 100; i++) {
    valuesA[i] = 0;
  }
}

/*
   Function to set all of the values in the "valuesB"
   array to 0. The values need to be reset so that the
   next point that uses the PI controller doesn't use
   old ouput values from the last point.
*/
void emptyArrB() {
  for (int i = 0; i < 100; i++) {
    valuesB[i] = 0;
  }
}

/*

   Function to read the encoder value and
   update the current length of cord A.
*/
void encoderReadA() {
  //Read the encoder output.
  int val = digitalRead(encoderINA);

  /* ----- ENCODER A ----- */
  if (val == 0) {                     //If the encoder output is 0.
    prevEncoderA = false;               //Set previous value to false.
  }

  if (dirA == 1) {                    //If direction is pull.
    if (val == 1 && !prevEncoderA) {    //If the encoder output is 1 and the previous value was false.
      currentA -= countsToMM(1);          //Decrement the current length of cord A.
      prevEncoderA = true;                //Set the previous value to true.
    }
  }
  if (dirA == 2) {                    //If the direction is slack
    if (val == 1 && !prevEncoderA) {    //If the encoder output is 1 and the previous value was false.
      currentA += countsToMM(1);          //Increment the current length of cord A.
      prevEncoderA = true;                //Set the previous value to true.
    }
  }
}

/* ----- ENCODER A ----- */
void encoderReadB() {
  //Read the encoder output.
  int val = digitalRead(encoderINB);

  if (val == 0) {                     //If the encoder output is 0.
    prevEncoderB = false;               //Set previous value to false.
  }

  if (dirB == 1) {                    //If direction is pull.
    if (val == 1 && !prevEncoderB) {    //If the encoder output is 1 and the previous value was false.
      currentB -= countsToMM(1);          //Decrement the current length of cord A.
      prevEncoderB = true;                //Set the previous value to true.
    }
  }
  if (dirB == 2) {                    //If the direction is slack
    if (val == 1 && !prevEncoderB) {    //If the encoder output is 1 and the previous value was false.
      currentB += countsToMM(1);          //Increment the current length of cord A.
      prevEncoderB = true;                //Set the previous value to true.
    }
  }
}

/*
   Function that converts an amount of counts
   on the encoder to a length in mm. This is based
   on the circumference of the wheel that turns the
   encoder when the cord moves.
*/
double countsToMM(int counts) {
  //Amount of counts per rotation of the encoder
  int PPM = 200;

  //Circumference of the wheel that rotates the encoder
  double circum = 188.5;

  //Amount of mm per encoder count.
  double mmPerCount = circum / PPM;

  //The total length in mm based on the amount of counts.
  double mm = counts * mmPerCount;

  //Return the total length.
  return mm;

}

/*
   Function that gets the current error of cord A.
*/
double getErrorA() {
  //Get the required length of cord A
  double nextA = placement(nextX, nextY, "A");

  //Calculate the difference between the current length and the required length.
  double errorA = nextA - currentA;

  //Return the error.
  return errorA;
}

/*
   Function that gets the current error of cord B.
*/
double getErrorB() {
  //Get the required length of cord B
  double nextB = placement(nextX, nextY, "B");

  //Calculate the difference between the current length and the required length.
  double errorB = nextB - currentB;

  //Return the error
  return errorB;
}

/*
   Function that calculates the required length of the
   cords to obtain a specified point in the UFA.
*/
double placement(double x, double y, String AorB) {
  //The height of the trapeziod is equal to the y coordinate
  double h = y + yOffset;

  //c1 is equal to the X coordinate of the next point, minus half of the platform length.
  double c1 = x - (p / 2);

  //c2 is equal to the length c, minus the X coordinate of the next point, plus half of the platform length.
  double c2 = c - (x + ( p / 2));

  //Find length a and b by Pythagoras theorem.
  double a = sqrt(pow(h, 2) + pow(c1, 2));
  double b = sqrt(pow(h, 2) + pow(c2, 2));

  //Return cord length a, if the function argument specified it.
  if (AorB == "A") {
    return a;
  }
  //Return cord length b, if the function argument specified it
  if (AorB == "B") {
    return b;
  }

  //If nothing was specified, return 0.
  else {
    return 0;
  }
}

/*
   Function that drives motor A with a specified speed
   in a specified direction.

   Specifications:
   DIR = 1 -> PULL WIRE
   DIR = 2 -> SLACK WIRE
*/
void driveMotorA(int speedPWM, int dir) {
  //If direction is pull, set in1 and in2 pins
  if (dir == 1) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }
  //If direction is slack, set in1 and in2 pins
  else if (dir == 2) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }

  //Send a PWM signal to the enable pin of motor A, with the specified speed.
  analogWrite(enA, speedPWM);
}

/*
   Function that drives motor B with a specified speed
   in a specified direction.

   Specifications:
   DIR = 1 -> PULL WIRE
   DIR = 2 -> SLACK WIRE
*/
void driveMotorB(int speedPWM, int dir) {
  //If direction is pull, set in3 and in4 pins
  if (dir == 1) {
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
  }
  //If direction is slack, set in3 and in4 pins
  else if (dir == 2) {
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
  }
  //Send a PWM signal to the enable pin of motor B, with the specified speed.
  analogWrite(enB, speedPWM);
}

/*
 * Function to stop motor A.
 */
void stopMotorA() {
  //Set speed to 0.
  analogWrite(enA, 0);

  //Set direction pins to stop.
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
}
/*
 * Function to stop motor B.
 */
void stopMotorB() {
  //Set speed to 0.
  analogWrite(enB, 0);

  //Set direction pins to stop.
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);

}

/*
 * Function to move the probe based on a serial input.
 * (Used for tests).
 */
void steerMotor() {
  //Read serial input
  if (Serial.available() > 0) {
    //Set speed of motors.
    int speedT = 200;
    
    //Set different motor directions based on serial input.
    switch (Serial.read())
    {
      case 97: // a pull both
        dirA = 1;
        dirB = 1;
        driveMotorA(speedT, dirA);
        driveMotorB(speedT, dirB);
        break;

      case 115: // s slack both
        dirA = 2;
        dirB = 2;
        driveMotorA(speedT, dirA);
        driveMotorB(speedT, dirB);
        break;

      case 100: // d pull A slack B
        dirA = 1;
        dirB = 2;
        driveMotorA(speedT, dirA);
        driveMotorB(speedT, dirB);
        break;

      case 102: // f slack A pull B
        dirA = 2;
        dirB = 1;
        driveMotorA(speedT, dirA);
        driveMotorB(speedT, dirB);
        break;


      case 113: // q pull A
        dirA = 1;
        driveMotorA(speedT, dirA);
        break;

      case 119: // w slack A
        dirA = 2;
        driveMotorA(speedT, dirA);
        break;

      case 101: // e pull B
        dirB = 1;
        driveMotorB(speedT, dirB);
        break;

      case 114: // r slack B
        dirB = 2;
        driveMotorB(speedT, dirB);
        break;


      case 116: // t stop both
        stopMotorA();
        stopMotorB();
        break;

      case 127: // space stop both
        stopMotorA();
        stopMotorB();
        break;

      default:  //default stop both (doesnt work)
        stopMotorA();
        stopMotorB();
    }
  }
}

/*
 * Function to get the raw red value of color sensor A.
 */
int getRedPWA() {
  // Set sensor to read Red only
  digitalWrite(AS2, LOW);
  digitalWrite(AS3, LOW);
  
  // Define integer to represent Pulse Width
  int PW;
  
  // Read the output Pulse Width
  PW = pulseIn(AsensorOut, LOW);
  
  // Return the value
  return PW;
}

/*
 * Function to get the raw blue value of color sensor A.
 */
int getBluePWA() {
  //Set sensor to read Blue only
  digitalWrite(AS2, LOW);
  digitalWrite(AS3, HIGH);
  
  //Define integer to represent Pulse Width
  int PW;
  
  //Read the output Pulse Width
  PW = pulseIn(AsensorOut, LOW);
  
  //Return the value
  return PW;
}

/*
 * Function to get the raw red value of color sensor B.
 */
int getRedPWB() {
  //Set sensor to read Red only
  digitalWrite(BS2, LOW);
  digitalWrite(BS3, LOW);
  
  //Define integer to represent Pulse Width
  int PW;
  
  //Read the output Pulse Width
  PW = pulseIn(BsensorOut, LOW);
  
  //Return the value
  return PW;
}



/*
 * Function to get the raw blue value of color sensor B.
 */
int getBluePWB() {
  //Set sensor to read Blue only
  digitalWrite(BS2, LOW);
  digitalWrite(BS3, HIGH);
  
  //Define integer to represent Pulse Width
  int PW;
  
  //Read the output Pulse Width
  PW = pulseIn(BsensorOut, LOW);
  
  //Return the value
  return PW;
}


/*
 * Function to invert the current direction of motor A.
 */
void invertDirA() {
  //If the current direction is 1, set it to 2.
  if (dirA == 1) {
    dirA = 2;
  }
  
  //If the current direction is 2, set it to 1.
  else if (dirA == 2) {
    dirA = 1;
  }
}

/*
 * Function to invert the current direction of motor A.
 */
void invertDirB() {
  //If the current direction is 1, set it to 2.
  if (dirB == 1) {
    dirB = 2;
  }
  
  //If the current direction is 2, set it to 1.
  else if (dirB == 2) {
    dirB = 1;
  }
}

/*
 * Function to wait for the EMC32 signal.
 * This is simulated with a button press.
 */
void waitForEMC32() {
  //Continue the while loop until digitalRead of the EMC32 pin reads HIGH.
  while (digitalRead(EMC32Pin) == 0) {
    //Waiting :)
  }
}
