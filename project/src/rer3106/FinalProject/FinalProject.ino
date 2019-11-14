// ===========================================================
//
// FinalProject.ino  
// Description: Final Project
// Name: Robert Reed, Brendan Deller
// Date: November 14, 2019
// Class: CMPE-110
// Section: <Lab: 5, Thursday, 5:00PM>
// ===========================================================


#include <Romi_Motor_Power.h>
#include <Robot_Pins_v1.h>
#include <Encoder.h>
#include <Bump_Switch.h>
#include <QTRSensors.h>

// 'Start' button
#define BUTTON_PIN PUSH2



// LED pins
#define LED_PIN RED_LED
#define GREEN_LED_PIN GREEN_LED

// Motor
#define MAX_SPEED 100           // Fastest speed
#define MIN_SPEED 1             // Slowest speed

// Define the motors
Romi_Motor_Power left_motor;
Romi_Motor_Power right_motor;
Bump_Switch      bump_sw[6];

// Line sensor defines
#define TOTAL_LS_SENSORS 8
#define LS_IR_PIN_EVEN LS_PIN_IR
#define LS_IR_PIN_ODD 45
QTRSensorsRC qtrc;
unsigned int lineSensorValues[TOTAL_LS_SENSORS];

// variables for testing line sensors
int prevValueLeft = 0; // previous left line sensor value
int prevValueRight = 0; // previous right line sensor value
int prevValueCenter = 0; // previous center line sensor value

  boolean leftLine;
 boolean rightLine;
 boolean centerLine;


void setup() 
{ 
  Serial.begin(19200); //for serial IO to screen
  Serial.println("Welcome to my distance tester!");

  pinMode(BUTTON_PIN, INPUT_PULLUP); // setup 'start' button so it is LOW when pressed
  pinMode(LED_PIN, OUTPUT); // setup user LED to be an output
  pinMode(GREEN_LED_PIN, OUTPUT);

  left_motor.begin(MOTOR_L_SLP_PIN, MOTOR_L_DIR_PIN, MOTOR_L_PWM_PIN);
  right_motor.begin(MOTOR_R_SLP_PIN, MOTOR_R_DIR_PIN, MOTOR_R_PWM_PIN);

  setupEncoder(ENCODER_ELB_PIN, ENCODER_ELA_PIN, ENCODER_ERB_PIN, ENCODER_ERA_PIN);
  // setup the bump switches
  bump_sw[0].begin(BSW_PIN_1);
  bump_sw[1].begin(BSW_PIN_2);
  bump_sw[2].begin(BSW_PIN_3);
  bump_sw[3].begin(BSW_PIN_4);
  bump_sw[4].begin(BSW_PIN_5);
  bump_sw[5].begin(BSW_PIN_6);

    // setup line sensors
  unsigned char pins[] = {LS_PIN_1, LS_PIN_2, LS_PIN_3, LS_PIN_4, LS_PIN_5, LS_PIN_6, LS_PIN_7, LS_PIN_8};
  qtrc.init(pins, TOTAL_LS_SENSORS);

  // set the line sensor IR pins to HIGH
  pinMode(LS_IR_PIN_ODD, OUTPUT);
  pinMode(LS_IR_PIN_EVEN, OUTPUT);
  digitalWrite(LS_IR_PIN_EVEN, HIGH);
  digitalWrite(LS_IR_PIN_ODD, HIGH);

  leftLine=false;
  rightLine=false;
  centerLine=false;
  clearEncoders();
}

void loop() {
  // put your main code here, to run repeatedly: 
    pressToStart();          // wait for 'Start' button to be pressed
    followLine();
    //first();
    //driveUntilLine();
    //drawSquare();
//    forwardDistance(12);
//    makeLeftTurn(10,180);
//    forwardDistance(12);
//    makeRightTurn(10,180);
}

void followLine(){
    testLine(850);

  while(true){
    if(centerLine==true){
                  digitalWrite(GREEN_LED_PIN, HIGH);

    testLine(850);
  fwd(10);
    }
  if(leftLine==true){
    makeRightTurn(10, 45);
    }
  else if(rightLine==true)
    makeLeftTurn(10,45);
  
  }
                digitalWrite(GREEN_LED_PIN, LOW);

  stopBothMotors();
}

void first(){
    forwardDistance(30);
    makeRightTurn(10,180);
        forwardDistance(15);
    makeLeftTurn(10,180);
    driveUntilLine();
    makeLeftTurn(10,180);
    driveUntilBump();
  }

//Testing
void drawSquare(){
  forwardDistance(12);
    makeLeftTurn(10,180);
    forwardDistance(12);
    makeLeftTurn(10,180);
    forwardDistance(12);
    makeLeftTurn(10,180);
    forwardDistance(12);
    makeLeftTurn(10,180);
    
  }


// Function:    readLineSensors
// Description: update the lineSensorValues variable
// Inputs:      none
// Returns:     none
void readLineSensors()
{
  qtrc.read(lineSensorValues);
}

// Function:    driveUntilBump
// Description: drive robot forward until it detects a bump
// Inputs:      none
// Returns:     none
void driveUntilLine()
{
      testLine(850);

      

  while(leftLine==true || rightLine==true || centerLine==true) { // Loop for like ever
      fwd(10);
      testLine(850);
            if(leftLine==true || rightLine==true || centerLine==true){
              digitalWrite(GREEN_LED_PIN, HIGH);
            }
            else{
                            digitalWrite(GREEN_LED_PIN, LOW);

              }
        }
  
  stopBothMotors();

}






// Function:    testLine
// Description: prints the readings from all line sensors, if any have changed.
//              the sensor labels are upper-case if the value is above LINETHRESHOLD
// Inputs:      threshold - 1 (very light) to 1024 (very dark)
// Returns:     none
void testLine(unsigned long threshold)
{
  // update line sensor readings
  readLineSensors();

  int lineSensorValueRight = ((3 * lineSensorValues[0]) + (2 * lineSensorValues[1]) + lineSensorValues[2]) / 6;
  int lineSensorValueLeft = ((3 * lineSensorValues[7]) + (2 * lineSensorValues[6]) + lineSensorValues[5]) / 6;
  int lineSensorValueCenter = (lineSensorValues[3] + lineSensorValues[4]) / 2;

  // see if anything changed
  if ((lineSensorValueLeft != prevValueLeft) || (lineSensorValueRight != prevValueRight) || (lineSensorValueCenter != prevValueCenter))
  {
    // remember new counters
    prevValueLeft = lineSensorValueLeft;
    prevValueRight = lineSensorValueRight;
    prevValueCenter = lineSensorValueCenter;

    if (lineSensorValueLeft > threshold)
    {
       leftLine=true;
    }
    else
    {
       leftLine=false;
      }

    if (lineSensorValueCenter > threshold)
    {
    centerLine=true;   
    }
    else
    {
    centerLine=false;
    }

    if (lineSensorValueRight > threshold)
    {
    rightLine=true;  
    }
    else
    {
    rightLine=false;
    }
  }
}


// Function:    driveUntilBump
// Description: drive robot forward until it detects a bump
// Inputs:      none
// Returns:     none
void driveUntilBump()
{

  while(true) { // Loop for like ever
    if((bump_sw[0].read() == 0 || bump_sw[1].read() == 0 || bump_sw[2].read() == 0 || bump_sw[3].read() == 0 || bump_sw[4].read() == 0 || bump_sw[5].read() == 0))     // in the case that there are any bump switches activated, stop the motors
      stopBothMotors();
    else // Otherwise, allow the robot to drive forward
      fwd(30);
  }
}

void makeLeftTurn(int speed,int counts){
    clearEncoders();          // Reset the encoder values

  turnLeft(speed);             // turn to the left
  waitEncoderLeft(counts);  // wait until it turns enough
  clearEncoders();          // Reset the encoder values
  }

void makeRightTurn(int speed, int counts){
    clearEncoders();          // Reset the encoder values

  turnRight(speed);             // turn to the left
  waitEncoderRight(counts);  // wait until it turns enough
  clearEncoders();          // Reset the encoder values
  }
//Ours
void forwardDistance(float inches){
  int speed = 10;
  int counts = (int)(inches / 0.02465278);
  long    rCount = 0;
  long lCount = 0;
  clearEncoders();
  fwd(speed);
  while(rCount<counts && lCount<counts){
      digitalWrite(GREEN_LED_PIN, HIGH);

    rCount = abs(getEncoderRightCnt());
    lCount = abs(getEncoderLeftCnt());
  }
  stopBothMotors();
  digitalWrite(GREEN_LED_PIN, LOW);
  while(rCount<lCount){
      digitalWrite(GREEN_LED_PIN, HIGH);
          rCount = abs(getEncoderRightCnt());
    lCount = abs(getEncoderLeftCnt());
  right_motor.setSpeed(speed);
      right_motor.enableMotor();
  }
  stopBothMotors();
digitalWrite(GREEN_LED_PIN, LOW);
  while(lCount<rCount){
      digitalWrite(GREEN_LED_PIN, HIGH);
          rCount = abs(getEncoderRightCnt());
    lCount = abs(getEncoderLeftCnt());
left_motor.setSpeed(speed);
      left_motor.enableMotor();
  }
  stopBothMotors();
  digitalWrite(GREEN_LED_PIN, LOW);
    clearEncoders();          // Reset the encoder values

}


// Function:    fwd
// Description: set motor speed so robot drives forward
// Inputs:      speed - the speed the robot will move, 1=slow, 255=fast
// Returns:     none
void fwd(int speed)
{ 
  // validate the speed value
  if (speed < MIN_SPEED) speed = MIN_SPEED;
  if (speed > MAX_SPEED) speed = MAX_SPEED;
  
  // display the function we are in and its values
  Serial.print("forward:    ");
  Serial.println(speed, DEC);  

  // disable both motors
  left_motor.disableMotor();
  right_motor.disableMotor();

  // set both motors to forward
  left_motor.directionForward();
  right_motor.directionForward();
  
  // set speed for both motors
  left_motor.setSpeed(speed);
  right_motor.setSpeed(speed);

  // enable both motors
  left_motor.enableMotor();
  right_motor.enableMotor();
}

// Function:    clearEncoders
// Description: clears encoder counts (this stops the motors)
// Inputs:      none
// Returns:     none
void clearEncoders()
{
  stopBothMotors(); // stop the motors so coutns do not hcange on us
  wait(250);    // give wheels a chance to actually stop

  // clear the encoder counts
  resetLeftEncoderCnt();
  resetRightEncoderCnt();
}


// Function:    pressToStart
// Description: wait for the 'Start' button to be pressed
// Inputs:      none
// Returns:     none
// 
void pressToStart()
{
  unsigned long startTime;
  bool pressed = false;

  Serial.println(""); // print blank line
  Serial.println("Waiting for 'Start' button to be pressed...");

  // keep blinking LED until button is pressed
  while (!pressed)
  {
    // keep LED off for 1 sec, or until button is pressed
    digitalWrite(LED_PIN, LOW);  // turn LED off
    startTime = millis();        // get current time NOTE: the micros() counter will overflow after ~50 days
    while (!pressed && ((millis() - startTime) < 500))
    {
      // see if button is pressed (i.e. LOW)
      if (digitalRead(BUTTON_PIN) == LOW)
      {
        pressed = true;
      }
    }

    // keep LED on for 1 sec, or until button is pressed
    digitalWrite(LED_PIN, HIGH); // turn LED on
    startTime = millis();        // get current time NOTE: the millis() counter will overflow after ~50 days
    while (!pressed && ((millis() - startTime) < 500))
    {
      // see if button is pressed (i.e. LOW)
      if (digitalRead(BUTTON_PIN) == LOW)
      {
        pressed = true;
      }
    }
  }

  // make sure LED is off
  digitalWrite(LED_PIN, LOW);

  // give user a chance to remove their finger
  wait(1000);

  Serial.println(""); // print blank line
}


// Function:    stopMotors
// Description: stops the wheel motors
// Inputs:      none
// Returns:     none
void stopBothMotors()
{
  left_motor.setSpeed(0);
  right_motor.setSpeed(0);

  Serial.println("stopped");
}
// Function:    wait
// Description: wait for the desired length of time, before this function returns
// Inputs:      duration - number of milliseconds before this function returns
// Returns:     none
void wait(int duration)
{ 
  // display the function we are in and its values
  Serial.print("wait:       ");
  Serial.print(duration, DEC);
  Serial.println("ms");

  // wait for the desired time
  delay(duration);
}

// Function:    turnLeft
// Description: set motor speed so robot turns to the left
// Inputs:      speed - the speed the robot will turn, 1=slow, 255=fast
// Returns:     none
void turnLeft(int speed)
{
  // validate the speed value
  if (speed < MIN_SPEED) speed = MIN_SPEED;
  if (speed > MAX_SPEED) speed = MAX_SPEED;

  // display the function we are in and its values
  Serial.print("turn left:  ");
  Serial.println(speed, DEC);

  // disable both motors
  left_motor.disableMotor();
  right_motor.disableMotor();

  // set left motor backward and right motor forward
  left_motor.directionBackward();
  right_motor.directionForward();

  // set speed for both motors
  left_motor.setSpeed(speed);
  right_motor.setSpeed(speed);

  // enable both motors
  left_motor.enableMotor();
  right_motor.enableMotor();
}


// Function:    turnRight
// Description: set motor speed so robot turns to the right
// Inputs:      speed - the speed the robot will turn, 1=slow, 255=fast
// Returns:     none
void turnRight(int speed)
{
  // validate the speed value
  if (speed < MIN_SPEED) speed = MIN_SPEED;
  if (speed > MAX_SPEED) speed = MAX_SPEED;

  // display the function we are in and its values
  Serial.print("turn right: ");
  Serial.println(speed, DEC);

  // disable both motors
  left_motor.disableMotor();
  right_motor.disableMotor();

  // set left motor forward and right motor backward
  left_motor.directionForward();
  right_motor.directionBackward();

  // set speed for both motors
  left_motor.setSpeed(speed);
  right_motor.setSpeed(speed);

  // enable both motors
  left_motor.enableMotor();
  right_motor.enableMotor();
}

// Function:    waitEncoderLeft
// Description: waits for left wheel encoder to change the desried amount
// Inputs:      counts - number of encoder counts to wait for
// Returns:     none
void waitEncoderLeft(int counts)
{
  long    lCount = 0;     // left motor encoder counts

  while (true)
  {
    // get current value
    lCount = abs(getEncoderLeftCnt());

    // print new counter value
    Serial.print("wait left:  ");
    Serial.print(lCount, DEC);
    Serial.print("\twant: ");
    Serial.print(counts, DEC);
    Serial.println("");

    if (lCount >= counts)
    {
      return;
    }
  }
}



// Function:    waitEncoderRight
// Description: waits for right wheel encoder to change the desried amount
// Inputs:      counts - number of encoder counts to wait for
// Returns:     none
void waitEncoderRight(int counts)
{
  long    rCount = 0;     // right motor encoder counts

  while (true)
  {
    // get current value
    rCount = abs(getEncoderRightCnt());

    // print new counter value
    Serial.print("wait right: ");
    Serial.print(rCount, DEC);
    Serial.print("\twant: ");
    Serial.print(counts, DEC);
    Serial.println("");

    if (rCount >= counts)
    {
      return;
    }
  }
}