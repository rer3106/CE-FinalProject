// ===========================================================
//
// Lab9_Sensor.ino  
// Description: Robot sensor tester
// Name: Robert Reed, Brendan Deller
// Date: 10/31/19
// Class: CMPE-110
// Section: <Lab: 5, Thursday, 5:00PM>
//
// ===========================================================
//
#include <Romi_Motor_Power.h>
#include <Robot_Pins_v1.h>
#include <Bump_Switch.h>
#include <QTRSensors.h>
#include <Encoder.h>

// 'Start' button
#define BUTTON_PIN PUSH2

// Available LED pins
#define RED_LED_PIN RED_LED
#define GREEN_LED_PIN GREEN_LED

// Define motor speeds
#define MAX_SPEED 100           // Fastest speed
#define MIN_SPEED 1             // Slowest speed

// Define the motors
Romi_Motor_Power left_motor;
Romi_Motor_Power right_motor;

// Define the bump switches
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

void setup() 
{ 
  // setup serial output
  Serial.begin(19200);
  Serial.println("Welcome to my sensor tester!");

  // setup start button
  pinMode(BUTTON_PIN, INPUT_PULLUP); // setup 'start' button so it is LOW when pressed
  
  // setup LED pins to be used as outputs
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);

  // setup left and right motors
  left_motor.begin(MOTOR_L_SLP_PIN, MOTOR_L_DIR_PIN, MOTOR_L_PWM_PIN);
  right_motor.begin(MOTOR_R_SLP_PIN, MOTOR_R_DIR_PIN, MOTOR_R_PWM_PIN);

  // setup encoder pins
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

  // reset the encoder counts
  clearEncoders();
}

void loop()
{
  pressToStart();          // wait for 'Start' button to be pressed

  while (true)
  {
    // uncomment the following line to run testBump
    //testBump();

    // uncomment the following line to print the line following sensor values
    testLine(0);

    // uncomment the following line to run driveUntilBump
    //driveUntilBump();
  }
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

    // print new counter values
    Serial.print("line:       ");
    if (lineSensorValueLeft > threshold)
    {
       Serial.print("LEFT: ");
    }
    else
    {
       Serial.print("left: ");
    }
    Serial.print(lineSensorValueLeft, DEC);
    if (lineSensorValueCenter > threshold)
    {
      Serial.print("\tCENTER: ");
    }
    else
    {
      Serial.print("\tcenter: ");
    }
    Serial.print(lineSensorValueCenter, DEC);
    if (lineSensorValueRight > threshold)
    {
      Serial.print("\tRIGHT: ");
    }
    else
    {
      Serial.print("\tright: ");
    }
    Serial.print(lineSensorValueRight, DEC);
    Serial.println("");
  }

   delay(100);           // give sensors a chance to change
}

// Function:    testBump
// Description: light up LEDs that correspond to a bump sensor
// Inputs:      none
// Returns:     none
void testBump()
{
  // MODIFY THIS CODE. When any of the left bump switches are pressed the RED led
  // should light up - then turn off on release. When any of the right bump switches
  // are pressed the GREEN led should light up - then turn off on release. If bump
  // switches from both sides are pushed both the GREEN and RED led should light up.

  if (bump_sw[0].read() == 0)
  {
    digitalWrite(RED_LED_PIN, HIGH);
    Serial.println("Switch 0 pressed");
  } 
  else if (bump_sw[1].read() == 0)
  {
    digitalWrite(RED_LED_PIN, HIGH);
    Serial.println("Switch 1 pressed");
  }
  else if (bump_sw[2].read() == 0)
  {
    digitalWrite(RED_LED_PIN, HIGH);
    Serial.println("Switch 2 pressed");
  }
  else if (bump_sw[3].read() == 0)
  {
    digitalWrite(RED_LED_PIN, HIGH);
    Serial.println("Switch 3 pressed");
  }
  else if (bump_sw[4].read() == 0)
  {
    digitalWrite(RED_LED_PIN, HIGH);
    Serial.println("Switch 4 pressed");
  }
  else if (bump_sw[5].read() == 0)
  {
    digitalWrite(RED_LED_PIN, HIGH);
    Serial.println("Switch 5 pressed");
  }
  else
  {
    digitalWrite(RED_LED_PIN, LOW);
  }
}


// Function:    readLineSensors
// Description: update the lineSensorValues variable
// Inputs:      none
// Returns:     none
void readLineSensors()
{
  qtrc.read(lineSensorValues);
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
    digitalWrite(RED_LED_PIN, LOW);  // turn LED off
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
    digitalWrite(RED_LED_PIN, HIGH); // turn LED on
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
  digitalWrite(RED_LED_PIN, LOW);

  // give user a chance to remove their finger
  wait(1000);

  Serial.println(""); // print blank line
}

// Function:    driveUntilBump
// Description: drive robot forward until it detects a bump
// Inputs:      none
// Returns:     none
void driveUntilBump()
{
  // EDIT THE FOLLOWING PSEUDO CODE
  // hint: look at how testBump() works
  //
  // while (no bump detected)
  // {
  //    drive forward
  // }
  // stop driving
}



// Function:    stopMotors
// Description: stops the wheel motors
// Inputs:      none
// Returns:     none
void stopMotors()
{
  left_motor.setSpeed(0);
  right_motor.setSpeed(0);

  Serial.println("stopped");
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


// Function:    rev
// Description: set motor speed so robot drives backwards
// Inputs:      speed - the speed the robot will move, 1=slow, 255=fast
// Returns:     none
void rev(int speed)
{
  // validate the speed value
  if (speed < MIN_SPEED) speed = MIN_SPEED;
  if (speed > MAX_SPEED) speed = MAX_SPEED;
  
  // display the function we are in and its values
  Serial.print("backwards:  ");
  Serial.println(speed, DEC);
  
  // disable both motors
  left_motor.disableMotor();
  right_motor.disableMotor();

  // set both motors to forward
  left_motor.directionBackward();
  right_motor.directionBackward();
  
  // set speed for both motors
  left_motor.setSpeed(speed);
  right_motor.setSpeed(speed);

  // enable both motors
  left_motor.enableMotor();
  right_motor.enableMotor();
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

// Function:    clearEncoders
// Description: clears encoder counts (this stops the motors)
// Inputs:      none
// Returns:     none
void clearEncoders()
{
  // clear the encoder counts
  resetLeftEncoderCnt();
  resetRightEncoderCnt();
}

 
