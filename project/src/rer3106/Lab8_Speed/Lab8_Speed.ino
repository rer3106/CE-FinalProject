// ===========================================================
//
// Lab8_Speed.ino  
// Description: Robot speed
// Name: <team member names here>
// Date: <today's date here>
// Class: CMPE-110
// Section: <Lab: section, day, and time here>
// ===========================================================

#include <Romi_Motor_Power.h>
#include <Robot_Pins_v1.h>

// 'Start' button
#define BUTTON_PIN PUSH2

// LED pins
#define LED_PIN RED_LED

// Motor
#define MAX_SPEED 100           // Fastest speed
#define MIN_SPEED 1             // Slowest speed

// Define the motors
Romi_Motor_Power left_motor;
Romi_Motor_Power right_motor;


void setup() 
{ 
  Serial.begin(19200); //for serial IO to screen
  Serial.println("Welcome to my motor tester!");

  pinMode(BUTTON_PIN, INPUT_PULLUP); // setup 'start' button so it is LOW when pressed
  pinMode(LED_PIN, OUTPUT); // setup user LED to be an output

  left_motor.begin(MOTOR_L_SLP_PIN, MOTOR_L_DIR_PIN, MOTOR_L_PWM_PIN);
  right_motor.begin(MOTOR_R_SLP_PIN, MOTOR_R_DIR_PIN, MOTOR_R_PWM_PIN);
} 

// ********* Change the code below to program your robot! *********

void loop() 
{
  // uncomment to test normal speed
  speedNormal();

  // uncomment to test slow speed
  //speedSlow();
}



// Function:    speedSlow
// Description: try different slow speeds
// Inputs:      none
// Returns:     none
// 
void speedSlow()
{
  int i;               // loop control variable 

  // start at speed 10 and go by 5, up to 35
  for (i = 10; i <= 35; i += 5)
  {
    pressToStart();    // wait for the 'start' button to be pressed

    fwd(i);            // go forward at speed i
    wait(2000);        // attempt to drive for 2 sec
    stopMotors();      // stop driving
  }

  stopMotors();        // stop the motors
}



// Function:    speedNormal
// Description: try different normal sppeds
// Inputs:      none
// Returns:     none
// 
void speedNormal()
{
  int i;               // loop control variable 

  // start at speed 30 and go by 10, up to 80
  for (i = 30; i <= 80; i += 10)
  {
    pressToStart();    // wait for the 'start' button to be pressed

    fwd(i);            // go forward at speed i
    wait(2000);        // attempt to drive for 2 sec
    stopMotors();      // stop driving
  }

  stopMotors();        // stop the motors
}



// ********* Don't change any code below here. (But feel free to look and ask questions!) *********

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
  left_motor.setSpeed(speed-5);
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
