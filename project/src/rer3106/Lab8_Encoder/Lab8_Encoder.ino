// ===========================================================
//
// Lab8_Encoder.ino
// Description: Encoder tester
// Name: Robert Reed, Brendan Deller
// Date: October 24, 2019
// Class: CMPE-110
// Section: <Lab: 5, Thursday, 5:00PM>
// ===========================================================

#include <Romi_Motor_Power.h>
#include <Robot_Pins_v1.h>
#include <Encoder.h>

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

  setupEncoder(ENCODER_ELB_PIN, ENCODER_ELA_PIN, ENCODER_ERB_PIN, ENCODER_ERA_PIN);

  clearEncoders();
}

void loop()
{
  // uncomment the following line to print the wheel encoder values
  testEncoder();
}

    
// Function:    testEncoder
// Description: prints the readings from the wheel encoders, if any have changed.
// Inputs:      none
// Returns:     none
void testEncoder() 
{
  long    lCount = 0;     // left motor encoder counts
  long    rCount = 0;     // right motor encoder counts
  long    prevlCount = 0;
  long    prevrCount = 0;

  lCount = getEncoderLeftCnt();
  rCount = getEncoderRightCnt();

  // see if anything changed
  if ((lCount != prevlCount) || (rCount != prevrCount))
  {
    // remember new counters
    prevlCount = lCount;
    prevrCount = rCount;

    // print new counter values
    Serial.print("counter:    left: ");
    Serial.print(lCount, DEC);
    Serial.print("\tright: ");
    Serial.print(rCount, DEC);
    Serial.println(" counts");
  }

  delay(100);           // get counters a chance to change
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
