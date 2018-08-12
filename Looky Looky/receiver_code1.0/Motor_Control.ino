/*
 * Date: 10 July 2017
 * Ross Oliver
 * 
 * Individual motor control --> NOT the flight controller module.
 * This is for testing purposes only.
 * 
 * used in conjunction with reveiver_code1.0.
 * 
 * Motor 1/esc 1/s1 --> A0
 * Motor 2/esc 2/s2 --> A1
 * Motor 3/esc 3/s3 --> A2
 * Motor 4/esc 4/s4 --> A3
 */
// ### #Defines #########

#include "receiver_code1.0.c"


// esc pin definitions
// ie. Motors

int motor_1 = A0;
int motor_2 = A1;
int motor_3 = A2;
int motor_4 = A4;

int loop_timer;
int esc_loop_timer; // for the esc time period

void setup()
{
  // Setting up the for each motor, and inputs from receiver code.

  pinMode(motor_1, OUTPUT); // PINs A0, A1, A2, A3 for motors
  pinMode(motor_2, OUTPUT);
  pinMode(motor_3, OUTPUT);
  pinMode(motor_4, OUTPUT);
  
}

void loop()
{

  // assinging the input from receiver Code for each motor as to keep variable names consistent.
  esc_1 = receiver_input_channel_1;
  esc_2 = receiver_input_channel_2;
  esc_3 = receiver_input_channel_3;
  esc_4 = receiver_input_channel_4;

  //All the information for controlling the motor's is available.
  //The refresh rate is 250Hz. That means the esc's need there pulse every 4ms.
  while(micros() - loop_timer < 4000);                                      //We wait until 4000us are passed.
  loop_timer = micros();                                                    //Set the timer for the next loop.

  PORTD |= B11110000;                                                       //Set digital outputs 4,5,6 and 7 high.
  motor_1 = esc_1 + loop_timer;                                     //Calculate the time of the faling edge of the esc-1 pulse.
  motor_2 = esc_2 + loop_timer;                                     //Calculate the time of the faling edge of the esc-2 pulse.
  motor_3 = esc_3 + loop_timer;                                     //Calculate the time of the faling edge of the esc-3 pulse.
  motor_4 = esc_4 + loop_timer;                                     //Calculate the time of the faling edge of the esc-4 pulse.
 
  while(PORTD >= 16){                                                       //Stay in this loop until output 4,5,6 and 7 are low.
    esc_loop_timer = micros();                                              //Read the current time.
    if(motor_1 <= esc_loop_timer)PORTD &= B11101111;                //Set digital output 4 to low if the time is expired.
    if(motor_2 <= esc_loop_timer)PORTD &= B11011111;                //Set digital output 5 to low if the time is expired.
    if(motor_3 <= esc_loop_timer)PORTD &= B10111111;                //Set digital output 6 to low if the time is expired.
    if(motor_4 <= esc_loop_timer)PORTD &= B01111111;                //Set digital output 7 to low if the time is expired.
  }

}


 
