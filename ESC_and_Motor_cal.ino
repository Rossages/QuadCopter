/*
    THIS CAN PROBABLY BE USED TO CALIBRATE ANY ESC'S THROTTLE 
   
   Date: 10 July 2017
   Ross Oliver

   Individual motor control --> NOT the flight controller module.
   This is for testing purposes only.

   used in conjunction with reveiver_code1.0.

   Motor 1/esc 1/s1 --> 4
   Motor 2/esc 2/s2 --> 5
   Motor 3/esc 3/s3 --> 6
   Motor 4/esc 4/s4 --> 7

  ====== Arming of ESC's ========
  NOTE ** Connect the battery to ESC. then make sure that when you turn on the controller
  that the the sticks are ! NOT ! above the midway points,--> Puts into throttle calibration Mode.
  
  NOTE ** NOR in the bottom left of the transmitter --> puts it into the factory settings mode
  
  NOW; each motor should beep once after the transmitter has been turned on. --> The throttle signal has been detected
  When both (esc and transmitter) are turned on, put the sticks on the zero position and should 
  beep once per motor to tell you that wach esc has been properlly armed.
  // ====== ARMING Complete ======


*/


// ====== Declaring Variables ==========

// As to work out the time period that the Duty cycle is high for, for each channel. --> Either HIGH | LOW
byte prev_chan_1, prev_chan_2, prev_chan_3, prev_chan_4, prev_chan_5, prev_chan_6 ;

// To store the actual value for the duty cycle --> to be used as a value to determine how much power each motor needs
int receiver_input_channel_1, receiver_input_channel_2, receiver_input_channel_3, receiver_input_channel_4, receiver_input_channel_5, receiver_input_channel_6;

// Stores the micro time for when the Pulse on each channel starts.
uint32_t timer_1, timer_2, timer_3, timer_4, timer_5, timer_6;

// Stores the Trasmitter value to keep everything contant
uint32_t esc_1, esc_2, esc_3, esc_4;

// ====== Scalars / OffSets =======
int pos_dir = 1480; // Positive Direction

int neg_dir = 1520; // Negative direction

// ====== esc pin definitions ======
// ie. Motors
// esc1 --> Motor1 --> Pin7
// esc2 --> Motor2 --> Pin6
// esc3 --> Motor3 --> Pin5
// esc4 --> Motor4 --> Pin4
int motor_1, motor_2, motor_3, motor_4;

// to get an accurate pulse length needs to time how long it takes to make the pulse, 
// as to counteract it
int loop_timer;
int esc_loop_timer; // for the esc time period

// ====== PROTOTYPES ======
void pulse_width();

// ============ SETUP ============
void setup() {
  // Setting up the for each motor, and inputs from receiver code.

  // FOR MOTOR Setup as outputs :)
  DDRD |= B11110000;    //Configure digital poort 4, 5, 6 and 7 as output.

  // ====== Arming of ESC's ========
  // ** Connect the battery to ESC. then make sure that when you turn on the controller
  // that the the sticks are ! NOT ! above the midway points,--> Puts into throttle calibration Mode.
  // ** NOR in the bottom left of the transmitter --> puts it into the factory settings mode
  // NOW; each motor should beep once after the transmitter has been turned on. --> The throttle signal has been detected
  // When both (esc and transmitter) are turned on, put the sticks on the zero position and should 
  // beep once per motor to tell you that wach esc has been properlly armed.
  // ====== ARMING Complete ======

 
  //Arduino (Atmega) pins default to inputs, so they don't need to be explicitly declared as inputs
  // This is from the data sheet on the

  // ====== Receiver Channels SET Up ======
  PCICR |= (1 << PCIE0);    // set PCIE0 to enable PCMSK0 scan
  PCMSK0 |= (1 << PCINT0);  // set PCINT0 (digital input 8) to trigger an interrupt on state change --> Channel 1
  PCMSK0 |= (1 << PCINT1);  // set PCINT1 (digital input 9)to trigger an interrupt on state change --> Channel 2
  PCMSK0 |= (1 << PCINT2);  // set PCINT2 (digital input 10)to trigger an interrupt on state change --> Channel 3
  PCMSK0 |= (1 << PCINT3);  // set PCINT3 (digital input 11)to trigger an interrupt on state change --> Channel 4

  // ADDED these for future use for Channel 5 and 6 --> Can use these for landing sequence or something of a kind.
  // ***** Refer to Note above.
  PCMSK0 |= (1 << PCINT4); // set PCINT4 (digital input 12) --> for Channel 5 for SwA --> use for Arming the QuadCopter?
  PCMSK0 |= (1 << PCINT5); // set PCINT5 (digital input 13) --> for Channel 6 for Sw6 (Three Stage Switch)

  
  // Sets up serial rate
  Serial.begin(9600);
}


//Subroutine for displaying the receiver signals
void print_signals() {
  
  Serial.print("Ch_1:");
  if (receiver_input_channel_1 - pos_dir < 0)Serial.print("<<<"); // shows direction of the sticks on transmitter
  else if (receiver_input_channel_1 - neg_dir > 0)Serial.print(">>>");
  else Serial.print("-+-");
  Serial.print(receiver_input_channel_1);

  Serial.print("  Ch_2:");
  if (receiver_input_channel_2 - pos_dir < 0)Serial.print("vvv");
  else if (receiver_input_channel_2 - neg_dir > 0)Serial.print("^^^");
  else Serial.print("-+-");
  Serial.print(receiver_input_channel_2);

  Serial.print("  Ch_3:");
  if (receiver_input_channel_3 - pos_dir < 0)Serial.print("vvv");
  else if (receiver_input_channel_3 - neg_dir > 0)Serial.print("^^^");
  else Serial.print("-+-");
  Serial.print(receiver_input_channel_3);

  Serial.print("  Ch_4:");
  if (receiver_input_channel_4 - pos_dir < 0)Serial.print("<<<");
  else if (receiver_input_channel_4 - neg_dir > 0)Serial.print(">>>");
  else Serial.print("-+-");
  Serial.print(receiver_input_channel_4);

  Serial.print("  Ch_5 (SWA) :");
  if (receiver_input_channel_5 < 1000)Serial.print("^^^"); // Channel 5 (SWA in the up position) returns a value of 944->948.
  else if (receiver_input_channel_5 > 1800)Serial.print("vvv"); // Channel 5 (SWA in the up position) returns a value of 1944->1948
  Serial.print(receiver_input_channel_5);

  Serial.print("  Ch_6 (SWC) :");
  if (receiver_input_channel_6 < 1000)Serial.print("^^^"); // Channel 6 (SWC in the up position) returns a value of 984->988
  else if (1000 < receiver_input_channel_6 < 1600)Serial.print("-+-"); // Channel 6 (SWC in the middle position) returns 1492->1498
  else Serial.print("vvv"); // Channel 6 (SWC in the down position) returns a value of 1984->1988 ========================= this line doesnt work :( but Mehhhh
  Serial.println(receiver_input_channel_6);

}

//This routine is called every time input 8, 9, 10 or 11 changed state
// This measures how long the PPM Wave is on for compared to off. Giving a value
// for the arduino to convert into a real value for throttle for each motor.

// =========== ISR for Channel 1,2,3,4 Pulse MEasurements ====================
ISR(PCINT0_vect) {
  //Channel 1 ======================
  if (prev_chan_1 == 0 && PINB & B00000001 ) {       //Input 8 changed from 0 to 1 (LOW to HIGH)
    prev_chan_1 = 1;                                 //Remember current input state -- HIGH
    timer_1 = micros();                                 //Set timer_1 to micros()
  }
  else if (prev_chan_1 == 1 && !(PINB & B00000001)) { //Input 8 changed from 1 to 0 (HIGH to LOW)
    prev_chan_1 = 0;                                 //Remember current input state -- LOW
    receiver_input_channel_1 = micros() - timer_1;      //Channel 1 is micros() - timer_1
  }

  //Channel 2 ======================
  if (prev_chan_2 == 0 && PINB & B00000010 ) {       //Input 9 changed from 0 to 1 (LOW to HIGH)
    prev_chan_2 = 1;                                 //Remember current input state
    timer_2 = micros();                                 //Set timer_2 to micros()
  }
  else if (prev_chan_2 == 1 && !(PINB & B00000010)) { //Input 9 changed from 1 to 0 (HIGH to LOW)
    prev_chan_2 = 0;                                 //Remember current input state
    receiver_input_channel_2 = micros() - timer_2;      //Channel 2 is micros() - timer_2
  }

  //Channel 3 ======================
  if (prev_chan_3 == 0 && PINB & B00000100 ) {       //Input 10 changed from 0 to 1 (LOW to HIGH)
    prev_chan_3 = 1;                                 //Remember current input state
    timer_3 = micros();                                 //Set timer_3 to micros()
  }
  else if (prev_chan_3 == 1 && !(PINB & B00000100)) { //Input 10 changed from 1 to 0 (HIGH to LOW)
    prev_chan_3 = 0;                                 //Remember current input state
    receiver_input_channel_3 = micros() - timer_3;      //Channel 3 is micros() - timer_3
  }

  //Channel 4 ======================
  if (prev_chan_4 == 0 && PINB & B00001000 ) {       //Input 11 changed from 0 to 1 (LOW to HIGH)
    prev_chan_4 = 1;                                 //Remember current input state
    timer_4 = micros();                                 //Set timer_4 to micros()
  }
  else if (prev_chan_4 == 1 && !(PINB & B00001000)) { //Input 11 changed from 1 to 0 (HIGH to LOW)
    prev_chan_4 = 0;                                 //Remember current input state
    receiver_input_channel_4 = micros() - timer_4;      //Channel 4 is micros() - timer_4
  }

  //Channel 5 ( SWA ) =========================================================================== added for Switch's
  if (prev_chan_5 == 0 && PINB & B00010000 ) {       //Input 12 changed from 0 to 1 (LOW to HIGH)
    prev_chan_5 = 1;                                 //Remember current input state
    timer_5 = micros();                                 //Set timer_5 to micros()
  }
  else if (prev_chan_5 == 1 && !(PINB & B00010000)) { //Input 12 changed from 1 to 0 (HIGH to LOW)
    prev_chan_5 = 0;                                 //Remember current input state
    receiver_input_channel_5 = micros() - timer_5;      //Channel 5 is micros() - timer_5
  }

  // Channel 6 ( SWC ) ======================
  if (prev_chan_6 == 0 && PINB & B00100000) { //Input 13 changed from 0 to 1 (LOW to HIGH)
    prev_chan_6 = 1;                          //Remember current input state
    timer_6 = micros();                       //Set timer_6 to micros()
  }
  else if (prev_chan_6 == 1 && !(PINB & B00100000)) { //Input 13 changed from 1 to 0 (HIGH to LOW)
    prev_chan_6 = 0;                                    //Remember current input state
    receiver_input_channel_6 = micros() - timer_6;      //Channel 6 is micros() - timer_6
  }

  // The limits of Channel 3 and 4 dont work properlly is limited to 1012 to 1990 ish....... :(
  // We need the 1000 and 2000 limits for the throttle calibration for the motors. 
  // NOTE: Can potentially get rid of this Once calibrated.
  if (receiver_input_channel_3 > 1990)receiver_input_channel_3 = 2000; 
  if (receiver_input_channel_4 > 1990)receiver_input_channel_4 = 2000;

  if (receiver_input_channel_3 <1020)receiver_input_channel_3 = 1000;
  if (receiver_input_channel_4 < 1020)receiver_input_channel_4 = 1000;
  
  // assinging the input from receiver Code for each motor as to keep variable names consistent
  // for pulsewidth() code.
  esc_1 = receiver_input_channel_1;
  esc_2 = receiver_input_channel_2;
  esc_3 = receiver_input_channel_3;
  esc_4 = receiver_input_channel_4; 
}

// ========== PULSE WIDTH ============
void pulse_width(){
  // This produces the pulse that is sent to it. Ideally 1000us to 2000us and sends it to the ESC's
  
  loop_timer = micros();            //Set the timer for the next loop.

  PORTD |= B11110000;                //Set digital outputs 4,5,6 and 7 high.
  motor_1 = esc_1 + loop_timer;               //Calculate the time of the faling edge of the esc-1 pulse.
  motor_2 = esc_2 + loop_timer;               //Calculate the time of the faling edge of the esc-2 pulse.
  motor_3 = esc_3 + loop_timer;               //Calculate the time of the faling edge of the esc-3 pulse.
  motor_4 = esc_4 + loop_timer;               //Calculate the time of the faling edge of the esc-4 pulse.

  while (PORTD >= 16) {                    //Stay in this loop until output 4,5,6 and 7 are low.
    esc_loop_timer = micros();                 //Read the current time.
    if(motor_1 <= esc_loop_timer)PORTD &= B11101111;    //Set digital output 4 to low if the time is expired.
    if(motor_2 <= esc_loop_timer)PORTD &= B11011111;    //Set digital output 5 to low if the time is expired.
    if(motor_3 <= esc_loop_timer)PORTD &= B10111111;    //Set digital output 6 to low if the time is expired.
    if(motor_4 <= esc_loop_timer)PORTD &= B01111111;    //Set digital output 7 to low if the time is expired.
  }
}


// ============ MAIN LOOP ============
void loop()
{

  print_signals(); // Prints to the serial ---------- Comment out for LATER USE OF CODE
  pulse_width(); // so that ONLY esc4 works.

}



