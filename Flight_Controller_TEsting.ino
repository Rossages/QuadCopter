/* 
   
   Date: 15 November 2017
   Ross Oliver

   
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

  
 * 
 *           Channel3                          Channel 2
 *            |                                    |
 *            |                                    |
 *      _____________   Channel 4            _____________ Channel 1
 *            |                                    |
 *            |                                    |
 *            |                                    |
 *            


*/


// ====== Declaring Variables ==========

// === Variables for the Transmitter and Receiver ===
// As to work out the time period that the Duty cycle is high for, for each channel. --> Either HIGH | LOW
byte prev_chan_1, prev_chan_2, prev_chan_3, prev_chan_4, prev_chan_5, prev_chan_6 ;

// To store the actual value for the duty cycle --> to be used as a value to determine how much power each motor needs
int receiver_channel_1, receiver_channel_2, receiver_channel_3, receiver_channel_4, receiver_channel_5, receiver_channel_6;

// === Creating the Pulse for the ESC's ===

// Stores the micro time for when the Pulse on each channel starts.
uint32_t timer_1, timer_2, timer_3, timer_4, timer_5, timer_6;

// Stores the Trasmitter value to keep everything contant
uint32_t esc_1, esc_2, esc_3, esc_4;

// === Flight controller variables ===
// Corrected value for the ESC's --> ATM only translate the actions of the controller to actual flight for each motor.
int cor_esc_1, cor_esc_2, cor_esc_3, cor_esc_4;

// ====== Scalars / OffSets for Receiver =======
int pos_dir = 1480; // Positive Direction
int neg_dir = 1520; // Negative direction
int loop_timer;   // to get an accurate pulse length we need how how long it takes 
int esc_loop_timer; // to make the pulse, because the time taken for the loop to run may changed slightly.

// ====== esc pin definitions ======
// ie. Motors
// esc1 --> Mo_1 --> Pin7
// esc2 --> Mo_2 --> Pin6
// esc3 --> Mo_3 --> Pin5
// esc4 --> Mo_4 --> Pin4
// The pulse width value directly to the ESC's
int Mo_1, Mo_2, Mo_3, Mo_4;

bool killed = 0; // To tell if the Motors have been killed. Re-init
int Rearm; // Check to see if the motors can actually be reamred. - will be smushed between disable and enable intrupts.
bool init_controls = false;
int throttle_limit = 1600; // 60%

// ====== PROTOTYPES ======
void pulse_width();

// ================== SETUP ==================
void setup() {
  // Setting up the for each motor, and inputs from receiver.

  // MOTOR Setup as OUTPUTS :)
  DDRD |= B11110000;    //Configure digital port 4, 5, 6 and 7 as output.

  // ====== Arming of ESC's ========
  // ** Connect the battery to ESC. then make sure that when you turn on the controller
  // that the the sticks are ! NOT ! above the midway points,--> Puts into throttle calibration Mode.
  // ** NOR in the bottom left of the transmitter --> puts it into the factory settings mode
  // NOW; each motor should beep once after the transmitter has been turned on. --> The throttle signal has been detected
  // When both (esc and transmitter) are turned on, put the sticks on the zero position and should 
  // beep once per motor to tell you that wach esc has been properlly armed.
  // ====== ARMING Complete ======

  // =================== Arming the ESC's ==================

  // Sets up serial rate
  Serial.begin(9600);
  
    esc_1 = esc_2 = esc_3= esc_4 = 1000; // Now setting all of the esc into 0% throttle. 
    for (int j = 0; j <= 3200; j++) { 
      pulse_width(); // 4 second pulse of 4000us pulses -> i think. This should start the initiation sequence
    }

    esc_1 = esc_2 = esc_3 = esc_4 = 1050;
    for (int i = 0; i <= 800; i++) { 
      pulse_width();              //This should be plenty of time for the esc's to register the signal
    }

    esc_1 = esc_2 = esc_4 = esc_3 = 1000; // Now setting all of the esc into 0% throttle for all chanells but throttle.   
    for (int j = 0; j <= 3200; j++) { // This should complete the Arming of the ESC's, now that the throttle is back to zero. 
      pulse_width(); // 4 second pulse of 4000us pulses -> i think. This will hopfully be enough time for the esc's to not loose signal of the "transmitter"
    }
        
// to arm the Quad. Start SwA in UP pos. Then down. Then up to allow control to controller and reveceier.


  // ====== Receiver Channels SET Up ======
  //Arduino (Atmega) pins default to inputs, so they don't need to be explicitly declared as inputs
  // This is from the data sheet on the
   PCICR |= (1 << PCIE0);    // set PCIE0 to enable PCMSK0 scan
   PCMSK0 |= (1 << PCINT0);  // set PCINT0 (digital input 8) to trigger an interrupt on state change --> Channel 1
   PCMSK0 |= (1 << PCINT1);  // set PCINT1 (digital input 9)to trigger an interrupt on state change --> Channel 2
   PCMSK0 |= (1 << PCINT2);  // set PCINT2 (digital input 10)to trigger an interrupt on state change --> Channel 3
   PCMSK0 |= (1 << PCINT3);  // set PCINT3 (digital input 11)to trigger an interrupt on state change --> Channel 4
   // ADDED these for future use for Channel 5 and 6 --> Can use these for landing sequence or something of a kind.
   // ***** Refer to Note above.
   PCMSK0 |= (1 << PCINT4); // set PCINT4 (digital input 12) --> for Channel 5 for SwA --> use for Arming the QuadCopter?
   PCMSK0 |= (1 << PCINT5); // set PCINT5 (digital input 13) --> for Channel 6 for Sw6 (Three Stage Switch)

  
  /* TO DO: *Kill Motors with SWA in Down position
   * *increase Motors speed when you wana fly in a direction.
   *  * What happens when the motors are full tit and we want to turn?
   *    PID controller?
   *  * Three way switch ideas: 
   *    Self leveling On/Off
   *    Battery Voltage display??
   *    Warm up Motors? - currently 1030  THis will probably need to be much higher 
  */
  
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
    receiver_channel_1 = micros() - timer_1;      //Channel 1 is micros() - timer_1
  }

  //Channel 2 ======================
  if (prev_chan_2 == 0 && PINB & B00000010 ) {       //Input 9 changed from 0 to 1 (LOW to HIGH)
    prev_chan_2 = 1;                                 //Remember current input state
    timer_2 = micros();                                 //Set timer_2 to micros()
  }
  else if (prev_chan_2 == 1 && !(PINB & B00000010)) { //Input 9 changed from 1 to 0 (HIGH to LOW)
    prev_chan_2 = 0;                                 //Remember current input state
    receiver_channel_2 = micros() - timer_2;      //Channel 2 is micros() - timer_2
  }

  //Channel 3 ======================
  if (prev_chan_3 == 0 && PINB & B00000100 ) {       //Input 10 changed from 0 to 1 (LOW to HIGH)
    prev_chan_3 = 1;                                 //Remember current input state
    timer_3 = micros();                                 //Set timer_3 to micros()
  }
  else if (prev_chan_3 == 1 && !(PINB & B00000100)) { //Input 10 changed from 1 to 0 (HIGH to LOW)
    prev_chan_3 = 0;                                 //Remember current input state
    receiver_channel_3 = micros() - timer_3;      //Channel 3 is micros() - timer_3
  }

  //Channel 4 ======================
  if (prev_chan_4 == 0 && PINB & B00001000 ) {       //Input 11 changed from 0 to 1 (LOW to HIGH)
    prev_chan_4 = 1;                                 //Remember current input state
    timer_4 = micros();                                 //Set timer_4 to micros()
  }
  else if (prev_chan_4 == 1 && !(PINB & B00001000)) { //Input 11 changed from 1 to 0 (HIGH to LOW)
    prev_chan_4 = 0;                                 //Remember current input state
    receiver_channel_4 = micros() - timer_4;      //Channel 4 is micros() - timer_4
  }

  //Channel 5 ( SWA ) =========================================================================== added for Switch's
  if (prev_chan_5 == 0 && PINB & B00010000 ) {       //Input 12 changed from 0 to 1 (LOW to HIGH)
    prev_chan_5 = 1;                                 //Remember current input state
    timer_5 = micros();                                 //Set timer_5 to micros()
  }
  else if (prev_chan_5 == 1 && !(PINB & B00010000)) { //Input 12 changed from 1 to 0 (HIGH to LOW)
    prev_chan_5 = 0;                                 //Remember current input state
    receiver_channel_5 = micros() - timer_5;      //Channel 5 is micros() - timer_5
  }

  // Channel 6 ( SWC ) ======================
  if (prev_chan_6 == 0 && PINB & B00100000) { // Pretty sure Channel 5 and 6 Bit masks are right...
    prev_chan_6 = 1;
    timer_6 = micros();
  }
  else if (prev_chan_6 == 1 && !(PINB & B00100000)) {
    prev_chan_6 = 0;
    receiver_channel_6 = micros() - timer_6;
  }

  
  // assinging the input from receiver Code for each motor as to keep variable names consistent
  // for pulsewidth() code.
  esc_1 = receiver_channel_1;
  esc_2 = receiver_channel_2;
  esc_3 = receiver_channel_3;
  esc_4 = receiver_channel_4; 
}

// ========== PULSE WIDTH ============
void pulse_width(){
  // This produces the pulse that is sent to it. Ideally 1000us to 2000us and sends it to the ESC's

  
  loop_timer = micros();            //Set the timer for the next loop.

  PORTD |= B11110000;                //Set digital outputs 4,5,6 and 7 high.
  Mo_1 = esc_1 + loop_timer;               //Calculate the time of the faling edge of the esc-1 pulse.
  Mo_2 = esc_2 + loop_timer;               //Calculate the time of the faling edge of the esc-2 pulse.
  Mo_3 = esc_3 + loop_timer;               //Calculate the time of the faling edge of the esc-3 pulse.
  Mo_4 = esc_4 + loop_timer;               //Calculate the time of the faling edge of the esc-4 pulse.

  while (PORTD >= 16) {                    //Stay in this loop until output 4,5,6 and 7 are low.
    esc_loop_timer = micros();                 //Read the current time.
    if(Mo_1 <= esc_loop_timer)PORTD &= B11101111;    //Set digital output 4 to low if the time is expired.
    if(Mo_2 <= esc_loop_timer)PORTD &= B11011111;    //Set digital output 5 to low if the time is expired.
    if(Mo_3 <= esc_loop_timer)PORTD &= B10111111;    //Set digital output 6 to low if the time is expired.
    if(Mo_4 <= esc_loop_timer)PORTD &= B01111111;    //Set digital output 7 to low if the time is expired.
  }
}

// ========== FLIGHT CONTROLLER ==========
void flight_controller() {
  // will Control what the signal from the receiver is and 
  //convert and forward it to the esc's
  

  // Throttle is channel_3, as it effects all motors.
  esc_1 = receiver_channel_3; 
  esc_2 = receiver_channel_3;
  esc_3 = receiver_channel_3;
  esc_4 = receiver_channel_3;
  
  // === PITCH ===
  cor_esc_2 = receiver_channel_2 - 1500; // centre point of channel 2; + forward, - backwards
  
  if (cor_esc_2 > 5) { // The stick is pushed forward, ch2 value will increase pushing more power to the motors 1 and 3
      // 5 is to allow the motors a margin of error. The transmitter fluctuates +-4us so margin of error +-5us
      // This makes the idle/motor warm up more stable.
    esc_2 += cor_esc_2;
    esc_4 += cor_esc_2;
  
    if (esc_1 || esc_3 >= throttle_limit) { //if they greater then the limit of 80% power, decrease opposite side
      esc_1 -= cor_esc_2; 
      esc_3 -= cor_esc_2;
    }
    
  /* if the total throttle is less then 50% or 1500us then all esc values will be less then 0.  
      If they are less then zero then the motors need to behave in the inverse characteristics.
      because of this i have choosen the -5us as to add some stability around the 50% throttle mark.*/
    
  } else if (cor_esc_2 < -5) { 
    esc_1 += abs(cor_esc_2);
    esc_3 += abs(cor_esc_2);
  
  if (esc_2 || esc_4 >= throttle_limit) { // Restrict the Motors to 80% power
    esc_2 -= abs(cor_esc_2);
    esc_4 -= abs(cor_esc_2);
    }
  }

  // === ROLL ===
  cor_esc_1 = receiver_channel_1 - 1500; // centre point of channel 1; - left roll, + right roll

  if (cor_esc_1 > 5) { // Right pos --> Roll Right --> more power into motor 3 & 4
    
    esc_3 += cor_esc_1; // adding to motor.
    esc_4 += cor_esc_1; 

    if (esc_3 || esc_4 > throttle_limit) {
      esc_1 -= cor_esc_1; // If cor_esc_>0 then we already adding to it. So if we *2 then
      esc_2 -= cor_esc_1; // this allows me to have less conditions while acheiving the same thing.
    }
    
  } else if (cor_esc_1 < -5) { //left Roll
    esc_1 += abs(cor_esc_1);
    esc_2 += abs(cor_esc_1);
  
  if ( esc_3 || esc_4 > throttle_limit){ // Restrict the Motors to 80% power
    esc_3 -= abs(cor_esc_1);
    esc_4 -= abs(cor_esc_1);
    }
  }

  // === YAW ===
  cor_esc_4 = receiver_channel_4 - 1500; // centre point of channel 4; - CCW YAW, + CW YAW 

  if (cor_esc_4 > 5) { // CW direction
    esc_2 += cor_esc_4; //*** CHECK THIS DIRECTIONS CANT REMEMBER WHAT WAY MOTORS ROTATE - ATM assuming motors 2 and 3 rotate CW
    esc_3 += cor_esc_4; //*** CHECK THIS DIRECTIONS CANT REMEMBER WHAT WAY MOTORS ROTATE

    
    if ( esc_2 || esc_2 > throttle_limit){
      esc_1 -= cor_esc_4;
      esc_4 -= cor_esc_4;
    }
  
  } else if (cor_esc_4 < -5) {
    esc_1 += abs(cor_esc_4); //*** CHECK THESE DIRECTIONS CANT REMEMBER WHAT WAY MOTORS ROTATE - ATM assuming motors 2 and 3 rotate CCW
    esc_4 += abs(cor_esc_4); //*** CHECK THESE DIRECTIONS CANT REMEMBER WHAT WAY MOTORS ROTATE

    
    if ( esc_1 || esc_4 > throttle_limit){
      esc_2 -= abs(cor_esc_4);
      esc_3 -= abs(cor_esc_4);
    }
  }

  /* if esc_# is over 1800 then rather then + to the esc value, we should -.
    LIMIT the Motors to 80% of their speed capacity. So I dont wear them out too quickly.
    When throttle is at zero all motors staying warm/on.
  */
  
  if (esc_1 > throttle_limit)esc_1 = throttle_limit;
  if (esc_2 > throttle_limit)esc_2 = throttle_limit;
  if (esc_3 > throttle_limit)esc_3 = throttle_limit;
  if (esc_4 > throttle_limit)esc_4 = throttle_limit;

  
  if (esc_1 < 1030)esc_1 = 1030; // LOW RPM can cause problems with the motors.
  if (esc_2 < 1030)esc_2 = 1030;  // Motors will shutter at really low rpm. Looks and sounds bad.
  if (esc_3 < 1030)esc_3 = 1030;
  if (esc_4 < 1030)esc_4 = 1030;
  
  pulse_width();
}

// ============ MAIN LOOP ============
void loop()
{
  
  //print_signals(); // Prints each Channels pulse length value to the serial
  //pulse_width(); // produces a pulse for each esc with a value range of 1000 --> 2000 
  flight_controller(); // Control each motor 
 

  // === Characteristics of SWA ( two way switch ) ===

  if (receiver_channel_5 > 1800) { // Down positon --> ****** Kill Motors 0% ******
    
    PCICR &= B00001111;                                                         // Disables inturrupts on receiver channels - Pins 8, 9, 10,11
    killed = 1;
    while (receiver_channel_5 > 1800) {                                         // The 980us is because th delay function takes time to excecute. so the 980us == 1000us theoretically.
        // This will kleep talking to the esc - should be enough to keep them turned on
        PORTD |= B11110000;                                                     //Set digital poort 4, 5, 6 and 7 high.
        delayMicroseconds(980);                                                //Wait 980us. -> should be enough to keep the esc's tunred on, But keep motors off.
        PORTD &= B00001111;                                                     //Set digital poort 4, 5, 6 and 7 low.
        delay(3);                                                                //Wait 3ms before the next loop.
    }
    
    // This is ensure that the throttle is back down to zero when motors are re-armed. SO THAT I CANT BE A NOOB AGAIN.
    // ***** CHECK THIS WORKS ***** 
    // Note: Hopefully this doesnt turn the motors. even if the motors twitch it should be ok! 
    
    PCICR |= B00100000;         // Enables inturrupts on receiver channel 3 - Pins 10 - throttle channel.
    Rearm = receiver_channel_3; // the throttle value. -> shouldnt make thme esc's more as the check for the channel should be so short.
    PCICR &= B00001111;         // Disables inturrupts on receiver channels - Pins 8, 9, 10, 11

    //if (receiver_channel_5 < 1000 && receiever_channel < 1100) { // SWA is now in the down position. && throttle is less then 10%
    PCICR |= B11110000;         // Enables inturrupts on receiver channels - Pins 8, 9, 10,11
    killed = 0;
    //}
  }

}




