/*
 * Date Finished 6/7/2017
 * Ross Oliver Quadcopter Receiver Code. --> 4 Channels from FS-i6B receiver
 * paired to a FS-i6 Transmitter.
 * 
 * 
 *           Channel3                          Channel 2
 *            |                                    |
 *            |                                    |
 *      _____________   Channel 4            _____________ Channel 1
 *            |                                    |
 *            |                                    |
 *            |                                    |
 *            
 *            
 * The Above diagram are the associated channels for the receiver.
 * 
 * This Code has been Modified and was Based off; http://www.brokking.net/ymfc-al_main.html
 * 
 * Channel # (Hooked up to ) Pin #
 * ---------------------------
 * Channel 1 --> Pin 8
 * Channel 2 --> Pin 9
 * Channel 3 --> Pin 10
 * Channel 4 --> Pin 11
 * ---------------------------
 * 
 * The receiver periods/values are always in increments of 4us. This is because the function Micros() takes 4 micro seconds to incriment, there for this is the 
 * best resolution that is avalible when measuring the time for the Arduino Uno.
 * 
 * Powered by the 5V/GND output off the arduino as doesnt draw much current.. :)
 * 
 * 
 * ******NOTE: Channel 5 and 6 are currently set up for SWA and SWC respectivly on the transmitter
 * 
 * 
 * 13.2.8 PCMSK0 – Pin Change Mask Register 0 -----> for Bit Masks
 * ATMEL data sheet can be found at : 
 * http://www.atmel.com/images/Atmel-8271-8-bit-AVR-Microcontroller-ATmega48A-48PA-88A-88PA-168A-168PA-328-328P_datasheet_Complete.pdf
 * 
 * 
 */


// ====== Declaring Variables ==========

// As to work out the time period that the Duty cycle is high for, for each channel. --> Either HIGH | LOW
byte prev_chan_1, prev_chan_2, prev_chan_3, prev_chan_4, prev_chan_5, prev_chan_6 ; 

// To store the actual value for the duty cycle --> to be used as a value to determine how much power each motor needs
int receiver_input_channel_1, receiver_input_channel_2, receiver_input_channel_3, receiver_input_channel_4, receiver_input_channel_5, receiver_input_channel_6;

// Stores the micro time for when the Pulse on each channel starts.
uint32_t timer_1, timer_2, timer_3, timer_4, timer_5, timer_6;

// ====== Scalars / OffSets =======
int pos_dir = 1480; // Positive Direction

int neg_dir = 1520; // Negative direction

// ======== Setup =================
void setup(){
  
  //Arduino (Atmega) pins default to inputs, so they don't need to be explicitly declared as inputs
  // This is from the data sheet on the 
  
  PCICR |= (1 << PCIE0);    // set PCIE0 to enable PCMSK0 scan 
  PCMSK0 |= (1 << PCINT0);  // set PCINT0 (digital input 8) to trigger an interrupt on state change --> Channel 1
  PCMSK0 |= (1 << PCINT1);  // set PCINT1 (digital input 9)to trigger an interrupt on state change --> Channel 2
  PCMSK0 |= (1 << PCINT2);  // set PCINT2 (digital input 10)to trigger an interrupt on state change --> Channel 3
  PCMSK0 |= (1 << PCINT3);  // set PCINT3 (digital input 11)to trigger an interrupt on state change --> Channel 4

  // ADDED these for future use for Channel 5 and 6 --> Can use these for landing sequence or something of a kind.
  // ***** Refer to Note above.
  PCMSK0 |= (1 << PCINT4); // set PCINT4 (digital input 12) --> for Channel 5 for SwA --> use for Arming the QuadCopter
  PCMSK0 |= (1 << PCINT5); // set PCINT5 (digital input 13) --> for Channel 6 for Sw6 (Three Stage Switch)
  Serial.begin(9600); 
}

// ========= Main program loop =============
void loop(){
  delay(250); // JUST TO MAKE THE SERIAL WINDOW CHILL OUT
  print_signals(); // Prints to the serial ---------- Comment out for LATER USE OF CODE
}

//This routine is called every time input 8, 9, 10 or 11 changed state
// This measures how long the PPM Wave is on for compared to off. Giving a value 
// for the arduino to convert into a real value for throttle for each motor.

// =========== ISR for Channel 1,2,3,4 Pulse MEasurements ====================
ISR(PCINT0_vect){
  //Channel 1 ======================
  if(prev_chan_1 == 0 && PINB & B00000001 ){         //Input 8 changed from 0 to 1 (LOW to HIGH)
    prev_chan_1 = 1;                                 //Remember current input state -- HIGH
    timer_1 = micros();                                 //Set timer_1 to micros()
  }
  else if(prev_chan_1 == 1 && !(PINB & B00000001)){  //Input 8 changed from 1 to 0 (HIGH to LOW)
    prev_chan_1 = 0;                                 //Remember current input state -- LOW
    receiver_input_channel_1 = micros() - timer_1;      //Channel 1 is micros() - timer_1
  }
  
  //Channel 2 ======================
  if(prev_chan_2 == 0 && PINB & B00000010 ){         //Input 9 changed from 0 to 1 (LOW to HIGH)
    prev_chan_2 = 1;                                 //Remember current input state
    timer_2 = micros();                                 //Set timer_2 to micros()
  }
  else if(prev_chan_2 == 1 && !(PINB & B00000010)){  //Input 9 changed from 1 to 0 (HIGH to LOW)
    prev_chan_2 = 0;                                 //Remember current input state
    receiver_input_channel_2 = micros() - timer_2;      //Channel 2 is micros() - timer_2
  }
  
  //Channel 3 ======================
  if(prev_chan_3 == 0 && PINB & B00000100 ){         //Input 10 changed from 0 to 1 (LOW to HIGH)
    prev_chan_3 = 1;                                 //Remember current input state
    timer_3 = micros();                                 //Set timer_3 to micros()
  }
  else if(prev_chan_3 == 1 && !(PINB & B00000100)){  //Input 10 changed from 1 to 0 (HIGH to LOW)
    prev_chan_3 = 0;                                 //Remember current input state
    receiver_input_channel_3 = micros() - timer_3;      //Channel 3 is micros() - timer_3
  }
  
  //Channel 4 ======================
  if(prev_chan_4 == 0 && PINB & B00001000 ){         //Input 11 changed from 0 to 1 (LOW to HIGH)
    prev_chan_4 = 1;                                 //Remember current input state
    timer_4 = micros();                                 //Set timer_4 to micros()
  }
  else if(prev_chan_4 == 1 && !(PINB & B00001000)){  //Input 11 changed from 1 to 0 (HIGH to LOW)
    prev_chan_4 = 0;                                 //Remember current input state
    receiver_input_channel_4 = micros() - timer_4;      //Channel 4 is micros() - timer_4
  }

  //Channel 5 ( SWA ) =========================================================================== added for Switche's
  if(prev_chan_5 == 0 && PINB & B00010000 ){         //Input 12 changed from 0 to 1 (LOW to HIGH)
    prev_chan_5 = 1;                                 //Remember current input state
    timer_5 = micros();                                 //Set timer_5 to micros()
  } 
  else if(prev_chan_5 == 1 && !(PINB & B00010000)){  //Input 12 changed from 1 to 0 (HIGH to LOW)
    prev_chan_5 = 0;                                 //Remember current input state
    receiver_input_channel_5 = micros() - timer_5;      //Channel 5 is micros() - timer_5
  }

  // Channel 6 ( SWC ) ======================
  if (prev_chan_6 == 0 && PINB & B00100000){ // Pretty sure Channel 5 and 6 Bit masks are right... 
    prev_chan_6 = 1;
    timer_6 = micros();
  } 
  else if (prev_chan_6 == 1 && !(PINB & B00100000)){
    prev_chan_6 = 0;
    receiver_input_channel_6 = micros() - timer_6;
  }
  // ===============================================================================================
}



//Subroutine for displaying the receiver signals
void print_signals(){
  
  Serial.print("Ch_1:");
  if(receiver_input_channel_1 - pos_dir < 0)Serial.print("<<<"); // shows direction of the sticks on transmitter
  else if(receiver_input_channel_1 - neg_dir > 0)Serial.print(">>>");
  else Serial.print("-+-");
  Serial.print(receiver_input_channel_1);
  
  Serial.print("  Ch_2:");
  if(receiver_input_channel_2 - pos_dir< 0)Serial.print("vvv"); 
  else if(receiver_input_channel_2 - neg_dir > 0)Serial.print("^^^");
  else Serial.print("-+-");
  Serial.print(receiver_input_channel_2);
  
  Serial.print("  Ch_3:");
  if(receiver_input_channel_3 - pos_dir < 0)Serial.print("vvv"); 
  else if(receiver_input_channel_3 - neg_dir > 0)Serial.print("^^^");
  else Serial.print("-+-");
  Serial.print(receiver_input_channel_3);
  
  Serial.print("  Ch_4:");
  if(receiver_input_channel_4 - pos_dir < 0)Serial.print("<<<"); 
  else if(receiver_input_channel_4 - neg_dir > 0)Serial.print(">>>");
  else Serial.print("-+-");
  Serial.print(receiver_input_channel_4);
  
  Serial.print("  Ch_5 (SWA) :");
  if (receiver_input_channel_5 < 1000)Serial.print("^^^"); // Channel 5 (SWA in the up position) returns a value of 944->948.
  else if (receiver_input_channel_5 > 1800)Serial.print("vvv"); // Channel 5 (SWA in the up position) returns a value of 1944->1948
  Serial.print(receiver_input_channel_5);

  Serial.print("  Ch_6 (SWC) :");
  if (receiver_input_channel_6 < 1000)Serial.print("^^^"); // Channel 6 (SWC in the up position) returns a value of 984->988
  else if (1000 < receiver_input_channel_6 < 1600)Serial.print("-+-"); // Channel 6 (SWC in the middle position) returns 1492->1498
  else if (receiver_input_channel_6 > 1700)Serial.print("vvv"); // Channel 6 (SWC in the down position) returns a value of 1984->1988 ========================= this line doesnt work :( but Mehhhh
  Serial.println(receiver_input_channel_6);
  
}
