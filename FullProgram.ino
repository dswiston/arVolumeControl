#include <avr/power.h>

// -------------------------------------------------------------------------------------------
// IR DEFINITIONS and VARIABLES
// -------------------------------------------------------------------------------------------
// define button codes
#define btnMute    148
#define btnDown    147
#define btnUp      146
// define waveform constants
#define irPin               2               // the pin the IR sensor is connected to
#define irNumBits          12               // number of bits in the IR waveform
#define irStartMin       2000             // the starting pulse's minimum duration (us)
#define irStartMax       3000             // the starting pules's maximum duration (us)
#define irDataMax        1500             // timeout value when reading data (us)
#define irBinOne         1000             // threshold for defining a pulse to be a binary 1 (us)

volatile unsigned int irSignal = 0;
volatile unsigned long startTime;
// -------------------------------------------------------------------------------------------
// END IR DEFINITIONS and VARIABLES
// -------------------------------------------------------------------------------------------


// -------------------------------------------------------------------------------------------
// Rotary Encoder DEFINITIONS and VARIABLES
// -------------------------------------------------------------------------------------------
#define encoderPinA 0
#define encoderPinB 1

// The variables below are changed by the rotary encoder interrupts
volatile unsigned int inA = 0;
volatile unsigned int inB = 0;

/* This variable represents the possible combinations of a rotary encoders current and previous states
   Since a rotaty encoder uses a gray code we know that only one of the two bits should ever change.
   The locations in the vector that have a '1' mean the encoder was turned clockwise.  The locations
   that have a '-1' mean the encoder was turned counter-clockwise.  The locations that have a '0' mean
   that the state transition was invalid (most likely due to switch bouncing).  Recognizing and ignoring
   the invalid transitions performs an efficient debouncing operation. */
const byte encStates[] = {0,2,1,0,1,0,0,2,2,0,0,1,0,1,2,0};

// This represents the rotary encoder's current position - potentially updated
volatile byte rotaryEncoder = 0;
// This variable stores the encoders previous state.  It actually stores 4 old states but we only use one of them
volatile byte oldEncoderState = 0;
// -------------------------------------------------------------------------------------------
// End Rotary Encoder DEFINITIONS and VARIABLES
// -------------------------------------------------------------------------------------------


// -------------------------------------------------------------------------------------------
// PGA2311 DEFINITIONS
// -------------------------------------------------------------------------------------------
#define pgaCsPin 4
#define pgaSdiPin 5
#define pgaMutePin 6
#define pgaSclkPin 7
// -------------------------------------------------------------------------------------------
// END PGA2311 DEFINITIONS
// -------------------------------------------------------------------------------------------


// -------------------------------------------------------------------------------------------
// LED DEFINITIONS
// -------------------------------------------------------------------------------------------
#define ledSdi    8   //Data
#define ledClk    9   //Clock
#define ledLe     10  //Latch
// The values below are used to light specific LEDs.  The array is indexed into.  The funky 
// order of these numbers is a result of my circuit board layout (LED0 is in the middle).
unsigned int ledSequence[] = {256, 512, 1024, 2048, 4096, 8192, 16384, 32768, 1, 2, 4, 8, 16, 32, 64, 128};
// -------------------------------------------------------------------------------------------
// END LED DEFINITIONS
// -------------------------------------------------------------------------------------------


#define maxVolumeLevel 255
#define minVolumeLevel 0
volatile float volumeLevel = 0;


void setup()
{

  // Disable the following peripherals to save power
  power_adc_disable();
  power_spi_disable();
  power_twi_disable();
  
  // Initialize each pga serial interface pin to be an output
  pinMode(pgaCsPin,OUTPUT);
  digitalWrite(pgaCsPin,HIGH);
  pinMode(pgaSclkPin,OUTPUT);
  digitalWrite(pgaSclkPin,HIGH);
  pinMode(pgaSdiPin,OUTPUT);
  digitalWrite(pgaSdiPin,HIGH);
  pinMode(pgaMutePin,OUTPUT);
  digitalWrite(pgaMutePin,HIGH);
  
  // Initially set the volume to mute
  PgaSetVolume(0);
  
  // Initialize the rotary encoder pins to use internal pullup resistors
  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP);
  // Attach interrupts to each pin so we don't have to waste time polling
  attachInterrupt(3, readEncoderA, CHANGE);
  attachInterrupt(2, readEncoderB, CHANGE);
  
  // Initialize the IR pin
  pinMode(irPin, INPUT);
  // attach an interrupt to the IR pin
  attachInterrupt(1, irActivity, CHANGE);
  
  // Set the LED pins
  pinMode(ledSdi, OUTPUT);
  pinMode(ledClk, OUTPUT);
  pinMode(ledLe, OUTPUT);

  SetVolumeLed(ledSequence[0]);
  
  // Initialize the serial port
  Serial.begin(9600);
  
}

void loop()
{ 
   // The rotary encoder saw movement up or down
   if (rotaryEncoder > 0)
   {
      if (rotaryEncoder == 1)
      {
         // Movement was up, increase the volume
         volumeLevel = PgaSetVolume(volumeLevel - 0.5);
      }
      else
      {
         // Movement was down,decrease the volume
         volumeLevel = PgaSetVolume(volumeLevel + 0.5);
      }
      // Reset/clear the encoder's state in preperation for the next event
      rotaryEncoder = 0;
    }
   
   // If an IR signal was received, figure out what to do with it  
   if (irSignal > 0)
   { 
      //Switch statement to check for specific buttons pressed
      switch (irSignal)
      {     
         case btnMute:
           // Mute the system
           volumeLevel = PgaSetVolume(0.0);
           break;
      
         case btnDown:
           // Reduce the volume (1/3 because one button press causes a minimum of three codes to be sent)
           volumeLevel = PgaSetVolume(volumeLevel - 0.333);
           break;   
      
         case btnUp:
           // Increase the volume (1/3 because one button press causes a minimum of three codes to be sent)
           volumeLevel = PgaSetVolume(volumeLevel + 0.333);
           break; 
      }
      // Reset/clear the IR's state in preperation for the next event
      irSignal = 0;
   }
  
/*  if (Serial.available())
  {
    volumeLevel = PgaSetVolume(volumeLevel + Serial.read() - '0');
  }*/
   
}

void readEncoderA()
{ 
  inA = digitalRead(encoderPinA);              // read pinA, this is the pin that changed
  oldEncoderState <<= 2;                       // bit-shift the encoder's old state history by two bits
  oldEncoderState |= ( inB<<1 | inA );         // add the new state to the lowest two bits of the encoder's state history
  rotaryEncoder = encStates[( oldEncoderState & 0x0f )]; // increment the encoder's position based on the current state and the previous state
}

void readEncoderB()
{
  inB = digitalRead(encoderPinB);              // read pinB, this is the pin that changed
  oldEncoderState <<= 2;                       // bit-shift the encoder's old state history by two bits
  oldEncoderState |= ( inB<<1 | inA );         // add the new state to the lowest two bits of the encoder's state history
  rotaryEncoder = encStates[( oldEncoderState & 0x0f )]; // increment the encoder's position based on the current state and the previous state
}

float PgaSetVolume(float value)
{
  
  // Make sure we didn't violate the maximum or minimum settings
  if (value > maxVolumeLevel)
  {
     value = maxVolumeLevel;
  }
  else if (value < minVolumeLevel)
  {
     value = minVolumeLevel;
  }

  // Convert the volume state to a byte (0-255)
  int byteVal = (byte)((int)value);
  
  // To write to the device, CS is asserted low, then the clock and data are sent.
  // The right channel and the left channel are written as a single 16bit word.
  // The right channel is written first (MSB to LSB) and then the left (MSB to LSB)
  digitalWrite(pgaCsPin, LOW);  // assert CS
  SpiWrite(byteVal);            // right channel
  SpiWrite(byteVal);            // left channel
  digitalWrite(pgaCsPin, HIGH); // deassert CS
  
  // Set the LED indicator
  SetVolumeLed(ledSequence[byteVal>>4]);

  Serial.print("Volume: ");
  Serial.println(byteVal,DEC);
  Serial.println(byteVal>>4,DEC);
  
  return value;
}

static inline void SpiWrite(byte byteToWrite)
{
   byte i;
   // loop thru each of the 8-bits in the byte
   for (i=0; i < 8; i++)
   {
     // strobe clock
     digitalWrite(pgaSclkPin, LOW);
     // Send the bit (MSB to LSB).  "0x80 & ..." looks at only the MSB
     if (0x80 & byteToWrite)
     {
       digitalWrite(pgaSdiPin, HIGH);
     }
     else
     {
       digitalWrite(pgaSdiPin, LOW);
     }
     // unstrobe the clock ;)
     digitalWrite(pgaSclkPin, HIGH);
     // left-shift one byte, dumping off the MSB bit
     byteToWrite <<= 1;
   }
}

/* This interrupt is called when a change state is detected on the IR pin.  It is
   compatible with SONY IR codes.  The very first entry into the interrupt will 
   be when the IR signal is pulled low.  The SONY IR code has a lengthy low pulse 
   to signify the start of a code.  The second entry into the interrupt will be 
   when the IR signal has transitioned high.  An initial check is done to make sure
   that the start pulse was the correct size.  This allows the interrupt to ignore
   (not trigger) on other incompatible IR signals.  If the initial pulse check passes
   the criteria, the resulting bit sequence is collected in a loop.  The bit sequence
   is then converted into the resulting IR code.  The code is then made available to
   the master loop.  */ 
void irActivity()
{
  // Check for a starting sequence
  if ( digitalRead(irPin) == 0)
  {
    startTime = micros();
    return;
  }

  // If we got to here, it must have been on a rising edge
  // Check for bogus start pulse
  if ( (micros() - startTime < irStartMin) | (micros() - startTime > irStartMax) )
  {
    return;
  }

  // This appears to be a real IR sequence, setup variables to begin collecting the data
  unsigned int ndx;
  unsigned int data[12];
  
  // Collect the data
  for (ndx=0; ndx < irNumBits; ndx++)
  {
    data[ndx] = pulseIn(irPin,LOW,irDataMax);
    // Check to see if we hit the timeout. Should never hit this.  If so, punt.
    if ( data[ndx] == 0 )
    {
      return;
    }   
  }
  
  // Calculate the resulting IR code.  The variable "key" is a global that is accessible 
  // to the main loop. 
  for (ndx=0; ndx < irNumBits; ndx++)
  {
    if ( data[ndx] > irBinOne )
    {
      irSignal += 1<<ndx;
    }
  }

}

void SetVolumeLed(int ledVal)
{ 
  digitalWrite(ledLe,LOW); // Let the TI chip know you are talking to it
  shiftOut(ledSdi,ledClk,MSBFIRST,(ledVal >> 8));  // write out the first byte
  shiftOut(ledSdi,ledClk,MSBFIRST,ledVal);  // write out the second byte
  digitalWrite(ledLe,HIGH); // Stop communicating with the TI chip
}
