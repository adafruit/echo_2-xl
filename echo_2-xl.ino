/*
  ADAVOICE is an Arduino-based voice pitch changer plus WAV playback.
  Fun for Halloween costumes, comic convention getups and other shenanigans!

  Hardware requirements:
  - Arduino Uno, Duemilanove or Diecimila (not Mega or Leonardo compatible).
  - Adafruit Wave Shield
  - Speaker attached to Wave Shield output
  - Battery for portable use
  - SD card
  Software requirements:
  - WaveHC library for Arduino
  - WAV files on FAT-formatted SD card

  This example sketch uses a 3x4 keypad for triggering sounds...but with
  some changes could be adapted to use several discrete buttons, Hall effect
  sensors, force-sensing resistors (FSRs), I2C keypads, etc. (or if you just
  want the voice effect, no buttons at all).

  Connections:
  - 3.3V to mic amp+, 1 leg of potentiometer and Arduino AREF pin
  - GND to mic amp-, opposite leg of potentiometer
  - Analog pin 0 to mic amp output
  - Analog pin 1 to center tap of potentiometer
  - Wave Shield output to speaker or amplifier
  - Matrix is wired to pins A2, A3, A4, A5 (rows) and 6, 7, 8 (columns)
  - Wave shield is assumed wired as in product tutorial

  Potentiometer sets playback pitch.  Pitch adjustment does NOT work in
  realtime -- audio sampling requires 100% of the ADC.  Pitch setting is
  read at startup (or reset) and after a WAV finishes playing.

  POINT SPEAKER AWAY FROM MIC to avoid feedback.

  Written by Adafruit industries, with portions adapted from the
  'PiSpeakHC' sketch included with WaveHC library.
*/

#include <WaveHC.h>
#include <WaveUtil.h>

SdReader  card;  // This object holds the information for the card
FatVolume vol;   // This holds the information for the partition on the card
FatReader root;  // This holds the information for the volumes root directory
FatReader file;  // This object represent the WAV file for a pi digit or period
WaveHC    wave;  // This is the only wave (audio) object, -- we only play one at a time
#define error(msg) error_P(PSTR(msg))  // Macro allows error messages in flash memory

#define ADC_CHANNEL 0 // Microphone on Analog pin 0

// Wave shield DAC: digital pins 2, 3, 4, 5
#define DAC_CS_PORT    PORTD
#define DAC_CS         PORTD2
#define DAC_CLK_PORT   PORTD
#define DAC_CLK        PORTD3
#define DAC_DI_PORT    PORTD
#define DAC_DI         PORTD4
#define DAC_LATCH_PORT PORTD
#define DAC_LATCH      PORTD5
#define PITCH          910

#define BUTTON1        8
#define BUTTON2        9
#define BUTTON3        16
#define BUTTON4        19

boolean startup = true;

uint16_t in = 0, out = 0, xf = 0, nSamples; // Audio sample counters
uint8_t  adc_save;                          // Default ADC mode

// WaveHC didn't declare it's working buffers private or static,
// so we can be sneaky and borrow the same RAM for audio sampling!
extern uint8_t
buffer1[PLAYBUFFLEN],                   // Audio sample LSB
        buffer2[PLAYBUFFLEN];                   // Audio sample MSB
#define XFADE     16                      // Number of samples for cross-fade
#define MAX_SAMPLES (PLAYBUFFLEN - XFADE) // Remaining available audio samples

uint8_t prev = 255;

//Wav file names
const char *sound[] = {
  "TURNON" , "SHUTDOWN", "LAUGH"   ,
  "WINNER"  , "BEEPS" , "TWEETS"    ,
  "ALERT", "DRONE"  , "DOOPDIP",
  "DOODOO", "COMPUTE", "BOOTUP"    ,
  "SONG"
};


//////////////////////////////////// SETUP

void setup() {
  uint8_t i;

  Serial.begin(9600);

  // The WaveHC library normally initializes the DAC pins...but only after
  // an SD card is detected and a valid file is passed.  Need to init the
  // pins manually here so that voice FX works even without a card.
  pinMode(2, OUTPUT);    // Chip select
  pinMode(3, OUTPUT);    // Serial clock
  pinMode(4, OUTPUT);    // Serial data
  pinMode(5, OUTPUT);    // Latch
  pinMode(15, OUTPUT);    // Output
  digitalWrite(2, HIGH); // Set chip select high

  pinMode(BUTTON1, INPUT_PULLUP);
  pinMode(BUTTON2, INPUT_PULLUP);
  pinMode(BUTTON3, INPUT_PULLUP);
  pinMode(BUTTON4, INPUT_PULLUP);

  // Init SD library, show root directory.  Note that errors are displayed
  // but NOT regarded as fatal -- the program will continue with voice FX!
  if (!card.init())             SerialPrint_P("Card init. failed!");
  else if (!vol.init(card))     SerialPrint_P("No partition!");
  else if (!root.openRoot(vol)) SerialPrint_P("Couldn't open dir");
  else {
    PgmPrintln("Files found:");
    root.ls();
    // Play startup sound (last file in array).
    playfile(sizeof(sound) / sizeof(sound[0]) - 1);
  }

  // Optional, but may make sampling and playback a little smoother:
  // Disable Timer0 interrupt.  This means delay(), millis() etc. won't
  // work.  Comment this out if you really, really need those functions.
  TIMSK0 = 0;

  // Set up Analog-to-Digital converter:
  //  analogReference(DEFAULT); // 5V to AREF
  adc_save = ADCSRA;         // Save ADC setting for restore later

  //check the button switchboard for initial state
  prev = checkButtons();

  //play startup sound
  playfile(0);
  digitalWrite(15, HIGH);
  while (wave.isplaying); // Wait for startup sound to finish...
  digitalWrite(15, LOW);

  startPitchShift();     // and start the pitch-shift mode by default.
}


//////////////////////////////////// LOOP

// As written here, the loop function scans a keypad to triggers sounds
// (stopping and restarting the voice effect as needed).  If all you need
// is a couple of buttons, it may be easier to tear this out and start
// over with some simple digitalRead() calls.

void loop() {

  uint8_t button = checkButtons();

  if ((button != prev) & (button != 255)) { // Same button as before?
    playfile(button);                    // and play new sound.
    prev = button;
  }

  //Light LED with voice
  uint16_t val = ((buffer2[out] << 8) | buffer1[out]);
  //  Serial.print("val ="); Serial.println(val, DEC);
  if ((val > 700) || (val < 300)) {
    digitalWrite(15, HIGH);
  }
  else {
    digitalWrite(15, LOW);
  }

  if (!wave.isplaying && !(TIMSK2 & _BV(TOIE2))) {
    startPitchShift();
  }

}


uint8_t checkButtons() {

  uint8_t button = 255;

  if (digitalRead(BUTTON1) == LOW) { //8,9,11,12
    button = 1;
  }
  else if (digitalRead(BUTTON2) == LOW) { //8,9,11,12
    button = 2;
  }
  else if (digitalRead(BUTTON3) == LOW) { //8,9,11,12
    button = 3;
  }
  else if (digitalRead(BUTTON4) == LOW) { //8,9,11,12
    button = 4;
  }

  return button;
}


//////////////////////////////////// HELPERS

// Open and start playing a WAV file
void playfile(int idx) {

  if (wave.isplaying) wave.stop();     // Stop current WAV (if any)
  else               stopPitchShift(); // or stop voice effect

  char filename[13];

  (void)sprintf(filename, "%s.wav", sound[idx]);
  Serial.print("File: ");
  Serial.println(filename);

  if (!file.open(root, filename)) {
    PgmPrint("Couldn't open file ");
    Serial.print(filename);
    return;
  }
  if (!wave.create(file)) {
    PgmPrintln("Not a valid WAV");
    return;
  }
  wave.play();

}


//////////////////////////////////// PITCH-SHIFT CODE

void startPitchShift() {

  // Read analog pitch setting before starting audio sampling:
  //  int pitch = analogRead(1);
  int pitch = PITCH;
  //  Serial.print("Pitch: ");
  //  Serial.println(pitch);

  // Right now the sketch just uses a fixed sound buffer length of
  // 128 samples.  It may be the case that the buffer length should
  // vary with pitch for better results...further experimentation
  // is required here.
  nSamples = 128;
  //nSamples = F_CPU / 3200 / OCR2A; // ???
  //if(nSamples > MAX_SAMPLES)      nSamples = MAX_SAMPLES;
  //else if(nSamples < (XFADE * 2)) nSamples = XFADE * 2;

  memset(buffer1, 0, nSamples + XFADE); // Clear sample buffers
  memset(buffer2, 2, nSamples + XFADE); // (set all samples to 512)

  // WaveHC library already defines a Timer1 interrupt handler.  Since we
  // want to use the stock library and not require a special fork, Timer2
  // is used for a sample-playing interrupt here.  As it's only an 8-bit
  // timer, a sizeable prescaler is used (32:1) to generate intervals
  // spanning the desired range (~4.8 KHz to ~19 KHz, or +/- 1 octave
  // from the sampling frequency).  This does limit the available number
  // of speed 'steps' in between (about 79 total), but seems enough.
  TCCR2A = _BV(WGM21) | _BV(WGM20); // Mode 7 (fast PWM), OC2 disconnected
  TCCR2B = _BV(WGM22) | _BV(CS21) | _BV(CS20);  // 32:1 prescale
  OCR2A  = map(pitch, 0, 1023,
               F_CPU / 32 / (9615 / 2),  // Lowest pitch  = -1 octave
               F_CPU / 32 / (9615 * 2)); // Highest pitch = +1 octave

  // Start up ADC in free-run mode for audio sampling:
  DIDR0 |= _BV(ADC0D);  // Disable digital input buffer on ADC0
  ADMUX  = ADC_CHANNEL; // Channel sel, right-adj, AREF to 3.3V regulator
  ADCSRB = 0;           // Free-run mode
  ADCSRA = _BV(ADEN) |  // Enable ADC
           _BV(ADSC)  |        // Start conversions
           _BV(ADATE) |        // Auto-trigger enable
           _BV(ADIE)  |        // Interrupt enable
           _BV(ADPS2) |        // 128:1 prescale...
           _BV(ADPS1) |        //  ...yields 125 KHz ADC clock...
           _BV(ADPS0);         //  ...13 cycles/conversion = ~9615 Hz

  TIMSK2 |= _BV(TOIE2); // Enable Timer2 overflow interrupt
  sei();                // Enable interrupts
}

void stopPitchShift() {
  ADCSRA = adc_save; // Disable ADC interrupt and allow normal use
  TIMSK2 = 0;        // Disable Timer2 Interrupt
}

ISR(ADC_vect, ISR_BLOCK) { // ADC conversion complete

  // Save old sample from 'in' position to xfade buffer:
  buffer1[nSamples + xf] = buffer1[in];
  buffer2[nSamples + xf] = buffer2[in];
  if (++xf >= XFADE) xf = 0;

  // Store new value in sample buffers:
  buffer1[in] = ADCL; // MUST read ADCL first!
  buffer2[in] = ADCH;
  if (++in >= nSamples) in = 0;



}

ISR(TIMER2_OVF_vect) { // Playback interrupt
  uint16_t s;
  uint8_t  w, inv, hi, lo, bit;
  int      o2, i2, pos;

  // Cross fade around circular buffer 'seam'.
  if ((o2 = (int)out) == (i2 = (int)in)) {
    // Sample positions coincide.  Use cross-fade buffer data directly.
    pos = nSamples + xf;
    hi = (buffer2[pos] << 2) | (buffer1[pos] >> 6); // Expand 10-bit data
    lo = (buffer1[pos] << 2) |  buffer2[pos];       // to 12 bits
  } if ((o2 < i2) && (o2 > (i2 - XFADE))) {
    // Output sample is close to end of input samples.  Cross-fade to
    // avoid click.  The shift operations here assume that XFADE is 16;
    // will need adjustment if that changes.
    w   = in - out;  // Weight of sample (1-n)
    inv = XFADE - w; // Weight of xfade
    pos = nSamples + ((inv + xf) % XFADE);
    s   = ((buffer2[out] << 8) | buffer1[out]) * w +
          ((buffer2[pos] << 8) | buffer1[pos]) * inv;
    hi = s >> 10; // Shift 14 bit result
    lo = s >> 2;  // down to 12 bits
  } else if (o2 > (i2 + nSamples - XFADE)) {
    // More cross-fade condition
    w   = in + nSamples - out;
    inv = XFADE - w;
    pos = nSamples + ((inv + xf) % XFADE);
    s   = ((buffer2[out] << 8) | buffer1[out]) * w +
          ((buffer2[pos] << 8) | buffer1[pos]) * inv;
    hi = s >> 10; // Shift 14 bit result
    lo = s >> 2;  // down to 12 bits
  } else {
    // Input and output counters don't coincide -- just use sample directly.
    hi = (buffer2[out] << 2) | (buffer1[out] >> 6); // Expand 10-bit data
    lo = (buffer1[out] << 2) |  buffer2[out];       // to 12 bits
  }

  // Might be possible to tweak 'hi' and 'lo' at this point to achieve
  // different voice modulations -- robot effect, etc.?

  DAC_CS_PORT &= ~_BV(DAC_CS); // Select DAC
  // Clock out 4 bits DAC config (not in loop because it's constant)
  DAC_DI_PORT  &= ~_BV(DAC_DI); // 0 = Select DAC A, unbuffered
  DAC_CLK_PORT |=  _BV(DAC_CLK); DAC_CLK_PORT &= ~_BV(DAC_CLK);
  DAC_CLK_PORT |=  _BV(DAC_CLK); DAC_CLK_PORT &= ~_BV(DAC_CLK);
  DAC_DI_PORT  |=  _BV(DAC_DI); // 1X gain, enable = 1
  DAC_CLK_PORT |=  _BV(DAC_CLK); DAC_CLK_PORT &= ~_BV(DAC_CLK);
  DAC_CLK_PORT |=  _BV(DAC_CLK); DAC_CLK_PORT &= ~_BV(DAC_CLK);
  for (bit = 0x08; bit; bit >>= 1) { // Clock out first 4 bits of data
    if (hi & bit) DAC_DI_PORT |=  _BV(DAC_DI);
    else         DAC_DI_PORT &= ~_BV(DAC_DI);
    DAC_CLK_PORT |=  _BV(DAC_CLK); DAC_CLK_PORT &= ~_BV(DAC_CLK);
  }
  for (bit = 0x80; bit; bit >>= 1) { // Clock out last 8 bits of data
    if (lo & bit) DAC_DI_PORT |=  _BV(DAC_DI);
    else         DAC_DI_PORT &= ~_BV(DAC_DI);
    DAC_CLK_PORT |=  _BV(DAC_CLK); DAC_CLK_PORT &= ~_BV(DAC_CLK);
  }
  DAC_CS_PORT    |=  _BV(DAC_CS);    // Unselect DAC

  if (++out >= nSamples) out = 0;

}

