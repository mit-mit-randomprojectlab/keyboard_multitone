/* 
keyboard_multitone - sketch to test basic keyboard matrix using teensy
and using a custom implementation of "Tone" (square-wave generation) that uses
all four PIT timers to generate up to four simultaneous tones.

Code for PIT timers largely derived and simplified from Teensy "IntervalTimer.cpp" in 
hardware/teensy/avr/cores/teensy3/IntervalTimer.cpp

By mit-mit
*/

#include "kinetis.h"
#include "core_pins.h"
#include "pins_arduino.h"
#include "HardwareSerial.h"

#include "pitches.h"

// Multi-tone Stuff

static uint32_t tone_toggle_count0;
static uint32_t tone_toggle_count1;
static uint32_t tone_toggle_count2;
static uint32_t tone_toggle_count3;

static volatile uint8_t *tone_reg0;
static volatile uint8_t *tone_reg1;
static volatile uint8_t *tone_reg2;
static volatile uint8_t *tone_reg3;

static uint8_t tone_state0=0;
static uint8_t tone_state1=0;
static uint8_t tone_state2=0;
static uint8_t tone_state3=0;

static float tone_usec0=0.0;
static float tone_usec1=0.0;
static float tone_usec2=0.0;
static float tone_usec3=0.0;

static uint32_t tone_new_count0=0;
static uint32_t tone_new_count1=0;
static uint32_t tone_new_count2=0;
static uint32_t tone_new_count3=0;

#define TONE_CLEAR_PIN0   tone_reg0[0] = 1
#define TONE_CLEAR_PIN1   tone_reg1[0] = 1
#define TONE_CLEAR_PIN2   tone_reg2[0] = 1
#define TONE_CLEAR_PIN3   tone_reg3[0] = 1

#define TONE_TOGGLE_PIN0  tone_reg0[128] = 1
#define TONE_TOGGLE_PIN1  tone_reg1[128] = 1
#define TONE_TOGGLE_PIN2  tone_reg2[128] = 1
#define TONE_TOGGLE_PIN3  tone_reg3[128] = 1

#define TONE_OUTPUT_PIN0  tone_reg0[384] = 1
#define TONE_OUTPUT_PIN1  tone_reg1[384] = 1
#define TONE_OUTPUT_PIN2  tone_reg2[384] = 1
#define TONE_OUTPUT_PIN3  tone_reg3[384] = 1

uint8_t soundpins[] = {8,17,18,19};
//uint8_t soundpins[] = {19,8,8,8};

static bool PIT_enabled = false;

void enable_PIT() {
  SIM_SCGC6 |= SIM_SCGC6_PIT;
  PIT_MCR = 0;
  PIT_enabled = true;
}

typedef volatile uint32_t* reg;
reg PIT_LDVAL;
reg PIT_TCTRL;
uint8_t IRQ_PIT_CH;

void start_PIT(uint8_t PIT_id, uint32_t newValue) {
  
  // point to the correct registers
  PIT_LDVAL = &PIT_LDVAL0 + PIT_id * 4;
  PIT_TCTRL = &PIT_TCTRL0 + PIT_id * 4;
  
  // write value to register and enable interrupt
  *PIT_TCTRL = 0;
  *PIT_LDVAL = newValue;
  *PIT_TCTRL = 3;
  
  IRQ_PIT_CH = IRQ_PIT_CH0 + PIT_id;
  NVIC_SET_PRIORITY(IRQ_PIT_CH, 128);
  NVIC_ENABLE_IRQ(IRQ_PIT_CH);

}

// Interupt functions
void pit0_isr() { 
  PIT_TFLG0 = 1; 
  TONE_TOGGLE_PIN0; // toggle
  tone_toggle_count0--;
  if (tone_toggle_count0 == 0xFFFFFFFB) tone_toggle_count0 = 0xFFFFFFFD;
}
void pit1_isr() { 
  PIT_TFLG1 = 1; 
  TONE_TOGGLE_PIN1; // toggle
  tone_toggle_count1--;
  if (tone_toggle_count1 == 0xFFFFFFFB) tone_toggle_count1 = 0xFFFFFFFD;
}
void pit2_isr() { 
  PIT_TFLG2 = 1; 
  TONE_TOGGLE_PIN2; // toggle
  tone_toggle_count2--;
  if (tone_toggle_count2 == 0xFFFFFFFB) tone_toggle_count2 = 0xFFFFFFFD;
}
void pit3_isr() { 
  PIT_TFLG3 = 1; 
  TONE_TOGGLE_PIN3; // toggle
  tone_toggle_count3--;
  if (tone_toggle_count3 == 0xFFFFFFFB) tone_toggle_count3 = 0xFFFFFFFD;
}

void tone_multi(uint8_t channel, uint16_t frequency, uint32_t duration)
{
  uint32_t count;
  volatile uint32_t *config;
  float usec;
  uint32_t newValue;

  uint8_t pin = soundpins[channel];
  
  if (pin >= CORE_NUM_DIGITAL) return;
  if (duration > 0) {
    count = (frequency * duration / 1000) * 2;
    if (!(count & 1)) count++; // always full waveform cycles
  } else {
    count = 0xFFFFFFFD;
  }
  usec = (float)500000.0 / (float)frequency;
  config = portConfigRegister(pin);

  __disable_irq();
  if (channel == 0) {
    if (0) { //(tone_state0 == 1) {
      tone_usec0 = usec;
      tone_new_count0 = count;
      tone_toggle_count0 = (tone_toggle_count0 & 1);
    } 
    else {
      //TONE_CLEAR_PIN0;
      tone_state0 = 1;
      tone_reg0 = portClearRegister(pin);
      TONE_CLEAR_PIN0; // clear pin
      TONE_OUTPUT_PIN0; // output mode;
      *config = PORT_PCR_SRE | PORT_PCR_DSE | PORT_PCR_MUX(1);
      tone_toggle_count0 = count;
      tone_usec0 = usec;
      newValue = (float)(F_BUS / 1000000) * usec - 0.5;
      if (!PIT_enabled) enable_PIT();
      start_PIT(channel,newValue);
    }
  }
  else if (channel == 1) {
    if (0) { //(tone_state1 == 1) {
      tone_usec1 = usec;
      tone_new_count1 = count;
      tone_toggle_count1 = (tone_toggle_count1 & 1);
    } 
    else {
      //TONE_CLEAR_PIN1;
      tone_state1 = 1;
      tone_reg1 = portClearRegister(pin);
      TONE_CLEAR_PIN1; // clear pin
      TONE_OUTPUT_PIN1; // output mode;
      *config = PORT_PCR_SRE | PORT_PCR_DSE | PORT_PCR_MUX(1);
      tone_toggle_count1 = count;
      tone_usec1 = usec;
      newValue = (float)(F_BUS / 1000000) * usec - 0.5;
      if (!PIT_enabled) enable_PIT();
      start_PIT(channel,newValue);
    }
  }
  else if (channel == 2) {
    if (0) { //(tone_state2 == 1) {
      tone_usec2 = usec;
      tone_new_count2 = count;
      tone_toggle_count2 = (tone_toggle_count2 & 1);
    } 
    else {
      //TONE_CLEAR_PIN2;
      tone_state2 = 1;
      tone_reg2 = portClearRegister(pin);
      TONE_CLEAR_PIN2; // clear pin
      TONE_OUTPUT_PIN2; // output mode;
      *config = PORT_PCR_SRE | PORT_PCR_DSE | PORT_PCR_MUX(1);
      tone_toggle_count2 = count;
      tone_usec2 = usec;
      newValue = (float)(F_BUS / 1000000) * usec - 0.5;
      if (!PIT_enabled) enable_PIT();
      start_PIT(channel,newValue);
    }
  }
  else {
    if (0) { //(tone_state3 == 1) {
      tone_usec3 = usec;
      tone_new_count3 = count;
      tone_toggle_count3 = (tone_toggle_count3 & 1);
    } 
    else {
      //TONE_CLEAR_PIN3;
      tone_state3 = 1;
      tone_reg3 = portClearRegister(pin);
      TONE_CLEAR_PIN3; // clear pin
      TONE_OUTPUT_PIN3; // output mode;
      *config = PORT_PCR_SRE | PORT_PCR_DSE | PORT_PCR_MUX(1);
      tone_toggle_count3 = count;
      tone_usec3 = usec;
      newValue = (float)(F_BUS / 1000000) * usec - 0.5;
      if (!PIT_enabled) enable_PIT();
      start_PIT(channel,newValue);
    }
  }
  __enable_irq();
  
}

void noTone_multi(uint8_t channel)
{
  uint8_t pin = soundpins[channel];
  if (pin >= CORE_NUM_DIGITAL) return;
  __disable_irq();
  
  PIT_TCTRL = &PIT_TCTRL0 + channel * 4;
  IRQ_PIT_CH = IRQ_PIT_CH0 + channel;
  *PIT_TCTRL = 0;
  NVIC_DISABLE_IRQ(IRQ_PIT_CH);
  
  if (channel == 0) {
    if (tone_state0 == 1) {
      TONE_CLEAR_PIN0; // clear
      tone_state0 = 0;
    }
  }
  else if (channel == 1) {
    if (tone_state1 == 1) {
      TONE_CLEAR_PIN1; // clear
      tone_state1 = 0;
    }
  }
  else if (channel == 2) {
    if (tone_state2 == 1) {
      TONE_CLEAR_PIN2; // clear
      tone_state2 = 0;
    }
  }
  else {
    if (tone_state3 == 1) {
      TONE_CLEAR_PIN3; // clear
      tone_state3 = 0;
    }
  }
  __enable_irq();
}





// Stuff for running keyboard
char keystate[49] = {0,0,0,0,0,0,
  0,0,0,0,0,0,
  0,0,0,0,0,0,
  0,0,0,0,0,0,
  0,0,0,0,0,0,
  0,0,0,0,0,0,
  0,0,0,0,0,0,
  0,0,0,0,0,0,
  0};

char keystate_prev[49] = {0,0,0,0,0,0,
  0,0,0,0,0,0,
  0,0,0,0,0,0,
  0,0,0,0,0,0,
  0,0,0,0,0,0,
  0,0,0,0,0,0,
  0,0,0,0,0,0,
  0,0,0,0,0,0,
  0};

char keystate_channel[49] = {0,0,0,0,0,0,
  0,0,0,0,0,0,
  0,0,0,0,0,0,
  0,0,0,0,0,0,
  0,0,0,0,0,0,
  0,0,0,0,0,0,
  0,0,0,0,0,0,
  0,0,0,0,0,0,
  0};

char pins_rows[6] = {5,4,3,2,1,0};
char pins_cols[9] = {11,14,16,6,7,10,12,15,9};

char current_channel = 0;
char channel_keyids[4] = {255,255,255,255};
char channel_keyids_prev[4] = {255,255,255,255};

void init_keys() {
  for (int i = 0; i < 6; i++) {
    pinMode(pins_rows[i], OUTPUT);
    digitalWrite(pins_rows[i], LOW);
  }
  for (int i = 0; i < 9; i++) {
    pinMode(pins_cols[i], OUTPUT);
    digitalWrite(pins_cols[i], HIGH);
  }
}

// read_keys - Implements switch matrix
void read_keys() {
  int k = 0;
  
  // Read bottom C
  digitalWrite(pins_cols[0], LOW);
  pinMode(pins_rows[5], INPUT_PULLUP);
  delayMicroseconds(10);
  if (digitalRead(pins_rows[5])) {
    keystate[k] = 0;
  }
  else {
    keystate[k] = 1;
  }
  pinMode(pins_rows[5], OUTPUT);
  digitalWrite(pins_rows[5], LOW);
  k++;
  digitalWrite(pins_cols[0], HIGH);
  
  // Read remaining keys
  for (int i = 1; i < 9; i++) {
    digitalWrite(pins_cols[i], LOW);
    for (int j = 0; j < 6; j++) {
      pinMode(pins_rows[j], INPUT_PULLUP);
      delayMicroseconds(10);
      if (digitalRead(pins_rows[j])) {
        keystate[k] = 0;
      }
      else {
        keystate[k] = 1;
      }
      pinMode(pins_rows[j], OUTPUT);
      digitalWrite(pins_rows[j], LOW);
      k++;
      delayMicroseconds(50);
    }
    digitalWrite(pins_cols[i], HIGH);
  }
}

void setup() {
  //Serial.begin(9600);
  
  // setup pin 13 LED for when keys pressed
  pinMode(13, OUTPUT);
  
  // Initialise all pins ready for cycling through reading
  init_keys();

  // initialise key/channel data
  for (int i = 0; i < 49; i++) {
    keystate_channel[i] = 255;
  }
  
}

void loop() {
  
  // read keys
  read_keys();
  
  // assign notes on multiple channels
  int played_note = 0;
  for (int i = 0; i < 49; i++) {
    if (keystate[i] == 1) {
      digitalWrite(13, HIGH);
      played_note = 1;
      if ( (keystate_prev[i] == 0) || (keystate_channel[i] == 255) ) { // start up a key
        for (int channel = 0; channel < 3; channel++) { // look for a channel
          if (channel_keyids[channel] == 255) { // was free
            keystate_channel[i] = channel;
            channel_keyids[channel] = i;
            break;
          }
          else if (channel_keyids[channel] < i) { // was taken by a lower key, grab it
            keystate_channel[(int)channel_keyids[channel]] = 255; // tell other key it no longer has channel
            keystate_channel[i] = channel;
            channel_keyids[channel] = i;
            break;
          }
        }
      }
    }
    else if (keystate_prev[i] == 1) { // key just got turned off
      if (keystate_channel[i] < 255) { // has a channel, need to turn off
        channel_keyids[(int)keystate_channel[i]] = 255;
        keystate_channel[i] = 255;
      }
    }
  }

  // fire up tones/turn off tones
  for (int i = 0; i < 3; i++) {
    if (channel_keyids[i] != channel_keyids_prev[i]) { // key assigned to channel changed
      if (channel_keyids[i] == 255) {
        noTone_multi(i);
      }
      else {
        tone_multi(i, pitches[channel_keyids[i]+25], 0);
      }
    }
  }

  // Update previous states
  for (int i = 0; i < 49; i++) {
    keystate_prev[i] = keystate[i];
  }
  for (int i = 0; i < 3; i++) {
    channel_keyids_prev[i] = channel_keyids[i];
  }

  if (played_note == 0) { // light up keyboard
    digitalWrite(13, LOW);
  }

}
