/*
 * File:        FinalProject.c
 * Author:      James Talmage
 *              Sean Carroll
 *              Natalie Moore
 * Adapted from:
 *              main.c by
 * Author:      Syed Tahmid Mahbub
 * Target PIC:  PIC32MX250F128B
 */
/*
The MIT License (MIT)

Copyright (c) 2015 Sean Carroll, Natalie Moore, James Talmage

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

// Disable warnings from plib threading library
#define _SUPPRESS_PLIB_WARNING 1

// Import relevant libraries
#include "config.h"
// For malloc
#include <stdlib.h>
// threading library
#include <plib.h>
// config.h sets 40 MHz
#define	SYS_FREQ 40000000
#include "pt_cornell_1_2_1.h"

// --- Fixed point type macros -------------------------------------------------
typedef signed int fix16 ;
//multiply two fixed 16:16
#define multfix16(a,b) ((fix16)(((( signed long long)(a))* \
                       (( signed long long)(b)))>>16))
#define float2fix16(a) ((fix16)((a)*65536.0)) // 2^16
#define fix2float16(a) ((float)(a)/65536.0)
#define fix2int16(a)   ((int)((a)>>16))
#define int2fix16(a)   ((fix16)((a)<<16))
#define divfix16(a,b)  ((fix16)((((signed long long)(a)<<16)/(b))))
#define sqrtfix16(a)    (float2fix16(sqrt(fix2float16(a))))
#define absfix16(a)      abs(a)
// --- End fixed point type macros ---------------------------------------------

// --- SPI Stuff ---------------------------------------------------------------
volatile unsigned int DAC_data; // output value
volatile SpiChannel spi_chn = SPI_CHANNEL2; // the SPI channel to use
volatile int spi_clk_div = 2; // 20 MHz max speed for this DAC
// A-channel, 1x, active
#define DAC_config_chan_A 0b0011000000000000
// --- End SPI Stuff -----------------------------------------------------------


// --- Tuning Parameters -------------------------------------------------------
#define FS                              11025 // Sampling Freqeuncy
#define TIMER_PR                (SYS_FREQ/FS) // Fs = SYS_FREQ/TIMER_PR
#define TRIANGLE_TABLE_SIZE                16
#define UPPER_SUBSTRING_OFFSET              1
#define LOWER_SUBSTRING_OFFSET              1
#define PLUCK_OFFSET                       20 // Offset from 0 to begin triangle
                                              // wave in initial shift register
#define ECHO_LENGTH                       300
#define MAX_STRING_LENGTH                  86
#define NUMBER_OF_KEYS                     10
#define LENGTH_OF_SOUND                (FS*3)
#define DECAY_MID                       32700 // float2fix16(0.49896)
#define DECAY_LOWER                     20440 // float2fix16(0.31189)
#define DECAY_HIGHER                    31000 // float2fix16(0.47302)
#define SECOND_STRIKE                      87 // Delay until second strike
#define ECHO_AMPLITUDE_SHIFT                3 // Gives echo magnitude as
                                              // echo_mag =
                                              //  signal >> ECHO_AMPLITUDE_SHIFT
#define FIX_16_1                        65536 // int2fix16(1)
// --- End Tuning Parameters ---------------------------------------------------

// --- Key enumerations --------------------------------------------------------
#define C3                          0
#define D3                          1
#define E3                          2
#define F3                          3
#define G3                          4
#define A3                          5
#define B3                          6
#define C4                          7
#define D4                          8
#define E4                          9
// --- End key enumerations ----------------------------------------------------

// === Key data structure ======================================================
// Main data structure used to represent a piano key
typedef struct _Key {
    // Length of middle string
    unsigned char len_mid;
    // Length of string with lower frequency
    unsigned char len_lower;
    // Length of string with higher frequency
    unsigned char len_higher;

    // Flag determining if key is pressed
    unsigned char key_pressed;
    // Counter for length of song
    unsigned int  key_count;
    // Counter from initial strike, used to trigger second hammer strike
    unsigned int  strike_count;

    // Shift registers representing each string
    fix16 sr_mid[MAX_STRING_LENGTH];
    fix16 sr_lower[MAX_STRING_LENGTH];
    fix16 sr_higher[MAX_STRING_LENGTH];

    // Tuning parameters for individual strings
    fix16 tune_mid;
    fix16 tune_lower;
    fix16 tune_higher;

    // Helper variables used in low passing and all passing strings
    fix16 lowpass_out_mid;
    fix16 last_tune_out_mid;
    fix16 last_tune_in_mid;
    fix16 lowpass_out_lower;
    fix16 last_tune_out_lower;
    fix16 last_tune_in_lower;
    fix16 lowpass_out_higher;
    fix16 last_tune_out_higher;
    fix16 last_tune_in_higher;

    // Pointers for the different arrays
    unsigned char ptrin_mid;
    unsigned char ptrout_mid;
    unsigned char ptrin_lower;
    unsigned char ptrout_lower;
    unsigned char ptrin_higher;
    unsigned char ptrout_higher;
} Key;
// === End of key data structure ===============================================

// --- Global variables --------------------------------------------------------
// Array of pointers to each key
Key *Keys[NUMBER_OF_KEYS];
// Lookup table for triangle wave used in initial strike calculation
static fix16 triangle_table[TRIANGLE_TABLE_SIZE];
// Circular buffer for echo
volatile fix16 echo_sr[ECHO_LENGTH];
volatile unsigned int echo_ptr = 0;
// Initial Strike Shift Register, used for all strings, calculated once boot
static fix16 init_sr[MAX_STRING_LENGTH];
// Filter settings for low pass filter of initial strike
static fix16 a[3] = {float2fix16( 1.0   ), \
                     float2fix16(-1.9671), \
                     float2fix16( 0.9691)};
static fix16 b[3] = {float2fix16( 0.0155), \
                     float2fix16( 0.0   ), \
                     float2fix16(-0.0155)};
// Variables to keep track of which ports are high, thus which keys are pressed
volatile unsigned int keys_pressed      = 0x0000;
volatile unsigned int last_keys_pressed = 0xffff;
// Thread structures
static struct pt pt_timer, pt_music;
// --- End of global variables -------------------------------------------------

// === hitKey Function =========================================================
// Function to take in a pointer to a Key and perform necessary resets to begin
// playing the key sound
void hitKey(Key *k){
    // Reset all strings back to initial strike shift register
    memcpy(k->sr_mid,    init_sr, sizeof(init_sr));
    memcpy(k->sr_lower,  init_sr, sizeof(init_sr));
    memcpy(k->sr_higher, init_sr, sizeof(init_sr));

    k->key_pressed  = 1;
    k->key_count    = 0;
    k->strike_count = 0;

    k->lowpass_out_mid = 0;
    k->lowpass_out_lower = 0;
    k->lowpass_out_higher = 0;
    k->last_tune_in_mid = 0;
    k->last_tune_in_lower = 0;
    k->last_tune_in_higher = 0;
    k->last_tune_out_mid = 0;
    k->last_tune_out_lower = 0;
    k->last_tune_out_higher = 0;

    k->ptrin_mid = 0;
    k->ptrin_lower = 0;
    k->ptrin_higher = 0;
    k->ptrout_mid = 1;
    k->ptrout_lower = 1;
    k->ptrout_higher = 1;
}
// === End of hitKey function ==================================================

// === initKey Function ========================================================
// Key Initializer, takes the length of main string and tuning parameter
// Intended to be run once on boot for each key
// Initializes all parameters for Key struct, without setting key_pressed flag
Key *initKey(unsigned char len, float tm, float tl, float th){
    Key *out = malloc(sizeof(Key));

    out->len_mid     = len;
    out->len_lower   = len + LOWER_SUBSTRING_OFFSET;
    out->len_higher  = len - UPPER_SUBSTRING_OFFSET;
    out->tune_mid    = float2fix16(tm);
    out->tune_lower  = float2fix16(tl);
    out->tune_higher = float2fix16(th);

    //Initialized to not pressed
    out->key_pressed  = 0;
    out->key_count    = 0;
    out->strike_count = 0;

    memcpy(out->sr_mid,    init_sr, sizeof(init_sr));
    memcpy(out->sr_lower,  init_sr, sizeof(init_sr));
    memcpy(out->sr_higher, init_sr, sizeof(init_sr));

    out->lowpass_out_mid      = 0;
    out->lowpass_out_lower    = 0;
    out->lowpass_out_higher   = 0;
    out->last_tune_in_mid     = 0;
    out->last_tune_in_lower   = 0;
    out->last_tune_in_higher  = 0;
    out->last_tune_out_mid    = 0;
    out->last_tune_out_lower  = 0;
    out->last_tune_out_higher = 0;

    out->ptrin_mid     = 0;
    out->ptrin_lower   = 0;
    out->ptrin_higher  = 0;
    out->ptrout_mid    = 1;
    out->ptrout_lower  = 1;
    out->ptrout_higher = 1;

    return out;
}
// === End of initKey function =================================================

// === initShiftRegister Function ==============================================
// Initializes main shift register
// Intention is that this is calculated once on boot, then all strings can
// simply memcpy this to their arrays, which is much faster than recalculating
// on every key press.
void initShiftRegister(){
    static int i = 0;
    fix16 minVal;
    fix16 maxVal;
    static fix16 x[MAX_STRING_LENGTH];

    for (i = 0; i < ECHO_LENGTH; i++) echo_sr[i] = 0;

    // Place triangle wave in shift register
    for (i = 0; i < MAX_STRING_LENGTH; i++){
        if      (i < PLUCK_OFFSET)
            init_sr[i] = 0;
        else if (i < TRIANGLE_TABLE_SIZE + PLUCK_OFFSET)
            init_sr[i] = triangle_table[i-PLUCK_OFFSET];
        else
            init_sr[i] = 0;
    }

    memcpy(x, init_sr, sizeof(init_sr));

    // Lowpass Filter
    for(i = 2; i < MAX_STRING_LENGTH; i++){
        init_sr[i] = multfix16(b[0],      x[ i ])
                   + multfix16(b[1],      x[i-1])
                   + multfix16(b[2],      x[i-2])
                   - multfix16(a[1],init_sr[i-1])
                   - multfix16(a[2],init_sr[i-2]);
    }

    // Normalize Shift Register
    minVal = init_sr[0];
    maxVal = init_sr[0];

    for(i = 0; i < MAX_STRING_LENGTH; i++){
        if (init_sr[i] < minVal) minVal = init_sr[i];
        if (init_sr[i] > maxVal) maxVal = init_sr[i];
    }

    maxVal = max(abs(minVal), maxVal);

    for(i = 0; i < MAX_STRING_LENGTH; i++){
        init_sr[i] = multfix16(init_sr[i], divfix16(int2fix16(2047), maxVal));
    }
}
// === End of initShiftRegister function =======================================

// === initDAC Function ========================================================
// modified from Tahmid's DAC Tutorial
void initDAC(void) {
  // control CS for DAC
  mPORTBSetPinsDigitalOut(BIT_4);
  mPORTBSetBits(BIT_4);
  // use RPB5 (pin 14) for SDO2
  PPSOutput(2, RPB5, SDO2);
  //Use SPI chn 2
  SpiChnOpen(spi_chn,
             SPI_OPEN_ON | SPI_OPEN_MODE16 | SPI_OPEN_MSTEN | SPI_OPEN_CKE_REV,
             spi_clk_div);
}
// === End of initDAC function =================================================

// === writeDAC Function =======================================================
// from Tahmid's DAC Tutorial, changed to SPI2
inline void writeDAC(unsigned short data) {
  // CS low to start transaction
  mPORTBClearBits(BIT_4);
  // Wait until Transmit buffer isn't full
  while (TxBufFullSPI2());
  // write to spi2
  WriteSPI2(DAC_config_chan_A | data);
  // Wait until transmission finishes
  while (SPI2STATbits.SPIBUSY);
  // CS high
  mPORTBSetBits(BIT_4);
}
// === end of writeDAC function ================================================

// === generateTriangleTable Function ==========================================
void generateTriangleTable(void) {
    static int i;
    for (i = 0; i < TRIANGLE_TABLE_SIZE; i++){
        if (i <= TRIANGLE_TABLE_SIZE/2)
            //Ramp up triangle
            triangle_table[i] =
                    float2fix16(32768.0*(2.0/TRIANGLE_TABLE_SIZE)*i);
        else
            //Ramp back down
            triangle_table[i] = (float2fix16(32768.0*(2.0/TRIANGLE_TABLE_SIZE)
                                 * (-i+TRIANGLE_TABLE_SIZE/2))
                              + 32768.0);
    }
}
// === end of generateTriangleTable function ===================================


// === Main ISR ================================================================
// ISR triggered by Timer 2 at FS Hz
void __ISR(_TIMER_2_VECTOR, ipl2) T2Int(void) {
    fix16 value;
    static unsigned char i = 0;
    fix16 keys_down        = 0;
    value = 0;
    // Loop over all keys, checking if it is pressed or not
    for (i = 0; i < NUMBER_OF_KEYS; i++){
        if (Keys[i]->key_pressed && Keys[i]->key_count < LENGTH_OF_SOUND){
            // Add together the different strings, with the middle string
            // having the most weight.
            value = value + (Keys[i]->sr_mid[Keys[i]->ptrin_mid]       >> 1) +
                            (Keys[i]->sr_lower[Keys[i]->ptrin_lower]   >> 3) +
                            (Keys[i]->sr_higher[Keys[i]->ptrin_higher] >> 3);
            if (Keys[i]->strike_count == SECOND_STRIKE){
                memcpy(Keys[i]->sr_mid,    init_sr, sizeof(init_sr));
                memcpy(Keys[i]->sr_lower,  init_sr, sizeof(init_sr));
                memcpy(Keys[i]->sr_higher, init_sr, sizeof(init_sr));

                Keys[i]->lowpass_out_mid = 0;
                Keys[i]->lowpass_out_lower = 0;
                Keys[i]->lowpass_out_higher = 0;
                Keys[i]->last_tune_in_mid = 0;
                Keys[i]->last_tune_in_lower = 0;
                Keys[i]->last_tune_in_higher = 0;
                Keys[i]->last_tune_out_mid = 0;
                Keys[i]->last_tune_out_lower = 0;
                Keys[i]->last_tune_out_higher = 0;

                Keys[i]->ptrin_mid = 0;
                Keys[i]->ptrin_lower = 0;
                Keys[i]->ptrin_higher = 0;
                Keys[i]->ptrout_mid = 1;
                Keys[i]->ptrout_lower = 1;
                Keys[i]->ptrout_higher = 1;
            }
            if (Keys[i]->strike_count <= SECOND_STRIKE)
                Keys[i]->strike_count++;
            Keys[i]->key_count++;
            keys_down = keys_down + FIX_16_1;
        }
    }
    // Normalize the output sound
    if (keys_down > 0) value = divfix16(value, keys_down);

    writeDAC(DAC_config_chan_A |
            ((2047 // DC Offset
            + fix2int16(value + (echo_sr[echo_ptr] >> ECHO_AMPLITUDE_SHIFT)))
          & 0xfff));

    // Store value to replace old echo
    echo_sr[echo_ptr] = value;
    if (echo_ptr >= ECHO_LENGTH - 1)    echo_ptr = 0;
    else                                echo_ptr++;

    // Continue with Low pass and All pass filtering of keys pressed
    for (i = 0; i < NUMBER_OF_KEYS; i++){
        if (Keys[i]->key_pressed && Keys[i]->key_count < LENGTH_OF_SOUND){
            //low pass for string dynamics
            Keys[i]->lowpass_out_mid    =
                    multfix16((Keys[i]->sr_mid[Keys[i]->ptrin_mid] +
                               Keys[i]->sr_mid[Keys[i]->ptrout_mid]),
                              DECAY_MID);
            Keys[i]->lowpass_out_lower  =
                    multfix16((Keys[i]->sr_lower[Keys[i]->ptrin_lower] +
                               Keys[i]->sr_lower[Keys[i]->ptrout_lower]),
                              DECAY_LOWER);
            Keys[i]->lowpass_out_higher =
                    multfix16((Keys[i]->sr_higher[Keys[i]->ptrin_higher] +
                               Keys[i]->sr_higher[Keys[i]->ptrout_higher]),
                              DECAY_HIGHER);

            //All pass to tune string and feedback to shift register
            Keys[i]->sr_mid[Keys[i]->ptrin_mid]       =
                    multfix16(Keys[i]->tune_mid,
                              (Keys[i]->lowpass_out_mid -
                               Keys[i]->last_tune_out_mid)) +
                    Keys[i]->last_tune_in_mid;
            Keys[i]->sr_lower[Keys[i]->ptrin_lower]   =
                    multfix16(Keys[i]->tune_lower,
                              (Keys[i]->lowpass_out_lower -
                               Keys[i]->last_tune_out_lower)) +
                    Keys[i]->last_tune_in_lower;
            Keys[i]->sr_higher[Keys[i]->ptrin_higher] =
                    multfix16(Keys[i]->tune_higher,
                              (Keys[i]->lowpass_out_higher -
                               Keys[i]->last_tune_out_higher)) +
                    Keys[i]->last_tune_in_higher;

            //All pass state variables
            Keys[i]->last_tune_out_mid    =
                    Keys[i]->sr_mid[Keys[i]->ptrin_mid];
            Keys[i]->last_tune_in_mid     =
                    Keys[i]->lowpass_out_mid;
            Keys[i]->last_tune_out_lower  =
                    Keys[i]->sr_lower[Keys[i]->ptrin_lower];
            Keys[i]->last_tune_in_lower   =
                    Keys[i]->lowpass_out_lower;
            Keys[i]->last_tune_out_higher =
                    Keys[i]->sr_higher[Keys[i]->ptrin_higher];
            Keys[i]->last_tune_in_higher  =
                    Keys[i]->lowpass_out_higher;

            // Update pointers
            if (Keys[i]->ptrin_mid     >= Keys[i]->len_mid - 1)
                Keys[i]->ptrin_mid = 0;
            else
                Keys[i]->ptrin_mid++;
            if (Keys[i]->ptrout_mid    >= Keys[i]->len_mid - 1)
                Keys[i]->ptrout_mid = 0;
            else
                Keys[i]->ptrout_mid++;
            if (Keys[i]->ptrin_lower   >= Keys[i]->len_lower - 1)
                Keys[i]->ptrin_lower = 0;
            else
                Keys[i]->ptrin_lower++;
            if (Keys[i]->ptrout_lower  >= Keys[i]->len_lower - 1)
                Keys[i]->ptrout_lower = 0;
            else
                Keys[i]->ptrout_lower++;
            if (Keys[i]->ptrin_higher  >= Keys[i]->len_higher - 1)
                Keys[i]->ptrin_higher = 0;
            else
                Keys[i]->ptrin_higher++;
            if (Keys[i]->ptrout_higher >= Keys[i]->len_higher - 1)
                Keys[i]->ptrout_higher = 0;
            else
                Keys[i]->ptrout_higher++;
        }
    }
    mT2ClearIntFlag();
}
// === End of Main ISR =========================================================

// --- Settings for Music Thread -----------------------------------------------
//#define CHOPSTICKS     0
//#define ODE_TO_JOY     0
#define GLOVE          0
// --- End of Settings for Music Thread ----------------------------------------

// === Music Thread ============================================================
// Thread to handle the pressing of keys
static PT_THREAD(protothread_music(struct pt *pt)) {
  PT_BEGIN(pt);
#ifndef GLOVE
  static unsigned char music_i = 0;
  #define TIME_BETWEEN 300
#endif
  while (1) {
#ifdef CHOPSTICKS
    for (music_i = 0; music_i < 6; music_i++){
        hitKey(Keys[G3]);
        hitKey(Keys[F3]);
        PT_YIELD_TIME_msec(TIME_BETWEEN);
        Keys[G3]->key_pressed = 0;
        Keys[F3]->key_pressed = 0;
    }
    for (music_i = 0; music_i < 6; music_i++){
        hitKey(Keys[G3]);
        hitKey(Keys[E3]);
        PT_YIELD_TIME_msec(TIME_BETWEEN);
        Keys[G3]->key_pressed = 0;
        Keys[E3]->key_pressed = 0;
    }
    for (music_i = 0; music_i < 4; music_i++){
        hitKey(Keys[B3]);
        hitKey(Keys[D3]);
        PT_YIELD_TIME_msec(TIME_BETWEEN);
        Keys[B3]->key_pressed = 0;
        Keys[D3]->key_pressed = 0;
    }

    hitKey(Keys[A3]);
    hitKey(Keys[E3]);
    PT_YIELD_TIME_msec(TIME_BETWEEN);
    Keys[A3]->key_pressed = 0;
    Keys[E3]->key_pressed = 0;

    hitKey(Keys[B3]);
    hitKey(Keys[D3]);
    PT_YIELD_TIME_msec(TIME_BETWEEN);
    Keys[B3]->key_pressed = 0;
    Keys[D3]->key_pressed = 0;

    for (music_i = 0; music_i < 4; music_i++){
        hitKey(Keys[C3]);
        hitKey(Keys[C4]);
        PT_YIELD_TIME_msec(TIME_BETWEEN);
        Keys[C3]->key_pressed = 0;
        Keys[C4]->key_pressed = 0;
    }
    
    hitKey(Keys[B3]);
    hitKey(Keys[D3]);
    PT_YIELD_TIME_msec(TIME_BETWEEN);
    Keys[B3]->key_pressed = 0;
    Keys[D3]->key_pressed = 0;

    hitKey(Keys[A3]);
    hitKey(Keys[E3]);
    PT_YIELD_TIME_msec(TIME_BETWEEN);
    Keys[A3]->key_pressed = 0;
    Keys[E3]->key_pressed = 0;

#endif
#ifdef ODE_TO_JOY
    //E-E-F-G-G-F-E-D-C-C-D-E-E-D-D
    hitKey(Keys[E3]);
    PT_YIELD_TIME_msec(TIME_BETWEEN);
    Keys[E3]->key_pressed = 0;
    hitKey(Keys[E3]);
    PT_YIELD_TIME_msec(TIME_BETWEEN);
    Keys[E3]->key_pressed = 0;
    hitKey(Keys[F3]);
    PT_YIELD_TIME_msec(TIME_BETWEEN);
    Keys[F3]->key_pressed = 0;
    hitKey(Keys[G3]);
    PT_YIELD_TIME_msec(TIME_BETWEEN);
    Keys[G3]->key_pressed = 0;
    hitKey(Keys[G3]);
    PT_YIELD_TIME_msec(TIME_BETWEEN);
    Keys[G3]->key_pressed = 0;
    hitKey(Keys[F3]);
    PT_YIELD_TIME_msec(TIME_BETWEEN);
    Keys[F3]->key_pressed = 0;
    hitKey(Keys[E3]);
    PT_YIELD_TIME_msec(TIME_BETWEEN);
    Keys[E3]->key_pressed = 0;
    hitKey(Keys[D3]);
    PT_YIELD_TIME_msec(TIME_BETWEEN);
    Keys[D3]->key_pressed = 0;
    hitKey(Keys[C3]);
    PT_YIELD_TIME_msec(TIME_BETWEEN);
    Keys[C3]->key_pressed = 0;
    hitKey(Keys[C3]);
    PT_YIELD_TIME_msec(TIME_BETWEEN);
    Keys[C3]->key_pressed = 0;
    hitKey(Keys[D3]);
    PT_YIELD_TIME_msec(TIME_BETWEEN);
    Keys[D3]->key_pressed = 0;
    hitKey(Keys[E3]);
    PT_YIELD_TIME_msec(TIME_BETWEEN);
    Keys[E3]->key_pressed = 0;
    hitKey(Keys[E3]);
    PT_YIELD_TIME_msec(TIME_BETWEEN);
    Keys[E3]->key_pressed = 0;
    hitKey(Keys[D3]);
    PT_YIELD_TIME_msec(TIME_BETWEEN);
    Keys[D3]->key_pressed = 0;
    hitKey(Keys[D3]);
    PT_YIELD_TIME_msec(TIME_BETWEEN);
    Keys[D3]->key_pressed = 0;
    //E-E-F-G-G-F-E-D-C-C-D-E-D-C-C
    hitKey(Keys[E3]);
    PT_YIELD_TIME_msec(TIME_BETWEEN);
    Keys[E3]->key_pressed = 0;
    hitKey(Keys[E3]);
    PT_YIELD_TIME_msec(TIME_BETWEEN);
    Keys[E3]->key_pressed = 0;
    hitKey(Keys[F3]);
    PT_YIELD_TIME_msec(TIME_BETWEEN);
    Keys[F3]->key_pressed = 0;
    hitKey(Keys[G3]);
    PT_YIELD_TIME_msec(TIME_BETWEEN);
    Keys[G3]->key_pressed = 0;
    hitKey(Keys[G3]);
    PT_YIELD_TIME_msec(TIME_BETWEEN);
    Keys[G3]->key_pressed = 0;
    hitKey(Keys[F3]);
    PT_YIELD_TIME_msec(TIME_BETWEEN);
    Keys[F3]->key_pressed = 0;
    hitKey(Keys[E3]);
    PT_YIELD_TIME_msec(TIME_BETWEEN);
    Keys[E3]->key_pressed = 0;
    hitKey(Keys[D3]);
    PT_YIELD_TIME_msec(TIME_BETWEEN);
    Keys[D3]->key_pressed = 0;
    hitKey(Keys[C3]);
    PT_YIELD_TIME_msec(TIME_BETWEEN);
    Keys[C3]->key_pressed = 0;
    hitKey(Keys[C3]);
    PT_YIELD_TIME_msec(TIME_BETWEEN);
    Keys[C3]->key_pressed = 0;
    hitKey(Keys[D3]);
    PT_YIELD_TIME_msec(TIME_BETWEEN);
    Keys[D3]->key_pressed = 0;
    hitKey(Keys[E3]);
    PT_YIELD_TIME_msec(TIME_BETWEEN);
    Keys[E3]->key_pressed = 0;
    hitKey(Keys[D3]);
    PT_YIELD_TIME_msec(TIME_BETWEEN);
    Keys[D3]->key_pressed = 0;
    hitKey(Keys[C3]);
    PT_YIELD_TIME_msec(TIME_BETWEEN);
    Keys[C3]->key_pressed = 0;
    hitKey(Keys[C3]);
    PT_YIELD_TIME_msec(TIME_BETWEEN);
    Keys[C3]->key_pressed = 0;
#endif
#ifdef GLOVE
    PT_YIELD_TIME_msec(1);
    // Mapping of Ports to fingers
    /*                         1       2       3        4        7  */
    mPORTBSetPinsDigitalIn(BIT_7 | BIT_8 | BIT_9 | BIT_10 | BIT_13 );
    /*                         9      10       5        6        8*/
    mPORTASetPinsDigitalIn(BIT_0 | BIT_1 | BIT_2 |  BIT_3 |  BIT_4);
    last_keys_pressed = keys_pressed;
    keys_pressed = (mPORTBRead() & 0x7f80) | (mPORTARead() & 0x001f);
    // && !(last_keys_pressed & BIT_6
    if ((keys_pressed & BIT_7) && !(last_keys_pressed & BIT_7)){
        //key was just pressed
        hitKey(Keys[C3]);
    }
    else if (!(keys_pressed & BIT_7) && (last_keys_pressed & BIT_7)){
        //key released
        Keys[C3]->key_pressed = 0;
    }
    if ((keys_pressed & BIT_8) && !(last_keys_pressed & BIT_8)){
        //key was just pressed
        hitKey(Keys[D3]);
    }
    else if (!(keys_pressed & BIT_8) && (last_keys_pressed & BIT_8)){
        //key released
        Keys[D3]->key_pressed = 0;
    }
    if ((keys_pressed & BIT_9) && !(last_keys_pressed & BIT_9)){
        //key was just pressed
        hitKey(Keys[E3]);
    }
    else if (!(keys_pressed & BIT_9) && (last_keys_pressed & BIT_9)){
        //key released
        Keys[E3]->key_pressed = 0;
    }
    if ((keys_pressed & BIT_10) && !(last_keys_pressed & BIT_10)){
        //key was just pressed
        hitKey(Keys[F3]);
    }
    else if (!(keys_pressed & BIT_10) && (last_keys_pressed & BIT_10)){
        //key released
        Keys[F3]->key_pressed = 0;
    }
    if ((keys_pressed & BIT_2) && !(last_keys_pressed & BIT_2)){
        //key was just pressed
        hitKey(Keys[G3]);
    }
    else if (!(keys_pressed & BIT_2) && (last_keys_pressed & BIT_2)){
        //key released
        Keys[G3]->key_pressed = 0;
    }
    if ((keys_pressed & BIT_3) && !(last_keys_pressed & BIT_3)){
        //key was just pressed
        hitKey(Keys[A3]);
    }
    else if (!(keys_pressed & BIT_3) && (last_keys_pressed & BIT_3)){
        //key released
        Keys[A3]->key_pressed = 0;
    }
    if ((keys_pressed & BIT_13) && !(last_keys_pressed & BIT_13)){
        //key was just pressed
        hitKey(Keys[B3]);
    }
    else if (!(keys_pressed & BIT_13) && (last_keys_pressed & BIT_13)){
        //key released
        Keys[B3]->key_pressed = 0;
    }
    if ((keys_pressed & BIT_4) && !(last_keys_pressed & BIT_4)){
        //key was just pressed
        hitKey(Keys[C4]);
    }
    else if (!(keys_pressed & BIT_4) && (last_keys_pressed & BIT_4)){
        //key released
        Keys[C4]->key_pressed = 0;
    }
    if ((keys_pressed & BIT_0) && !(last_keys_pressed & BIT_0)){
        //key was just pressed
        hitKey(Keys[D4]);
    }
    else if (!(keys_pressed & BIT_0) && (last_keys_pressed & BIT_0)){
        //key released
        Keys[D4]->key_pressed = 0;
    }
    if ((keys_pressed & BIT_1) && !(last_keys_pressed & BIT_1)){
        //key was just pressed
        hitKey(Keys[E4]);
    }
    else if (!(keys_pressed & BIT_1) && (last_keys_pressed & BIT_1)){
        //key released
        Keys[E4]->key_pressed = 0;
    }
#endif
  } // END WHILE(1)
  PT_END(pt);
}
// === End of Music Thread =====================================================

// === Main ====================================================================
void main(void) {
  SYSTEMConfigPerformance(SYS_FREQ);

  ANSELA = 0;
  ANSELB = 0;
  CM1CON = 0;
  CM2CON = 0;

  // config threads
  PT_setup();

  generateTriangleTable();

  initDAC();

  // timer interrupt
  OpenTimer2(T2_ON | T2_PS_1_1, TIMER_PR);
  ConfigIntTimer2(T2_INT_ON | T2_INT_PRIOR_2);
  mT2ClearIntFlag(); // and clear the interrupt flag

  initShiftRegister();

  //                <String length>, <tune mid>, <tune lower>, <tune higher>
  Keys[C3] = initKey(85,              .975,       .975,         .975         );
  Keys[D3] = initKey(76,              .975,       .975,         .975         );
  Keys[E3] = initKey(67,              .300,       .300,         .300         );
  Keys[F3] = initKey(64,              .975,       .975,         .975         );
  Keys[G3] = initKey(57,              .975,       .975,         .975         );
  Keys[A3] = initKey(51,              .975,       .975,         .975         );
  Keys[B3] = initKey(45,              .500,       .500,         .500         );
  Keys[C4] = initKey(42,              .200,       .200,         .200         );
  Keys[D4] = initKey(38,              .775,       .775,         .775         );
  Keys[E4] = initKey(34,              .675,       .675,         .675         );

  // setup system wide interrupts
  INTEnableSystemMultiVectoredInt();

  // init the threads
  PT_INIT(&pt_music);

  while (1) {
    PT_SCHEDULE(protothread_music(&pt_music));
  }
}
// === End of Main =============================================================
