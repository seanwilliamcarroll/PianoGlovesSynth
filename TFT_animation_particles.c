/*
 * File:        TFT_test_BRL4.c
 * Author:      Bruce Land
 * Adapted from:
 *              main.c by
 * Author:      Syed Tahmid Mahbub
 * Target PIC:  PIC32MX250F128B
 */

////////////////////////////////////
// clock AND protoThreads configure!
// You MUST check this file!
#include "config.h"
// threading library
#include "pt_cornell_1_2.h"

////////////////////////////////////
// graphics libraries
#include "tft_master.h"
#include "tft_gfx.h"
// need for rand function
#include <stdlib.h>
////////////////////////////////////


/* Demo code for interfacing TFT (ILI9340 controller) to PIC32
 * The library has been modified from a similar Adafruit library
 */
// Adafruit data:
/***************************************************
  This is an example sketch for the Adafruit 2.2" SPI display.
  This library works with the Adafruit 2.2" TFT Breakout w/SD card
  ----> http://www.adafruit.com/products/1480

  Check out the links above for our tutorials and wiring diagrams
  These displays use SPI to communicate, 4 or 5 pins are required to
  interface (RST is optional)
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  MIT license, all text above must be included in any redistribution
 ****************************************************/

// string buffer
char buffer[60];
int pt_start_time, pt_end_time ;
// === thread structures ============================================
// thread control structs
// note that UART input and output are threads
static struct pt pt_timer, pt_color, pt_anim ;

// system 1 second interval tick
int sys_time_seconds ;

// === the fixed point macros ========================================
typedef signed int fix16 ;
#define multfix16(a,b) ((fix16)(((( signed long long)(a))*(( signed long long)(b)))>>16)) //multiply two fixed 16:16
#define float2fix16(a) ((fix16)((a)*65536.0)) // 2^16
#define fix2float16(a) ((float)(a)/65536.0)
#define fix2int16(a)    ((int)((a)>>16))
#define int2fix16(a)    ((fix16)((a)<<16))
#define divfix16(a,b) ((fix16)((((signed long long)(a)<<16)/(b)))) 
#define sqrtfix16(a) (float2fix16(sqrt(fix2float16(a)))) 
#define absfix16(a) abs(a)

// === Timer Thread =================================================
// update a 1 second tick counter
static PT_THREAD (protothread_timer(struct pt *pt))
{
    PT_BEGIN(pt);
     tft_setCursor(0, 0);
     tft_setTextColor(ILI9340_WHITE);  tft_setTextSize(1);
     tft_writeString("Time milliseconds for animation\n");
      while(1) {
        // yield time 1 second
        PT_YIELD_TIME_msec(1000) ;
        sys_time_seconds++ ;
        
        // draw sys_time
        tft_fillRoundRect(0,10, 100, 14, 1, ILI9340_BLACK);// x,y,w,h,radius,color
        tft_setCursor(0, 10);
        tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(2);
        sprintf(buffer,"%d", pt_end_time-pt_start_time);
        tft_writeString(buffer);
        // NEVER exit while
      } // END WHILE(1)
  PT_END(pt);
} // timer thread

/* === Color Thread =================================================
// draw 3 color patches for R,G,B from a random number
static int color ;
static int i;
static PT_THREAD (protothread_color(struct pt *pt))
{
    PT_BEGIN(pt);
      while(1) {
        // yield time 1 second
        PT_YIELD_TIME_msec(2000) ;

        // choose a random color
        color = rand() & 0xffff ;
       
        // draw color string
        tft_fillRoundRect(0,50, 150, 14, 1, ILI9340_BLACK);// x,y,w,h,radius,color
        tft_setCursor(0, 50);
        tft_setTextColor(ILI9340_WHITE); tft_setTextSize(1);
        sprintf(buffer," %04x  %04x  %04x  %04x", color & 0x1f, color & 0x7e0, color & 0xf800, color);
        tft_writeString(buffer);

        // draw the actual color patches
        tft_fillRoundRect(5,70, 30, 30, 1, color & 0x1f);// x,y,w,h,radius,blues
        tft_fillRoundRect(40,70, 30, 30, 1, color & 0x7e0);// x,y,w,h,radius,greens
        tft_fillRoundRect(75,70, 30, 30, 1, color & 0xf800);// x,y,w,h,radius,reds
        // now draw the RGB mixed color
        tft_fillRoundRect(110,70, 30, 30, 1, color);// x,y,w,h,radius,mix color
        // NEVER exit while
      } // END WHILE(1)
  PT_END(pt);
} // color thread
*/
// === Animation Thread =============================================
// update a 1 second tick counter
#define N 500
static fix16 xc[N], yc[N], vxc[N], vyc[N];
static fix16 g = float2fix16(0.1), drag = float2fix16(.001);

static PT_THREAD (protothread_anim(struct pt *pt))
{
    PT_BEGIN(pt);
    static int i;
    // init  speeds
    for (i=0; i<N; i++){
        xc[i]=int2fix16(10);
        yc[i]=int2fix16(50);
        vxc[i] = int2fix16(4) + rand()>>13;
        vyc[i] = float2fix16(.1) + rand()>>14;
    }  
    
    while(1) {
        // yield time 1 second
        PT_YIELD_TIME_msec(2);
        pt_start_time = PT_GET_TIME();
        for (i=0; i<N; i++){
            // erase disk
             //tft_fillCircle(fix2int16(xc[i]), fix2int16(yc[i]), 2, ILI9340_BLACK); //x, y, radius, color
             tft_drawPixel(fix2int16(xc[i]), fix2int16(yc[i]), ILI9340_BLACK); //x, y, , color
             tft_drawPixel(fix2int16(xc[i])+1, fix2int16(yc[i]), ILI9340_BLACK); //x, y, , color
             tft_drawPixel(fix2int16(xc[i]-1), fix2int16(yc[i]), ILI9340_BLACK); //x, y, , color
             tft_drawPixel(fix2int16(xc[i]), fix2int16(yc[i]+1), ILI9340_BLACK); //x, y, , color
             tft_drawPixel(fix2int16(xc[i]), fix2int16(yc[i]-1), ILI9340_BLACK); //x, y, , color
            // compute new velocities
             vyc[i] = vyc[i] - g - multfix16(vyc[i], drag) ;
             vxc[i] = vxc[i] - multfix16(vxc[i], drag);

             xc[i] = xc[i] + vxc[i];
             yc[i] = yc[i] - vyc[i];

             if (xc[i]<int2fix16(5) || xc[i]>int2fix16(235)) vxc[i] = -vxc[i]; 
             if (yc[i]>int2fix16(315)) vyc[i] = -vyc[i]; 

             //  draw disk
             //tft_fillCircle(fix2int16(xc[i]), fix2int16(yc[i]), 2, ILI9340_GREEN); //x, y, radius, color
             tft_drawPixel(fix2int16(xc[i]), fix2int16(yc[i]), ILI9340_GREEN); //x, y, , color
             tft_drawPixel(fix2int16(xc[i])+1, fix2int16(yc[i]), ILI9340_GREEN); //x, y, , color
             tft_drawPixel(fix2int16(xc[i]-1), fix2int16(yc[i]), ILI9340_GREEN); //x, y, , color
             tft_drawPixel(fix2int16(xc[i]), fix2int16(yc[i]+1), ILI9340_GREEN); //x, y, , color
             tft_drawPixel(fix2int16(xc[i]), fix2int16(yc[i]-1), ILI9340_GREEN); //x, y, , color
        }
        pt_end_time = PT_GET_TIME();
        // NEVER exit while
      } // END WHILE(1)
  PT_END(pt);
} // animation thread

// === Main  ======================================================
void main(void) {
 //SYSTEMConfigPerformance(PBCLK);
  
  ANSELA = 0; ANSELB = 0; 

  // === config threads ==========
  // turns OFF UART support and debugger pin, unless defines are set
  PT_setup();

  // === setup system wide interrupts  ========
  INTEnableSystemMultiVectoredInt();

  // init the threads
  PT_INIT(&pt_timer);
  PT_INIT(&pt_color);
  PT_INIT(&pt_anim);

  // init the display
  tft_init_hw();
  tft_begin();
  tft_fillScreen(ILI9340_BLACK);
  //240x320 vertical display
  tft_setRotation(0); // Use tft_setRotation(1) for 320x240

  // seed random color
  srand(1);

  // round-robin scheduler for threads
  while (1){
      PT_SCHEDULE(protothread_timer(&pt_timer));
      //PT_SCHEDULE(protothread_color(&pt_color));
      PT_SCHEDULE(protothread_anim(&pt_anim));
      }
  } // main

// === end  ======================================================

