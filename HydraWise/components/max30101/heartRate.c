/*
 Optical Heart Rate Detection (PBA Algorithm)
 By: Nathan Seidle
 SparkFun Electronics
 Date: October 2nd, 2016
 
 Given a series of IR samples from the MAX30105 we discern when a heart beat is occurring

 Let's have a brief chat about what this code does. We're going to try to detect
 heart-rate optically. This is tricky and prone to give false readings. We really don't
 want to get anyone hurt so use this code only as an example of how to process optical
 data. Build fun stuff with our MAX30105 breakout board but don't use it for actual
 medical diagnosis.

 Excellent background on optical heart rate detection:
 http://www.ti.com/lit/an/slaa655/slaa655.pdf

 Good reading:
 http://www.techforfuture.nl/fjc_documents/mitrabaratchi-measuringheartratewithopticalsensor.pdf
 https://fruct.org/publications/fruct13/files/Lau.pdf

 This is an implementation of Maxim's PBA (Penpheral Beat Amplitude) algorithm. It's been 
 converted to work within the Arduino framework.
*/

/* Copyright (C) 2016 Maxim Integrated Products, Inc., All Rights Reserved.
*
* Permission is hereby granted, free of charge, to any person obtaining a
* copy of this software and associated documentation files (the "Software"),
* to deal in the Software without restriction, including without limitation
* the rights to use, copy, modify, merge, publish, distribute, sublicense,
* and/or sell copies of the Software, and to permit persons to whom the
* Software is furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included
* in all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
* OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
* IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
* OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
* ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
* OTHER DEALINGS IN THE SOFTWARE.
*
* Except as contained in this notice, the name of Maxim Integrated
* Products, Inc. shall not be used except as stated in the Maxim Integrated
* Products, Inc. Branding Policy.
*
* The mere transfer of this software does not imply any licenses
* of trade secrets, proprietary technology, copyrights, patents,
* trademarks, maskwork rights, or any other form of intellectual
* property whatsoever. Maxim Integrated Products, Inc. retains all
* ownership rights.
* 
*/

// Code based on Library above (from CPP to C)

#include "heartRate.h"
#include <stdint.h>

// Variables for beat detection
int16_t IR_AC_Max = 20;
int16_t IR_AC_Min = -20;

int16_t IR_AC_Signal_Current = 0;
int16_t IR_AC_Signal_Previous;
int16_t IR_AC_Signal_min = 0;
int16_t IR_AC_Signal_max = 0;
int16_t IR_Average_Estimated;

int16_t positiveEdge = 0;
int16_t negativeEdge = 0;
int32_t ir_avg_reg = 0;

int16_t cbuf[32];
uint8_t offset = 0;

// FIR filter coefficients
static const uint16_t FIRCoeffs[12] = {172, 321, 579, 927, 1360, 1858, 2390, 2916, 3391, 3768, 4012, 4096};

// Function prototypes
int16_t averageDCEstimator(int32_t *p, uint16_t x);
int16_t lowPassFIRFilter(int16_t din);
int32_t mul16(int16_t x, int16_t y);

// Heart rate detection function
// Returns 1 if a beat is detected, 0 otherwise
int checkForBeat(int32_t sample) {
    int beatDetected = 0;

    // Save current state
    IR_AC_Signal_Previous = IR_AC_Signal_Current;
  
    // Process next data sample
    IR_Average_Estimated = averageDCEstimator(&ir_avg_reg, (uint16_t)sample);
    IR_AC_Signal_Current = lowPassFIRFilter((int16_t)(sample - IR_Average_Estimated));

    // Detect positive zero crossing (rising edge)
    if ((IR_AC_Signal_Previous < 0) && (IR_AC_Signal_Current >= 0)) {
        IR_AC_Max = IR_AC_Signal_max;
        IR_AC_Min = IR_AC_Signal_min;

        positiveEdge = 1;
        negativeEdge = 0;
        IR_AC_Signal_max = 0;

        if (((IR_AC_Max - IR_AC_Min) > 20) && ((IR_AC_Max - IR_AC_Min) < 1000)) {
            // Heartbeat detected!
            beatDetected = 1;
        }
    }

    // Detect negative zero crossing (falling edge)
    if ((IR_AC_Signal_Previous > 0) && (IR_AC_Signal_Current <= 0)) {
        positiveEdge = 0;
        negativeEdge = 1;
        IR_AC_Signal_min = 0;
    }

    // Find maximum value in positive cycle
    if (positiveEdge && (IR_AC_Signal_Current > IR_AC_Signal_Previous)) {
        IR_AC_Signal_max = IR_AC_Signal_Current;
    }

    // Find minimum value in negative cycle
    if (negativeEdge && (IR_AC_Signal_Current < IR_AC_Signal_Previous)) {
        IR_AC_Signal_min = IR_AC_Signal_Current;
    }

    return beatDetected;
}

// Average DC Estimator
int16_t averageDCEstimator(int32_t *p, uint16_t x) {
    *p += ((((int32_t)x << 15) - *p) >> 4);
    return (int16_t)(*p >> 15);
}

// Low Pass FIR Filter
int16_t lowPassFIRFilter(int16_t din) {  
    cbuf[offset] = din;
    int32_t z = mul16(FIRCoeffs[11], cbuf[(offset - 11) & 0x1F]);

    for (uint8_t i = 0; i < 11; i++) {
        z += mul16(FIRCoeffs[i], cbuf[(offset - i) & 0x1F] + cbuf[(offset - 22 + i) & 0x1F]);
    }

    offset++;
    offset %= 32; // Wrap condition

    return (int16_t)(z >> 15);
}

// Integer multiplier function
int32_t mul16(int16_t x, int16_t y) {
    return ((int32_t)x * (int32_t)y);
}
