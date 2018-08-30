/*
 * cc.c
 *
 * Command and Control
 *
 *  Created on: Jun 12, 2018
 *      Author: artur
 */

#include "stdbool.h"

#include "board.h"
#include "io.h"

#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>

#include <ti/sysbios/hal/Seconds.h>
#include <time.h>

#include "LoRaMac.h"


time_t t;
struct tm *ltm;
char *curTime;

bool inicialized = false;

//Seconds_getTime(Seconds_Time *ts); the Seconds_Time structure has two fields: seconds & nanoseconds. With these values you will have a better approximation to actual milliseconds:
//uint64_t var = (ts->seconds * 1000) + (ts->nsecs / 1000000);


void print_clock(){
    t = time(NULL);
    ltm = localtime(&t);
    curTime = asctime(ltm);
    uartprintf("### Time(GMT): %s\r", curTime);
}
