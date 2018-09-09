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

void print_clock(){
    t = time(NULL);
    ltm = localtime(&t);
    curTime = asctime(ltm);
    uartprintf("### Time(GMT): %s\r", curTime);
}
