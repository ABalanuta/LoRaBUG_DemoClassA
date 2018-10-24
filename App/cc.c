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

char* getTimeStr(){

    t = time(NULL);
    t -= 4*3600; // -4h Eastern Time
    return asctime(localtime(&t));
}

char* getTimeStrFromSeconds(Uint32 secs){

    t = (time_t)(secs);
    t += 2208988800;
    t -= 4*3600; // -4h Eastern Time
    return asctime(localtime(&t));
}

void print_clock(){
    uartprintf("### Time(EST): %s\r", getTimeStr());
}

