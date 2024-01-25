#include <ch.h>
#include <hal.h>
#include <chprintf.h>
#include <memory_protection.h>
#include <sensors/VL53L0X/VL53L0X.h>
#include <motors.h>
#include <selector.h>
#include <sensors/imu.h>
#include "main.h"
#include "TOF.h"
#include "roues.h"
#include "IMUmesure.h"

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

/* DEBUT Main */

int main(void)
{   
    /* Initialisations */
    halInit();
    chSysInit();
    mpu_init();
    motors_init();
    messagebus_init(&bus, &bus_lock, &bus_condvar);
    VL53L0X_start();
    imu_start();

    /* Boucle d'attente pour commencer (selecteur en position 9), attend 2 secondes avant de commencer */
    int sel = 0;
    while(sel != 9){
        sel = get_selector();
    }
    chThdSleepMilliseconds(2000);

    /* Lancement des thread et du calibrage du TOF */
    TOF_calibrage();
    ThreadMoteur_start();
    ThreadTOF_start();
    ThreadIMU_start();

    while (1) {
        chThdSleepMilliseconds(1000);
    }
}

/* FIN Main */

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}

