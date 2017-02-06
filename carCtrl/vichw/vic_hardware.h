#ifndef VIC_HARDWARE_H
#define VIC_HARDWARE_H

/* constants */
const int DEFAULT_PWM = 150;
const int PWM_CLOCK   = 192;
const int PWM_RANGE   = 2000;


/* speed controller constants */
const int BCM_PIN_18    = 1;
const int MAX_SPEED_PWN = 200;
const int MIN_SPEED_PWM = 100;

/* servo controller constants */
const int BCM_PIN_13    = 23;
const int MAX_SERVO_PWN = 180;
const int MIN_SERVO_PWM = 110;

/* Must call before using any hardware functions */
void vichw_init(void);

#endif