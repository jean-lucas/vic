#ifndef VIC_HARDWARE_H
#define VIC_HARDWARE_H

/* extern constants */
extern int DEFAULT_PWM_SPDC;
extern int DEFAULT_PWM_SRVO;

/* speed controller extern constants */
extern const int MAX_SPEED_PWN;
extern const int MIN_SPEED_PWM;

/* servo controller extern constants */
extern const int MAX_SERVO_PWN;
extern const int MIN_SERVO_PWM;

/* Must call before using any hardware functions */


int vichw_init(int offset);
void vichw_deinit(void);



#endif