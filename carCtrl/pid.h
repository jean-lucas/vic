#ifndef PID_H
#define PID_H

struct pid_context
{
	double kp;
	double ki;
	double kd;
	double bias;
	double sp;
	double integral;
	double max;
	double min;
	double oldPV;
	int firstRun;
};



/* mv = Kp *(e + 1/Ti*integral(e) + Td * derivative(e)) Sample time in seconds*/
void pid_tune(struct pid_context *ctx, double kp, double Ti, double Td, double bias, double sample_time);
void pid_set_clipping(struct pid_context *ctx, double max, double min);
void pid_set(struct pid_context *ctx, double sp);
double pid_update(struct pid_context *ctx, double pv);
#endif