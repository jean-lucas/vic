#include "pid.h"
// #include "stdio.h"
/* mv = Kp *(e + 1/Ti*integral(e) + Td * derivative(e)) Sample time in seconds*/
void pid_tune(struct pid_context *ctx, double kp, double Ti, double Td, double bias, double sample_time)
{
	ctx->bias = bias;
	ctx->kp = kp;
	ctx->ki = kp * sample_time / Ti;
	ctx->kd = kp * Td / sample_time;
	ctx->bias = bias;
	ctx->oldPV = 0;
}

void pid_set_clipping(struct pid_context *ctx, double max, double min)
{
	ctx->max = max;
	ctx->min = min;
}

void pid_set(struct pid_context *ctx, double sp)
{
	ctx->sp = sp;
	ctx->integral = 0;
	ctx->firstRun = 1;
}

double pid_update(struct pid_context *ctx, double pv)
{
	double error = ctx->sp - pv;
	double dpv;
	double output;

	if (ctx->firstRun) {
		ctx->oldPV = pv;
		ctx->firstRun = 0;
	}
	dpv = pv - ctx->oldPV;

	ctx->integral += ctx->ki * error;
	if (ctx->integral > ctx->max) ctx->integral = ctx->max;
	// if (ctx->integral < ctx->min) ctx->integral = ctx->min;

	output = ctx->kp * error + ctx->integral - ctx->kd * dpv + ctx->bias;
	if (output > ctx->max) output = ctx->max;
	if (output < ctx->min) output = ctx->min;
	ctx->oldPV = pv;
	return output;
}