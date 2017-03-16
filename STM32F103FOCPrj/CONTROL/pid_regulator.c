#include "pid_regulator.h"
void PID_Calc(PID_Regulator_t *pid)
{
	pid->err[0] = pid->ref - pid->fdb;
	pid->output = pid->kp * pid->err[0] + pid->kd * (pid->err[0] - pid->err[1]);
	if(pid->output > pid->outputMax)  pid->output = pid->outputMax;
}
