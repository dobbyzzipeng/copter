#ifndef _RAMP_H_
#define _RAMP_H_
#include "stdint.h"
#include "stm32f10x_tim.h"
/*                 __                             _______        _________
*description       ||                             |             /
*                  ||    ->  __                   |      ->    /
*              ____||____  __||__    while   _____|        ___/
*    by zzipeng@2017/02/16
*/
#define SLOP 1000
typedef struct RampGen_t
{
	int32_t thisinput;
	int32_t lastoutput;
	int32_t thistime;
	int32_t lasttime;
	TIM_TypeDef * TIMx;
	float slop;
	int32_t count;
	int32_t XSCALE;
	float out;
	void (*Init)(struct RampGen_t *ramp, int32_t XSCALE);
	float (*Calc)(struct RampGen_t *ramp);
	void (*SetCounter)(struct RampGen_t *ramp, int32_t count);
	void (*ResetCounter)(struct RampGen_t *ramp);
	void (*SetScale)(struct RampGen_t *ramp, int32_t scale);
	uint8_t (*IsOverflow)(struct RampGen_t *ramp);
	int32_t (*RamptimeGet)(void);
}RampGen_t;

#define RAMP_GEN_DAFAULT \
{ \
							.thisinput = 0, \
							.lastoutput = 0, \
							.thistime = 0, \
							.lasttime = 0, \
							.TIMx = TIM5, \
							.slop = SLOP, \
							.count = 0, \
							.XSCALE = 0, \
							.out = 0, \
							.Init = &RampInit, \
							.Calc = &RampCalc, \
							.SetCounter = &RampSetCounter, \
							.ResetCounter = &RampResetCounter, \
							.SetScale = &RampSetScale, \
							.IsOverflow = &RampIsOverflow, \
						} \

						
void RampInit(RampGen_t *ramp, int32_t XSCALE);
float RampCalc(RampGen_t *ramp);
void RampSetCounter(struct RampGen_t *ramp, int32_t count);
void RampResetCounter(struct RampGen_t *ramp);
void RampSetScale(struct RampGen_t *ramp, int32_t scale);
uint8_t RampIsOverflow(struct RampGen_t *ramp);
int32_t RamptimeGet(RampGen_t *ramp);
#endif
