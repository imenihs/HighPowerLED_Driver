/*
 * File:   oid.h
 * Author: imenihs
 *
 * Created on 2024/06/07 22:43:44
 */

#ifndef __PID_H
#define __PID_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "stm32f1xx_hal.h"

	typedef int32_t fixedPoint32_t;

	// PIDコントローラの構造体
	typedef struct
	{
		fixedPoint32_t Kp;				 // 比例ゲイン
		fixedPoint32_t Ki;				 // 積分ゲイン
		fixedPoint32_t Kd;				 // 微分ゲイン
		int32_t prev_error;		 // 前回の誤差
		int32_t prev_prev_error; // 前々回の誤差
		fixedPoint32_t control_signal;	 // 現在の制御信号
		fixedPoint32_t output_min;		 // 出力の最小値
		fixedPoint32_t output_max;		 // 出力の最大値
	} PIDController_t;

	fixedPoint32_t fixed_point_mul(fixedPoint32_t a, int32_t b);
	fixedPoint32_t fixed_point_val(float a);
	void PID_Init(PIDController_t *pid, fixedPoint32_t Kp, fixedPoint32_t Ki, fixedPoint32_t Kd, fixedPoint32_t output_min, fixedPoint32_t output_max);
	fixedPoint32_t PID_Proc(PIDController_t *pid, int32_t setpoint, int32_t measured_value);

#ifdef __cplusplus
}
#endif

#endif /* __PID_H */
