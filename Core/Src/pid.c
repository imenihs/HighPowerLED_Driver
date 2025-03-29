#include "stm32f1xx_hal.h"
#include "pid.h"

#define FIXED_POINT_SCALING_FACTOR 65536

// 固定小数点数の乗算
fixedPoint32_t fixed_point_mul(fixedPoint32_t a, int32_t b)
{
	return (a * b) / FIXED_POINT_SCALING_FACTOR;
}

// 固定小数点に置換
fixedPoint32_t fixed_point_val(float a)
{
	return (a * FIXED_POINT_SCALING_FACTOR);
}

void PID_Init(PIDController_t *pid, fixedPoint32_t Kp, fixedPoint32_t Ki, fixedPoint32_t Kd, fixedPoint32_t output_min, fixedPoint32_t output_max)
{
	pid->Kp = Kp;
	pid->Ki = Ki;
	pid->Kd = Kd;
	pid->prev_error = 0;
	pid->prev_prev_error = 0;
	pid->control_signal = 0;
	pid->output_min = output_min;
	pid->output_max = output_max;
}

// 速度型PID制御の計算
fixedPoint32_t PID_Proc(PIDController_t *pid, int32_t setpoint, int32_t measured_value)
{
	int32_t error = setpoint - measured_value;
	int32_t delta_error = error - pid->prev_error;
	int32_t delta_delta_error = error - 2 * pid->prev_error + pid->prev_prev_error;

	// 速度型PID制御の演算（固定小数点演算を使用）
	fixedPoint32_t delta_u = fixed_point_mul(pid->Kp, delta_error) +
							 fixed_point_mul(pid->Ki, error) +
							 fixed_point_mul(pid->Kd, delta_delta_error);

	pid->control_signal += delta_u;

	// 出力の制限
	if (pid->control_signal > pid->output_max)
	{
		pid->control_signal = pid->output_max;
	}
	else if (pid->control_signal < pid->output_min)
	{
		pid->control_signal = pid->output_min;
	}

	// 過去のエラーを更新
	pid->prev_prev_error = pid->prev_error;
	pid->prev_error = error;

	return pid->control_signal;
}
