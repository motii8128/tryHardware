/*
 * robomaster.c
 *
 *  Created on: May 9, 2025
 *      Author: motii
 */

#include "robomaster.h"

RoboMaster initalizeRoboMaster()
{
	RoboMaster a;
	// 各最大値を初期化
	a.m2006_current = M2006_CURRENT;
	a.m3508_current = M3508_CURRENT;
	a.m2006_max_rpm = M2006_RPM;
	a.m3508_max_rpm = M3508_RPM;

	// モーター８個分初期化する
	for(int i = 0; i < 8; i++)
	{
		// 各受信データを０で初期値する
		a.ampare[i] = 0;
		a.angle_count[i] = 0;
		//　受信が一回目であるときをわかりやすくするため−１にしてる
		a.prev_angle_count[i] = -1;
		a.revolution[i] = 0;
		a.position[i] = 0.0;
		a.rpm[i] = 0;
		a.temp[i] = 0;

		a.buf_1[i] = 0;
		a.buf_2[i] = 0;
		// PIDはめっちゃ普通な値で初期化しておく。つまりsetGainしなくても一応動く
		// 比例ゲイン：1.0、積分ゲイン：0.01、微分ゲイン：0.01
		a.pid[i] = pidInitialize(1.0, 0.01, 0.01);

		// すべてのモーターをM3508として初期化する
		// M2006より最大回転速度が小さいため
		a.motor_type[i] = M3508;
	}

	return a;
}

void setType(RoboMaster* rm, const uint8_t id, const enum MotorType m_type)
{
	// 配列の存在しない要素に触れるのを防ぐ
	if(id > 8 || id < 1)
	{
		return;
	}

	// IDを1~8で受け付けているので−１する
	rm->motor_type[id-1] = m_type;
	return;
}

void setGain(RoboMaster* rm, const uint8_t id, float p_gain, float i_gain, float d_gain)
{
	// 配列の存在しない要素に触れるのを防ぐ
	if(id > 8 || id < 1)
	{
		return;
	}
	// PID初期化関数を用いて指定通りにゲインを設定する
	rm->pid[id-1] = pidInitialize(p_gain, i_gain, d_gain);
	return;
}

void setTarget(RoboMaster* rm, const uint8_t id, const enum ControlType control_type, const float target)
{
	// 配列の存在しない要素に触れるのを防ぐ
	if(id > 8 || id < 1)
	{
		return;
	}

	// IDを1~8で受け付けているので−１する
	const uint8_t index = id-1;

	//　設定されたモーターの種類に応じて最大回転速度を取得する
	const float max_rpm = (rm->motor_type[index] == M2006) ? rm->m2006_max_rpm : rm->m3508_max_rpm;

	// 最大電流値を取得する。何回も構造体内部を参照するよりはここで定数として取得したほうが速い？かも
	const float max_current = (rm->motor_type[index] == M2006) ? rm->m2006_current : rm->m3508_current;

	// PID出力結果を格納する変数。この先、制御方法で分岐するため
	float pid_out = 0;

	if(control_type == Velocity) /* 制御方法が速度制御だった場合 */
	{
		// 目標回転速度とエンコーダの回転速度を用いて速度制御する
		// 制御周期は0.02 = 20ms
		// ローパスフィルターゲインは0.2。つまり新しい目標値：前回の目標値 = 0.2：0.8の比率で値を採用する
		pid_out = pidCompute(&rm->pid[index], target, rm->rpm[index], 0.02, 0.2);
	}
	else if(control_type == Position) /* 制御方法が位置制御だった場合 */
	{
		// 目標合計回転数と現在合計回転数の差を出す
		float delta_position = target - rm->position[index];

		// 一秒あたりの回転速度を算出する。ここでは0.02秒で目標まで到達すると仮定した
		float rps = delta_position / 0.02;

		// 一秒あたりの回転速度をRPMに直す
		float target_rpm = rps * 60;

		// 大抵の場合、最大回転速度をオーバーするので最大値に丸める
		if(target_rpm > max_rpm)target_rpm = max_rpm;
		if(target_rpm < (-1.0 * max_rpm))target_rpm = -1.0 * max_rpm;

		// 計算した目標回転速度とエンコーダの回転速度に基づいてPIDを計算する
		// 制御周期は0.02 = 20ms
		// ローパスフィルターゲインは1.0。つまり常に新しい値を採用するので、実質ローパスフィルターを使っていないのと同じになる。
		pid_out = pidCompute(&rm->pid[index], target_rpm, rm->rpm[index], 0.02, 1.0);
	}

	// PIDの結果を電流指令値に変換する
	// PIDの結果の単位はrpmなので最大回転速度で割ることで最大回転速度に対する割合を出す
	// 最大回転速度時に最大電流値であると仮定して、割合に最大電流値をかけてあげることで目標電流値としてる
	int16_t target_current = (int16_t)(pid_out * max_current / max_rpm);

	// 目標電流値が最大、最小を超えていないかチェックする
	if(target_current > max_current)target_current = max_current;
	if(target_current < (-1.0* max_current))target_current = -1.0 * max_current;

	if(id < 5) /* IDが1~4の場合 */
	{
		rm->buf_1[2*id-2] = (target_current >> 8) & 0xFF;
		rm->buf_1[2*id-1] = target_current & 0xFF;
	}
	else /* IDが5~8の場合 */
	{
		rm->buf_2[2*id-10] = (target_current >> 8) & 0xFF;
		rm->buf_2[2*id-9] = target_current & 0xFF;
	}
}

void setRecvData(RoboMaster* rm, const uint8_t id, uint8_t* buf)
{
	// 配列の存在しない要素に触れるのを防ぐ
	if(id > 8 || id < 1)
	{
		return;
	}

	// 各受信データを整理する
	int16_t angle_data = buf[0] << 8 | buf[1];
	int16_t rpm_data = buf[2] << 8 | buf[3];
	int16_t ampare_data = buf[4] << 8 | buf[5];
	int8_t temp_data = buf[6];

	// そのまま入れても良いデータを格納
	rm->angle_count[id-1] = angle_data;
	rm->rpm[id-1] = rpm_data;
	rm->ampare[id-1] = ampare_data;
	rm->temp[id-1] = temp_data;

	// これは初期化時に−１を入れているので一回目の受信時にこのifの中に入る
	if(rm->prev_angle_count[id-1] == -1)
	{
		// 即座に前回の角度を現在の角度と同じにする。エラー防止
		rm->prev_angle_count[id-1] = rm->angle_count[id-1];
	}
	// 現在の角度と前回の角度の差
	int16_t count_prop = rm->prev_angle_count[id-1] - rm->angle_count[id-1];


	if(count_prop > 4096) /* 差が半周（8192 / 2）よりも上だった場合１回転した後であると考える */
	{
		// 合計回転数に１足す
		rm->revolution[id-1] += 1;
	}
	else if(count_prop < -4096) /* 逆なら１回転戻ったと考える */
	{
		// 合計回転数を１減らす
		rm->revolution[id-1] -= 1;
	}
	// 前回の角度を今回取得した角度で更新
	rm->prev_angle_count[id-1] = rm->angle_count[id-1];

	// 現在位置を計算する
	// 何回転したか　＋　１回転にみたない回転数
	// 「１回転にみたない回転数」というのは例えばrm->angle_count[id-1]が4096なら0.5回転と判断
	// 結果、1.5回転とか124.3回転みたいな値がrm->position[id-1]には入る
	rm->position[id-1] = rm->revolution[id-1] + (float)(rm->angle_count[id-1] / 8192.0);
}

void controlRoboMaster(RoboMaster* rm, CAN_HandleTypeDef* can)
{
	// ここらへんは固定の設定
	static CAN_TxHeaderTypeDef TxHeader;
	TxHeader.StdId = 0x200;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.DLC = 8;
	TxHeader.TransmitGlobalTime = DISABLE;
	static uint32_t TxMailBox;

	// メールボックスがフリーであると確認する
	if(HAL_CAN_GetTxMailboxesFreeLevel(can) > 0)
	{
		// CAN送信(ID1~4)
		HAL_CAN_AddTxMessage(can, &TxHeader, rm->buf_1, &TxMailBox);
	}

	// 実はID5~8の送信書いてない
	TxHeader.StdId = 0x1FF;
}

/////////////////////////////// ここからPID系の関数 ///////////////////////////////

PID pidInitialize(float p_g, float i_g, float d_g)
{
	PID pid;
	pid.p_gain = p_g;
	pid.i_gain = i_g;
	pid.d_gain = d_g;
	pid.integral = 0.0;
	pid.prev_prop = 0.0;
	pid.prev_d = 0.0;

	return pid;
}

float pidCompute(PID *pid, float target, float actual, float delta_time, float lpf_alpha)
{
	// 目標値と現在値から比例量を計算する
	float prop = target - actual;

	// 積分量には比例量に時間をかけたものを追加する
	pid->integral += prop * delta_time;

	// 微分量は(今回の比例量ー前回の比例量)÷制御周期
	float derivative = (prop - pid->prev_prop) / delta_time;

	// 微分量に対してローパスフィルターを実行する
	// 微分量は急に膨大な数値が出ることがあるためローパスフィルターによって値を遅らせる
	pid->prev_d = pidLowPathFilter(lpf_alpha, derivative, pid->prev_d);

	// 積分量が大きくなりすぎるのを防ぐ
	// これをしないと誤差が蓄積しすぎたときにオーバーシュートを起こす
	pidIntegralLimit(pid, 10230, -10230);

	// 各ゲイン×量の和を計算結果とする
	float pid_out = pid->p_gain * prop + pid->i_gain * pid->integral + pid->d_gain * pid->prev_d;

	// 今回の比例量を前回の比例量として保存
	pid->prev_prop = prop;

	return pid_out;
}

void pidIntegralLimit(PID *pid, float max, float min)
{
	if(pid->integral > max)
	{
		pid->integral = max;
	}
	else if(pid->integral < min)
	{
		pid->integral = min;
	}
}

float pidLowPathFilter(float alpha, float new_rpm, float prev_rpm)
{
	return alpha * new_rpm + (1.0 - alpha) * prev_rpm;
}
