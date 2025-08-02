/*
 * robomaster.hpp
 *
 *  Created on: May 9, 2025
 *      Author: motii
 */

#ifndef INC_ROBOMASTER_H_
#define INC_ROBOMASTER_H_

#include <stdint.h>
#include <math.h>
#include "stm32l4xx_hal.h"

// M2006の最大電流指令
#define M2006_CURRENT 10000
// M3508の最大電流指令
#define M3508_CURRENT 16000

// M2006の最大回転数
#define M2006_RPM 13000

// M3508の最大回転数
#define M3508_RPM 9000

// 速度制御か位置制御か選ぶときに使う列挙型
enum ControlType
{
	Position,
	Velocity
};

// M2006かM3508か選ぶときに使う列挙型
enum MotorType
{
	M3508,
	M2006
};

// PIDを扱う構造体
typedef struct PID
{
	float p_gain; /* 比例 ゲイン */
	float i_gain; /* 積分ゲイン */
	float d_gain; /* 微分ゲイン */
	float integral; /* 積分量を格納する */
	float prev_prop; /* 前回の比例量を格納する */
	float prev_d; /* 前回の微分量を格納する */
}PID;

// ロボマス構造体本体
typedef struct
{
	uint8_t buf_1[8]; /* 送信データ ID1~4 */
	uint8_t buf_2[8]; /* 送信データ ID5~8 */

	// 受信データ
	int16_t angle_count[8]; /* ロボマスの現在角度(0~8192) */
	int16_t prev_angle_count[8]; /* 前回のロボマスの角度(0~8192) */
	int16_t revolution[8]; /* 合計で何回転したか */
	float position[8]; /* 実際に使う位置情報。例：540度なら1.5回転なのでここに1.5が入る */
	int16_t rpm[8]; /* ロボマスの速度 */
	int16_t ampare[8]; /* ロボマスの電流 */
	int8_t temp[8]; /* ロボマスの温度 */

	PID pid[8]; /* PID構造体 */
	enum MotorType motor_type[8]; /* ロボマスの種類 */

	float m2006_current; /* M2006の最大電流値。初期化時にマクロから取得する */
	float m3508_current; /* M3508の最大電流値。初期化時にマクロから取得する */
	float m2006_max_rpm; /* M2006の最大回転速度。初期化時にマクロから取得する */
	float m3508_max_rpm; /* M3508の最大回転速度。初期化時にマクロから取得する */
}RoboMaster;

// ロボマス構造体を初期化する
// 引数：なし
// 返り値：ロボマス構造体
RoboMaster initalizeRoboMaster();

// 各IDにモーターの種類を登録する
// 引数：（ロボマス構造体ポインタ、モーターのCANのID(1~8)、モーターの種類）
void setType(RoboMaster* rm, const uint8_t id, const enum MotorType m_type);

// 各IDにPIDのゲインを設定する
// 引数：（ロボマス構造体ポインタ、モーターのCANのID(1~8)、比例ゲイン、積分ゲイン、微分ゲイン）
void setGain(RoboMaster* rm, const uint8_t id, float p_gain, float i_gain, float d_gain);

// 各IDに目的速度、もしくは目的位置を設定する（PID計算を実行するのはここ）
// 引数：（ロボマス構造体ポインタ、モーターのCANのID(1~8)、制御方法（速度or位置）、目標値（速度ならRPMを位置なら合計回転数））
void setTarget(RoboMaster* rm, const uint8_t id, const enum ControlType control_type, const float target);

// CANの受信コールバック関数内で使う
// 引数：（ロボマス構造体ポインタ、モーターのCANのID(1~8)、受信データ）
void setRecvData(RoboMaster* rm, const uint8_t id, uint8_t* rx_buf);

// これまでの設定に基づきCAN通信を実行しモーターをコントロールする
// 引数：（ロボマス構造体ポインタ、CAN通信用構造体(hcan1的なやつ)）
void controlRoboMaster(RoboMaster* rm, CAN_HandleTypeDef* can);

// PID構造体の初期化を行う
// 引数：（比例ゲイン、積分ゲイン、微分ゲイン）
PID pidInitialize(float p_g, float i_g, float d_g);

// PID計算を行う
// 引数：（PID構造体ポインタ、目標値、現在値、制御周期[s]、ローパスフィルターのゲイン）
// 返り値：PID計算結果。この結果は電流値ではないのでCAN通信の送信内容ではない。
float pidCompute(PID *pid, float target, float actual, float delta_time, float lpf_alpha);

// PID計算における積分量に限界を設ける。最大値もしくは最小値を超えた場合に値を丸める。
// 引数：（PID構造体ポインタ、最大値、最小値）
void pidIntegralLimit(PID *pid, float max, float min);

// ローパスフィルター（一次遅れフィルタ）
// 引数：（ゲイン、新しい目標値、前回の目標値）
// 返り値：フィルタした値
float pidLowPathFilter(float alpha, float new_rpm, float prev_rpm);

#endif /* INC_ROBOMASTER_H_ */
