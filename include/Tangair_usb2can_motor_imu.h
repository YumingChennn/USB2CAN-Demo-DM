// # Copyright (c) 2023-2025 TANGAIR 
// # SPDX-License-Identifier: Apache-2.0
#ifndef Tangair_usb2can_motor_imu_H
#define Tangair_usb2can_motor_imu_H

#include <assert.h>
#include <chrono>
#include <cmath>
#include <iostream>
#include <stdio.h>
#include <signal.h>
#include <string>
#include <sys/mman.h>
#include <sys/timerfd.h>
#include <thread>
#include <sched.h>
#include <unistd.h>
#include <atomic>
#include "usb_can.h"
#include <vector>

#include <math.h>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/idl/go2/LowCmd_.hpp>
#include <unitree/common/time/time_tool.hpp>
#include <unitree/common/thread/thread.hpp>


// 辅助函数
std::vector<std::vector<double>> mujoco_ang2real_ang(const std::vector<double>& dof_pos);

float uint_to_float(int x_int, float x_min, float x_max, int bits);
int float_to_uint(float x, float x_min, float x_max, int bits);
uint32_t crc32_core(uint32_t *ptr, uint32_t len);

//达妙其他电机参数    {P, V, T}
				// {12.5, 30, 10 }, // DM4310
				// {12.5, 50, 10 }, // DM4310_48V
				// {12.5, 8, 28 },  // DM4340
				// {12.5, 10, 28 }, // DM4340_48V
				// {12.5, 45, 20 }, // DM6006
				// {12.5, 45, 40 }, // DM8006
				// {12.5, 45, 54 }, // DM8009
				// {12.5,25,  200}, // DM10010L
				// {12.5,20, 200},  // DM10010
				// {12.5,280,1},    // DMH3510
				// {12.5,45,10},    // DMH6215
				// {12.5,45,10}     // DMG6220

// 达妙电机,此处为DM10010L参数
#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -30.0f
#define V_MAX 30.0f
#define T_MIN -10.0f
#define T_MAX 10.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f

using namespace unitree::common;
using namespace unitree::robot;

#define TOPIC_LOWCMD "rt/lowcmd"
#define TOPIC_LOWSTATE "rt/lowstate"

constexpr double PosStopF = (2.146E+9f);
constexpr double VelStopF = (16000.0f);


#define PI (3.1415926f)

typedef struct
{
	uint16_t id;


	float position;
	float speed;
	float kp;
	float kd;
	float torque;

	//位置限制
	float max_position;
	float min_position;

} Motor_CAN_Send_Struct;

typedef struct
{
	uint16_t ERR;

	uint16_t current_position; //
	uint16_t current_speed;	   //
	uint16_t current_torque;   //
	uint16_t current_temp_MOS;	   //
	uint16_t current_temp_Rotor;	   //


	float current_position_f;//
	float current_speed_f;//
	float current_torque_f;//

} Motor_CAN_Recieve_Struct;

typedef struct
{
	Motor_CAN_Send_Struct ID_1_motor_send, ID_2_motor_send, ID_3_motor_send, ID_5_motor_send, ID_6_motor_send, ID_7_motor_send;
	Motor_CAN_Recieve_Struct ID_1_motor_recieve, ID_2_motor_recieve, ID_3_motor_recieve, ID_5_motor_recieve, ID_6_motor_recieve, ID_7_motor_recieve;

} USB2CAN_CAN_Bus_Struct;



class Tangair_usb2can
{
public:
	
	std::atomic<bool> all_thread_done_{false};
	std::atomic<bool> running_{false};
	

	Tangair_usb2can();
	~Tangair_usb2can();
	void Init();
  
	// CAN设备0
	int USB2CAN0_;

	// CAN设备1
	int USB2CAN1_;


    int can_dev0_rx_count;
	int can_dev0_rx_count_thread;
	int can_dev1_rx_count;
	int can_dev1_rx_count_thread;

	void StartPositionLoop();
	void StopAllThreads();
	
	void SetMotorTarget(Motor_CAN_Send_Struct &motor, double pos, double kp = 3.0, double kd = 0.1);
	void SetTargetPosition(const std::array<std::array<double, 4>, 3> &positions, double kp = 3.0, double kd = 0.1);
	void ResetPositionToZero(); // 設定位置

	std::thread _CAN_TX_test_thread;
	void CAN_TX_position_sin_thread();  // 不要帶參數

	std::thread _CAN_RX_device_0_thread;
	void CAN_RX_device_0_thread();

	std::thread _CAN_RX_device_1_thread;
	void CAN_RX_device_1_thread();


	/*****************************************************************************************************/
	/*********************************       ***电机相关***      ***********************************************/
	/*****************************************************************************************************/
	// 电机基本操作变量
	FrameInfo txMsg_CAN = {
		.canID = 0,
		.frameType = STANDARD,
		.dataLength = 8,
	};

	uint8_t Data_CAN[8];
	Motor_CAN_Send_Struct Motor_Data_Single;

	//接收暂存
	Motor_CAN_Recieve_Struct CAN_DEV0_RX;
	Motor_CAN_Recieve_Struct CAN_DEV1_RX;


	// 腿部数据
	USB2CAN_CAN_Bus_Struct USB2CAN0_CAN_Bus_1; // 模块0，can1
	USB2CAN_CAN_Bus_Struct USB2CAN0_CAN_Bus_2;  // 模块0，can2
	USB2CAN_CAN_Bus_Struct USB2CAN1_CAN_Bus_1;  // 模块1，can1
	USB2CAN_CAN_Bus_Struct USB2CAN1_CAN_Bus_2;	  // 模块1，can2



	//初始化
	void USB2CAN_CAN_Bus_inti_set(USB2CAN_CAN_Bus_Struct* Leg_Data);

	void USB2CAN_CAN_Bus_Init();

	void Motor_Enable(int32_t dev, uint8_t channel, Motor_CAN_Send_Struct *Motor_Data);

	void Motor_Disable(int32_t dev, uint8_t channel, Motor_CAN_Send_Struct *Motor_Data);

	void Motor_Zore(int32_t dev, uint8_t channel, Motor_CAN_Send_Struct *Motor_Data);

	void CAN_Send_Control(int32_t dev, uint8_t channel, Motor_CAN_Send_Struct *Motor_Data); // 运控�??,CAN1=CAN_TX_MAILBOX0,CAN2=CAN_TX_MAILBOX1

	void Motor_Passive_SET(int32_t dev, uint8_t channel, Motor_CAN_Send_Struct *Motor_Data);

	void ENABLE_ALL_MOTOR(int delay_us);

	void DISABLE_ALL_MOTOR(int delay_us);

	void ZERO_ALL_MOTOR(int delay_us);

	void PASSIVE_ALL_MOTOR(int delay_us);

	void CAN_TX_ALL_MOTOR(int delay_us);

	void InitLowCmd();
    
	void LowCmdMessageHandler(const void *messages);
    
	void LowCmdWrite();
	
private:
	std::array<std::array<double, 4>, 3> target_pos;
	std::array<std::array<double, 4>, 3> real_angles_;
	std::vector<double> dof_pos;

	double stand_up_joint_pos[12] = {0.00571868, 0.608813, -1.21763, -0.00571868, 0.608813, -1.21763,
                                     0.00571868, 0.608813, -1.21763, -0.00571868, 0.608813, -1.21763};
    double stand_down_joint_pos[12] = {0.0473455, 1.22187, -2.44375, -0.0473455, 1.22187, -2.44375, 0.0473455,
                                       1.22187, -2.44375, -0.0473455, 1.22187, -2.44375};
    double dt = 0.002;
    double runing_time = 0.0;
    double phase = 0.0;

    unitree_go::msg::dds_::LowCmd_ low_cmd{};     // default init
    unitree_go::msg::dds_::LowState_ low_state{}; // default init

    /*publisher*/
    ChannelPublisherPtr<unitree_go::msg::dds_::LowCmd_> lowcmd_publisher;
    /*subscriber*/
    ChannelSubscriberPtr<unitree_go::msg::dds_::LowCmd_> lowcmd_subscriber;

    /*LowCmd write thread*/
    ThreadPtr lowCmdWriteThreadPtr;
};

#endif

// # Copyright (c) 2023-2025 TANGAIR 
// # SPDX-License-Identifier: Apache-2.0
