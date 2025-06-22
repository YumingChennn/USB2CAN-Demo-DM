// # Copyright (c) 2023-2025 TANGAIR 
// # SPDX-License-Identifier: Apache-2.0
#include "Tangair_usb2can_motor_imu.h"
#include <chrono>
#include <cmath>
#include <array> 
#include <iostream>
#include <algorithm> // for std::find
#include <stdexcept> // for std::runtime_error
using namespace std::chrono;


/// @brief 构造函数，初始化
/// @return
Tangair_usb2can::Tangair_usb2can() 
{
    std::cout << "begin " << running_.load();

    USB2CAN0_ = openUSBCAN("/dev/ttyRedDog");
    if (USB2CAN0_ == -1)
        std::cout << std::endl
                  << "ttyRedDog open INcorrect!!!" << std::endl;
    else
        std::cout << std::endl
                  << "ttyRedDog opened ,num=" << USB2CAN0_ << std::endl;

    // 电机ID配置
    USB2CAN_CAN_Bus_Init();

    // 启动成功
    std::cout << std::endl
              << "ttyRedDog   NODE INIT__OK   by TANGAIR" << std::endl
              << std::endl
              << std::endl;
}

/// @brief 析构函数
Tangair_usb2can::~Tangair_usb2can()
{
    std::cout << "End";
    StopAllThreads();

    // 关闭设备
    closeUSBCAN(USB2CAN0_);
}

void Tangair_usb2can::Init()
{
    // InitLowCmd();
    // /*create publisher*/
    // lowcmd_publisher.reset(new ChannelPublisher<unitree_go::msg::dds_::LowCmd_>(TOPIC_LOWCMD));
    // lowcmd_publisher->InitChannel();

    /*create subscriber*/
    lowcmd_subscriber.reset(new ChannelSubscriber<unitree_go::msg::dds_::LowCmd_>(TOPIC_LOWCMD));
    lowcmd_subscriber->InitChannel(std::bind(&Tangair_usb2can::LowCmdMessageHandler, this, std::placeholders::_1), 1);

    /*loop publishing thread*/
    // lowCmdWriteThreadPtr = CreateRecurrentThreadEx("writebasiccmd", UT_CPU_ID_NONE, int(dt * 1000000), &Custom::LowCmdWrite, this);
}

void Tangair_usb2can::LowCmdMessageHandler(const void *message)
{
    low_cmd = *(unitree_go::msg::dds_::LowCmd_ *)message;

    std::vector<double> dof_pos;
    for (int i = 0; i < 12; ++i) {
        dof_pos.push_back(low_cmd.motor_cmd()[i].q());
    }

    // ✅ 正確：手動將 vector 拷貝到 array
    auto temp = mujoco_ang2real_ang(dof_pos);
    for (size_t i = 0; i < 3; ++i) {
        for (size_t j = 0; j < 4; ++j) {
            real_angles_[i][j] = temp[i][j];
        }
    }

    // for (size_t row = 0; row < real_angles_.size(); ++row) {
    //     for (double val : real_angles_[row]) {
    //         std::cout << val << " ";
    //     }
    // }

}

void Tangair_usb2can::StartPositionLoop() {
    if (running_) return;
    running_ = true;

    _CAN_TX_test_thread = std::thread(&Tangair_usb2can::CAN_TX_position_sin_thread, this);
    _CAN_RX_device_0_thread = std::thread(&Tangair_usb2can::CAN_RX_device_0_thread, this);
}

void Tangair_usb2can::StopAllThreads() {
    if (!running_) return;

    running_ = false;

    if (_CAN_TX_test_thread.joinable()) _CAN_TX_test_thread.join();
    if (_CAN_RX_device_0_thread.joinable()) _CAN_RX_device_0_thread.join();

    DISABLE_ALL_MOTOR(237);
    std::cout << "[Tangair] 所有執行緒已安全停止。\n";
}

/*****************************************************************************************************/
/*********************************       ***测试相关***      ***********************************************/
/*****************************************************************************************************/

void Tangair_usb2can::SetMotorTarget(Motor_CAN_Send_Struct &motor, double pos, double kp, double kd) {
    motor.position = pos;
    motor.speed = 0;
    motor.torque = 0;
    motor.kp = kp;
    motor.kd = kd;
}

void Tangair_usb2can::SetTargetPosition(const std::array<std::array<double, 4>, 3> &positions, double kp, double kd) {
    
    // FR
    SetMotorTarget(USB2CAN0_CAN_Bus_2.ID_1_motor_send, positions[2][0], kp, kd);
    SetMotorTarget(USB2CAN0_CAN_Bus_2.ID_2_motor_send, positions[1][0], kp, kd);
    SetMotorTarget(USB2CAN0_CAN_Bus_2.ID_3_motor_send, positions[0][0], kp, kd);
    
    // FL
    SetMotorTarget(USB2CAN0_CAN_Bus_1.ID_1_motor_send, positions[2][1], kp, kd);
    SetMotorTarget(USB2CAN0_CAN_Bus_1.ID_2_motor_send, positions[1][1], kp, kd);
    SetMotorTarget(USB2CAN0_CAN_Bus_1.ID_3_motor_send, positions[0][1], kp, kd);

    // RR
    SetMotorTarget(USB2CAN0_CAN_Bus_2.ID_5_motor_send, positions[2][2], kp, kd);
    SetMotorTarget(USB2CAN0_CAN_Bus_2.ID_6_motor_send, positions[1][2], kp, kd);
    SetMotorTarget(USB2CAN0_CAN_Bus_2.ID_7_motor_send, positions[0][2], kp, kd);

    // RL
    SetMotorTarget(USB2CAN0_CAN_Bus_1.ID_5_motor_send, positions[2][3], kp, kd);
    SetMotorTarget(USB2CAN0_CAN_Bus_1.ID_6_motor_send, positions[1][3], kp, kd);
    SetMotorTarget(USB2CAN0_CAN_Bus_1.ID_7_motor_send, positions[0][3], kp, kd);
}


void Tangair_usb2can::ResetPositionToZero()
{
    SetTargetPosition({{ {  3.0, -3.0, -3.0,  3.0}, 
                        { -1.6,  1.6,  1.6, -1.6},
                        {  0.0,  0.0,  0.0,  0.0}}}, 3.0, 0.1);
    CAN_TX_ALL_MOTOR(130);
}

void Tangair_usb2can::CAN_TX_position_sin_thread()
{
    std::cout << "[THREAD] CAN_TX_position_sin_thread start\n";
    uint32_t tx_count = 0;

    auto last_time_tx = high_resolution_clock::now();
    int count_tx = 0;

    SetTargetPosition({{ {  3.0, -3.0, -3.0,  3.0}, 
                        { -1.6,  1.6,  1.6, -1.6},
                        {  0.0,  0.0,  0.0,  0.0}}}, 3.0, 0.1);

    // 開啟電機
    ENABLE_ALL_MOTOR(130);

    while (running_) {
        count_tx++;
        tx_count++;

    for (size_t i = 0; i < 3; ++i) {
        for (size_t j = 0; j < 4; ++j) {
            target_pos[i][j] = real_angles_[i][j];
        }
    }

    // 呼叫 SetTargetPosition，使用 real_angles 轉換結果
    SetTargetPosition(target_pos, 3.0, 0.1);


        CAN_TX_ALL_MOTOR(130);

        // std::this_thread::sleep_for(std::chrono::milliseconds(1));
        auto now_tx = high_resolution_clock::now();
        auto duration_tx = duration_cast<seconds>(now_tx - last_time_tx).count();
        if (duration_tx >= 1) {
        std::cout << "[Frequency] CAN TX = " << count_tx << " Hz" << std::endl;
        count_tx = 0;
        last_time_tx = now_tx;
         // 打印数据tp时间ms，2370hz的控制频率的话，1s一次，

        std::cout << std::endl
                    << "USB2CAN0_CAN1——1.current_position_f=  " << USB2CAN0_CAN_Bus_1.ID_1_motor_recieve.current_position_f << "  rad " << std::endl
                    << "USB2CAN0_CAN1——2.current_position_f=  " << USB2CAN0_CAN_Bus_1.ID_2_motor_recieve.current_position_f << "  rad " << std::endl
                    << "USB2CAN0_CAN1——3.current_position_f=  " << USB2CAN0_CAN_Bus_1.ID_3_motor_recieve.current_position_f << "  rad " << std::endl
                    << "USB2CAN0_CAN1——5.current_position_f=  " << USB2CAN0_CAN_Bus_1.ID_5_motor_recieve.current_position_f << "  rad " << std::endl
                    << "USB2CAN0_CAN1——6.current_position_f=  " << USB2CAN0_CAN_Bus_1.ID_6_motor_recieve.current_position_f << "  rad " << std::endl
                    << "USB2CAN0_CAN1——7.current_position_f=  " << USB2CAN0_CAN_Bus_1.ID_7_motor_recieve.current_position_f << "  rad " << std::endl
                    << "USB2CAN0_CAN2——1.current_position_f=  " << USB2CAN0_CAN_Bus_2.ID_1_motor_recieve.current_position_f << "  rad " << std::endl
                    << "USB2CAN0_CAN2——2.current_position_f=  " << USB2CAN0_CAN_Bus_2.ID_2_motor_recieve.current_position_f << "  rad " << std::endl
                    << "USB2CAN0_CAN2——3.current_position_f=  " << USB2CAN0_CAN_Bus_2.ID_3_motor_recieve.current_position_f << "  rad " << std::endl
                    << "USB2CAN0_CAN2——5.current_position_f=  " << USB2CAN0_CAN_Bus_2.ID_5_motor_recieve.current_position_f << "  rad " << std::endl
                    << "USB2CAN0_CAN2——6.current_position_f=  " << USB2CAN0_CAN_Bus_2.ID_6_motor_recieve.current_position_f << "  rad " << std::endl
                    << "USB2CAN0_CAN2——7.current_position_f=  " << USB2CAN0_CAN_Bus_2.ID_7_motor_recieve.current_position_f << "  rad " << std::endl;
        }
    }

    std::cout << "CAN_TX_position_thread Exit~~" << std::endl;
}

/// @brief can设备0，接收线程函数
void Tangair_usb2can::CAN_RX_device_0_thread()
{
    can_dev0_rx_count = 0;
    can_dev0_rx_count_thread=0;

    auto last_time_rx = high_resolution_clock::now();
    int count_rx = 0;

    while (running_)
    {   
        uint8_t channel;
        FrameInfo info_rx;
        uint8_t data_rx[8] = {0};

        can_dev0_rx_count_thread++;

        // 阻塞1s接收
        int recieve_re = readUSBCAN(USB2CAN0_, &channel, &info_rx, data_rx, 1e6);
        // 接收到数据
        if (recieve_re != -1)
        {   
            count_rx++;

            can_dev0_rx_count++;
            // 解码
            CAN_DEV0_RX.ERR = data_rx[0]>>4&0X0F;
            
            CAN_DEV0_RX.current_position = (data_rx[1]<<8)|data_rx[2]; //电机位置数据
			CAN_DEV0_RX.current_speed  = (data_rx[3]<<4)|(data_rx[4]>>4); //电机速度数据
			CAN_DEV0_RX.current_torque = ((data_rx[4]&0xF)<<8)|data_rx[5]; //电机扭矩数据
			CAN_DEV0_RX.current_temp_MOS  = data_rx[6];
            CAN_DEV0_RX.current_temp_Rotor  = data_rx[7];
            // 转换
            CAN_DEV0_RX.current_position_f = uint_to_float(CAN_DEV0_RX.current_position, (P_MIN), (P_MAX), 16);
            CAN_DEV0_RX.current_speed_f = uint_to_float(CAN_DEV0_RX.current_speed, (V_MIN), (V_MAX), 12);    
            CAN_DEV0_RX.current_torque_f = uint_to_float(CAN_DEV0_RX.current_torque, (T_MIN), (T_MAX), 12);
  
            if (channel == 1) // 模块0，can1
            {
                switch (info_rx.canID)
                {
                case 0X11:
                {
                    USB2CAN0_CAN_Bus_1.ID_1_motor_recieve = CAN_DEV0_RX;
                   
                    break;
                }
                case 0X12:
                {
                    USB2CAN0_CAN_Bus_1.ID_2_motor_recieve = CAN_DEV0_RX;
                   
                    break;
                }
                case 0X13:
                {
                    USB2CAN0_CAN_Bus_1.ID_3_motor_recieve = CAN_DEV0_RX;
                   
                    break;
                }
                case 0X15:
                {
                    USB2CAN0_CAN_Bus_1.ID_5_motor_recieve = CAN_DEV0_RX;
                   
                    break;
                }
                case 0X16:
                {
                    USB2CAN0_CAN_Bus_1.ID_6_motor_recieve = CAN_DEV0_RX;
                   
                    break;
                }
                case 0X17:
                {
                    USB2CAN0_CAN_Bus_1.ID_7_motor_recieve = CAN_DEV0_RX;
                   
                    break;
                }
                default:
                    break;
                }
            }
            else if (channel == 2) // 模块0，can2
            {
                switch (info_rx.canID)
                {
                case 0X11:
                {
                    USB2CAN0_CAN_Bus_2.ID_1_motor_recieve = CAN_DEV0_RX;
                  
                    break;
                }
                case 0X12:
                {
                    USB2CAN0_CAN_Bus_2.ID_2_motor_recieve = CAN_DEV0_RX;
                   
                    break;
                }
                case 0X13:
                {
                    USB2CAN0_CAN_Bus_2.ID_3_motor_recieve = CAN_DEV0_RX;
                   
                    break;
                }
                case 0X15:
                {
                    USB2CAN0_CAN_Bus_2.ID_5_motor_recieve = CAN_DEV0_RX;
                  
                    break;
                }
                case 0X16:
                {
                    USB2CAN0_CAN_Bus_2.ID_6_motor_recieve = CAN_DEV0_RX;
                   
                    break;
                }
                case 0X17:
                {
                    USB2CAN0_CAN_Bus_2.ID_7_motor_recieve = CAN_DEV0_RX;
                   
                    break;
                }
               
                default:
                    break;
                }
            }
            auto now_rx = high_resolution_clock::now();
            auto duration_rx = duration_cast<seconds>(now_rx - last_time_rx).count();
            if (duration_rx >= 1) {
            std::cout << "[Frequency] CAN RX = " << count_rx / 12 << " Hz" << std::endl;
            count_rx = 0;
            last_time_rx = now_rx;
            }
        }
    }
    std::cout << "CAN_RX_device_0_thread  Exit~~" << std::endl;
}

/*****************************************************************************************************/
/*********************************       ***电机相关***      ***********************************************/
/*****************************************************************************************************/

void Tangair_usb2can::USB2CAN_CAN_Bus_inti_set(USB2CAN_CAN_Bus_Struct *CAN_Bus)
{
    CAN_Bus->ID_1_motor_send.id = 0X01;

    CAN_Bus->ID_2_motor_send.id = 0X02;

    CAN_Bus->ID_3_motor_send.id = 0X03;

    CAN_Bus->ID_5_motor_send.id = 0X05;

    CAN_Bus->ID_6_motor_send.id = 0X06;

    CAN_Bus->ID_7_motor_send.id = 0X07;
}

void Tangair_usb2can::USB2CAN_CAN_Bus_Init()
{
    USB2CAN_CAN_Bus_inti_set(&USB2CAN0_CAN_Bus_1);
    USB2CAN_CAN_Bus_inti_set(&USB2CAN0_CAN_Bus_2);
}

/// @brief 使能
/// @param dev
/// @param channel
/// @param Motor_Data
void Tangair_usb2can::Motor_Enable(int32_t dev, uint8_t channel, Motor_CAN_Send_Struct *Motor_Data)
{
    txMsg_CAN.canID = Motor_Data->id;
    // printf("0x%02X", txMsg_CAN.canID);

    Data_CAN[0] = 0xFF;
    Data_CAN[1] = 0xFF;
    Data_CAN[2] = 0xFF;
    Data_CAN[3] = 0xFF;
    Data_CAN[4] = 0xFF;
    Data_CAN[5] = 0xFF;
    Data_CAN[6] = 0xFF;
    Data_CAN[7] = 0xFC;

    int ret = sendUSBCAN(dev, channel, &txMsg_CAN, Data_CAN);
    // printf("[DEBUG] sendUSBCAN return value: %d\n", ret);
}

/// @brief 电机失能
/// @param dev
/// @param channel
/// @param Motor_Data
void Tangair_usb2can::Motor_Disable(int32_t dev, uint8_t channel, Motor_CAN_Send_Struct *Motor_Data)
{
    txMsg_CAN.canID = Motor_Data->id;

    Data_CAN[0] = 0xFF;
    Data_CAN[1] = 0xFF;
    Data_CAN[2] = 0xFF;
    Data_CAN[3] = 0xFF;
    Data_CAN[4] = 0xFF;
    Data_CAN[5] = 0xFF;
    Data_CAN[6] = 0xFF;
    Data_CAN[7] = 0xFD;

    sendUSBCAN(dev, channel, &txMsg_CAN, Data_CAN);
}

/// @brief 设置零点
/// @param dev
/// @param channel
/// @param Motor_Data
void Tangair_usb2can::Motor_Zore(int32_t dev, uint8_t channel, Motor_CAN_Send_Struct *Motor_Data)
{
    txMsg_CAN.canID = Motor_Data->id;

    Data_CAN[0] = 0xFF;
    Data_CAN[1] = 0xFF;
    Data_CAN[2] = 0xFF;
    Data_CAN[3] = 0xFF;
    Data_CAN[4] = 0xFF;
    Data_CAN[5] = 0xFF;
    Data_CAN[6] = 0xFF;
    Data_CAN[7] = 0xFE;

    sendUSBCAN(dev, channel, &txMsg_CAN, Data_CAN);
}

/// @brief 电机控制
/// @param dev 模块设备号
/// @param channel can1或者can2
/// @param Motor_Data 电机数据
void Tangair_usb2can::CAN_Send_Control(int32_t dev, uint8_t channel, Motor_CAN_Send_Struct *Motor_Data) // 运控�??,CAN1=CAN_TX_MAILBOX0,CAN2=CAN_TX_MAILBOX1
{
    // 运控模式专用的局部变
    FrameInfo txMsg_Control = {
        .canID = Motor_Data->id,
        .frameType = STANDARD,
        .dataLength = 8,
    };
    uint8_t Data_CAN_Control[8];

    //限制范围
    if(Motor_Data->kp>KP_MAX) Motor_Data->kp=KP_MAX;
        else if(Motor_Data->kp<KP_MIN) Motor_Data->kp=KP_MIN;
    if(Motor_Data->kd>KD_MAX ) Motor_Data->kd=KD_MAX;
        else if(Motor_Data->kd<KD_MIN) Motor_Data->kd=KD_MIN;
    if(Motor_Data->position>P_MAX)	Motor_Data->position=P_MAX;
        else if(Motor_Data->position<P_MIN) Motor_Data->position=P_MIN;
    if(Motor_Data->speed>V_MAX)	Motor_Data->speed=V_MAX;
        else if(Motor_Data->speed<V_MIN) Motor_Data->speed=V_MIN;
    if(Motor_Data->torque>T_MAX)	Motor_Data->torque=T_MAX;
        else if(Motor_Data->torque<T_MIN) Motor_Data->torque=T_MIN;

    Data_CAN_Control[0] = float_to_uint(Motor_Data->position, P_MIN, P_MAX, 16)>>8; //位置�?? 8
    Data_CAN_Control[1] = float_to_uint(Motor_Data->position, P_MIN, P_MAX, 16)&0xFF; //位置�?? 8
    Data_CAN_Control[2] = float_to_uint(Motor_Data->speed, V_MIN, V_MAX, 12)>>4; //速度�?? 8 �??
    Data_CAN_Control[3] = ((float_to_uint(Motor_Data->speed, V_MIN, V_MAX, 12)&0xF)<<4)|(float_to_uint(Motor_Data->kp, KP_MIN, KP_MAX, 12)>>8); //速度�?? 4 �?? KP �?? 4 �??
    Data_CAN_Control[4] = float_to_uint(Motor_Data->kp, KP_MIN, KP_MAX, 12)&0xFF; //KP �?? 8 �??
    Data_CAN_Control[5] = float_to_uint(Motor_Data->kd, KD_MIN, KD_MAX, 12)>>4; //Kd �?? 8 �??
    Data_CAN_Control[6] = ((float_to_uint(Motor_Data->kd, KD_MIN, KD_MAX, 12)&0xF)<<4)|(float_to_uint(Motor_Data->torque, T_MIN, T_MAX, 12)>>8); //KP �?? 4 位扭矩高 4 �??
    Data_CAN_Control[7] = float_to_uint(Motor_Data->torque, T_MIN, T_MAX, 12)&0xFF; //扭矩�?? 8

    int ret = sendUSBCAN(dev, channel, &txMsg_Control, Data_CAN_Control);
    // printf("[DEBUG] sendUSBCAN return value: %d\n", ret);

}

/// @brief 电机阻尼模式
/// @param dev
/// @param channel
/// @param Motor_Data
void Tangair_usb2can::Motor_Passive_SET(int32_t dev, uint8_t channel, Motor_CAN_Send_Struct *Motor_Data)
{
    Motor_Data->speed = 0;
    Motor_Data->kp = 0;
    Motor_Data->kd = 2.0;
    Motor_Data->torque = 0;

    CAN_Send_Control(dev, channel, Motor_Data);
}

void Tangair_usb2can::ENABLE_ALL_MOTOR(int delay_us)
{   
    // FRH
    Motor_Enable(USB2CAN0_, 2, &USB2CAN0_CAN_Bus_2.ID_1_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
    // FRT
    Motor_Enable(USB2CAN0_, 2, &USB2CAN0_CAN_Bus_2.ID_2_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
    // FRC
    Motor_Enable(USB2CAN0_, 2, &USB2CAN0_CAN_Bus_2.ID_3_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
    
    // FLH
    Motor_Enable(USB2CAN0_, 1, &USB2CAN0_CAN_Bus_1.ID_1_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
    // FLT
    Motor_Enable(USB2CAN0_, 1, &USB2CAN0_CAN_Bus_1.ID_2_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
    // FLC
    Motor_Enable(USB2CAN0_, 1, &USB2CAN0_CAN_Bus_1.ID_3_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us

    // RRH
    Motor_Enable(USB2CAN0_, 2, &USB2CAN0_CAN_Bus_2.ID_5_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
    // RRT
    Motor_Enable(USB2CAN0_, 2, &USB2CAN0_CAN_Bus_2.ID_6_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
    // RRC
    Motor_Enable(USB2CAN0_, 2, &USB2CAN0_CAN_Bus_2.ID_7_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us

    // RLH
    Motor_Enable(USB2CAN0_, 1, &USB2CAN0_CAN_Bus_1.ID_5_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
    // RLT
    Motor_Enable(USB2CAN0_, 1, &USB2CAN0_CAN_Bus_1.ID_6_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
    // RLC
    Motor_Enable(USB2CAN0_, 1, &USB2CAN0_CAN_Bus_1.ID_7_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
}

void Tangair_usb2can::DISABLE_ALL_MOTOR(int delay_us)
{
    // FRH
    Motor_Disable(USB2CAN0_, 2, &USB2CAN0_CAN_Bus_2.ID_1_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
    // FLT
    Motor_Disable(USB2CAN0_, 2, &USB2CAN0_CAN_Bus_2.ID_2_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
    // FLC
    Motor_Disable(USB2CAN0_, 2, &USB2CAN0_CAN_Bus_2.ID_3_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us

    // FLH
    Motor_Disable(USB2CAN0_, 1, &USB2CAN0_CAN_Bus_1.ID_1_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
    // FLT
    Motor_Disable(USB2CAN0_, 1, &USB2CAN0_CAN_Bus_1.ID_2_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
    // FLC
    Motor_Disable(USB2CAN0_, 1, &USB2CAN0_CAN_Bus_1.ID_3_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us

    // RRH
    Motor_Disable(USB2CAN0_, 2, &USB2CAN0_CAN_Bus_2.ID_5_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
    // RRT
    Motor_Disable(USB2CAN0_, 2, &USB2CAN0_CAN_Bus_2.ID_6_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
    // RRC
    Motor_Disable(USB2CAN0_, 2, &USB2CAN0_CAN_Bus_2.ID_7_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
    
    // RLH
    Motor_Disable(USB2CAN0_, 1, &USB2CAN0_CAN_Bus_1.ID_5_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
    // RLT
    Motor_Disable(USB2CAN0_, 1, &USB2CAN0_CAN_Bus_1.ID_6_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
    // RLC
    Motor_Disable(USB2CAN0_, 1, &USB2CAN0_CAN_Bus_1.ID_7_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
}

void Tangair_usb2can::ZERO_ALL_MOTOR(int delay_us)
{
    // FRH
    Motor_Zore(USB2CAN0_, 2, &USB2CAN0_CAN_Bus_2.ID_1_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
    // FRT
    Motor_Zore(USB2CAN0_, 2, &USB2CAN0_CAN_Bus_2.ID_2_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
    // FRC
    Motor_Zore(USB2CAN0_, 2, &USB2CAN0_CAN_Bus_2.ID_3_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
    
    // FLH
    Motor_Zore(USB2CAN0_, 1, &USB2CAN0_CAN_Bus_1.ID_1_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
    // FLT
    Motor_Zore(USB2CAN0_, 1, &USB2CAN0_CAN_Bus_1.ID_2_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
    // FLC
    Motor_Zore(USB2CAN0_, 1, &USB2CAN0_CAN_Bus_1.ID_3_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
    
    // RRH
    Motor_Zore(USB2CAN0_, 2, &USB2CAN0_CAN_Bus_2.ID_5_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
    // RRT
    Motor_Zore(USB2CAN0_, 2, &USB2CAN0_CAN_Bus_2.ID_6_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
    // RRC
    Motor_Zore(USB2CAN0_, 2, &USB2CAN0_CAN_Bus_2.ID_7_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us

    // RLH
    Motor_Zore(USB2CAN0_, 1, &USB2CAN0_CAN_Bus_1.ID_5_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
    // RLT
    Motor_Zore(USB2CAN0_, 1, &USB2CAN0_CAN_Bus_1.ID_6_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
    // RLC
    Motor_Zore(USB2CAN0_, 1, &USB2CAN0_CAN_Bus_1.ID_7_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
}

void Tangair_usb2can::PASSIVE_ALL_MOTOR(int delay_us)
{   
    // FRH
    Motor_Passive_SET(USB2CAN0_, 2, &USB2CAN0_CAN_Bus_2.ID_1_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
    // FRT
    Motor_Passive_SET(USB2CAN0_, 2, &USB2CAN0_CAN_Bus_2.ID_2_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
    // FRC
    Motor_Passive_SET(USB2CAN0_, 2, &USB2CAN0_CAN_Bus_2.ID_3_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
    
    // FLH
    Motor_Passive_SET(USB2CAN0_, 1, &USB2CAN0_CAN_Bus_1.ID_1_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
    // FLT
    Motor_Passive_SET(USB2CAN0_, 1, &USB2CAN0_CAN_Bus_1.ID_2_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
    // FLC
    Motor_Passive_SET(USB2CAN0_, 1, &USB2CAN0_CAN_Bus_1.ID_3_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
    
    // RRH
    Motor_Passive_SET(USB2CAN0_, 2, &USB2CAN0_CAN_Bus_2.ID_5_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
    // RRT
    Motor_Passive_SET(USB2CAN0_, 2, &USB2CAN0_CAN_Bus_2.ID_6_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
    // RRC
    Motor_Passive_SET(USB2CAN0_, 2, &USB2CAN0_CAN_Bus_2.ID_7_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us

    // RLH
    Motor_Passive_SET(USB2CAN0_, 1, &USB2CAN0_CAN_Bus_1.ID_5_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
    // RLT
    Motor_Passive_SET(USB2CAN0_, 1, &USB2CAN0_CAN_Bus_1.ID_6_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
    // RLC
    Motor_Passive_SET(USB2CAN0_, 1, &USB2CAN0_CAN_Bus_1.ID_7_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
}

/// @brief can控制发送，12个电机的数据
// 目前能达到1000hz的控制频率--------3000hz的总线发送频率---------同一路can的发送间隔在300us
void Tangair_usb2can::CAN_TX_ALL_MOTOR(int delay_us)
{
    auto t = std::chrono::high_resolution_clock::now();//这一句耗时50us

    //FRH
    CAN_Send_Control(USB2CAN0_, 2, &USB2CAN0_CAN_Bus_2.ID_1_motor_send);
    t += std::chrono::microseconds(delay_us);
    std::this_thread::sleep_until(t);
    //FRT
    CAN_Send_Control(USB2CAN0_, 2, &USB2CAN0_CAN_Bus_2.ID_2_motor_send);
    t += std::chrono::microseconds(delay_us);
    std::this_thread::sleep_until(t);
    //FRC
    CAN_Send_Control(USB2CAN0_, 2, &USB2CAN0_CAN_Bus_2.ID_3_motor_send);
    t += std::chrono::microseconds(delay_us);
    std::this_thread::sleep_until(t);

    //FLH
    CAN_Send_Control(USB2CAN0_, 1, &USB2CAN0_CAN_Bus_1.ID_1_motor_send);
    t += std::chrono::microseconds(delay_us);
    std::this_thread::sleep_until(t);
    //FLT
    CAN_Send_Control(USB2CAN0_, 1, &USB2CAN0_CAN_Bus_1.ID_2_motor_send);
    t += std::chrono::microseconds(delay_us);
    std::this_thread::sleep_until(t);
    //FLC
    CAN_Send_Control(USB2CAN0_, 1, &USB2CAN0_CAN_Bus_1.ID_3_motor_send);
    t += std::chrono::microseconds(delay_us);
    std::this_thread::sleep_until(t);

    //RRH
    CAN_Send_Control(USB2CAN0_, 2, &USB2CAN0_CAN_Bus_2.ID_5_motor_send);
    t += std::chrono::microseconds(delay_us);
    std::this_thread::sleep_until(t);
    //RRT
    CAN_Send_Control(USB2CAN0_, 2, &USB2CAN0_CAN_Bus_2.ID_6_motor_send);
    t += std::chrono::microseconds(delay_us);
    std::this_thread::sleep_until(t);
    //RRC
    CAN_Send_Control(USB2CAN0_, 2, &USB2CAN0_CAN_Bus_2.ID_7_motor_send);
    t += std::chrono::microseconds(delay_us);
    std::this_thread::sleep_until(t);

    //RLH
    CAN_Send_Control(USB2CAN0_, 1, &USB2CAN0_CAN_Bus_1.ID_5_motor_send);
    t += std::chrono::microseconds(delay_us);
    std::this_thread::sleep_until(t);
    //RLT
    CAN_Send_Control(USB2CAN0_, 1, &USB2CAN0_CAN_Bus_1.ID_6_motor_send);
    t += std::chrono::microseconds(delay_us);
    std::this_thread::sleep_until(t);
    //RLC
    CAN_Send_Control(USB2CAN0_, 1, &USB2CAN0_CAN_Bus_1.ID_7_motor_send);
    t += std::chrono::microseconds(delay_us);
    std::this_thread::sleep_until(t);
}

/// @brief 辅助函数
std::vector<std::vector<double>> mujoco_ang2real_ang(const std::vector<double>& dof_pos) {
    std::vector<std::string> motor_order = {
        "frd", "fld", "rrd", "rld",  // Lower legs
        "fru", "flu", "rru", "rlu",  // Upper legs
        "frh", "flh", "rrh", "rlh"   // Hips
    };

    std::vector<std::string> mujoco_order = {
        "frh", "fru", "frd",
        "flh", "flu", "fld",
        "rrh", "rru", "rrd",
        "rlh", "rlu", "rld"
    };

    std::vector<int> index_map;
    for (const auto& name : motor_order) {
        auto it = std::find(mujoco_order.begin(), mujoco_order.end(), name);
        if (it == mujoco_order.end()) {
            throw std::runtime_error("Motor name not found in mujoco_order: " + name);
        }
        index_map.push_back(std::distance(mujoco_order.begin(), it));
    }

    std::vector<double> reordered_dof_pos;
    for (int i : index_map) {
        reordered_dof_pos.push_back(dof_pos[i]);
    }

    std::vector<std::vector<double>> result = {
        { -reordered_dof_pos[0],  reordered_dof_pos[1],  -reordered_dof_pos[2],  reordered_dof_pos[3] },
        { -reordered_dof_pos[4],  reordered_dof_pos[5],  -reordered_dof_pos[6],  reordered_dof_pos[7] },
        {  reordered_dof_pos[8],  reordered_dof_pos[9],  -reordered_dof_pos[10], -reordered_dof_pos[11] }
    };

    return result;
}

int float_to_uint(float x, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    if (x > x_max)
        x = x_max;
    else if (x < x_min)
        x = x_min;
    return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    /// converts unsigned int to float, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

uint32_t crc32_core(uint32_t *ptr, uint32_t len)
{
    unsigned int xbit = 0;
    unsigned int data = 0;
    unsigned int CRC32 = 0xFFFFFFFF;
    const unsigned int dwPolynomial = 0x04c11db7;

    for (unsigned int i = 0; i < len; i++)
    {
        xbit = 1 << 31;
        data = ptr[i];
        for (unsigned int bits = 0; bits < 32; bits++)
        {
            if (CRC32 & 0x80000000)
            {
                CRC32 <<= 1;
                CRC32 ^= dwPolynomial;
            }
            else
            {
                CRC32 <<= 1;
            }

            if (data & xbit)
                CRC32 ^= dwPolynomial;
            xbit >>= 1;
        }
    }

    return CRC32;
}

