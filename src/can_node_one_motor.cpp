// Copyright (c) 2023–2025 TANGAIR
// SPDX-License-Identifier: Apache-2.0

#include "Tangair_usb2can_one_motor.h"
#include <memory>
#include <iostream>
#include <csignal>
#include <thread>
#include <unistd.h>
#include <sched.h>
#include <atomic>

std::shared_ptr<Tangair_usb2can> CAN_ptr;
std::thread pos_thread;
std::thread read_thread;

// 中斷旗標
volatile sig_atomic_t shutdown_requested = 0;

// 訊號處理函式
void signal_callback_handler(int signum) {
    shutdown_requested = 1;
}

int main() {
    // 設定 real-time 行程排程
    pid_t pid = getpid();
    sched_param param;
    param.sched_priority = sched_get_priority_max(SCHED_FIFO);
    if (sched_setscheduler(pid, SCHED_FIFO, &param) == -1) {
        std::cout << "[ERROR] 設定即時排程失敗。\n";
    }

    CAN_ptr = std::make_shared<Tangair_usb2can>();
    signal(SIGINT, signal_callback_handler);

    std::cout << "\n請輸入指令啟動馬達操作："
                 "\n(enable / disable / passive / set / reset / position / stop / exit)\n";

    while (true) {
        if (shutdown_requested) {
            std::cout << "\n[INFO] 收到中斷訊號，正在安全結束...\n";
            CAN_ptr->StopAllThreads();
            break;
        }

        std::string input;
        std::cout << ">> ";
        std::cin >> input;

        if (input == "enable") {
            CAN_ptr->ENABLE_ALL_MOTOR(100);
        } else if (input == "disable") {
            CAN_ptr->DISABLE_ALL_MOTOR(100);
        } else if (input == "passive") {
            CAN_ptr->PASSIVE_ALL_MOTOR(100);
        } else if (input == "set") {
            CAN_ptr->ZERO_ALL_MOTOR(100);

        } else if (input == "reset") {
            CAN_ptr->StopAllThreads();
            CAN_ptr->ResetPositionToZero();
        } else if (input == "position") {
            CAN_ptr->StopAllThreads();
            CAN_ptr->StartPositionLoop();
        } else if (input == "stop") {
            CAN_ptr->StopAllThreads();
        } else if (input == "exit") {
            CAN_ptr->StopAllThreads();
            break;
        } else {
            std::cout << "[提示] 不支援的指令，請輸入："
                         "enable / disable / passive / set / reset / read / position / stop / exit\n";
        }
    }

    std::cout << "[INFO] 程式結束。\n";
    return 0;
}
