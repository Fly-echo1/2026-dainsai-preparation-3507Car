/**
 * @file  task2neo.h
 * @brief Task2neo:
 *   阶段1: 陀螺仪直行→B站停车 (蜂鸣+延时)
 *   阶段2: 灰度PI巡线→C点停车
 *   阶段3: 调头旋转 (开环/闭环可选)
 *   阶段4: 陀螺仪直行 (重复AB段)
 */

#ifndef __TASK2NEO_H
#define __TASK2NEO_H

/* ===== 阶段3 调头模式选择 (二选一) ===== */
 //#define TURN_MODE_CLOSED_LOOP /* 闭环PID调头 (注释此行则切回开环) */
#define TURN_MODE_OPEN_LOOP /* 开环调头 */

/* ===== 闭环调头 PID 参数 ===== */
#define TURN_ANGLE_KP 5.0f        /* 角度 P 系数                    */
#define TURN_ANGLE_KI 0.1f        /* 角度 I 系数                    */
#define TURN_ANGLE_KD 0.0f        /* 角度 D 系数                    */
#define TURN_TARGET_YAW (-180.0f) /* 目标 yaw 角                */
#define TURN_TOLERANCE 3.0f       /* 旋转到位容忍度 (°)         */

void Task2neo_Run(void);

#endif /* __TASK2NEO_H */
