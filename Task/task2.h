/**
 * @file  task2.h
 * @brief Task 2: A→B→C→D→A 完整一圈。
 *        四段路段: AB直线 → BC弧线 → CD直线 → DA弧线。
 *        直线用陀螺仪 Yaw PID 保持航向, 弧线用灰度查表+角速度 PID 巡线。
 */

#ifndef __TASK2_H
#define __TASK2_H

void Task2_Run(void);

#endif /* __TASK2_H */
