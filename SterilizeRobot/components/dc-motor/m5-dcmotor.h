/**
 * @file m5-dcmotor.h
 * @brief  Functions for control the M5 DC motor and read Encoder.
 *         Protocol : i2C
 *         Module M5 DC Motor  start address for control mot i2C is 0x56 
 *         next motor will be  + 2 bytes for 4 Motors will end at 0x56 + 0x07
 *         Encoder will start at 0x56+ 0x08
 * 
 */

#pragma once

#include "stdint.h"

#define DCMOTOR_MODULE_ADDRESS   0x56 
#define MOTOR_ADDR_BASE      0x00
#define ENCODER_ADDR_BASE    0x08
#define MOTOR_CTRL_ADDR     (MOTOR_ADDR_BASE + 0)
#define MOTOR_CTRL_LEN      2
#define NUMS_OF_MOTOR       4
#define MOTOR_TOTAL_LEN     (MOTOR_CTRL_LEN * NUMS_OF_MOTOR)
#define ENCODER_READ_ADDR   (MOTOR_CTRL_ADDR + MOTOR_TOTAL_LEN)
#define ENCODER_READ_LEN    4
#define NUMS_OF_ENCODER     4
#define ENCODER_TOTAL_LEN   (ENCODER_READ_LEN * NUMS_OF_ENCODER)

/*
#define STEP_V      51
#define FRONT 4
#define X_LOCAL  60
#define Y_LOCAL  80
#define XF  30
#define YF  30
*/
// For easy wiring  use Motor 1 and 4
#define  MOTOR_1  0
#define  MOTOR_2  2
#define  MOTOR_3  1
#define   MOTOR_4  3


/**
 * @brief Initailize for DC Motor  Allocate memory for DC Motor Driver
 * 
 * 
 */
/* @[declare_dcmotor_init] */
void DCMOTOR_Init(void);
/* @[declare_dcmotor_init] */

/**
 * @brief Set speed and direction for Motor N
 * 
 * @param[in] num   Motor Number 1- 4
 * @param[in] speed Motor Speed of Motor from  1 - 255 mean forward    -1 - (-255) mean backward  0 = stop
 * @param[in] angle Angle to rotate  0 - 360  0 mean continuous rotate.  
 */
/* @[declare_dcmotor_setspeed] */
void DCMOTOR_SetSpeed(int16_t num, int16_t speed);
/* @[declare_dcmotor_setspeed] */


/**
 * @brief Get Encode rotation
 * 
 * @param[in] num Motor Number 1 - 4
 * 
 * @return The Encoder position.
 */
/* @[declare_dcmotor_readencoder] */
int32_t DCMOTOR_ReadEncoder(int16_t num);
/* @[declare_dcmotor_readencoder] */


//MOTOR CONTROL
void move_forward (int16_t speed,  int16_t angle);
void move_backward (int16_t speed, int16_t angle);
void turn_left (int16_t angle);
void turn_right (int16_t angle);
void stop_motor(void);
