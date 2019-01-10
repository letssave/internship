/*
 * accelerometer.h
 *
 *  Created on: Aug 20, 2018
 *      Author: nikola
 */

#ifndef ACCELEROMETER_H_
#define ACCELEROMETER_H_

#include <stdio.h>
#include <string.h>
#include "board.h"
#include <stdbool.h>
#include "fsl_i2c.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define ACCELEROMETER_I2C_MASTER_BASE (I2C0_BASE)
#define ACCELEROMETER_MASTER_CLOCK_FREQUENCY (12000000)

#define ACCELEROMETER_I2C_MASTER ((I2C_Type *)ACCELEROMETER_I2C_MASTER_BASE)

#define ACCE_I2C_MASTER_SLAVE_ADDR_7BIT 0x30U
#define ACCE_I2C_BAUDRATE 100000U
#define ACCE_I2C_DATA_LENGTH 32U

#define ACCSS_GINT0 kGINT_Port1
#define ACCSS_POL_MASK0 ~(1U << 18)
#define ACCSS_ENA_MASK0 (1U << 18)

#define ACCSS_GINT1 kGINT_Port0
#define ACCSS_POL_MASK1 ~(1U << 30)
#define ACCSS_ENA_MASK1 (1U << 30)

#define ACCLR_ADDR 0x19

// Register addresses:
#define _REG_OUTADC1_L   (0x08)
#define _REG_WHOAMI      (0x0F)
#define _REG_TEMPCFG     (0x1F)
#define _REG_CTRL1       (0x20)
#define _REG_CTRL3       (0x22)
#define _REG_CTRL4       (0x23)
#define _REG_CTRL5       (0x24)
#define _REG_OUT_X_L     (0x28)
#define _REG_INT1SRC     (0x31)
#define _REG_CLICKCFG    (0x38)
#define _REG_CLICKSRC    (0x39)
#define _REG_CLICKTHS    (0x3A)
#define _REG_TIMELIMIT   (0x3B)
#define _REG_TIMELATENCY (0x3C)
#define _REG_TIMEWINDOW  (0x3D)

//Device Registers
#define LIS3DH_STATUS_REG_AUX         0x07
#define LIS3DH_OUT_ADC1_L             0x08
#define LIS3DH_OUT_ADC1_H             0x09
#define LIS3DH_OUT_ADC2_L             0x0A
#define LIS3DH_OUT_ADC2_H             0x0B
#define LIS3DH_OUT_ADC3_L             0x0C
#define LIS3DH_OUT_ADC3_H             0x0D
#define LIS3DH_INT_COUNTER_REG        0x0E
#define LIS3DH_WHO_AM_I               0x0F

#define LIS3DH_TEMP_CFG_REG           0x1F
#define LIS3DH_CTRL_REG1              0x20
#define LIS3DH_CTRL_REG2              0x21
#define LIS3DH_CTRL_REG3              0x22
#define LIS3DH_CTRL_REG4              0x23
#define LIS3DH_CTRL_REG5              0x24
#define LIS3DH_CTRL_REG6              0x25
#define LIS3DH_REFERENCE              0x26
#define LIS3DH_STATUS_REG2            0x27
#define LIS3DH_OUT_X_L                0x28
#define LIS3DH_OUT_X_H                0x29
#define LIS3DH_OUT_Y_L                0x2A
#define LIS3DH_OUT_Y_H                0x2B
#define LIS3DH_OUT_Z_L                0x2C
#define LIS3DH_OUT_Z_H                0x2D
#define LIS3DH_FIFO_CTRL_REG          0x2E
#define LIS3DH_FIFO_SRC_REG           0x2F
#define LIS3DH_INT1_CFG               0x30
#define LIS3DH_INT1_SRC               0x31
#define LIS3DH_INT1_THS               0x32

#define LIS3DH_INT2_DURATION          0x33
#define LIS3DH_INT2_CFG               0x34
#define LIS3DH_INT2_SRC               0x35
#define LIS3DH_INT2_THS               0x36
#define LIS3DH_INT2_DURATION          0x37

#define LIS3DH_CLICK_CFG              0x38
#define LIS3DH_CLICK_SRC              0x39
#define LIS3DH_CLICK_THS              0x3A
#define LIS3DH_TIME_LIMIT             0x3B
#define LIS3DH_TIME_LATENCY           0x3C
#define LIS3DH_TIME_WINDOW            0x3D

/* Select two inputs, active low for GINT1. SW2 & SW3 must be connected to the same port */
#define DEMO_GINT1_POL_MASK ~((1U << BOARD_SW3_GPIO_PIN) | (1U << BOARD_SW4_GPIO_PIN))
#define DEMO_GINT1_ENA_MASK ((1U << BOARD_SW3_GPIO_PIN) | (1U << BOARD_SW4_GPIO_PIN))


#define ACCSS_GINT kGINT_Port1
#define ACCSS_POL_MASK ~(1U << 18)
#define ACCSS_ENA_MASK (1U << 18)


status_t writeRegister(uint8_t offset, uint8_t dataToWrite);
status_t readRegister(uint8_t* outputPointer, uint8_t offset);
status_t readRegisterRegion(uint8_t *outputPointer , uint8_t offset, uint8_t length);
status_t readRegisterInt16( int16_t* outputPointer, uint8_t offset );
float calcAccel( int16_t input );
int16_t readRawAccelX( void );
float readFloatAccelX( void );
int16_t readRawAccelY( void );
float readFloatAccelY( void );
int16_t readRawAccelZ( void );
float readFloatAccelZ( void );
void applySettings( void );
void accslr_configIntterupts();
void accslr_configClick();



extern volatile uint8_t accslr_irq;

void acc_int0_callback(void);
void acc_int1_callback(void);

status_t accelerometer_init(void);

//#TODO make sound when device is shaken
extern bool play_cling_sound;




#endif /* ACCELEROMETER_H_ */
