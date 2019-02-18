/*
 * accelerometer.c
 *
 *  Created on: Aug 20, 2018
 *      Author: nikola
 */
#include "accelerometer.h"

#include "fsl_inputmux.h"
#include "fsl_pint.h"
#define DEMO_PINT_PIN_INT2_SRC kINPUTMUX_GpioPort0Pin30ToPintsel

uint8_t g_master_txBuff[ACCE_I2C_DATA_LENGTH];
uint8_t g_master_rxBuff[ACCE_I2C_DATA_LENGTH];
volatile bool acc_g_MasterCompletionFlag = false;
volatile uint8_t accslr_irq2 = 0;

//This struct holds the settings the driver uses to do calculations
typedef struct
{
	//ADC and Temperature settings
	uint8_t adcEnabled;
	uint8_t tempEnabled;

	//Accelerometer settings
	uint16_t accelSampleRate;  //Hz.  Can be: 0,1,10,25,50,100,200,400,1600,5000 Hz
	uint8_t accelRange;      //Max G force readable.  Can be: 2, 4, 8, 16

	uint8_t xAccelEnabled;
	uint8_t yAccelEnabled;
	uint8_t zAccelEnabled;

	//Fifo settings
	uint8_t fifoEnabled;
	uint8_t fifoMode; //can be 0x0,0x1,0x2,0x3
	uint8_t fifoThreshold;

	//Interrupt settings

} SensorSettings;

SensorSettings accslr_settings;

bool play_cling_sound = false;

/*******************************************************************************
 * Code

 ******************************************************************************/

//****************************************************************************//
//
//  writeRegister
//
//  Parameters:
//    offset -- register to write
//    dataToWrite -- 8 bit data to write to register
//
//****************************************************************************//
status_t writeRegister(uint8_t offset, uint8_t dataToWrite) {

	status_t reVal = kStatus_Success;

	reVal = I2C_MasterRepeatedStart(ACCELEROMETER_I2C_MASTER, ACCLR_ADDR, kI2C_Write);
	if (reVal != kStatus_Success)
	{
		return -1;
	}

	reVal = I2C_MasterWriteBlocking(ACCELEROMETER_I2C_MASTER, &offset, 1, kI2C_TransferNoStopFlag);
	if (reVal != kStatus_Success)
	{
		return -1;
	}

	reVal = I2C_MasterWriteBlocking(ACCELEROMETER_I2C_MASTER, &dataToWrite, 1, kI2C_TransferNoStopFlag);
	if (reVal != kStatus_Success)
	{
		return -1;
	}

	return reVal;
}






//****************************************************************************//
//
//  ReadRegister
//
//  Parameters:
//    *outputPointer -- Pass &variable (address of) to save read data to
//    offset -- register to read
//
//****************************************************************************//
status_t readRegister(uint8_t* outputPointer, uint8_t offset) {
	//Return value
	uint8_t numBytes = 1;
	status_t reVal = kStatus_Success;


	reVal = I2C_MasterRepeatedStart(ACCELEROMETER_I2C_MASTER, ACCLR_ADDR, kI2C_Write);
	if (reVal != kStatus_Success)
	{
		return -1;
	}

	reVal = I2C_MasterWriteBlocking(ACCELEROMETER_I2C_MASTER, &offset, 1, kI2C_TransferNoStopFlag);
	if (reVal != kStatus_Success)
	{
		return -1;
	}

	reVal = I2C_MasterRepeatedStart(ACCELEROMETER_I2C_MASTER, ACCLR_ADDR, kI2C_Read);
	if (reVal != kStatus_Success)
	{
		return -1;
	}

	reVal = I2C_MasterReadBlocking(ACCELEROMETER_I2C_MASTER, outputPointer, numBytes, 0);
	if (reVal != kStatus_Success)
	{
		return -1;
	}

	return reVal;
}
//****************************************************************************//
//
//  ReadRegisterRegion
//
//  Parameters:
//    *outputPointer -- Pass &variable (base address of) to save read data to
//    offset -- register to read
//    length -- number of bytes to read
//
//  Note:  Does not know if the target memory space is an array or not, or
//    if there is the array is big enough.  if the variable passed is only
//    two bytes long and 3 bytes are requested, this will over-write some
//    other memory!
//
//****************************************************************************//
status_t readRegisterRegion(uint8_t *outputPointer , uint8_t offset, uint8_t length)
{
	status_t reVal = 0;

	offset |= 0x80;

    reVal = I2C_MasterRepeatedStart(ACCELEROMETER_I2C_MASTER, ACCLR_ADDR, kI2C_Write);
	if (reVal != kStatus_Success)
	{
		return -1;
	}

	reVal = I2C_MasterWriteBlocking(ACCELEROMETER_I2C_MASTER, &offset, 1, kI2C_TransferNoStopFlag);
	if (reVal != kStatus_Success)
	{
		return -1;
	}

	reVal = I2C_MasterRepeatedStart(ACCELEROMETER_I2C_MASTER, ACCLR_ADDR, kI2C_Read);
	if (reVal != kStatus_Success)
	{
		return -1;
	}

	reVal = I2C_MasterReadBlocking(ACCELEROMETER_I2C_MASTER, outputPointer, length, 0);
	if (reVal != kStatus_Success)
	{
		return -1;
	}

	return kStatus_Success;
}
//****************************************************************************//
//
//  readRegisterInt16
//
//  Parameters:
//    *outputPointer -- Pass &variable (base address of) to save read data to
//    offset -- register to read
//
//****************************************************************************//
status_t readRegisterInt16( int16_t* outputPointer, uint8_t offset )
{
	{
		//offset |= 0x80; //turn auto-increment bit on
		uint8_t myBuffer[2];
		status_t returnError = readRegisterRegion(myBuffer, offset, 2);  //Does memory transfer
		int16_t output = (int16_t)myBuffer[0] | (int16_t)(myBuffer[1] << 8);
		*outputPointer = output;
		return returnError;
	}

}

float calcAccel( int16_t input )
{
	float output;
	switch(accslr_settings.accelRange) //2g default
	{
		case 2:
		output = (float)input / 15987;
		break;
		case 4:
		output = (float)input / 7840;
		break;
		case 8:
		output = (float)input / 3883;
		break;
		case 16:
		output = (float)input / 1280;
		break;
		default:
		output = 0;
		break;
	}
	return output;
}

int16_t readRawAccelX( void )
{
	int16_t output;
	status_t errorLevel = readRegisterInt16( &output, LIS3DH_OUT_X_L );
	if( errorLevel != kStatus_Success )
	{
		return -1;
	}
	return output;
}

float readFloatAccelX( void )
{
	float output = calcAccel(readRawAccelX());
	return output;
}


int16_t readRawAccelY( void )
{
	int16_t output;
	status_t errorLevel = readRegisterInt16( &output, LIS3DH_OUT_Y_L );
	if( errorLevel != kStatus_Success )
	{
		return -1;
	}
	return output;
}

float readFloatAccelY( void )
{
	float output = calcAccel(readRawAccelY());
	return output;
}

int16_t readRawAccelZ( void )
{
	int16_t output;
	status_t errorLevel = readRegisterInt16( &output, LIS3DH_OUT_Z_L );
	if( errorLevel != kStatus_Success )
	{
		return -1;
	}
	return output;

}

float readFloatAccelZ( void )
{
	float output = calcAccel(readRawAccelZ());
	return output;
}


//****************************************************************************//
//
//  Configuration section
//
//  This uses the stored SensorSettings to start the IMU
//  Use statements such as "myIMU.settings.commInterface = SPI_MODE;" or
//  "myIMU.settings.accelEnabled = 1;" to configure before calling .begin();
//
//****************************************************************************//
void applySettings( void )
{

	//Construct with these default settings
	//ADC stuff
	accslr_settings.adcEnabled = 1;

	//Temperature settings
	accslr_settings.tempEnabled = 1;

	//Accelerometer settings
	accslr_settings.accelSampleRate = 5000;  //Hz.  Can be: 0,1,10,25,50,100,200,400,1600,5000 Hz
	accslr_settings.accelRange = 16;      //Max G force readable.  Can be: 2, 4, 8, 16

	accslr_settings.xAccelEnabled = 1;
	accslr_settings.yAccelEnabled = 1;
	accslr_settings.zAccelEnabled = 1;

	//FIFO control settings
	accslr_settings.fifoEnabled = 0;
	accslr_settings.fifoThreshold = 20;  //Can be 0 to 32
	accslr_settings.fifoMode = 0;  //FIFO mode.



	uint8_t dataToWrite = 0;  //Temporary variable

	//Build TEMP_CFG_REG
	dataToWrite = 0; //Start Fresh!
	dataToWrite = ((accslr_settings.tempEnabled & 0x01) << 6) | ((accslr_settings.adcEnabled & 0x01) << 7);
	//Now, write the patched together data
	writeRegister(LIS3DH_TEMP_CFG_REG, dataToWrite);

	//Build CTRL_REG1
	dataToWrite = 0; //Start Fresh!
	//  Convert ODR
	switch(accslr_settings.accelSampleRate)
	{
		case 1:
		dataToWrite |= (0x01 << 4);
		break;
		case 10:
		dataToWrite |= (0x02 << 4);
		break;
		case 25:
		dataToWrite |= (0x03 << 4);
		break;
		case 50:
		dataToWrite |= (0x04 << 4);
		break;
		case 100:
		dataToWrite |= (0x05 << 4);
		break;
		case 200:
		dataToWrite |= (0x06 << 4);
		break;
		default:
		case 400:
		dataToWrite |= (0x07 << 4);
		break;
		case 1600:
		dataToWrite |= (0x08 << 4);
		break;
		case 5000:
		dataToWrite |= (0x09 << 4);
		break;
	}

	dataToWrite |= (accslr_settings.zAccelEnabled & 0x01) << 2;
	dataToWrite |= (accslr_settings.yAccelEnabled & 0x01) << 1;
	dataToWrite |= (accslr_settings.xAccelEnabled & 0x01);
	//Now, write the patched together data

	writeRegister(LIS3DH_CTRL_REG1, dataToWrite);

	//Build CTRL_REG4
	dataToWrite = 0; //Start Fresh!
	//  Convert scaling
	switch(accslr_settings.accelRange)
	{
		case 2:
		dataToWrite |= (0x00 << 4);
		break;
		case 4:
		dataToWrite |= (0x01 << 4);
		break;
		case 8:
		dataToWrite |= (0x02 << 4);
		break;
		default:
		case 16:
		dataToWrite |= (0x03 << 4);
		break;
	}
	dataToWrite |= 0x80; //set block update
	dataToWrite |= 0x08; //set high resolution

	//Now, write the patched together data
	writeRegister(LIS3DH_CTRL_REG4, dataToWrite);


	//Add things if needed bellow this line

}

//****************************************************************************//
//
//  Setup for callback configuration for int2
//
//
//****************************************************************************//
void clickCallbackSetup(void){
    /* Connect trigger sources to PINT */
    INPUTMUX_Init(INPUTMUX);
    INPUTMUX_AttachSignal(INPUTMUX, kPINT_PinInt2, DEMO_PINT_PIN_INT2_SRC);

    /* Turnoff clock to inputmux to save power. Clock is only needed to make changes */
    INPUTMUX_Deinit(INPUTMUX);

    /* Initialize PINT */
    PINT_Init(PINT);

    /* Setup Pin Interrupt 2 for rising edge */
    PINT_PinInterruptConfig(PINT, kPINT_PinInt2, kPINT_PinIntEnableRiseEdge, acc_int2_callback);

    /* Enable callbacks for PINT */
    PINT_EnableCallback(PINT);
}

//****************************************************************************//
//
//  Configuration of click interrupts
//
//	Configures the click-function as well as placing the interrupt on the
// 	on the int-2 pin.
//
//****************************************************************************//
void accslr_configIntterupts(void)
{
	//Sets the CLICK interrupts on INT2-Pin
	writeRegister(LIS3DH_CTRL_REG6, 0x80);

	//Config for interupt2 - High event interrupt
	//writeRegister(LIS3DH_INT2_CFG, 0x42);
	//writeRegister(LIS3DH_INT2_CFG, 0xa5); //And interrupt event: z-high & y-low & x-low
	writeRegister(LIS3DH_INT2_CFG, 0x20); // z-high event

	//Config. for interrupt threshold
	writeRegister(LIS3DH_INT2_THS, 0xff); //Max threshold
}

//****************************************************************************//
// Configures the click options
//
//  Parameters:
//  	int single_click. if = 1, single click is enabled, 0 = double click
//****************************************************************************//
void accslr_configClick(int single_click)
{
	if (single_click)
	{
		/* Configuration for single-click*/
		//Setting the time limit
		writeRegister(LIS3DH_TIME_LIMIT, 0x20); //(0x01)Short time 2.5ms

		//Setting the time latency, for de-bouncing and jittering
		//writeRegister(LIS3DH_TIME_LATENCY, 0x1f); //short latency 52ms
		writeRegister(LIS3DH_TIME_LATENCY, 0x1f);

		//Enable single-click
		//writeRegister(LIS3DH_CLICK_CFG, 0x15);//axis: x,y and z
		writeRegister(LIS3DH_CLICK_CFG, 0x10);//axis = z

		/*Interupt high until src is read(0x80), also adjusts the click threshold*/
		//writeRegister(LIS3DH_CLICK_THS,0x4f);
		writeRegister(LIS3DH_CLICK_THS,0x2a);

	}else
	{
		/* Configuration for double-click*/
		//Setting the time limit
		//writeRegister(LIS3DH_TIME_LIMIT, 0x01); //Short time 2.5ms
		//writeRegister(LIS3DH_TIME_LIMIT, 0x33); //Long time 127ms
		writeRegister(LIS3DH_TIME_LIMIT, 0x20);

		//Setting the time latency, debouncing
		//writeRegister(LIS3DH_TIME_LATENCY, 0x15); //short latency 52ms
		//writeRegister(LIS3DH_TIME_LATENCY, 0xff); //Long latency 637ms
		writeRegister(LIS3DH_TIME_LATENCY, 0x1f);
		//writeRegister(LIS3DH_TIME_LATENCY, 0x15);

		//Setting the time window click, max time detection procedure can start
		//writeRegister(LIS3DH_TIME_WINDOW, 0x42); //short window 165ms
		writeRegister(LIS3DH_TIME_WINDOW, 0xff); //long window 637ms
		//writeRegister(LIS3DH_TIME_WINDOW, 0x5a); //200ms

		writeRegister(LIS3DH_CLICK_CFG, 0x00); //reset
		//Enable double-click
		//writeRegister(LIS3DH_CLICK_CFG, 0x2a);
		writeRegister(LIS3DH_CLICK_CFG, 0x20); //z-axis only

		/*Interupt high until src is read(0x80), also adjusts the click threshold*/
		//writeRegister(LIS3DH_CLICK_THS,0x7f); //Max Threshold
		//writeRegister(LIS3DH_CLICK_THS,0x4f);
		writeRegister(LIS3DH_CLICK_THS,0x2a);
	}
}

volatile uint8_t accslr_irq = 0;
void acc_int0_callback(void)
{
	accslr_irq = 1;
}


void acc_int1_callback(void)
{
    //PRINTF("\f\r\n ACCE INT1 event detected\r\n");
}

void acc_int2_callback(pint_pin_int_t pintr, uint32_t pmatch_status)
{
	accslr_irq2 = 1;
    //PRINTF("\f\r\nPINT Pin Interrupt %d event detected.", pintr);
}


status_t accelerometer_init(void){
    i2c_master_config_t masterConfig;
    status_t reVal = kStatus_Fail;

    /* attach 12 MHz clock to FLEXCOMM0 (I2C master) */
    CLOCK_AttachClk(kFRO12M_to_FLEXCOMM0);

    /* reset FLEXCOMM for I2C */
    RESET_PeripheralReset(kFC0_RST_SHIFT_RSTn);

	/*
	 * masterConfig.debugEnable = false;
	 * masterConfig.ignoreAck = false;
	 * masterConfig.pinConfig = kI2C_2PinOpenDrain;
	 * masterConfig.baudRate_Bps = 100000U;
	 * masterConfig.busIdleTimeout_ns = 0;
	 * masterConfig.pinLowTimeout_ns = 0;
	 * masterConfig.sdaGlitchFilterWidth_ns = 0;
	 * masterConfig.sclGlitchFilterWidth_ns = 0;
	 */
	I2C_MasterGetDefaultConfig(&masterConfig);

	/* Change the default baudrate configuration */
	masterConfig.baudRate_Bps = ACCE_I2C_BAUDRATE;

	/* Initialize the I2C master peripheral */
	I2C_MasterInit(ACCELEROMETER_I2C_MASTER, &masterConfig, ACCELEROMETER_MASTER_CLOCK_FREQUENCY);


    //Check the ID register to determine if the operation was a success.
	uint8_t readCheck;
	readRegister(&readCheck, LIS3DH_WHO_AM_I);
	if( readCheck != 0x33 )
	{
		return -1;
	}
	//PRINTF("\r\ni2c addr %d .\r\n", readCheck);

	applySettings();

	/*CLICK configurattion start*/
	clickCallbackSetup();
	accslr_configIntterupts();
	//Arguments: 1 for single click. 0 for double click
	accslr_configClick(0);
	/*CLICK configurattion end*/

	//reset IRQ pending
    uint8_t dataRead;
	readRegister(&dataRead, LIS3DH_INT1_SRC);//cleared by reading

	return kStatus_Success;
}
