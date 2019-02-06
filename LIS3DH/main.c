/*
 * The Clear BSD License
 * Copyright (c) 2013 - 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided
 * that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS LICENSE.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "board.h"

#include "pin_mux.h"
#include <stdbool.h>

#include "accelerometer.h"
#include "fsl_inputmux.h"
#include "fsl_pint.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define DEMO_PINT_PIN_INT2_SRC kINPUTMUX_GpioPort0Pin30ToPintsel

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/
volatile uint8_t accslr_irq2 = 0;
void pint_intr_callback(pint_pin_int_t pintr, uint32_t pmatch_status)
{
	accslr_irq2 = 1;
    PRINTF("\f\r\nPINT Pin Interrupt %d event detected.", pintr);
}


/*!
 * @brief Main function
 */
int main(void)
{

    /* Init board hardware. */
    /* attach 12 MHz clock to FLEXCOMM0 (debug console) */
    CLOCK_AttachClk(BOARD_DEBUG_UART_CLK_ATTACH);

    //Interupt clock
    CLOCK_AttachClk(kFRO12M_to_FLEXCOMM0);

    BOARD_InitPins();
    BOARD_BootClockFROHF48M();
    BOARD_InitDebugConsole();

    /*Callback setup start*/
    /* Connect trigger sources to PINT */
    INPUTMUX_Init(INPUTMUX);
    INPUTMUX_AttachSignal(INPUTMUX, kPINT_PinInt2, DEMO_PINT_PIN_INT2_SRC);
    /* Turnoff clock to inputmux to save power. Clock is only needed to make changes */
    INPUTMUX_Deinit(INPUTMUX);
    /* Initialize PINT */
    PINT_Init(PINT);
    /* Setup Pin Interrupt 2 for rising edge */
    PINT_PinInterruptConfig(PINT, kPINT_PinInt2, kPINT_PinIntEnableRiseEdge, pint_intr_callback);
    /* Enable callbacks for PINT */
    PINT_EnableCallback(PINT);
    /*Callback setup finished*/

    PRINTF("hello world.\r\n");
    accelerometer_init();

    uint8_t identity;
    readRegister(&identity, LIS3DH_WHO_AM_I);
	if(identity > 0){
		PRINTF("ACC identity checked\n");
	}
    PRINTF("ACC init done, waiting for IR\r\n");
    uint8_t teller = 0;
    uint8_t antiShake = 0;

    while (1)
    {
        //Checking if an interrupt is registered
        __WFI();

        if(accslr_irq2){
        	accslr_irq2 = 0;
        	teller++;
        	uint8_t clk_src;
        	readRegister(&clk_src, LIS3DH_CLICK_SRC);
        	PRINTF("Click nr.%d, reg: %d \r\n",teller,clk_src);

        }

        //PRINTF('AntiShake %d',antiShake);
		//if(antiShake == 0xff){
		//	PRINTF("Time");
		//}
		//antiShake++;
    }
}
