//*****************************************************************************
//
// Copyright (C) 2014 Texas Instruments Incorporated - http://www.ti.com/ 
// 
// 
//  Redistribution and use in source and binary forms, with or without 
//  modification, are permitted provided that the following conditions 
//  are met:
//
//    Redistributions of source code must retain the above copyright 
//    notice, this list of conditions and the following disclaimer.
//
//    Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the 
//    documentation and/or other materials provided with the   
//    distribution.
//
//    Neither the name of Texas Instruments Incorporated nor the names of
//    its contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
//  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
//  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
//  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
//  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
//  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
//  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
//  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
//  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
//  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
//  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
//*****************************************************************************

//*****************************************************************************
//
// Application Name     - ADC
// Application Overview - The application is a reference to usage of ADC DriverLib 
//                        functions on CC3200. Developers/Users can refer to this 
//                        simple application and re-use the functions in 
//                        their applications.
//
// Application Details  -
// http://processors.wiki.ti.com/index.php/CC32xx_ADC
// or
// docs\examples\CC32xx_ADC.pdf
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup adc_example
//! @{
//
//*****************************************************************************

// Standard includes
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

// Driverlib includes
#include "utils.h"
#include "hw_memmap.h"
#include "hw_common_reg.h"
#include "hw_types.h"
#include "hw_adc.h"
#include "hw_ints.h"
#include "hw_gprcm.h"
#include "rom.h"
#include "rom_map.h"
#include "interrupt.h"
#include "prcm.h"
#include "uart.h"
#include "pinmux.h"
#include "pin.h"
#include "adc.h"

#include "adc_userinput.h"
#include "uart_if.h"

#include "hw_types.h"
#include "hw_ints.h"
#include "hw_uart.h"
#include "uart.h"
#include "prcm.h"
#include "rom.h"
#include "rom_map.h"
#include "systick.h"
#include "utils.h"
#include "udma.h"
#include "interrupt.h"

// Common interface includes
#include "udma_if.h"
#include "uart_if.h"
#include "pinmux.h"


#define USER_INPUT 
#define UART_PRINT         Report
#define FOREVER            1
#define APP_NAME           "ADC Reference"
#define NO_OF_SAMPLES 		128

unsigned long pulAdcSamples[4096];

//*****************************************************************************
//                      GLOBAL VARIABLES
//*****************************************************************************
#if defined(ccs)
extern void (* const g_pfnVectors[])(void);
#endif
#if defined(ewarm)
extern uVectorEntry __vector_table;
#endif

/****************************************************************************/
/*                      LOCAL FUNCTION PROTOTYPES                           */
/****************************************************************************/
static void BoardInit(void);
static void DisplayBanner(char * AppName);

//*****************************************************************************
//
//! Application startup display on UART
//!
//! \param  none
//!
//! \return none
//!
//*****************************************************************************
static void
DisplayBanner(char * AppName)
{
    Report("\n\n\n\r");
    Report("\t\t *************************************************\n\r");
    Report("\t\t       CC3200 %s Application       \n\r", AppName);
    Report("\t\t *************************************************\n\r");
    Report("\n\n\n\r");
}

//*****************************************************************************
//
//! Board Initialization & Configuration
//!
//! \param  None
//!
//! \return None
//
//*****************************************************************************
static void
BoardInit(void)
{
/* In case of TI-RTOS vector table is initialize by OS itself */
#ifndef USE_TIRTOS
    //
    // Set vector table base
    //
#if defined(ccs)
    MAP_IntVTableBaseSet((unsigned long)&g_pfnVectors[0]);
#endif
#if defined(ewarm)
    MAP_IntVTableBaseSet((unsigned long)&__vector_table);
#endif
#endif
    //
    // Enable Processor
    //
    MAP_IntMasterEnable();
    MAP_IntEnable(FAULT_SYSTICK);

    PRCMCC3200MCUInit();
}

//*****************************************************************************
//
//! main - calls Crypt function after populating either from pre- defined vector 
//! or from User
//!
//! \param  none
//!
//! \return none
//!
//*****************************************************************************
void 
main()
{
BoardInit();
PinMuxConfig();
InitTerm();
DisplayBanner(APP_NAME);
InitAdcDma();
}

unsigned long DmaDataDumpPing[640];
unsigned long DmaDataDumpPong[640];
void ADCIntHandler(void)
{
unsigned long ulChannelStructIndex, ulMode, ulControl;
tDMAControlTable *pControlTable;
unsigned long *pDataDumpBuff = NULL;
unsigned short Status;
unsigned short uiIndex;

Status = ADCIntStatus(ADC_BASE, ADC_CH_2);
ADCIntClear(ADC_BASE, ADC_CH_2,Status|ADC_DMA_DONE);
ulMode = MAP_uDMAChannelModeGet(UDMA_CH16_ADC_CH2 | UDMA_PRI_SELECT);
if(ulMode == UDMA_MODE_STOP)
{
ulChannelStructIndex = UDMA_CH16_ADC_CH2 | UDMA_PRI_SELECT;
pDataDumpBuff = &(DmaDataDumpPing[0]);
}
else
{
ulMode = MAP_uDMAChannelModeGet(UDMA_CH16_ADC_CH2 | UDMA_ALT_SELECT);
if(ulMode == UDMA_MODE_STOP)
{
ulChannelStructIndex = UDMA_CH16_ADC_CH2 | UDMA_ALT_SELECT;
pDataDumpBuff = &(DmaDataDumpPong[0]);
}
}

if(pDataDumpBuff != NULL)
{
ulChannelStructIndex &= 0x3f;
pControlTable = uDMAControlBaseGet();
ulControl = (pControlTable[ulChannelStructIndex].ulControl &
~(UDMA_CHCTL_XFERSIZE_M | UDMA_CHCTL_XFERMODE_M));
ulControl |= UDMA_MODE_PINGPONG | ((640 - 1) << 4);
uDMAChannelControlSet(ulChannelStructIndex,ulControl);
#if 1
UART_PRINT("\n\rVoltage is %f\n\r",(((float)((pDataDumpBuff[0] >> 2 ) & 0x0FFF))*1.4)/4096); 
#endif 
}
}

void InitAdcDma( void )
{
unsigned short Status;

UDMAInit();
PinTypeADC(PIN_59,0xFF);
MAP_uDMAChannelAssign(UDMA_CH16_ADC_CH2);
UDMASetupTransfer(UDMA_CH16_ADC_CH2|UDMA_PRI_SELECT, UDMA_MODE_PINGPONG, 640, UDMA_SIZE_32, UDMA_ARB_1, (void *)(0x4402E874+ADC_CH_2), UDMA_SRC_INC_NONE, (void *)&(DmaDataDumpPing[0]), UDMA_DST_INC_32);

UDMASetupTransfer(UDMA_CH16_ADC_CH2|UDMA_ALT_SELECT, UDMA_MODE_PINGPONG, 640, UDMA_SIZE_32, UDMA_ARB_1, (void *)(0x4402E874+ADC_CH_2), UDMA_SRC_INC_NONE, (void *)&(DmaDataDumpPong[0]), UDMA_DST_INC_32);

ADCDMAEnable(ADC_BASE, ADC_CH_2);
ADCIntRegister(ADC_BASE, ADC_CH_2,ADCIntHandler);
Status = ADCIntStatus(ADC_BASE, ADC_CH_2);
ADCIntClear(ADC_BASE, ADC_CH_2,Status|ADC_DMA_DONE);
ADCIntEnable(ADC_BASE, ADC_CH_2,ADC_DMA_DONE);
ADCChannelEnable(ADC_BASE, ADC_CH_2);
ADCEnable(ADC_BASE);
while(1);
}
