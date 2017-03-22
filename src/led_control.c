/******************************************************************************

 @file : led_control.c

 @brief : Control RGB led intensity by using PWM

 Group: WCS, BTS
 Target Device: CC2650, CC2640

******************************************************************************
 
 Copyright (c) 2013-2016, Texas Instruments Incorporated
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:

 *  Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

 *  Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.

 *  Neither the name of Texas Instruments Incorporated nor the names of
    its contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 ******************************************************************************/
/*********************************************************************
 * INCLUDES
 */
#include <stdio.h>
#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerCC26XX.h>
#include <ti/sysbios/knl/Clock.h>
#include <driverlib/cpu.h>
#include <ti/drivers/pin/PINCC26XX.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/BIOS.h>
#include <ti/drivers/PIN.h>
#include <driverLib/timer.h>   

#include "Board.h"

/*********************************************************************
 * MACROS
 */

//-------------- Task configuration --------------------
#define ble_TASK_PRIORITY                     1
#define ble_TASK_STACK_SIZE                   512

Task_Struct bleTask;
Char bleTaskStack[ble_TASK_STACK_SIZE];

/* -----------------------------------------------------------------------------
*                           Constants and macros
* ------------------------------------------------------------------------------
*/
#define PWM_DIV_FACTOR      256
#define TIMER_LOADSET       (PWM_DIV_FACTOR-1)

#define delay_ms(i) ( CPUdelay(10000*(i)) )

//--------------- global variables ---------------------
PIN_Handle periHandle;
PIN_State pinState;

//--------------- Pin configuration --------------------
static PIN_Config pin_configTable[] =
{
    RED_LED      | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW   | PIN_PUSHPULL | PIN_DRVSTR_MAX,     /* LED initially off */
    GREEN_LED    | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW   | PIN_PUSHPULL | PIN_DRVSTR_MAX,     /* LED initially off */
    BLUE_LED     | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW   | PIN_PUSHPULL | PIN_DRVSTR_MAX,     /* LED initially off */        
  PIN_TERMINATE
};

/*********************************************************************
 * @fn      intensity_change
 *
 * @brief   RGB light intensity change
 */
void intensity_change(uint8_t id, uint8_t green_value, uint8_t blue_value, uint8_t red_value)
{
  if(periHandle != 0) {
    switch(id)
    {
      case GREEN_LED:
        TimerDisable(GPT0_BASE, TIMER_A);
        PINCC26XX_setMux(periHandle, id, IOC_PORT_MCU_PORT_EVENT0);       /* Configure pin for PWM output */
        TimerMatchSet(GPT0_BASE, TIMER_A, TIMER_LOADSET - green_value);
        TimerEnable(GPT0_BASE, TIMER_A);
        break;
        
      case BLUE_LED:
        TimerDisable(GPT0_BASE, TIMER_B);
        PINCC26XX_setMux(periHandle, id, IOC_PORT_MCU_PORT_EVENT1);       /* Configure pin for PWM output */
        TimerMatchSet(GPT0_BASE, TIMER_B, TIMER_LOADSET - blue_value);
        TimerEnable(GPT0_BASE, TIMER_B);
        break;
          
      case RED_LED:
        TimerDisable(GPT1_BASE, TIMER_A);
        PINCC26XX_setMux(periHandle, id, IOC_PORT_MCU_PORT_EVENT2);       /* Configure pin for PWM output */
        TimerMatchSet(GPT1_BASE, TIMER_A, TIMER_LOADSET - red_value);
        TimerEnable(GPT1_BASE, TIMER_A);
        break;
      }
  }
}

/*********************************************************************
 * @fn      timer_config
 *
 * @brief   Configure GPT Timer for RED, Green and Blue LED
 *
 * @param   none.
 *
 * @return  None.
 */
void timer_config(void)
{
  // Turn on PERIPH power domain and clock for GPT0
  Power_setDependency(PowerCC26XX_PERIPH_GPT0);
  Power_setDependency(PowerCC26XX_PERIPH_GPT1);
  
  // Assign GPT0 to Green and Blue LED
  TimerConfigure(GPT0_BASE, TIMER_CFG_SPLIT_PAIR|TIMER_CFG_A_PWM|TIMER_CFG_B_PWM);
  TimerLoadSet(GPT0_BASE, TIMER_BOTH, TIMER_LOADSET);
  
  // Assign GPT1 to Red LED
  TimerConfigure(GPT1_BASE, TIMER_CFG_SPLIT_PAIR|TIMER_CFG_A_PWM);
  TimerLoadSet(GPT1_BASE, TIMER_A, TIMER_LOADSET);
}

void led_close(void)
{
  if (periHandle != NULL)
  {
    // Turn off PERIPH power domain and clock for GPT0 and GPT1
    Power_releaseDependency(PowerCC26XX_PERIPH_GPT0);
    Power_releaseDependency(PowerCC26XX_PERIPH_GPT1);

    PIN_close(periHandle);
    periHandle = NULL;
  }
}
  
  
/*********************************************************************
 * @fn      MainApp_taskFxn_taskFxn
 *
 * @brief   Application task entry point for the Main Application.
 *
 * @param   a0, a1 - not used.
 *
 * @return  None.
 */
static void ble_taskFxn(UArg a0, UArg a1)
{    
  uint8_t i = 0;   
    
  periHandle = PIN_open(&pinState, pin_configTable);        /* Board Configuration for LEDs */

  timer_config();
  
  while(1) {
    for(i = 0; i < 255; i++) {
      intensity_change(RED_LED, 0, 0, i);
      delay_ms(15);
    }
    delay_ms(50);
    for(i = 255; i > 0; i--) {
      intensity_change(RED_LED, 0, 0, i);
      delay_ms(15);
    }
    delay_ms(50);
  }
}

/*********************************************************************
 * @fn      RBG_control_createTask
 *
 * @brief   Task creation function for the main Application.
 *
 * @param   None.
 *
 * @return  None.
 */
void RGB_control_createTask(void)
{
  Task_Params taskParams;

  // Configure task
  Task_Params_init(&taskParams);
  taskParams.stack = bleTaskStack;
  taskParams.stackSize = ble_TASK_STACK_SIZE;
  taskParams.priority = ble_TASK_PRIORITY;

  Task_construct(&bleTask, ble_taskFxn, &taskParams, NULL);
}
