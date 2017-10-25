/**************************************************************************************************
  Filename:       SimpleBLEPeripheral_Main.c
  Revised:        $Date: 2010-07-06 15:39:18 -0700 (Tue, 06 Jul 2010) $
  Revision:       $Revision: 22902 $

  Description:    This file contains the main and callback functions for
                  the Simple BLE Peripheral sample application.

  Copyright 2010 - 2011 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED “AS IS" WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
**************************************************************************************************/

/**************************************************************************************************
 *                                           Includes
 **************************************************************************************************/
/* Hal Drivers */
#include "hal_types.h"
#include "hal_key.h"
#include "hal_timer.h"
#include "hal_drivers.h"
#include "hal_led.h"
#include "hci.h"

/* OSAL */
#include "OSAL.h"
#include "OSAL_Tasks.h"
#include "OSAL_PwrMgr.h"
#include "osal_snv.h"
#include "OnBoard.h"
 
#if defined ( PLUS_BROADCASTER )
  #include "peripheralBroadcaster.h"
#else
  #include "peripheral.h"
#endif

//extern uint16  Uart_RX_Bytes_client;
extern void delay_tm(unsigned int num);
//extern XDATA uint8  Uart_RX_Buffer_client[SBP_UART_RX_BUF_SIZE];

/**************************************************************************************************
 * FUNCTIONS
 **************************************************************************************************/


/* This callback is triggered when a key is pressed */
void MSA_Main_KeyCallback(uint8 keys, uint8 state);




/**************************************************************************************************
 * @fn          main
 *
 * @brief       Start of application.
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */
int main(void)
{
  unsigned int i=0,ii=0;
  //Uart_RX_Bytes_client=0;
  /* Initialize hardware */
  
  HAL_BOARD_INIT();
#if 1
  // Initialize board I/O
  InitBoard( OB_COLD );
 
  /* Initialze the HAL driver */
  HalDriverInit();

  /* Initialize NV system */
  osal_snv_init();

  /* Initialize LL */

  /* Initialize the operating system */
  osal_init_system();

  /* Enable interrupts */
  HAL_ENABLE_INTERRUPTS();

  // Final board initialization
  InitBoard( OB_READY );

  #if defined ( POWER_SAVING )
   
    osal_pwrmgr_device( PWRMGR_BATTERY );
  #endif
  #endif

   
//SerialApp_Init(1);
	i=10000;
  /* Start OSAL */
 // while(--i)
  //HCI_EXT_HaltDuringRfCmd( HCI_EXT_HALT_DURING_RF_DISABLE);
  i=1;
// sbpSerialAppWrite("success",7);	
	 
  while(1)   
  {
	

	osal_start_system();  
		//inmain_process();
/*	 ///////////////// for advertise name set
	 if((i==1)&&( gapProfileState != GAPROLE_ADVERTISING )&&name_buff[0])
	{
		for(i=0;i<strlen(name_buff);i++)
		{
			if(name_buff[i]=='\n')
			{
				
				 
				name_buff[i]=0;
				//name  must less than 16byes
				  name_buff[15]=0; 
			 	osal_memcpy(scanRspData+2,name_buff,10);
				
				 //strcpy(scanRspData+2,name_buff);
				//GAPROLE_ADVERT_ENABLED ==0x305
				//GAPROLE_SCAN_RSP_DATA ==0x308
                             GAPRole_SetParameter( 0x308, 27, scanRspData );

				i=2; 
			}
		}
		if((i==2)&&( gapProfileState != GAPROLE_CONNECTED ))
		{
				GAPRole_SetParameter( 0x305, sizeof( uint8 ), &initial_advertising_enable );
			i=0;
		}
		//GAPRole_SetParameter(0x308, sizeof(scanRspData), &scanRspData);
		 
	}
     //////////////////////////
	 */
	///////////send the last packet that less than 20bytes	 
	
	//////////////////////
	//	
  }
//  return 0;
}

/**************************************************************************************************
                                           CALL-BACKS
**************************************************************************************************/


/*************************************************************************************************
**************************************************************************************************/
