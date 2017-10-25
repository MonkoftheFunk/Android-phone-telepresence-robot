/**************************************************************************************************
  Filename:       simpleBLEPeripheral.c
  Revised:        $Date: 2010-08-06 08:56:11 -0700 (Fri, 06 Aug 2010) $
  Revision:       $Revision: 23333 $

  Description:    This file contains the Simple BLE Peripheral sample application
                  for use with the CC2540 Bluetooth Low Energy Protocol Stack.

  Copyright 2010 - 2012 Texas Instruments Incorporated. All rights reserved.

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
  PROVIDED 揂S IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
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

/*********************************************************************
 * INCLUDES
 */

#include "bcomdef.h"
#include "OSAL.h"
#include "OSAL_PwrMgr.h"

#include "OnBoard.h"
#include "hal_adc.h"
#include "hal_led.h"
#include "hal_key.h"
#include "hal_lcd.h"

#include "gatt.h"

#include "hci.h"

#include "gapgattserver.h"
#include "gattservapp.h"
#include "devinfoservice.h"
#include "simpleGATTprofile.h"

#if defined( CC2540_MINIDK )
  #include "simplekeys.h"
#endif

#if defined ( PLUS_BROADCASTER )
  #include "peripheralBroadcaster.h"
#else
  #include "peripheral.h"
#endif

#include "gapbondmgr.h"

#include "simpleBLEPeripheral.h"

#include "DHT11.h"

#if defined FEATURE_OAD
  #include "oad.h"
  #include "oad_target.h"
#endif

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

// How often to perform periodic event
#define SBP_PERIODIC_EVT_PERIOD                   5000

// What is the advertising interval when device is discoverable (units of 625us, 160=100ms)
#define DEFAULT_ADVERTISING_INTERVAL          160


// Limited discoverable mode advertises for 30.72s, and then stops
// General discoverable mode advertises indefinitely

#if defined ( CC2540_MINIDK )
#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_LIMITED
#else
#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_GENERAL
#endif  // defined ( CC2540_MINIDK )

// Minimum connection interval (units of 1.25ms, 80=100ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL       20 

// Maximum connection interval (units of 1.25ms, 800=1000ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     800

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_DESIRED_SLAVE_LATENCY         0

// Supervision timeout value (units of 10ms, 1000=10s) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_CONN_TIMEOUT          300

// Whether to enable automatic parameter update request when a connection is formed
#define DEFAULT_ENABLE_UPDATE_REQUEST        TRUE// FALSE

// Connection Pause Peripheral time value (in seconds)
#define DEFAULT_CONN_PAUSE_PERIPHERAL         6
// Company Identifier: Texas Instruments Inc. (13)
#define TI_COMPANY_ID                         0x000D

#define INVALID_CONNHANDLE                    0xFFFF

// Length of bd addr as a string
#define B_ADDR_STR_LEN                        15

#if defined ( PLUS_BROADCASTER )
  #define ADV_IN_CONN_WAIT                    500 // delay 500 ms
#endif

int8 rssi_ack;
 
   
uint32 Connect_flag=0;/*0:setting;1:disconnected; 2:connected*/
/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
  uint8 simpleBLEPeripheral_TaskID;   // Task ID for internal task/event processing

  gaprole_States_t gapProfileState = GAPROLE_INIT;

static  uint8 scanRspData_end[] =
{
  0x05,   // length of this data
  GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE,
  LO_UINT16( DEFAULT_DESIRED_MIN_CONN_INTERVAL ),   // 100ms
  HI_UINT16( DEFAULT_DESIRED_MIN_CONN_INTERVAL ),
  LO_UINT16( DEFAULT_DESIRED_MAX_CONN_INTERVAL ),   // 1s
  HI_UINT16( DEFAULT_DESIRED_MAX_CONN_INTERVAL ),
  // Tx power level
  0x02,   // length of this data
  GAP_ADTYPE_POWER_LEVEL,
  0       // 0dBm
};
// GAP - SCAN RSP data (max size = 31 bytes)
static  uint8 scanRspData_name[ 31];//;
#if 0
=
{
  // complete name
  0x11,   // length of this data
   GAP_ADTYPE_LOCAL_NAME_COMPLETE,
  //MBI_qLabs_BLE4.0
  0x4D,0x42,0x49,0x5F,0x71,0x4C,0x61,0x62,0x73,
  0x5F,0x42,0x4C,0x45,0x34,0x2E,0x30,
  
  0x52,   // 'R'
  0x53,   // 'S'
  0x68,   // 'h'
  0x65,   // 'e'
  0x65,   // 'e'
  0x65,   // 'p'
  0x70,   // 'B'
  0x4c,   // 'L'
  0x45,   // 'E'
  0x50,   // 'P'
  0x65,   // 'e'
  0x72,   // 'r'
  0x69,   // 'i'
  0x70,   // 'p'
  0x68,   // 'h'
  0x65,   // 'e'
  0x72,   // 'r'
  0x61,   // 'a'
  0x6c,   // 'l'

  // connection interval range
  0x05,   // length of this data
  GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE,
  LO_UINT16( DEFAULT_DESIRED_MIN_CONN_INTERVAL ),   // 100ms
  HI_UINT16( DEFAULT_DESIRED_MIN_CONN_INTERVAL ),
  LO_UINT16( DEFAULT_DESIRED_MAX_CONN_INTERVAL ),   // 1s
  HI_UINT16( DEFAULT_DESIRED_MAX_CONN_INTERVAL ),

  // Tx power level
  0x02,   // length of this data
  GAP_ADTYPE_POWER_LEVEL,
  0       // 0dBm
};
#endif
// GAP - Advertisement data (max size = 31 bytes, though this is
// best kept short to conserve power while advertisting)
static uint8 advertData[] =
{
  // Flags; this sets the device to use limited discoverable
  // mode (advertises for 30 seconds at a time) instead of general
  // discoverable mode (advertises indefinitely)
  0x02,   // length of this data
  GAP_ADTYPE_FLAGS,
  DEFAULT_DISCOVERABLE_MODE | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,

  // service UUID, to notify central devices what services are included
  // in this peripheral
  0x03,   // length of this data
  GAP_ADTYPE_16BIT_MORE,      // some of the UUID's, but not all
  LO_UINT16( SIMPLEPROFILE_SERV_UUID ),
  HI_UINT16( SIMPLEPROFILE_SERV_UUID ),

};

extern void delay_tm(unsigned int num);
// GAP GATT Attributes
static uint8 attDeviceName[GAP_DEVICE_NAME_LEN] = "MBI_qLabsBLE4.1";

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void simpleBLEPeripheral_ProcessOSALMsg( osal_event_hdr_t *pMsg );
static void peripheralStateNotificationCB( gaprole_States_t newState );
static void performPeriodicTask( void );
static void simpleProfileChangeCB( uint8 paramID,uint8 length );

#if defined( CC2540_MINIDK )
static void simpleBLEPeripheral_HandleKeys( uint8 shift, uint8 keys );
#endif

#if (defined HAL_LCD) && (HAL_LCD == TRUE)
static char *bdAddr2Str ( uint8 *pAddr );
#endif // (defined HAL_LCD) && (HAL_LCD == TRUE)

static void Callback_RssiCB( int8 rssi );


/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static gapRolesCBs_t simpleBLEPeripheral_PeripheralCBs =
{
  peripheralStateNotificationCB,  // Profile State Change Callbacks
  Callback_RssiCB//NULL                            // When a valid RSSI is read from controller (not used by application)
};

// GAP Bond Manager Callbacks
static gapBondCBs_t simpleBLEPeripheral_BondMgrCBs =
{
  NULL,                     // Passcode callback (not used by application)
  NULL                      // Pairing / Bonding state Callback (not used by application)
};

// Simple GATT Profile Callbacks
static simpleProfileCBs_t simpleBLEPeripheral_SimpleProfileCBs =
{
  simpleProfileChangeCB    // Charactersitic value change callback
};

unsigned int	pwm;
unsigned int count;      //0.5ms 
unsigned char jd;         // 
Receive_D  Receive_CMD[max_num];
 uint8 newValueBuf[SIMPLEPROFILE_CHAR1_LEN] ;
static car_status   car;
douji_status  douji;
int idle_buf=0;

#define motor_A0  P0_2
#define motor_A1  P0_3
#define motor_B0  P0_4
#define motor_B1  P0_5
#define duoji_A	P0_1
#define car_led	P0_0
#define motor_Set(pin_d, mode) pin_d=mode

void GPIO_GPIOInit()
{
  /*先关所有led，防止初始化时led闪烁*/
	motor_A0 =0; 
	motor_A1 =0; 
	motor_B0 =0; 
	motor_B1 =0; 
	duoji_A =1; 
	car_led=0;
	  /*设置LEDs引脚为输出*/
	  P0DIR |= BV(2)|BV(3)|BV(4)|BV(5);
	 // P0DIR |= BV(0)|BV(1);
	  
	  /*设置LEDs引脚为GPIO引脚*/
	  P0SEL &= ~(BV(2)|BV(3)|BV(4)|BV(5));
	 // P0SEL &= ~(BV(0)|BV(1));
	//  P1SEL = 0; // Configure Port 1 as GPIO
	//   P1DIR = 0xFF;
	   
	motor_A0 =1; 
	motor_A1 =1; 
	motor_B0 =1; 
	motor_B1 =1; 
	duoji_A =1; 
	car_led=1;
        motor_A0 =0; 
	motor_A1 =0; 
	motor_B0 =0; 
	motor_B1 =0; 
	duoji_A =1; 
	car_led=0;

	douji.x=5;
	douji.x_old=5;
	douji.x_n=0;
      
}

void led_flash()
{
	car_led=1;
	delay_tm(300);
	car_led=0;
	delay_tm(300);
	car_led=1;
	delay_tm(300);
	car_led=0;

}
void  car_stop()
{
	motor_Set(motor_A0, 0);
	motor_Set(motor_A1, 0);
	motor_Set(motor_B0, 0);
	motor_Set(motor_B1, 0);
	car.left=car.right=0;
	car.carn=0;
}


/*void motor_Set( uint8 pin_d, uint8 mode )
{
 // 

   // cmd_st.key_timer=0;

  //  PIN_setOutputValue(hKeyPins, pin_d, mode);
	 pin_d=mode;
}
*/
void cmd_pase(char *data_c,int len)
{
 	int i;
	 
	//char *data_c=Receive_CMD[0].receive_buf;

	//memset(RX2_Buffer,0,BUF_LENTH);

			

	for(i=0;i<len;i++)
	{
		if(data_c[i]=='t')		   // turn right or left  
		{
			
			car.right=atoi(data_c+i+1);
                        car.left=-car.right;
			//motor_Set(duoji_A, 1);
			douji.x=0; 
			car.carn=0;
			break;
		}
		
		else if(data_c[i]=='b')	   // both motor speed
		{
		 	car.right=car.left=  atoi(data_c+i+1);
			//motor_Set(duoji_A, 1);
			douji.x=0; 
			car.carn=0;
			//if(car.right==2)
			//	car_led = ~car_led;  

			break;
		}
		

			
	}
}

void receing_data(char datar)
{

	if(datar==';')
	{
		Receive_CMD[0].receive_buf[Receive_CMD[0].receive_len++] = 0;	
		Receive_CMD[0].receive_status=2;
		idle_buf++;
		/////
		
		cmd_pase(Receive_CMD[0].receive_buf,Receive_CMD[0].receive_len);
		Receive_CMD[0].receive_status=0;
		Receive_CMD[0].receive_len=0;
		
		
	}
	else
	//else  if(Receive_CMD[0].receive_status<2)
	
	{
		Receive_CMD[0].receive_buf[Receive_CMD[0].receive_len++] = datar;	
		Receive_CMD[0].receive_status=1;
		if(Receive_CMD[0].receive_len>max_rebuf) 
			Receive_CMD[0].receive_len=0;

	}

        
}

#if 1


void car_control()
{

 // car_led = (car.left||car.right);  
	car.left_count++;
	if((car.left_count>=8))
	{
		car.left_count=1;
        if(car.left!=1)
        {
          motor_Set(motor_A0, 0);
          motor_Set(motor_A1, 0);
        }
	}

	if(car.left==0)
	{ // stop
		motor_Set(motor_A0, 0);
		motor_Set(motor_A1, 0);
	}	 

	  if(car.left==car.left_count)
		motor_Set(motor_A0, 1);
 		//motor_A0=1;	

	  if(car.left==-car.left_count)
		motor_Set(motor_A1, 1);
 		//motor_A1=1;		



////////////*/

    car.right_count++;	
    if((car.right_count>=8))
	{
		car.right_count=1;
                if(car.right!=1)
                {
                 // car.right=0;
                  motor_Set( motor_B0, 0 );
                  motor_Set( motor_B1, 0 );
                      
		}

	}


        if(car.right==car.right_count)
        {
		motor_Set(motor_B0, 1);
        //car_led = 1;         
        }
 	//	motor_B0=1;		

	  if(car.right==(-car.right_count))
        {
		motor_Set(motor_B1, 1);
              // car_led = 1;   
        }

	  if(car.right==0)
	{ // stop
		motor_Set(motor_B0, 0);
		motor_Set(motor_B1, 0);
	            
	}	 

	
	

	 



}

void duoji_control()
{
      //  if(douji.xn<50)  
      char x_tmp=0;
	douji.x_count++;
	if(((douji.x_count )==100)&&(douji.xn<50))
	{
		//duoji_A=1;
		motor_Set(duoji_A, 1);
	 	douji.x_count=0;
		douji.xn++;
		x_tmp=douji.xn/5;
		if(x_tmp<=douji.x_n) 
			douji.x=douji.x_old+x_tmp;
	}
	else if(douji.x==douji.x_count)
		motor_Set(duoji_A, 0);

	else if(douji.xn>=50)
	{
		motor_Set(duoji_A, 1);
		douji.x=0;
	}

}


#endif


uint32 counter = 0;
uint8  timeout = 0;
//  uint32 car_idle=0;
void HalTimer3Init()
{
  //T3CTL &= ~0x03;
 T3CTL = 0xAE;	//32分频，中断使能，模模式
  T3CC0 = 0xC8;	// 1Mhz==1us  200us中断
  T3CCTL0 = 0x44;//一定要设置比较的方式，不然不会中断的!!!,T3CCTL0.IM==1,T3CCTL0.MODE==1
  /*

	   T3CTL  = 0xed;	  //定时器3 128分频，倒计数模式，溢出中断使能，计数器清0
	  
	    T3CC0 = 0x80;	  //定时器3通道0捕获/比较值
	  
	    IEN1  |= (1<<3);   //定时器3中断使能
	  
	    EA = 1;			  //开总中断
	  
	    T3CTL |= 0x10;	  //启动定时器3
*/

}
/*开始计时*/
void HalTimer3Start()
{
  T3CTL |= 0x10; 
}
/*停止计时*/
void HalTimer3Stop()
{
  T3CTL &= ~0x10; 
}
/*中断使能*/
void HalTimer3EnableInt()
{
  EA = 1;
  T3IE = 1;
  //IEN1 |= BV(3);
}
/*中断不使能*/
void HalTimer3DisableInt()
{
  T3IE = 0;
  //IEN1 &= ~ BV(3);
}




/*中断函数//中断5次为1ms*/
#pragma vector = T3_VECTOR 
 __interrupt void T3_ISR(void) 
{ 
  IRCON &= ~0x08;	   //定时器3中断标志位清0
  TIMIF &= ~0x01; 	  //定时器3溢出中断标志位清0
 /* if(car.left==0&&car.right==0)
    car_idle=0;
  else
    car_idle=1;
  */  
  if(car.left)
  {
    
    if(car.left!=car.right)
      car.carn+=2;
    else
      car.carn++;
    if(car.carn>5000)//1s timeout
    {
      car_stop();
    }
    else
      car_control();
  }           
  T3CTL |= 0x10;        //再次启动定时器3


} 


int inmain_process(void)
{
	static int i=0;
	if( Receive_CMD[i].receive_status==2)	 
	{
		 cmd_pase(Receive_CMD[i].receive_buf,Receive_CMD[i].receive_len);
		 Receive_CMD[i].receive_status=0;
		 Receive_CMD[i].receive_len=0;
		// PrintString1("eeeee");

		 memset(Receive_CMD[i].receive_buf,0,max_rebuf);

	}

	i++;
	if(i>=max_num)
		i=0;
}
/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      SimpleBLEPeripheral_Init
 *
 * @brief   Initialization function for the Simple BLE Peripheral App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notificaiton ... ).
 *
 * @param   task_id - the ID assigned by OSAL.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return  none
 */
void SimpleBLEPeripheral_Init( uint8 task_id )
{
  simpleBLEPeripheral_TaskID = task_id;
  int i=1;
  uint8  name_buff[20];
  // Setup the GAP
  VOID GAP_SetParamValue( TGAP_CONN_PAUSE_PERIPHERAL, DEFAULT_CONN_PAUSE_PERIPHERAL );
	
  //serial port initialization
  ////////////////////////
	  //memmset(cmd_st,0,sizeof(cmd_st))
	  for(i=0;i<max_num;i++) 
	  {
		  Receive_CMD[i].receive_status=0;
		  Receive_CMD[i].receive_len =0;
	  }
	  car.right=0;
	  jd=4;
	  car.left=0;
  	  car.right_count=0;
	  car.left_count=0;
	  idle_buf=0;
  ///////////////
  GPIO_GPIOInit();
 // led_flash();	
	
 // SerialApp_Init(task_id);
//  /* Enable interrupts */
   /*定时器初始化*/
  HalTimer3Init();
  HalTimer3EnableInt();
  HalTimer3Start();
////////////////wait for advertise name set finished
/*	while(1)
	{
		delay_tm(100);
		sbpSerialAppCallback(0, 1);
		for(i=0;i<strlen(name_buff);i++)
		{
			if(name_buff[i]=='\n')
			{
				name_buff[i]=0;
				i=100;	
				sbpSerialAppWrite("success",7);	

				break;
			}
		}
 		if(i==100)
			break;
	}



*/	
	name_buff[15]=0;
	sprintf(name_buff,"RSheep9");
	i=strlen(name_buff);
	scanRspData_name[0]=1+i;
	scanRspData_name[1]=GAP_ADTYPE_LOCAL_NAME_COMPLETE;
	memcpy(scanRspData_name+2,name_buff,i);
	memcpy(scanRspData_name+2+i,scanRspData_end,9);
	i= 2+i+9;
///////////////////////	

	
  // Setup the GAP Peripheral Role Profile
  {

    #if defined( CC2540_MINIDK )
      // For the CC2540DK-MINI keyfob, device doesn't start advertising until button is pressed
      uint8 initial_advertising_enable = FALSE;
	
    #else
      // For other hardware platforms, device starts advertising upon initialization
      uint8 initial_advertising_enable = TRUE;
    #endif

     //  initial_advertising_enable = FALSE;

    // By setting this to zero, the device will go into the waiting state after
    // being discoverable for 30.72 second, and will not being advertising again
    // until the enabler is set back to TRUE
    uint16 gapRole_AdvertOffTime = 0;

    uint8 enable_update_request = DEFAULT_ENABLE_UPDATE_REQUEST;
    uint16 desired_min_interval = DEFAULT_DESIRED_MIN_CONN_INTERVAL;
    uint16 desired_max_interval = DEFAULT_DESIRED_MAX_CONN_INTERVAL;
    uint16 desired_slave_latency = DEFAULT_DESIRED_SLAVE_LATENCY;
    uint16 desired_conn_timeout = DEFAULT_DESIRED_CONN_TIMEOUT;

    // Set the GAP Role Parameters
    GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &initial_advertising_enable );
    GAPRole_SetParameter( GAPROLE_ADVERT_OFF_TIME, sizeof( uint16 ), &gapRole_AdvertOffTime );

    GAPRole_SetParameter( GAPROLE_SCAN_RSP_DATA,i, scanRspData_name );
    GAPRole_SetParameter( GAPROLE_ADVERT_DATA, sizeof( advertData ), advertData );

    GAPRole_SetParameter( GAPROLE_PARAM_UPDATE_ENABLE, sizeof( uint8 ), &enable_update_request );
    GAPRole_SetParameter( GAPROLE_MIN_CONN_INTERVAL, sizeof( uint16 ), &desired_min_interval );
    GAPRole_SetParameter( GAPROLE_MAX_CONN_INTERVAL, sizeof( uint16 ), &desired_max_interval );
    GAPRole_SetParameter( GAPROLE_SLAVE_LATENCY, sizeof( uint16 ), &desired_slave_latency );
    GAPRole_SetParameter( GAPROLE_TIMEOUT_MULTIPLIER, sizeof( uint16 ), &desired_conn_timeout );
  }

  // Set the GAP Characteristics
  GGS_SetParameter( GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName );

  // Set advertising interval
  {
    uint16 advInt = DEFAULT_ADVERTISING_INTERVAL;

    GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MIN, advInt );
    GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MAX, advInt );
    GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MIN, advInt );
    GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MAX, advInt );
  }

  // Setup the GAP Bond Manager
  {
    uint32 passkey = 0; // passkey "000000"
    uint8 pairMode = GAPBOND_PAIRING_MODE_NO_PAIRING;//GAPBOND_PAIRING_MODE_WAIT_FOR_REQ;
    uint8 mitm = TRUE;
    uint8 ioCap = GAPBOND_IO_CAP_DISPLAY_ONLY;
    uint8 bonding = TRUE;
    GAPBondMgr_SetParameter( GAPBOND_DEFAULT_PASSCODE, sizeof ( uint32 ), &passkey );
    GAPBondMgr_SetParameter( GAPBOND_PAIRING_MODE, sizeof ( uint8 ), &pairMode );
    GAPBondMgr_SetParameter( GAPBOND_MITM_PROTECTION, sizeof ( uint8 ), &mitm );
    GAPBondMgr_SetParameter( GAPBOND_IO_CAPABILITIES, sizeof ( uint8 ), &ioCap );
    GAPBondMgr_SetParameter( GAPBOND_BONDING_ENABLED, sizeof ( uint8 ), &bonding );
  }

  // Initialize GATT attributes
  GGS_AddService( GATT_ALL_SERVICES );            // GAP
  GATTServApp_AddService( GATT_ALL_SERVICES );    // GATT attributes
  DevInfo_AddService();                           // Device Information Service
  SimpleProfile_AddService( GATT_ALL_SERVICES );  // Simple GATT Profile
#if defined FEATURE_OAD
  VOID OADTarget_AddService();                    // OAD Profile
#endif

  // Setup the SimpleProfile Characteristic Values
  {
    uint8 charValue1 = 1;
    uint8 charValue2 = 2;
    uint8 charValue3 = 3;
    uint8 charValue4 = 4;
    uint8 charValue5[SIMPLEPROFILE_CHAR5_LEN] = { 1, 2, 3, 4, 5 };
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR1, sizeof ( uint8 ), &charValue1 );
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR2, sizeof ( uint8 ), &charValue2 );
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR3, sizeof ( uint8 ), &charValue3 );
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR4, sizeof ( uint8 ), &charValue4 );
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR5, SIMPLEPROFILE_CHAR5_LEN, charValue5 );
  }


#if defined( CC2540_MINIDK )

  SK_AddService( GATT_ALL_SERVICES ); // Simple Keys Profile

  // Register for all key events - This app will handle all key events
  RegisterForKeys( simpleBLEPeripheral_TaskID );

  // makes sure LEDs are off
  HalLedSet( (HAL_LED_1 | HAL_LED_2), HAL_LED_MODE_OFF );

  // For keyfob board set GPIO pins into a power-optimized state
  // Note that there is still some leakage current from the buzzer,
  // accelerometer, LEDs, and buttons on the PCB.

  P0SEL = 0; // Configure Port 0 as GPIO
  P1SEL = 0; // Configure Port 1 as GPIO
  P2SEL = 0; // Configure Port 2 as GPIO

  P0DIR = 0xFC; // Port 0 pins P0.0 and P0.1 as input (buttons),
                // all others (P0.2-P0.7) as output
  P1DIR = 0xFF; // All port 1 pins (P1.0-P1.7) as output
  P2DIR = 0x1F; // All port 1 pins (P2.0-P2.4) as output

  P0 = 0x03; // All pins on port 0 to low except for P0.0 and P0.1 (buttons)
  P1 = 0;   // All pins on port 1 to low
  P2 = 0;   // All pins on port 2 to low

#endif // #if defined( CC2540_MINIDK )

#if (defined HAL_LCD) && (HAL_LCD == TRUE)

#if defined FEATURE_OAD
  #if defined (HAL_IMAGE_A)
    HalLcdWriteStringValue( "BLE Peri-A", OAD_VER_NUM( _imgHdr.ver ), 16, HAL_LCD_LINE_1 );
  #else
    HalLcdWriteStringValue( "BLE Peri-B", OAD_VER_NUM( _imgHdr.ver ), 16, HAL_LCD_LINE_1 );
  #endif // HAL_IMAGE_A
#else
//  HalLcdWriteString( "BLE Peripheral", HAL_LCD_LINE_1 );
 // HalLcdWriteString( scanRspData+2, HAL_LCD_LINE_1 );

#endif // FEATURE_OAD

#endif // (defined HAL_LCD) && (HAL_LCD == TRUE)

  // Register callback with SimpleGATTprofile
  VOID SimpleProfile_RegisterAppCBs( &simpleBLEPeripheral_SimpleProfileCBs );

  // Enable clock divide on halt
  // This reduces active current while radio is active and CC254x MCU
  // is halted
  HCI_EXT_ClkDivOnHaltCmd( HCI_EXT_ENABLE_CLK_DIVIDE_ON_HALT );

#if defined ( DC_DC_P0_7 )

  // Enable stack to toggle bypass control on TPS62730 (DC/DC converter)
  HCI_EXT_MapPmIoPortCmd( HCI_EXT_PM_IO_PORT_P0, HCI_EXT_PM_IO_PORT_PIN7 );

#endif // defined ( DC_DC_P0_7 )

  // Setup a delayed profile startup
  osal_set_event( simpleBLEPeripheral_TaskID, SBP_START_DEVICE_EVT );

}
#define SBP_DHT11_READ_HUMITURE_EVT_PERIOD      1000 
#define SBP_SEND_TO_APP_EVT_PERIOD      100 
  uint8 sbDHT11_data[4] = {0};     
bStatus_t  Send_status7=0; 
uint8 tmp_da[24];
uint8   SendNoti_TO_APP(uint8 *pBuffer,uint16 length)
{
	 
	 
	attHandleValueNoti_t pReport;
        if(Connect_flag!=2)
	   return 1;
 	pReport.handle=0x2E;  	 
	pReport.len = length;
 	memcpy(pReport.value, tmp_da, length);
	Send_status7=GATT_Notification( 0, &pReport, FALSE );;

	return Send_status7;
}


/*********************************************************************
 * @fn      SimpleBLEPeripheral_ProcessEvent
 *
 * @brief   Simple BLE Peripheral Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  events not processed
 */
uint16 SimpleBLEPeripheral_ProcessEvent( uint8 task_id, uint16 events )
{

  VOID task_id; // OSAL required parameter that isn't used in this function
  
   //周期采集DHT11温湿度事件  
  if ( events & SBP_DHT11_READ_HUMITURE_EVT )  
  {  
    //关总中断，不关会出现0的数据  
    // EA = 0;             
    //采集DHT11温湿度  
     if(car.left==0) 
     {
       DHT11_Read_Humiture(sbDHT11_data);   
     //  EA = 1;  
      //开总中断  
    
      //启动定时器执行串口打印DHT11温湿度事件  
      //osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_DHT11_USART_EVT, SBP_DHT11_USART_EVT_PERIOD );   
    
      //启动定时器执行周期采集DHT11温湿度事件  
       if(sbDHT11_data[0]==0)
          osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_DHT11_READ_HUMITURE_EVT, SBP_DHT11_READ_HUMITURE_EVT_PERIOD );  
       else
       {
          sprintf(tmp_da,"DHT:%d.%d;%d.%d\0",sbDHT11_data[0],sbDHT11_data[1],sbDHT11_data[2],sbDHT11_data[3]);
          Send_status7=SendNoti_TO_APP(sbDHT11_data,strlen(tmp_da));  
     
       }
     }
      //else
      //   osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_SEND_TO_APP_EVT, SBP_SEND_TO_APP_EVT_PERIOD ); 
     
    return (events ^ SBP_DHT11_READ_HUMITURE_EVT);      
  }   
  if ( events & SBP_SEND_TO_APP_EVT )  
  {     
     Send_status7=SendNoti_TO_APP(sbDHT11_data,4);       
     return (events ^ SBP_SEND_TO_APP_EVT);      
  }  
  
  if ( events & SYS_EVENT_MSG )
  {
    uint8 *pMsg;

    if ( (pMsg = osal_msg_receive( simpleBLEPeripheral_TaskID )) != NULL )
    {
	simpleBLEPeripheral_ProcessOSALMsg( (osal_event_hdr_t *)pMsg );


 #if defined( CC2540_MINIDK )
      // Release the OSAL message
      VOID osal_msg_deallocate( pMsg );
 #endif
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

  if ( events & SBP_START_DEVICE_EVT )
  {

    // Start the Device
    VOID GAPRole_StartDevice( &simpleBLEPeripheral_PeripheralCBs );

    // Start Bond Manager
    VOID GAPBondMgr_Register( &simpleBLEPeripheral_BondMgrCBs );

    // Set timer for first periodic event
    osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_PERIODIC_EVT, SBP_PERIODIC_EVT_PERIOD );
//启动定时器执行周期采集DHT11温湿度事件  
   // osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_DHT11_READ_HUMITURE_EVT, SBP_DHT11_READ_HUMITURE_EVT_PERIOD );
    
    
    return ( events ^ SBP_START_DEVICE_EVT );
  }

  if ( events & SBP_PERIODIC_EVT )
  {
    // Restart timer
    if ( SBP_PERIODIC_EVT_PERIOD )
    {
      osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_PERIODIC_EVT, SBP_PERIODIC_EVT_PERIOD );
    }

    // Perform periodic application task
    performPeriodicTask();

    return (events ^ SBP_PERIODIC_EVT);
  }

#if defined ( PLUS_BROADCASTER )
   
  if ( events & SBP_ADV_IN_CONNECTION_EVT )
  {
    uint8 turnOnAdv = TRUE;
    // Turn on advertising while in a connection
    GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &turnOnAdv );

    return (events ^ SBP_ADV_IN_CONNECTION_EVT);
  }
#endif // PLUS_BROADCASTER

  // Discard unknown events
  return 0;
}



/*********************************************************************

 */
uint16 SimpleBLEPeripheral_ProcessEvent_HCI( uint8 task_id, uint16 events )
{

  VOID task_id; // OSAL required parameter that isn't used in this function

   

  // Discard unknown events
  return 0;
}


/*********************************************************************
 * @fn      simpleBLEPeripheral_ProcessOSALMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void simpleBLEPeripheral_ProcessOSALMsg( osal_event_hdr_t *pMsg )
{

  switch ( pMsg->event )
  {
  #if defined( CC2540_MINIDK )
    case KEY_CHANGE:
      simpleBLEPeripheral_HandleKeys( ((keyChange_t *)pMsg)->state, ((keyChange_t *)pMsg)->keys );
      break;
  #endif // #if defined( CC2540_MINIDK )

  default:
    // do nothing
    break;
  }
}

#if defined( CC2540_MINIDK )

/*********************************************************************
 * @fn      simpleBLEPeripheral_HandleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   shift - true if in shift/alt.
 * @param   keys - bit field for key events. Valid entries:
 *                 HAL_KEY_SW_2
 *                 HAL_KEY_SW_1
 *
 * @return  none
 */
static void simpleBLEPeripheral_HandleKeys( uint8 shift, uint8 keys )
{
  uint8 SK_Keys = 0;

  VOID shift;  // Intentionally unreferenced parameter

  if ( keys & HAL_KEY_SW_1 )
  {
    SK_Keys |= SK_KEY_LEFT;
  }

  if ( keys & HAL_KEY_SW_2 )
  {

    SK_Keys |= SK_KEY_RIGHT;

    // if device is not in a connection, pressing the right key should toggle
    // advertising on and off
    if( gapProfileState != GAPROLE_CONNECTED )
    {
      uint8 current_adv_enabled_status;
      uint8 new_adv_enabled_status;

      //Find the current GAP advertisement status
      GAPRole_GetParameter( GAPROLE_ADVERT_ENABLED, &current_adv_enabled_status );

      if( current_adv_enabled_status == FALSE )
      {
        new_adv_enabled_status = TRUE;
      }
      else
      {
        new_adv_enabled_status = FALSE;
      }

      //change the GAP advertisement status to opposite of current status
      GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &new_adv_enabled_status );
    }

  }

  // Set the value of the keys state to the Simple Keys Profile;
  // This will send out a notification of the keys state if enabled
  SK_SetParameter( SK_KEY_ATTR, sizeof ( uint8 ), &SK_Keys );
}
#endif // #if defined( CC2540_MINIDK )


static void Callback_RssiCB(   int8 rssi )
{
	rssi_ack=rssi;
}



/*********************************************************************
 * @fn      peripheralStateNotificationCB
 *
 * @brief   Notification from the profile of a state change.
 *
 * @param   newState - new state
 *
 * @return  none
 */
static void peripheralStateNotificationCB( gaprole_States_t newState )
{
  switch ( newState )
  {
    case GAPROLE_STARTED:
      {
        uint8 ownAddress[B_ADDR_LEN];
        uint8 systemId[DEVINFO_SYSTEM_ID_LEN];

        GAPRole_GetParameter(GAPROLE_BD_ADDR, ownAddress);

        // use 6 bytes of device address for 8 bytes of system ID value
        systemId[0] = ownAddress[0];
        systemId[1] = ownAddress[1];
        systemId[2] = ownAddress[2];

        // set middle bytes to zero
        systemId[4] = 0x00;
        systemId[3] = 0x00;

        // shift three bytes up
        systemId[7] = ownAddress[5];
        systemId[6] = ownAddress[4];
        systemId[5] = ownAddress[3];

        DevInfo_SetParameter(DEVINFO_SYSTEM_ID, DEVINFO_SYSTEM_ID_LEN, systemId);

        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
          // Display device address
          HalLcdWriteString( bdAddr2Str( ownAddress ),  HAL_LCD_LINE_2 );
          HalLcdWriteString( "Initialized",  HAL_LCD_LINE_3 );
        #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
      }
      break;

    case GAPROLE_ADVERTISING:
      {
       // #if (defined HAL_LCD) && (HAL_LCD == TRUE)
        //  HalLcdWriteString( "Advertising",  HAL_LCD_LINE_3 );
	  // sbpSerialAppWrite("Advertising", sizeof("Advertising"));
	// sbpSerialAppWrite("success", sizeof("success"));
	    Connect_flag=1;
	//sbpSerialAppCallback(0, 1);
		
      //  #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
      }
      break;

    case GAPROLE_CONNECTED:
      {
       // #if (defined HAL_LCD) && (HAL_LCD == TRUE)
        //  HalLcdWriteString( "Connected",  HAL_LCD_LINE_3 );
	  // sbpSerialAppWrite("Connected", sizeof("Connected"));
	  // sbpSerialAppCallback(0, 1);
	    Connect_flag=2;
      //  #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
      }
      break;

    case GAPROLE_WAITING:
      {
        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
          HalLcdWriteString( "Disconnected",  HAL_LCD_LINE_3 );
        #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
		  Connect_flag=1;
      }
      break;

    case GAPROLE_WAITING_AFTER_TIMEOUT:
      {
        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
          HalLcdWriteString( "Timed Out",  HAL_LCD_LINE_3 );
        #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
      }
      break;

    case GAPROLE_ERROR:
      {
        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
     //     HalLcdWriteString( "Error",  HAL_LCD_LINE_3 );
        #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
      }
      break;

    default:
      {
        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
      //    HalLcdWriteString( "",  HAL_LCD_LINE_3 );
        #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
      }
      break;

  }

  gapProfileState = newState;

#if !defined( CC2540_MINIDK )
  VOID gapProfileState;     // added to prevent compiler warning with
                            // "CC2540 Slave" configurations
#endif


}

/*********************************************************************
 * @fn      performPeriodicTask
 *
 * @brief   Perform a periodic application task. This function gets
 *          called every five seconds as a result of the SBP_PERIODIC_EVT
 *          OSAL event. In this example, the value of the third
 *          characteristic in the SimpleGATTProfile service is retrieved
 *          from the profile, and then copied into the value of the
 *          the fourth characteristic.
 *
 * @param   none
 *
 * @return  none
 */
static void performPeriodicTask( void )
{

  
  uint8 valueToCopy;
  uint8 stat;

  // Call to retrieve the value of the third characteristic in the profile
  stat = SimpleProfile_GetParameter( SIMPLEPROFILE_CHAR3, &valueToCopy);

  if( stat == SUCCESS )
  {
    /*
     * Call to set that value of the fourth characteristic in the profile. Note
     * that if notifications of the fourth characteristic have been enabled by
     * a GATT client device, then a notification will be sent every time this
     * function is called.
     */
   // SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR4, sizeof(uint8), &valueToCopy);
  }
}

/*********************************************************************
 * @fn      simpleProfileChangeCB
 *
 * @brief   Callback from SimpleBLEProfile indicating a value change
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  none
 */

static void simpleProfileChangeCB( uint8 paramID,uint8 length )
{
  uint8 newValue,i;
 
  //sbpSerialAppWrite ("ChangeCB", sizeof("ChangeCB"));
  switch( paramID )
  {
    case SIMPLEPROFILE_CHAR1:
      //SimpleProfile_GetParameter( SIMPLEPROFILE_CHAR1, &newValue );
      SimpleProfile_GetParameter( SIMPLEPROFILE_CHAR1, newValueBuf );   		
    //  #if (defined HAL_LCD) && (HAL_LCD == TRUE)
    //   HalLcdWriteStringValue(newValueBuf, (uint16)(newValue), 10,  HAL_LCD_LINE_3 );
     //   HalLcdWriteString((char*)newValueBuf, HAL_LCD_LINE_4 );
  //   	len=newValueBuf[SIMPLEPROFILE_CHAR1_LEN-3];
	//sbpSerialAppWrite (newValueBuf, len);
	 for(i=0;i<length;i++)
		receing_data(newValueBuf[i]);
	//cmd_pase(newValueBuf,length);
	if(length>0&&(newValueBuf[0]=='G'||newValueBuf[0]=='g'))
          osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_DHT11_READ_HUMITURE_EVT, 1000 );
        // else
        {
           
        }
	//sbpSerialAppWrite (newValueBuf, length);
   //   #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)

      break;

    case SIMPLEPROFILE_CHAR3:
      SimpleProfile_GetParameter( SIMPLEPROFILE_CHAR3, &newValue );

      #if (defined HAL_LCD) && (HAL_LCD == TRUE)
        HalLcdWriteStringValue( "Char 3:", (uint16)(newValue), 10,  HAL_LCD_LINE_3 );
      #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)

      break;

    default:
      // should not reach here!
      break;
  }
}

#if (defined HAL_LCD) && (HAL_LCD == TRUE)
/*********************************************************************
 * @fn      bdAddr2Str
 *
 * @brief   Convert Bluetooth address to string. Only needed when
 *          LCD display is used.
 *
 * @return  none
 */
char *bdAddr2Str( uint8 *pAddr )
{
  uint8       i;
  char        hex[] = "0123456789ABCDEF";
  static char str[B_ADDR_STR_LEN];
  char        *pStr = str;

  *pStr++ = '0';
  *pStr++ = 'x';

  // Start from end of addr
  pAddr += B_ADDR_LEN;

  for ( i = B_ADDR_LEN; i > 0; i-- )
  {
    *pStr++ = hex[*--pAddr >> 4];
    *pStr++ = hex[*pAddr & 0x0F];
  }

  *pStr = 0;

  return str;
}
#endif // (defined HAL_LCD) && (HAL_LCD == TRUE)

/*********************************************************************
*********************************************************************/
