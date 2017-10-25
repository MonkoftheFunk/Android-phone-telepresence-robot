#include "bcomdef.h"
#include "OSAL.h"
#include "OSAL_PwrMgr.h"

#include "OnBoard.h"
#include "hal_adc.h"
#include "hal_led.h"
#include "hal_key.h"
#include "hal_lcd.h"

#include "hal_uart.h"


#include "gatt.h"
#include "gap.h"

#include "gapgattserver.h"
#include "gattservapp.h"
#include "simpleGATTprofile.h"


#include "SerialApp.h"
#if defined ( PLUS_BROADCASTER )
  #include "peripheralBroadcaster.h"
#else
  #include "peripheral.h"
#endif

   uint8  pktBuffer[20];
   uint8  name_buff[20]; // for advertise name set

#define  My_Uart_ISR_client   0

uint16  Uart_RX_Bytes_client=0;
bStatus_t  Send_status=0;
uint32 Connect_flag=0;/*0:setting;1:disconnected; 2:connected*/
extern int8 rssi_ack;
extern   uint8 simpleBLEPeripheral_TaskID;   // Task ID for internal task/event processing


#if My_Uart_ISR_client==1

void UART_Init()
{
	/*1、配置U0CSR寄存器*/
	U0CSR |= BV(7); //UART mode
	U0CSR |= BV(6); //Receiver enabled

	/*2、配置U0UCR寄存器*/
	U0UCR &= ~(BV(6)); //disabled Flow control
	U0UCR &= ~(BV(4)); //8-bit
	U0UCR &= ~(BV(3)); //parity disabled
	U0UCR &= ~(BV(2)); // 1 stop bit
	 U0UCR |= BV(1);    //high stop bit
	U0UCR &= ~(BV(0)); //low start bit
	/*3、设置波特率*/
	//115200
	/* U0BAUD = 0xD8;

	 U0GCR |= 0x0B;
	 */
	U0BAUD = 59; // BAUD_M = 59 and BAUD_E = 8 => Baud rate of 9600
	U0GCR &= ~0x1F; // Clear BAUD_E bits
	U0GCR |= 0x08; // BAUD_E = 8
	
	/*4、FLUSH*/
	U0UCR |= BV(7);
}

void UART_GPIO_Init()
{
	PERCFG &= ~(BV(0)); //选择P0端口uart
	P0SEL |=BV(5)|BV(4)|BV(3)|BV(2);
	P2DIR &=~(BV(7)|BV(6));
}
void UART_EnableRxInt()
{
	EA = 1;
	URX0IE = 1;
}
/*想UART0 发送数据*/
void UART_SendString(uint8 *data,int len)
{
	int i;
	for(i=0;i<len;i++)
	{
		U0DBUF = *data++;
		while(UTX0IF == 0);
		UTX0IF = 0;
	}
}
//uint8 temp;
#pragma vector = URX0_VECTOR 
__interrupt void UART0_ISR(void) 
{ 

	 
	//while ((U0CSR & 0x04) == 0);
	Uart_RX_Buffer_client[Uart_RX_Bytes_client]=U0DBUF; 
	Uart_RX_Bytes_client++;
	if(Uart_RX_Bytes_client>=SBP_UART_RX_BUF_SIZE)
		Uart_RX_Bytes_client=0;
	Uart_RX_Buffer_client[Uart_RX_Bytes_client]=0; 
	URX0IF = 0;    //清中断标志 
	 

} 

#endif



/*该函数将会在任务函数的初始化函数中调用*/
void SerialApp_Init( uint8 taskID )
{

#if My_Uart_ISR_client==1

	UART_GPIO_Init();
	UART_Init();
	UART_EnableRxInt();
#else
	 
	serialAppInitTransport();
#endif

	memset(name_buff,0,sizeof(name_buff));
	Connect_flag=0;
  //调用uart初始化代码
  //记录任务函数的taskID，备用
 

  
}

/*uart初始化代码，配置串口的波特率、流控制等*/
void serialAppInitTransport( )
{
  halUARTCfg_t uartConfig;

  // configure UART
  uartConfig.configured           = TRUE;
  uartConfig.baudRate             = HAL_UART_BR_57600;//波特率
  uartConfig.flowControl          = SBP_UART_FC;//流控制
  uartConfig.flowControlThreshold = SBP_UART_FC_THRESHOLD;//流控制阈值，当开启flowControl时，该设置有效
  uartConfig.rx.maxBufSize        =   SBP_UART_RX_BUF_SIZE;//uart接收缓冲区大小
  uartConfig.tx.maxBufSize        =  SBP_UART_TX_BUF_SIZE;//uart发送缓冲区大小
  uartConfig.idleTimeout          = SBP_UART_IDLE_TIMEOUT;
  uartConfig.intEnable            = SBP_UART_INT_ENABLE;//是否开启中断
  uartConfig.callBackFunc         =sbpSerialAppCallback;//uart接收回调函数，在该函数中读取可用uart数据

  // start UART
  // Note: Assumes no issue opening UART port.
  (void)HalUARTOpen( SBP_UART_PORT, &uartConfig );

  return;
}

#if 0

#else


void Set_ADname(uint8 data_t)
{
	static int j=0;
	if(j>=255)
        {
           	
		return;
        }
	if((data_t=='A')&&(j==0))
		j=1;
	else if((data_t=='D')&&(j==1))
		j=2;
	else if((data_t=='n')&&(j==2))
		j=3;
	else if((data_t=='a')&&(j==3))
		j=4;
	else if((data_t=='m')&&(j==4))
		j=5;
	else if((data_t=='e')&&(j==5))
		j=6;
	else if((data_t==':')&&(j==6))
		j=7;
	else if((j>=7)&&(j<30))
	{
		j++;
		if((j-8)>=20)
			return;
		name_buff[j-8]=data_t;
		if(data_t=='\n')
			j=255;
		
	}
	else
		j=0;
}

void delay_tm(unsigned int num)
{
	unsigned int i;
	for(i=0;i<512*num;i++);

}



/*uart接收回调函数*/
void sbpSerialAppCallback(uint8 port, uint8 event)
{
	// unused input parameter; PC-Lint error 715.
	int numBytes;
 	static uint16 i=0;
	// (void)event;
	//  HalLcdWriteString("Data form my UART:", HAL_LCD_LINE_4 );
	//返回可读的字节

	if(Connect_flag==0)
	{
		if ( (numBytes = Hal_UART_RxBufLen(port)) > 0 )
		{
			delay_tm(1);
			if(numBytes>20)
				numBytes=20;
			(void)HalUARTRead (port, pktBuffer, numBytes);
			
			for(i=0;i<numBytes;i++)
			{
			 	Set_ADname(pktBuffer[i]);
			 }
			 
		}

		 
	}
	////////////
	
	else// if(Connect_flag==2)	
	{
		if((numBytes = Hal_UART_RxBufLen(port)) > 0)
		{
                      //  delay_tm(1);
			Uart_RX_Bytes_client++;
			if((numBytes>=20))
			{
		 		
				(void)HalUARTRead (port, pktBuffer, 20);
				 Send_status=sbpSerialAppSendNoti(pktBuffer,20);
	                 //  HalLcdWriteStringValue(pktBuffer,12, 10,  HAL_LCD_LINE_4 );
				//  SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR1, 1, pktBuffer );
				Uart_RX_Bytes_client=0;
			}
		}
	}
}
#endif

void sbpSerialAppCallback_peripheral_last( )
{
  uint8  pktBuffer[20];
  // unused input parameter; PC-Lint error 715.
  uint16 numBytes;

  //返回可读的字节
  if ( (numBytes = Hal_UART_RxBufLen(SBP_UART_PORT)) > 0 )
  {
  	//读取全部有效的数据，这里可以一个一个读取，以解析特定的命令
	if(numBytes>20)
		numBytes=20;
	 
	{	
		 
		(void)HalUARTRead (SBP_UART_PORT, pktBuffer, numBytes);
		sbpSerialAppSendNoti(pktBuffer,numBytes);
	}
 	
  }
  
}


void sbpSerialAppWrite(uint8 *pBuffer, uint16 length)
{
#if My_Uart_ISR_client==1

	UART_SendString(pBuffer, length);
 #else
	 HalUARTWrite (SBP_UART_PORT, pBuffer, length);
 #endif

}
/*
uint8 sbpGattWriteString(uint8 *pBuffer, uint16 length)
{
	uint8 status;
	uint8 len;
	gattPrepareWriteReq_t req;

	//18 bytes a packes,last byte is lenght;
	
	len = SIMPLEPROFILE_CHAR1_LEN-2;
	
	req.handle = simpleBLECharHdl;
	req.offset = 0;
	req.len = len;
	//req.pValue = "0123456";
	//if(i==0)
		req.pValue =  osal_msg_allocate(len);
	//req.pValue=pBuffer;
	
	osal_memcpy(req.pValue,pBuffer,SIMPLEPROFILE_CHAR1_LEN-3);
	req.pValue[SIMPLEPROFILE_CHAR1_LEN-3]=length;
	 
	status = GATT_WriteLongCharValue( GAP_CONNHANDLE_INIT, &req, simpleBLEPeripheral_TaskID );
	osal_msg_deallocate(req.pValue);
	 
        return status;
}
*/


uint8  sbpSerialAppSendNoti(uint8 *pBuffer,uint16 length)
{
	static int i=0;
	//sbpGattWriteString(pBuffer,length);
	//return 0;
	uint8  pBuf[10];
	attHandleValueNoti_t pReport;
	
  	if(Connect_flag!=2)
		return 0;
	pReport.len = length;
	osal_memcpy(pReport.value, pBuffer, length);
	Send_status=GATT_Notification( 0, &pReport, FALSE );

	pBuf[0]='0'+Send_status;
	//GAPRole_GetParameter(GAPROLE_RSSI_READ_RATE, pBuf+1);
      //  if(pBuf[1]||pBuf[2])
	pBuf[1]=rssi_ack;
	pBuf[2]=0;
	sbpSerialAppWrite(pBuf,2);

	return Send_status;
}


