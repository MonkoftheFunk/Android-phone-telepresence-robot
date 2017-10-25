    //******************************************************************************          
    //name:         DHT11.c        
    //introduce:    读取DHT11温湿度      
    //author:       甜甜的大香瓜        
    //changetime:   2016.04.11      
    //email:        897503845@qq.com      
    //******************************************************************************  
    //map  
    #include "DHT11.h"  
    //#include <ioCC2540.h>  
    #include "string.h"  
      #include "OnBoard.h"
      
    //define  
    #define DATA_PIN                P1_0                                            //输入口为P0.0  
    #define SET_PIN_IN           P1DIR &= ~0x01;        // P0DIR &= ~0x04;         asm("NOP"); asm("NOP")  //配置IO为输入  
    #define SET_PIN_OUT          P1DIR |= (1 << 0);     // P0DIR |= (1 << 3);      asm("NOP"); asm("NOP")  //配置IO为输出   
    #define SET_DATA_PIN            DATA_PIN = 1;           asm("NOP"); asm("NOP")  //拉高P0.0   
    #define CLR_DATA_PIN            DATA_PIN = 0;           asm("NOP"); asm("NOP")  //拉低P0.0   
      
    typedef signed   char   int8;       
    typedef unsigned char   uint8;      
        
    typedef signed   short  int16;       
    typedef unsigned short  uint16;     
        
    typedef signed   long   int32;     
    typedef unsigned long   uint32;  
      
      
    //function  
    static void Delay_Us(uint32 nUs);  
    static uint8 DHT11_Read_Char(void);  
    //******************************************************************************                
    //name:             Delay_Us                
    //introduce:        us延时函数              
    //parameter:        nUs：us数           
    //return:           none              
    //author:           甜甜的大香瓜         
    //email:            897503845@qq.com       
    //changetime:       2016.04.11             
    //******************************************************************************   
    static void Delay_Us(uint32 nUs)   
    {  
      while (nUs--)  
      {  
        asm("NOP");/*asm("NOP");asm("NOP");asm("NOP");  
        asm("NOP");asm("NOP");asm("NOP");asm("NOP");  
        asm("NOP");asm("NOP");asm("NOP");asm("NOP");  
        asm("NOP");asm("NOP");asm("NOP");asm("NOP");  
        asm("NOP");asm("NOP");asm("NOP");asm("NOP");  
        asm("NOP");asm("NOP");asm("NOP");asm("NOP");  
        asm("NOP");asm("NOP");asm("NOP");asm("NOP");  
        asm("NOP");asm("NOP");asm("NOP");asm("NOP");  */
      }  
    }  
      
    //******************************************************************************                
    //name:             DHT11_Read_Char                
    //introduce:        DHT11的读单个字节函数              
    //parameter:        none          
    //return:           none              
    //author:           甜甜的大香瓜         
    //email:            897503845@qq.com       
    //changetime:       2016.04.11             
    //******************************************************************************   
    static uint8 DHT11_Read_Char(void)   
    {       
        uint8 i;  
        uint8 nDht11_bit;  
        uint8 nDht11_char;      
        uint8 nFLAG;  
          
        for(i=0; i < 8; i++){  
            
            nFLAG = 2;   
            while((DATA_PIN == 0) && (nFLAG++));         
              
            Delay_Us(4);            
              
            if(DATA_PIN == 1){  
              nDht11_bit = 1;  
            }  
            else{  
              nDht11_bit = 0;          
            }  
              
            nFLAG = 2;  
            while((DATA_PIN == 1) && (nFLAG++));      
            if(nFLAG == 1){  
              break;  
            }  
              
            nDht11_char <<= 1;  
            nDht11_char |= nDht11_bit;   
        }   
      
        return nDht11_char;  
    }  
    
    //******************************************************************************                
    //name:             DHT11_Read_Humiture                
    //introduce:        DHT11的读温湿度函数              
    //parameter:        none          
    //return:           TRUE:读取正确，FALSE:读取错误              
    //author:           甜甜的大香瓜         
    //email:            897503845@qq.com       
    //changetime:       2016.04.11             
    //******************************************************************************   
     void DHT11_Read_Humiture(uint8 *pDHT11_data)     
    {  
        uint8 nHumidity_integer;  
        uint8 nHumidity_decimal;  
        uint8 nTemperature_integer;  
        uint8 nTemperature_decimal;   
        uint8 nChecksum;    
        uint8 nChecksum_count;  
        uint8 nFLAG;  
      EA = 0;
        //主机开始信号   
        SET_PIN_OUT;                                //IO口配置为输出  
          
        SET_DATA_PIN;                               //输出高电平  
           
     //  Delay_Us(100000);  
        CLR_DATA_PIN;                               //输出低电平  
            
        Delay_Us(4000);                            //延时至少18MS  
               
        SET_DATA_PIN;                               //输出高电平  
      
        Delay_Us(4);  
           
        //切换到DHT11端发数据  
        SET_PIN_IN;                                 //IO口配置为输入  
         
        Delay_Us(4);    
             
       if(DATA_PIN == 0) 
        {  
            //DHT11响应                         
           nFLAG = 2;   
           //EA = 0;
            while((DATA_PIN == 0) && (nFLAG++));  
              
            nFLAG = 2;  
              
            while((DATA_PIN == 1) && (nFLAG++));   
              
              
            //DHT11读值  
            nHumidity_integer = DHT11_Read_Char();          //湿度高位，整数  
              
            nHumidity_decimal = DHT11_Read_Char();          //湿度低位，小数  
              
            nTemperature_integer = DHT11_Read_Char();       //温度高位，整数  
              
            nTemperature_decimal = DHT11_Read_Char();       //温度低位，小数   
              
            nChecksum = DHT11_Read_Char();                  //校验和   
      
             EA = 1; 
            //结束  
            SET_DATA_PIN;  
              
                  
            //计算校验和  
            nChecksum_count = nHumidity_integer + nHumidity_decimal + nTemperature_integer + nTemperature_decimal;  
              
            //验证数据正确性  
            if(nChecksum == nChecksum_count)  
            {            
                //正确，则将读出的值保存到全局变量  
                pDHT11_data[0] = nHumidity_integer;  
                  
                pDHT11_data[1] = nHumidity_decimal;  
                  
                pDHT11_data[2] = nTemperature_integer;  
                  
                pDHT11_data[3] = nTemperature_decimal;                
            }  
            else  
            {            
                 //错误，则值全为0  
                pDHT11_data[0] = 0;  
                  
                pDHT11_data[1] = 0;  
                  
                pDHT11_data[2] = 0;  
                  
                pDHT11_data[3] = 0;   
                 EA = 1;       
                return  ;  
            }   
        }  
        else                          
        {                                 
            //错误，则值全为0  
            pDHT11_data[0] = 0;  
                  
            pDHT11_data[1] = 0;  
                  
            pDHT11_data[2] = 0;  
                  
            pDHT11_data[3] = 0;   
             EA = 1;       
            return  ;        
        }  
          EA = 1; 
        return  ;  
    }  