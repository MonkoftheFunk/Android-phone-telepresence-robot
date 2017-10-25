    //******************************************************************************          
    //name:         DHT11.c        
    //introduce:    ��ȡDHT11��ʪ��      
    //author:       ����Ĵ����        
    //changetime:   2016.04.11      
    //email:        897503845@qq.com      
    //******************************************************************************  
    //map  
    #include "DHT11.h"  
    //#include <ioCC2540.h>  
    #include "string.h"  
      #include "OnBoard.h"
      
    //define  
    #define DATA_PIN                P1_0                                            //�����ΪP0.0  
    #define SET_PIN_IN           P1DIR &= ~0x01;        // P0DIR &= ~0x04;         asm("NOP"); asm("NOP")  //����IOΪ����  
    #define SET_PIN_OUT          P1DIR |= (1 << 0);     // P0DIR |= (1 << 3);      asm("NOP"); asm("NOP")  //����IOΪ���   
    #define SET_DATA_PIN            DATA_PIN = 1;           asm("NOP"); asm("NOP")  //����P0.0   
    #define CLR_DATA_PIN            DATA_PIN = 0;           asm("NOP"); asm("NOP")  //����P0.0   
      
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
    //introduce:        us��ʱ����              
    //parameter:        nUs��us��           
    //return:           none              
    //author:           ����Ĵ����         
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
    //introduce:        DHT11�Ķ������ֽں���              
    //parameter:        none          
    //return:           none              
    //author:           ����Ĵ����         
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
    //introduce:        DHT11�Ķ���ʪ�Ⱥ���              
    //parameter:        none          
    //return:           TRUE:��ȡ��ȷ��FALSE:��ȡ����              
    //author:           ����Ĵ����         
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
        //������ʼ�ź�   
        SET_PIN_OUT;                                //IO������Ϊ���  
          
        SET_DATA_PIN;                               //����ߵ�ƽ  
           
     //  Delay_Us(100000);  
        CLR_DATA_PIN;                               //����͵�ƽ  
            
        Delay_Us(4000);                            //��ʱ����18MS  
               
        SET_DATA_PIN;                               //����ߵ�ƽ  
      
        Delay_Us(4);  
           
        //�л���DHT11�˷�����  
        SET_PIN_IN;                                 //IO������Ϊ����  
         
        Delay_Us(4);    
             
       if(DATA_PIN == 0) 
        {  
            //DHT11��Ӧ                         
           nFLAG = 2;   
           //EA = 0;
            while((DATA_PIN == 0) && (nFLAG++));  
              
            nFLAG = 2;  
              
            while((DATA_PIN == 1) && (nFLAG++));   
              
              
            //DHT11��ֵ  
            nHumidity_integer = DHT11_Read_Char();          //ʪ�ȸ�λ������  
              
            nHumidity_decimal = DHT11_Read_Char();          //ʪ�ȵ�λ��С��  
              
            nTemperature_integer = DHT11_Read_Char();       //�¶ȸ�λ������  
              
            nTemperature_decimal = DHT11_Read_Char();       //�¶ȵ�λ��С��   
              
            nChecksum = DHT11_Read_Char();                  //У���   
      
             EA = 1; 
            //����  
            SET_DATA_PIN;  
              
                  
            //����У���  
            nChecksum_count = nHumidity_integer + nHumidity_decimal + nTemperature_integer + nTemperature_decimal;  
              
            //��֤������ȷ��  
            if(nChecksum == nChecksum_count)  
            {            
                //��ȷ���򽫶�����ֵ���浽ȫ�ֱ���  
                pDHT11_data[0] = nHumidity_integer;  
                  
                pDHT11_data[1] = nHumidity_decimal;  
                  
                pDHT11_data[2] = nTemperature_integer;  
                  
                pDHT11_data[3] = nTemperature_decimal;                
            }  
            else  
            {            
                 //������ֵȫΪ0  
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
            //������ֵȫΪ0  
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