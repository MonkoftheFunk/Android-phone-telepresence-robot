    //******************************************************************************          
    //name:         DHT11.h        
    //introduce:    ��ȡDHT11��ʪ��      
    //author:       ����Ĵ����        
    //changetime:   2016.04.11      
    //email:        897503845@qq.com      
    //******************************************************************************  
    #ifndef __DHT11_H__  
    #define __DHT11_H__  
      
    //define  
    typedef enum{  
      DHT11_TRUE,  
      DHT11_FALSE  
    }boolean;  
      
    //function  
      void DHT11_Read_Humiture(unsigned char *pDHT11_data);                //��ʪ��������  
      
    #endif  