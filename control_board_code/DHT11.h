    //******************************************************************************          
    //name:         DHT11.h        
    //introduce:    读取DHT11温湿度      
    //author:       甜甜的大香瓜        
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
      void DHT11_Read_Humiture(unsigned char *pDHT11_data);                //温湿传感启动  
      
    #endif  