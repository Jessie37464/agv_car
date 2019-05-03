#include "contact.h"
#include "motor.h"


void LeftMovingSpeedW(float val)//左轮方向和速度控制函数
{     
    if(val>=0)
    {  
       HAL_GPIO_WritePin(GPIOB,left1_Pin,GPIO_PIN_RESET);
			 HAL_GPIO_WritePin(GPIOB,left2_Pin,GPIO_PIN_RESET);
        	
    }
    else if(val<0)
    {  
        HAL_GPIO_WritePin(GPIOB,left1_Pin,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB,left2_Pin,GPIO_PIN_SET);
        val=-val;

    }	
		if(val>48) val=48;
		if(val<0) val=0;
		
	
		PWM_LEFT(val);
		PWM_LEFT2(val);
		

}

void RightMovingSpeedW( float val2)//右轮方向和速度控制函数
{    
    if(val2>=0)
    {  
      
       HAL_GPIO_WritePin(GPIOB,right1_Pin,GPIO_PIN_SET);
			 HAL_GPIO_WritePin(GPIOB,right2_Pin,GPIO_PIN_SET);

    }
    else if(val2<0)
    {  
      
        HAL_GPIO_WritePin(GPIOB,right1_Pin,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB,right2_Pin,GPIO_PIN_RESET);
        val2=-val2;
		}
		
		if(val2>48) val2=48;
		if(val2<0) val2=0;
		

		PWM_RIGHT(val2);
		PWM_RIGHT2(val2);
	
}



void car_control(float rightspeed,float leftspeed)//小车速度转化和控制函数
{
		float test_right=rightspeed/1000;
    float test_left=leftspeed/1000;
		
    RightMovingSpeedW(test_right);
    LeftMovingSpeedW(test_left);
	 
	
	
}



