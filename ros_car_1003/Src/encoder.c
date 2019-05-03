 #include "stm32f1xx_hal.h"
 #include "encoder.h"
 #include "motor.h"
 
 int span;
 extern int speed_right;
 extern int speed_left;

	int output_speed (uint8_t receive_buffer[20],uint8_t receive_length)
	{ 
		int speed;
		
		if(receive_length>8&&receive_length<18)//正常数组长度
		{
			if(receive_buffer[5]=='-')//负数
			{
				if(receive_buffer[7]==':')//1
				{
				speed=output_one_neg_int_speed(receive_buffer);
				}
				else if (receive_buffer[8]==':')//2
				{
				speed=output_two_neg_int_speed( receive_buffer);
				
				}
				else if (receive_buffer[9]==':')//3
				{
				speed=output_three_neg_int_speed( receive_buffer);
				
				}
				else if (receive_buffer[10]==':')//4
				{
				speed=output_four_neg_int_speed( receive_buffer);
				
				}
			
			}
			else//正数
			{
				if(receive_buffer[6]==':')//1
				{
				speed=output_one_pos_int_speed( receive_buffer);
				}
				else if (receive_buffer[7]==':')//2
				{
				
				speed=output_two_pos_int_speed( receive_buffer);
				}
				else if (receive_buffer[8]==':')//3
				{
				speed=output_three_pos_int_speed( receive_buffer);
				
				}
				else if (receive_buffer[9]==':')//4
				{
				speed=output_four_pos_int_speed( receive_buffer);
				
				}
					
			
			}
			
		}
		else //不能正常采集到数据
		{
		speed=0;
		
		}
	 return speed;
	}
	
	int output_four_pos_int_speed(uint8_t speed_buffer[20])//四位正数
{
       int rx_speed1_num;
       int rx_speed2_num;
       int rx_speed_average;
     
       uint8_t rx_speed1[4];
       uint8_t rx_speed2[4];

       for(int i=5;i<9;i++)
		{
		rx_speed1[i-5]=speed_buffer[i];
		
		}
       for(int j=10;j<14;j++)
		{
		rx_speed2[j-10]=speed_buffer[j];
		}
		
		
			//convert to int
    rx_speed1_num=(rx_speed1[0]-0x30)*1000+(rx_speed1[1]-0x30)*100+(rx_speed1[2]-0x30)*10+(rx_speed1[3]-0x30)*1;
    rx_speed2_num=(rx_speed2[0]-0x30)*1000+(rx_speed2[1]-0x30)*100+(rx_speed2[2]-0x30)*10+(rx_speed2[3]-0x30)*1;
    rx_speed_average=(rx_speed1_num+rx_speed2_num)/2;
	
   return rx_speed_average;
}

int  output_four_neg_int_speed(uint8_t speed_buffer2[20])  //四位负数
{
       int rx_speed1_num;
       int rx_speed2_num;
       int rx_speed_average;
     
       uint8_t rx_speed1[4];
       uint8_t rx_speed2[4];

       for(int i=6;i<10;i++)
		{
		rx_speed1[i-6]=speed_buffer2[i];
		
		}
       for(int j=11;j<14;j++)
		{
		rx_speed2[j-11]=speed_buffer2[j];
		}
		
		
			//convert to int
    rx_speed1_num=(rx_speed1[0]-0x30)*1000+(rx_speed1[1]-0x30)*100+(rx_speed1[2]-0x30)*10+(rx_speed1[3]-0x30)*1;
    rx_speed2_num=(rx_speed2[0]-0x30)*1000+(rx_speed2[1]-0x30)*100+(rx_speed2[2]-0x30)*10+(rx_speed2[3]-0x30)*1;
    rx_speed_average=(rx_speed1_num+rx_speed2_num)/2;
   	
   return rx_speed_average;
}

 
 int output_three_pos_int_speed(uint8_t speed_buffer[20])//三位正数
{
       int rx_speed1_num;
       int rx_speed2_num;
       int rx_speed_average;
     
       uint8_t rx_speed1[3];
       uint8_t rx_speed2[3];

       for(int i=5;i<8;i++)
		{
		rx_speed1[i-5]=speed_buffer[i];
		
		}
       for(int j=9;j<12;j++)
		{
		rx_speed2[j-9]=speed_buffer[j];
		}
		
		
			//convert to int
    rx_speed1_num=(rx_speed1[0]-0x30)*100+(rx_speed1[1]-0x30)*10+(rx_speed1[2]-0x30)*1;
    rx_speed2_num=(rx_speed2[0]-0x30)*100+(rx_speed2[1]-0x30)*10+(rx_speed2[2]-0x30)*1;
    rx_speed_average=(rx_speed1_num+rx_speed2_num)/2;
	
   return rx_speed_average;
}

int  output_three_neg_int_speed(uint8_t speed_buffer2[20])  //三位负数
{
       int rx_speed1_num;
       int rx_speed2_num;
       int rx_speed_average;
     
       uint8_t rx_speed1[3];
       uint8_t rx_speed2[3];

       for(int i=6;i<9;i++)
		{
		rx_speed1[i-6]=speed_buffer2[i];
		
		}
       for(int j=11;j<14;j++)
		{
		rx_speed2[j-11]=speed_buffer2[j];
		}
		
		
			//convert to int
    rx_speed1_num=(rx_speed1[0]-0x30)*100+(rx_speed1[1]-0x30)*10+(rx_speed1[2]-0x30)*1;
    rx_speed2_num=(rx_speed2[0]-0x30)*100+(rx_speed2[1]-0x30)*10+(rx_speed2[2]-0x30)*1;
    rx_speed_average=(rx_speed1_num+rx_speed2_num)/2;
   	
   return rx_speed_average;
}

 int output_two_pos_int_speed(uint8_t speed_buffer[20])//二位正数
{
       int rx_speed1_num;
       int rx_speed2_num;
       int rx_speed_average;
     
       uint8_t rx_speed1[2];
       uint8_t rx_speed2[2];

			for(int i=5;i<7;i++)
		{
		rx_speed1[i-5]=speed_buffer[i];
		
		}
       for(int j=8;j<10;j++)
		{
		rx_speed2[j-8]=speed_buffer[j];
		}
		
		
			//convert to int
    rx_speed1_num=(rx_speed1[0]-0x30)*10+(rx_speed1[1]-0x30)*1;
    rx_speed2_num=(rx_speed2[0]-0x30)*10+(rx_speed2[1]-0x30)*1;
    rx_speed_average=(rx_speed1_num+rx_speed2_num)/2;

	
   return rx_speed_average;
}

int  output_two_neg_int_speed(uint8_t speed_buffer2[20])  //二位负数
{
       int rx_speed1_num;
       int rx_speed2_num;
       int rx_speed_average;
     
       uint8_t rx_speed1[2];
       uint8_t rx_speed2[2];
	 

       for(int i=6;i<8;i++)
		{
		rx_speed1[i-6]=speed_buffer2[i];
		
		}
       for(int j=10;j<12;j++)
		{
		rx_speed2[j-10]=speed_buffer2[j];
		}
				
			//convert to int
    rx_speed1_num=(rx_speed1[0]-0x30)*10+(rx_speed1[1]-0x30)*1;
    rx_speed2_num=(rx_speed2[0]-0x30)*10+(rx_speed2[1]-0x30)*1;
    rx_speed_average=(rx_speed1_num+rx_speed2_num)/2;
    
		
   return rx_speed_average;
}


 int output_one_pos_int_speed(uint8_t speed_buffer[20])//一位正数
{
       int rx_speed1_num;
       int rx_speed2_num;
       int rx_speed_average;
     
       uint8_t rx_speed1[1];
       uint8_t rx_speed2[1];

			for(int i=5;i<6;i++)
		{
		rx_speed1[i-5]=speed_buffer[i];
		
		}
       for(int j=7;j<8;j++)
		{
		rx_speed2[j-7]=speed_buffer[j];
		}
		
		
			//convert to int
    rx_speed1_num=(rx_speed1[0]-0x30)*1;
    rx_speed2_num=(rx_speed2[0]-0x30)*1;
    rx_speed_average=(rx_speed1_num+rx_speed2_num)/2;

	
   return rx_speed_average;
}

int  output_one_neg_int_speed(uint8_t speed_buffer2[20])  //一位负数
{
       int rx_speed1_num;
       int rx_speed2_num;
       int rx_speed_average;
     
       uint8_t rx_speed1[1];
       uint8_t rx_speed2[1];
	 

       for(int i=6;i<7;i++)
		{
		rx_speed1[i-6]=speed_buffer2[i];
		
		}
       for(int j=9;j<10;j++)
		{
		rx_speed2[j-9]=speed_buffer2[j];
		}
				
			//convert to int
    rx_speed1_num=(rx_speed1[0]-0x30)*1;//char转化为int
    rx_speed2_num=(rx_speed2[0]-0x30)*1;
    rx_speed_average=(rx_speed1_num+rx_speed2_num)/2;
    
		
   return rx_speed_average;
}




