 #include "odometry.h"
#include "stm32f1xx_hal.h"
/***********************************************  输出  *****************************************************************/

float position_x=0,position_y=0,oriention=0,velocity_linear=0,velocity_angular=0;

/***********************************************  输入  *****************************************************************/


extern float odometry_right,odometry_left;//串口得到的左右轮速度

/***********************************************  变量  *****************************************************************/

float wheel_interval= 455.0;//    轴距
float multiplier=0.5f;           //倍频4
float deceleration_ratio=30.0f;  //减速比30
float wheel_diameter=310.0f;     //轮子直径，单位mm
float pi_1_2=1.570796f;			 //π/2
float pi=3.141593f;              //π
float pi_3_2=4.712389f;			 //π*3/2
float pi_2_1=6.283186f;			 //π*2
float dt=0.005f;                 //采样时间间隔5ms
float line_number=1024.0f;       //码盘线数
float oriention_interval=0;  //dt时间内方向变化值

float sin_=0;        //角度计算值
float cos_=0;

float delta_distance=0,delta_oriention=0;   //采样时间间隔内运动的距离

float const_frame=0,const_angle=0,distance_sum=0,distance_diff=0;

float oriention_1=0;

uint8_t once=1;

/****************************************************************************************************************/

//里程计计算函数
void odometry(float right,float left)
{	
	if(once)  //常数仅计算一次
	{
		const_frame=wheel_diameter*pi/(line_number*multiplier*deceleration_ratio);
		const_angle=const_frame/wheel_interval;
		once=0;
	}
    
	distance_sum = 0.5f*(right+left);//在很短的时间内，小车行驶的路程为两轮速度和
	distance_diff = right-left;//在很短的时间内，小车行驶的角度为两轮速度差

    //根据左右轮的方向，纠正短时间内，小车行驶的路程和角度量的正负
	if((odometry_right>0)&&(odometry_left>0))            //左右均正
	{
		delta_distance =25*distance_sum;
		delta_oriention = 0*distance_diff;
	}
	else if((odometry_right<0)&&(odometry_left<0))       //左右均负
	{
		delta_distance = -25*distance_sum;
		delta_oriention = 0*distance_diff;
	}
	else if((odometry_right>0)&&(odometry_left<0))       //左正右负
	{
		delta_distance = 0*distance_diff;//-38
		delta_oriention = -0*distance_sum;		
	}
	else if((odometry_right<0)&&(odometry_left>0))       //左负右正
	{
		delta_distance = 0*distance_diff;
		delta_oriention = 38 *distance_sum;
	}
	else
	{
		delta_distance=0;
		delta_oriention=0;
	}
    
	oriention_interval = delta_oriention * const_angle;//采样时间内走的角度	
	oriention = oriention + oriention_interval;//计算出里程计方向角
	oriention_1 = oriention + 0.5f * oriention_interval;//里程计方向角数据位数变化，用于三角函数计算
	
  sin_ = sin(oriention_1);//计算出采样时间内y坐标
	cos_ = cos(oriention_1);//计算出采样时间内x坐标
	
  position_x = position_x + delta_distance * cos_ * const_frame;//计算出里程计x坐标
	position_y = position_y + delta_distance * sin_ * const_frame;//计算出里程计y坐标
    
	velocity_linear = delta_distance*const_frame / dt;//计算出里程计线速度
	velocity_angular = oriention_interval / dt;//计算出里程计角速度
	
    //方向角角度纠正
	if(oriention > pi)
	{
		oriention -= pi_2_1;
	}
	else
	{
		if(oriention < -pi)
		{
			oriention += pi_2_1;
		}
	}
}
