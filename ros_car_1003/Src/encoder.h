 #include "stm32f1xx_hal.h"
 #include "pid.h"
 
 #define SPEED_SAMPLING_TIME  9   
 #define SPEED_SAMPLING_FREQ (uint16_t)(2000/(SPEED_SAMPLING_TIME+1))  //200hz，小车速度采样频率
 #define ENCODER_PPR           (uint16_t)(1024)  // 电机2码盘线数
 
 
int  output_four_pos_int_speed(uint8_t speed_buffer[20]);
int  output_four_neg_int_speed(uint8_t speed_buffer2[20]);

int  output_three_pos_int_speed(uint8_t speed_buffer2[20]);
int  output_three_neg_int_speed(uint8_t speed_buffer2[20]);

int  output_two_pos_int_speed(uint8_t speed_buffer2[20]);
int  output_two_neg_int_speed(uint8_t speed_buffer2[20]);

int  output_one_pos_int_speed(uint8_t speed_buffer2[20]);
int  output_one_neg_int_speed(uint8_t speed_buffer2[20]);

int output_speed (uint8_t receive_buffer[20],uint8_t receive_length);

