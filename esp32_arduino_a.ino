#include <Arduino.h>

#include <ESP32Servo.h>
#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32MultiArray.h>

#include "SBUS.h"

// a SBUS object, which is on hardware
// serial port 1
SBUS x8r(Serial2);

// channel, fail safe, and lost frames data
uint16_t channels[16];
bool failSafe;
bool lostFrame;

Servo servo_left;
Servo servo_right;

Servo servo_head;
Servo servo_tail;

ros::NodeHandle nh;

int signal= 1500,a = 1;

int value1 = 1500;//左
int value2 = 1500;//右
int value3 = 1500;//头
int value4 = 1500;//尾

int lock_control[4] = {0,0,0,0}; //控制信号输出
#define a_up 25
#define a_down 26
#define b_up 27
#define b_down 14

float analog_value = 0;
//adc测量电压
#define ANALOG_PIN 13

//速度
void messageCb1( const std_msgs::Float32& toggle_msg){

  value1 = int(toggle_msg.data);

}

//方向
void messageCb2( const std_msgs::Float32& toggle_msg){

  value2 = int(toggle_msg.data);
 
}

void messageCb3( const std_msgs::Float32& toggle_msg){

  value3 = int(toggle_msg.data);
 
}

void messageCb4( const std_msgs::Float32& toggle_msg){

  value4 = int(toggle_msg.data);
 
}

void messageCb_lockControl(const std_msgs::Int32MultiArray& msg){
  
  lock_control[0] = msg.data[0];
  lock_control[1] = msg.data[1];
  lock_control[2] = msg.data[2];
  lock_control[3] = msg.data[3];
  
}

ros::Subscriber<std_msgs::Float32> sub1("left_thrust_cmd", &messageCb1);
ros::Subscriber<std_msgs::Float32> sub2("right_thrust_cmd", &messageCb2 );

ros::Subscriber<std_msgs::Float32> sub3("head_thrust_cmd", &messageCb3 );
ros::Subscriber<std_msgs::Float32> sub4("tail_thrust_cmd", &messageCb4 );

ros::Subscriber<std_msgs::Int32MultiArray> sub5("latch_command", &messageCb_lockControl );

//std_msgs::Float32 adc_msg;
//ros::Publisher chatter("voltage", &adc_msg);

void setup()
{
//   Serial.begin(115200);
 

//闩锁控制引脚
   pinMode(a_up, OUTPUT);
   pinMode(a_down, OUTPUT);
   pinMode(b_up, OUTPUT);
   pinMode(b_down, OUTPUT);
//   
   digitalWrite(a_up, HIGH);
   digitalWrite(a_down, HIGH);
   digitalWrite(a_down, HIGH);
   digitalWrite(a_down, HIGH);
//   delay(5000);
//   digitalWrite(a_up, 0);
//   delay(5000);
//   digitalWrite(a_up, 1);
//   delay(5000);
//   digitalWrite(a_up, 0);
//   delay(5000);
//   pinMode(a_down, OUTPUT);
//   pinMode(b_up, OUTPUT);
//   pinMode(b_down, OUTPUT);

   x8r.begin();
   
   nh.initNode(); //占用串口，所以无法打印，但是通信应该是可以得的

   nh.subscribe(sub1);
   nh.subscribe(sub2);
   nh.subscribe(sub3);
   nh.subscribe(sub4);
   nh.subscribe(sub5);
   
   delay(5000);
 
   servo_left.attach(18);//esp18
   servo_right.attach(5);//esp5
   servo_head.attach(17);//17->22
   servo_tail.attach(16);//16->23
  
   servo_left.writeMicroseconds(1500); 
   servo_right.writeMicroseconds(1500); 
   servo_head.writeMicroseconds(1500); 
   servo_tail.writeMicroseconds(1500); 

  delay(1500);//1500

}

void loop()
{

   static int count_a_up,count_b_up,count_a_down,count_b_down;
   static int lock_flag_a_up = 0,lock_flag_a_down = 0;
   static int lock_flag_b_up = 0,lock_flag_b_down = 0;

//  signal = signal + a*1;
//  if(signal >= 1900) a = -1;
//  else if(signal <= 1100) a = 1;
///////
   x8r.read(&channels[0], &failSafe, &lostFrame);
///////
//    for(int i = 0; i<16; i++){
//
//         Serial.print(i);
//         Serial.print(":");
//         Serial.print(channels[i]);
//         Serial.print("  ");
//      
//    }
//    Serial.print("\n");

// channels[8]和channels[4]各控制一个 向上306，控制钩子开关，一个方向给一个引脚输出一个低电平脉冲
     
   if(channels[9] == 306) {

    int a0 = 3000-((channels[1]+67)+483);
    int a1 = (1500 - (channels[0]+500));

    int  b0 = ((channels[1]+67)+483);
    int  b1 = (1500 - (channels[0]+500));

    int c0 = (channels[3]+500);
    int d0 = 3000-(channels[3]+500);

    int left = a0 - a1;
    int right = b0 - b1;

    if(left >= 1700) left = 1700;
    if(left <= 1300) left = 1300;

    if(right >= 1700) right = 1700;
    if(right <= 1300) right = 1300;

    if(c0 >= 1700) c0 = 1700;
    if(c0 <= 1300) c0 = 1300;

    if(d0 >= 1700) d0 = 1700;
    if(d0 <= 1300) d0 = 1300;

    if(channels[8] == 306){

       if(!lock_flag_a_up){
        
         count_a_up++;//控制脉冲时间
         if(count_a_up < 100){ //0.5秒
            digitalWrite(a_up, LOW);
            
         }
         else{
            digitalWrite(a_up, HIGH);
            count_a_up = 0;
            lock_flag_a_up = 1;
            lock_flag_a_down = 0;
         }
       } 
      }
    else{
       
       if(!lock_flag_a_down){
         
         count_a_down++;//控制脉冲时间
         if(count_a_down < 100){ //0.5秒
            digitalWrite(a_down, LOW);
            
         }
         else{
            digitalWrite(a_down, HIGH);
            count_a_down = 0;
            lock_flag_a_down = 1;
            lock_flag_a_up = 0;
         }
       } 
      }   

    if(channels[4] == 306){
       if(!lock_flag_b_up){
        
         count_b_up++;//控制脉冲时间
         if(count_b_up < 100){ //0.5秒
            digitalWrite(b_up, LOW);
            
         }
         else{
            digitalWrite(b_up, HIGH);
            count_b_up = 0;
            lock_flag_b_up = 1;
            lock_flag_b_down = 0;
         }
       } 
      }
    else{
       
       if(!lock_flag_b_down){
         
         count_b_down++;//控制脉冲时间
         if(count_b_down < 100){ //0.5秒
            digitalWrite(b_down, LOW);
            
         }
         else{
            digitalWrite(b_down, HIGH);
            count_b_down = 0;
            lock_flag_b_down = 1;
            lock_flag_b_up = 0;
         }
       } 
      }       
      
    
    servo_left.writeMicroseconds(left); //forward fan
    servo_right.writeMicroseconds(right); 
    servo_head.writeMicroseconds(c0); 
    servo_tail.writeMicroseconds(d0);    
   
    }

   else{

    servo_left.writeMicroseconds(value1); 
    servo_right.writeMicroseconds(value2); 
    servo_head.writeMicroseconds(value3); 
    servo_tail.writeMicroseconds(value4); 

    digitalWrite(a_up, lock_control[0]);
    digitalWrite(a_down, lock_control[1]);
    digitalWrite(b_up, lock_control[2]);
    digitalWrite(b_down, lock_control[3]);
    
   }

////  analog_value = analogRead(ANALOG_PIN);
////  adc_msg.data = analog_value;
////  chatter.publish( &adc_msg );



   nh.spinOnce();
   delay(5);
}