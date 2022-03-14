#include "ros/ros.h"
#include "dynamic_reconfigure/server.h"
#include "parameter_set/drConfig.h"
#include "otter_control/controller.h"
#include "/home/ros/catkin_ws/src/complete_coverage/guidance/include/guidance/guidance.h"

#include <iostream>
#include <string.h>
using namespace std;

//本地参数
string str;
double m_path_[point_number];
int velocity_set,list_param;
double V_P,V_I,V_D,T_P,T_I,T_D,Connect_P,Connect_I,Connect_D,Stick_P,Stick_I,Stick_D,Connect_P_orien,Connect_I_orien;

//函数声明 
int GetInt(); 
float GetFloat(string str);
float SVF(char array[]);

void OtterController::set_param(){

    kp_con_orient = Connect_P_orien;
    ki_con_orient = Connect_I_orien;
    // Heading controller
    Kp_psi = -T_P;
    Ki_psi = -T_I;
    Kd_psi = -T_D;

    // Speed controller
    Kp_u = -V_P;
    Ki_u = -V_I;

    //connected controller
    kp_con = Connect_P;
    ki_con = Connect_I;
    kd_con = Connect_D;

    //点保持参数
    kp_stick = Stick_P;
    ki_stick = Stick_I;
    kd_stick = Stick_D;


}

void otter_coverage::Guidance::path_set(){

    for(int i =0; i < point_number; i++)
        m_path[i] = m_path_[i]; //如果要添加路径点数，需要修改point_number， 在guidance.h

    u = velocity_set; 


}

void cb(parameter_set::drConfig& config, uint32_t level){
    // ROS_INFO("parameter_show:%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d,%s",
    //     config.velocity_set,
    //     config.V_P,
    //     config.V_I,
    //     config.V_D,
    //     config.T_P,
    //     config.T_I,
    //     config.T_D,
    //     config.Connect_P,
    //     config.Connect_I,
    //     config.Connect_D,
    //     config.Connect_P_orien,
    //     config.Connect_I_orien,
    //     config.Stick_P,
    //     config.Stick_I,
    //     config.Stick_D,
    //     config.list_param,
    //     config.path_point.c_str()
    // );
    velocity_set = config.velocity_set;
    V_P = config.V_P;
    V_I = config.V_I;
    V_D = config.V_D;
    V_P = config.T_P;
    T_I = config.T_I;
    T_D = config.T_D;
    Connect_P = config.Connect_P;
    Connect_I = config.Connect_I;
    Connect_D = config.Connect_D;
    Connect_P_orien = config.Connect_P_orien;
    Connect_I_orien = config.Connect_I_orien;
    Stick_P = config.Stick_P;
    Stick_I = config.Stick_I;
    Stick_D = config.Stick_D;
    list_param = config.list_param;
    
    str = config.path_point.c_str();
    GetFloat(str);

}
 
float SVF(char array[])
{
	float value;
    /* 需要注意的是，这里没有终结符，故需要知道数组的 */
    /* 大小（数组的大小是编译时常量）*/
    char *dest_str; // 目标字符串
    dest_str = (char *)malloc(sizeof(char) * (sizeof(array) + 1));/* 为字符串分配堆空间 */
    strncpy(dest_str, array, sizeof(array));// 用C标准库函数strncpy拷贝字符
	value = atof(dest_str);
	return value;
}
float GetFloat(string str)
{
	//string str="(10+22.2+100+12.2+2.30)*2.0";
	//string str="(11.0+33.0+100+0.11)*22.0";
	// cout<<str<<endl;
	char TempStr[10];
	const char *pstr;
	pstr = &str[0];
	int i = 0, j = 0;
	int k = 0, m,g;
	int e10;
	int digit;
    int count = 0;
	for (i = 0; *(pstr + i) != '\0'; i++){
		if ((*(pstr + i) >= '0') && (*(pstr + i) <= '9'))
			j++;
		else{
			if(*(pstr + i) == '.') 
			{
				i=i+1;
				j=j+1;
				while((*(pstr + i) >= '0') && (*(pstr + i) <= '9')){
					j++;
					i++;
				}	
				if (j > 0){

					for(k=0;k<j;k++)
					{
						TempStr[k]=str[i-j+k];

					}
					float value;
					value = SVF(TempStr);
					memset(TempStr, 0, sizeof TempStr);          
					// cout<<"当前的小数值："<<value<<endl;                    
					j=0;
				}
			}
			else
			{
				j=0;
			}
		}
	}
} 

int main(int argc, char *argv[])
{

    ros::init(argc,argv,"dr");

    dynamic_reconfigure::Server<parameter_set::drConfig> server;

    dynamic_reconfigure::Server<parameter_set::drConfig>::CallbackType cbType;
    cbType = boost::bind(&cb,_1,_2);

    server.setCallback(cbType);

    ros::spin();
    return 0;
}
