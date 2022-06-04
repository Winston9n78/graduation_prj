#ifndef PARAM_SERVER_H_
#define PARAM_SERVER_H_

#include <iostream>
#include <string.h>

#define point_number 2*5  //路径点数变多时对应修改

using namespace std;

int GetInt(); 
double GetFloat(string str, double (&path)[point_number]);
float SVF(char array[]);

#endif


