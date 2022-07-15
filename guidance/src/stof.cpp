#include "stof/stof.h"

 
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
double GetFloat(string str, double (&path)[point_number])
{
	//string str="(10+22.2+100+12.2+2.30)*2.0";
	//string str="(11.0+33.0+100+0.11)*22.0";
	// cout<<str<<endl;
	
	int p = 0;

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
					path[p] = value; 
					p++;                  
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

