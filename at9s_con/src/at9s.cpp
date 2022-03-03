#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ipc.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <asm-generic/termbits.h>
#include <errno.h>
#include <time.h>
#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>

#define RCINPUT_MEASURE_INTERVAL_US 4700
/* define range mapping here, -+100% -> 1000..2000 */
#define SBUS_RANGE_MIN 200.0f
#define SBUS_RANGE_MAX 1800.0f
#define SBUS_TARGET_MIN 1000.0f
#define SBUS_TARGET_MAX 2000.0f
/* pre-calculate the floating point stuff as far as possible at compile time */
#define SBUS_SCALE_FACTOR ((SBUS_TARGET_MAX - SBUS_TARGET_MIN) / (SBUS_RANGE_MAX - SBUS_RANGE_MIN))
#define SBUS_SCALE_OFFSET (int)(SBUS_TARGET_MIN - (SBUS_SCALE_FACTOR * SBUS_RANGE_MIN + 0.5f))

clock_t  start;
int      device_fd;                /** serial port device to read SBUS; */
int      channels_data[16]; /** 16 channels support; */
uint8_t  buffer[25];
char     device[30] = "/dev/ttyUSB1";
uint8_t  sbusData[25] = { 0x0f, 0x01, 0x04, 0x20, 0x00, 0xff, 0x07, 0x40, 0x00, 0x02, 0x10, 
           0x80, 0x2c, 0x64, 0x21, 0x0b, 0x59, 0x08, 0x40, 0x00, 0x02, 0x10, 0x80, 0x00, 0x00 };

int init()
{
    /* open the serial port */
    device_fd = open(device, O_RDWR | O_NONBLOCK | O_CLOEXEC);

    if (-1 == device_fd) {
        printf("Open SBUS input %s failed, status %d \n", device, (int) device_fd);
        fflush(stdout);
        return -1;
    }

    struct termios2 tio;
    if (0 != ioctl(device_fd, TCGETS2, &tio)) {
        close(device_fd);
        device_fd = -1;
        return -1;
    }

    /* Setting serial port,8E2, non-blocking.100Kbps */
    tio.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    tio.c_iflag |= (INPCK | IGNPAR);
    tio.c_oflag &= ~OPOST;
    tio.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    tio.c_cflag &= ~(CSIZE | CRTSCTS | PARODD | CBAUD);
    /* use BOTHER to specify speed directly in c_[io]speed member */
    tio.c_cflag |= (CS8 | CSTOPB | CLOCAL | PARENB | BOTHER | CREAD);
    tio.c_ispeed = 100000;
    tio.c_ospeed = 100000;
    tio.c_cc[VMIN] = 25;
    tio.c_cc[VTIME] = 0;

    if (0 != ioctl(device_fd, TCSETS2, &tio)) {
        close(device_fd);
        device_fd = -1;
        return -1;
    }

    printf("Open SBUS input %s worked, status %d \n", device, (int) device_fd);
    return 0;
}

 // 数组数据整体按位右移一位
#define MSB        0x80
#define LSB        0x01
 int right_shift(unsigned char *str, int len)
 {
     int i;
     
     for(i = len-1; i >= 0; i--)
     {
         str[i] = str[i] >> 1;
         if(i > 0 && str[i-1] & LSB)
         {
             str[i] = str[i] | MSB;    
         }    
     }
     
     return 0;    
}

int cnt = 0;
void read_pkt(void)
{
    int nread;
    int count_bad = 0;
    int ptr = 0;
    uint8_t  tempData[25];
    while (1) {
        ptr = 0;
        while (ptr < 25){

            nread = read(device_fd, &tempData , sizeof(tempData));

            // right_shift(tempData, sizeof(tempData)/sizeof(tempData[0]));
            // printf("nread=%d ptr = %d\n",nread,ptr);
            // for(int y=0;y<nread;y++)printf("0x%02x ",tempData[y]);
            // printf("\n");

            for( int x = 0; x < nread; x=x+1){
                sbusData[ptr] = tempData[x];
                ////printf("    x=%2d ptr = %2d 0x%02x 0x%02x\n",x,ptr,sbusData[ptr],tempData[x]);
                ptr = ptr + 1;
            }
            usleep(5000);
        }    
        //if(nread>0 && nread != 25)printf("ptr=%d\n",ptr);
        //good packet
        if (25 == ptr){
            if (0x0f == sbusData[0] && 0x00 == sbusData[24]) { break; } 
            else { ++count_bad; printf("0x%02x, 0x%02x\n",sbusData[0],sbusData[24]);} 
        }
    }

    /* parse sbus data to pwm */
    channels_data[0]  = (uint16_t)(((sbusData[1]       | sbusData[2]  << 8) & 0x07FF) * SBUS_SCALE_FACTOR + .5f) + SBUS_SCALE_OFFSET;
    channels_data[1]  = (uint16_t)(((sbusData[2]  >> 3 | sbusData[3]  << 5) & 0x07FF) * SBUS_SCALE_FACTOR + .5f) + SBUS_SCALE_OFFSET;
    channels_data[2]  = (uint16_t)(((sbusData[3]  >> 6 | sbusData[4]  << 2 | sbusData[5] << 10) & 0x07FF) * SBUS_SCALE_FACTOR + .5f) + SBUS_SCALE_OFFSET;
    channels_data[3]  = (uint16_t)(((sbusData[5]  >> 1 | sbusData[6]  << 7) & 0x07FF) * SBUS_SCALE_FACTOR + .5f) + SBUS_SCALE_OFFSET;
    channels_data[4]  = (uint16_t)(((sbusData[6]  >> 4 | sbusData[7]  << 4) & 0x07FF) * SBUS_SCALE_FACTOR + .5f) + SBUS_SCALE_OFFSET;
    channels_data[5]  = (uint16_t)(((sbusData[7]  >> 7 | sbusData[8]  << 1 | sbusData[9] << 9) & 0x07FF) * SBUS_SCALE_FACTOR + .5f) + SBUS_SCALE_OFFSET; 
    channels_data[6]  = (uint16_t)(((sbusData[9]  >> 2 | sbusData[10] << 6) & 0x07FF) * SBUS_SCALE_FACTOR + .5f) + SBUS_SCALE_OFFSET;
    channels_data[7]  = (uint16_t)(((sbusData[10] >> 5 | sbusData[11] << 3) & 0x07FF) * SBUS_SCALE_FACTOR + .5f) + SBUS_SCALE_OFFSET; 
    // & the other 8 + 2 channels if you need them
    channels_data[8]  = (uint16_t)(((sbusData[12]      | sbusData[13] << 8) & 0x07FF) * SBUS_SCALE_FACTOR + .5f) + SBUS_SCALE_OFFSET;
    channels_data[9]  = (uint16_t)(((sbusData[13] >> 3 | sbusData[14] << 5) & 0x07FF) * SBUS_SCALE_FACTOR + .5f) + SBUS_SCALE_OFFSET;
    channels_data[10] = (uint16_t)(((sbusData[14] >> 6 | sbusData[15] << 2 | sbusData[16] << 10) & 0x07FF) * SBUS_SCALE_FACTOR + .5f) + SBUS_SCALE_OFFSET;
    channels_data[11] = (uint16_t)(((sbusData[16] >> 1 | sbusData[17] << 7) & 0x07FF) * SBUS_SCALE_FACTOR + .5f) + SBUS_SCALE_OFFSET;
    channels_data[12] = (uint16_t)(((sbusData[17] >> 4 | sbusData[18] << 4) & 0x07FF) * SBUS_SCALE_FACTOR + .5f) + SBUS_SCALE_OFFSET;
    channels_data[13] = (uint16_t)(((sbusData[18] >> 7 | sbusData[19] << 1 | sbusData[20] << 9) & 0x07FF) * SBUS_SCALE_FACTOR + .5f) + SBUS_SCALE_OFFSET;
    channels_data[14] = (uint16_t)(((sbusData[20] >> 2 | sbusData[21] << 6) & 0x07FF) * SBUS_SCALE_FACTOR + .5f) + SBUS_SCALE_OFFSET;
    channels_data[15] = (uint16_t)(((sbusData[21] >> 5 | sbusData[22] << 3) & 0x07FF) * SBUS_SCALE_FACTOR + .5f) + SBUS_SCALE_OFFSET;

    // printf("%8.3fms  %3d  ch1 =%5d   ch2 =%5d   ch3 =%5d   ch4 =%5d   ch5 =%5d ch8 =%5d ch9 =%5d\n", 
    //    (float) 0.001*(clock()-start),  count_bad, channels_data[0], channels_data[1], channels_data[2], channels_data[3],channels_data[4],channels_data[8],channels_data[9]);     
    fflush(stdout);
}

int write_pkt () {
    channels_data[0] = 1500; channels_data[1] = 1500; channels_data[2] = 1500; channels_data[3] = 1510;
    channels_data[4] = 2000; channels_data[5] = 1000; channels_data[6] = 1000; channels_data[7] = 1000;
    channels_data[8] = 1000; channels_data[9] = 1000; channels_data[10]= 1000; channels_data[11]= 1000;
    channels_data[12]= 1000; channels_data[13]= 1000; channels_data[14]= 1000; channels_data[15]= 1000;
    for (int x= 0; x <16; x++){
       channels_data[x] = 0x07ff & (int) (((channels_data[x]-SBUS_TARGET_MIN)/SBUS_SCALE_FACTOR)+1.0f) + (int)SBUS_RANGE_MIN;
    }
    sbusData[0]  = 0x0f;
    sbusData[1]  = 0xff & (                         ((channels_data[0]%(1<<9))<<0));   // 0.7 0.6 0.5 0.4 0.3 0.2 0.1 0.0
    sbusData[2]  = 0xff & ((channels_data[0]>>8)  + ((channels_data[1]%(1<<6))<<3));   // 1.4 1.3 1.2 1.1 1.0 0.a 0.9 0.8
    sbusData[3]  = 0xff & ((channels_data[1]>>5)  + ((channels_data[2]%(1<<2))<<6));   // 2.1 2.0 1.a 1.9 1.8 1.7 1.6 1.5
    sbusData[4]  = 0xff & (                       + ((channels_data[2]%(1<<10))>>2));  // 2.9 2.8 2.7 2.6 2.5 2.4 2.3 2.2
    sbusData[5]  = 0xff & ((channels_data[2]>>10) + ((channels_data[3]%(1<<7))<<1));   // 3.6 3.5 3.4 3.3 3.2 3.1 3.0 2.a
    sbusData[6]  = 0xff & ((channels_data[3]>>7)  + ((channels_data[4]%(1<<4))<<4));   // 4.3 4.2 4.1 4.0 3.a 3.9 3.8 3.7
    sbusData[7]  = 0xff & ((channels_data[4]>>4)  + ((channels_data[5]%(1<<1))<<7));   // 5.0 4.a 4.9 4.8 4.7 4.6 4.5 4.4
    sbusData[8]  = 0xff & ((channels_data[5]>>1)  + ((channels_data[5]%(1<<9))>>1));   // 5.8 5.7 5.6 5.5 5.4 5.3 5.2 5.1
    sbusData[9]  = 0xff & ((channels_data[5]>>9)  + ((channels_data[6]%(1<<6))<<2));   // 6.5 6.4 6.3 6.2 6.1 6.0 5.a 5.9
    sbusData[10] = 0xff & ((channels_data[6]>>6)  + ((channels_data[7]%(1<<3))<<5));   // 7.2 7.1 7.0 6.a 6.9 6.8 6.7 6.6
    sbusData[11] = 0xff & ((channels_data[7]>>3)                                  );   // 7.a 7.9 7.8 7.7 7.6 7.5 7.4 7.3
    sbusData[12] = 0xff & (                         ((channels_data[8]%(1<<9))<<0));   // same pattern channels_data[x+8]
    sbusData[13] = 0xff & ((channels_data[8]>>8)  + ((channels_data[9]%(1<<6))<<3));   
    sbusData[14] = 0xff & ((channels_data[9]>>5)  + ((channels_data[10]%(1<<2))<<6));  
    sbusData[15] = 0xff & (                       + ((channels_data[10]%(1<<10))>>2)); 
    sbusData[16] = 0xff & ((channels_data[10]>>10)+ ((channels_data[11]%(1<<7))<<1));  
    sbusData[17] = 0xff & ((channels_data[11]>>7) + ((channels_data[12]%(1<<4))<<4));  
    sbusData[18] = 0xff & ((channels_data[12]>>4) + ((channels_data[13]%(1<<1))<<7));  
    sbusData[19] = 0xff & ((channels_data[13]>>1) + ((channels_data[13]%(1<<9))>>1));  
    sbusData[20] = 0xff & ((channels_data[13]>>9) + ((channels_data[14]%(1<<6))<<2));  
    sbusData[21] = 0xff & ((channels_data[14]>>6) + ((channels_data[15]%(1<<3))<<5));  
    sbusData[22] = 0xff & ((channels_data[15]>>3)                                  );  
    sbusData[23] = 0x00;
    sbusData[24] = 0x00;
    write(device_fd, &sbusData, sizeof(sbusData));
}

int main(int argc, char **argv)
{
    //if (argc == 1){strcpy(device,argv[1]);}
    start = clock();
    init();

    ros::init(argc, argv, "at9s_node");
    ros::Publisher at9s_pub;
    ros::NodeHandle nh;

    at9s_pub = nh.advertise<std_msgs::Int32MultiArray>("keyboard_control", 10);

    double frequency = 100.0;
    double deltaTime = 1.0 / frequency;
    ros::Rate rate(frequency);

    while(nh.ok()){ 
        
        read_pkt(); 

        std_msgs::Int32MultiArray at9s;

        for(int i = 0; i < 16; i++)
            at9s.data.push_back(channels_data[i]);
        
        at9s_pub.publish(at9s);

        // printf("ch1 =%5d   ch2 =%5d   ch3 =%5d   ch4 =%5d   ch5 =%5d ch9 =%5d ch10 =%5d\n", 
        //      at9s.data[0], at9s.data[1], at9s.data[2], at9s.data[3],at9s.data[4],at9s.data[8],at9s.data[9]); 
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}