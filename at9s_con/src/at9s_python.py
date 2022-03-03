#!/usr/bin/env python
#-*- coding: utf-8 -*-

import array #array模块是python中实现的一种高效的数组存储类型
import serial #serial模块封装了对串行端口的访问
import codecs #Python中专门用作编码转换的模块
import time

class SBUSReceiver():
    def __init__(self, _uart_port='/dev/ttyUSB0'):

        #初始化树莓派串口参数
        self.ser = serial.Serial(
            port=_uart_port,            #树莓派的硬件串口/dev/ttyAMA0
            baudrate = 100000,          #波特率为100k
            parity=serial.PARITY_EVEN,  #偶校验
            stopbits=serial.STOPBITS_TWO,#2个停止位 STOPBITS_TWO
            bytesize=serial.EIGHTBITS,   #8个数据位
            timeout = 0,
        )

        # 常数
        self.START_BYTE = b'\x0f'  #起始字节为0x0f
        self.END_BYTE = b'\x00'     #结束字节为0x00
        self.SBUS_FRAME_LEN = 25    #SBUS帧有25个字节
        self.SBUS_NUM_CHAN = 18     #18个通道
        self.OUT_OF_SYNC_THD = 10
        self.SBUS_NUM_CHANNELS = 18 #18个通道
        self.SBUS_SIGNAL_OK = 0     #信号正常为0
        self.SBUS_SIGNAL_LOST = 1       #信号丢失为1
        self.SBUS_SIGNAL_FAILSAFE = 2   #输出failsafe信号时为2

        # 堆栈变量初始化
        self.isReady = True
        self.lastFrameTime = 0
        self.sbusBuff = bytearray(1)  # 用于同步的单个字节
        #bytearray(n) 方法返回一个长度为n的初始化数组；
        self.sbusFrame = bytearray(25)  # 单个SBUS数据帧，25个字节
        self.sbusChannels = array.array('H', [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])  # 接收到的各频道值
        #array.array(typecode,[initializer]) --typecode:元素类型代码；initializer:初始化器，若数组为空，则省略初始化器
        self.failSafeStatus = self.SBUS_SIGNAL_FAILSAFE


    def get_rx_channels(self):
        """
        用于读取最后的SBUS通道值
        返回:由18个无符号短元素组成的数组，包含16个标准通道值+ 2个数字(ch17和18)
        """
        return self.sbusChannels

    def get_rx_channel(self, num_ch):
        """
        用于读取最后的SBUS某一特定通道的值
        num_ch: 要读取的某个通道的通道序号
        返回:某一通道的值
        """
        return self.sbusChannels[num_ch]

    def get_failsafe_status(self):
        """
        用于获取最后的FAILSAFE状态
        返回: FAILSAFE状态值
        """
        return self.failSafeStatus


    def decode_frame(self):
        """
        对每帧数据进行解码，每个通道的值在两个或三个不同的字节之间，要读取出来很麻烦
        不过futaba已经发布了下面的解码代码
        """
        def toInt(_from):
            #encode() 方法以指定的编码格式编码字符串。
            #int() 函数用于将一个字符串或数字转换为整型。
            return int(codecs.encode(_from, 'hex'), 16) 
            
        #CH1 = [data2]的低3位 + [data1]的8位（678+12345678 = 678,12345678）
        self.sbusChannels[0]  = ((toInt(self.sbusFrame[1])      |toInt(self.sbusFrame[2])<<8)                                   & 0x07FF);
        #CH2 = [data3]的低6位 + [data2]的高5位（345678+12345 = 345678,12345 ）
        self.sbusChannels[1]  = ((toInt(self.sbusFrame[2])>>3   |toInt(self.sbusFrame[3])<<5)                                   & 0x07FF);
        #CH3 = [data5]的低1位 + [data4]的8位 + [data3]的高2位（8+12345678+12 = 8,12345678,12）
        self.sbusChannels[2]  = ((toInt(self.sbusFrame[3])>>6   |toInt(self.sbusFrame[4])<<2 |toInt(self.sbusFrame[5])<<10)     & 0x07FF);
        
        self.sbusChannels[3]  = ((toInt(self.sbusFrame[5])>>1   |toInt(self.sbusFrame[6])<<7)                                   & 0x07FF);
        self.sbusChannels[4]  = ((toInt(self.sbusFrame[6])>>4   |toInt(self.sbusFrame[7])<<4)                                   & 0x07FF);
        self.sbusChannels[5]  = ((toInt(self.sbusFrame[7])>>7   |toInt(self.sbusFrame[8])<<1 |toInt(self.sbusFrame[9])<<9)      & 0x07FF);
        self.sbusChannels[6]  = ((toInt(self.sbusFrame[9])>>2   |toInt(self.sbusFrame[10])<<6)                                  & 0x07FF);
        self.sbusChannels[7]  = ((toInt(self.sbusFrame[10])>>5  |toInt(self.sbusFrame[11])<<3)                                  & 0x07FF);
        self.sbusChannels[8]  = ((toInt(self.sbusFrame[12])     |toInt(self.sbusFrame[13])<<8)                                  & 0x07FF);
        self.sbusChannels[9]  = ((toInt(self.sbusFrame[13])>>3  |toInt(self.sbusFrame[14])<<5)                                  & 0x07FF);
        self.sbusChannels[10] = ((toInt(self.sbusFrame[14])>>6  |toInt(self.sbusFrame[15])<<2|toInt(self.sbusFrame[16])<<10)    & 0x07FF);
        self.sbusChannels[11] = ((toInt(self.sbusFrame[16])>>1  |toInt(self.sbusFrame[17])<<7)                                  & 0x07FF);
        self.sbusChannels[12] = ((toInt(self.sbusFrame[17])>>4  |toInt(self.sbusFrame[18])<<4)                                  & 0x07FF);
        self.sbusChannels[13] = ((toInt(self.sbusFrame[18])>>7  |toInt(self.sbusFrame[19])<<1|toInt(self.sbusFrame[20])<<9)     & 0x07FF);
        self.sbusChannels[14] = ((toInt(self.sbusFrame[20])>>2  |toInt(self.sbusFrame[21])<<6)                                  & 0x07FF);
        self.sbusChannels[15] = ((toInt(self.sbusFrame[21])>>5  |toInt(self.sbusFrame[22])<<3)                                  & 0x07FF);

        

        #17频道，第24字节的最低一位
        if toInt(self.sbusFrame[23])  & 0x0001 :
            self.sbusChannels[16] = 2047
        else:
            self.sbusChannels[16] = 0
        #18频道，第24字节的低第二位，所以要右移一位
        if (toInt(self.sbusFrame[23]) >> 1) & 0x0001 :
            self.sbusChannels[17] = 2047
        else:
            self.sbusChannels[17] = 0

        #帧丢失位为1时，第24字节的低第三位，与0x04进行与运算
        self.failSafeStatus = self.SBUS_SIGNAL_OK
        if toInt(self.sbusFrame[23]) & (1 << 2):
            self.failSafeStatus = self.SBUS_SIGNAL_LOST
        #故障保护激活位为1时，第24字节的低第四位，与0x08进行与运算   
        if toInt(self.sbusFrame[23]) & (1 << 3):
            self.failSafeStatus = self.SBUS_SIGNAL_FAILSAFE


    def update(self):
        """
        我们需要至少2帧大小，以确保找到一个完整的帧
        所以我们取出所有的缓存（清空它），读取全部数据，直到捕获新的数据
        首先找到END BYTE并向后查找SBUS_FRAME_LEN，看看它是否是START BYTE
        """
        #我们是否有足够的数据在缓冲区和有没有线程在后台?
        if self.ser.inWaiting() >= self.SBUS_FRAME_LEN*2 and self.isReady: #inWaiting()返回接收缓存中的字节数
            self.isReady = False    #表明有线程在运行，isReady = False
            print(1)
            # 读取所有临时帧数据
            tempFrame = self.ser.read(self.ser.inWaiting())
            # 在缓冲区帧的每个字符中，我们寻找结束字节
            for end in range(0, self.SBUS_FRAME_LEN):
                #寻找结束字节，从后向前查找
                if tempFrame[len(tempFrame)-1-end] == self.END_BYTE :
                
                    #从最后的命中点减去SBUS_FRAME_LEN寻找起始字节
                    if tempFrame[len(tempFrame)-end-self.SBUS_FRAME_LEN] == self.START_BYTE :
                        # 如果相等，则帧数据正确，数据以8E2包到达，因此它已经被校验过

                        # 从临时帧数据中取出刚验证正确的一段正确帧数据
                        lastUpdate = tempFrame[len(tempFrame)-end-self.SBUS_FRAME_LEN:len(tempFrame)-1-end]
                        if not self.sbusFrame == lastUpdate: #相等即表示没有操作，不用再次解码
                            self.sbusFrame = lastUpdate
                            self.decode_frame() #调用解码函数

                        self.lastFrameTime = time.time() # 跟踪最近的更新时间
                        self.isReady = True
                        break

if __name__ == '__main__':

    sbus = SBUSReceiver('/dev/ttyUSB0')

    while True:
        time.sleep(0.001)

        # X8R的SBUS信号是间隔6ms发送一次，一次持续发送3ms；
        # 不要调用sbus.update()太快，如果sbus.ser.inWaiting()>50,且增长很多，可以调用sbus.update()快点，即time.sleep()延迟短点；
        # 如果sbus.ser.inWaiting()<50,可以调用sbus.update()慢点，即time.sleep()延迟长点；
        sbus.update()

        #在您的代码中，您可以调用sbus.get_rx_channels()来获取所有数据，或者调用sbus.get_rx_channels()[n]来获取第n个通道的值；
        #或get_rx_channel(self, num_ch)来获得第num_ch个通道的值；

        # print(sbus.get_failsafe_status(), sbus.get_rx_channels(), str(sbus.ser.inWaiting()).zfill(4) , (time.time()-sbus.lastFrameTime))
        
        #str() 函数将对象转化为适于人阅读的形式，将指定的值转换为字符串。
        #zfill() 方法返回指定长度的字符串，原字符串右对齐，前面填充0。
        #(time.time()-sbus.lastFrameTime)用于展示得到最近这次数据的延迟