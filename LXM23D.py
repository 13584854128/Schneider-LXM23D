#!/usr/bin/python
#__*__coding:utf-8__*__

#################################################################################
# modbus_tk 与LXM23D通信
# 伺服驱站号为02（地址：P3-00=0x02, 0x0300,0x0301)
# 波特率：9600（地址：P3-01=0x0101, 0x0302,0x0303)
# 通信格式 Modbus RTU, 8位数据位，奇校验 ODD, 1位停止位(地址：P3-02=0x08, 0x0304,0x0305)
# 伺服工作模式PR P1-01=1
# 电子齿轮比：P1-44=100， P1-45=1，若P6-03=12800,则每128000个脉冲为一圈
# 驱动器状态：P0-01(0002H,0003H)

###################################################################################
import serial
import modbus_tk
import modbus_tk.defines as cst
from modbus_tk import modbus_rtu
import sched
import time
import threading
import struct
from time import sleep
import sys


#ser = serial.Serial('COM8', 9600,parity=serial.PARITY_ODD,stopbits=serial.STOPBITS_ONE, timeout=2)
#ser.open()

class LXM():
    def __init__(self,servoAddress):
        self.servoAddress=servoAddress
        self.logger=modbus_tk.utils.create_logger('console')
        self.ser= serial.Serial('COM8', 9600,parity=serial.PARITY_ODD,stopbits=serial.STOPBITS_ONE, timeout=2)
        self.master=modbus_rtu.RtuMaster(self.ser)
        self.master.set_timeout(3)
        self.master.set_verbose(True)
        self.logger.info("connected")
        print("connected")

        self.comd_address=0x050E
        self.alm1_address=0x0400
        self.alm2_address=0x0402
        self.alm3_address=0x0406
        self.alm4_address=0x0408
        self.alm5_address=0x040A
        self.statusAddress=0x0002
        self.Done=True

    def servoOn(self):
        try:
            #logger.info(master.execute(101,cst.READ_HOLDING_REGISTERS,0,10)) # ‘101’从站站号，‘0’十进制起始地址，‘10’寄存器数量
            #logger.info(master.execute(0x02, cst.WRITE_SINGLE_REGISTER, 100, output_value=10000))#101-6-0-100-0-54-64-39
            #logger.info(master.execute(0x02, cst.WRITE_MULTIPLE_REGISTERS, 0x030C, output_value=10000))
            #P2-10=101 0x0214,0x0215, 输入引脚DI1功能规划,结合P3-06来确定DI1是由外部端子控制，还是通过通讯方式，由P4-07控制
            self.logger.info(self.master.execute(self.servoAddress, cst.WRITE_MULTIPLE_REGISTERS, 0x0214, output_value=[0x0101, 0x0000]))
        except Exception as e:
            print (e)
        try:
            # P3-06=01 0x030C,0x030D, 即确定DI1通过参数P4-07控制，而不是外部端子，该参数掉电后无法保持，需重设
            # 如果想上电即使能，就将CN1接口的11和17管脚短接、49和9管脚短接，并将P2-68设为1即可
            self.logger.info(self.master.execute(self.servoAddress, cst.WRITE_MULTIPLE_REGISTERS, 0x030C, output_value=[0x0001, 0x0000]))
        except Exception as e:
            print (e)
        try:
            # P4-07=1 0x040E,0x040F（数字输入接点多重功能P356）写入1时，即为将DI1输入接点置1，基于P2 - 10的设置，伺服电机被使能
            self.logger.info(self.master.execute(self.servoAddress, cst.WRITE_MULTIPLE_REGISTERS, 0x040E, output_value=[0x0001, 0x0000]))
        except Exception as e:
            print (e)
    def servoOff(self):
        try:
            # P4-07=0 0x040E,0x040F（数字输入接点多重功能P356）写入1时，即为将DI1输入接点置0，基于P2 - 10的设置，伺服电机被解除使能
            self.logger.info(self.master.execute(self.servoAddress, cst.WRITE_MULTIPLE_REGISTERS, 0x040E, output_value=[0x0000, 0x0000]))
        except Exception as e:
            print (e)
    def servoJogP(self):
        try:
            # P4-05=1000(0x03E8) 0x040A,0x040B 设定点动的速度为1000RPM,500RPM(0x01F4)100RPM(0x0064)
            self.logger.info(self.master.execute(self.servoAddress, cst.WRITE_MULTIPLE_REGISTERS, 0x040A, output_value=[0x000A, 0x0000]))
        except Exception as e:
            print (e)
        try:
            # P4-05=4498(0x1386) 0x040A,0x040B 设定点动的方向4498正向，4499反向
            self.logger.info(self.master.execute(self.servoAddress, cst.WRITE_MULTIPLE_REGISTERS, 0x040A, output_value=[0x1386, 0x0000]))
        except Exception as e:
            print (e)
    def servoJogN(self):
        try:
            # P4-05=1000(0x03E8) 0x040A,0x040B 设定点动的速度为1000RPM,500RPM(0x01F4),100RPM(0x0064)
            self.logger.info(self.master.execute(self.servoAddress, cst.WRITE_MULTIPLE_REGISTERS, 0x040A, output_value=[0x000A, 0x0000]))
        except Exception as e:
            print (e)
        try:
            # P4-05=4499(0x1387) 0x040A,0x040B 设定点动的方向4498正向，4499反向
            self.logger.info(self.master.execute(self.servoAddress, cst.WRITE_MULTIPLE_REGISTERS, 0x040A, output_value=[0x1387, 0x0000]))
        except Exception as e:
            print (e)
    def servoJogStop(self):
        try:
            # P4-05=0(0x0000) 0x040A,0x040B 设定点动停止
            self.logger.info(self.master.execute(self.servoAddress, cst.WRITE_MULTIPLE_REGISTERS, 0x040A, output_value=[0x0000, 0x0000]))
        except Exception as e:
            print (e)
    '''
    Pr模式的位置数据，全部以使用者单位PUU(Pulse of User Unit) 表示。也代表上位
    机的位置单位，与驱动器内部的位置单位的比例，即为驱动器的电子齿轮比。
    1) 驱动器的位置单位(pulse)：编码器单位，每转1280000脉冲(pulse / rev)，固
    定不变。
    2) 使用者单位(PUU)：上位机单位，若每转为P脉冲(PUU / rev)，则齿轮比须设定
    为：GEAR_NUM(P1 - 44) / GEAR_DEN(P1 - 45) = 1280000 / P
    本例P1-44设为100，P-45设为1，设每12800个脉冲为一圈   
    '''
    def oneStep(self,speed,pulse,cmd):
        try:
            # P5-60 0x0578, 0x0579内部目标速度设定 0#，设为200
            self.logger.info(self.master.execute(self.servoAddress, cst.WRITE_MULTIPLE_REGISTERS, 0x0578,
                                                 output_value=[speed, 0x0000]))
        except Exception as e:
            print (e)
        try:
            # P6 - 02 0604H, 0605H设置位置模式，相对位置模式 0000 0080，绝对位置模式 0000 0000
            self.logger.info(self.master.execute(self.servoAddress, cst.WRITE_MULTIPLE_REGISTERS, 0x0604,
                                                 output_value=[0x0080, 0x0000]))
        except Exception as e:
            print (e)
        try:
            # P6 - 03 0606H, 0607H设定目标位置,电机反馈脉冲数（使用者单位），如设为A，则每次运动后，驱动器显示器数值加10（需将P0-02设为0）
            self.logger.info(self.master.execute(self.servoAddress, cst.WRITE_MULTIPLE_REGISTERS, 0x0606,
                                                 output_value=pulse))
            print ('pulse',pulse)
        except Exception as e:
            print (e)
        try:
            # P5-07 050EH, 050FH PR命令触发寄存器，0-8，0：原点回归，1：1#路径，2：2#路径，3...
            self.logger.info(self.master.execute(self.servoAddress, cst.WRITE_MULTIPLE_REGISTERS, 0x050E,
                                                 output_value=cmd))
        except Exception as e:
            print (e)

    def stopOneStep(self):
        try:
            # P5-07 050EH, 050FH PR命令触发寄存器，0-8，0：原点回归，1：1#路径，2：2#路径，3...,1000为停止
            self.logger.info(self.master.execute(self.servoAddress, cst.WRITE_MULTIPLE_REGISTERS, 0x050E,
                                                 output_value=[0x03E8,0x0000]))
        except Exception as e:
            print (e)
    '''
    def read(self,readAddress,drivecmd):
        import _thread
        try:
            #_thread.start_new_thread(self._read(readAddress), (event,))
            read_thread=_thread.start_new_thread(self._read,(readAddress,drivecmd))
            read_thread.join()
        except:
            print ('Error, unable to start a new thread')
    '''
    #实时监控命令完成情况
    def read(self,readAddress,drivecmd):
        self.read_thread=threading.Thread(target=self._read,args=(readAddress,drivecmd))
        self.read_thread.start()
        #read_thread.join() #待新线程，即被调用进程（read_thread)执行完成后，再返回调用进程

    def _read(self,readAddress,drivecmd):

        try:
            # 读取 P5-07 050EH, 050FH的值，读取值为原命令，说明命令未完成；为原命令+10000，说明命令发送完成；为原命令+20000，说明电机定位完成
            read=self.master.execute(self.servoAddress, cst.READ_HOLDING_REGISTERS, readAddress, 1)
            self.logger.info(read)
        except modbus_tk.modbus.ModbusError as exc:
            print("%s- Code=%d", exc, exc.get_exception_code())
        while(read[0]!=drivecmd[0]+20000):
            try:
                # 读取 P5-07 050EH, 050FH的值，读取值为原命令，说明命令未完成；为原命令+10000，说明命令发送完成；为原命令+20000，说明电机定位完成
                read = self.master.execute(self.servoAddress, cst.READ_HOLDING_REGISTERS, readAddress, 1)
                self.logger.info(read)
            except modbus_tk.modbus.ModbusError as exc:
                print("%s- Code=%d", exc, exc.get_exception_code())
            if read[0]==drivecmd[0]+20000:
                self.Done=True
                break
        return True

    def wrtie(self,address,data):
        try:
            self.master.execute(self.servoAddress, cst.WRITE_MULTIPLE_REGISTERS, address,
                                                 output_value=[data,0x0000])
        except Exception as e:
            print (e)
    #周期性的监控伺服中某个变量的状态，如P0-01（0x0002），其中delay时间要小于P3-04（通信超时）
    def monitor(self,add):
        try:
            self.status = self.master.execute(self.servoAddress, cst.READ_HOLDING_REGISTERS, add, 1)
            self.logger.info(self.status)
        except modbus_tk.modbus.ModbusError as exc:
            print("%s- Code=%d", exc, exc.get_exception_code())

    def servoMonitor(self,add):
        s = sched.scheduler(time.time, time.sleep)
        s.enter(delay=1, priority=2, action=self.servoMonitor, argument=(add,)) #argument 指的是self._read()的参数，delay为周期
        self.monitor(add)
        t=threading.Thread(target=s.run)
        t.start()

    # except modbus_tk.modbus.ModbusError as exc:
    #     print(123)
    #     print("%s- Code=%d", exc, exc.get_exception_code())
    #     #logger.error("%s- Code=%d", exc, exc.get_exception_code()


if __name__ == "__main__":
    driver1=LXM(0x02) #伺服驱动地址0x02
    driver1.servoOn()

    driver1.servoMonitor(driver1.statusAddress)
    driver1.wrtie(0x0306,0x0001) #通信出错时，1停机,0不停机
    driver1.wrtie(0x0308,0x0003) #通信超时设置，0-20S，0为关闭



    time.sleep(1)
    driver1.servoOff()

    #driver1.servoJogStop()
    driver1_speed=0x00C8  #速度200
    driver1_cmd=[0x0001,0x0000]  #PR命令，1#为执行一号路径
    driver_address=0x050E #读取任务完成情况，P5-07（0x050E)
    driver1_pulse=[0xF000,0x0000] #脉冲数量
    print(driver1_pulse)
    driver1.oneStep(speed=0x00C8,cmd=driver1_cmd,pulse=driver1_pulse)






    #     driver1.servoJogP()
    #     print('>>>>>>>>>>>>>>>>>>>>>>>>')
    #     import time
    #     time.sleep(5)
    #     driver1.servoJogStop()
    #     print('------------------------')
    #     import time
    #     time.sleep(2)
    #     driver1.servoJogN()
    #     print('<<<<<<<<<<<<<<<<<<<<<<<<')
    #     import time
    #     time.sleep(5)
    #     driver1.servoJogStop()
    #     print('------------------------')
    #     import time
    #     time.sleep(2)