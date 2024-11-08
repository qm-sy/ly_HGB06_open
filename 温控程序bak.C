//  温制程序 	2020-6-9
//CPU STC15W4K32S4	   11.0592Mhz

			   

#include  "STC15W4Kxx.H"
#include <stdlib.h>
#include "modbus.h"
#include "intrins.H"  //_nop_()
#include <stdio.h> //sprintf 用此函数
#include  <bin.c>
#include "math.h"


#define uchar  unsigned char
#define uint   unsigned int
#define ulong  unsigned long



#define	 Kp 8  //比例系数
#define	 Ki  0.5  //积分系数
#define	 Kd  2  //微分系数
#define  PID_MAX  900
#define  PID_MIN  0


#define ADC_POWER   0x80            //ADC电源控制位
#define ADC_FLAG    0x10            //ADC完成标志
#define ADC_START   0x08            //ADC起始控制位
#define ADC_SPEEDLL 0x00            //540个时钟
#define ADC_SPEEDL  0x20            //360个时钟
#define ADC_SPEEDH  0x40            //180个时钟
#define ADC_SPEEDHH 0x60            //90个时钟

#define CMD_IDLE    0               //空闲模式
#define CMD_READ    1               //IAP字节读命令
#define CMD_PROGRAM 2               //IAP字节编程命令
#define CMD_ERASE   3               //IAP扇区擦除命令

//#define ENABLE_IAP 0x80           //if SYSCLK<30MHz
//#define ENABLE_IAP 0x81           //if SYSCLK<24MHz
//#define ENABLE_IAP  0x82            //if SYSCLK<20MHz
#define ENABLE_IAP 0x83           //if SYSCLK<12MHz
//#define ENABLE_IAP 0x84           //if SYSCLK<6MHz
//#define ENABLE_IAP 0x85           //if SYSCLK<3MHz
//#define ENABLE_IAP 0x86           //if SYSCLK<2MHz
//#define ENABLE_IAP 0x87           //if SYSCLK<1MHz


//ID号的存放在程序区的地址为程序空间的最后7字节
//#define ID_ADDR_ROM 0x03f9      //1K程序空间的MCU(如STC15F201EA, STC15F101EA)
//#define ID_ADDR_ROM 0x07f9      //2K程序空间的MCU(如STC15F402EACS, STC15F202EA, STC15F102EA)
//#define ID_ADDR_ROM 0x0bf9      //3K程序空间的MCU(如STC15F203EA, STC15F103EA)
//#define ID_ADDR_ROM 0x0ff9      //4K程序空间的MCU(如STC15F804EACS, STC15F404EACS, STC15F204EA, STC15F104EA)
//#define ID_ADDR_ROM 0x13f9      //5K程序空间的MCU(如STC15F205EA, STC15F105EA)
//#define ID_ADDR_ROM 0x1ff9      //8K程序空间的MCU(如STC15F2K08S2,  STC15F808EACS, STC15F408EACS)
//#define ID_ADDR_ROM 0x27f9      //10K程序空间的MCU(如STC15F410EACS)
//#define ID_ADDR_ROM 0x2ff9      //12K程序空间的MCU(如STC15F812EACS, STC15F412EACS)
//#define ID_ADDR_ROM 0x3ff9      //16K程序空间的MCU(如STC15F2K16S2, STC15F816EACS)
//#define ID_ADDR_ROM 0x4ff9      //20K程序空间的MCU(如STC15F2K20S2, STC15F820EACS)
//#define ID_ADDR_ROM 0x5ff9      //24K程序空间的MCU(如              STC15F824EACS)
//#define ID_ADDR_ROM 0x6ff9      //28K程序空间的MCU(如              STC15F828EACS)
#define ID_ADDR_ROM 0x7ff9      //32K程序空间的MCU(如STC15F2K32S2)
//#define ID_ADDR_ROM 0x9ff9      //40K程序空间的MCU(如STC15F2K40S2)
//#define ID_ADDR_ROM 0xbff9      //48K程序空间的MCU(如STC15F2K48S2)
//#define ID_ADDR_ROM 0xcff9      //52K程序空间的MCU(如STC15F2K52S2)
//#define ID_ADDR_ROM 0xdff9      //56K程序空间的MCU(如STC15F2K56S2)
//#define ID_ADDR_ROM 0xeff9      //60K程序空间的MCU(如STC15F2K60S2)


#define S1_S0 0x40              //P_SW1.6
#define S1_S1 0x80              //P_SW1.7

#define CCP_S0 0x10                 //P_SW1.4
#define CCP_S1 0x20                 //P_SW1.5


#define S2_S0 0x01              //P_SW2.0
#define S3_S0 0x02              //P_SW2.1
#define S4_S0 0x04              //P_SW2.2

#define S2RI  0x01              //S2CON.0
#define S2TI  0x02              //S2CON.1
#define S2RB8 0x04              //S2CON.2
#define S2TB8 0x08              //S2CON.3
#define S2REN 0x10 			    //S2CON.4

#define S3RI  0x01              //S3CON.0
#define S3TI  0x02              //S3CON.1
#define S3RB8 0x04              //S3CON.2
#define S3TB8 0x08              //S3CON.3

#define S4RI  0x01              //S4CON.0
#define S4TI  0x02              //S4CON.1
#define S4RB8 0x04              //S4CON.2
#define S4TB8 0x08              //S4CON.3

#define PWM2345_S  0x10

#define POLY        0x8005

#define  nop    _nop_()

#define CYCLE   4096     //定义PWM周期


  
#define time_max  1450  //输出最高电压值

 /*
#define   fd_key        3
#define   mode_key      1
#define   fn_key        2
#define   power_key     5
#define   up_key        4
#define   dn_key        0
#define   fn_up_key     12
#define   fn_dn_key     8
	   */

#define   fd_key        2
#define   mode_key      4
#define   fn_key        3
#define   power_key     0
#define   up_key        1
#define   dn_key        5
#define   fn_up_key     9
#define   fn_dn_key     13


//************************************************************
//485接收缓冲区
unsigned char xdata g_recv_buffer[64];
int xdata g_recv_buffer_index = 0;
int xdata g_need_times = 10;            //超时定时时间 10 x 5ms
int xdata g_current_need_times = 0;     //超时实际时间
volatile unsigned char idata g_need_process_datas = 0;   //需要处理数据标志

uchar  idata  recv_cnt;

//************************************************************
  
 int   idata last_error1;
 int   idata last_error2;
 int   idata last_error3;
 int   idata last_error4;
 int   idata last_error5;
 int   idata last_error6;

 float idata I_term1;
 float idata I_term2;
 float idata I_term3;
 float idata I_term4;
 float idata I_term5;
 float idata I_term6;

 int   idata  tm1_pwm;
 int   idata  tm2_pwm;
 int   idata  tm3_pwm;
 int   idata  tm4_pwm;
 int   idata  tm5_pwm;
 int   idata  tm6_pwm;

 int  idata  tmp_cnt1;
 int  idata  tmp_cnt2;
 int  idata  tmp_cnt3;
 int  idata  tmp_cnt4;
 int  idata  tmp_cnt5;
 int  idata  tmp_cnt6;

 bit   tm1_bit;
 bit   tm2_bit;
 bit   tm3_bit;
 bit   tm4_bit;
 bit   tm5_bit;
 bit   tm6_bit;

 bit   tm1_on_bit;
 bit   tm2_on_bit;
 bit   tm3_on_bit;
 bit   tm4_on_bit;
 bit   tm5_on_bit;
 bit   tm6_on_bit;

 bit   t1_work_bit;
 bit   t2_work_bit;
 bit   t3_work_bit;
 bit   t4_work_bit;

 bit hot_power_bit;
 bit mode_num_bit;

 uchar   hot_power;
 uchar   fan_power;
 uchar  idata   hot_in_data;

 uchar  idata  keybuf;



 uchar idata   key_buf;
	   
 uchar idata  t_cnt;

 uchar idata  fan_cnt;

 uchar idata  dis_temp_cnt;
 
 uchar idata  cnt3;
 uchar idata  cnt4;
 uchar idata  cnt5;

 uchar  idata  fan_close_cnt;

 uchar idata  ss_cnt;

 uchar idata  temp_num;

 uchar idata  temp_set_cnt;
 uchar idata  save_cnt;
 
 uint  idata  temp[7];

 uint  idata  pwm5;

 uint  idata  fan_pwm;
 uchar idata  fan_num;

 uchar idata  mode_num;
 uchar idata  fenduan_num;
 uchar idata  hot_num;

 int   idata  temp_set1;
 int   idata  temp_set2;
 int   idata  temp_set3;
 int   idata  temp_set4;
 int   idata  temp_set5;
 int   idata  temp_set6;
 
 uchar   xdata  tm_set1;
 uchar   xdata  tm_set2;
 uchar   xdata  tm_set3;
 uchar   xdata  tm_set4;
 uchar   xdata  tm_set5;
 uchar   xdata  tm_set6;

 int     idata  temp1;
 int     idata  temp2;
 int     idata  temp3;
 int     idata  temp4;
 int     idata  temp5;
 int     idata  temp6;

 uchar idata  tb_num;

 uchar idata  power_cnt;

 uchar  idata  mode_wr[10]; 	//存储每个模式中的10个参数
 


 uchar idata lcd_03h; //显示地址缓存
 uchar idata lcd_06h; //显示地址缓存
 uchar idata lcd_07h; //显示地址缓存
 uchar idata lcd_0ah; //显示地址缓存
 uchar idata lcd_0bh; //显示地址缓存
 uchar idata lcd_0eh; //显示地址缓存
 uchar idata lcd_0fh; //显示地址缓存
 
 uint  idata  tm1;
 uint  idata  tm2;
 uint  idata  tm3;
 uint  idata  tm4;
 uint  idata  tm5;
 uint  idata  tm6;

	 
 unsigned long idata PeriodCnt = 0; //PWM 周期计数值
 uchar  idata  HighRH = 0; //高电平重载值的高字节
 uchar  idata  HighRL = 0; //高电平重载值的低字节
 uchar  idata  LowRH = 0; //低电平重载值的高字节
 uchar  idata  LowRL = 0; //低电平重载值的低字节


 uchar   xdata   ID_old[7];	//芯片ID数据
 uchar   xdata   ID_new[7]; //加密ID后的数据 
 uchar   code    *cptr;

 unsigned char code Tab[]={
  0xF5,0x05,0xD3,0x97,0x27,0xB6,0xF6,0x15,
  0xF7,0xB7,0x77,0xE6,0xF0,0xC7,0xF2,0x72,	 //0~F 代码
  0x00, //不显示
  0x02,//显示-
  };
//----------------------------------------------------------------------------------
			 
sbit TM1722_DIO   = P3^7;
sbit TM1722_CLK   = P4^1;
sbit TM1722_STB   = P3^6;

 
sbit hot_in=P0^7;  
sbit hot_ok=P0^6;  
sbit hot_fan=P0^5;  
sbit lcd_bl=P2^0;  

sbit led1=P4^4;
sbit led2=P2^6;
sbit led3=P4^3;
sbit led4=P2^5;
sbit led5=P4^2;
sbit led6=P2^4;

sbit out2=P2^1;

sbit out3=P2^2;

sbit out1=P1^7;

sbit out4=P1^6;
sbit out5=P2^7;
sbit out6=P0^4;


sbit speak=P5^4;
//bit speak;

			
bit uart_ok;
bit sp_bit=1;
bit clk_bit;

bit zb_bit;

bit dis_temp_bit;
bit hot_HT_bit;

bit  dis_HT_bit;
bit  dis_ss_bit;

bit save_bit;
bit auto_bit;

bit key_find_bit;
bit temp_set_bit;
bit ss_bit;

bit power_bit;
bit fan_bit;
bit fan_bit2;
bit fan_bit3;

 

bit  PWMOUT;

bit hot_fan_bit;

bit fan_close_bit;
bit fan_close_ok;
bit tb_bit;
bit fan_rs_bit;



//****************************************************************************
//	MF52E 10K at 25, B = 3950, ADC = 12 bits

uint code temp_table[]={
282 	,//0
291 	,//1
300 	,//2
309 	,//3
318 	,//4
327 	,//5
337 	,//6
346 	,//7
356 	,//8
366 	,//9
375 	,//10
385 	,//11
395 	,//12
404 	,//13
414 	,//14
424 	,//15
434 	,//16
444 	,//17
454 	,//18
463 	,//19
473 	,//20
483 	,//21
493 	,//22
502 	,//23
512 	,//24
522 	,//25
531 	,//26
541 	,//27
550 	,//28
559 	,//29
568 	,//30
578 	,//31
587 	,//32
596 	,//33
604 	,//34
613 	,//35
622 	,//36
630 	,//37
639 	,//38
647 	,//39
655 	,//40
663 	,//41
671 	,//42
679 	,//43
687 	,//44
694 	,//45
702 	,//46
709 	,//47
716 	,//48
723 	,//49
730 	,//50
737 	,//51
743 	,//52
750 	,//53
756 	,//54
763 	,//55
769 	,//56
775 	,//57
781 	,//58
786 	,//59
792 	,//60
797 	,//61
803 	,//62
808 	,//63
813 	,//64
818 	,//65
823 	,//66
828 	,//67
833 	,//68
837 	,//69
842 	,//70
846 	,//71
850 	,//72
854 	,//73
858 	,//74
862 	,//75
866 	,//76
870 	,//77
874 	,//78
877 	,//79
881 	,//80
884 	,//81
888 	,//82
891 	,//83
894 	,//84
897 	,//85
900 	,//86
903 	,//87
906 	,//88
909 	,//89
911 	,//90
914 	,//91
917 	,//92
919 	,//93
921 	,//94
924 	,//95
926 	,//96
928 	,//97
931 	,//98
933 	,//99
935 	,//100
937 	,//101
939 	,//102
941 	,//103
943 	,//104
945 	,//105
946 	,//106
948 	,//107
950 	,//108
952 	,//109
953 	,//110
955 	,//111
956 	,//112
958 	,//113
959 	,//114
961 	,//115
962 	,//116
964 	,//117
965 	,//118
966 	,//119
967 	//120
};
	 
//_________________________________________________________________________

  
//*******************************************************************************


uchar ID_read_ok(void)
{
 uchar i;
 cptr = ID_ADDR_ROM;         //从程序区读取ID号
    for (i=0; i<7; i++)         //读7个字节
    {
       ID_old[i]= *cptr++;      
    }
  //ID数据 以下4种算法  +15  *5  /4  <<7 
  ID_old[0]=((ID_old[0]+15)*5/4)<<7;    
  ID_old[1]=((ID_old[1]+15)*5/4)<<7;
  ID_old[2]=((ID_old[2]+15)*5/4)<<7;
  ID_old[3]=((ID_old[3]+15)*5/4)<<7;
  ID_old[4]=((ID_old[4]+15)*5/4)<<7;
  ID_old[5]=((ID_old[5]+15)*5/4)<<7;
  ID_old[6]=((ID_old[6]+15)*5/4)<<7;

 cptr = 0x5000;         //从程序区读取加密ID号	地址为0x5000
    for (i=0; i<7; i++)         //读7个字节
    {
       ID_new[i]= *cptr++;     
    } 


 for (i=0; i<7; i++){
		              if(ID_old[i]!=ID_new[i])  return 1;  //比对数据不正确

                     }
 return 0;	 //比对数据正确

}


/********************  计算温度 ***********************************************/
// 计算结果: 0对应-40.0度, 400对应0度, 625对应25.0度, 最大1600对应120.0度. 
// 为了通用, ADC输入为12bit的ADC值.
// 电路和软件算法设计: Coody
/**********************************************/
		 
//#define		D_SCALE		10		//结果放大倍数, 放大10倍就是保留一位小数
uint get_temperature(uint adc)
{
	uint	code *p;
	uint	i;
	uchar	j,k,min,max;
	
	//adc = 4096 - adc;	//Rt接地
	p = temp_table;
	//if(adc < p[0])		return (0xfffe);
	//if(adc > p[160])	return (0xffff);
		
	min = 0;		//0度
	max = 120;		//120度

	for(j=0; j<5; j++)	//对分查表
	{
		k = min / 2 + max / 2;
		if(adc <= p[k])	max = k;
		else			min = k;
	}
		 if(adc == p[min])	i = min;
	else if(adc == p[max])	i = max;
	else	// min < temp < max
	{
		while(min <= max)
		{
			min++;
			if(adc == p[min])	{i = min;	break;}
			else if(adc < p[min])
			{
				min--;
				//i = p[min];	//min
				//j = (adc - i) * D_SCALE / (p[min+1] - i);
				i = min;
				//i *= D_SCALE;
				//i += j;
				break;
			}
		}
	}
	return i;
}
	 

//************************************************************************

void Delay1ms(unsigned int dd)		//@11.0592MHz
{
	unsigned char i, j;
  while(dd){
        	    _nop_();
	            _nop_();
	            _nop_();
	         i = 11;
	         j = 190;
	         do
	           {
	         	while (--j);
	           } while (--i);

		     WDT_CONTR=0X34;
			 dd--;
            }
}



//*****************************************************************************


void UartInit(void)		//9600bps@11.0592MHz
{
	SCON = 0x50;		//8位数据,可变波特率 串口1

	//S3CON = 0x10;		//8位数据,可变波特率
	//S3CON &= 0xBF;		//串口3选择定时器2为波特率发生器

	//S4CON = 0x10;		//8位数据,可变波特率
	//S4CON &= 0xBF;		//串口4选择定时器2为波特率发生器

    AUXR |= 0x01;		//串口1选择定时器2为波特率发生器
	AUXR |= 0x04;		//定时器2时钟为Fosc,即1T
	T2L = 0xE0;		//设定定时初值
	T2H = 0xFE;		//设定定时初值
	AUXR |= 0x10;		//启动定时器2
	RI=0;
	TI=0;
    ES=1;
	//S3CON &= ~S3RI;
	//S4CON &= ~S4RI;
	//IE2 = 0x08;	  //打开串口3中断
	
}
		  

void Timer0Init(void)		//1毫秒@11.0592MHz
{
	AUXR |= 0x80;		//定时器时钟1T模式
	TMOD &= 0xF0;		//设置定时器模式
	TL0 = 0xCD;		//设置定时初值
	TH0 = 0xD4;		//设置定时初值
	TF0 = 0;		//清除TF0标志
	TR0 = 1;		//定时器0开始计时
	ET0=1;
}




void Timer3Init(void)		//50毫秒@11.0592MHz
{
	T4T3M &= 0xFD;		//定时器时钟12T模式
	T3L = 0x00;		//设置定时初值
	T3H = 0x4C;		//设置定时初值
	T4T3M |= 0x08;		//定时器3开始计时
	IE2 |= 0x20;       //开定时器3中断
}






/*----------------------------------------------------------------------------
关闭IAP
----------------------------*/
void IapIdle()
{
    IAP_CONTR = 0;                  //关闭IAP功能
    IAP_CMD = 0;                    //清除命令寄存器
    IAP_TRIG = 0;                   //清除触发寄存器
    IAP_ADDRH = 0x80;               //将地址设置到非IAP区域
    IAP_ADDRL = 0;
}

/*----------------------------
从ISP/IAP/EEPROM区域读取一字节
----------------------------*/
uchar IapReadByte(uint addr)
{
    uchar dat;                       //数据缓冲区

    IAP_CONTR = ENABLE_IAP;         //使能IAP
    IAP_CMD = CMD_READ;             //设置IAP命令
    IAP_ADDRL = addr;               //设置IAP低地址
    IAP_ADDRH = addr >> 8;          //设置IAP高地址
    IAP_TRIG = 0x5a;                //写触发命令(0x5a)
    IAP_TRIG = 0xa5;                //写触发命令(0xa5)
    _nop_();                        //等待ISP/IAP/EEPROM操作完成
    dat = IAP_DATA;                 //读ISP/IAP/EEPROM数据
    IapIdle();                      //关闭IAP功能

    return dat;                     //返回
}

/*----------------------------
写一字节数据到ISP/IAP/EEPROM区域
----------------------------*/
void IapProgramByte(uint addr, uchar dat)
{
    IAP_CONTR = ENABLE_IAP;         //使能IAP
    IAP_CMD = CMD_PROGRAM;          //设置IAP命令
    IAP_ADDRL = addr;               //设置IAP低地址
    IAP_ADDRH = addr >> 8;          //设置IAP高地址
    IAP_DATA = dat;                 //写ISP/IAP/EEPROM数据
    IAP_TRIG = 0x5a;                //写触发命令(0x5a)
    IAP_TRIG = 0xa5;                //写触发命令(0xa5)
    _nop_();                        //等待ISP/IAP/EEPROM操作完成
    IapIdle();
}

/*----------------------------
扇区擦除
----------------------------*/
void IapEraseSector(uint addr)
{
    IAP_CONTR = ENABLE_IAP;         //使能IAP
    IAP_CMD = CMD_ERASE;            //设置IAP命令
    IAP_ADDRL = addr;               //设置IAP低地址
    IAP_ADDRH = addr >> 8;          //设置IAP高地址
    IAP_TRIG = 0x5a;                //写触发命令(0x5a)
    IAP_TRIG = 0xa5;                //写触发命令(0xa5)
    _nop_();                        //等待ISP/IAP/EEPROM操作完成
    IapIdle();
}

//******************************************************************
	
void save_mode_num(void)   //保存模式数值
{
 uint addr;
 
 addr=0x0000;
 EA=0;
 IapEraseSector(addr);    //扇区擦除

 IapProgramByte(addr,mode_num); 
 
               
 EA=1;
}


void fan_close_save(void)
{
 uint addr;
 
 addr=0x1000;
 EA=0;
 IapEraseSector(addr);    //扇区擦除

 IapProgramByte(addr,fan_close_cnt); 
 
               
 EA=1;

}





void hot_power_save(void)  //存储加热开关
{
 uint addr;
 
 addr=0x1200;
 EA=0;
 IapEraseSector(addr);    //扇区擦除

 IapProgramByte(addr,hot_power); 
 
               
 EA=1;

}


void fan_power_save(void)  //存储风扇开关
{
 uint addr;
 
 addr=0x1400;
 EA=0;
 IapEraseSector(addr);    //扇区擦除

 IapProgramByte(addr,fan_power); 
 
               
 EA=1;

}


void fan_close_read(void) 
{
 uint addr;
 
 addr=0x1000;
 EA=0;
 fan_close_cnt=IapReadByte(addr);
 EA=1;

 if((fan_close_cnt<10)||(fan_close_cnt>250))	  fan_close_cnt=120;
}


void hot_power_read(void) 
{
 uint addr;
 
 addr=0x1200;
 EA=0;
 hot_power=IapReadByte(addr);
 EA=1;

 if(hot_power>1)    hot_power=0;
								
}



void fan_power_read(void) 
{
 uint addr;
 
 addr=0x1400;
 EA=0;
 fan_power=IapReadByte(addr);
 EA=1;

 if(fan_power>1)    fan_power=0;
								
}

//**********************************************************
void  read_mode_wr_data(void) //读取每个模式中的参数
{
 fenduan_num=mode_wr[0]; //分段
 tb_num=mode_wr[1];	    //同步
 fan_num=mode_wr[2];	//风力
 hot_num=mode_wr[3];	//同步

 tm_set1=mode_wr[4];	//温度1
 tm_set2=mode_wr[5];	//温度2
 tm_set3=mode_wr[6];	//温度3
 tm_set4=mode_wr[7];	//温度4
 tm_set5=mode_wr[8];	//温度5
 tm_set6=mode_wr[9];	//温度6

 if((fenduan_num==0)||(fenduan_num>7))  fenduan_num=1;
 if(tb_num>1)       tb_num=0; //同步只有0和1，0没有同步，1为同步
 if(fan_num>6)      fan_num=0;
 if(hot_num>100)    hot_num=50;

 if(tm_set1>200)  	tm_set1=40;
 if(tm_set2>200)  	tm_set2=40;
 if(tm_set3>200)  	tm_set3=40;
 if(tm_set4>200)  	tm_set4=40;
 if(tm_set5>200)  	tm_set5=40;
 if(tm_set6>200)  	tm_set6=40;

}


void  save_mode_wr(void)
{
 uchar n;
 uint addr;

 mode_wr[0]=fenduan_num; //分段
 mode_wr[1]=tb_num;	     //同步
 mode_wr[2]=fan_num;	//风力
 mode_wr[3]=hot_num;	//同步

 mode_wr[4]=tm_set1;	//温度1
 mode_wr[5]=tm_set2;	//温度2
 mode_wr[6]=tm_set3;	//温度3
 mode_wr[7]=tm_set4;	//温度4
 mode_wr[8]=tm_set5;	//温度5
 mode_wr[9]=tm_set6;	//温度6

 if(mode_num==1) 
   {
    addr=0x0200;
	EA=0;
	IapEraseSector(addr);    //扇区擦除
    for(n=0;n<10;n++) {IapProgramByte(addr,mode_wr[n]);	addr++;}
	EA=1;
   }

 else if(mode_num==2) 
   {
    addr=0x0400;
	EA=0;
	IapEraseSector(addr);    //扇区擦除
    for(n=0;n<10;n++) {IapProgramByte(addr,mode_wr[n]);	addr++;}
	EA=1;
   }

 else if(mode_num==3) 
   {
    addr=0x0600;
	EA=0;
	IapEraseSector(addr);    //扇区擦除
    for(n=0;n<10;n++) {IapProgramByte(addr,mode_wr[n]);	addr++;}
	EA=1;
   }

 else if(mode_num==4) 
   {
    addr=0x0800;
	EA=0;
	IapEraseSector(addr);    //扇区擦除
    for(n=0;n<10;n++) {IapProgramByte(addr,mode_wr[n]);	addr++;}
	EA=1;
   }

 else if(mode_num==5) 
   {
    addr=0x0a00;
	EA=0;
	IapEraseSector(addr);    //扇区擦除
    for(n=0;n<10;n++) {IapProgramByte(addr,mode_wr[n]);	addr++;}
	EA=1;
   }
}


void read_mode(void)
{
 uint addr;
 EA=0;
 addr=0x0000;
 mode_num=IapReadByte(addr);
 
 EA=1;


 }

  
void read_data(void)
{
 uint addr;
 uchar n;

 if((mode_num==0)||(mode_num>3)) {mode_num=1;}

 if(mode_num==1) 
   {
   	EA=0;
	addr=0x0200;
    for(n=0;n<10;n++) {mode_wr[n]=IapReadByte(addr);	addr++;}
   	EA=1;
   }

 else if(mode_num==2) 
   {
   	EA=0;
	addr=0x0400;
    for(n=0;n<10;n++) {mode_wr[n]=IapReadByte(addr);	addr++;}
   	EA=1;
   }

 else if(mode_num==3) 
   {
   	EA=0;
	addr=0x0600;
    for(n=0;n<10;n++) {mode_wr[n]=IapReadByte(addr);	addr++;}
   	EA=1;
   }

 else if(mode_num==4) 
   {
   	EA=0;
	addr=0x0800;
    for(n=0;n<10;n++) {mode_wr[n]=IapReadByte(addr);	addr++;}
   	EA=1;
   }

 else if(mode_num==5) 
   {
   	EA=0;
	addr=0x0a00;
    for(n=0;n<10;n++) {mode_wr[n]=IapReadByte(addr);	addr++;}
   	EA=1;
   }

 read_mode_wr_data();  //加载模式对应的参数
 



}

	



//__________________________________________________________________________


//***********************************************************************

void InitADC(void)
{
    P1ASF = B0011_1111;                   //设置P1.0~P1.5口为AD口
    ADC_RES = 0;                    //清除结果寄存器
    ADC_RESL=0;
    ADC_CONTR = ADC_POWER | ADC_SPEEDL;
   
}


//----------------------------
//读取ADC结果
//----------------------------
uint GetADCResult(uchar ch)      //ch为输入模拟量通道号0~7
{
 uint dd=0;

    ADC_CONTR = ADC_POWER | ADC_SPEEDL | ch | ADC_START;
    _nop_();                        //等待4个NOP
    _nop_();
    _nop_();
    _nop_();
    while (!(ADC_CONTR & ADC_FLAG));//等待ADC转换完成
    ADC_CONTR &= ~ADC_FLAG;         //Close ADC

    dd= ADC_RES;                 //返回ADC结果
    dd=dd<<2;
    dd=dd|ADC_RESL;
	return dd;
}
 


uint Temperature_LPF(uchar ch)
{
 uint a;
 uchar i,j,k;
 for(i=0;i<=6;i++)
  {
    temp[i]= GetADCResult(ch); //采集7个数据.
    Delay1ms(2);
  }
 for(j=0;j<=6;j++)
  {                      //按从小到大排序
    for(k=j;k<=6;k++)
	 {
       if(temp[j]>=temp[k])
	     {
           a=temp[j];
           temp[j]=temp[k];
           temp[k]=a;
          }
      }
   }
 temp[0]=temp[6]=0;
//去掉最大最小值
return (temp[1]+temp[2]+temp[3]+temp[4]+temp[5])/5; //求平均值

}



/*

float Get_Tempture(uint adc)
{
        float RV,RT,Tmp;
        RV=BaseVol/1024.0*(float)adc;//ADC为10位ADC,求出NTC电压:RV=ADCValu/1024*BaseVoltag
        RT=RV*10/RV;//(BaseVol-RV);//求出当前温度阻值 (BaseVoltage-RV)/R16=RV/RT;
        Tmp=1/(1/TN+(log(RT/RN)/B))-273.15;//%RT = RN exp*B(1/T-1/TN)%
         return Tmp;
}
  */





void send_buffer(unsigned char *buf,int len)
{	
	while (len--) {
		TI=0;     //Clear transmit interrupt flag
		SBUF = *buf++;
		while (TI == 0);
		TI=0;
	}
}
 



//*********************************************************************


void Delay50us(void)		//@11.0592MHz
{
	unsigned char i, j;
   
	_nop_();
	i = 1;
	j = 134;
	do
	{
		while (--j);
	} while (--i);
 
}
		   


                                                  
              

/******************************************
函数：写一字节函数
参数：dat---写入的一字节数据
返回值：无
******************************************/
void TM1722_Write_Byte(unsigned char dat)
{
  unsigned char i;
         
  Delay50us(); //用于片选信号的延时
  TM1722_STB=0;     //有效的片选信号
  for(i=0;i<8;i++)
  {
   TM1722_CLK=0;
   TM1722_DIO=dat&0x01;
   TM1722_CLK=1;    //时钟上升沿，送入一位数据
   dat>>=1;      
  }
  Delay50us();   //用于片选信号的延时
}






/******************************************
函数：写一位数码管函数
参数：num_addr---数码管位，num---显示数字数据
返回值：无
******************************************/
void TM1722_Write_Word(uchar num_addr,uchar num)
{
  uchar temp1,temp2;
  uchar addr1;
  uchar addr2;

  if(num_addr==1)       {addr1=0xcb;  addr2=0xca;  lcd_0bh=lcd_0bh&B1111_0000; lcd_0bh=temp1=lcd_0bh|(Tab[num]&0xf0)>>4;  lcd_0ah=lcd_0ah&B1111_0000; lcd_0ah=temp2=lcd_0ah|(Tab[num]&0x0f);}
  else if(num_addr==2)  {addr1=0xc7;  addr2=0xc7;  lcd_07h=lcd_07h&B0000_1000; lcd_07h=temp2=lcd_07h|Tab[num];}
  else if(num_addr==3)  {addr1=0xc6;  addr2=0xc6;  lcd_06h=lcd_06h&B0000_1000; lcd_06h=temp2=lcd_06h|Tab[num];}
 


  TM1722_STB=1;            //端口配置初始化
  TM1722_CLK=1;
  TM1722_DIO=1;
  TM1722_Write_Byte(0x00); //工作模式
  TM1722_STB=1;
  TM1722_Write_Byte(0x44);   //固定地址模式
  TM1722_STB=1;

  if(num_addr!=1) goto  next_num1;
   
  TM1722_Write_Byte(addr1);//显示寄存器的00H单元开始
  TM1722_Write_Byte(temp1);     //给显示寄存器送数据，
  TM1722_STB=1;	 

  next_num1: nop;

  TM1722_Write_Byte(addr2);//显示寄存器的00H单元开始
  TM1722_Write_Byte(temp2);     //给显示寄存器送数据，
  TM1722_STB=1;

  TM1722_Write_Byte(0x93); //显示开
  TM1722_STB=1;
  
}   
 

void fan_dis(uchar num)   //风量条显示
{
  if(num==0)       {lcd_0fh=lcd_0fh&B0011_0000;  lcd_0fh=lcd_0fh|B0000_0000; }
  else if(num==1)  {lcd_0fh=lcd_0fh&B0011_0000;  lcd_0fh=lcd_0fh|B0100_0000; }
  else if(num==2)  {lcd_0fh=lcd_0fh&B0011_0000;  lcd_0fh=lcd_0fh|B1100_0000; }
  else if(num==3)  {lcd_0fh=lcd_0fh&B0011_0000;  lcd_0fh=lcd_0fh|B1100_1000; }
  else if(num==4)  {lcd_0fh=lcd_0fh&B0011_0000;  lcd_0fh=lcd_0fh|B1100_1100; }
  else if(num==5)  {lcd_0fh=lcd_0fh&B0011_0000;  lcd_0fh=lcd_0fh|B1100_1110; }
  else if(num==6)  {lcd_0fh=lcd_0fh&B0011_0000;  lcd_0fh=lcd_0fh|B1100_1111; }

  TM1722_STB=1;            //端口配置初始化
  TM1722_CLK=1;
  TM1722_DIO=1;
  TM1722_Write_Byte(0x00); //工作模式
  TM1722_STB=1;
  TM1722_Write_Byte(0x44);   //固定地址模式
  TM1722_STB=1;
 

  TM1722_Write_Byte(0xcf);         //显示寄存器的00H单元开始
  TM1722_Write_Byte(lcd_0fh);     //给显示寄存器送数据，
  TM1722_STB=1;

  TM1722_Write_Byte(0x93); //显示开
  TM1722_STB=1;
  
}   









void lcd_clear(uchar dat)  //清显示缓存
{

  lcd_03h=dat;
  lcd_06h=dat;
  lcd_07h=dat;
  lcd_0ah=dat;
  lcd_0bh=dat;
  lcd_0eh=dat;
  lcd_0fh=dat;

  TM1722_STB=1;            //端口配置初始化
  TM1722_CLK=1;
  TM1722_DIO=1;
  TM1722_Write_Byte(0x00); //工作模式
  TM1722_STB=1;
  TM1722_Write_Byte(0x44);   //固定地址模式
  TM1722_STB=1;

  							
  TM1722_Write_Byte(0xc3);         //显示寄存器的00H单元开始
  TM1722_Write_Byte(lcd_03h);     //给显示寄存器送数据，
  TM1722_STB=1;

  TM1722_Write_Byte(0xc6);         //显示寄存器的00H单元开始
  TM1722_Write_Byte(lcd_06h);     //给显示寄存器送数据，
  TM1722_STB=1;

  TM1722_Write_Byte(0xc7);         //显示寄存器的00H单元开始
  TM1722_Write_Byte(lcd_07h);     //给显示寄存器送数据，
  TM1722_STB=1;

  TM1722_Write_Byte(0xca);         //显示寄存器的00H单元开始
  TM1722_Write_Byte(lcd_0ah);     //给显示寄存器送数据，
  TM1722_STB=1;


  TM1722_Write_Byte(0xcb);         //显示寄存器的00H单元开始
  TM1722_Write_Byte(lcd_0bh);     //给显示寄存器送数据，
  TM1722_STB=1;

  TM1722_Write_Byte(0xce);         //显示寄存器的00H单元开始
  TM1722_Write_Byte(lcd_0eh);     //给显示寄存器送数据，
  TM1722_STB=1;

  TM1722_Write_Byte(0xcf);         //显示寄存器的00H单元开始
  TM1722_Write_Byte(lcd_0fh);     //给显示寄存器送数据，
  TM1722_STB=1;

  TM1722_Write_Byte(0x93); //显示开
  TM1722_STB=1;

}



void  dis1(bit sss,uint ss)  //sss=0,不显示   sss=1 显示出来  
{
 uchar a1,a2,a3;
 if(sss)
   {
    a1=ss%1000/100;
    a2=ss%100/10;
    a3=ss%10;

 	 if(a1==0) TM1722_Write_Word(1,16);
     else      TM1722_Write_Word(1,a1);

	 if((a1==0)&&(a2==0)) TM1722_Write_Word(2,16);
     else                 TM1722_Write_Word(2,a2); 
	  
     TM1722_Write_Word(3,a3); 
    
   }
 else
  {
     TM1722_Write_Word(1,16);  
     TM1722_Write_Word(2,16);  
     TM1722_Write_Word(3,16); 
     
   }

}





void temp_dis(bit on_off) //1--显示，0--不显示	 点亮 S10 
{
  if(on_off) {lcd_03h=lcd_03h|B0100_0000; 	}
  else 		 {lcd_03h=lcd_03h&B1011_1111; 	}

  TM1722_STB=1;            //端口配置初始化
  TM1722_CLK=1;
  TM1722_DIO=1;
  TM1722_Write_Byte(0x00); //工作模式
  TM1722_STB=1;
  TM1722_Write_Byte(0x44);   //固定地址模式
  TM1722_STB=1;

  TM1722_Write_Byte(0xc3);         //显示寄存器的00H单元开始
  TM1722_Write_Byte(lcd_03h);     //给显示寄存器送数据，
  TM1722_STB=1;


  TM1722_Write_Byte(0x93); //显示开
  TM1722_STB=1;
}



void hot_dis(bit on_off) //1--显示，0--不显示	 点亮S1
{
  if(on_off) {lcd_0eh=lcd_0eh|B0000_1000; 	}
  else 		 {lcd_0eh=lcd_0eh&B1111_0111; 	}

  TM1722_STB=1;            //端口配置初始化
  TM1722_CLK=1;
  TM1722_DIO=1;
  TM1722_Write_Byte(0x00); //工作模式
  TM1722_STB=1;
  TM1722_Write_Byte(0x44);   //固定地址模式
  TM1722_STB=1;

  TM1722_Write_Byte(0xce);         //显示寄存器的00H单元开始
  TM1722_Write_Byte(lcd_0eh);     //给显示寄存器送数据，
  TM1722_STB=1;


  TM1722_Write_Byte(0x93); //显示开
  TM1722_STB=1;
}



void hot1_dis(bit on_off) //1--显示，0--不显示	 点亮S2
{
  if(on_off) {lcd_0eh=lcd_0eh|B0000_0100; 	}
  else 		 {lcd_0eh=lcd_0eh&B1111_1011; 	}

  TM1722_STB=1;            //端口配置初始化
  TM1722_CLK=1;
  TM1722_DIO=1;
  TM1722_Write_Byte(0x00); //工作模式
  TM1722_STB=1;
  TM1722_Write_Byte(0x44);   //固定地址模式
  TM1722_STB=1;

  TM1722_Write_Byte(0xce);         //显示寄存器的00H单元开始
  TM1722_Write_Byte(lcd_0eh);     //给显示寄存器送数据，
  TM1722_STB=1;


  TM1722_Write_Byte(0x93); //显示开
  TM1722_STB=1;
}





void hot2_dis(bit on_off) //1--显示，0--不显示	 点亮S3
{
  if(on_off) {lcd_0eh=lcd_0eh|B0000_0010; 	}
  else 		 {lcd_0eh=lcd_0eh&B1111_1101; 	}

  TM1722_STB=1;            //端口配置初始化
  TM1722_CLK=1;
  TM1722_DIO=1;
  TM1722_Write_Byte(0x00); //工作模式
  TM1722_STB=1;
  TM1722_Write_Byte(0x44);   //固定地址模式
  TM1722_STB=1;

  TM1722_Write_Byte(0xce);         //显示寄存器的00H单元开始
  TM1722_Write_Byte(lcd_0eh);     //给显示寄存器送数据，
  TM1722_STB=1;


  TM1722_Write_Byte(0x93); //显示开
  TM1722_STB=1;
}
   



void hot3_dis(bit on_off) //1--显示，0--不显示	 点亮S4
{
  if(on_off) {lcd_0eh=lcd_0eh|B0000_0001; 	}
  else 		 {lcd_0eh=lcd_0eh&B1111_1110; 	}

  TM1722_STB=1;            //端口配置初始化
  TM1722_CLK=1;
  TM1722_DIO=1;
  TM1722_Write_Byte(0x00); //工作模式
  TM1722_STB=1;
  TM1722_Write_Byte(0x44);   //固定地址模式
  TM1722_STB=1;

  TM1722_Write_Byte(0xce);         //显示寄存器的00H单元开始
  TM1722_Write_Byte(lcd_0eh);     //给显示寄存器送数据，
  TM1722_STB=1;


  TM1722_Write_Byte(0x93); //显示开
  TM1722_STB=1;
}





void H1_dis(bit on_off) //1--显示，0--不显示	 点亮 数字1  H1
{
  if(on_off) {lcd_0bh=lcd_0bh|B0100_0000; 	}
  else 		 {lcd_0bh=lcd_0bh&B1011_1111; 	}

  TM1722_STB=1;            //端口配置初始化
  TM1722_CLK=1;
  TM1722_DIO=1;
  TM1722_Write_Byte(0x00); //工作模式
  TM1722_STB=1;
  TM1722_Write_Byte(0x44);   //固定地址模式
  TM1722_STB=1;

  TM1722_Write_Byte(0xcb);         //显示寄存器的00H单元开始
  TM1722_Write_Byte(lcd_0bh);     //给显示寄存器送数据，
  TM1722_STB=1;


  TM1722_Write_Byte(0x93); //显示开
  TM1722_STB=1;
}


void H2_dis(bit on_off) //1--显示，0--不显示	 点亮 数字2  H2
{
  if(on_off) {lcd_0bh=lcd_0bh|B0010_0000; 	}
  else 		 {lcd_0bh=lcd_0bh&B1101_1111; 	}

  TM1722_STB=1;            //端口配置初始化
  TM1722_CLK=1;
  TM1722_DIO=1;
  TM1722_Write_Byte(0x00); //工作模式
  TM1722_STB=1;
  TM1722_Write_Byte(0x44);   //固定地址模式
  TM1722_STB=1;

  TM1722_Write_Byte(0xcb);         //显示寄存器的00H单元开始
  TM1722_Write_Byte(lcd_0bh);     //给显示寄存器送数据，
  TM1722_STB=1;


  TM1722_Write_Byte(0x93); //显示开
  TM1722_STB=1;
}





void H3_dis(bit on_off) //1--显示，0--不显示	 点亮 数字3  H3
{
  if(on_off) {lcd_0bh=lcd_0bh|B0001_0000; 	}
  else 		 {lcd_0bh=lcd_0bh&B1110_1111; 	}

  TM1722_STB=1;            //端口配置初始化
  TM1722_CLK=1;
  TM1722_DIO=1;
  TM1722_Write_Byte(0x00); //工作模式
  TM1722_STB=1;
  TM1722_Write_Byte(0x44);   //固定地址模式
  TM1722_STB=1;

  TM1722_Write_Byte(0xcb);         //显示寄存器的00H单元开始
  TM1722_Write_Byte(lcd_0bh);     //给显示寄存器送数据，
  TM1722_STB=1;


  TM1722_Write_Byte(0x93); //显示开
  TM1722_STB=1;
}



void tb_dis(bit on_off) //1--显示，0--不显示	 点亮 S5  同步标志
{
  if(on_off) {lcd_03h=lcd_03h|B0001_0000; 	}
  else 		 {lcd_03h=lcd_03h&B1110_1111; 	}

  TM1722_STB=1;            //端口配置初始化
  TM1722_CLK=1;
  TM1722_DIO=1;
  TM1722_Write_Byte(0x00); //工作模式
  TM1722_STB=1;
  TM1722_Write_Byte(0x44);   //固定地址模式
  TM1722_STB=1;

  TM1722_Write_Byte(0xc3);         //显示寄存器的00H单元开始
  TM1722_Write_Byte(lcd_03h);     //给显示寄存器送数据，
  TM1722_STB=1;


  TM1722_Write_Byte(0x93); //显示开
  TM1722_STB=1;
}







void jg_dis(bit on_off) //1--显示，0--不显示	 点亮 S6  警告标志
{
  if(on_off) {lcd_03h=lcd_03h|B0010_0000; 	}
  else 		 {lcd_03h=lcd_03h&B1101_1111; 	}

  TM1722_STB=1;            //端口配置初始化
  TM1722_CLK=1;
  TM1722_DIO=1;
  TM1722_Write_Byte(0x00); //工作模式
  TM1722_STB=1;
  TM1722_Write_Byte(0x44);   //固定地址模式
  TM1722_STB=1;

  TM1722_Write_Byte(0xc3);         //显示寄存器的00H单元开始
  TM1722_Write_Byte(lcd_03h);     //给显示寄存器送数据，
  TM1722_STB=1;


  TM1722_Write_Byte(0x93); //显示开
  TM1722_STB=1;
}






void mode_dis(bit on_off) //1--显示，0--不显示	 点亮 S11  Mode
{
  if(on_off) {lcd_03h=lcd_03h|B1000_0000; 	}
  else 		 {lcd_03h=lcd_03h&B0111_1111; 	}

  TM1722_STB=1;            //端口配置初始化
  TM1722_CLK=1;
  TM1722_DIO=1;
  TM1722_Write_Byte(0x00); //工作模式
  TM1722_STB=1;
  TM1722_Write_Byte(0x44);   //固定地址模式
  TM1722_STB=1;

  TM1722_Write_Byte(0xc3);         //显示寄存器的00H单元开始
  TM1722_Write_Byte(lcd_03h);     //给显示寄存器送数据，
  TM1722_STB=1;


  TM1722_Write_Byte(0x93); //显示开
  TM1722_STB=1;
}








void bf_dis(bit on_off) //1--显示，0--不显示	 点亮 S12  % 标志
{
  if(on_off) {lcd_06h=lcd_06h|B0000_1000; 	}
  else 		 {lcd_06h=lcd_06h&B1111_0111; 	}

  TM1722_STB=1;            //端口配置初始化
  TM1722_CLK=1;
  TM1722_DIO=1;
  TM1722_Write_Byte(0x00); //工作模式
  TM1722_STB=1;
  TM1722_Write_Byte(0x44);   //固定地址模式
  TM1722_STB=1;

  TM1722_Write_Byte(0xc6);         //显示寄存器的00H单元开始
  TM1722_Write_Byte(lcd_06h);     //给显示寄存器送数据，
  TM1722_STB=1;


  TM1722_Write_Byte(0x93); //显示开
  TM1722_STB=1;
}





void set_dis(bit on_off) //1--显示，0--不显示	 点亮 H4  Set 标志
{
  if(on_off) {lcd_03h=lcd_03h|B0000_0100; 	}
  else 		 {lcd_03h=lcd_03h&B1111_1011; 	}

  TM1722_STB=1;            //端口配置初始化
  TM1722_CLK=1;
  TM1722_DIO=1;
  TM1722_Write_Byte(0x00); //工作模式
  TM1722_STB=1;
  TM1722_Write_Byte(0x44);   //固定地址模式
  TM1722_STB=1;

  TM1722_Write_Byte(0xc3);         //显示寄存器的00H单元开始
  TM1722_Write_Byte(lcd_03h);     //给显示寄存器送数据，
  TM1722_STB=1;


  TM1722_Write_Byte(0x93); //显示开
  TM1722_STB=1;
}



void cur_dis(bit on_off) //1--显示，0--不显示	 点亮 H5  Cur 标志
{
  if(on_off) {lcd_03h=lcd_03h|B0000_1000; 	}
  else 		 {lcd_03h=lcd_03h&B1111_0111; 	}

  TM1722_STB=1;            //端口配置初始化
  TM1722_CLK=1;
  TM1722_DIO=1;
  TM1722_Write_Byte(0x00); //工作模式
  TM1722_STB=1;
  TM1722_Write_Byte(0x44);   //固定地址模式
  TM1722_STB=1;

  TM1722_Write_Byte(0xc3);         //显示寄存器的00H单元开始
  TM1722_Write_Byte(lcd_03h);     //给显示寄存器送数据，
  TM1722_STB=1;


  TM1722_Write_Byte(0x93); //显示开
  TM1722_STB=1;
}




void f_dis(bit on_off) //1--显示，0--不显示	 点亮S7
{
  if(on_off) {lcd_0eh=lcd_0eh|B1000_0000; 	}
  else 		 {lcd_0eh=lcd_0eh&B0111_1111; 	}

  TM1722_STB=1;            //端口配置初始化
  TM1722_CLK=1;
  TM1722_DIO=1;
  TM1722_Write_Byte(0x00); //工作模式
  TM1722_STB=1;
  TM1722_Write_Byte(0x44);   //固定地址模式
  TM1722_STB=1;

  TM1722_Write_Byte(0xce);         //显示寄存器的00H单元开始
  TM1722_Write_Byte(lcd_0eh);     //给显示寄存器送数据，
  TM1722_STB=1;


  TM1722_Write_Byte(0x93); //显示开
  TM1722_STB=1;
}





void m_dis(bit on_off) //1--显示，0--不显示	 点亮S8
{
  if(on_off) {lcd_0eh=lcd_0eh|B0100_0000; 	}
  else 		 {lcd_0eh=lcd_0eh&B1011_1111; 	}

  TM1722_STB=1;            //端口配置初始化
  TM1722_CLK=1;
  TM1722_DIO=1;
  TM1722_Write_Byte(0x00); //工作模式
  TM1722_STB=1;
  TM1722_Write_Byte(0x44);   //固定地址模式
  TM1722_STB=1;

  TM1722_Write_Byte(0xce);         //显示寄存器的00H单元开始
  TM1722_Write_Byte(lcd_0eh);     //给显示寄存器送数据，
  TM1722_STB=1;


  TM1722_Write_Byte(0x93); //显示开
  TM1722_STB=1;
}




void r_dis(bit on_off) //1--显示，0--不显示	 点亮S9
{
  if(on_off) {lcd_0eh=lcd_0eh|B0010_0000; 	}
  else 		 {lcd_0eh=lcd_0eh&B1101_1111; 	}

  TM1722_STB=1;            //端口配置初始化
  TM1722_CLK=1;
  TM1722_DIO=1;
  TM1722_Write_Byte(0x00); //工作模式
  TM1722_STB=1;
  TM1722_Write_Byte(0x44);   //固定地址模式
  TM1722_STB=1;

  TM1722_Write_Byte(0xce);         //显示寄存器的00H单元开始
  TM1722_Write_Byte(lcd_0eh);     //给显示寄存器送数据，
  TM1722_STB=1;


  TM1722_Write_Byte(0x93); //显示开
  TM1722_STB=1;
}




void p1_dis(bit on_off) //1--显示，0--不显示	 点亮P1
{
  if(on_off) {lcd_0ah=lcd_0ah|B0000_1000; 	}
  else 		 {lcd_0ah=lcd_0ah&B1111_0111; 	}

  TM1722_STB=1;            //端口配置初始化
  TM1722_CLK=1;
  TM1722_DIO=1;
  TM1722_Write_Byte(0x00); //工作模式
  TM1722_STB=1;
  TM1722_Write_Byte(0x44);   //固定地址模式
  TM1722_STB=1;

  TM1722_Write_Byte(0xca);         //显示寄存器的00H单元开始
  TM1722_Write_Byte(lcd_0ah);     //给显示寄存器送数据，
  TM1722_STB=1;


  TM1722_Write_Byte(0x93); //显示开
  TM1722_STB=1;
}




void p2_dis(bit on_off) //1--显示，0--不显示	 点亮 P2
{
  if(on_off) {lcd_07h=lcd_07h|B0000_1000; 	}
  else 		 {lcd_07h=lcd_07h&B1111_0111; 	}

  TM1722_STB=1;            //端口配置初始化
  TM1722_CLK=1;
  TM1722_DIO=1;
  TM1722_Write_Byte(0x00); //工作模式
  TM1722_STB=1;
  TM1722_Write_Byte(0x44);   //固定地址模式
  TM1722_STB=1;

  TM1722_Write_Byte(0xc7);         //显示寄存器的00H单元开始
  TM1722_Write_Byte(lcd_07h);     //给显示寄存器送数据，
  TM1722_STB=1;


  TM1722_Write_Byte(0x93); //显示开
  TM1722_STB=1;
}




void s13_dis(bit on_off) //1--显示，0--不显示	 点亮S13
{
  if(on_off) {lcd_0eh=lcd_0eh|B0001_0000; 	}
  else 		 {lcd_0eh=lcd_0eh&B1110_1111; 	}

  TM1722_STB=1;            //端口配置初始化
  TM1722_CLK=1;
  TM1722_DIO=1;
  TM1722_Write_Byte(0x00); //工作模式
  TM1722_STB=1;
  TM1722_Write_Byte(0x44);   //固定地址模式
  TM1722_STB=1;

  TM1722_Write_Byte(0xce);         //显示寄存器的00H单元开始
  TM1722_Write_Byte(lcd_0eh);     //给显示寄存器送数据，
  TM1722_STB=1;


  TM1722_Write_Byte(0x93); //显示开
  TM1722_STB=1;
}




void s14_dis(bit on_off) //1--显示，0--不显示	 点亮S14  旋转叶
{
  if(on_off) {lcd_0fh=lcd_0fh|B0001_0000; 	}
  else 		 {lcd_0fh=lcd_0fh&B1110_1111; 	}

  TM1722_STB=1;            //端口配置初始化
  TM1722_CLK=1;
  TM1722_DIO=1;
  TM1722_Write_Byte(0x00); //工作模式
  TM1722_STB=1;
  TM1722_Write_Byte(0x44);   //固定地址模式
  TM1722_STB=1;

  TM1722_Write_Byte(0xcf);         //显示寄存器的00H单元开始
  TM1722_Write_Byte(lcd_0fh);     //给显示寄存器送数据，
  TM1722_STB=1;


  TM1722_Write_Byte(0x93); //显示开
  TM1722_STB=1;
}


void s15_dis(bit on_off) //1--显示，0--不显示	 点亮S14  旋转叶中心点
{
  if(on_off) {lcd_0fh=lcd_0fh|B0010_0000; 	}
  else 		 {lcd_0fh=lcd_0fh&B1101_1111; 	}

  TM1722_STB=1;            //端口配置初始化
  TM1722_CLK=1;
  TM1722_DIO=1;
  TM1722_Write_Byte(0x00); //工作模式
  TM1722_STB=1;
  TM1722_Write_Byte(0x44);   //固定地址模式
  TM1722_STB=1;

  TM1722_Write_Byte(0xcf);         //显示寄存器的00H单元开始
  TM1722_Write_Byte(lcd_0fh);     //给显示寄存器送数据，
  TM1722_STB=1;


  TM1722_Write_Byte(0x93); //显示开
  TM1722_STB=1;
}






void hot_select(uchar dd)
{
 switch(dd)
  {
   case 1: {hot1_dis(1); hot2_dis(0); hot3_dis(0); H1_dis(1);  H2_dis(0); H3_dis(0); }break;	// 加热管1
   case 2: {hot1_dis(1); hot2_dis(1); hot3_dis(0); H1_dis(1);  H2_dis(1); H3_dis(0); }break;	// 加热管1+2
   case 3: {hot1_dis(1); hot2_dis(1); hot3_dis(1); H1_dis(1);  H2_dis(1); H3_dis(1); }break;	// 加热管1+2+3
   case 4: {hot1_dis(1); hot2_dis(0); hot3_dis(1); H1_dis(1);  H2_dis(0); H3_dis(1); }break;	// 加热管1+3
   case 5: {hot1_dis(0); hot2_dis(1); hot3_dis(1); H1_dis(0);  H2_dis(1); H3_dis(1); }break;	// 加热管2+3
   case 6: {hot1_dis(0); hot2_dis(1); hot3_dis(0); H1_dis(0);  H2_dis(1); H3_dis(0); }break;	// 加热管2
   case 7: {hot1_dis(0); hot2_dis(0); hot3_dis(1); H1_dis(0);  H2_dis(0); H3_dis(1); }break;	// 加热管3
  }


}






void  mode_chose(void)
{
 read_data();

 lcd_clear(0x00);  //清屏


 dis1(1,mode_num); mode_dis(1);  Delay1ms(1000);  mode_dis(0);
 
 
 temp_dis(1);
 cur_dis(1);

 fan_dis(fan_num);


 //hot_select(fenduan_num);

 //if(tb_num==1)  tb_dis(1);  else  tb_dis(0); 

 s13_dis(1);
 s14_dis(1);
 s15_dis(1);
}




//***********************************************************************
void  pwm_set(uint pwmdata)
{
 
 P_SW2 |= 0x80;      //使能访问XSFR
 PWM5T2 = pwmdata; //设置PWM4第2次反转的PWM计数
                                               
 P_SW2 &= ~0x80;

}




void fan_run(void)
{
 P_SW2 |= 0x80;    //使能访问XSFR
 PWMCR =0x88;      //打开PWM运行
 P_SW2 &= ~0x80;

}





void fan_stop(void)
{
 pwm_set(10);
 P_SW2 |= 0x80;    //使能访问XSFR
 PWMCR =0x00;     //关闭PWM
 P_SW2 &= ~0x80;
 pwm5=10;

}





void fan_run_stop(uchar dd)	 //风量风扇启动或停止
{
 switch(dd)
  {
   case 0: { fan_stop();  fan_bit=0; }break;

   case 1: { fan_run(); pwm5=2000; pwm_set(pwm5); fan_bit=1; fan_rs_bit=1;}break;

   case 2: { fan_run(); pwm5=2400; pwm_set(pwm5); fan_bit=1; fan_rs_bit=1;}break;

   case 3: { fan_run(); pwm5=2800; pwm_set(pwm5); fan_bit=1; fan_rs_bit=1;}break;

   case 4: { fan_run(); pwm5=3200; pwm_set(pwm5); fan_bit=1; fan_rs_bit=1;}break;

   case 5: { fan_run(); pwm5=3600; pwm_set(pwm5); fan_bit=1; fan_rs_bit=1;}break;

   case 6: { fan_run(); pwm5=4000; pwm_set(pwm5); fan_bit=1; fan_rs_bit=1;}break;

  }

}

//***********************************************************************

 






void Uart() interrupt 4 
{
 if (RI) {
		  RI=0;                         
    if (g_need_process_datas) return;              //如果正在处理数据，则不接收
		if (g_recv_buffer_index >= 64) g_recv_buffer_index = 0;  //溢出处理
		g_recv_buffer[g_recv_buffer_index++] = SBUF;   //将接收到的数据保存到 接收缓冲区
		g_current_need_times = g_need_times;           //收到数据，则重新计算超时时间			
	}
 
}

//***********************************************************************
int PID1(int Set_value, int Real_value) //标准PID温度控制算法
{
int error;
float P_term, D_term;
int pid_out;


error=Set_value - Real_value;//误差量

P_term =Kp*error; //比例量

I_term1+=Ki*error ;//积分量

if(I_term1>PID_MAX) I_term1=PID_MAX; //限定积分量 上限

else if(I_term1<0) I_term1=0; //限定积分量下限

D_term =Kd*(error - last_error1) ;//微分量

last_error1=error; //缓存当 前误差量

pid_out=(signed int) (P_term+I_term1+D_term); //PID 控制量计算

if(pid_out>PID_MAX) pid_out=PID_MAX; //控制量上限=PID_ MAX

else if(pid_out<PID_MIN) pid_out=PID_MIN;//控制量下限=100

return(pid_out);
}

//***********************************************************************
int PID2(int Set_value, int Real_value) //标准PID温度控制算法
{
int error;
float P_term, D_term;
int pid_out;


error=Set_value - Real_value;//误差量

P_term =Kp*error; //比例量

I_term2+=Ki*error ;//积分量

if(I_term2>PID_MAX) I_term2=PID_MAX; //限定积分量 上限

else if(I_term2<0) I_term2=0; //限定积分量下限

D_term =Kd*(error - last_error2) ;//微分量

last_error2=error; //缓存当 前误差量

pid_out=(signed int) (P_term+I_term2+D_term); //PID 控制量计算

if(pid_out>PID_MAX) pid_out=PID_MAX; //控制量上限=PID_ MAX

else if(pid_out<PID_MIN) pid_out=PID_MIN;//控制量下限=100

return(pid_out);
}

//***********************************************************************
int PID3(int Set_value, int Real_value) //标准PID温度控制算法
{
int error;
float P_term, D_term;
int pid_out;


error=Set_value - Real_value;//误差量

P_term =Kp*error; //比例量

I_term3+=Ki*error ;//积分量

if(I_term3>PID_MAX) I_term3=PID_MAX; //限定积分量 上限

else if(I_term3<0) I_term3=0; //限定积分量下限

D_term =Kd*(error - last_error3) ;//微分量

last_error3=error; //缓存当 前误差量

pid_out=(signed int) (P_term+I_term3+D_term); //PID 控制量计算

if(pid_out>PID_MAX) pid_out=PID_MAX; //控制量上限=PID_ MAX

else if(pid_out<PID_MIN) pid_out=PID_MIN;//控制量下限=100

return(pid_out);
}





//***********************************************************************
int PID4(int Set_value, int Real_value) //标准PID温度控制算法
{
int error;
float P_term, D_term;
int pid_out;


error=Set_value - Real_value;//误差量

P_term =Kp*error; //比例量

I_term4+=Ki*error ;//积分量

if(I_term4>PID_MAX) I_term4=PID_MAX; //限定积分量 上限

else if(I_term4<0) I_term4=0; //限定积分量下限

D_term =Kd*(error - last_error4) ;//微分量

last_error4=error; //缓存当 前误差量

pid_out=(signed int) (P_term+I_term4+D_term); //PID 控制量计算

if(pid_out>PID_MAX) pid_out=PID_MAX; //控制量上限=PID_ MAX

else if(pid_out<PID_MIN) pid_out=PID_MIN;//控制量下限=100

return(pid_out);
}

 /*

//***********************************************************************
int PID5(int Set_value, int Real_value) //标准PID温度控制算法
{
int error;
float P_term, D_term;
int pid_out;


error=Set_value - Real_value;//误差量

P_term =Kp*error; //比例量

I_term5+=Ki*error ;//积分量

if(I_term5>PID_MAX) I_term5=PID_MAX; //限定积分量 上限

else if(I_term5<0) I_term5=0; //限定积分量下限

D_term =Kd*(error - last_error5) ;//微分量

last_error5=error; //缓存当 前误差量

pid_out=(signed int) (P_term+I_term5+D_term); //PID 控制量计算

if(pid_out>PID_MAX) pid_out=PID_MAX; //控制量上限=PID_ MAX

else if(pid_out<PID_MIN) pid_out=PID_MIN;//控制量下限=100

return(pid_out);
}


//***********************************************************************
int PID6(int Set_value, int Real_value) //标准PID温度控制算法
{
int error;
float P_term, D_term;
int pid_out;


error=Set_value - Real_value;//误差量

P_term =Kp*error; //比例量

I_term6+=Ki*error ;//积分量

if(I_term6>PID_MAX) I_term6=PID_MAX; //限定积分量 上限

else if(I_term6<0) I_term6=0; //限定积分量下限

D_term =Kd*(error - last_error6) ;//微分量

last_error6=error; //缓存当 前误差量

pid_out=(signed int) (P_term+I_term6+D_term); //PID 控制量计算

if(pid_out>PID_MAX) pid_out=PID_MAX; //控制量上限=PID_ MAX

else if(pid_out<PID_MIN) pid_out=PID_MIN;//控制量下限=100

return(pid_out);
}	 */

//***********************************************************************




void tm0_isr() interrupt 1  using 1	 	//1毫秒@11.0592MHz
{
 recv_cnt++;
 if(recv_cnt>4)
  {
   recv_cnt=0;
    if (g_recv_buffer_index > 0 && g_need_process_datas == 0) {
        if (g_current_need_times-- == 0) {  //定时时间到了    
			g_need_process_datas = 1;         //需要处理数据		
		   }		
     }
   }





//-----------------------------第一路脉冲控制输出-------------------------------------------------------------------
 if(tm1_on_bit)
  {
   tmp_cnt1++;

   if(tmp_cnt1==1)                           {if(temp1<temp_set1){led6=0; out1=0;}  tm1_bit=0;}

   if((tm1_bit==0)&&(tmp_cnt1>=tm1_pwm))     {led6=1; out1=1; tm1_bit=1;}

   else if((tm1_bit==1)&&(tmp_cnt1>=1000))	 {tm1_bit=0; tmp_cnt1=0;}
  }

 //-----------------------------第二路脉冲控制输出-------------------------------------------------------------------
 if(tm2_on_bit)
  {
   tmp_cnt2++;

   if(tmp_cnt2==1)                           {if(temp2<temp_set2){led4=0; out2=0;}  tm2_bit=0;}

   if((tm2_bit==0)&&(tmp_cnt2>=tm2_pwm))     {led4=1; out2=1; tm2_bit=1;}

   else if((tm2_bit==1)&&(tmp_cnt2>=1000))	 {tm2_bit=0; tmp_cnt2=0;}
  }

 //-----------------------------第三路脉冲控制输出-------------------------------------------------------------------
 if(tm3_on_bit)
  {
   tmp_cnt3++;

   if(tmp_cnt3==1)                           {if(temp3<temp_set3){led2=0;  out3=0;}  tm3_bit=0;}

   if((tm3_bit==0)&&(tmp_cnt3>=tm3_pwm))     {led2=1; out3=1; tm3_bit=1;}

   else if((tm3_bit==1)&&(tmp_cnt3>=1000))	 {tm3_bit=0; tmp_cnt3=0;}
  }

  if(sp_bit==0)
  {
   cnt3++;
   if(cnt3>100) {cnt3=0; speak=sp_bit=1;}
  
  }
 
}










//******************************************************************************************







void t3int() interrupt 19  using 2	  //50ms  定时器3  11.0592
{

 key_buf=P0&0x0f; //每50ms 扫描一次按键

 //if(key_buf==0x0f)  key_find_bit=0;


 if(fan_close_bit) //同步信号关，风机延时关闭
   {
	cnt4++;
	if(cnt4>1200) //1秒钟到	 1200 为1分钟
	  {
	    cnt4=0; 
		cnt5++;   
		if(cnt5>=fan_close_cnt)	 {cnt5=0; fan_close_ok=1;}
		  
	  }
	
   }

 dis_temp_cnt++; //温度检测0.5秒检测一次
 if(dis_temp_cnt>10) {dis_temp_cnt=0; dis_temp_bit=1;}


 if(fan_bit)  //风扇图标旋转
   {
	fan_cnt++;
	if(fan_cnt>2)  { fan_cnt=0; fan_bit2=1; fan_bit3=~fan_bit3; }
   }

 /*
 if(sp_bit==0)
  {
   cnt3++;
   if(cnt3>2) {cnt3=0; speak=sp_bit=1;}
  
  }  */





if(power_bit==0) //待机电源按键指示闪烁
  {
   
   if(power_cnt>9) { led5=~led5; power_cnt=0; }
   else power_cnt++;

  }





if(temp_set_bit)  //温度设定时数字闪烁
 {
  save_cnt++;
  if(save_cnt>80) {save_cnt=0;  save_bit=1; } //4秒后保存修改值


  if(temp_set_cnt>9) {temp_set_cnt=0;   ss_bit=~ss_bit;}
  else temp_set_cnt++;

 }

else {
	  ss_cnt++;
	  if(ss_cnt>40) { ss_cnt=0; hot_HT_bit=1;}


      }



}



//******************************************************************************************************

void main(void)
{ 

   hot_ok=0;
   hot_fan=0;
   lcd_bl=1;
   power_bit=1;

    P0M0 =B0110_0000;  ////P0.5 P0.6 设置成推勉输出
    P0M1 = 0x00;
	
		 

    //P1M0 = B1100_0000;  //P1.6, P1.7设置成推勉输出
	P1M0 = 0x00;
    P1M1 = 0x00;


    P2M0 = B0000_1000; //P2.3设置成推勉输出
    P2M1 = 0x00;


    P3M0 = 0x00;   //普通IO
    P3M1 = 0x00;

    P4M0 = 0x00;  //普通IO
    P4M1 = 0x00;

    P5M0 = 0x00;
    P5M1 = 0x00;

	
//***************************************************************************
	P_SW2 |= 0x80;                  //使能访问XSFR
    PWMCFG = 0x08;                  //配置PWM5的输出初始电平为高电平
	//PWMCR  = 0x04;					//配置PWM通道2、3、4、5受PWM波形发生器控制
    PWMCKS = 0x00;                  //选择PWM的时钟为Fosc/1
    PWMC = CYCLE;                   //设置PWM周期
                  
 //----------------------------------------------------------------------------------
    PWM5T1 = 0;                  //设置PWM5第1次反转的PWM计数
    PWM5T2 = 10;                //设置PWM5第2次反转的PWM计数
                                    //占空比为(PWM5T2-PWM5T1)/PWMC
    PWM5CR = 0x00;                  //选择PWM5输出到P2.3,不使能PWM5中断
 //----------------------------------------------------------------------------------


	PWMFDCR=0x30;                   //打开PWM外部异常功能并发生PWM外部异常时，PWM的输出口立即被设置为高阻输入模式
    
    //PWMCR |= 0x80;                  //使能PWM模块

    P_SW2 &= ~0x80;


//**********************************************************************************
  /*	 
  ACC = P_SW1;				  //串口1切换
  ACC &= ~(S1_S0 | S1_S1);    //S1_S0=1 S1_S1=0
  ACC |= S1_S0;               //(P3.6/RxD_2, P3.7/TxD_2)
  P_SW1 = ACC; 
 		
	  */
  							  //串口2切换
 

							  //PWM口切换
	//P_SW2 &=~PWM2345_S;	  //PWM_2345_S=0 P3.7/PWM2 P2.1/PWM3 P2.2/PWM4 P2.3/PWM5 P2.4/PWMFLT
 	//P_SW2 |= PWM2345_S;		  //PWM_2345_S=1 P2.7/PWM2 P4.5/PWM3 P4.4/PWM4 P4.2/PWM5 P0.5/PWMFLT


 
  
 UartInit();

 InitADC();

 Timer0Init();

 Timer3Init();
   
 //ES=1;
 

 EA=1;

 read_mode();	
 read_data();
 fan_close_read();

 hot_power_read();
 fan_power_read();

 lcd_clear(0x00);

 hot_in=1;

	 

 if(key_buf==fn_key)  //设定风扇延时关闭时间，按住功能键上电进入
  {
   speak=sp_bit=0;
   lcd_bl=0;
   led2=led1=led3=0;
   dis1(1,fan_close_cnt);

   while(key_buf==fn_key) WDT_CONTR=0X34;

   while(1)
     {
	  Delay1ms(50);

	  if(key_buf==up_key)      {speak=sp_bit=0; fan_close_cnt++; if(fan_close_cnt>250) fan_close_cnt=250; dis1(1,fan_close_cnt);   Delay1ms(200);;}

	  else if(key_buf==dn_key) {speak=sp_bit=0; fan_close_cnt--; if(fan_close_cnt<10)  fan_close_cnt=10;  dis1(1,fan_close_cnt);   Delay1ms(200);}

	  else if(key_buf==fn_key) {speak=sp_bit=0; fan_close_save();  while(key_buf==fn_key) WDT_CONTR=0X34; break;}
	 }
	led1=led2=led3=1;
	led4=led6=1;
	lcd_clear(0x00); 
	lcd_bl=1;
  }


 led5=0;
	  
 //while(1)
  //{
   //Delay1ms(50);
   //if(key_buf==power_key) 
    //{
	 speak=sp_bit=0; 
	 led5=1;
	 lcd_clear(0xff); 
	 lcd_bl=0; 
	 power_bit=1;  
	 led6=0;	Delay1ms(200);
	 led4=0;	Delay1ms(200);
	 led2=0;	Delay1ms(200);
	 led1=0;	Delay1ms(200);
	 led3=0;	Delay1ms(200);
	 led5=0;	
	 Delay1ms(1000);
	 lcd_clear(0x00);
	 
	
	// break;
	//}

  //}	  

 led1=led2=led3=1;
 led4=led5=led6=1;
 tb_bit=0;

 mode_chose();
		  

 save_bit=0;

 hot_HT_bit=0;
 hot_fan_bit=0;
 ss_cnt=0;
 auto_bit=0;

 hot_fan=1;	   //测试下风扇输出有没有
 Delay1ms(200);
 hot_fan=0;
 
 s13_dis(1); s14_dis(0);

 temp_set1=(int)tm_set1;
 temp_set2=(int)tm_set2;
 temp_set3=(int)tm_set3;
 temp_set4=(int)tm_set4;
 temp_set5=(int)tm_set5;
 temp_set6=(int)tm_set6;

 last_error1=0;
 last_error2=0;
 last_error3=0;
 last_error4=0;
 last_error5=0;
 last_error6=0;

 I_term1=0;
 I_term2=0;
 I_term3=0;
 I_term4=0;
 I_term5=0;
 I_term6=0;

 temp_num=1;	
 dis_temp_bit=1;
 hot_HT_bit=1;
 if(fan_power==0)   { fan_run_stop(fan_num);   fan_rs_bit=1;  }//打开风量

 if(hot_power==0)	led5=0;  else  led5=1;

 recv_cnt=0;
 g_recv_buffer_index = 0;              //设置接收缓冲区为空
 g_current_need_times = g_need_times;  //间隔超时时间设置
 g_need_process_datas = 0;             //设置 需要处理数据 标志位 为0
 //---------------------------------------------------------------------------------------
 while(1)
 {
   WDT_CONTR=0X34;

   if (g_need_process_datas) {       //有数据需要处理，则处理
		
		if (parse_recv_buffer(g_recv_buffer,g_recv_buffer_index)) {    //处理数据
			  led1=~led1;
			  
		}else{
		
		}
		g_recv_buffer_index = 0;
		g_need_process_datas = 0;    //服务标志清空				
	}

   
   

        if((hot_in==0)&&(tb_bit==0))  //检测到有同步信号过来
         {
	       tb_bit=1;
		   hot_in_data=1;	  
	      }

		 else if((hot_in==1)&&(tb_bit==1))  //检测到有同步信号关闭
		  {	   
		   tb_bit=0;
		   hot_in_data=0;
		  }	       

	//没有打开同步功能，按设置状态自动运行
	/*
		  if(auto_bit==0)
		    {
			 auto_bit=1;
			 
			 fan_run_stop(fan_num); //打开风量

			 //hot_dis(1);  //显示加热标志

	
			}	 */

	     

		




   //风扇图标旋转显示
   if(fan_bit2) { fan_bit2=0;  if(fan_bit3) {s13_dis(1); s14_dis(0);} else {s13_dis(0); s14_dis(1);} }




   if(dis_temp_bit)
    {
	 dis_temp_bit=0;
	 //---------------------------3路温度检测并PID控制---------------------------------------------------
     tm1=Temperature_LPF(0);
     tm1 =get_temperature(tm1);	//计算第一路温度值
	 temp1 =(int)tm1;
	 if(tm1>=0)  t1_work_bit=1;  else  {t1_work_bit=0; out1=1; led6=1;}
			
	 tm2=Temperature_LPF(1);
     tm2 =get_temperature(tm2);	//计算第二路温度值
	 temp2 =(int)tm2;
	 if(tm2>=0)  t2_work_bit=1;	else  {t2_work_bit=0; out2=1; led4=1;}

	 tm3=Temperature_LPF(2);
     tm3 =get_temperature(tm3);	//计算第三路温度值
	 temp3 =(int)tm3;
	 if(tm3>=0)  t3_work_bit=1;	else  {t3_work_bit=0; out3=1; led2=1;}

	 tm4=Temperature_LPF(3);
     tm4 =get_temperature(tm4);	//计算第四路温度值
	 temp4 =(int)tm4;
	 if(tm4>=0)  t4_work_bit=1;	else  {t4_work_bit=0; out4=1; led5=1;}


	 //------------------------第一路温度控制程序-------------------------------------------------------------------------------
  if(hot_power==0)
   {	 
	 if(t1_work_bit)
	{ 						   
	 if(temp1<=(temp_set1-2))                             {led6=0; out1=0; tm1_on_bit=0; tmp_cnt1=0;}
	 else if((temp1>(temp_set1-2))&&(temp1<temp_set1))    {tm1_on_bit=1; tm1_pwm=PID1(temp_set1,temp1);}
	 else if(temp1>temp_set1)		                      {led6=1; out1=1; tm1_on_bit=0; tmp_cnt1=0;}
	} 																	   
	 //------------------------第二路温度控制程序-------------------------------------------------------------------------------
	 if(t2_work_bit)
	{						   
	 if(temp2<=(temp_set2-2))                             {led4=0; out2=0; tm2_on_bit=0; tmp_cnt2=0;}
	 else if((temp2>(temp_set2-2))&&(temp2<temp_set2))    {tm2_on_bit=1; tm2_pwm=PID2(temp_set2,temp2);}
	 else if(temp2>temp_set2)		                      {led4=1; out2=1; tm2_on_bit=0; tmp_cnt2=0;}
	}
	 //------------------------第三路温度控制程序-------------------------------------------------------------------------------
	 if(t3_work_bit)
	{						   
	 if(temp3<=(temp_set3-2))                             {led2=0; out3=0; tm3_on_bit=0; tmp_cnt3=0;}
	 else if((temp3>(temp_set3-2))&&(temp3<temp_set3))    {tm3_on_bit=1; tm3_pwm=PID3(temp_set3,temp3);}
	 else if(temp3>temp_set3)		                      {led2=1; out3=1; tm3_on_bit=0; tmp_cnt3=0;}
	}
	
	//------------------------第四路温度控制程序-------------------------------------------------------------------------------
	 if(t4_work_bit)
	{						   
	 if(temp4<=(temp_set4-2))                             {led5=0; out4=0; tm4_on_bit=0; tmp_cnt4=0;}
	 else if((temp4>(temp_set4-2))&&(temp4<temp_set4))    {tm4_on_bit=1; tm4_pwm=PID4(temp_set4,temp4);}
	 else if(temp4>temp_set4)		                      {led5=1; out4=1; tm4_on_bit=0; tmp_cnt4=0;}
	}
   }		 
  }	 





   if(hot_HT_bit) //实测温度和加热设定值循环显示
      {
	   hot_HT_bit=0;		
	   
	   switch(temp_num)	 //显示实测温度值
          {

            case 1: { dis1(1,tm1); f_dis(1); r_dis(0); m_dis(0); if(led6==0) hot_dis(1); else hot_dis(0);}break;

            case 2: { dis1(1,tm2); f_dis(0); r_dis(0); m_dis(1); if(led4==0) hot_dis(1); else hot_dis(0);}break;

            case 3: { dis1(1,tm3); f_dis(0); r_dis(1); m_dis(0); if(led2==0) hot_dis(1); else hot_dis(0);}break;

			case 4: { dis1(1,tm4); H2_dis(0); H3_dis(1); if(led5==0) hot_dis(1); else hot_dis(0);}break;
          }   

       temp_num++;
	   if(temp_num>4)  temp_num=1; 
	  } 
   
	else  {
			switch(temp_num-1)	 //显示实测温度值
            {

              case 1: { dis1(1,tm1); }break;

              case 2: { dis1(1,tm2); }break;

              case 3: { dis1(1,tm3); }break;
		  
		      case 4: { dis1(1,tm4); }break;
              
             } 

	      }





					
   //自动保存修改的数据
   if(save_bit) {save_bit=0; temp_num=1; save_mode_wr(); temp_set_bit=0; hot_HT_bit=1;  }

   if(hot_power_bit)  //收到通讯命令，打开或关闭加热
   {
	hot_power_bit=0;
	if(hot_power==0)	//关闭加热功能
				       {
					     led5=1;
						 hot_power=1;
						 TM1722_Write_Word(1,0);
						 TM1722_Write_Word(2,15);
						 TM1722_Write_Word(3,15);
						 led6=led4=led2=1;
						 out1=out2=out3=1;
						 hot_power_save();
						 Delay1ms(600);
					   }
				   else  {
						   hot_power=0;	 	//打开加热功能
						   led5=0;
						   hot_power_save();
						   TM1722_Write_Word(1,17);
						   TM1722_Write_Word(2,17);
						   TM1722_Write_Word(3,17);
						   Delay1ms(300);
				          }
   }


   if(mode_num_bit)	//收到通讯命令，切换模式
    {
	 mode_num_bit=0;
	 speak=sp_bit=0; 
	 fan_bit=0; auto_bit=0; temp_set_bit=1; save_cnt=0;  mode_chose(); save_mode_num(); fan_run_stop(fan_num);  
	   		  temp_set1=(int)tm_set1;
              temp_set2=(int)tm_set2;
              temp_set3=(int)tm_set3;
			  temp_set4=(int)tm_set4;
	}



   if(key_buf==power_key)  //检测电源按键
      {
	    t_cnt=30;
			    
        while((key_buf==power_key)&&(t_cnt>0)) {t_cnt--; Delay1ms(10);}
	    if(t_cnt==0) //长按0.3秒关
			      {
				   speak=sp_bit=0;
				   while(key_buf==power_key) WDT_CONTR=0X34;

				   if(hot_power==0)	//关闭加热功能
				       {
					     led5=1;
						 hot_power=1;
						 TM1722_Write_Word(1,0);
						 TM1722_Write_Word(2,15);
						 TM1722_Write_Word(3,15);
						 led6=led4=led2=1;
						 out1=out2=out3=1;
						 hot_power_save();
						 Delay1ms(600);
					   }
				   else  {
						   hot_power=0;	 	//打开加热功能
						   led5=0;
						   hot_power_save();
						   TM1722_Write_Word(1,17);
						   TM1722_Write_Word(2,17);
						   TM1722_Write_Word(3,17);
						   Delay1ms(300);
				          }
				  }
	  }


	 //按fn键启停风扇
	if(key_buf==fn_key)      {
	                          speak=sp_bit=0; 
	                          if(fan_rs_bit) {fan_rs_bit=0; fan_stop();  fan_bit=0; fan_power=1;} 

							  else	{fan_rs_bit=1;fan_run_stop(fan_num); fan_power=0;}

							  fan_power_save();

	                          while(key_buf==fn_key) WDT_CONTR=0X34;
							 }


    //风力加
   if(key_buf==up_key)      {speak=sp_bit=0; temp_set_bit=1; save_cnt=0; fan_num++; if(fan_num>6)   fan_num=6;     fan_dis(fan_num); if((tb_num==0)||((tb_num==1)&&(hot_in==0))) fan_run_stop(fan_num); while(key_buf==up_key) WDT_CONTR=0X34;}

   //风力减
   else if(key_buf==dn_key) {speak=sp_bit=0; temp_set_bit=1; save_cnt=0; if(fan_num==0) fan_num=0; else fan_num--; fan_dis(fan_num);	if((tb_num==0)||((tb_num==1)&&(hot_in==0))) fan_run_stop(fan_num); while(key_buf==dn_key) WDT_CONTR=0X34;}

   	


   //模式选择
   if(key_buf==mode_key)  
    {
	 t_cnt=150;
			    
      while((key_buf==mode_key)&&(t_cnt>0)) {t_cnt--; Delay1ms(10);}
	  if(t_cnt==0) //长按1.5秒进入同步设定 
			      {	 /*
				   speak=sp_bit=0;

				   if(tb_num==0) {tb_num=1; tb_dis(1);}

				   else          {tb_num=0; tb_dis(0);}

				   while(key_buf==mode_key) WDT_CONTR=0X34;

				   
					 */
				   
				  }

	   else	{ speak=sp_bit=0; fan_bit=0; auto_bit=0; temp_set_bit=1; save_cnt=0; mode_num+=1; if(mode_num>3) mode_num=1; mode_chose(); save_mode_num(); fan_run_stop(fan_num); while(key_buf==mode_key) WDT_CONTR=0X34; 
	   		  temp_set1=(int)tm_set1;
              temp_set2=(int)tm_set2;
              temp_set3=(int)tm_set3;
	         }
			 //模式选择
	}

   
   
   //温度通道选择与设定
   if(key_buf==fd_key) //通道键
     { 
		temp_num=1;
		speak=sp_bit=0; 
		cur_dis(0);	   
		set_dis(1); 
		temp_dis(1);
		f_dis(1); m_dis(0); r_dis(0);
		H1_dis(0); H2_dis(0); H3_dis(0);
		dis1(1,tm_set1);
		temp_set_bit=1;
		save_cnt=0;

		while(key_buf==fd_key) 
				     {
					   WDT_CONTR=0X34;

					   if(ss_bit) dis1(1,tm_set1);  else dis1(0,tm_set1);
					 }
		      
		 while(1)  //-------------------第一路温度设定-------------------------------------------
				   {
					  Delay1ms(30);

					  if(save_bit) {temp_set1=(int)tm_set1; goto set_end;  }

					  if(ss_bit) dis1(1,tm_set1);  else dis1(0,tm_set1);

					  if(key_buf==up_key)         {save_cnt=0; speak=sp_bit=0; tm_set1+=1; if(tm_set1>200) tm_set1=200; dis1(1,tm_set1); Delay1ms(150);}

   
                      else if(key_buf==dn_key)    {save_cnt=0; speak=sp_bit=0;  if(tm_set1<=0) tm_set1=0; else tm_set1-=1;  dis1(1,tm_set1); Delay1ms(150);}


					  else if(key_buf==fd_key)	  {save_cnt=0; speak=sp_bit=0; while(key_buf==fd_key) WDT_CONTR=0X34;  break;}

					 }

		 temp_set1=(int)tm_set1;

		 f_dis(0);
		 m_dis(1);
		 dis1(1,tm_set2);		   
					  
		while(1)  //-------------------第二路温度设定-------------------------------------------
				   {
					  Delay1ms(30);

					  if(save_bit) {temp_set2=(int)tm_set2; goto set_end;  }

					  if(ss_bit) dis1(1,tm_set2);  else dis1(0,tm_set2);

					  if(key_buf==up_key)         {save_cnt=0; speak=sp_bit=0; tm_set2+=1; if(tm_set2>200) tm_set2=200; dis1(1,tm_set2); Delay1ms(150);}

   
                      else if(key_buf==dn_key)    {save_cnt=0; speak=sp_bit=0; if(tm_set2<=0) tm_set2=0; else tm_set2-=1;   dis1(1,tm_set2); Delay1ms(150);}


					  else if(key_buf==fd_key)	  {save_cnt=0; speak=sp_bit=0; while(key_buf==fd_key) WDT_CONTR=0X34;  break;}

					 }		  
		temp_set2=(int)tm_set2;

		 m_dis(0);
		 r_dis(1);
		 dis1(1,tm_set3);		   
					  
		while(1)  //-------------------第三路温度设定-------------------------------------------
				   {
					  Delay1ms(30);

					  if(save_bit) {temp_set3=(int)tm_set3; goto set_end;  }

					  if(ss_bit) dis1(1,tm_set3);  else dis1(0,tm_set3);

					  if(key_buf==up_key)         {save_cnt=0; speak=sp_bit=0; tm_set3+=1; if(tm_set3>200) tm_set3=200; dis1(1,tm_set3); Delay1ms(150);}

   
                      else if(key_buf==dn_key)    {save_cnt=0; speak=sp_bit=0; if(tm_set3<=0) tm_set3=0; else tm_set3-=1;   dis1(1,tm_set3); Delay1ms(150);}


					  else if(key_buf==fd_key)	  {save_cnt=0; speak=sp_bit=0; while(key_buf==fd_key) WDT_CONTR=0X34;  break;}

					 }	
					 
	   temp_set3=(int)tm_set3;	

	   


	   
	   set_end: nop; nop;
	   save_bit=0;			 
	   temp_num=1;				 
	   temp_set_bit=0;
	   
	   f_dis(0);
	   m_dis(0);
	   r_dis(0); 
	   cur_dis(1);	   
	   set_dis(0);
	   save_mode_wr(); //保存参数数据				   
	  }
    
   
 }
		    
}
