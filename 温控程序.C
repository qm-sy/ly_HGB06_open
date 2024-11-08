//  温制程序 	2020-6-9
//CPU STC15W4K32S4	   11.0592Mhz
			   

#include  "STC15W4Kxx.H"
#include <stdlib.h>
#include "modbus.h"
#include "intrins.H"  
#include <stdio.h> 
#include  <bin.c>
#include "math.h"


#define uchar  unsigned char
#define uint   unsigned int
#define ulong  unsigned long

#define B 3950.0

#define TN 298.15

#define RN 10

#define BaseVol 5.04 



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

#define CYCLE   4096    



#define Kp 18      
#define Ki 13     
#define Kd 0.8    

#define time_max  1450  


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
int xdata g_need_times = 10;            
int xdata g_current_need_times = 0;     
volatile unsigned char idata g_need_process_datas = 0;   

//**************************************************************  

 

 uchar idata   key_buf;
	   
 uchar idata  t_cnt;

 uchar idata  fan_cnt;

 uchar idata  dis_temp_cnt;
 
 uchar idata  cnt3;
 uchar idata  cnt4;
 uchar idata  cnt5;

 uchar  idata  fan_close_cnt;

 uint  idata  ss_cnt;

 uchar idata  temp_set_cnt;
 uint  idata  save_cnt;
 
 uint  idata  temp[7];

 uint  idata  pwm5;

 uint  idata  fan_pwm;
 uchar idata  fan_num;

 uchar idata  mode_num;
 uchar idata  fenduan_num;
 uchar idata  hot_num;
 uchar idata  temp_set;
 uchar idata  tb_num;

 uchar idata  power_cnt;

 uchar  idata  mode_wr[5]; 	
 


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

 uint  idata  tt1;
 uint  idata  tt2;
 uint  idata  time;

 uchar idata out_clear_cnt;
 
 uchar idata power_run; //运行标志

 unsigned char idata dev_address;

	 
 unsigned long idata PeriodCnt = 0; //PWM 周期计数值
 uchar  idata  HighRH = 0; //高电平重载值的高字节
 uchar  idata  HighRL = 0; //高电平重载值的低字节
 uchar  idata  LowRH = 0; //低电平重载值的高字节
 uchar  idata  LowRL = 0; //低电平重载值的低字节


 uchar   xdata   ID_old[7];	
 uchar   xdata   ID_new[7];  
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

sbit T4=P1^3;
sbit T5=P1^4;



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

bit hot_bit;

bit power_bit;
bit fan_bit;
bit fan_bit2;
bit fan_bit3;

bit tb_num_bit;
bit save_all_bit;
bit mode_num_bit;
bit hot_power_bit;
bit power_on_bit;	//通讯命令-打开电源
bit power_on_bit2;

 

bit  PWMOUT;

bit hot_fan_bit;

bit fan_close_bit;
bit fan_close_ok;
bit tb_bit;



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

  /*


// 配置并启动 PWM，fr-频率，dc-占空比 
void ConfigPWM(unsigned int fr, unsigned char dc){
    unsigned int high, low;
   
    PeriodCnt = (11059200/12) / fr; //计算一个周期所需的计数值   9216
    high = (PeriodCnt*dc) / 100; //计算高电平所需的计数值
    low = PeriodCnt - high; //计算低电平所需的计数值
    high = 65536 - high + 12; //计算高电平的重载值并补偿中断延时
    low = 65536 - low + 12;//计算低电平的重载值并补偿中断延时
   
    HighRH = (unsigned char)(high>>8); //高电平重载值拆分为高低字节
    HighRL = (unsigned char)high;
    LowRH = (unsigned char)(low>>8); //低电平重载值拆分为高低字节
    LowRL = (unsigned char)low;
   
    TMOD &= 0xF0; //清零 T0 的控制位
    TMOD |= 0x01; //配置 T0 为模式 1
    TH0 = HighRH; //加载 T0 重载值
    TL0 = HighRL;
    ET0 = 1; //使能 T0 中断
    TR0 = 1; //启动 T0
    PWMOUT = 1; //输出高电平
}





// 占空比调整函数，频率不变只调整占空比 
void AdjustDutyCycle(uchar dc){
    unsigned int high, low;

	if(dc>=100) dc=99;
	else if(dc==0)  dc=1;

	
    high = (PeriodCnt*dc) / 100; //计算高电平所需的计数值
    low = PeriodCnt - high; //计算低电平所需的计数值
    high = 65536 - high + 12; //计算高电平的重载值并补偿中断延时
    low = 65536 - low + 12;//计算低电平的重载值并补偿中断延时

    
   
    HighRH = (unsigned char)(high>>8); //高电平重载值拆分为高低字节
    HighRL = (unsigned char)high;
    LowRH = (unsigned char)(low>>8); //低电平重载值拆分为高低字节
    LowRL = (unsigned char)low;
}




// 关闭 PWM 
void ClosePWM(void){
    TR0 = 0; //停止定时器
    ET0 = 0; //禁止中断
    PWMOUT = 1; //输出高电平
	
}

  */


//*******************************************************************************

 /*
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

 cptr = 0x5f00;         //从程序区读取加密ID号	地址为0x5f00
    for (i=0; i<7; i++)         //读7个字节
    {
       ID_new[i]= *cptr++;     
    } 


 for (i=0; i<7; i++){
		              if(ID_old[i]!=ID_new[i])  return 1;  //比对数据不正确

                     }
 return 0;	 //比对数据正确

}
		*/

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


//**************************************************************************
/*

int PID(int Set_value,int Real_value) //标准PID温度控制算法
{
        float uk ,uk1 ,duk;
        int pid_out,e,e1 ,e2;
        e=Set_value-Real_value;//误差量
        duk=Kp*(e-e1)+Ki*e+Kd*(e-2*e1+e2);  //+Kd*(e-2e1+e2)
        uk=uk1+duk;
        pid_out=(int)uk;
        uk1=uk;
        e2=e1;
        e1=e;
        if(pid_out>1000) //1000
        {
                pid_out=990;//        990
        }
        else if(pid_out<10)         //10
        {
                pid_out=10;                //10
        }
        //outp=pid_out;

        return(pid_out);
               
}
	*/

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
		  

void Timer0Init(void)		//10ms@11.0592MHz
{
	AUXR &= 0x7F;		//定时器时钟12T模式
	TMOD &= 0xF0;		//设置定时器模式
	TMOD |= 0x01;		//设置定时器模式
	TL0 = 0x00;		//设置定时初值
	TH0 = 0xDC;		//设置定时初值
	TF0 = 0;		//清除TF0标志
	TR0 = 0;		//定时器0开始计时
	ET0=0;
}



void Timer1Init(void)	
{
	TMOD &= 0x0f;		//设置计数器模式
	TMOD |= 0x50;		//设置计数器模式
	TL1 = 0;		//设置定时初值
	TH1 = 0;		//设置定时重载值
    ET1=0;
	//TR1 = 1;		//定时器1开始计数
}





void Timer3Init(void)		//5毫秒@11.0592MHz
{
	T4T3M |= 0x02;		//定时器时钟1T模式
	T3L = 0x00;		//设置定时初始值
	T3H = 0x28;		//设置定时初始值
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
	
void save_mode_num(void)  
{
 uint addr;
 
 addr=0x0000;
 EA=0;
 IapEraseSector(addr);    

 IapProgramByte(addr,mode_num); 
 
               
 EA=1;
}


void fan_close_save(void)
{
 uint addr;
 
 addr=0x1000;
 EA=0;
 IapEraseSector(addr);    

 IapProgramByte(addr,fan_close_cnt); 
 
               
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



//**********************************************************
void  read_mode_wr_data(void) 
{
 fenduan_num=mode_wr[0]; 
 tb_num=mode_wr[1];	    
 fan_num=mode_wr[2];	
 hot_num=mode_wr[3];	
 temp_set=mode_wr[4];	

 if((fenduan_num==0)||(fenduan_num>7))  fenduan_num=1;
 if(tb_num>1)       tb_num=0; 
 if(fan_num>6)      fan_num=0;
 if(hot_num>100)    hot_num=50;
 if(temp_set>220)  	temp_set=80;

}


void  save_mode_wr(void)
{
 uchar n;
 uint addr;

 mode_wr[0]=fenduan_num; 
 mode_wr[1]=tb_num;	     
 mode_wr[2]=fan_num;	
 mode_wr[3]=hot_num;	
 mode_wr[4]=temp_set;

 if(mode_num==1) 
   {
    addr=0x0200;
	EA=0;
	IapEraseSector(addr);    
    for(n=0;n<5;n++) {IapProgramByte(addr,mode_wr[n]);	addr++;}
	EA=1;
   }

 else if(mode_num==2) 
   {
    addr=0x0400;
	EA=0;
	IapEraseSector(addr);    
    for(n=0;n<5;n++) {IapProgramByte(addr,mode_wr[n]);	addr++;}
	EA=1;
   }

 else if(mode_num==3) 
   {
    addr=0x0600;
	EA=0;
	IapEraseSector(addr);    
    for(n=0;n<5;n++) {IapProgramByte(addr,mode_wr[n]);	addr++;}
	EA=1;
   }

 else if(mode_num==4) 
   {
    addr=0x0800;
	EA=0;
	IapEraseSector(addr);    
    for(n=0;n<5;n++) {IapProgramByte(addr,mode_wr[n]);	addr++;}
	EA=1;
   }

 else if(mode_num==5) 
   {
    addr=0x0a00;
	EA=0;
	IapEraseSector(addr);    
    for(n=0;n<5;n++) {IapProgramByte(addr,mode_wr[n]);	addr++;}
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

 if((mode_num==0)||(mode_num>5)) {mode_num=1;}

 if(mode_num==1) 
   {
   	EA=0;
	addr=0x0200;
    for(n=0;n<5;n++) {mode_wr[n]=IapReadByte(addr);	addr++;}
   	EA=1;
   }

 else if(mode_num==2) 
   {
   	EA=0;
	addr=0x0400;
    for(n=0;n<5;n++) {mode_wr[n]=IapReadByte(addr);	addr++;}
   	EA=1;
   }

 else if(mode_num==3) 
   {
   	EA=0;
	addr=0x0600;
    for(n=0;n<5;n++) {mode_wr[n]=IapReadByte(addr);	addr++;}
   	EA=1;
   }

 else if(mode_num==4) 
   {
   	EA=0;
	addr=0x0800;
    for(n=0;n<5;n++) {mode_wr[n]=IapReadByte(addr);	addr++;}
   	EA=1;
   }

 else if(mode_num==5) 
   {
   	EA=0;
	addr=0x0a00;
    for(n=0;n<5;n++) {mode_wr[n]=IapReadByte(addr);	addr++;}
   	EA=1;
   }

 read_mode_wr_data();  
 



}

	



//__________________________________________________________________________


//***********************************************************************

void InitADC(void)
{
    P1ASF = B0010_0111;                   
    ADC_RES = 0;                    
    ADC_RESL=0;
    ADC_CONTR = ADC_POWER | ADC_SPEEDL;
   
}


//----------------------------
//读取ADC结果
//----------------------------
uint GetADCResult(uchar ch)      
{
 uint dd=0;

    ADC_CONTR = ADC_POWER | ADC_SPEEDL | ch | ADC_START;
    _nop_();                      
    _nop_();
    _nop_();
    _nop_();
    while (!(ADC_CONTR & ADC_FLAG));
    ADC_CONTR &= ~ADC_FLAG;         

    dd= ADC_RES;                 
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
    temp[i]= GetADCResult(ch); 
    Delay1ms(5);
  }
 for(j=0;j<=6;j++)
  {                      
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

return (temp[1]+temp[2]+temp[3]+temp[4]+temp[5])/5; 

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





/**************************************************************************
 - 功能描述：51单片机的串口发送字节的函数
 - 隶属模块：STC51串口操作
 - 函数属性：外部，使用户使用
 - 参数说明：mydata:要发送的一个字节
 - 返回说明：无
 - 注：发送一个字节，是串口发送的基础操作
 **************************************************************************/
 
void UART_Send_Byte(unsigned char mydata)	
{

 TI=0;         //清除TI位
  
 SBUF=mydata;
 while(TI==0);
 TI=0;          //清除TI位
 
 
}

 

/**************************************************************************
 - 功能描述：51单片机的串口发送字符串
 - 隶属模块：STC51串口操作
 - 函数属性：外部，使用户使用
 - 参数说明：s:指向字符串的指针
 - 返回说明：无
 **************************************************************************/
 



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
 


  TM1722_STB=1;           
  TM1722_CLK=1;
  TM1722_DIO=1;
  TM1722_Write_Byte(0x00); 
  TM1722_STB=1;
  TM1722_Write_Byte(0x44);   
  TM1722_STB=1;

  if(num_addr!=1) goto  next_num1;
   
  TM1722_Write_Byte(addr1);
  TM1722_Write_Byte(temp1);     
  TM1722_STB=1;	 

  next_num1: nop;

  TM1722_Write_Byte(addr2);//显示寄存器的00H单元开始
  TM1722_Write_Byte(temp2);     //给显示寄存器送数据，
  TM1722_STB=1;

  TM1722_Write_Byte(0x93); 
  TM1722_STB=1;
  
}   
 

void fan_dis(uchar num)   
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









void lcd_clear(uchar dat)  
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

  TM1722_Write_Byte(0x93); 
  TM1722_STB=1;

}



void  dis1(bit sss,uint ss)    
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





void temp_dis(bit on_off) 
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


  TM1722_Write_Byte(0x93); 
  TM1722_STB=1;
}



void hot_dis(bit on_off) 
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


  TM1722_Write_Byte(0x93); 
  TM1722_STB=1;
}



void hot1_dis(bit on_off) 
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


  TM1722_Write_Byte(0x93); 
  TM1722_STB=1;
}





void hot2_dis(bit on_off) 
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
   



void hot3_dis(bit on_off) 
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


  TM1722_Write_Byte(0x93); 
  TM1722_STB=1;
}





void H1_dis(bit on_off) 
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


  TM1722_Write_Byte(0x93); 
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


  TM1722_Write_Byte(0x93); 
  TM1722_STB=1;
}





void H3_dis(bit on_off) 
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


  TM1722_Write_Byte(0x93); 
  TM1722_STB=1;
}



void tb_dis(bit on_off) 
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


  TM1722_Write_Byte(0x93); 
  TM1722_STB=1;
}







void jg_dis(bit on_off) 
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


  TM1722_Write_Byte(0x93); 
  TM1722_STB=1;
}






void mode_dis(bit on_off) 
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


  TM1722_Write_Byte(0x93); 
  TM1722_STB=1;
}








void bf_dis(bit on_off) 
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


  TM1722_Write_Byte(0x93); 
  TM1722_STB=1;
}





void set_dis(bit on_off) 
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


  TM1722_Write_Byte(0x93); 
  TM1722_STB=1;
}



void cur_dis(bit on_off) 
{
  if(on_off) {lcd_03h=lcd_03h|B0000_1000; 	}
  else 		 {lcd_03h=lcd_03h&B1111_0111; 	}

  TM1722_STB=1;           
  TM1722_CLK=1;
  TM1722_DIO=1;
  TM1722_Write_Byte(0x00); //工作模式
  TM1722_STB=1;
  TM1722_Write_Byte(0x44);   //固定地址模式
  TM1722_STB=1;

  TM1722_Write_Byte(0xc3);         //显示寄存器的00H单元开始
  TM1722_Write_Byte(lcd_03h);     //给显示寄存器送数据，
  TM1722_STB=1;


  TM1722_Write_Byte(0x93); 
  TM1722_STB=1;
}




void f_dis(bit on_off) 
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


  TM1722_Write_Byte(0x93); 
  TM1722_STB=1;
}





void m_dis(bit on_off) 
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


  TM1722_Write_Byte(0x93); 
  TM1722_STB=1;
}




void r_dis(bit on_off) 
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


  TM1722_Write_Byte(0x93); 
  TM1722_STB=1;
}




void p1_dis(bit on_off) 
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


  TM1722_Write_Byte(0x93); 
  TM1722_STB=1;
}




void p2_dis(bit on_off) 
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


  TM1722_Write_Byte(0x93); 
  TM1722_STB=1;
}




void s13_dis(bit on_off) 
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


  TM1722_Write_Byte(0x93); 
  TM1722_STB=1;
}




void s14_dis(bit on_off) 
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


  TM1722_Write_Byte(0x93); 
  TM1722_STB=1;
}


void s15_dis(bit on_off) 
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


  TM1722_Write_Byte(0x93); 
  TM1722_STB=1;
}






void hot_select(uchar dd)
{
 switch(dd)
  {
   case 1: {hot1_dis(1); hot2_dis(0); hot3_dis(0); H1_dis(1);  H2_dis(0); H3_dis(0); }break;	
   case 2: {hot1_dis(1); hot2_dis(1); hot3_dis(0); H1_dis(1);  H2_dis(1); H3_dis(0); }break;	
   case 3: {hot1_dis(1); hot2_dis(1); hot3_dis(1); H1_dis(1);  H2_dis(1); H3_dis(1); }break;	
   case 4: {hot1_dis(1); hot2_dis(0); hot3_dis(1); H1_dis(1);  H2_dis(0); H3_dis(1); }break;	
   case 5: {hot1_dis(0); hot2_dis(1); hot3_dis(1); H1_dis(0);  H2_dis(1); H3_dis(1); }break;	
   case 6: {hot1_dis(0); hot2_dis(1); hot3_dis(0); H1_dis(0);  H2_dis(1); H3_dis(0); }break;	
   case 7: {hot1_dis(0); hot2_dis(0); hot3_dis(1); H1_dis(0);  H2_dis(0); H3_dis(1); }break;
  }


}






void  mode_chose(void)
{
 read_data();

 lcd_clear(0x00);  


 dis1(1,mode_num); mode_dis(1);  Delay1ms(1000);  mode_dis(0);
 
 
 dis1(1,hot_num); 
 bf_dis(1);

 fan_dis(fan_num);

 hot_select(fenduan_num);

 if(tb_num==1)  tb_dis(1);  else  tb_dis(0); 

 s13_dis(1);
 s14_dis(1);
 s15_dis(1);
}




//***********************************************************************
void  pwm_set(uint pwmdata)
{
 
 P_SW2 |= 0x80;      
 PWM5T2 = pwmdata; 
                                               
 P_SW2 &= ~0x80;

}




void fan_run(void)
{
 P_SW2 |= 0x80;    
 PWMCR =0x88;      
 P_SW2 &= ~0x80;

}





void fan_stop(void)
{
 pwm_set(10);
 P_SW2 |= 0x80;    
 PWMCR =0x00;     
 P_SW2 &= ~0x80;
 pwm5=10;

}





void fan_run_stop(uchar dd)	 
{
 switch(dd)
  {
   case 0: { fan_stop();  fan_bit=0; }break;

   case 1: { fan_run(); pwm5=2000; pwm_set(pwm5); fan_bit=1;}break;

   case 2: { fan_run(); pwm5=2400; pwm_set(pwm5); fan_bit=1;}break;

   case 3: { fan_run(); pwm5=2800; pwm_set(pwm5); fan_bit=1;}break;

   case 4: { fan_run(); pwm5=3200; pwm_set(pwm5); fan_bit=1;}break;

   case 5: { fan_run(); pwm5=3600; pwm_set(pwm5); fan_bit=1;}break;

   case 6: { fan_run(); pwm5=4000; pwm_set(pwm5); fan_bit=1;}break;

  }

}

//***********************************************************************

 
void send_buffer(unsigned char *buf,int len)
{	
	while (len--) {
		TI=0;     
		SBUF = *buf++;
		while (TI == 0);
		TI=0;
	}
}





void Uart() interrupt 4   	
{
 if (RI) {
		  RI=0;                         
    if (g_need_process_datas) return;              
		if (g_recv_buffer_index >= 64) g_recv_buffer_index = 0;  
		g_recv_buffer[g_recv_buffer_index++] = SBUF;   
		g_current_need_times = g_need_times;           			
	}
 
}
//****************************************************************************************

//***********************************************************************
void time_inset(void)
{
 tt1=time*5+57979;	 
 tt2=65000;
}



void exint0()  interrupt 0 using 1        
{					   
 out1=out2=out3=1;	
 TL0 = tt1;		//设置定时初值
 TH0 = tt1>>8;	//设置定时重载值
 zb_bit=0;
 TF0=0;
 ET0=1;
 TR0=1;	 
}



void tm0_isr() interrupt 1  using 2
{
 if(zb_bit==0) {
   				switch(fenduan_num)
                   {
                     case 1: {out1=0; }break;
					 case 2: {out1=0; out2=0; }break;
					 case 3: {out1=0; out2=0; out3=0; }break;
					 case 4: {out1=0; out3=0; }break;
					 case 5: {out2=0; out3=0; }break;
					 case 6: {out2=0; }break;
					 case 7: {out3=0; }break;

					}
            	
                TR0=0;  
				TL0 = tt2;		//设置定时初值
                TH0 = tt2>>8;	//设置定时重载值
				TF0=0;
				TR0=1;
				zb_bit=1;
				
			    }
 else  {out1=out2=out3=1;  TR0=0; ET0=0;}
}










//******************************************************************************************






void t3int() interrupt 19  using 3	  //5ms  定时器3  11.0592
{

    if (g_recv_buffer_index > 0 && g_need_process_datas == 0) 
	  {
        if (g_current_need_times-- == 0) {     
			g_need_process_datas = 1;         		
		   }		
      }
  


 key_buf=P0&0x0f; 

 if(fan_close_bit) 
   {
	cnt4++;
	if(cnt4>199) 
	  {
	    cnt4=0; 
		cnt5++;   
		if(cnt5>=fan_close_cnt)	 {cnt5=0; fan_close_ok=1;}
		  
	  }
	
   }

 dis_temp_cnt++; 
 if(dis_temp_cnt>100) {dis_temp_cnt=0; dis_temp_bit=1;}

 
 if(fan_bit)  
   {
	fan_cnt++;
	if(fan_cnt>20)  { fan_cnt=0; fan_bit2=1; fan_bit3=~fan_bit3; }
   }  


 if(sp_bit==0)
  {
   cnt3++;
   if(cnt3>20) {cnt3=0; speak=sp_bit=1;}
  
  } 




  
if(power_bit==0) 
  {
   
   if(power_cnt>90) { led5=~led5; power_cnt=0; }
   else power_cnt++;

  }	 





if(temp_set_bit)  
 {
  save_cnt++;
  if(save_cnt>400) {save_cnt=0;  save_bit=1; } 


  if(temp_set_cnt>90) {temp_set_cnt=0;   ss_bit=~ss_bit;}
  else temp_set_cnt++;

 }

else {
	  ss_cnt++;
	  if(ss_cnt>600) { ss_cnt=0; hot_HT_bit=1;}


      }

	 

}



//******************************************************************************************************

void main(void)
{ 

   hot_ok=0;
   hot_fan=0;
   lcd_bl=1;
   power_bit=1;

    P0M0 =B0110_0000;  
    P0M1 = 0x00;
	
		 

    P1M0 = B1100_0000;  
    P1M1 = B0001_1000;


    P2M0 = B1000_1110; 
    P2M1 = 0x00;


    P3M0 = 0x00;   
    P3M1 = 0x00;

    P4M0 = 0x00;  
    P4M1 = 0x00;

    P5M0 = 0x00;
    P5M1 = 0x00;

	out1=out2=out3=1;

	
//***************************************************************************
	P_SW2 |= 0x80;                  
    PWMCFG = 0x08;                  
	//PWMCR  = 0x04;					
    PWMCKS = 0x00;                  
    PWMC = CYCLE;                   
                  
 //----------------------------------------------------------------------------------
    PWM5T1 = 0;                  
    PWM5T2 = 10;                
                                    
    PWM5CR = 0x00;                  
 //----------------------------------------------------------------------------------


	PWMFDCR=0x30;                   
    
    //PWMCR |= 0x80;                  

    P_SW2 &= ~0x80;


//**********************************************************************************
  /*	 
  ACC = P_SW1;				  //串口1切换
  ACC &= ~(S1_S0 | S1_S1);    //S1_S0=1 S1_S1=0
  ACC |= S1_S0;               //(P3.6/RxD_2, P3.7/TxD_2)
  P_SW1 = ACC; 
 		
	  */
  							 
 
 UartInit();

 InitADC();

 Timer0Init();

 Timer1Init();

 Timer3Init();
   
 //ES=1;


 IT0 = 1;      
 

 EA=1;
 T4=T5=1;
 read_mode();	
 read_data();
 fan_close_read();


 T4=T5=1;
 nop; nop; nop;
 nop; nop; nop;
 if((T4==0)&&(T5==0))       dev_address=3;
 else if((T4==1)&&(T5==0))  dev_address=4;
 else if((T4==0)&&(T5==1))  dev_address=5;

 nop;
 restar: nop; nop;

 lcd_clear(0x00);

 hot_in=1;
 power_run=0;
	 
 Delay1ms(50);

 if(key_buf==fn_key)  
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
 out_clear_cnt=0;
 

 while(1)
  {
   Delay1ms(2);
    nop;
	nop;
   
	 if (g_need_process_datas) {       
		
		if (parse_recv_buffer(g_recv_buffer,g_recv_buffer_index)==2)   
		   {   led1=~led1;}	  
			  
		g_recv_buffer_index = 0;
		g_need_process_datas = 0;    				
	  } 

	if(power_on_bit) { power_on_bit=0; power_on_bit2=0; lcd_bl=0; power_bit=1; power_run=1; break; }

	
	if(key_buf==power_key) 
    {
	 speak=sp_bit=0; 
	 led5=1;
	 lcd_clear(0xff); 
	 lcd_bl=0; 
	 power_bit=1;  
	 power_run=1;
	 led6=0;	Delay1ms(200);
	 led4=0;	Delay1ms(200);
	 led2=0;	Delay1ms(200);
	 led1=0;	Delay1ms(200);
	 led3=0;	Delay1ms(200);
	 led5=0;	
	 Delay1ms(1000);
	 lcd_clear(0x00);
	 
	 //led1=led2=led3=1;
	 //led4=led6=1;
	 break;
	}	
  }	
   
 
 restar2: nop; nop;

 led1=led2=led3=0;
 led4=led6=0;
 tb_bit=0;

 mode_chose();
		  
 led5=0;

 save_bit=0;

 tm1=Temperature_LPF(0);	 
 tm1 =get_temperature(tm1);	


 hot_HT_bit=0;
 hot_fan_bit=0;
 ss_cnt=0;
 auto_bit=0;

 hot_fan=1;	   
 Delay1ms(200);
 hot_fan=0;
 
 s13_dis(1); s14_dis(0);

 while(1)
 {
   WDT_CONTR=0X34;

   if (g_need_process_datas) {       
		
		if (parse_recv_buffer(g_recv_buffer,g_recv_buffer_index)==2)     
			 {   led1=~led1;} 
			  
		g_recv_buffer_index = 0;
		g_need_process_datas = 0;    			
	}

	if(save_all_bit) 
	  {
	   save_all_bit=0;

	    save_mode_wr();    
		fan_close_save();
	   goto  restar2;
	  }
	  

	if(mode_num_bit)  
	  {
	    mode_num_bit=0;
		mode_chose(); 
		save_mode_num();

	  }

	 if(tb_num_bit)	 
	  {
	   tb_num_bit=0;

	   if(tb_num==0) tb_dis(0);

	   else          tb_dis(1);
		  
		save_mode_wr();
		fan_bit=0;
				 
		led1=led2=led3=1;
	    led4=led5=led6=1;

	    EX0=0; TR1=0; ET0=0;	 
		out1=out2=out3=1;

		hot_dis(0);  

		fan_stop(); 

		goto  restar2;
	  }

    if(power_on_bit2)	
	 {
	   power_on_bit2=0;
	               lcd_clear(0x00); 
	               lcd_bl=1;
				   fan_bit=0;
				   auto_bit=0;
				   led1=led2=led3=1;
	               led4=led5=led6=1;

				   power_run=0;
				   hot_fan=0;

				   EX0=0; TR1=0; ET0=0;	 
				   out1=out2=out3=1;

				   hot_dis(0);  

				   fan_stop();  
				   goto  restar;

	  }

   if(tb_num==1) 
     {

        if(hot_in==0)  
         {
	       if(tb_bit==0)
			 { 
			  tb_bit=1;
			  auto_bit=0;
			  fan_close_bit=0; 

			  fan_run_stop(fan_num);  

			  hot_dis(1);  

			  time=hot_num*14; if(time>time_max)  time=time_max;  
			  time_inset();
			  TH1=TL1=0; IE0=0; EX0=1; 						  


	          }
	      }
		 else {	
			   if(tb_bit==1)
			     {
				   tb_bit=0;
			       auto_bit=0;
				   cnt4=cnt5=0;
				   fan_close_bit=1;  

				   hot_dis(0); 

				   EX0=0; TR1=0; ET0=0;	 
				   out1=out2=out3=1;
				  }

				if(fan_close_ok) {fan_close_ok=0; fan_stop(); fan_bit=0;}

		       }

	  }

	else { 
		  if(auto_bit==0)
		    {
			 auto_bit=1;
			 
			 fan_run_stop(fan_num); 

			 hot_dis(1); 

			 time=hot_num*14; if(time>time_max)	time=time_max;  
			 time_inset();
			 TH1=TL1=0; IE0=0; EX0=1; 
			}

	     }



   
   if(fan_bit2) { fan_bit2=0;  if(fan_bit3) {s13_dis(1); s14_dis(0);} else {s13_dis(0); s14_dis(1);} }




   if(dis_temp_bit)
    {
	 dis_temp_bit=0;

     tm1=Temperature_LPF(0);
     tm1 =get_temperature(tm1);	

	 tm6=Temperature_LPF(5);	
     tm6 =get_temperature(tm6);	

	 if((dis_ss_bit==1)&&(temp_set_bit==0))  { if(tm1>0) dis1(1,tm1); }

	 if(tm1>=temp_set)   hot_ok=1; 
	 else                hot_ok=0;


	 if((tm6>45)&&(hot_fan_bit==0))      {  hot_fan=1;   hot_fan_bit=1; } 

	 else if((tm6<40)&&(hot_fan_bit==1)) {  hot_fan=0;   hot_fan_bit=0; power_bit=1; led5=0;} 

	 if(tm6>100)  { power_bit=0; if(led5) {speak=sp_bit=0; jg_dis(0);} else jg_dis(1); }

	  if(tm6>118)  { 
	                 EX0=0; TR1=0; ET0=0;	 
				     out1=out2=out3=1; 
					 hot_dis(0);  
					 jg_dis(1);
					 hot_ok=1; 
					  
					 while(1){
							  Delay1ms(100);
							  if(led5) { speak=sp_bit=0;  }
							  
							  if(key_buf==power_key)  
								{
								  speak=sp_bit=0;
				                  lcd_clear(0x00); 
	                              lcd_bl=1;
				                  fan_bit=0;
				                  auto_bit=0;
				                  led1=led2=led3=1;
	                              led4=led5=led6=1;

				                  EX0=0; TR1=0; ET0=0;	 
				                  out1=out2=out3=1;

				                  fan_stop();  

				                  while(key_buf==power_key) WDT_CONTR=0X34;

				                  goto  restar;

								}
					         }

				   }

	}





   if(hot_HT_bit) 
      {
	   hot_HT_bit=0;		
				   

       if(dis_ss_bit) {dis_ss_bit=0; temp_dis(0); bf_dis(1); dis1(1,hot_num); }	 
	  
	   else           {dis_ss_bit=1; if(tm1>0) {temp_dis(1); bf_dis(0); dis1(1,tm1);}} 
	  } 
   






					
   
   if(save_bit) {save_bit=0; temp_set_bit=0; hot_HT_bit=1; save_mode_wr(); }





   if(key_buf==power_key)  
      {
	    t_cnt=150;
			    
        while((key_buf==power_key)&&(t_cnt>0)) {t_cnt--; Delay1ms(10);}
	    if(t_cnt==0)  
			      {
				   speak=sp_bit=0;
				   lcd_clear(0x00); 
	               lcd_bl=1;
				   fan_bit=0;
				   auto_bit=0;
				   led1=led2=led3=1;
	               led4=led5=led6=1;

				   power_run=0;
				   hot_fan=0;

				   EX0=0; TR1=0; ET0=0;	 
				   out1=out2=out3=1;

				   hot_dis(0);  

				   fan_stop();  

				   while(key_buf==power_key) WDT_CONTR=0X34;

				   goto  restar;

				  }
	  }


	 
	if((key_buf==fn_key)&&(power_bit==0))      {speak=sp_bit=0; 	power_bit=1; led5=0;  while(key_buf==fn_key) WDT_CONTR=0X34;}


    
   if(key_buf==fn_up_key)      {speak=sp_bit=0; temp_set_bit=1; save_cnt=0; fan_num++; if(fan_num>6)   fan_num=6;     fan_dis(fan_num); if((tb_num==0)||((tb_num==1)&&(hot_in==0))) fan_run_stop(fan_num); while(key_buf==fn_up_key) WDT_CONTR=0X34;}

   
   else if(key_buf==fn_dn_key) {speak=sp_bit=0; temp_set_bit=1; save_cnt=0; if(fan_num==0) fan_num=0; else fan_num--; fan_dis(fan_num);	if((tb_num==0)||((tb_num==1)&&(hot_in==0))) fan_run_stop(fan_num); while(key_buf==fn_dn_key) WDT_CONTR=0X34;}

   
   else if(key_buf==up_key)    
       {
	     speak=sp_bit=0; temp_dis(0); bf_dis(1); temp_set_bit=1; 

		 save_cnt=0; hot_num+=5; if(hot_num>100) hot_num=100; dis1(1,hot_num); 

		 time=hot_num*14; if(time>time_max)	time=time_max;  time_inset();

		 if(hot_bit)  {hot_bit=0; TH1=TL1=0; IE0=0; EX0=1; }

		 while(key_buf==up_key) WDT_CONTR=0X34;
		}

   
   else if(key_buf==dn_key)  
       {
	    speak=sp_bit=0; temp_dis(0); bf_dis(1); temp_set_bit=1;

	    save_cnt=0; if(hot_num>=5) hot_num-=5;  dis1(1,hot_num);

		if(hot_num<5) {hot_num=0; EX0=0; TR1=0; ET0=0; out1=out2=out3=1; if(tb_num==0) hot_bit=1;} 

		else {time=hot_num*14; if(time>time_max)	time=time_max;  time_inset(); }

		while(key_buf==dn_key) WDT_CONTR=0X34;
		
	   }


   
   if(key_buf==mode_key)  
    {
	 t_cnt=150;
			    
      while((key_buf==mode_key)&&(t_cnt>0)) {t_cnt--; Delay1ms(10);}
	  if(t_cnt==0)  
			      {
				   speak=sp_bit=0;

				   if(tb_num==0) {tb_num=1; tb_dis(1);}

				   else          {tb_num=0; tb_dis(0);}

				   while(key_buf==mode_key) WDT_CONTR=0X34;

				  
				   save_mode_wr();

				   fan_bit=0;
				 
				   led1=led2=led3=1;
	               led4=led5=led6=1;

				   EX0=0; TR1=0; ET0=0;	 
				   out1=out2=out3=1;

				   hot_dis(0);  

				   fan_stop();  

				   goto  restar2;
				  }

	   else	{ speak=sp_bit=0; fan_bit=0; auto_bit=0; temp_set_bit=1; save_cnt=0; mode_num+=1; if(mode_num>5) mode_num=1; mode_chose(); save_mode_num(); while(key_buf==mode_key) WDT_CONTR=0X34; }
			 
	}

   
   
   
   if(key_buf==fd_key) 
     { 
			  t_cnt=150;
			    

		      while((key_buf==fd_key)&&(t_cnt>0)) {t_cnt--; Delay1ms(10);}

			  if(t_cnt==0) 
			      {
				   speak=sp_bit=0; 
				   temp_dis(1);
				   bf_dis(0);
				   dis1(1,temp_set);
				   temp_set_bit=1;
				   while(key_buf==fd_key) 
				     {
					   WDT_CONTR=0X34;

					   if(ss_bit) dis1(1,temp_set);  else dis1(0,temp_set);
					 }


				   while(1)
				     {
					  Delay1ms(30);

					  if(ss_bit) dis1(1,temp_set);  else dis1(0,temp_set);

					  if(key_buf==up_key)         {speak=sp_bit=0; temp_set+=1; if(temp_set>200) temp_set=200; dis1(1,temp_set); Delay1ms(150);}

   
                      else if(key_buf==dn_key)    {speak=sp_bit=0; temp_set-=1; if(temp_set<50) temp_set=50;   dis1(1,temp_set); Delay1ms(150);}


					  else if(key_buf==fd_key)	  {speak=sp_bit=0; while(key_buf==fd_key) WDT_CONTR=0X34; save_mode_wr(); break;}

					 }

				   
				   temp_set_bit=0;
				   save_bit=0;
				   temp_dis(0);
				   bf_dis(1);
				   dis1(1,hot_num);
					  
				  }
			  else {speak=sp_bit=0; temp_set_bit=1; save_cnt=0; fenduan_num++; if(fenduan_num>7) fenduan_num=1; hot_select(fenduan_num); while(key_buf==fd_key) WDT_CONTR=0X34;}
					
	  }
   
		  

  
   
 }
		   
   


}


 
	 
