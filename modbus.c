#include "modbus.h"
#include  "STC15W4Kxx.H"
#include "crc.h"

#define FUNCTION_READ_COILS_1                 1     //读线圈，功能码 1
#define FUNCTION_READ_DISCREATE_INPUT_2       2     //读离散量输入，功能码 2
#define FUNCTION_READ_HOLDING_REGISTERS_3     3     //读保持寄存器，功能码 3
#define FUNCTION_READ_INPUT_REGISTERS_4       4     //读输入寄存器，功能码4
#define FUNCTION_WRITE_SINGLE_COIL_5          5     //写单个线圈，功能码 5
#define FUNCTION_WRITE_SINGLE_REGISTER_6      6     //写单个寄存器，功能码 6
#define FUNCTION_WRITE_MULTIPLE_COILS_F       0xf   //写多个线圈，功能码 0xf
#define FUNCTION_WRITE_MULTIPLE_REGISTERS_10  0x10  //写多个寄存器,功能码 0x10
#define FUNCTION_READ_WRITE_MULTIPLE_REGISTERS_17  0x17  //读写多个寄存器


#define uchar  unsigned char
#define uint   unsigned int
#define ulong  unsigned long


extern  uint  idata  tm1;
extern  uchar idata  temp_set;


extern uchar  idata  fan_close_cnt;
extern uchar  idata  fenduan_num;
extern uchar idata power_run;


extern  uchar idata  fan_num;
extern  uchar idata  tb_num;
extern  uchar idata  hot_num;
extern  uchar idata  mode_num;

//extern  uchar   hot_power;
//extern  uchar   fan_power;
extern  uchar  idata hot_in_data;

extern bit save_all_bit;
extern bit hot_power_bit;
extern bit mode_num_bit;
extern bit tb_num_bit;
extern bit power_on_bit;
extern bit power_on_bit2;

extern unsigned char idata dev_address;   			    //设备地址
unsigned char xdata dev_broadcast_address = 0;  //设备广播地址

void  send_buffer(unsigned char *buf,int len);
//void  save_mode_wr(void);
//void  fan_close_save(void);

unsigned char xdata send_modbus_buffer[256];
int xdata send_modbus_buf_count = 0;
unsigned char xdata localbuf[256];
char xdata msg[64];




void send_bad_msg(unsigned char address,unsigned char function,unsigned char errorCode)
{
	unsigned char return_msg[5];
	unsigned short crc = 0;
	
	return_msg[0] = address;
	return_msg[1] = 0x80 + function;
	return_msg[2] = errorCode;
	crc = crc16(return_msg,3);
	return_msg[3] = crc;
	return_msg[4] = crc >> 8;
	send_buffer(return_msg,5);  //发送数据给主机
}

int broadcast_process(unsigned char *buf,int len)
{	 //广播地址
	buf = buf;
	len = len;
	//debug_out("broadcast_process\r\n\0");
	return 1;
}

 
 /*
int write_multiple_register(unsigned short start_address,unsigned short value,unsigned char *localbuf,unsigned char localbufLength)
{	 //写多保持寄存器
	unsigned char index = 0;
	sprintf(msg,"write_multiple_register()\r\n\0");
	//debug_out(msg);
	sprintf(msg,"start_address=%u,cnt=%u\r\n\0",start_address,value);
	//debug_out(msg);
	for (index = 0; index < localbufLength; index++) {
		sprintf(msg,"buf[%d]=0x%x, %d\r\n\0",(int)index,(int)localbuf[index],(int)localbuf[index]);
		//debug_out(msg);
	}
	return 1;
}

  /*
int write_multiple_coils(unsigned short start_address,unsigned short value,unsigned char *localbuf,unsigned char localbufLength)
{	 //写多个线圈
	unsigned char index = 0;
	sprintf(msg,"write_multiple_coils()\r\n\0");
	//debug_out(msg);
	sprintf(msg,"start_address=%u,cnt=%u\r\n\0",start_address,value);
	//debug_out(msg);
	for (index = 0; index < localbufLength; index++) {
		sprintf(msg,"buf[%d]=0x%x, %d\r\n\0",(int)index,(int)localbuf[index],(int)localbuf[index]);
		//debug_out(msg);
	}
	return 1;
}

 /*
int write_single_register(unsigned short address,unsigned short value)
{	  //写单个寄存器
	save_mode_num();
	return 1;
} */


  /*
int write_single_coil(unsigned short address,unsigned short value)
{		//写单线圈寄存器
	sprintf(msg,"write_single_coil()\r\n\0");
	//debug_out(msg);
	sprintf(msg,"start_address=%u,value=%u\r\n\0",address,value);
	//debug_out(msg);
	return 1;
}
  	   */

//读线圈，功能码 1
int function_READ_COILS_1(unsigned char *buf) 
{
	int ret = 0;
	unsigned char out_count = 0;       //输出字节数
	unsigned short start_address = 0;  //起始地址
	unsigned short count = 0;          //输出数量
	unsigned short crc  = 0;
	int send_total_count = 0;
	
	
	//输入的起始地址和输出的数量
	start_address  = buf[2] << 8;
	start_address += buf[3];
	count  = buf[4] << 8;
	count += buf[5];
	//sprintf(msg,"addr=%u,count=%u\r\n\0",start_address,count);
	//debug_out(msg);
	
	//数量是否有效，如果无效 则发送异常码 3
	if (count < 1 || count > 0x07D0) {
		send_bad_msg(buf[0],buf[1],3);   //发送 异常码 3 
		ret = 1;
    return ret;		
	}
	
	//地址是否ok,地址+输出数量是否ok,否则发送 异常码 2
	
	//读取线圈是否ok,否则发送 异常码 4
	out_count = count / 8;
	if (count % 8 > 0) out_count++;
	send_modbus_buf_count = 0;
	send_modbus_buffer[0] = buf[0];   //地址
	send_modbus_buffer[1] = buf[1];   //功能
	send_modbus_buffer[2] = out_count;  //输出字节数
	while (out_count--) {
	  send_modbus_buffer[3+send_modbus_buf_count] = P5; //线圈值
		send_modbus_buf_count++;
  }		
	send_modbus_buffer[2] = send_modbus_buf_count;	 //获取 字节数
	send_total_count = send_modbus_buf_count + 5;    //总的要发送的字节数
	crc = crc16(send_modbus_buffer,send_total_count-2);  //计算crc
	send_modbus_buffer[send_total_count-2] = crc; 
	send_modbus_buffer[send_total_count-1] = crc >> 8;
	send_buffer(send_modbus_buffer,send_total_count);  //发送给主机
	
	
	return ret;
}

//读离散量输入，功能码 2
int function_READ_DISCREATE_INPUT_2(unsigned char *buf)
{
  int ret = 0;
	unsigned char out_count = 0;       //输出字节数
	unsigned short start_address = 0;  //起始地址
	unsigned short count = 0;          //输出数量
	unsigned short crc  = 0;
	int send_total_count = 0;
	
	//输入的起始地址和输出的数量
	start_address  = buf[2] << 8;
	start_address += buf[3];
	count  = buf[4] << 8;
	count += buf[5];
	//sprintf(msg,"addr=%u,count=%u\r\n\0",start_address,count);
	//debug_out(msg);
	
	//数量是否有效，如果无效 则发送异常码 3
	if (count < 1 || count > 0x07D0) {
		send_bad_msg(buf[0],buf[1],3);   //发送 异常码 3 
		ret = 1;
    return ret;		
	}
	
	//地址是否ok,地址+输出数量是否ok,否则发送 异常码 2
	
	//读取线圈是否ok,否则发送 异常码 4
	out_count = count / 8;
	if (count % 8 > 0) out_count++;
	send_modbus_buf_count = 0;
	send_modbus_buffer[0] = buf[0];   //地址
	send_modbus_buffer[1] = buf[1];   //功能
	send_modbus_buffer[2] = out_count;  //输出字节数
	while (out_count--) {
	  send_modbus_buffer[3+send_modbus_buf_count] = P5; //线圈值
		send_modbus_buf_count++;
  }		
	send_modbus_buffer[2] = send_modbus_buf_count;	 //获取 字节数
	send_total_count = send_modbus_buf_count + 5;    //总的要发送的字节数
	crc = crc16(send_modbus_buffer,send_total_count-2);  //计算crc
	send_modbus_buffer[send_total_count-2] = crc; 
	send_modbus_buffer[send_total_count-1] = crc >> 8;
	send_buffer(send_modbus_buffer,send_total_count);  //发送给主机
	
	//debug_out("function_READ_DISCREATE_INPUT_2\r\n\0");
	return ret;
}

//读保持寄存器，功能码 3
int function_READ_HOLDING_REGISTERS_3(unsigned char *buf)
{
  int ret = 0;
	unsigned char out_count = 0;       //输出字节数
	unsigned short start_address = 0;  //起始地址
	unsigned short count = 0;          //输出数量
	unsigned short crc  = 0;
	int send_total_count = 0;
	
	//输入的起始地址和输出的数量
	start_address  = buf[2] << 8;
	start_address += buf[3];
	count  = buf[4] << 8;
	count += buf[5];
	count-=3;
	
	
	//数量是否有效，如果无效 则发送异常码 3
	if (count < 1 || count > 0x07D) {
		send_bad_msg(buf[0],buf[1],3);   //发送 异常码 3 
		ret = 1;
    return ret;		
	}
	
	//地址是否ok,地址+输出数量是否ok,否则发送 异常码 2
	
	//读取寄存器是否ok,否则发送 异常码 4
	out_count = count / 8;
	if (count % 8 > 0) out_count++;
	send_modbus_buf_count = 0;
	send_modbus_buffer[0] = buf[0];   //地址
	send_modbus_buffer[1] = buf[1];   //功能
	send_modbus_buffer[2] = out_count;  //输出字节数
    if(start_address==40001)
	 {
	  while (count--) {
	  
	  send_modbus_buffer[3+send_modbus_buf_count] =mode_num;    send_modbus_buf_count++; //模式
	  send_modbus_buffer[3+send_modbus_buf_count] =fenduan_num; send_modbus_buf_count++; //分段

	  send_modbus_buffer[3+send_modbus_buf_count] =tb_num;   send_modbus_buf_count++; //同步
	  send_modbus_buffer[3+send_modbus_buf_count] =fan_num;  send_modbus_buf_count++; //风力
	  send_modbus_buffer[3+send_modbus_buf_count] =hot_num;  send_modbus_buf_count++; //加热程度
	  send_modbus_buffer[3+send_modbus_buf_count] =power_run; send_modbus_buf_count++; //是否运行状态

	  send_modbus_buffer[3+send_modbus_buf_count] =(uchar)tm1;    send_modbus_buf_count++; //温度实际值
	  send_modbus_buffer[3+send_modbus_buf_count] =fan_close_cnt; send_modbus_buf_count++; //风扇延时时间
	  } 
	  send_modbus_buffer[2] = send_modbus_buf_count;	//重新计算获得 内容的字节数
	  send_total_count = send_modbus_buf_count + 5;   //发送的总长度 字节数
	  crc = crc16(send_modbus_buffer,send_total_count-2);  //计算crc校验和
	  send_modbus_buffer[send_total_count-2] = crc; 
	  send_modbus_buffer[send_total_count-1] = crc >> 8;
	  send_buffer(send_modbus_buffer,send_total_count);  //发送给主机
	  ret = 2;
	 }

	else { send_bad_msg(buf[0],buf[1],2); ret = 1;}  //地址是否ok, 否则发送 异常码 2
	return ret;
}

//读输入寄存器，功能码4
int function_READ_INPUT_REGISTERS_4(unsigned char *buf)
{
	int ret = 0;
	unsigned char out_count = 0;       //输出字节数
	unsigned short start_address = 0;  //起始地址
	unsigned short count = 0;          //输出数量
	unsigned short crc  = 0;
	int send_total_count = 0;
	
	//输入的起始地址和输出的数量
	start_address  = buf[2] << 8;
	start_address += buf[3];
	count  = buf[4] << 8;
	count += buf[5];
	count-=3;
	
	
	//数量是否有效，如果无效 则发送异常码 3
	if (count < 1 || count > 0x07D) {
		send_bad_msg(buf[0],buf[1],3);   //发送 异常码 3 
		ret = 1;
    return ret;		
	}
	
	//地址是否ok,地址+输出数量是否ok,否则发送 异常码 2
	
	//读取输入寄存器是否ok,否则发送 异常码 4
	
	send_modbus_buf_count = 0;
	send_modbus_buffer[0] = buf[0];   //地址
	send_modbus_buffer[1] = buf[1];   //功能
	send_modbus_buffer[2] = out_count;  //输出字节数
	if(start_address==40001)
	 {															
	  while (count--) {

	  send_modbus_buffer[3+send_modbus_buf_count] =mode_num;    send_modbus_buf_count++; //模式
	  send_modbus_buffer[3+send_modbus_buf_count] =fenduan_num; send_modbus_buf_count++; //分段

	  send_modbus_buffer[3+send_modbus_buf_count] =tb_num;   send_modbus_buf_count++; //同步
	  send_modbus_buffer[3+send_modbus_buf_count] =fan_num;  send_modbus_buf_count++; //风力
	  send_modbus_buffer[3+send_modbus_buf_count] =hot_num;  send_modbus_buf_count++; //加热程度
	  send_modbus_buffer[3+send_modbus_buf_count] =temp_set; send_modbus_buf_count++; //温度设定值

	  send_modbus_buffer[3+send_modbus_buf_count] =(uchar)tm1;    send_modbus_buf_count++; //温度实际值
	  send_modbus_buffer[3+send_modbus_buf_count] =fan_close_cnt; send_modbus_buf_count++; //风扇延时时间
	  } 
	  send_modbus_buffer[2] = send_modbus_buf_count;	//重新计算获得 内容的字节数
	  send_total_count = send_modbus_buf_count + 5;   //发送的总长度 字节数
	  crc = crc16(send_modbus_buffer,send_total_count-2);  //计算crc校验和
	  send_modbus_buffer[send_total_count-2] = crc; 
	  send_modbus_buffer[send_total_count-1] = crc >> 8;
	  send_buffer(send_modbus_buffer,send_total_count);  //发送给主机
	 } 
	

	else { send_bad_msg(buf[0],buf[1],2); ret = 1;}  //地址是否ok, 否则发送 异常码 2
	return ret;
}

//写单个线圈，功能码 5
int function_WRITE_SINGLE_COIL_5(unsigned char *buf,int len)
{
	int ret = 0;
	unsigned short start_address = 0;  //起始地址
	unsigned short value = 0;          //输出量
	
	//输入的起始地址和输出的数量
	start_address  = buf[2] << 8;
	start_address += buf[3];
	value  = buf[4] << 8;
	value += buf[5];
	
	
	//数量是否有效，如果无效 则发送异常码 3
	if (value != 0x0 && value != 0xFF00) {
		send_bad_msg(buf[0],buf[1],3);   //发送 异常码 3 
		ret = 1;
    return ret;		
	}
      //这里可以添加执行的数据或命令

	//if((start_address==40001)&&(value==0))	    P27=0;
	//if((start_address==40001)&&(value==0xff00))	P27=1;

	
	//地址是否ok, 否则发送 异常码 2
	

    send_buffer(buf,len);            //发送 正确数据给主机   	
	
	
	return ret;
}

//写单个寄存器，功能码 6
int function_WRITE_SINGLE_REGISTER_6(unsigned char *buf,int len)
{
  int ret = 0;
	unsigned short start_address = 0;  //起始地址
	unsigned short value = 0;          //输出量
	
	
	//输入的起始地址和输出的数量
	start_address  = buf[2] << 8;
	start_address += buf[3];
	value  = buf[4] << 8;
	value += buf[5];
	
	
	
	if (value < 0x0 || value > 0xFFFF) {
		send_bad_msg(buf[0],buf[1],3);  
		ret = 1;
    return ret;		
	}
	if(start_address==42005)	 
	  {
	   if((buf[5]>0)&&(buf[5]<5)) {
	     mode_num=buf[5];
		 mode_num_bit=1;
	     //save_mode_num();
	     //read_data();
	     send_buffer(buf,len);          
		 ret = 2;
		}
	  } 
	else if(start_address==42006)	 
	  {
	     tb_num=buf[5];
		 tb_num_bit=1;
	     send_buffer(buf,len);            
		 ret = 2;
		}  
		
	else if(start_address==42007)	 
	  {
	     if(buf[5]==0x01) power_on_bit=1;
		 
	     send_buffer(buf,len);           
		 ret = 2;
		}

    else if(start_address==42008)	 
	  {
	     if(buf[5]==0x02) power_on_bit2=1;
		 
	     send_buffer(buf,len);            
		 ret = 2;
		}			 	
	
	return ret;
}










int function_WRITE_MULTIPLE_COILS_F(unsigned char *buf)
{
  int ret = 0;
	unsigned short crc = 0;
	unsigned short start_address = 0; 
	unsigned short count = 0;          	
	unsigned char localbufLength = 0;
	unsigned char index = 0;
	
	
	//输入的起始地址和输出的数量
	start_address  = buf[2] << 8;
	start_address += buf[3];
	count  = buf[4] << 8;
	count += buf[5];

	
	
	if (count < 0x1 || count > 0x7B0) {
		send_bad_msg(buf[0],buf[1],3);   
		ret = 1;
    return ret;		
	}
	
	
	
	localbufLength = buf[6];
	for (index = 0; index < localbufLength; index++) {
    localbuf[index] = buf[7+index];		
	}
	
	
		crc = crc16(buf,6);
		buf[6] = crc;
		buf[7] = crc >> 8;
		send_buffer(buf,8);             	
	return ret;
}








int function_WRITE_MULTIPLE_REGISTERS_10(unsigned char *buf)
{
  int ret = 0;
	unsigned short crc = 0;
	unsigned short start_address = 0;  
	unsigned short count = 0;          	
	unsigned char localbufLength = 0;
	unsigned char index = 0;
	
	

	start_address  = buf[2] << 8;
	start_address += buf[3];
	count  = buf[4] << 8;
	count += buf[5];

	
	
	if (count < 0x1 || count > 0x7B) {
		send_bad_msg(buf[0],buf[1],3);  
		ret = 1;
    return ret;		
	}

     //地址是否ok, 否则发送 异常码 2
    if(start_address!=41000) {send_bad_msg(buf[0],buf[1],2);   
	   ret = 1;
       return ret;
	  }
	
	   
	   fenduan_num=buf[7]; 
	   fan_num=buf[8];	   
	   hot_num=buf[9];	   
	   temp_set=buf[10];   
	   fan_close_cnt=buf[11]; 


		crc = crc16(buf,6);
		buf[6] = crc;
		buf[7] = crc >> 8;
		send_buffer(buf,8);             
			
		save_all_bit=1;
		ret = 2;
	return ret;
}








int function_READ_WRITE_MULTIPLE_REGISTERS_17(unsigned char *buf)
{
  int ret = 0;
	unsigned short crc = 0;
	unsigned short read_start_address = 0;  
	unsigned short write_start_address = 0;  
	unsigned short readCount = 0;          	
	unsigned char writebytes = 0;
	unsigned short writeCount = 0;          	
	unsigned char localbufLength = 0;
	unsigned char index = 0;
	int send_total_count = 0;
	
	
	//输入的起始地址和输出的数量
	read_start_address  = buf[2] << 8;
	read_start_address += buf[3];
	readCount  = buf[4] << 8;
	readCount += buf[5];
	
	write_start_address  = buf[6] << 8;
	write_start_address += buf[7];
	writeCount  = buf[8] << 8;
	writeCount += buf[9];
	
	writebytes = buf[10];
	 
	
	if (readCount < 0x1 || readCount > 0x7D) {
		send_bad_msg(buf[0],buf[1],3);   
		ret = 1;
    return ret;		
	}
	if (writeCount < 0x1 || writeCount > 0x79) {
		send_bad_msg(buf[0],buf[1],3);  
		ret = 1;
    return ret;		
	}
	
	
	
	localbufLength = writebytes;
	for (index = 0; index < localbufLength; index++) {
    localbuf[index] = buf[11+index];		
	}
	
	
	  send_modbus_buf_count = 0;
	  send_modbus_buffer[0] = buf[0];   
	  send_modbus_buffer[1] = buf[1];   
	  send_modbus_buffer[2] = readCount;  
	  while (readCount--) {
	      send_modbus_buffer[3+send_modbus_buf_count] = P0; 
		  send_modbus_buf_count++;
		  send_modbus_buffer[3+send_modbus_buf_count] = P0; 
		  send_modbus_buf_count++;
         }		
	  send_modbus_buffer[2] = send_modbus_buf_count;	
	  send_total_count = send_modbus_buf_count + 5;   
	  crc = crc16(send_modbus_buffer,send_total_count-2);  
	  send_modbus_buffer[send_total_count-2] = crc; 
	  send_modbus_buffer[send_total_count-1] = crc >> 8;
	  send_buffer(send_modbus_buffer,send_total_count);  		
		
	
	return ret;
}




int not_support_function_code(unsigned char address,unsigned char function)
{
	int ret = 0;	
	send_bad_msg(address,function,1);
	
	return ret;
}




int parse_recv_buffer(unsigned char *buf,int len)
{	
	int ret = 0;
	unsigned short temp_crc = 0;
	unsigned short temp_crc2 = 0;
	
	if (len < 4)     return 0;        
	if (buf == NULL) return 0;
							   
	if (buf[0] != dev_address && buf[0] != dev_broadcast_address) {  
		ret = 1;
		return ret;
	}
	temp_crc  = buf[len-1] << 8;
	temp_crc += buf[len-2];
	temp_crc2 = crc16(buf,len-2);  

	if (temp_crc != temp_crc2) {  
		
		ret = 1;
		return ret;
	}
	
	if (buf[0] == 0) {   
		return broadcast_process(buf,len);
	   }
	
	
	switch (buf[1]) {
		case FUNCTION_READ_COILS_1                 : ret = function_READ_COILS_1(buf); break;   			  
		case FUNCTION_READ_DISCREATE_INPUT_2       : ret = function_READ_DISCREATE_INPUT_2(buf);   break;       
		case FUNCTION_READ_HOLDING_REGISTERS_3     : ret = function_READ_HOLDING_REGISTERS_3(buf); break;     
		case FUNCTION_READ_INPUT_REGISTERS_4       : ret = function_READ_INPUT_REGISTERS_4(buf);   break;       
		case FUNCTION_WRITE_SINGLE_COIL_5          : ret = function_WRITE_SINGLE_COIL_5(buf,len);      break;          
		case FUNCTION_WRITE_SINGLE_REGISTER_6      : ret = function_WRITE_SINGLE_REGISTER_6(buf,len);  break;      
		case FUNCTION_WRITE_MULTIPLE_COILS_F       : ret = function_WRITE_MULTIPLE_COILS_F(buf);    break;       
		case FUNCTION_WRITE_MULTIPLE_REGISTERS_10  : ret = function_WRITE_MULTIPLE_REGISTERS_10(buf); break;  
		case FUNCTION_READ_WRITE_MULTIPLE_REGISTERS_17  : ret = function_READ_WRITE_MULTIPLE_REGISTERS_17(buf); break; 	
		default: ret = not_support_function_code(buf[0],buf[1]);        
	}	
	
	return ret;
}
