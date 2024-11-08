#ifndef __MOD_BUS_H
#define __MOD_BUS_H

#include <stdio.h>
#include <stdlib.h>




//分析收到的数据，并处理
int parse_recv_buffer(unsigned char *buf,int len);

void read_data(void);
void save_mode_num(void);
void hot_power_save(void);
void fan_power_save(void);

#endif
