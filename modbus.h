#ifndef __MOD_BUS_H
#define __MOD_BUS_H

#include <stdio.h>
#include <stdlib.h>




//�����յ������ݣ�������
int parse_recv_buffer(unsigned char *buf,int len);

void read_data(void);
void save_mode_num(void);
void hot_power_save(void);
void fan_power_save(void);

#endif
