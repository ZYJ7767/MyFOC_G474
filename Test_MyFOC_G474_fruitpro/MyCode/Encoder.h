#ifndef __ENCODE_H
#define __ENCODE_H

#include "stdint.h"
#include "tim.h"
#include "Foc_Function.h"
#include "main.h"

uint16_t Encode_get_e_theta(uint16_t value);
float Encode_get_speed(uint32_t value, uint8_t ms, uint16_t *encode_cnt);








#endif

