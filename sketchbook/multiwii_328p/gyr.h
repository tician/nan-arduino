/*
 *******************************************************************************
 *  ITG-3200 MEMS Gyroscope
 *******************************************************************************
 *  
 *******************************************************************************
 *  LEGAL STUFF
 *******************************************************************************
 *  Copyright (c) 2013 Matthew Paulishen. All rights reserved.
 *  
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *  
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *  
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *  
 *******************************************************************************
 */

#ifndef GYROSCOPE_H_

#include "Wire.h"
#include "sensor.h"

volatile sensor_t gyr_;

// ITG-3200
#define GYR_ADDR			0x68

// WHO_AM_I (0x00)
//  7: reserved
//  6: I2C Address (_1)
//  5: I2C Address (_1)
//  4: I2C Address (_0)
//  3: I2C Address (_1)
//  2: I2C Address (_0)
//  1: I2C Address (_0)
//  0: reserved
#define GYR_ADDR_0			0x00

// Sample Rate Divider (0x15)
// F_sample = F_internal/(divider+1), F_internal = 1kHz/8kHz
//  7: (_0~255)
//  6: (_0~255)
//  5: (_0~255)
//  4: (_0~255)
//  3: (_0~255)
//  2: (_0~255)
//  1: (_0~255)
//  0: (_0~255)
#define GYR_ADDR_1			0x15
#define GYR_CONF_1	((0<<7)|(0<<6)|(0<<5)|(0<<4)|(0<<3)|(0<<2)|(0<<1)|(0<<0))

// DLPF, Full Scale (0x16)
//  7: reserved
//  6: reserved
//  5: reserved
//  4: Full-Scale (0,1,2,_3) +/-2000 [deg/s]
//  3: Full-Scale (0,1,2,_3)
//  2: Digital Low-Pass Filter (_256/8,188/1,98/1,42/1,20/1,10/1,5/1,reserved) Hz/kHz
//  1: Digital Low-Pass Filter (_256/8,188/1,98/1,42/1,20/1,10/1,5/1,reserved)
//  0: Digital Low-Pass Filter (_256/8,188/1,98/1,42/1,20/1,10/1,5/1,reserved)
#define GYR_ADDR_2			0x16
#define GYR_CONF_2	((0<<7)|(0<<6)|(0<<5)|(1<<4)|(1<<3)|(0<<2)|(0<<1)|(0<<0))

// Interrupt Configuration (0x17)
//  7: ACTL (_active-high,active-low)
//  6: OPEN (_push-pull,open-drain)
//  5: LATCH (_50uspulse,latch)
//  4: CLEAR (_status,any)
//  3: reserved
//  2: I_RDY (_disable,enable)
//  1: reserved
//  0: I_RAW (_disable,enable)
#define GYR_ADDR_3			0x17
#define GYR_CONF_3	((0<<7)|(0<<6)|(0<<5)|(0<<4)|(0<<3)|(0<<2)|(0<<1)|(0<<0))

// Interrupt Status (0x1A)
//  7: reserved
//  6: reserved
//  5: reserved
//  4: reserved
//  3: reserved
//  2: ITG_RDY	
//  1: reserved
//  0: RAW_RDY	
#define GYR_ADDR_I			0x1A

// Data Register Temp-H (0x1B)
#define GYR_ADDR_TH			0x1B
// Data Register Temp-L (0x1C)
#define GYR_ADDR_TL			0x1C
// Data Register XH (0x1D)
#define GYR_ADDR_XH			0x1D
// Data Register XL (0x1E)
#define GYR_ADDR_XL			0x1E
// Data Register YH (0x1F)
#define GYR_ADDR_YH			0x1F
// Data Register YL (0x20)
#define GYR_ADDR_YL			0x20
// Data Register ZH (0x21)
#define GYR_ADDR_ZH			0x21
// Data Register ZL (0x22)
#define GYR_ADDR_ZL			0x22

// Power Management (0x3E)
//  7: Reset (_none,reset)
//  6: Sleep (_normal,sleep)
//  5: Stdby_x (_normal,standby)
//  4: Stdby_y (_normal,standby)
//  3: Stdby_z (_normal,standby)
//  2: Clk	(_Internal,PLL_X,PLL_Y,PLL_Z,PLL_E36,PLL_E19,reserved,reserved)
//  1: Clk	(_Internal,PLL_X,PLL_Y,PLL_Z,PLL_E36,PLL_E19,reserved,reserved)
//  0: Clk	(_Internal,PLL_X,PLL_Y,PLL_Z,PLL_E36,PLL_E19,reserved,reserved)
#define GYR_ADDR_P			0x3E
#define GYR_CONF_P	((0<<7)|(0<<6)|(0<<5)|(0<<4)|(0<<3)|(0<<2)|(0<<1)|(1<<0))


void gyrStart(void)
{
	Wire.beginTransmission(GYR_ADDR);
	Wire.write(GYR_ADDR_1);
	Wire.write(GYR_CONF_1);
	Wire.write(GYR_ADDR_2);
	Wire.write(GYR_CONF_2);
	Wire.write(GYR_ADDR_3);
	Wire.write(GYR_CONF_3);
	Wire.write(GYR_ADDR_P);
	Wire.write(GYR_CONF_P);
	Wire.endTransmission();
}

void gyrGet(void)
{
	int16_t x=0,y=0,z=0;
	Wire.beginTransmission(GYR_ADDR);
	Wire.requestFrom(GYR_ADDR_XH, 6);
	x = (Wire.read())<<8);
	x += (Wire.read()&0xFF);
	y = (Wire.read()<<8);
	y += (Wire.read()&0xFF);
	z = (Wire.read()<<8);
	z += (Wire.read()&0xFF);

//#define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] =  Y; gyroADC[PITCH] = -X; gyroADC[YAW] = -Z;}
	gyr_.x = y;
	gyr_.y = -x;
	gyr_.z = -z;
}

#endif

