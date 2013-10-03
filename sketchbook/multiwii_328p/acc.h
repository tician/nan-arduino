/*
 *******************************************************************************
 *  BMA180 Accelerometer
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

#ifndef ACCELEROMETER_H_
#define ACCELEROMETER_H_

#include "Wire.h"
#include "sensor.h"

volatile sensor_t acc_;

// BMA180
#define ACC_ADDR			0x40

// 
//  7: 
//  6: 
//  5: 
//  4: 
//  3: 
//  2: 
//  1: 
//  0: 

// Data Register XL (0x02)
#define ACC_ADDR_XL			0x02
// Data Register XH (0x03)
#define ACC_ADDR_XH			0x03
// Data Register YL (0x04)
#define ACC_ADDR_YL			0x04
// Data Register YH (0x05)
#define ACC_ADDR_YH			0x05
// Data Register ZL (0x06)
#define ACC_ADDR_ZL			0x06
// Data Register ZH (0x07)
#define ACC_ADDR_ZH			0x07
// Data Register Temp (0x08)
#define ACC_ADDR_T			0x08

void accStart(void)
{
	
}

void accGet(void)
{
	Wire.beginTransmission(ACC_ADDR);
	Wire.requestFrom(ACC_ADDR_S, 1);
	if (Wire.read()&(1<<0))
	{
		int16_t x=0,y=0,z=0;
		Wire.requestFrom(ACC_ADDR_XL, 6);
		x = (Wire.read())>>2);
		x += (Wire.read()<<6);
		x += ((x&(1<<13))<<2);
		x &= ~(1<<13);
		y = (Wire.read()>>2);
		y += (Wire.read()<<6);
		y += ((y&(1<<13))<<2);
		y &= ~(1<<13);
		z = (Wire.read()>>2);
		z += (Wire.read()<<6);
		z += ((z&(1<<13))<<2);
		z &= ~(1<<13);

//#define ACC_ORIENTATION(X, Y, Z) {accADC[ROLL]  = -X; accADC[PITCH]  = -Y; accADC[YAW]  =  Z;}
		acc_.x = -x;
		acc_.y = -y;
		acc_.z = z;
	}
}

#endif

