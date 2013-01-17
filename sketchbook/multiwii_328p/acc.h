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
		Wire.requestFrom(ACC_ADDR_XH, 6);
		x = (Wire.read())<<8);
		x += (Wire.read()&0xFF);
		y = (Wire.read()<<8);
		y += (Wire.read()&0xFF);
		z = (Wire.read()<<8);
		z += (Wire.read()&0xFF);

//#define ACC_ORIENTATION(X, Y, Z) {accADC[ROLL]  = -X; accADC[PITCH]  = -Y; accADC[YAW]  =  Z;}
		acc_.x = -x;
		acc_.y = -y;
		acc_.z = z;
	}
}

#endif

