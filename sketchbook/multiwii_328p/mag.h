/*
 *******************************************************************************
 *  HMC5883 Magnetometer
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

#ifndef MAGNETOMETER_H_
#define MAGNETOMETER_H_

#include "Wire.h"
#include "sensor.h"

volatile sensor_t mag_;

// HMC5883
#define MAG_ADDR			0x3C

// Configuration Register A (0x00)
//  7: must be cleared
//  6: samples averaged per output (1,2,4,_8)
//  5: samples averaged per output (1,2,4,_8)
//  4: output rate (0.75,1.5,3,7.5,_15,30,75,N/A)
//  3: output rate (0.75,1.5,3,7.5,_15,30,75,N/A)
//  2: output rate (0.75,1.5,3,7.5,_15,30,75,N/A)
//  1: offset mode (_none, +bias, -bias, reserved)
//  0: offset mode (_none, +bias, -bias, reserved)
#define MAG_ADDR_A			0x00
#define MAG_CONF_A	((0<<7)|(1<<6)|(1<<5)|(1<<4)|(1<<3)|(1<<2)|(0<<1)|(0<<0))

// Configuration Register B (0x01)
//  7: gain (1370,_1090,820,660,440,390,330,230) [LSB/Gauss]
//  6: gain (1370,_1090,820,660,440,390,330,230) [LSB/Gauss]
//  5: gain (1370,_1090,820,660,440,390,330,230) [LSB/Gauss]
//  4: must be cleared
//  3: must be cleared
//  2: must be cleared
//  1: must be cleared
//  0: must be cleared
#define MAG_ADDR_B			0x01
#define MAG_CONF_B	((0<<7)|(0<<6)|(1<<5)|(0<<4)|(0<<3)|(0<<2)|(0<<1)|(0<<0))

// Mode Register (0x02)
//  7: must be cleared
//  6: must be cleared
//  5: must be cleared
//  4: must be cleared
//  3: must be cleared
//  2: must be cleared
//  1: sampling mode (continuous,_single,idle,idle)
//  0: sampling mode (continuous,_single,idle,idle)
#define MAG_ADDR_M			0x02
#define MAG_CONF_M	((0<<7)|(0<<6)|(0<<5)|(0<<4)|(0<<3)|(0<<2)|(0<<1)|(0<<0))

// Data Register X-MSB (0x03)
#define MAG_ADDR_XH			0x03
// Data Register X-LSB (0x04)
#define MAG_ADDR_XL			0x04
// Data Register Y-MSB (0x05)
#define MAG_ADDR_YH			0x05
// Data Register Y-LSB (0x06)
#define MAG_ADDR_YL			0x06
// Data Register Z-MSB (0x07)
#define MAG_ADDR_ZH			0x07
// Data Register Z-LSB (0x08)
#define MAG_ADDR_ZL			0x08

// Status Register (0x09)
//  7: reserved
//  6: reserved
//  5: reserved
//  4: reserved
//  3: reserved
//  2: reserved
//  1: lock (not finished writing new data to registers)
//  0: ready (new data ready to be read out)
#define MAG_ADDR_S			0x09


// Identification Register A (0x0A) [0x48:'H']
#define MAG_ADDR_IA			0x0A
// Identification Register B (0x0B) [0x34:'4']
#define MAG_ADDR_IB			0x0B
// Identification Register C (0x0C) [0x33:'3']
#define MAG_ADDR_IC			0x0C

// read: 0x3D 0x03 0x06 (six octets from 0x3D starting at register 3) (Big-Endian)

void magStart(void)
{
	Wire.beginTransmission(MAG_ADDR);
	Wire.write(MAG_ADDR_A);
	Wire.write(MAG_CONF_A);
	Wire.write(MAG_CONF_B);
	Wire.write(MAG_CONF_M);
	Wire.endTransmission();
}

void magGet(void)
{
	Wire.beginTransmission(MAG_ADDR);
	Wire.requestFrom(MAG_ADDR_S, 1);
	if (Wire.read()&(1<<0))
	{
		int16_t x=0,y=0,z=0;
		Wire.requestFrom(MAG_ADDR_XH, 6);
		x = (Wire.read())<<8);
		x += (Wire.read()&0xFF);
		y = (Wire.read()<<8);
		y += (Wire.read()&0xFF);
		z = (Wire.read()<<8);
		z += (Wire.read()&0xFF);

//#define MAG_ORIENTATION(X, Y, Z) {magADC[ROLL]  =  X; magADC[PITCH]  =  Y; magADC[YAW]  = Z;}
		mag_.x = x;
		mag_.y = y;
		mag_.z = z;
	}
}

#endif

