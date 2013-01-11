/*
 *******************************************************************************
 *  flyingnun.h
 *******************************************************************************
 *  A header file of questionable quality to control a WiiNunchuck
 *  
 *******************************************************************************
 *  LEGAL STUFF
 *******************************************************************************
 *  Copyright (c) 2011, 2012 Matthew Paulishen. All rights reserved.
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

#include <Wire.h>

struct nunchuck_t
{
	uint16_t ax, ay, az;
	uint8_t sx, sy;
	uint8_t bc, bz;
};

uint8_t upchuck(void)
{
	Wire.begin();
// Start
	// Wiimote extension address: 0x52
	Wire.beginTransmission(0x52);
// Unencrypted data startup sequence
	// Set register:	0x(4)A400F0
	Wire.write(0xF0);
	// Write value:		0x55
	Wire.write(0x55);
	// Set register:	0x(4)A400FB
	Wire.write(0xFB);
	// Write value:		0x00
	Wire.write(0x00);
// Stop
	Wire.endTransmission();

	return 0;
}

uint8_t barfitup(nunchuck_t* nun)
{
// Start
	// Wiimote extension address: 0x52
	Wire.beginTransmission(0x52);
// Unencrypted data request sequence
	// Set register:	0x(4)A40000
	Wire.write(0x00);
// Stop
	Wire.endTransmission();


// Start
	// Wiimote extension address: 0x52
	Wire.beginTransmission(0x52);
// Unencrypted data receive sequence
	Wire.requestFrom (0x52, 6);
	// Read byte at: 0x(4)A40000
	nun->sx = Wire.read();
	// Read byte at: 0x(4)A40001
	nun->sy = Wire.read();
	// Read byte at: 0x(4)A40002
	nun->ax = (Wire.read()<<2);
	// Read byte at: 0x(4)A40003
	nun->ay = (Wire.read()<<2);
	// Read byte at: 0x(4)A40004
	nun->az = (Wire.read()<<2);
	// Read byte at: 0x(4)A40005
	uint8_t temp = Wire.read();
// Stop
//	Wire.endTransmission();

	temp ^= 0xFF;
	nun->bz = temp&0x01;	// bit 0
	temp >>= 1;
	nun->bc = temp&0x01;	// bit 1
	temp >>= 1;
	temp ^= 0xFF;
	nun->ax += temp&0x03;	// bit 2,3
	temp >>= 2;
	nun->ay += temp&0x03;	// bit 4,5
	temp >>= 2;
	nun->az += temp&0x03;	// bit 6,7

	return 0;
}
