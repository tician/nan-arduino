/*
 *******************************************************************************
 *  BMP085 Barometer
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

#ifndef BAROMETER_H_
#define BAROMETER_H_

#include "Wire.h"
#include "sensor.h"

volatile sensor_t bar_;

// BMP085
#define BAR_ADDR			0xF4
// Uncompensated temperature and pressure acquisition sequence
// StartTemp(write(0x2E to 0xF4)) -> wait(4.5ms) -> GetTemp(read(0xF6 0xF7)) -> StartPress(write(0x34+(oss<<6) to 0xF4)) -> wait(?) -> GetPress(read(0xF6 0xF7)) (Big-Endian)


#endif

