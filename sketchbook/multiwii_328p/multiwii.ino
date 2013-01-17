/*
 *******************************************************************************
 *  multiwii - rosserial Publisher for HK MultiWii 328p clone
 *******************************************************************************
 *  
 *******************************************************************************
 *  LEGAL STUFF
 *******************************************************************************
 *  Copyright (c) 2012, 2013 Matthew Paulishen. All rights reserved.
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



// HobbyKing MultiWii ITG3205 + BMA180 + BMP085 + HMC5883L port
//   http://www.hobbyking.com/hobbyking/store/uh_viewItem.asp?idProduct=27033
#include <ros.h>
#define TWI_FREQ 400000L
#include <Wire.h>

#include "acc.h"
#include "bar.h"
#include "gyr.h"
#include "mag.h"

ros::NodeHandle nh;

sensor_msgs::Joy joy_msg;
ros::Publisher nutter("joy", &joy_msg);
float joy_axes[20] = {};

//std_msgs::String str_msg;
//ros::Publisher chatter("chatter", &str_msg);
//char str[19] = "I'm still alive...";

nunchuck_t curr_nun;

void setup()
{
	nh.initNode();
	nh.advertise(nutter);
//	nh.advertise(chatter);
	upchuck();

	for (int i=4; i<16; i++)
	{
		joy_axes[i] = 1.0;
	}

	joy_axes[19] = 0.0;
}

void loop()
{
	barfitup(&curr_nun);
	int16_t temp;

	joy_msg.header.stamp = nh.now();
	joy_msg.axes_length = 20;

	// Left Stick: +L -R
	temp = curr_nun.sx - 133;
	joy_axes[0] = (float) (temp/-128.0);
	// Left Stick: +F -B
	temp = curr_nun.sy - 134;
	joy_axes[1] = (float) (temp/128.0);
	// Right Stick: +L -R
//	joy_axes[2] = 0.0;
	// Right Stick: +F -B
//	joy_axes[3] = 0.0;

	if (curr_nun.bz)
		joy_axes[8] = -1.0;
	else
		joy_axes[8] = 1.0;
	if (curr_nun.bc)
		joy_axes[10] = -1.0;
	else
		joy_axes[10] = 1.0;
	
	// ACCEL: +L -R
	temp = curr_nun.ax - 512;
	joy_axes[16] = (float) temp/512.0;
	// ACCEL: +F -B
	temp = curr_nun.ay - 512;
	joy_axes[17] = (float) temp/-512.0;
	// ACCEL: +U -D
	temp = curr_nun.az - 512;
	joy_axes[18] = (float) temp/512.0;

	joy_msg.axes = joy_axes;
	
	nutter.publish(&joy_msg);


//	char str_out[30] = {};
	
//	temp = curr_nun.sx - 133;
//	sprintf(str_out, "sx:%d", temp);

//	str_msg.data = str_out;
//	chatter.publish(&str_msg);

	nh.spinOnce();
	delay(1000);
}
