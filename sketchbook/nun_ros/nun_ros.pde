/*
 *******************************************************************************
 *  nun_ros - rosserial Publisher for Wii Nunchuck
 *******************************************************************************
 *  matches ps3joy output format
 *     +0 => left (sx<133); -0 => right (sx>133)
 *     +1 => forward (sy>134); -1 => backward (sy<134)
 *     8 => R2=Z: 1 at no press, -1 at full press
 *     10 => R1=C: 1 at no press, -1 at full press
 *     +16 => left accel (ax>512); -16 => right accel (ax<512)
 *     +17 => forward accel (ay<512); -17 => backward accel (ay>512)
 *     +18 => up accel (az>512); -18 => down accel (az<512)
 *  
 *******************************************************************************
 *  LEGAL STUFF
 *******************************************************************************
 *  Copyright (c) 2012 Matthew Paulishen. All rights reserved.
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

#include <ros.h>
//#include <std_msgs/String.h>
#include <sensor_msgs/Joy.h>
#include <Wire.h>
#include <flyingnun.h>

ros::NodeHandle  nh;

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
