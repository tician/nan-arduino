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
#include <sensor_msgs/Imu.h>
//#include <std_msgs/String.h>

#define TWI_FREQ 400000L
#include <Wire.h>

#include "acc.h"
#include "bar.h"
#include "gyr.h"
#include "mag.h"

ros::NodeHandle nh;

/*
 * This is a message to hold data from an IMU (Inertial Measurement Unit)
 *
 * Accelerations should be in m/s^2 (not in g's), and rotational velocity
 *   should be in rad/sec.
 * If the covariance of the measurement is known, it should be filled in
 *  (if all you know is the variance of each measurement, e.g. from the
 *   datasheet, just put those along the diagonal)
 * A covariance matrix of all zeros will be interpreted as "covariance unknown",
 *   and to use the data a covariance will have to be assumed or gotten from
 *   some other source
 * If you have no estimate for one of the data elements (e.g. your IMU doesn't
 *   produce an orientation estimate), please set element 0 of the associated
 *   covariance matrix to -1.
 * If you are interpreting this message, please check for a value of -1 in
 *   the first element of each covariance matrix, and disregard the associated
 *   estimate.
 *
 * Header header
 *     uint32 seq
 *     time stamp
 *     string frame_id
 * geometry_msgs/Quaternion orientation
 *     float64 x
 *     float64 y
 *     float64 z
 *     float64 w
 * float64[9] orientation_covariance
 * geometry_msgs/Vector3 angular_velocity
 *     float64 x
 *     float64 y
 *     float64 z
 * float64[9] angular_velocity_covariance
 * geometry_msgs/Vector3 linear_acceleration
 *     float64 x
 *     float64 y
 *     float64 z
 * float64[9] linear_acceleration_covariance
 */
sensor_msgs::Imu imu_msg;
ros::Publisher nutter("emu", &imu_msg);
geometry_msgs::Quaternion ori_;
geometry_msgs::Vector3 ang_;
geometry_msgs::Vector3 lin_;

float ori_cov_[9] =
{
	0,0,0,
	0,0,0,
	0,0,0
};
float ang_cov_[9] =
{
	0,0,0,
	0,0,0,
	0,0,0
};
float lin_cov_[9] =
{
	0,0,0,
	0,0,0,
	0,0,0
};



char frame_id_[] = "emu_center";

//std_msgs::String str_msg;
//ros::Publisher chatter("chatter", &str_msg);
//char str[19] = "I'm still alive...";

void getCovariances(void)
{

}

void setup()
{
	nh.initNode();
	nh.advertise(nutter);
//	nh.advertise(chatter);

	getCovariances();

	Wire.begin();


}

void loop()
{

	barfitup(&curr_nun);
	int16_t temp;

	imu_msg.header.stamp = nh.now();
	imu_msg.header.frame_id = frame_id_;



	imu_msg.orientation.x = ori_[0];
	imu_msg.orientation.y = ori_[1];
	imu_msg.orientation.z = ori_[2];
	imu_msg.orientation.w = ori_[3];




	nutter.publish(&imu_msg);

//	char str_out[30] = {};

//	temp = curr_nun.sx - 133;
//	sprintf(str_out, "sx:%d", temp);

//	str_msg.data = str_out;
//	chatter.publish(&str_msg);

	nh.spinOnce();
	delay(1000);
}
