#pragma once
#include "Naes.h"

#ifndef ORIENTATION_H
#define ORIENTATION_H

class Orientation {
	public:
		//Default Constructor
		Orientation();

		//Deconstructor
		~Orientation();

		//Accessor Functions
		void zeroingCallback(const sensor_msgs::Imu::ConstPtr& msg) {
			q0 = msg->orientation.w;
			q1 = msg->orientation.x;
			q2 = msg->orientation.y;
			q3 = msg->orientation.z;

			t0 = +2.0 * (q0 * q1 + q2 * q3);
			t1 = +1.0 - 2.0 * (q1 * q1 + q2 * q2);
			roll_raw = std::atan2(t0, t1);
			t2 = +2.0 * (q0 * q2 - q3 * q1);
			t2 = ((t2 > 1.0) ? 1.0 : t2);
			t2 = ((t2 < -1.0) ? -1.0 : t2);
			pitch_raw = std::asin(t2);
			t3 = +2.0 * (q0 * q3 + q1 * q2);
			t4 = +1.0 - 2.0 * (q2 * q2 + q3 * q3);
			yaw_raw = std::atan2(t3, t4);
			if(yaw_raw >= M_PI) {
				yaw = yaw_raw - (2 * M_PI);
			}
			else if(yaw_raw < -M_PI){
				yaw = yaw_raw + (2 * M_PI);
			}
			else{
				yaw = yaw_raw;
			}
		}


		void orientationCallback(const sensor_msgs::Imu::ConstPtr& msg) {
			q0 = msg->orientation.w;
			q1 = msg->orientation.x;
			q2 = msg->orientation.y;
			q3 = msg->orientation.z;

			t0 = +2.0 * (q0 * q1 + q2 * q3);
			t1 = +1.0 - 2.0 * (q1 * q1 + q2 * q2);
			roll_raw = std::atan2(t0, t1);
			t2 = +2.0 * (q0 * q2 - q3 * q1);
			t2 = ((t2 > 1.0) ? 1.0 : t2);
			t2 = ((t2 < -1.0) ? -1.0 : t2);
			pitch_raw = std::asin(t2);
			t3 = +2.0 * (q0 * q3 + q1 * q2);
			t4 = +1.0 - 2.0 * (q2 * q2 + q3 * q3);
			yaw_raw = std::atan2(t3, t4);
			if(yaw_raw >= M_PI) {
				yaw = yaw_raw - (2 * M_PI);}
			else if(yaw_raw < -M_PI){
				yaw = yaw_raw + (2 * M_PI);}
			else{
				yaw = yaw_raw;}


			/******GETTING YAW_ADJUSTED********/		
			if(right_adjustment == true){
				if( (yaw + offset) < (-1 * M_PI) ){ //adjustment going across the border going right
					yaw_adjusted = (yaw + offset) + (2 * M_PI);}
				else{
					yaw_adjusted = yaw + offset;}
			}
			else{ //left_adjust == true
				if( (yaw + offset) >= M_PI ){
					yaw_adjusted = yaw + offset - (2 * M_PI);}
				else{
					yaw_adjusted = yaw + offset;}
			}




		}//end of orientationCallback

		float get_roll(){
			return roll;}
		float get_pitch(){
			return pitch;}
		float get_yaw(){
			return yaw;}
		float get_yaw_adjusted(){
			return yaw_adjusted;}
		float get_roll_raw(){
                        return roll_raw;}
                float get_pitch_raw(){
                        return pitch_raw;}
                float get_yaw_raw(){
                        return yaw_raw;}
		bool get_end_state(){
			return end;}

		//Mutator Functions
		void set_offset(float value){
			left_adjustment = false;
			right_adjustment = false;
//			offset = M_PI - value;
			if( (value > (M_PI/2)) || (value <= (-1 * (M_PI/2))) ){
				right_adjustment = true;
				if( value > (M_PI/2)){
					offset = (M_PI/2) - value;}
				else{
					offset = (M_PI/2) - (value + (2 * M_PI));}
			}
			else{
				left_adjustment = true;
				offset = M_PI - value;}
		}

		void set_end_to_false(){
			end = false;}


	private:
		//Member Variables
		float q0, q1, q2, q3, t0, t1, t2, t3, t4;
		float roll, pitch, yaw, roll_raw, pitch_raw, yaw_raw, yaw_adjusted;
		float offset;
		bool left_adjustment, right_adjustment, end;

};

Orientation::Orientation(){
}

Orientation::~Orientation(){
}

#endif
