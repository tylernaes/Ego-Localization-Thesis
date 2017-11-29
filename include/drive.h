#pragma once
#include "Naes.h"
#include "localization.h"

#ifndef DRIVE_H
#define DRIVE_H

class Drive {
	public:
		Drive(){
			/*PUBLICATIONS*/
			wheelsPub = nh.advertise<naes::wheel_speeds>("ROStoArduino", 1);
			stopPub = nh.advertise<naes::stop>("stop", 1);

			/*SUBSCRIPTIONS*/
			currentLocationSub = nh.subscribe<naes::current_position>("/currentPosition", 2, &Drive::driveCallback, this);

		}

		//MUTATOR
		void set_initial_phi(float value){
			initial_phi = value;}
		void set_final_phi(float value){
			straight.left_wheel = 50;
			straight.right_wheel = 50;
			left_turn.left_wheel = 25;
			left_turn.right_wheel = 75;
			right_turn.left_wheel = 75;
			right_turn.right_wheel = 25;
			slight_left.left_wheel = 48;
			slight_left.right_wheel = 52;
			slight_right.left_wheel = 52;
			slight_right.right_wheel = 48;
			brake.right_wheel = 0;
			brake.left_wheel = 0;
			stopmsg.stop = "stop";
			final_phi = value;}

		void set_turning_left_false(){
			stopCall = false;
			turning_left = false;}
		void set_turning_right_false(){
			stopCall = false;
			turning_right = false;}
		void set_straight_after_false(){
			straight_after = false;}
		void set_straight_before_true(){
			straight_before = true;}

		void set_after_count_to_zero(){
			after_count = 0;}

		void set_left_wheel_value(int left){
			left_wheel = left;}

		void set_right_wheel_value(int right){
			right_wheel = right;}

		void set_duration(int dur){
			duration = dur;}

		void set_stopmark_false(){
			stopmark = false;}

		bool get_stopmark(){
			return stopmark;}

		void publishing(){
			straight.left_wheel = left_wheel;
			straight.right_wheel = right_wheel;
			wheelsPub.publish(straight);}

		//Accessor


		void driveCallback(const naes::current_position::ConstPtr& position){
			current_phi = position->phi;
			current_x = position->x;
			current_y = position->y;
			if(stopCall == true){
				stopPub.publish(stopmsg);
				straight_after = false;}
			if(straight_before == true){
				if(current_y >= 0.40){
					straight_before = false;
					turning_right = true;}
				else{
					wheelsPub.publish(straight);}
			}
			else if(turning_right == true){
				if(current_phi > 0){
					wheelsPub.publish(left_turn);}
				else{
					last_y = current_y;
					wheelsPub.publish(straight);
					turning_right = false;
					straight_after = true;}
			}
			else if(straight_after == true){
				if(current_y >= last_y){
					if(after_count < 24){
						wheelsPub.publish(slight_left);}
					after_count = after_count + 1;
				}
				else{
					if(after_count < 24){
						wheelsPub.publish(slight_right);}
					after_count = after_count + 1;
				}
				if(after_count >= 25){
					wheelsPub.publish(brake);

					stopCall = true;}
			}
		}


	private:
		ros::NodeHandle nh;
		ros::Publisher wheelsPub;
		ros::Publisher stopPub;
		ros::Subscriber currentLocationSub;
		naes::wheel_speeds motors, brake, forward, rotate_left, rotate_right, straight, custom, left_turn, right_turn, slight_left, slight_right;
		naes::stop stopmsg;
		float initial_phi, final_phi, current_phi, current_x, current_y, last_y;
		bool turning_left, turning_right, straight_after, straight_before, stopCall, stopmark;
		int after_count, left_wheel, right_wheel, duration;

};
#endif
