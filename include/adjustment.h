#pragma once
#include "Naes.h"

#ifndef ADJUSTMENT_H
#define ADJUSTMENT_H

/*
need to slow down the wheel adjustment speeds. This can be done with a counter that activates every predetermined number of rotations
through the callback functions.
*/


class Adjustment {
	public:
		//Default Constructor
		Adjustment(){
			/*PUBLICATIONS*/
			motorsPub = nh.advertise<naes::wheel_speeds>("ROStoArduino", 10);
			stopPub = nh.advertise<naes::stop>("stop", 1);

			/*SUBSCRIPTIONS*/
			currentLocationSub = nh.subscribe<naes::current_position>("/currentPosition", 2, &Adjustment::adjustmentCallback, this);}


		void publish_brakes(){
			motorsPub.publish(brakes);}
		void publish_forward(){
			motorsPub.publish(forward);
			left_speed = normal;
			right_speed = normal;}
		void publish_rotate_left(){
			motorsPub.publish(rotate_left);
			static_turn = true;}
		void publish_rotate_right(){
			motorsPub.publish(rotate_right);
			static_turn = true;}
		void come_out_of_static_turn(){
			motorsPub.publish(forward);
			left_speed = normal;
			right_speed = normal;
			static_turn = false;}
		void publish_custom(){
//			cout << "left_speed, right_speed is: " << left_speed << ", " << right_speed << endl;
			custom.left_wheel = left_speed;
			custom.right_wheel = right_speed;
			motorsPub.publish(custom);}
		void make_left_adjustment(){
			cout << "future_offset is: " << future_offset << ", going slightly more left" << endl;
			if(count < wheel_change){
				count = count + 1;}
			else{
				count = 0;
				left_speed = left_speed - adjustment_constant;
                        	right_speed = right_speed + adjustment_constant;
				publish_custom();}
		}
		void make_right_adjustment(){
			cout << "future_offset is: " << future_offset << ", going slightly more right" << endl;
			if(count < wheel_change){
				count = count + 1;}
			else{
				count = 0;
	                        left_speed = left_speed + adjustment_constant;
	                        right_speed = right_speed - adjustment_constant;
				publish_custom();}
		}
		void publish_origin_turn(){
			left_speed = 20;
			right_speed = -20;
			publish_custom();}

		void check_left_adjustment(){
			ratio = future_offset / distance;
			cout << "ADJUSTING LEFT...RATIO IS: " << ratio << endl;
			left_speed = normal - (ratio * adjustment_constant);
			if(left_speed < 10){
				left_speed = 10;}
			right_speed = normal + (normal - left_speed);


			publish_custom();}

/*
			if(ratio > (3/5)){
				left_speed = normal - 15; //was 6
				right_speed = normal + 15;} //was 6
			else if( (ratio <= (3/5)) && (ratio > (3/10)) ){
				left_speed = normal - 10; //was 2
				right_speed = normal + 10;} //was 2
			else if( (ratio <= (3/10)) && (ratio > (3/15)) ){
				left_speed = normal - 5; //was 1
				right_speed = normal + 5;} //was 1
			else{	
				left_speed = normal;
				right_speed = normal;}
			publish_custom();
		}
*/

		void check_right_adjustment(){
			ratio = future_offset / distance;
			cout << "ADJUSTING RIGHT...RATIO IS: " << ratio << endl;
			left_speed = normal + (ratio * adjustment_constant);
				if(left_speed > 90){
					left_speed = 90;}
			right_speed = normal + (normal - left_speed);
			publish_custom();}

/*
			if(ratio > (3/5)){
				left_speed = normal + 15;
				right_speed = normal - 15;}
			else if( (ratio <= (3/5)) && (ratio > (3/10)) ){
				left_speed = normal + 10;
				right_speed = normal - 10;}
			else if( (ratio <= (3/10)) && (ratio > (3/15)) ){
				left_speed = normal + 5;
				right_speed = normal - 5;}
			else{	
				left_speed = normal;
				right_speed = normal;}
			publish_custom();
		}
*/
















		//Mutator Functions
		void calculate_delta_phi_and_offset(){
			if( (current_phi > 0) && ((current_phi - phi_to_next) > M_PI)) { //going across border going left
				adjusted_current_phi = current_phi;
				adjusted_phi_to_next = phi_to_next + (2*M_PI);}
			else if( (current_phi <= 0) && ((phi_to_next - current_phi) > M_PI) ){ //going across border going right
				adjusted_phi_to_next = phi_to_next;
				adjusted_current_phi = current_phi + (2*M_PI);}
			else { //not going across border
				adjusted_phi_to_next = phi_to_next;
				adjusted_current_phi = current_phi;}
			delta_phi = adjusted_phi_to_next - adjusted_current_phi;
			future_offset = abs( distance * sin(delta_phi));}

/*		void set_plotted_xs(vector<float> xvalues){
			plotted_xs = xvalues;}
		void set_plotted_ys(vector<float> yvalues){
			plotted_ys = yvalues;}
		void set_plotted_phis(vector<float> phivalues){
			plotted_phis = phivalues;}
*/

		void currentPositionCallback(const naes::current_position::ConstPtr& msg1) {
                        current_x1 = msg1->x;
                        current_y1 = msg1->y;
                        current_phi1 = msg1->phi;}

		float get_current_phi1(){
			return current_phi1;}

		/* SET FUNCTIONS FOR NODE CONSTANTS */
		void set_max_offset(float offsetvalue){
			max_offset = offsetvalue;}
		void set_adjustment_constant(int adjustmentconstval){
			adjustment_constant = adjustmentconstval;}
		void set_desired_radius(float radiusvalue){
			desired_radius = radiusvalue;}
		void set_turn_value(float turnvalue){
			turn = turnvalue;}
		void set_iteration_to_zero(){
			iteration = 0;
			count = 0;}
		void set_static_turn_to_false(){
			static_turn = false;}
		void set_origin_state(bool value){
			origin = value;}
		void set_origin_rotate_value(float rvalue){
			origin_rotate_value = rvalue;}
		void set_wheel_change_value(int wvalue){
			wheel_change = wvalue;}
		void set_normal_wheel_value(int normwheelval){
			normal = normwheelval;
			forward.left_wheel = normwheelval;
			forward.right_wheel = normwheelval;
			rotate_left.left_wheel = -normwheelval;
			rotate_left.right_wheel = normwheelval;
			rotate_right.left_wheel = normwheelval;
			rotate_right.right_wheel = -normwheelval;
			brakes.left_wheel = 0;
			brakes.right_wheel = 0;
			check = 34;
			check_count = 0;}

		/* PUSHING BAG FILE INFORMATIONS */
		void pushx(float x){
			plotted_xs.push_back(x);}
		void pushy(float y){
			plotted_ys.push_back(y);}
		void pushphi(float angle){
			plotted_phis.push_back(angle);}
		void set_navigation_size(){
			size = plotted_xs.size();}

		//Callback Functions
		void adjustmentCallback(const naes::current_position::ConstPtr& msg){
				current_x = msg->x;
                                current_y = msg->y;
                                current_phi = msg->phi;
			if( iteration < size ){
				next_x = plotted_xs[iteration];
	        		next_y = plotted_ys[iteration];
        			next_phi = plotted_phis[iteration];
/*				cout << "current_x is: " << current_x << endl;
				cout << "current_y is: " << current_y << endl;
				cout << "current_phi is: " << current_phi << endl;
                                cout << "next_x is: " << next_x << endl;
				cout << "next_y is: " << next_y << endl;
                                cout << "next_phi is: " << next_phi << endl;
*/
				distance = sqrt( ( (next_x - current_x)*(next_x - current_x)) + ( (next_y - current_y)*(next_y - current_y)));
				if( (next_x < current_x) && (next_y > current_y) ){				//second quadrant
		                	phi_to_next = atan((next_y - current_y)/(next_x - current_x)) + M_PI;}
				else if( (next_x < current_x) && (next_y <= current_y) ){			//third quadrant
		                	phi_to_next = atan((next_y - current_y)/(next_x - current_x)) - M_PI;}
				else {										//first or fourth quadrant
		                	phi_to_next = atan((next_y - current_y)/(next_x - current_x));}
				if( distance <= desired_radius){	//It has reached it's desired destination, move to next desired point
					if(check != 0){
						cout << "repeat value: " << check_count << endl;
						cout << "RESULT 0 CALLED" << endl;
						check_count = 1;
						check = 0;
						iteration = iteration + 1;}
					else{
					check_count = check_count + 1;
                			iteration = iteration + 1;}
				}
				else{
					calculate_delta_phi_and_offset();
					if( current_phi > 0 ) {
						if( (current_phi - phi_to_next) > M_PI) { //go accross border going left
							if(delta_phi > turn) { //need to rotate left
								if(check != 1){
									cout << "repeat value: " << check_count << endl;
									cout << "RESULT 1 CALLED" << endl;
									check_count = 1;
									check = 1;
									publish_rotate_left();}
								else{
									check_count = check_count + 1;
									publish_rotate_left();}
							}
							else { //continue path or make wheel adjustments
								if(static_turn == true) { //If it just came out of rotation state, go straight
									cout << "RESULT 2(static turn) CALLED" << endl;
									come_out_of_static_turn();}
								else{
									if(check != 3){
										cout << "repeat value: " << check_count << endl;
										cout << "RESULT 3 CALLED" << endl;
										check_count = 1;
										check = 3;
										check_left_adjustment();}
									else{
										check_count = check_count + 1;
										check_left_adjustment();}

/*									if( future_offset > max_offset ){ //needs to go slightly more left
										cout << "RESULT 3 CALLED" << endl;
										make_left_adjustment();}
									if( future_offset <= max_offset){
										cout << "RESULT 3.5 CALLED" << endl;
										publish_forward();}
*/
								}
							}
						}
						else { //doesnt need to go across the border
							if(delta_phi > turn) { //need to rotate left
								if(check != 4){
									cout << "repeat value: " << check_count << endl;
									cout << "RESULT 4 CALLED" << endl;
									check_count = 1;
									check = 4;
					        	        	publish_rotate_left();}
								else{
									check_count = check_count + 1;
									publish_rotate_left();}
							}




					        	else if(delta_phi < (-1*turn)) { //need to rotate right
								if(check != 5){
									cout << "repeat value: " << check_count << endl;
									cout << "RESULT 5 CALLED" << endl;
									check_count = 1;
									check = 5;
						        	        publish_rotate_right();}
								else{
									check_count = check_count + 1;
									publish_rotate_right();}
							}




					        	else { //continue path or make wheel adjustments
					        	        if(static_turn == true) {
									cout << "RESULT 6(static turn) CALLED" << endl;
					        	      		come_out_of_static_turn();}
								else{
									if(delta_phi > 0){//need to check adjustment left
										if(check != 7){
											cout << "repeat value: " << check_count << endl;
											cout << "RESULT 7 CALLED" << endl;
											check_count = 1;
											check = 7;
											check_left_adjustment();}
										else{
											check_count = check_count + 1;
											check_left_adjustment();}
									}
									else{
										if(check != 8){
											cout << "repeat value: " << check_count << endl;
											cout << "RESULT 8 CALLED" << endl;
											check_count = 1;
											check = 8;
											check_right_adjustment();}
										else{
											check_count = check_count + 1;
											check_right_adjustment();}
									}


/*
	                						if( future_offset > max_offset ){
										if(delta_phi > 0){ //need to adjust wheels to go more left
											cout << "RESULT 7 CALLED" << endl;
											make_left_adjustment();}
	                	                				else{
											cout << "RESULT 8 CALLED" << endl;
											make_right_adjustment();}
									}
									if(future_offset <= max_offset ){
										cout << "RESULTS 8.5 CALLED" << endl;
										publish_forward();}
*/


								}

							}
						}
					}
					else { //current_phi <= 0
						if( (phi_to_next - current_phi) > M_PI) { //go accross border going right
							if(delta_phi < (-1*turn)) { //need to rotate right
								if(check != 9){
									cout << "repeat value: " << check_count << endl;
									cout << "RESULT 9 CALLED" << endl;
									check_count = 1;
									check = 9;
									publish_rotate_right();}
								else{
									check_count = check_count + 1;
									publish_rotate_right();}
							}



							else { //continue path or make wheel adjustments
								if(static_turn == true) {
									cout << "RESULT 10(static turn) CALLED" << endl;
									come_out_of_static_turn();}
								else{
									if(check != 11){
										cout << "repeat value: " << check_count << endl;
										cout << "RESULT 11 CALLED" << endl;
										check_count = 1;
										check = 11;
										check_right_adjustment();}
									else{
										check_count = check_count + 1;
										check_right_adjustment();}

/*
									if( future_offset > max_offset ){
										cout << "RESULT 11 CALLED" << endl;
										make_right_adjustment();}
									if(future_offset <= max_offset){
										cout << "RESULTS 11.5 CALLED" << endl;
										publish_forward();}
*/
								}
							}
						}
						else { //doesnt need to go across the border
							if(delta_phi > turn) { //need to rotate left
								if(check != 12){
									cout << "repeat value: " << check_count << endl;
									cout << "RESULT 12 CALLED" << endl;
									check_count = 1;
									check = 12;
		                					publish_rotate_left();}
								else{
									check_count = check_count +1;
									publish_rotate_left();}

							}
	        					else if(delta_phi < (-1*turn)) { //need to rotate right
								if(check != 13){
									cout << "repeat value: " << check_count << endl;
									cout << "RESULT 13 CALLED" << endl;
									check_count = 1;
									check = 13;
		                					publish_rotate_right();}
								else{
									check_count = check_count +1;
									publish_rotate_right();}
							}
	        					else { //continue path or make wheel adjustments
	                					if(static_turn == true) {
									cout << "RESULT 14(static turn) CALLED" << endl;
	                						come_out_of_static_turn();}
								else{
									if(delta_phi > 0){
										if(check != 15){
											cout << "repeat value: " << check_count << endl;
											cout << "RESULT 15 CALLED" << endl;
											check_count = 1;
											check = 15;
											check_left_adjustment();}
										else{
											check_count = check_count +1;
											check_left_adjustment();}
									}
									else{
										if(check != 16){
											cout << "repeat value: " << check_count << endl;
											cout << "RESULT 16 CALLED" << endl;
											check_count = 1;
											check = 16;
											check_right_adjustment();}
										else{
											check_count = check_count +1;
											check_right_adjustment();}
									}
/*
		                	      				if( future_offset > max_offset ){
		                	        	       			if(delta_phi > 0){ //need to adjust wheels to go more left
											cout << "RESULT 15 CALLED" << endl;
											make_left_adjustment();}
		                	        	        		else{
											cout << "RESULT 16 CALLED" << endl;
											make_right_adjustment();}
		                	      				}
									if(future_offset <= max_offset){
										cout << "RESULT 16.5 CALLED" << endl;
										publish_forward();}
*/
								}
							}
						}
					}
				}
			}
			else{ //at the last point
				cout << "current_phi is: " << current_phi << endl;
				if(origin == false){
					motorsPub.publish(brakes);
					cout << "THE ROBOT HAS FINISHED NAVIGATION!!!" << endl;
					stopmsg.stop = "stop";
					stopPub.publish(stopmsg);}
				if(origin == true){
					origin_check = abs(current_phi - (M_PI/2));
					cout << "origin_check is: " << origin_check << endl;
					if(origin_check >= origin_rotate_value){
						cout << "THE ROBOT IS AT ORIGIN AND NOW ROTATING" << endl;
						publish_origin_turn();}
					else{
						motorsPub.publish(brakes);
						cout << "THE ROBOT IS BACK AT THE ORIGIN!!!" << endl;
						stopmsg.stop = "stop";
						stopPub.publish(stopmsg);}
				}
			}
		}

	private:
		ros::NodeHandle nh;
		ros::Publisher motorsPub;
		ros::Publisher stopPub;
		ros::Subscriber currentLocationSub;
		naes::wheel_speeds motors, brakes, forward, rotate_left, rotate_right, custom;
		naes::stop stopmsg;

		//Member Variables
		vector<float> plotted_xs;
		vector<float> plotted_ys;
		vector<float> plotted_phis;
		float current_x, current_y, current_phi, next_x, next_y, next_phi;
		float max_offset;
		float desired_radius;
		float turn;
		float distance;
		float phi_to_next;
		float adjusted_current_phi, adjusted_phi_to_next, delta_phi, future_offset;
		float current_x1, current_y1, current_phi1;
		float origin_rotate_value;
		float origin_check;
		float ratio;
		int check, check_count;
		int adjustment_constant;
		int iteration;
		int normal;
		int size;
		int left_speed, right_speed;
		int wheel_change;
		int count;
		bool origin;
		bool static_turn;



};
#endif
