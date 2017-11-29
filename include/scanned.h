//save this as scanned.h
#pragma once
#include "Naes.h"
#include "localization.h"

#ifndef SCANNED_H
#define SCANNED_H

class Scanned {
	public:
		//Default Constructor
		Scanned(){
			/*PUBLICATIONS*/
			currentPositionPub = nh.advertise<naes::current_position>("/currentPosition", 1);
			stopPub = nh.advertise<naes::stop>("stop", 1);

			/*SUBSCRIPTIONS*/
			scanSub = nh.subscribe<sensor_msgs::LaserScan>("/scan", 2, &Scanned::scanCallback, this);}


		//Main Callback
		void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan){
			ego_points.clear();
			ego.clear();
			ego.resize(0);
			set_ego_size(scan->ranges.size());
			set_min_angle(scan->angle_min);
			set_angle_increment(scan->angle_increment);
			set_ego_points(scan->ranges);

			for(i=0; i < ego_size; i++){
				if(ego_points[i] < reject){ // Needs to be put in a group.
					angle = angle_min + (i * angle_increment);
					x = calcx(ego_points[i], angle);
					y = calcy(ego_points[i], angle);
					if(ego.size() == 0){ //This is the first group
						ego.resize(1);
						ego[0].pushx(x);
						ego[0].pushy(y);}
					else{
						size = ego.size();
						grouped = false;
						for(j=0; j<size; j++){
							test_x = ego[j].get_localization_x();
							test_y = ego[j].get_localization_y();
							abs_diff_x = abs(x-test_x);
							abs_diff_y = abs(y-test_y);
							if((abs_diff_x <= grouping_constant)&&(abs_diff_y <= grouping_constant)){
								ego[j].pushx(x);
								ego[j].pushy(y);
								grouped = true;}
						}
						if(grouped == false){ //need to make a new object in ego vector.
							ego.resize(size+1);
							ego[size].pushx(x);
							ego[size].pushy(y);}
					}
				}
			}
			o = ego.size();
			if(o == 3){
				toAverageX.clear();
				toAverageY.clear();
				for(k = 0; k < o; k++){
					toAverageX = ego[k].get_x_vector();
					toAverageY = ego[k].get_y_vector();
					m = toAverageX.size();
					n = toAverageY.size();
					xsum = 0.0;
					ysum = 0.0;
					for(kk = 0; kk < m; kk++){
						xsum = xsum + toAverageX[kk];}
					for(ll = 0; ll < n; ll++){
						ysum = ysum + toAverageY[ll];}
					x_final = xsum / m;
					y_final = ysum / n;
					phi_final = calc_average_phi(x_final, y_final);
					ego[k].clearxego();
					ego[k].clearyego();
					ego[k].clearphiego();
					ego[k].pushy(y_final);
					ego[k].pushx(x_final);
					ego[k].pushphi(phi_final);
					toAverageX.clear();
					toAverageY.clear();
					toAverageX.resize(1);
					toAverageY.resize(1);
				}
				//These are to now be rotated to appropriate ego_positions.
				x_0 = ego[0].get_localization_x();
				y_0 = ego[0].get_localization_y();
				x_1 = ego[1].get_localization_x();
				y_1 = ego[1].get_localization_y();
				x_2 = ego[2].get_localization_x();
				y_2 = ego[2].get_localization_y();

				/***************
				ROTATION SECTION
				***************/
				//need x__0..., dist calculations
				rotation = false;
				bad_rotation = false;
				rotate_count = 0;
				bad_reading_count = 0;
				while(rotation == false){
					bad_reading_count = 0;
					phi_0 = calc_phi(x_0, y_0);
					phi_1 = calc_phi(x_1, y_1);
					phi_2 = calc_phi(x_2, y_2);
					local_dist01 = sqrt(((x_1 - x_0)*(x_1 - x_0))+((y_1 - y_0)*(y_1 - y_0)));
				        local_dist02 = sqrt(((x_2 - x_0)*(x_2 - x_0))+((y_2 - y_0)*(y_2 - y_0)));
				        local_dist12 = sqrt(((x_2 - x_1)*(x_2 - x_1))+((y_2 - y_1)*(y_2 - y_1)));
					delta_dist01 = abs(dist01 - local_dist01);
					delta_dist02 = abs(dist02 - local_dist02);
					delta_dist12 = abs(dist12 - local_dist12);
					if(rotate_count > 5){
						rotation = true;
						cout << "TOO MANY ROTATIONS!!!!!!!" << endl;
						stop.stop = "stop";
						stopPub.publish(stop);}

					if((abs(dist01 - local_dist01)) >= offset){
						bad_reading_count = bad_reading_count + 1;}
					if((abs(dist02 - local_dist02)) >= offset){
	        	                        bad_reading_count = bad_reading_count + 1;}
					if((abs(dist12 - local_dist12)) >= offset){
	        	                        bad_reading_count = bad_reading_count + 1;}

					if(bad_reading_count > 1){
						rotate();
	        	                        rotate_count = rotate_count + 1;}

					if(bad_reading_count == 1){
						bad_rotation = true;
						rotation = true;}

					if(bad_reading_count == 0){
						rotation = true;}

				} //end of while(rotation == false);

				if(bad_rotation == false){	//NEED TO PERFORM TRIANGULATION AND PUBLISH HERE.
					r_0 = calc_radius(x_0, y_0);
					r_1 = calc_radius(x_1, y_1);
					r_2 = calc_radius(x_2, y_2);
/**********************************************************************************************************************************/

					dx = x__1 - x__0;
					dy = y__1 - y__0;

		                        //* Determine the straight?line distance between the centers. */
					d = sqrt((dy*dy) + (dx*dx));

					/* Check for solvability. need to add a buffer */
					if (d > (r_0 + r_1 + 0.05)){
						/* no solution. circles do not intersect. */
						cout << "no solution, circles do not intersect" << endl;}
					else if (d < abs(r_0 - r_1)){
						/* no solution. one circle is contained in the other */
						cout << "not solution, one circle is contained in the other" << endl;}
					else{ //SOLVABILITY ELSE STATEMENT
						/* 'point 2' is the point where the line through the circle
						* intersection points crosses the line between the circle
						* centers.
						*/

						/* Determine the distance from point 0 to point 2. */
						a = ((r_0*r_0) - (r_1*r_1) + (d*d)) / (2.0 * d) ;

						/* Determine the coordinates of point 2. */
						point2_x = x__0 + (dx * a/d);
						point2_y = y__0 + (dy * a/d);

						/* Determine the distance from point 2 to either of the
						* intersection points.
						*/
						h = sqrt((r_0*r_0) - (a*a));

						/* Now determine the offsets of the intersection points from
						* point 2.
						*/
						rx = -dy * (h/d);
						ry = dx * (h/d);

						/* Determine the absolute intersection points. */
						intersectionPoint1_x = point2_x + rx;
						intersectionPoint2_x = point2_x - rx;
						intersectionPoint1_y = point2_y + ry;
						intersectionPoint2_y = point2_y - ry;

						//Log.d("INTERSECTION Circle1 AND Circle2:", "(" + intersectionPoint1_x + "," +
						//intersectionPoint1_y + ")" + " AND (" + intersectionPoint2_x + "," + intersectionPoint2_y
						//+ ")");

						/* Lets determine if circle 3 intersects at either of the above intersection points.
						*/
						dx = intersectionPoint1_x - x__2;
						dy = intersectionPoint1_y - y__2;
						d1 = sqrt((dy*dy) + (dx*dx));
						dx = intersectionPoint2_x - x__2;
						dy = intersectionPoint2_y - y__2;
						d2 = sqrt((dy*dy) + (dx*dx));

						//HERE THE NEXX AND NEWY ARE FOUND

						if(abs(d1 - r_2) < 0.1) {
							newx = intersectionPoint1_x;
							newy = intersectionPoint1_y;}
						else if(abs(d2 - r_2) < 0.1) {
							newx = intersectionPoint2_x;
			                                newy = intersectionPoint2_y;}
						else {
							//cout << "INTERSECTION Circle1 AND Circle2 AND Circle3: NONE" << endl;
						}
						//HERE THE POSE IS FOUND
						if(newx >= x__0) {
							if(newy >= y__0) {//beta is in third quadrant
								beta0 = atan((y__0-newy)/(x__0-newx)) - (M_PI);
								if( ((M_PI)/2) + M_PI + beta0 + (M_PI - phi_0) < M_PI) {
									phi0 = ((M_PI)/2) + M_PI + beta0 + (M_PI - phi_0);}
								else{
									phi0 = ((M_PI)/2) + (M_PI + beta0) + (-M_PI - phi_0);}
							}
							else {//beta is in second quadrant
								beta0 = (M_PI)+ atan((y__0-newy)/(x__0-newx));
								if((beta0+((M_PI/2)-phi_0)) < M_PI) {
									phi0 = beta0+(M_PI/2)-phi_0;}
								else {
									phi0 = -M_PI - (M_PI - beta0) + (M_PI/2)-phi_0;}
							}
						}
						else {//beta is in first or fourth quadrant
							beta0 = atan((y__0-newy)/(x__0-newx));
							if(((-M_PI)/2) + beta0 + (M_PI - phi_0) < M_PI) {
								phi0 = ((-M_PI)/2) + beta0 + (M_PI - phi_0);}
							else {
								phi0 = ((-M_PI)/2) + beta0 + (-M_PI - phi_0);}
						}
						if(newx >= x__1) {
			                                if(newy >= y__1) {//beta is in third quadrant
			                                        beta1 = atan((y__1-newy)/(x__1-newx)) - (M_PI);
			                                        if( ((M_PI)/2) + M_PI + beta1 + (M_PI - phi_1) < M_PI) {
			                                                phi1 = ((M_PI)/2) + M_PI + beta1 + (M_PI - phi_1);}
			                                        else{
									phi1 = ((M_PI)/2) + (M_PI + beta1) + (-M_PI - phi_1);}
		                                	}
		                                	else {//beta is in second quadrant
		                                	        beta1 = (M_PI)+ atan((y__1-newy)/(x__1-newx));
		                                	        if((beta1+((M_PI/2)-phi_1)) < M_PI) {
		                                	                phi1 = beta1+(M_PI/2)-phi_1;}
		                                        	else {
									phi1 = -M_PI - (M_PI - beta1) + (M_PI/2)-phi_1;}
							}
			                        }
			                        else {//beta is in first or fourth quadrant
			                                beta1 = atan((y__1-newy)/(x__1-newx));
			                                if( ((-M_PI)/2) + beta1 + (M_PI - phi_1) < M_PI) {
			                                        phi1 = ((-M_PI)/2) + beta1 + (M_PI - phi_1);}
			                                else {
								phi1 = ((-M_PI)/2) + beta1 + (-M_PI - phi_1);}
	                        		}
						if(newx >= x__2) {
			                                if(newy >= y__2) {//beta is in third quadrant
			                                        beta2 = atan((y__2-newy)/(x__2-newx)) - (M_PI);
			                                        if( ((M_PI)/2) + M_PI + beta2 + (M_PI - phi_2) < M_PI) {
			                                                phi2 = ((M_PI)/2) + M_PI + beta2 + (M_PI - phi_2);}
	                                        		else{
									phi2 = ((M_PI)/2) + (M_PI + beta2) + (-M_PI - phi_2);}
	                                		}
	                                		else {//beta is in second quadrant
	                                        		beta2 = (M_PI)+ atan((y__2-newy)/(x__2-newx));
	                                        		if((beta2+((M_PI/2)-phi_2)) < M_PI) {
	                                                		phi2 = beta2+(M_PI/2)-phi_2;}
			                                        else {
									phi2 = -M_PI - (M_PI - beta2) + (M_PI/2)-phi_2;}
							}
			                        }
			                        else {//beta is in first or fourth quadrant
			                                beta2 = atan((y__2-newy)/(x__2-newx));
			                                if( ((-M_PI)/2) + beta2 + (M_PI - phi_2) < M_PI) {
			                                        phi2 = ((-M_PI)/2) + beta2 + (M_PI - phi_2);}
			                                else {
								phi2 = ((-M_PI)/2) + beta2 + (-M_PI - phi_2);}
						}
						if( (abs(phi0 - phi1) > 1) || (abs(phi1 -phi2) > 1) ) { //crossing over the pi mark
							if(phi0 < 0) {
								//cout << "phi0 crossed over the pi boundary. added 2*PI" << endl;
								phi0 = phi0 +(2* M_PI);}
							if(phi1 < 0) {
								//cout << "phi1 crossed over the pi boundary. added 2*PI" << endl;
			                                        phi1 = phi1 +(2* M_PI);}
							if(phi2 < 0) {
								//cout << "phi2 crossed over the pi boundary. added 2*PI" << endl;
			                                        phi2 = phi2 +(2* M_PI);}
							phiavg = (phi0 + phi1 + phi2) / 3;
							if(phiavg >= M_PI) {
								phiavg = phiavg - (2* M_PI);}
							newphi = phiavg;
						}
						else {
							newphi = (phi0 + phi1 + phi2) / 3;}
						coordinates.x = newx;
						coordinates.y = newy;
						coordinates.phi = newphi;
						currentPositionPub.publish(coordinates);
					} //end of solvability else statement
				} //end of if(bad_rotation == false). current position should be published by now.
				else{
//					cout << "bad_rotation == true so there is no published current position..." << endl;
				}
			} //end of if(o == 3);
			else{
				cout << "Number of Ego-Points detectedwas not 3, it was: " << o <<endl;}
		}//End of scanCallback


		//Mutator Functions

		void set_ego_size(int value){
			ego_size = value;
			ego_points.resize(value);}
		void set_min_angle(float value){
			angle_min = value;}
		void set_angle_increment(float value){
			angle_increment = value;}
		void set_ego_points(vector<float> points){
			ego_points = points;}
		void set_reject_value(float value){
			reject = value;}
		void set_grouping_constant_value(float value){
			grouping_constant = value;}

		float calcx(float rangevalue, float anglevalue){
		        float xvalue = rangevalue * cos(anglevalue);
		        return xvalue;}

		float calcy(float rangevalue, float anglevalue){
		        float yvalue = rangevalue * sin(anglevalue);
		        return yvalue;}

		float calc_average_phi(float xx, float yy){
			if(xx < 0 && yy > 0){// need to add pi
				average_phi = atan( (yy/xx) ) + M_PI;}
			else if( xx < 0 && yy <= 0){// need to subtract pi
				average_phi = atan( (yy/xx) ) - M_PI;}
			else{// regular tan inverse
				average_phi = atan( (yy/xx) );}
			return average_phi;
		}
		//Accessor Functions
		float get_current_phi(){
			return newphi;}

		/***************
		rotation section
		***************/
		void set_global_ego_points(float xx0, float yy0, float xx1, float yy1, float xx2, float yy2){
			x__0 = xx0;
			y__0 = yy0;
			x__1 = xx1;
			y__1 = yy1;
			x__2 = xx2;
			y__2 = yy2;

			dist01 = sqrt(((x__1 - x__0)*(x__1 - x__0))+((y__1 - y__0)*(y__1 - y__0)));
			dist02 = sqrt(((x__2 - x__0)*(x__2 - x__0))+((y__2 - y__0)*(y__2 - y__0)));
		        dist12 = sqrt(((x__2 - x__1)*(x__2 - x__1))+((y__2 - y__1)*(y__2 - y__1)));}

		void set_offset(float value){
			offset = value;}

		void rotate(){
			grabx0 = x_0;
			graby0 = y_0;
			grabx1 = x_1;
			graby1 = y_1;
                        grabx2 = x_2;
                        graby2 = y_2;

			x_0 = grabx1;
			y_0 = graby1;
			x_1 = grabx2;
			y_1 = graby2;
			x_2 = grabx0;
			y_2 = graby0;}

		float calc_phi(float xx, float yy){
			if(xx < 0 && yy > 0){// need to add pi
				phi = atan( (yy/xx) ) + M_PI;}
			else if( xx < 0 && yy <= 0){// need to subtract pi
				phi = atan( (yy/xx) ) - M_PI;}
			else{// regular tan inverse
				phi = atan( (yy/xx) );}
			return phi;
		}

		float calc_radius(float xx, float yy){
			radius = sqrt( (xx * xx) + (yy * yy) );
			return radius;}

		/***************
		triangulation section
		***************/
		//no functions to put here




	private:
		ros::NodeHandle nh;
		ros::Publisher currentPositionPub;
		ros::Publisher stopPub;
		ros::Subscriber scanSub;
		naes::stop stop;
		naes::current_position coordinates;

		//Member Variables
		/************************
		SCAN SECTION
		************************/
		vector<Localization> ego;
		vector<float> ego_points, toAverageX, toAverageY;
		float angle_increment, angle_min, reject, angle, grouping_constant;
		float x, y, test_x, test_y, abs_diff_x, abs_diff_y;
		float xsum, ysum, x_final, y_final, phi_final, average_phi;
		float x_0, y_0, phi_0, x_1, y_1, phi_1, x_2, y_2, phi_2;
		float x__0, y__0, x__1, y__1, x__2, y__2;
		int ego_size, size, scan_count;
		int i, j, m, n, o, k, kk, ll;
		bool grouped;

		/************************
		ROTATION SECTION
		************************/
		float dist01, dist02, dist12, local_dist01, local_dist02, local_dist12, delta_dist01, delta_dist02, delta_dist12;;
		float offset, phi, radius;
		float grabx0, graby0, grabx1, graby1, grabx2, graby2;
		float r_0, r_1, r_2;
		int rotate_count, bad_reading_count;
		bool rotation, bad_rotation;

		/********************
		TRIANGULATION SECTION
		********************/
		float newx, newy, newphi, phiavg;
		float beta0, beta1, beta2, phi0, phi1, phi2;
		float a, dx, dy, d, h, rx, ry, d1, d2;
		float point2_x, point2_y;
		float intersectionPoint1_x, intersectionPoint2_x, intersectionPoint1_y, intersectionPoint2_y;


};

//Scanned::~Scanned(){}



#endif
