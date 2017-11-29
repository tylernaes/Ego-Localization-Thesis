#include "localization.h"
#include "points.h"
#include "orientation.h"

float reject;
float grouping_constant;
int i;
int j;
int ego_size;
int size;
int scan_count;
int m;
int n;
int o;
int k;
int kk;
int ll;
float angle_increment;
float angle_min;
float angle;
float test_x;
float test_y;
float abs_diff_x;
float abs_diff_y;
float x;
float y;
float x_0, y_0, r_0, x_1, y_1, r_1, x_2, y_2, r_2;
float yaw, yaw_sum, yaw_final;
bool grouped;


float calcx(float range, float angle){
	float x = range * cos(angle);
	return x;}

float calcy(float range, float angle){
	float y = range * sin(angle);
	return y;}

Points p1;
//Orientation imu;
rosbag::Bag bag;
//rosbag::Bag yawbag;

int main(int argc, char** argv){
	sleep(5);

	reject = 1.7;

	grouping_constant = 0.15;

	ros::init(argc, argv, "zeroing_node");
	//Initiates the node

	ros::NodeHandle nh;
	//Nodehandle needed for subscribing and publishing

	bag.open("egoPoints.bag", rosbag::bagmode::Write);
	//yawbag.open("yaw_offset.bag", rosbag::bagmode::Write);

	naes::ego_points msg;
	//naes::orientations yawmsg;

	//ros::Subscriber subimu = nh.subscribe<sensor_msgs::Imu>("/imu/data", 1, &Orientation::zeroingCallback, &imu);

	ros::Subscriber sub;
	//the object "sub" will be used for subscribing to laserscan data.

	ros::Rate r(1);
	//rate of publishing & subscribing for ros::spin()

	vector<float> ego_points;
	//creates a vector of the current scan data to be used in the main code.

	vector<Localization> ego;
	//creates a vector of objects(ego) of the class(Localization)

	sub = nh.subscribe<sensor_msgs::LaserScan>("/scan",10, &Points::ScanCallback, &p1);

	while(ros::ok()){
		//yaw_sum = 0.0;
		scan_count = 0;
		while(scan_count < 3){
/*			if(ego.size() > 0){
				cout << "objects after scan" << endl;
				cout << "test number is: " << ego[0].get_x_size() << endl;
			}
*/			//sub = nh.subscribe<sensor_msgs::LaserScan>("/scan",10, &Points::ScanCallback, &p1);
			ego_points.clear();
			ego_points = p1.getVector();
			ego_size = ego_points.size();
			if(ego_size == 0){
				//This is used to make sure there is an eog_points vector to work with.
			}
			else{
				//yaw = imu.get_yaw();
				//cout << "yaw is: " << yaw << endl;
				//yaw_sum = yaw_sum + yaw;
				angle_increment = p1.get_inc();
				angle_min = p1.get_min();
				for(i=0; i < ego_size; i++){
					if(ego_points[i] < reject){
//						cout << "found a point at number: " << i << endl;
						angle = angle_min + (i * angle_increment);
//						cout << "its angle is: " << angle << endl;
						x = calcx(ego_points[i], angle);
//						cout << "its x coordinate is: " << x << endl;
						y = calcy(ego_points[i], angle);
//						cout << "its y coordinate is: " << y << endl;
						if(ego.size() == 0){
							ego.resize(1);
							ego[0].pushx(x);
							ego[0].pushy(y);
						}
						else{// must go through all the current objects.
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
									grouped = true;
								}
								else{ //it does not belong to this group, do nothing and check the next one;
								}
							}
							if(grouped == false){ //need to make a new object in ego vector.
								ego.resize(size+1);
								ego[size].pushx(x);
								ego[size].pushy(y);
							}
							else{ //already grouped from the loop. do nothing finished with ego_point[i];
							}
						}
					}
				}
				scan_count = scan_count + 1;
				cout << "number of ego points is: " << ego.size() << endl;
			}
			r.sleep();
			ros::spinOnce();
		}// end of scan_count while loop
		cout << "done with scan count of 10" << endl;
		ros::shutdown();
		o = ego.size();
		if(o == 3){
			vector<float> toAverageX;
			vector<float> toAverageY;
			for(k = 0; k < o; k++){
				cout << "size of x object: " << ego[k].get_x_size() << endl;
				cout << "size of y object: " << ego[k].get_y_size() << endl;
			}

			cout << "size of 'o' is: " << o << endl;
			//yaw_final = yaw_sum / scan_count;
			//cout << "yaw_final is: " << yaw_final << endl;
			for(k = 0; k < o; k++){
				toAverageX = ego[k].get_x_vector();
	//			cout << toAverageX[0] << endl;
				toAverageY = ego[k].get_y_vector();
				m = toAverageX.size();
				n = toAverageY.size();
				float xsum = 0.0;
				float ysum = 0.0;
				for(kk = 0; kk < m; kk++){
					xsum = xsum + toAverageX[kk];
				}
				for(ll = 0; ll < n; ll++){
					ysum = ysum + toAverageY[ll];
				}
				float x_final = xsum / m;
				float y_final = ysum / n;
				ego[k].clearxego();
				ego[k].clearyego();
				ego[k].pushy(y_final);
				ego[k].pushx(x_final);
				toAverageX.clear();
				toAverageY.clear();
				toAverageX.resize(1);
				toAverageY.resize(1);
				cout << "x-coordinate for the ego-point number " << k << " is: " << ego[k].get_localization_x() << endl;
				cout << "y-coordinate for the ego-point number " << k << " is: " << ego[k].get_localization_y() << endl;
			}

			x_0 = ego[0].get_localization_x();
			y_0 = ego[0].get_localization_y();
			x_1 = ego[1].get_localization_x();
			y_1 = ego[1].get_localization_y();
			x_2 = ego[2].get_localization_x();
			y_2 = ego[2].get_localization_y();
			msg.x_0 = x_0;
			msg.y_0 = y_0;
			msg.x_1 = x_1;
			msg.y_1 = y_1;
			msg.x_2 = x_2;
			msg.y_2 = y_2;
			//yawmsg.yaw = yaw_final;
			//yawmsg.phi = 0.0;
			bag.write("ego_points_topic", ros::Time::now(), msg);
			//yawbag.write("yaw_offset", ros::Time::now(), yawmsg);
			bag.close();
			//yawbag.close();
		}
		else{
			cout << "ERROR WITH ZEROING, THE NUMBER OF EGO POINTS WAS NOT THREE" << endl;}
	}// end of ros::ok()
	return 0;
}
