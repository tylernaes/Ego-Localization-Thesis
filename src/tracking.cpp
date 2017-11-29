#include "Naes.h"

rosbag::Bag bag; //creates an object called bag from Bag class.
//bag.open("tracking.bag", rosbag::bagmode::Write); //Im guessing open function provides name and declares what kind of bagmode it is in.


void trackerCallback(const naes::current_position::ConstPtr& msg) {
	if( (sqrt(((msg->x)*(msg->x))+((msg->y)*(msg->y)))) >= 0.10 ){
		bag.write("tracked", ros::Time::now(), msg);
		cout << "trackerCallback is triggered" << endl;
		cout << "current x position is ..." << msg->x << endl;
		cout << "current y position is ..." << msg->y << endl;
		cout << "current phi angle is ..." << msg->phi << endl;}
}

void shutdownCallback(const naes::stop::ConstPtr& msg){
	cout << "stop topic called in tracking_node" << endl;
        ros::shutdown();}

int main(int argc, char** argv){
	sleep(5);
	ros::init(argc, argv, "tracking_node");
	ros::NodeHandle nh;
//	rosbag::Bag bag; //creates an object called bag from Bag class.
	ros::Subscriber stopSub = nh.subscribe<naes::stop>("/stop",1, shutdownCallback);
	bag.open("leftfollowing.bag", rosbag::bagmode::Write); //Im guessing open function provides name and declares what kind of bagmode it's in.
	ros::Rate loop_rate(10);
	ros::Subscriber sub = nh.subscribe("currentPosition", 1, trackerCallback);
	while(ros::ok()) {
		ros::spinOnce();
		loop_rate.sleep();}
	bag.close();
	return 0;
}
