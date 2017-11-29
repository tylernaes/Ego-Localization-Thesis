#include "Naes.h"
#include "orientation.h"
#include "drive.h"

void shutdownCallback(const naes::stop::ConstPtr& msg){
	std::cout << "stop topic called in drive_track_node" << std::endl;
        ros::shutdown();}


//Orientation orientation;
int main(int argc, char** argv){
	sleep(3);
	ros::init(argc, argv, "drive_track_node");
	ros::NodeHandle handle;
	Drive drive;


	ros::Subscriber stopSub = handle.subscribe<naes::stop>("/stop",1, shutdownCallback);

	drive.set_final_phi(-M_PI);
	drive.set_straight_before_true();
	drive.set_turning_left_false();
	drive.set_straight_after_false();
	drive.set_after_count_to_zero();
	while(ros::ok()) {
		ros::spinOnce;
	}		
	ros::shutdown();
	return 0;
}
