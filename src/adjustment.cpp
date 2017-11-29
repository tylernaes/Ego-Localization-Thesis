#include "adjustment.h"

float max_offset = 0.03; //first test was 0.03.
int normal_wheel_value = 50; //first test was 30.
int adjustment_constant = 120; //first test was 1.
float desired_radius = 0.02; //first test was 0.05.
float turn = M_PI / 2; //first test was M_PI / 6.
bool origin = false;
float origin_rotate_value = 0.02;
int wheel_value = 3;

void shutdownCallback(const naes::stop::ConstPtr& msg){
	cout << "stop topic called in adjustment_node" << endl;
        ros::shutdown();}

int main(int argc, char** argv){
	sleep(5);
	ros::init(argc, argv, "adjustment_node");

	/* CREATING THE OBJECT */
	Adjustment adjustment;
	adjustment.set_max_offset(max_offset);
	adjustment.set_adjustment_constant(adjustment_constant);
	adjustment.set_desired_radius(desired_radius);
	adjustment.set_turn_value(turn);
	adjustment.set_iteration_to_zero();
	adjustment.set_static_turn_to_false();
	adjustment.set_normal_wheel_value(normal_wheel_value);
	adjustment.set_wheel_change_value(wheel_value);
	adjustment.publish_brakes();

	/* EXTRACTING TRACKED POINTS FROM BAG FILE */
	vector<string> topics;
	rosbag::Bag bag;
	bag.open("follow.bag", rosbag::bagmode::Read);
	topics.push_back(std::string("tracked"));

	rosbag::View view(bag, rosbag::TopicQuery(topics));

	foreach(rosbag::MessageInstance const m, view){
	        naes::current_position::ConstPtr s = m.instantiate<naes::current_position>();
	        if(s != NULL) {
			adjustment.pushx(s->x);
	                adjustment.pushy(s->y);
	                adjustment.pushphi(s->phi);
	        }
	}
	bag.close();
	adjustment.set_navigation_size();
	while(ros::ok()){
		ros::spin();
	}
	ros::shutdown();
	return 0;
}
