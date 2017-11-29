//#include "localization.h"
#include "scanned.h"

float x__0, y__0, x__1, y__1, x__2, y__2;

void shutdownCallback(const naes::stop::ConstPtr& msg){
	cout << "stop topic called in localization_node" << endl;
	ros::shutdown();}

rosbag::Bag bag;
vector<string> topics;

int main(int argc, char** argv){
	sleep(5);
	float reject = 2.0;
	float grouping_constant = 0.15;
	float offset = 0.1; // This came from rotation.cpp
	ros::init(argc, argv, "localization_node");

	Scanned scanned;
	scanned.set_reject_value(reject);
	scanned.set_grouping_constant_value(grouping_constant);

	ros::NodeHandle nh;
        //Nodehandle needed for subscribing and publishing
	bag.open("egoPoints.bag", rosbag::bagmode::Read);
        topics.push_back(std::string("ego_points_topic"));
        rosbag::View view(bag, rosbag::TopicQuery(topics));
        foreach(rosbag::MessageInstance const m, view){
                naes::ego_points::ConstPtr s = m.instantiate<naes::ego_points>();
                if(s != NULL) {
                        x__0 = s->x_0;
                        y__0 = s->y_0;
                        x__1 = s->x_1;
                        y__1 = s->y_1;
                        x__2 = s->x_2;
                        y__2 = s->y_2;
                }
        }
        bag.close();
	scanned.set_global_ego_points(x__0, y__0, x__1, y__1, x__2, y__2);
	scanned.set_offset(offset);


	ros::Subscriber stopSub = nh.subscribe<naes::stop>("/stop",1, shutdownCallback);
	//this will stop the node when the topic stop has something published to it.

//        ros::Rate loop_rate(100);
        //rate of publishing & subscribing for ros::spin()

	while(ros::ok()){
		ros::spin();		
//                ros::spinOnce();
//                loop_rate.sleep();

        }// end of ros::ok()
	ros::shutdown();
        return 0;
}
