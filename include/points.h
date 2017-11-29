#pragma once
#include "Naes.h"

#ifndef POINTS_H
#define POINTS_H

class Points {
        public:
                vector<float> scannedData;
                int i;
                int size;
                float x;
                float y;

                //Default Constructor
                Points() : scannedData(0) {}

                void set(vector<float> transmittingData){
                        transmittedData = transmittingData;
                }

                void set_min(float mini_angle){
                        minimum_angle = mini_angle;
                }
                void set_inc(float incr_angle){
                        angle_increment = incr_angle;
                }

                void ScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
                                scannedData.clear();
                                size = scan->ranges.size();
                                scannedData.resize(size);
                                for(i = 0; i < size; i = i + 1){
                                        scannedData.at(i) = scan->ranges[i];
                                }
                                set(scannedData);
                                set_min(scan->angle_min);
                                set_inc(scan->angle_increment);
                }

                int getSize() const {
                        return transmittedData.size();
                }

                vector<float> getVector(){
                        return transmittedData;
                }
                float get_min(){
                        return minimum_angle;
                }
                float get_inc(){
                        return angle_increment;
                }

//      private:
                vector<float> transmittedData;
                float minimum_angle;
                float angle_increment;
};

#endif
