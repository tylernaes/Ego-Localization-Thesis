#pragma once
#include "Naes.h"

#ifndef LOCALIZATION_H
#define LOCALIZATION_H

class Localization {
	public:
		//Default Constructor
		Localization();

		//Overload Constructor
		Localization(vector<float> x, vector<float> y){
		newx = x;
		newy = y;
		}

		//Deconstructor
		~Localization();

		//Accessor Functions
		vector<float> get_x_vector() const{
			return newx;}

		vector<float> get_y_vector() const{
			return newy;}

		vector<float> get_phi_vector() const{
			return phi;}

		float get_radius(){
			radius = sqrt(((newx[0])*(newx[0]))+((newy[0])*(newy[0])));
			return radius;}

		float get_localization_x(){
			return newx.at(0);}

		float get_localization_y(){
			return newy.at(0);}

		float get_localization_phi(){
			return phi.at(0);}

		float get_average_phi(){
			return average_phi;}

		int get_x_size(){
			return newx.size();}

		int get_y_size(){
			return newy.size();}

		int get_phi_size(){
			return phi.size();}

		float get_x_value(int gluck){
			return newx[gluck];}

		//Mutator Functions
		void pushx(float x){
			newx.push_back(x);}

		void pushy(float y){
			newy.push_back(y);}

		void pushphi(float angle){
			phi.push_back(angle);}

		void calc_radius(){
			radius = sqrt(((newx[0])*(newx[0]))+((newy[0])*(newy[0])));}

		float calc_average_phi(float xx, float yy){
			if(xx < 0 && yy > 0){// need to add pi
				average_phi = atan( (yy/xx) ) + M_PI;}
			else if( xx < 0 && yy <= 0){// need to subtract pi
				average_phi = atan( (yy/xx) ) - M_PI;}
			else{// regular tan inverse
				average_phi = atan( (yy/xx) );}
			return average_phi;
		}

		void clearxego(){
			newx.clear();
			newx.resize(0);}

		void clearyego(){
			newy.clear();
			newy.resize(0);}

		void clearphiego(){
			phi.clear();
			phi.resize(0);}

		void set_object_left_true(){
			object_left = true;}

		void set_object_right_true(){
                        object_right = true;}

		void set_object_front_true(){
                        object_front = true;}

		void set_object_rear_true(){
                        object_rear = true;}

		void clear_object_logics(){
			object_left = false;
			object_right = false;
			object_front = false;
			object_rear = false;}

	private:
		//Member Variables
		vector<float> newx;
		vector<float> newy;
		vector<float> phi;
		float radius;
		float average_phi;
		bool object_left;
		bool object_right;
		bool object_rear;
		bool object_front;
};

Localization::Localization(){
}

Localization::~Localization(){
}

#endif
