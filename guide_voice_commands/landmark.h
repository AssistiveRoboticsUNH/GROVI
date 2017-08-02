#ifndef LANDMARK_H
#define LANDMARK_H

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>
/*
*	This acts as sort of an object for the landmarks
*	Just stores a point basically
*/
class landmark {
	public:
		double px;
		double py;
		double pz;
		
		double ax;
		double ay;
		double az;
		double aw;
		
		geometry_msgs::Point getPoint();
		geometry_msgs::Quaternion getOrient();
		geometry_msgs::Pose getPose();
		
		landmark();
		landmark(double x, double y, double z, double w);
};

#endif
