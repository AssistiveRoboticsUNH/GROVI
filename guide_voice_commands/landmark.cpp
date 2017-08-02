#include "landmark.h"
/*
*	This acts as sort of an object for the landmarks
*	Just stores a point basically
*/

landmark::landmark() {

}

landmark::landmark(double x, double y, double z, double w) {
	px = x;
	py = y;
	az = z;
	aw = w;
}

geometry_msgs::Point landmark::getPoint() {
	geometry_msgs::Point p;
	p.x = px;
	p.y = py;
	p.z = pz;
	return p;
}
geometry_msgs::Quaternion landmark::getOrient() {
	geometry_msgs::Quaternion a;
	a.x = ax;
	a.y = ay;
	a.z = az;
	a.w = aw;
	return a;
}
geometry_msgs::Pose landmark::getPose(){
	geometry_msgs::Pose p;
	p.position = getPoint();
	p.orientation = getOrient();
	return p;
}
