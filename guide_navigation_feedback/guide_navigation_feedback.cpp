#include <thread>
#include <chrono>
#include <atomic>
#include <string>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include "ros/ros.h"
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <sound_play/sound_play.h>
#include <nav_msgs/Odometry.h>
#include <kobuki_msgs/SensorState.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include "guide_navigation_feedback/SimpleImage.h"
#include "guide_navigation_feedback/feedback.h"
#include <vector>
#include <math.h>

// Robot turning variables
#define LEFT 0
#define RIGHT 1
#define TURN_ANGLE M_PI/4

// Robot's slope states
#define FLAT 0
#define UP 1
#define DOWN 2

// Different kinds of slopes
#define NORMAL 1
#define BIG 2

// option flags
#define SLOPE_ON 1
#define TURN_IN_PLACE_ON 1
#define PATH_TURN_ON 1

//--------------------------------------
// 			Global Variables
//--------------------------------------
std::atomic<bool> active;
// Feedback variables
std::atomic<int> slopeOutputRequired;
std::atomic<int> turnOutputRequired;
std::atomic<int> turnInPlaceRequired;
// Feedback speech
sound_play::SoundClient* speech;
// feedback advertiser
ros::Publisher fbPub;
// localized the robot
geometry_msgs::PoseStamped* currentPos;
geometry_msgs::PoseStamped* currentGoal;
// used for checking turn in place
int turnCount;
// used for slope detection
ros::Publisher imgPub;
int slopeCount;
int slopeChange;
int imuSlopeCount;
int slopeState;
bool slopeZone;
int slopeType;
// timers for feedback
ros::Time lastTurn;
ros::Time lastTIP;
ros::Time lastSlope;
ros::Time lastImu;
// feedback information
int saveHalfway = 0;
geometry_msgs::PoseStamped progressLocs[3];
bool progressSaved = false;
int progMark = 0;

/*Identifies if the robot will turn based on the direction the robot is facing created from the parameters*/
int getTurnId(geometry_msgs::Point points[4]) {
	//std::cout << "start turnid\n";
	int turnId = -1;
	
	double dif1Y = points[1].y - points[0].y;
	double dif1X = points[1].x - points[0].x;
	
	double dif2Y = points[3].y - points[2].y;
	double dif2X = points[3].x - points[2].x;
	
	double ang1 = std::atan2(dif1Y, dif1X);
	double ang2 = std::atan2(dif2Y, dif2X);
	
	// calculate for change if sign turns (ie .9 and -.9 are only .2 away from eachother)
	if (ang1 + M_PI < ang2) {
		ang1 += 2*M_PI;
	}
	if (ang2 + M_PI < ang1) {
		ang2 += 2*M_PI;
	}
	
	// find difference between angles
	double trueAngDif = fabs(ang1 - ang2);
	if (trueAngDif > TURN_ANGLE) {
		if (ang1 - ang2 < 0) {
			turnId = LEFT;
		} else {
			turnId = RIGHT;
		}
	}
	return turnId;	
}

/* checks for turns in the upcoming path */
void checkForTurn(const nav_msgs::PathConstPtr& msg, int initPos, int lastPos, int delta) {
	//std::cout << "start turn\n";
	if (ros::Time::now() - lastTurn > ros::Duration(5)) {
		// needs a position and a goal before checking for new locations
		if (currentPos != NULL && currentGoal != NULL) {
			// if too close to goal, don't bother checking for direction
			double xDist = abs(currentPos->pose.position.x - currentGoal->pose.position.x);
			double yDist = abs(currentPos->pose.position.y - currentGoal->pose.position.y);
			if (xDist > 1 || yDist > 1){
				// begin checking for turns
				if (msg->poses.size() > lastPos){
					geometry_msgs::Point points[4];
					points[0] = msg->poses[initPos].pose.position;
					points[1] = msg->poses[initPos + delta].pose.position;
					points[2] = msg->poses[lastPos - delta].pose.position;
					points[3] = msg->poses[lastPos].pose.position;

					int turnId = getTurnId(points);
					if (turnId == LEFT || turnId == RIGHT) {
						std::string side;
						if (turnId == LEFT) {
							ROS_INFO("Side identified: LEFT");
							side = "left";
						}
						else {
							ROS_INFO("Side identified: RIGHT");
							side = "right";
						}
						std::string output = "We will turn " + side + " soon";
						if (turnOutputRequired == 1) {
							speech->say(output);
							lastTurn = ros::Time::now();
						}
					}
				}
			}
		}
	}
	//std::cout << "end turn\n";
}

/*Checks for turning in place*/
void spinChecker(const kobuki_msgs::SensorStateConstPtr& ss) {
	//std::cout << "start TIP\n";
	if (ros::Time::now() - lastTIP > ros::Duration(5)) {
		int delta = 8;
		// check power of motors
		if (abs(ss->left_pwm + ss->right_pwm) <= delta && ss->left_pwm != 0) {
			turnCount++;
			// if it is consistently turning
			if (turnCount == 10) {
				std::string dir;
				if (ss->left_pwm > 0){
					dir = "right";
				} else {
					dir = "left";
				}
				if (turnInPlaceRequired) {
					speech->say(dir + " turn in place.");
					ROS_INFO("Turning in place");
					lastTIP = ros::Time::now();
				}
			}
		} else {
			turnCount = 0;
		}
	}
}

// Takes data from depth image and translates it in to a
//			human readable image
void imgTransform(const sensor_msgs::ImageConstPtr& img) {
	//std::cout << "start img transform\n";
	guide_navigation_feedback::SimpleImage eimg;
	eimg.header = img->header;
	eimg.width = img->width;
	eimg.height = img->height;
	for (int i = 0; i < (img->width * img->height); i++) {
		// convert 4 8-bit ints into float
		int start = (i * 4) + 3;
		int distance = img->data[start];
		for (int j = start - 1; j >= start - 3; j--) {
			distance = distance << 8;
			distance += (int)(img->data[j]);
		}
		float dist = *((float*)(&distance));
		
		// new image stores distance as float
		eimg.data.push_back(dist);
	}
	imgPub.publish(eimg);
}

// Uses depth sensor to check for change in slope
void imgParser(const guide_navigation_feedback::SimpleImageConstPtr& img) {
	//std::cout << "start img parser\n";
	if (ros::Time::now() - lastSlope > ros::Duration(3)) {
		slopeZone = false;
	}
	
	// find change in slope on the ground in front of robot
	std::vector< std::vector<float> > depths;
	// start by gathering points to check in a 2d vector of floats
	for (int r = img->height - (img->height / 6); r < img->height - 15; r++) {
		std::vector<float> row;
		for (int c = img->width/2 - 40; c < img->width/2 + 40; c++) {
			int pos = r * img->width + c;
			row.push_back(img->data[pos]);
		}
		depths.push_back(row);
	}
	float sum = 0;
	int ccc = 0;
	// now find the average rate of change in each column
	for (int c = 0; c < depths[0].size(); c++) {
		float colSum = 0;
		for (int r = 0; r < depths.size() - 1; r++) {
			colSum += depths[r][c] - depths[r+1][c];
		}
		colSum /= (depths.size() - 1);
		if (!isnan(colSum)){
			ccc++;
			sum += colSum;
		}
	}
	// average change on ground
	sum /= ccc;
	if (ccc == 0)
		sum = .42;
	
	// find average distance in front to falsify ground data
	std::vector< std::vector<float> > front;
	// above center to omit nan data in depth sensor (look at image of depth sensor to know why)
	// this is same process as ground above
	for (int r = 15; r < img->height / 6; r++) {
		std::vector<float> row;
		for (int c = img->width/2 - 40; c < img->width/2 + 40; c++) {
			int pos = r * img->width + c;
			row.push_back(img->data[pos]);
		}
		front.push_back(row);
	}
	float distAvg = 0;
	int sz = 0;
	int closeCounter = 0;
	// same process as ground above
	for (int r = 0; r < front.size(); r++) {
		for (int c = 0; c < front[r].size(); c++) {
			if (!isnan(front[r][c])) {
				distAvg += front[r][c];
				if (front[r][c] < 1.25)
					closeCounter++;
				// part way on wall, but avg will mess it up
				if (closeCounter >= 20) {
					distAvg = 1.0;
					sz = 1;
					break;
				}
				sz++;
			}
		}
	}
	// average distance in front of robot
	distAvg /= sz;
	
	//std::cout  << "floor " << sum << " front: " << distAvg << std::endl;
	
	// if already confirmed to be in a slope area, check for type of slope
	if (slopeZone) {
		if (sum > .007 || sum < .24) {
			slopeType = BIG;
		} else {
			slopeType = NORMAL;
		}
	}
	
	// .0038 for upward slope, .0058 for downward slope
	if (sum < .0038 || sum > .0058) {
		slopeCount++;
	} else {
		slopeCount = 0;
		slopeChange = FLAT;
	}
	
	// can't be in front of a wall
	if (distAvg > 1.25 || (isnan(distAvg) && !isnan(sum))) {
		if (slopeCount == 5) {
			if (sum < .004)
				slopeChange = UP;
			else
				slopeChange = DOWN;
			lastSlope = ros::Time::now();
			slopeZone = true;
		}
	}
}

// subscriber callback procedure for slope detection confirmation
void slopeCheck(const sensor_msgs::ImuConstPtr& imu) {
	// Wait 5 seconds after checking slope to check for a new change in slope
	if (ros::Time::now() - lastImu > ros::Duration(5)) {
		float vel = imu->angular_velocity.y;
		// If currently on flat ground
		if (slopeState == FLAT) {
			// If |angular velocity| is greater than .1, and is changing slope
			if (fabs(vel) > .1 && slopeChange != FLAT) {
				ROS_INFO("Slope detected");
				slopeZone = false;
				if (slopeOutputRequired) {
					// double confirmation, announce change in slope
					std::string dir = vel > .08 ? "down" : "up";
					std::string type = slopeType == NORMAL ? "normal" : "steep";
					slopeState = slopeChange;
					slopeChange = FLAT;
					speech->say("Going " + dir + " a " + type + " slope.");
					std::this_thread::sleep_for(std::chrono::seconds(5));
				}
			}
		} else {
		// If currently on a slope
			// If |anguar velocity| is greater than .1, and is changing slope (not increasing slope size)
			if (fabs(vel) > .1 && ((slopeState == DOWN && slopeChange == UP) || (slopeState == UP && slopeChange == DOWN)) {
				slopeChange = FLAT;
				ROS_INFO("Flat ground");
				slopeZone = false;
				if (slopeOutputRequired) {
					// double confirmation, announce change in slope
					slopeState = FLAT;
					speech->say("Now on flat ground.");
					std::this_thread::sleep_for(std::chrono::seconds(5));
				}
			}
		}
	}
}

/*Idenitifies if the robot is turning left or right in it's path*/
void planParser(const nav_msgs::PathConstPtr& msg) {
	//std::cout << "start plan\n";
	try {
		if (turnOutputRequired == 1) {
			int firstPos = 0;
			int lastPos = 90;
			
			// check for change in direction
			checkForTurn(msg, firstPos, lastPos, 10);
		}
		if (saveHalfway) {
			saveHalfway--;
			// have to wait for second plan, because for some reason it comes in batches of 2,
			//		and the first one doesn't have the right end goal, it just has some random 
			//		point... I don't understand ROS...
			if (saveHalfway == 0) {
				int size = 0;
				double distance = 0;
				// calculate total distance of path.
				while (msg->poses[size].pose.position.x != currentGoal->pose.position.x &&
					   msg->poses[size].pose.position.y != currentGoal->pose.position.y) {
					distance += sqrt(pow(msg->poses[size+1].pose.position.x - msg->poses[size].pose.position.x, 2) +
									 pow(msg->poses[size+1].pose.position.y - msg->poses[size].pose.position.y, 2));
					size++;
				}
				double accDist = 0;
				double div = .25;
				// now mark a "progress point" at every 1/4 of distance
				for (int i = 0; i <= size; i++) {
					accDist += sqrt(pow(msg->poses[i+1].pose.position.x - msg->poses[i].pose.position.x, 2) +
									pow(msg->poses[i+1].pose.position.y - msg->poses[i].pose.position.y, 2));
					if (accDist > (distance * div)) {
						progressLocs[(int)(4*div) - 1] = msg->poses[i];
						div += .25;
						if (div == 1)
							break;
					}
				}
				progressSaved = true;
			}
		}
	} catch (...) { 
		ROS_INFO("Exception caught.");
	}
}

/*Saves the robot's current position to avoid tracking when close enough to goal position*/
void posSaver(const nav_msgs::OdometryConstPtr& o) {
	if (currentPos == NULL){
		ROS_INFO("Got initial position.");
		currentPos = new geometry_msgs::PoseStamped();
	}
	
	currentPos->pose.position.x = o->pose.pose.position.x;
	currentPos->pose.position.y = o->pose.pose.position.y;
	
	guide_navigation_feedback::feedback fb;
	fb.progMark = progMark;
	fbPub.publish(fb);
}

/*Checks amcl position for progress checks*/
void amclCheck(const geometry_msgs::PoseWithCovarianceStampedConstPtr& ppp) {
	if (progressSaved) {
		// find distance from current position to next progress location
		double distance = sqrt(pow(ppp->pose.pose.position.x - progressLocs[progMark].pose.position.x, 2) +
							   pow(ppp->pose.pose.position.y - progressLocs[progMark].pose.position.y, 2));
		// If it is .75 away, then it's close enough! (this is to adapt to subtle changes in path)
		if (distance < .75) {
			progMark++;
			// automatically announce halfway
			if (progMark == 2) {
				speech->say("Halfway to location.");
			}
			if (progMark == 4) {
				progressSaved = false;
			}
		}
	}
}

/*Saves the goal location the robot is going to*/
void goalSaver(const geometry_msgs::PoseStampedConstPtr& ps) {
	ROS_INFO("Got goal.");
	if (currentGoal == NULL)
		currentGoal = new geometry_msgs::PoseStamped();
		
	// used to make sure that it's using the second of two paths that get passed every time you set a goal
	//			see planParser for more info
	saveHalfway = 2;
	progMark = 0;
	
	currentGoal->pose.position.x = ps->pose.position.x;
	currentGoal->pose.position.y = ps->pose.position.y;
}

/*Timer that controls the feedback frecuency*/
/*Potentially obsolete? but leaving it in anyways incase something breaks*/
void resetFeedbackTimer()
{
    active = true; 
    while (active) {
		turnOutputRequired = PATH_TURN_ON;
		turnInPlaceRequired = TURN_IN_PLACE_ON;
		slopeOutputRequired = SLOPE_ON;
		std::this_thread::sleep_for(std::chrono::seconds(10));
    }
}

int main(int argc, char **argv) {
	turnOutputRequired = PATH_TURN_ON;
	turnInPlaceRequired = TURN_IN_PLACE_ON;
	slopeOutputRequired = SLOPE_ON;
	currentPos = NULL;
	currentGoal = NULL;
	slopeZone = false;
	slopeType = 0;
	ros::init(argc, argv, "guide_navigation_feedback");
	ros::NodeHandle n;
	speech = new sound_play::SoundClient();
	ROS_INFO("Navigation feedback initialized.");
	std::thread t(resetFeedbackTimer);
	// checks for turns
	ros::Subscriber subMarker = n.subscribe("/move_base/NavfnROS/plan", 10, planParser);
	// saves position
	ros::Subscriber subPosition = n.subscribe("/odom", 10, posSaver);
	// checks amcl position for progress
	ros::Subscriber subAmcl = n.subscribe("/amcl_pose", 10, amclCheck);
	// checks for tip
	ros::Subscriber subTurn = n.subscribe("/mobile_base/sensors/core", 10, spinChecker);
	// depth sensor
	ros::Subscriber subDepth = n.subscribe("/camera/depth/image", 10, imgTransform);
	imgPub = n.advertise<guide_navigation_feedback::SimpleImage>("/simple_image", 1000);
	ros::Subscriber subSimpleImage = n.subscribe("/simple_image", 10, imgParser);
	// saves goal
	ros::Subscriber subGoal = n.subscribe("/grovi/goal", 10, goalSaver);
	// records feedback info
	fbPub = n.advertise<guide_navigation_feedback::feedback>("/grovi/feedback", 1000);
	// imu slope confirmation
	ros::Subscriber subSlope = n.subscribe("/mobile_base/sensors/imu_data_raw", 20, slopeCheck);
	ros::spin();
	active = false;
	t.join(); 
	return 0;
}
