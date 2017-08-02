#include <thread>
#include <chrono>
#include <atomic>
#include <string>
#include <sstream>
#include <stdlib.h>
#include <vector>
#include <map>
#include <mutex>
#include <fstream>
#include <iostream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include <sound_play/sound_play.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "guide_navigation_feedback/feedback.h"
#include "landmark.h"

/*Robot's possible states*/
#define IDLE 0
#define WAITING_ACTION 1
#define WAITING_GOAL 2
#define NAVIGATING 3

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

/*Global variables*/
int state;
int goalCode;
std::atomic<bool> goalSet;
std::atomic<bool> active;
std::atomic<bool> cancelled;
std::map<int, std::string> mapCodeToLocationName;
std::map<std::string, int> mapLocationToCode;
sound_play::SoundClient* speech;
std::mutex navMutex;
MoveBaseClient* moveClient;
// publishes goal for navigation to use
ros::Publisher pubGoal;
ros::Time marker;
ros::Duration wait;
// recieves progress marker from navigation
int progMark;
// file where locations are stored (.txt format)
std::string locFile;

/* Coordinates for each possible goal location*/
landmark locs[88];

void commandParser(std::vector<std::string> commands);
void parseIdleCommand(std::vector<std::string> commands);
void parseActionCommand(std::vector<std::string> commands);
void parseGoalCommand(std::vector<std::string> commands);
void parseNavigatingCommand(std::vector<std::string> commands);

/*Prinst feedback on terminal*/
void printFeedback(std::string command) {
	std::string outstring = "Processing command: " + command;
	ROS_INFO("%s",outstring.c_str());
}

/*Sends a command to the sound play node*/
/* Allows for a wait time, due to when used without a mic,
*	it hears itself and will think it's talking to itself,
*	and it's really annoying, seriously use a mic if you can.*/
void say(std::string command, int waitTime) {
	marker = ros::Time::now();
	wait = ros::Duration(waitTime);
	speech->say(command);
}

/*Set of possible locations to navigate to
	Text is used by the robot to send feedback
	*/
void initLocationSet() {
	// input file
	std::ifstream file(locFile);
	
	int count = 0;
	// split by line
	std::string line;
	while( std::getline(file, line) ) {
		// split up each line in to name, and coordinates
		std::stringstream ss;
		ss.str(line);
		std::string name;
		std::getline(ss, name, ',');
		std::string coords[4];
		for (int i = 0; i < 4; i++) {
			std::getline(ss, coords[i], ',');
		}
		// create new landmarks
		locs[count] = landmark(std::stod(coords[0]), std::stod(coords[1]), std::stod(coords[2]), std::stod(coords[3]));
		// add to map
		mapCodeToLocationName[count] = name;
		mapLocationToCode[mapCodeToLocationName[count]] = count;
		count++;
	}
}

/*Send a navigation goal to the robot*/
void navigate(int code) {
	move_base_msgs::MoveBaseGoal goal;
	goal.target_pose.header.frame_id = "map";
	goal.target_pose.header.stamp = ros::Time::now();
	goal.target_pose.pose = locs[code].getPose();
	ROS_INFO("Sending goal");
	// let navigation know about the goal
	pubGoal.publish(goal.target_pose);
	moveClient->sendGoal(goal);
	goalSet = true;	
	navMutex.unlock();
}

/*Maps a location code to a location name*/
std::string getLocationName(int code) {
	return mapCodeToLocationName[code];
}

/*Function used by navigation threads to monitor and display the state of the navigation*/
void navigationThread() {
	while (active) {
		navMutex.lock();
		if (goalSet) {
		    moveClient->waitForResult();

			if(moveClient->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
				std::string msg = "We have reached " + getLocationName(goalCode);
				say(msg, 3);
				goalSet = false;
				cancelled = false;
				state = IDLE;
			}
			else {
				if (!cancelled) {
					std::string msg = "Navigation to " + getLocationName(goalCode) + " failed. Please contact support.";
					say(msg, 4);
					goalSet = false;
					state = IDLE;
				}
			}			
		}
	}
}

/*Gets a location code from a location name*/
int getGoalCode(std::string goal) {	
	if (mapLocationToCode.find(goal) == mapLocationToCode.end()){
		std::cout << "location code not found!\n";
		return -1;
	}
	return mapLocationToCode[goal];
}

/*Gets a token from the command sent by the user*/
void split(const std::string &s, char delim, std::vector<std::string> &tokens) {
	std::stringstream ss;
	ss.str(s);
	std::string token;
	std::vector<std::string>::iterator it = tokens.begin();
	while (std::getline(ss, token, delim)) {
		tokens.insert(it, token);
		it = tokens.begin();
	}
}

/*Splits a string given a delimitator*/
std::vector<std::string> split(const std::string &s, char delim) {
	std::vector<std::string> tokens;
	split(s, delim, tokens);
	return tokens;
}

/*Parse a command when the robot is idle*/
void parseIdleCommand(std::vector<std::string> commands) {
	std::cout << "Idle: ";
	for (int i = 0; i < commands.size(); i++) {
		std::cout << commands[i] << " ";
	}
	std::cout << std::endl;
	
	std::string command = commands.back();
	commands.pop_back();
	printFeedback(command);
	if (command == "grovi") {		
		state = WAITING_ACTION;
		// more to say, continue to new state
		if (!commands.empty()) {
			commandParser(commands);
		}
		else {
			if (commands.size() == 0) {
				say("Tell me", 2);
			} else {
				parseActionCommand(commands);
			}
		}
	}
	else if (command == "go") {
		if (goalSet) {
			state = NAVIGATING;
			say("Navigating to " + getLocationName(goalCode), 2);
			navigate(goalCode);
		}
		else {
			say("No goal has been defined", 3);
		}
	}
	else if (command == "feedback") {
		if (!goalSet)
			say("No goal has been defined", 3);
		else {
			std::string msg = "Navigation to " + getLocationName(goalCode) + " was stopped";
			say(msg, 3);
		}
	}
}

/*Parse a command when the robot is expecting an action*/
void parseActionCommand(std::vector<std::string> commands) {
	std::cout << "Action: ";
	for (int i = 0; i < commands.size(); i++) {
		std::cout << commands[i] << " ";
	}
	std::cout << std::endl;
	
	std::string command = commands.back();
	commands.pop_back();
	printFeedback(command);
	if (command == "stop") {
		state = IDLE;
		say("OK. Stopping", 1);
	}
	else if (command == "navigate") {
		state = WAITING_GOAL;
		if (commands.size() == 0) {
			say("Where do you want to go?", 3);
		} else {
		// more to say, continue to new state
			parseGoalCommand(commands);
		}
	}
	else if (command == "feedback"){
		say("I am waiting for a command", 2);
	} else {
		say("Tell me.", 1);
	}
}

/*Parse a command when the robot is expecting a navigation goal*/
void parseGoalCommand(std::vector<std::string> commands) {
	std::string goal = commands.back();
	commands.pop_back();
	while (commands.size() > 0) {
		goal += " " + commands.back();
		commands.pop_back();
	}
	
	std::cout << std::endl;
	std::cout << "goal: " << goal << std::endl;
	printFeedback(goal);
	if (goal == "stop") {
		state = IDLE;
		say("OK. Stopping", 1);
	}
	else if (goal == "feedback"){
		say("I am waiting for a goal", 2);
	}
	else if (goal == "") {
		say(goal + " is not a valid goal. Where do you want to go?", 6);
	}
	else {
		goalCode = getGoalCode(goal);
		if (goalCode == -1) {
			say(goal + " is not a valid goal. Where do you want to go?", 6 + (goal.length() / 6));
		}
		else {
			state = NAVIGATING;
			say("Navigating to " + goal, 3);
			navigate(goalCode);
		}
	}
}

/*Parse a command when the robot is navigating to a goal*/
void parseNavigatingCommand(std::vector<std::string> commands) {
	std::string command;
	for (int i = 0; i < commands.size(); i++) {
		std::cout << commands[i] << " ";
	}
	std::cout << std::endl;
	command = commands.back();
	commands.pop_back();
	printFeedback(command);
	if (command == "stop") {
		state = IDLE;
		moveClient->cancelGoal();
		cancelled = true;
		std::string msg = "Navigation to " + getLocationName(goalCode) + " has been stopped.";
		say(msg, 3);
	}
	// provides feedback based on the progress sent by navigation
	else if (command == "feedback"){
		switch (progMark){
			case 0:
				say("We are navigating to " + getLocationName(goalCode), 3);
				break;
			case 1:
				say("We are about one quarter of the way to " + getLocationName(goalCode), 4);
				break;
			case 2:
				say("We are about half way to " + getLocationName(goalCode), 3.5);
				break;
			case 3:
				say("We are about three quarters of the way to " + getLocationName(goalCode), 4.5);
				break;
			case 4:
				say("We are almost to " + getLocationName(goalCode), 3);
				break;
			default:
				say("Unknown feedback response", 3);
				break;
		}
	}
}

/*Executes a parsing routine depending on the robot's current state*/
void commandParser(std::vector<std::string> commands) {		
	switch (state) {
		case IDLE:
		parseIdleCommand(commands);
		break;
		case WAITING_ACTION:
		parseActionCommand(commands);
		break;
		case WAITING_GOAL:
		parseGoalCommand(commands);
		break;
		case NAVIGATING:
		parseNavigatingCommand(commands);
		break;
	}
}

/*Subscriber callback procedure for commands*/
void commandParser(const std_msgs::StringConstPtr& msg) {
	// Check the wait timer to make sure it doesn't hear itself
	if (ros::Time::now() >= marker + wait) {
		std::vector<std::string> commands = split(msg->data, ' ');
		commandParser(commands);
	} else {
		ROS_INFO("Command ignored");
	}
}

// records the progress of the path
void locFbParser(const guide_navigation_feedback::feedbackConstPtr& fb) {
	progMark = fb->progMark;
}

int main(int argc, char **argv) {
	state = IDLE;
	goalSet = false;
	ros::init(argc, argv, "guide_voice_commands");
	ros::NodeHandle n;
	active = true;
	cancelled = false;
	navMutex.lock();
	std::thread t(navigationThread);
	speech = new sound_play::SoundClient();
	moveClient = new MoveBaseClient("move_base", true);
	ROS_INFO("Node initialized.");
	n.getParam("/guide_voice_commands/location_file", locFile);
	initLocationSet();	
	ros::Subscriber subFeedback = n.subscribe("/recognizer/output", 20, commandParser);
	ros::Subscriber subLocFeedback = n.subscribe("/grovi/feedback", 20, locFbParser);
	pubGoal = n.advertise<geometry_msgs::PoseStamped>("/grovi/goal", 10);
	ros::spin();
	active = false;
	goalSet = false;
	navMutex.unlock();
	t.join(); 
	return 0;
}


