#include <thread>
#include <chrono>
#include <atomic>
#include <string>
#include <stdio.h>
#include "ros/ros.h"
#include <visualization_msgs/Marker.h>
#include <sound_play/sound_play.h>

std::atomic<int> matchCounter;
std::atomic<bool> active; 

/* Maps an ar tag id with a location name
	ToDo: move this information to a configuration file
*/
std::string getMarkerName(int id) {
	switch(id) {
		case 1001: return "South 101";
		case 1002: return "South 105";
		case 1003: return "South 106";
		case 1004: return "South 107";
		case 1005: return "South 108";
		case 1006: return "South 109";
		case 1007: return "South 111";
		case 1008: return "South 113";
		case 1009: return "South 115";
		case 1010: return "South 120";
		case 1011: return "South 123";
		case 1012: return "South 124";
		case 1013: return "South 125";
		case 1014: return "South 145";
		case 1015: return "South 161";
		case 1016: return "South 162";
		case 1017: return "South 166";
		case 1018: return "South 168";
		case 1019: return "South 171";
		case 1020: return "South 172";
		case 1021: return "South 173";
		case 1022: return "South 174";
		case 1023: return "South 175";
		case 1024: return "South 178";
		
		case 1025: return "North 101";
		case 1026: return "North 110";
		case 1027: return "North 111";
		case 1028: return "North 113";
		case 1029: return "North 118";
		case 1030: return "North 121";
		case 1031: return "North 124";
		case 1032: return "North 129";
		case 1033: return "North 130";
		case 1034: return "North 133";
		case 1035: return "North 134";
		case 1036: return "North 137";
		case 1037: return "North 141";
		case 1038: return "North 142";
		
		case 1039: return "West 111";
		case 1040: return "West 113";
		case 1041: return "West 114";
		case 1042: return "West 115";
		case 1043: return "West 117";
		case 1044: return "West 118";
		case 1045: return "West 121";
		case 1046: return "West 123";
		case 1047: return "West 131";
		case 1048: return "West 132";
		case 1049: return "West 133";
		case 1050: return "West 135";
		case 1051: return "West 136";
		case 1052: return "West 137";
		case 1053: return "West 138";
		case 1054: return "West 139";
		case 1055: return "West 141";
		case 1056: return "West 161";
		case 1057: return "West 171";
		case 1058: return "West 173";
		case 1059: return "West 175";
		case 1060: return "West 177";
		case 1061: return "West 181";
		case 1062: return "West 183";
		case 1063: return "West 189";
		case 1064: return "West 190";
		
		case 1065: return "Bathroom";
		case 1066: return "South Elevator";
		case 1067: return "West Elevator";
		case 1068: return "North Elevator";
		case 1069: return "South Stairs";
		case 1070: return "North Stairs";
		case 1071: return "West Stairs";
		case 1072: return "Exit";
	}
	return "an unknown location";
}

/*Identifies the information sent by the ar_track_alvar node*/
void markerParser(const visualization_msgs::MarkerConstPtr& msg) {
	int id = msg->id;
	if (id < 1073 && id > 1000) {
		ROS_INFO("[%i]", id);
		if (matchCounter == 5) {
			ROS_INFO("AR marker detected: [%i]", id);
				std::string output = "We are approaching " + getMarkerName(id);
				sound_play::SoundClient* speech = new sound_play::SoundClient();
				speech->say(output);
		}
		matchCounter++;
	}
}

/*Timer that controls feedback frecuency*/
void resetFaceRecogTimer()
{
    active = true; 
    while (active) {
       matchCounter = 0;
       std::this_thread::sleep_for(std::chrono::seconds(10));
    }
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "guide_ar_localization");
	ros::NodeHandle n;
	ROS_INFO("AR localization initialized.");
	std::thread t(resetFaceRecogTimer);
	ros::Subscriber subMarker = n.subscribe("/visualization_marker", 150, markerParser);
	ros::spin();
	active = false;
	t.join(); 
	return 0;
}
