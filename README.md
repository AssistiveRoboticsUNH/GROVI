# GROVI

## Description
GROVI is a guide robot to assist the visually impaired, designed to operate in Kingsbury Hall.

## Dependencies
The following packages are needed to run the GROVI nodes:
- actionlib
- ar_track_alvar
- face_recognition
- move_base_msgs
- nav_msgs
- pocketsphinx
- roscpp
- rospy
- sound_play
- std_msgs
- usb_cam
- visualization_msgs

## Startup
To launch GROVI, use the launch file in guide_navigation_feedback named grovi.launch

## Important Notes
- Parameters marker_size, max_new_marker_error and max_track_error included in file grovi_ar_localization.launch should be configured depending on the size of the physical AR markers that are being used.
- Parameters lm and dict included in file grovi_voice_commands.launch should be configured to match the language model and dictionary files that will be used for the robot.
- In the guide_voice_commands directory there is a directory called speech. The file speech_kb found in this directory was used to generate the language model and dictionary files used for GROVI. If the language needs to be modified this file should be modified and a new language model and dictionary file should be generated using CMU's online tool found here: http://www.speech.cs.cmu.edu/tools/lmtool-new.html
- If copying files in to new packages, make sure to rename any mention of path names in launch files.
- The path found in the first line of the kings_first_floor.yaml file found in turtlebot_navigation/maps should match the directory of the kings_first_floor.pgm file.
- The map being used can be found in guide_navigation_feedback/launch/grovi_navigation_feedback.launch

## TODO
The following are enhancements that can be done to this project
- Create a node that handles the messages sent to sound_play_node to avoid repeated messages and stop messages from interrupting each other.
- guide_ar_localization:
    - Map from ar tag ids to location names using a configuration file
- guide_face_recognition:
    - Change the feedback given by the robot to make it more understandable
- guide_navigation_feedback:
    - Enhance distance tracker to give percent or numerical distance traveled
- guide_voice_commands:
    - Adjust wait timers for responses from GROVI to be 0 if user is/isn't using headphones.
    - Incorporate "fluff" words such as "to" or "the" ie. GROVI navigate to the elevator
    - Allow the user to adjust speeds by command
