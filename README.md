# dj-vector

Play songs from YouTube on the Anki Vector robot, Alexa style

## Setup

1. Clone the [Vector Robot ROS wrapper](https://github.com/sebolab/vector-robot) to the `src` folder of your catkin workspace. Complete the steps in the "Usage" section to install the `anki_vector_ros` package and configure Vector's connnection.
2. Clone the [ROS vosk package](https://github.com/alphacep/ros-vosk) into `src` to enable offline voice recognition via the `/speech_recognition/final_result` and `/speech_recognition/partial_result` topics
3. Build this package from the catkin workspace root via `catkin_make`
4. Install `ffmpeg`: `snap install ffmpeg`

## Usage

Navigate to this folder and launch the main node and ROS wrapper backend:

```
roslaunch launch/dj.launch
```

Connect a microphone to your computer, then prompt Vector (named Guido after the Cars character in this repo): "Hey Guido, play <song title> [by <artist name>]"

After several seconds, the song will begin playing!

Adjust the `AUDIO_VOL` parameter in `main.py` from 1-100 as needed.
