#!/usr/bin/env python3
import os
import rospkg
import time
import shutil


from youtube_dl import YoutubeDL

import rospy
from rospy import Publisher, Subscriber
from anki_vector_ros.msg import RobotStatus
from std_msgs.msg import String, Int16
from speechrecognizer import SpeechRecognizer

# Ranges from 1 to 100
AUDIO_VOL = 10

WAKE_CMD_TIMEOUT = 5


class DJVector:
    def __init__(self) -> None:
        self.last_wake = 0
        self.woken_state = False
        self.assets_path = rospkg.RosPack().get_path("dj_vector") + "/assets"
        self.recognizer = SpeechRecognizer()

        Subscriber("/speech_recognition/final_result", String, self.process_speech)

        self.vol_pub = Publisher("/audio/vol", Int16, queue_size=0)
        self.sound_pub = Publisher("/audio/play", String, queue_size=0)
        self.speech_pub = Publisher("/behavior/say_text", String, queue_size=0)
        self.anim_pub = Publisher("/anim/play", String, queue_size=0)

        self.download_options = {
            "outtmpl": f"{self.assets_path}/song.mp3",
            "format": "worstaudio",
            "noplaylist": "True",
            "postprocessors": [
                {
                    "key": "FFmpegExtractAudio",
                    "preferredcodec": "wav",
                    "preferredquality": "192",
                }
            ],
            "postprocessor_args": "-acodec pcm_s16le -ac 1 -ar 16000".split(" "),
            "prefer_ffmpeg": True,
        }

        time.sleep(2)
        self.vol_pub.publish(AUDIO_VOL)

        os.makedirs(self.assets_path, exist_ok=True)

        self.process_time()

    def process_time(self):
        """Update variables that change daily."""
        while not rospy.is_shutdown():
            if self.woken_state and time.time() - self.last_wake > WAKE_CMD_TIMEOUT:
                self.woken_state = False
                self.anim_pub.publish("anim_wakeword_getout_01")

    def download_song(self, search_phrase):
        with YoutubeDL(self.download_options) as ydl:
            url = ydl.extract_info(f"ytsearch:{search_phrase}", download=False)[
                "entries"
            ][0]["webpage_url"]

            ydl.download([url])

        # Convert song to vector format
        # cmd = f"ffmpeg -i {self.assets_path}/song.wav -acodec pcm_s16le -ac 1 -ar 16000 {self.assets_path}/song_conv.wav"

        # sp.run(cmd.split(" "))

    def cleanup(self):
        self.sound_pub.publish(f"{self.assets_path}/song_conv.wav")
        shutil.rmtree(self.assets_path)

    def process_speech(self, msg):
        cur_time = time.time()
        # Measure time difference in seconds
        if cur_time - self.last_wake > WAKE_CMD_TIMEOUT and (
            "hey guido" in msg.data or "chiquita" in msg.data or "he guido" in msg.data
        ):
            self.woken_state = True
            self.last_wake = cur_time
            self.anim_pub.publish("anim_wakeword_getin_01")
            self.try_play_music()

    def try_play_music(self):
        results = self.recognizer.listen()

        if results is None:
            self.speech_pub.publish("I'm not sure what you said!")

        elif "play" not in results:
            self.speech_pub.publish("Hmmm, did you ask me to play a song?")
        else:
            phrase = results.split("play")[-1]
            self.speech_pub.publish(f"Searching for {phrase}")
            self.download_song(phrase)
            time.sleep(0.5)
            self.sound_pub.publish(f"{self.assets_path}/song.wav")


if __name__ == "__main__":
    rospy.init_node("dj_vector")
    rospy.wait_for_message("/status", RobotStatus)

    dj = DJVector()
    rospy.spin()

    rospy.on_shutdown(dj.cleanup)
