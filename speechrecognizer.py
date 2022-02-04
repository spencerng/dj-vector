import queue
import sounddevice as sd
from google.cloud import speech
import speech_recognition as sr


class SpeechRecognizer:
    def __init__(self, mic_id=None):
        self.client = speech.SpeechClient()

        self.recognizer = sr.Recognizer()

        if mic_id is None:
            self.mic = sr.Microphone()
        else:
            self.mic = sr.Microphone(mic_id)

        self.config = speech.RecognitionConfig(
            encoding=speech.RecognitionConfig.AudioEncoding.LINEAR16,
            language_code="en-US",
            audio_channel_count=1,
            enable_separate_recognition_per_channel=False,
        )

        with self.mic as source:
            self.recognizer.adjust_for_ambient_noise(source)

        self.device_info = sd.query_devices()
        self.queue = queue.Queue()

    def audio_callback(self, indata, frames, time, status):
        """This is called (from a separate thread) for each audio block."""
        self.queue.put(bytes(indata))

    def listen(self):
        with self.mic as source:
            print("Speak!")
            audio = self.recognizer.listen(source)
            try:
                print("Recognizing...")
                return self.recognizer.recognize_google(audio)
            except sr.UnknownValueError:
                return None
            except sr.RequestError as e:
                print(
                    "Could not request results from Google Speech Recognition service; {0}".format(
                        e
                    )
                )
