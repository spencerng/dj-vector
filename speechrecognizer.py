import speech_recognition as sr


class SpeechRecognizer:
    def __init__(self, mic_id=None):
        self.recognizer = sr.Recognizer()

        if mic_id is None:
            self.mic = sr.Microphone()
        else:
            self.mic = sr.Microphone(mic_id)

        with self.mic as source:
            self.recognizer.adjust_for_ambient_noise(source)

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
