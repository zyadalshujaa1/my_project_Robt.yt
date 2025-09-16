#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import speech_recognition as sr

class SpeechRecognitionNode:
    def __init__(self):
        rospy.init_node('speech_recognition_node')
        self.pub = rospy.Publisher('user_input', String, queue_size=10)
        self.status = rospy.Publisher('/status', String, queue_size=10)
        self.r = sr.Recognizer()
        self.mic = sr.Microphone()

        self.is_sound_playing = False
        self.gpt_replied = True
        self.lang_type = "ar"
        self.keyword_status = True

        rospy.Subscriber('end_of_voice', String, self.voice_playback_callback)
        rospy.Subscriber('gpt_response', String, self.gpt_response_callback)
        rospy.Subscriber('lang_talk', String, self.lang_callback)

        rospy.loginfo("Waiting for GPT reply...")
        self.listen_for_speech()
        rospy.set_param('/person_name','محمد')
        rospy.spin()

    def voice_playback_callback(self, data):
        if data.data == "end_of_voice":
            self.is_sound_playing = False
            if self.gpt_replied:
                self.listen_for_speech()

    def lang_callback(self, data):
        if data.data == "en":
            self.lang_type = "en"
        if data.data == "fr":
            self.lang_type = "fr"

    def gpt_response_callback(self, data):
        if data.data:
            self.gpt_replied = True

    def listen_for_speech(self):
        self.r.energy_threshold = 300

        with self.mic as source:
            self.r.adjust_for_ambient_noise(source)
            rospy.logwarn("Listening for speech...")
            self.status.publish("Listening for speech...")
            try:
                # rospy.log
                audio = self.r.listen(source, timeout=10)
                rospy.logwarn("Processing speech...")
                self.status.publish("Processing speech...")

                text = self.r.recognize_google(audio, language=self.lang_type)
                rospy.loginfo("You said: %s", text)

                lower_text = text.lower()

                if not self.keyword_status:
                    keywords = ["انجليزي","مرحبا", "زيتا", "hello" , "زيدان" , "جيتا"]
                    if any(keyword in lower_text for keyword in keywords):
                        for keyword in keywords:
                            lower_text = lower_text.replace(keyword, "من انت").strip()
                        self.pub.publish(lower_text)
                        self.keyword_status = True
                    else:
                        self.keyword_status = False
                        self.pub.publish("zeta")
                else:
                    self.pub.publish(lower_text)

            except sr.WaitTimeoutError:
                rospy.logwarn("No speech detected within the timeout")
                self.status.publish("No speech detected within the timeout")

                self.pub.publish("zeta")
                self.keyword_status = True
                self.lang_type = "ar"

            except sr.UnknownValueError:
                rospy.logwarn("Unable to recognize speech")
                self.status.publish("Unable to recognize speech")
                self.pub.publish("zeta")
                self.keyword_status = True
                self.lang_type = "ar"

            self.gpt_replied = False

if __name__ == '__main__':
    try:
        node = SpeechRecognitionNode()
    except rospy.ROSInterruptException:
        pass
