#!/usr/bin/env python

import rospy
import speech_recognition as sr
from std_msgs.msg import String
from zeta_chat.srv import RecognizeSpeech, RecognizeSpeechResponse

class SpeechRecognitionNode:
    def __init__(self):
        rospy.init_node('speech_recognition_node')

        # Initialize a flag to indicate if the voice is currently playing
        self.voice_playing = False

        # Subscribe to the end of voice playback event
        rospy.Subscriber('end_of_voice', String, self.voice_playback_callback)

        # Initialize the RecognizeSpeech service
        rospy.Service('recognize_speech', RecognizeSpeech, self.recognize_speech_service)

    def voice_playback_callback(self, data):
        if data.data == "end_of_voice":
            # Voice playback has ended
            self.voice_playing = False

    def recognize_speech_service(self, request):
        self.voice_playing = request.voice_playing
        audio_text = self.speech_recognition_thread()
        return RecognizeSpeechResponse(audio_text)

    def speech_recognition_thread(self):
        r = sr.Recognizer()
        mic = sr.Microphone()
        r.energy_threshold = 4000

        with mic as source:
            rospy.loginfo("Adjusting for ambient noise...")
            r.adjust_for_ambient_noise(source)
            rospy.loginfo("Listening for speech...")
            while not rospy.is_shutdown():
                if not self.voice_playing:
                    try:
                        audio = r.listen(source, timeout=5.0)  # Set timeout for 5 seconds
                        rospy.loginfo("Processing speech...")
                        text = r.recognize_google(audio, language='en-US')
                        rospy.loginfo("You said: %s", text)
                        return text
                    except sr.UnknownValueError:
                        rospy.logwarn("Unable to recognize speech")
                    except sr.WaitTimeoutError:
                        rospy.loginfo("Listening timed out")

if __name__ == '__main__':
    try:
        node = SpeechRecognitionNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
