#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from zeta_chat.srv import SynthesizeSpeech, SynthesizeSpeechResponse

import boto3
import json
from pydub import AudioSegment
from pydub.playback import play

class VoiceNode:
    def __init__(self):
        rospy.init_node('voice_node')

        # Load the Amazon API keys from the JSON file
        api_keys_file = '/home/shadow1/voice/src/zeta_chat/config/amazon_api_keys.json'
        with open(api_keys_file, 'r') as file:
            api_keys_data = json.load(file)

        aws_access_key_id = api_keys_data['access_key']
        aws_secret_access_key = api_keys_data['secret_access_key']
        aws_region = api_keys_data['region']

        # Initialize Amazon Polly client
        self.polly_client = boto3.Session(
            aws_access_key_id=aws_access_key_id,
            aws_secret_access_key=aws_secret_access_key,
            region_name=aws_region
        ).client('polly')

        # Create a publisher for the end of voice playback event
        self.end_of_voice_pub = rospy.Publisher('end_of_voice', String, queue_size=10)

        # Create the SynthesizeSpeech service
        rospy.Service('synthesize_speech', SynthesizeSpeech, self.synthesize_speech_service)

    def synthesize_speech_service(self, request):
        text = request.text

        # Convert the text to speech using Amazon Polly
        response = self.polly_client.synthesize_speech(
            VoiceId='Hala',
            OutputFormat='mp3',
            Text=text,
            Engine='neural'
        )

        # Save the speech response to a temporary file
        with open('/tmp/response.mp3', 'wb') as file:
            file.write(response['AudioStream'].read())

        # Play the response audio
        sound = AudioSegment.from_file("/tmp/response.mp3", format="mp3")
        play(sound)

        # Publish an event indicating the end of voice playback
        self.end_of_voice_pub.publish(String("end_of_voice"))

        # Return a success response
        return SynthesizeSpeechResponse(success=True)

if __name__ == '__main__':
    try:
        node = VoiceNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
