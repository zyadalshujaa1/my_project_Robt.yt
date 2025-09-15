#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import boto3
import json
from pydub import AudioSegment
from pydub.playback import play

# Initialize Amazon Polly client and the publisher for the end of voice playback event as global variables
polly_client = None
end_of_voice_pub = None
# pathDir= '/home/shadow1/voice/'
pathDir= '/home/pi/zetaxbot/'
def voice_chat_callback(data):
    global polly_client
    if data.data == "none":
        text= ""
    else :
        text = data.data

    # Convert the text to speech using Amazon Polly
    response = polly_client.synthesize_speech(
        VoiceId='Hala',
        OutputFormat='mp3',
        Text= text,
        Engine='neural'
    )

    # Save the speech response to a temporary file
    with open('/tmp/response.mp3', 'wb') as file:
        file.write(response['AudioStream'].read())
        file.close()

    # Play the response audio
    sound = AudioSegment.from_file("/tmp/response.mp3", format="mp3")
    rospy.set_param('/sound_status', True)  # Indicate that the sound is playing
    play(sound)
    rospy.set_param('/sound_status', False)  # Indicate that the sound is not playing

    # Publish an event indicating the end of voice playback
    end_of_voice_pub.publish(String("end_of_voice"))

def voice_node():
    global polly_client, end_of_voice_pub
    rospy.init_node('voice_node')

    # Load the Amazon API keys from the JSON file
    api_keys_file = pathDir+'src/voice_chat/config/amazon_api_keys.json'
    with open(api_keys_file, 'r') as file:
        api_keys_data = json.load(file)

    aws_access_key_id = api_keys_data['access_key']
    aws_secret_access_key = api_keys_data['secret_access_key']
    aws_region = api_keys_data['region']

    # Initialize Amazon Polly client
    polly_client = boto3.Session(
        aws_access_key_id=aws_access_key_id,
        aws_secret_access_key=aws_secret_access_key,
        region_name=aws_region
    ).client('polly')

    # Create a publisher for the end of voice playback event
    end_of_voice_pub = rospy.Publisher('end_of_voice', String, queue_size=10)

    # Publish an event indicating the end of voice playback initially
    end_of_voice_pub.publish(String("end_of_voice"))

    rospy.Subscriber('gpt_response', String, voice_chat_callback)

    rospy.spin()

if __name__ == '__main__':
    try:
        voice_node()
    except rospy.ROSInterruptException:
        pass
