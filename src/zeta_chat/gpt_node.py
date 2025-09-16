#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from zeta_chat.srv import GetGptResponse, GetGptResponseResponse
import openai
import json

class GPTNode:
    def __init__(self):
        rospy.init_node('gpt_node')

        # Load the OpenAI API key
        openai_key_file = '/home/shadow1/voice/src/zeta_chat/config/GPT_SECRET_KEY.json'
        with open(openai_key_file, 'r') as file:
            openai.api_key = json.load(file)['API_KEY']

        rospy.Service('get_gpt_response', GetGptResponse, self.get_gpt_response_service)

    def get_gpt_response_service(self, request):
        user_input = request.user_input

        # Send the user input to GPT-3 for a reply
        response = self.chatgpt_api(user_input)
        rospy.loginfo("Zeta Robot says: %s", response)

        return GetGptResponseResponse(response)

    def chatgpt_api(self, input_text):
        messages = [{"role": "system", "content": "You are a helpful assistant."}]

        if input_text:
            messages.append({"role": "user", "content": input_text})
            chat_completion = openai.ChatCompletion.create(model="gpt-3.5-turbo", messages=messages)

        reply = chat_completion.choices[0].message['content']
        return reply

if __name__ == '__main__':
    try:
        gpt_node = GPTNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
