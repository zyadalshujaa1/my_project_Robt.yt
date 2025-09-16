#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from zeta_chat.srv import ClassifyInput, ClassifyInputResponse
from zeta_chat.srv import SynthesizeSpeech

class ClassificationNode:
    def __init__(self):
        rospy.init_node('classification_node')
        self.action_pub = rospy.Publisher('action_status', String, queue_size=10)
        self.gpt_pub = rospy.Publisher('user_input_gpt', String, queue_size=10)
        rospy.Subscriber('user_input', String, self.process_user_input)

        # Create the ClassifyInput service
        rospy.Service('classify_input', ClassifyInput, self.classify_input_service)

        # Wait for the SynthesizeSpeech service to be available
        rospy.wait_for_service('synthesize_speech')
        self.synthesize_speech = rospy.ServiceProxy('synthesize_speech', SynthesizeSpeech)

    def classify_input(self, user_input):
        if "pick the tool" in user_input:
            return "tool_pick"
        else:
            return None

    def classify_input_service(self, request):
        user_input = request.text
        action = self.classify_input(user_input)

        if action:
            rospy.loginfo("Robot is %s.", action.replace("_", " "))
            self.action_pub.publish(f"{action.capitalize()} successfully.")
        else:
            rospy.loginfo("User input not classified for an action.")
            # If input not classified, use GPT-3 for a reply
            self.gpt_pub.publish(user_input)
        
        # Synthesize the response using the voice node
        try:
            self.synthesize_speech(user_input)
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call to synthesize_speech failed: {e}")

        # Return a success response
        return ClassifyInputResponse(success=True)

    def process_user_input(self, data):
        user_input = data.data
        # Call the classify_input_service to handle the user input
        self.classify_input_service(user_input)

if __name__ == '__main__':
    try:
        node = ClassificationNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
