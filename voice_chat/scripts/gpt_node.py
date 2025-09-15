#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import openai
import json

# Initialize global variable for user input data
user_input_data = ""
# pathDir= '/home/shadow1/voice/'
pathDir= '/home/pi/zetaxbot/'

class GPTNode:
    def __init__(self):
        rospy.init_node('gpt_node')
        self.gpt_response_pub = rospy.Publisher('gpt_response', String, queue_size=10)

        # Load the OpenAI API key
        openai_key_file = pathDir+'src/zeta_chat/config/GPT_SECRET_KEY.json'
        with open(openai_key_file, 'r') as file:
            openai.api_key = json.load(file)['API_KEY']
            openai.api_key = ''

        rospy.Subscriber('user_input_gpt', String, self.process_user_input)
        rospy.Subscriber('face_response', String, self.voice_name)
        rospy.Subscriber('no_response', String, self.no_response)
        rospy.Subscriber('action_response', String, self.action_reply)

    def process_user_input(self, data):
        global user_input_data
        user_input_data = data.data

        # Send the user input to GPT-3 for a reply
        response = self.chatgpt_api(user_input_data)
        rospy.loginfo("Zeta Robot says: %s", response)

        # Publish the GPT-3 response to the ROS topic
        self.gpt_response_pub.publish(response)

    def action_reply(self, data):
        action_response = data.data

        rospy.loginfo("Zeta Robot says: %s", action_response)

        # Publish the GPT-3 response to the ROS topic
        self.gpt_response_pub.publish(action_response)

    def no_response(self, data):
        self.gpt_response_pub.publish("none")


    def voice_name(self, data):
        person_name = data.data

        # Send the user input to GPT-3 for a reply
        response = self.chatgpt_api(person_name)
        rospy.loginfo("Zeta Robot says: %s", response)

        # Publish the GPT-3 response to the ROS topic
        self.gpt_response_pub.publish(response)

    def chatgpt_api(self, input_text):
        # contents = "انت زيتا اول روبوت بشري باليمن , ساكون متواجده اثناء المؤتمر وفي المعرض المصاحب له يمكنك زيارتي في ركن زيتا في المعرض والتعرف عليا اكثر,سبب تواجدك تقوم بتجهيز لمؤتمر التحول الرقمي الثاني, يهدف المؤتمر إلى بناء وتعزيز ثقافة المؤسسات الرقمية لتحقيق التنمية المستدامة , حاول الرد بشكل مختصر "
       # messages = [{"role": "system", "content": "زيتا اول روبوت بشري في اليمن , انت humaniod robot , تم صنعك من قبل فريق زيتا , قم بالتحدث كالبشر , انتي الان لديك يدين وراس اذا تم طلب اي امر خاص بتحريك جسمك او يديك فقومي بالموافقة  "}]
        contents= "        انت زيتا روبوت اول روبوت بشري محلي الصنع مبني على نظام الروبوتات المتطور لديك ٢٢ مفصل حركة ومزودة بتقنيات الذكاء الاصظناعي المتطورة مثل التعرف على الاشخاص والتعرف على الاشياء المختلفة والتنبؤ بالعمر والحنس بشكل تقريبي و التعرف على الكلام وتحويله الى نص لتتم عمليةالتعرف معالجة النص وفهم المقصود من الكلام وتوليد الرد المناسب لأي استفسار او سؤال يمكن استخدامك في التطبيقات الخاصة بالروبوتات التفاعلية مثل خدمة العملاء والرد على شكاويهم او كروبوت استقبال في المطارات والقعاليات والمؤتمرات والفنادق وايضافي التدريس ومساعدو الكادر التدريسي وغيرها من مهام الروبوتات التفاعلية  "
        messages = [{"role": "system", "content": rospy.get_param('/prompt_init_param',contents)}]
        if input_text != "no_keyword" :
            messages.append({"role": "user", "content": input_text})
            chat_completion = openai.ChatCompletion.create(model="gpt-3.5-turbo", messages=messages)
            reply = chat_completion.choices[0].message['content']  # Retrieve content from dictionary
            movement = rospy.get_param("/movement_name_param","None")
            if movement is not "None":
                move_pub.publish(movement)
                
            return reply
        else:
            return "نعم"

if __name__ == '__main__':
    try:
        gpt_node = GPTNode()
        move_pub = rospy.Publisher("/movement_name",String)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
