#!/usr/bin/python3
# -*- coding: utf-8 -*-
import json
import rospy
from std_msgs.msg import String

def send_speech(name, speech):
    msgs_dict = {}
    current_time = rospy.get_rostime()

    msgs_dict = {
        "header": {
            "timestamp": "%i.%i" % (current_time.secs, current_time.nsecs),
            "source": "perception",
            "target": ["dialog", "planning"],
            "content": ["human_speech"]
        },
        "human_speech": {
            "name": name,
            "speech": "%s" % speech
        }
    }

    json_string = json.dumps(msgs_dict, ensure_ascii=False, indent=4)
    pub_recog_topic.publish(json_string)




def send_sp(name, loc_x, loc_y, loc_z):
    current_time = rospy.get_rostime()
    frame_id = "simonpic"  # simonpic

    msgs_dict = {
        "header": {
            "timestamp": "%i.%i" % (current_time.secs, current_time.nsecs),
            "source": "perception",
            "target": ["planning"],
            "content": ["human_recognition"]
        },
        "human_recognition": {
            "name": name,
            "frame_id": frame_id,
            "loc_x": loc_x,
            "loc_y": loc_y,
            "loc_z": loc_z
        }
    }
    json_string = json.dumps(msgs_dict, ensure_ascii=False, indent=4)
    pub_recog_topic.publish(json_string)



def callback_cmd(data):
    start_flag = data.data
    name = "이병현"

    speech_list = []
    if start_flag == "start":
        # 실벗 : 안녕하세요 이병현 어르신
        speech = "응 안녕"
        send_speech(name, speech)
        rospy.sleep(3)
        # 실벗 : 어제는 잘 주무셨나요?
        speech = "잘잤다."
        send_speech(name, speech)
        rospy.sleep(3)
        # 실벗 : 푹 주무셨다니 다행이에요.
        speech = "고맙다."
        send_speech(name, speech)
        rospy.sleep(3)
        # 실벗 : 질병(고혈압)은 좀 어떠세요?
        speech = "좋아진 것 같아."
        send_speech(name, speech)
        rospy.sleep(3)
        # 실벗 : 식사는 뭐 드셨나요?
        speech = "설렁탕 먹었어"
        send_speech(name, speech)
        rospy.sleep(3)
        # 실벗 : 고혈압에는 싱거운 싱겁게 드시는게 중요한 거 아시죠?
        speech = "아 그럼 니가 매일 말해줘서 알지."
        send_speech(name, speech)
        rospy.sleep(3)
        # 실벗 : 약복용은 하셨나요?
        speech = "아직 안먹었어"
        send_speech(name, speech)
        rospy.sleep(3)
        # 실벗 : 약복용일정(식후 30분)에 약 복용 하셔야 하는거 아시죠?
        speech = "그렇지"
        send_speech(name, speech)
        rospy.sleep(3)


def terminal_loop():
    print("=============================================")
    print("             STT Dummy ROS Node              ")
    print("=============================================")
    print("사용자의 이름을 입력해주세요.                ")
    name = input("-> ")

    while True:
        print("사용자의 발화문을 입력해 주세요.")
        speech = input("-> ")
        send_speech(str(name), str(speech))
        input("Press Enter to continue...")





def scenario_simulator():
    global pub_recog_topic
    rospy.init_node('dummy_stt_converter', anonymous=False)
    rospy.Subscriber("simulation_trigger", String, callback_cmd)
    pub_recog_topic = rospy.Publisher("recognitionResult", String, queue_size=100)
    # pub_dialog_topic = rospy.Publisher("dialogResult", String, queue_size=100)
    # pub_task_topic = rospy.Publisher("taskCompletion", String, queue_size=100)
    # pub_recog_topic = rospy.Publisher("recognitionResult", String, queue_size=100)
    terminal_loop()
    rospy.spin()
    # rospy.s


if __name__ == '__main__':
    scenario_simulator()
