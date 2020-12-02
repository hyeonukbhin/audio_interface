#!/usr/bin/python2.7
# -*- coding: utf-8 -*-
import json
import rospy
from std_msgs.msg import String
import rospkg
import csv
from signal import signal, SIGINT
from sys import exit
import sys

reload(sys)
sys.setdefaultencoding('utf-8')


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


import random


# def send_dialog(name, intent, info_dict):
#     current_time = rospy.get_rostime()
#     # info_dict = {
#     #     "gender": "남성",
#     #     "age": "노인",
#     #     "sleep_status": "positive",
#     #     "disease_status": "positive",
#     #     "meal_menu": "설렁탕",
#     #     "take_medicine": "negative"
#     # }
#     msgs_dict = {
#         "header": {
#             "timestamp": "%i.%i" % (current_time.secs, current_time.nsecs),
#             "source": "planning",
#             "target": ["dialog"],
#             "content": ["dialog_generation"]
#         },
#         "dialog_generation": {
#             "id": random.randint(100, 199),
#             "name": name,
#             "intent": intent,
#             "social_context": {
#                 **info_dict
#             }
#         }
#     }
#     # print(msgs_dict)
#
#     json_string = json.dumps(msgs_dict, ensure_ascii=False, indent=4)
#     pub_task_topic.publish(json_string)


def callback_cmd(user_idx, speed=1):
    # start_flag = data.data
    # name = "User1"
    speech_user_1 = ["나는 슬퍼요.",
                     "머리를 다시 자르고 싶어",
                     "여전히 숙제를 끝내고 있습니다.",
                     "나는 지금 매우 지루합니다.",
                     "미술을 하고 나서 더이상 특별한 일이 없습니다.",
                     "군인들을 위한 쿠키를 만들었습니다.",
                     "파란새와 갈매기를 그리고 있습니다.",
                     "고양이가 침대에서 혼자 있는 방법을 생각합니다.",
                     "우리는 복잡한 일들에 직면해 있습니다.",
                     "할일이 너무 많아.",
                     "여동생과 팔씨름을 했습니다.",
                     "학교에 가고 싶지 않아요.",
                     "머리가 아파요.",
                     "수수께끼는 정말 재미있습니다.",
                     "학교는 짜증나.",
                     "기분이 안좋아요. 너무 너무 너무",
                     "내가 누구인지 잘 모르겠어요."
                     "독일어 숙제를 아직 하지 않았어요.",
                     "생생한 과일들이 나를 유혹해요!!!",
                     "아무것도 하고싶지 않아요."
                     ]

    speech_user_2 = ["모든 것을 알아 내려고 노력하고 있습니다.",
                     "생생한 꽃, 케이티 이온, 넷플릭스 즉석 영화의 작은 것들",
                     "그녀의 새로운 블랙 피코트와 사랑에 빠졌습니다. 또 누가 넘어갈까요?",
                     "마침내 내일 휴식을 취하기를 바라고있다!",
                     "술 크루즈 순회 공연에서 마이클 스콧의 춤이 정말 재미 있어요.",
                     "옛 친구와 함께 오래된 학교에서 새로운 곳으로 가게되어 기쁘게 생각합니다.",
                     "그녀는 일회용 카메라를 다시 사용하기 시작할 것입니다. 나는 스테파니에게 무엇이 나타날지 모른다는 게 매우 설렌다는 데 동의합니다",
                     "오늘과 이번 주말에 아주 흥분됩니다.",
                     "s g-ma는 가장 멋지다",
                     "나쁜 하루를 보내고 있습니다. 신발, 좋은 아보카도 햄버거가 지금 필요합니다.",
                     "여름 온도가 높습니다. 제이미, 조나, 미스터 지, 크리스 릴리에 푹 빠졌어",
                     "돈, 치아 청소, 버스, $40의 드레스, 따뜻하고 편안한 테마의 크리스마스 쇼핑, 그것들이 내가 얼마나 추운지 잊어버리게 합니다.",
                     "그녀가 사랑을 재발견하고 있다.",
                     "왜 NETFLIX에 대기열에 6개 이상의 DVD가 있어야합니까? 나는 5개가 충분하다고 생각합니다.",
                     "진정한 피가 할로윈 정신으로 나를 데려오고 있습니다.",
                     "재정 문제를 잊어 버리십시오.",
                     "제가 사용하는 침대에 문제가 있어요!!",
                     "저랑 파리에 함께 가실 분 계신가요?",
                     "강아지와 하는 산책이 너무 나를 행복하게 만들어요",
                     "개인적으로 화가나는 일이 있어서 며칠동안 연락이 되지 않습니다.",
                     "더이상 친구나 드라마에 의존하지 않을거에요.",
                     "엄마의 판단에 맞서기 위해 북쪽으로 갑니다.",
                     "나의 구세주는 샌들, 드레스, 셔츠입니다."
                     ]

    intent_list = ["saying_welcome",
                   "check_information_sleep",
                   "transmit_information_reaction",
                   "check_information_disease",
                   "check_information_meal",
                   "transmit_information_disease_advice",
                   "check_information_health",
                   "transmit_information_health_advice",
                   "saying_good_bye",
                   "saying_good_bye"
                   ]

    speech_user_3 = ["응 안녕",
                     "어 너구나. 잘잤다.",
                     "꿈 꿀 새도 없이 잠들어 버졌지.",
                     "고맙다.",
                     "좋아졌어 같아.",
                     "아 그럼 니가 맨날 말해주잖아.",
                     "오늘 아침 먹고 30분 후에 먹었지 죽이랑 시금치 무침 먹었어.",
                     "안 짜게 먹었어.",
                     "그렇지.",
                     "응 먹었다. 니 덕분에 약 시간 안놓치고 꾸준히 약 먹는다.",
                     "오냐.",
                     "",
                     # "어 다녀왔지.",
                     # "그렇지?",
                     # "아 어제 어쩐지 뭐 새로운 약을 주더라. 이것도 식후 30분 후에 먹는건가? 어제 들었는데도 내가 요즘 자꾸 깜빡해.",
                     # "오늘도 니가 약 먹을 시간 알려줘야 겠다.",
                     ]

    info_dict_list = [{},
                      {"name": "이병현", "gender": "남성", "age": "노인"},
                      {},
                      {"sleep_status": "positive"},
                      {"disease_name": "고혈압"},
                      {"": ""},
                      {"disease_name": "고혈압", "disease_advice": "싱거운 음식 섭취"},
                      {"": ""},
                      {"take_medicine_schedule": "식후 30분 후"},
                      {"": ""},
                      {"": ""},
                      ]

    speech_user_4 = ["네.",
                     "네.. 처음입니다.",
                     "아 그렇군요 저도 요즘 기관지염 때문에 고생입니다.",
                     "아 그게 알고 있는데도 물 마시는걸 자꾸 잊어버립니다.",
                     "가만히 있어보자 오늘 아침에 마셨나?",
                     "그래주면 고맙습니다.",
                     "네.",
                     "아이쿠 깜빡할뻔 했네. 물은 어디서 마실 수 있나요?",
                     "고마워요."
                     ]

    speech_user_5 = ["안녕하세요",
                     "최근에 기분이 너무 좋지 않아서 기분 전환 좀 할겸 최근에 미용실에 갔었어",
                     "그런데 예약을 하고 가지 않아서 사람이 엄청 많더라고 그래서 너무 짜증났어",
                     "거기다가 담당 미용사 분이 친절하지 않아서 머리 자르는 내내 불쾌했어",
                     "이번에 갔던 미용실은 다시는 가고 싶지 않아",
                     "정말 기분이 별로였어",
                     "그래도 머리를 자르고 나니 슬펐던 기분이 조금 나아지더라구",
                     "가끔씩 이런 전환이 필요한것 같아",
                     ]



    speech_user_6 = ["안녕 반가워",
                     "최근에 악기를 배우기 시작했어",
                     "뭘 배울까 고민하다가 기타를 배우기로 결정했어",
                     "혼자서 치는것보다 다른 사람들이 하는것도 보고 열심히 하고 싶어서 학원에 등록했어",
                     "생각했던것보다 분위기나 사람들이 너무 좋아서 굉장히 흥분 했었어",
                     "앞으로 정말 열심히 노력해서 잘하게 될꺼야",
                     "다른 사람들 앞에서 연주를 하는 것을 목표로 하고 있어",
                     "이게 최근에 가장 재미 있었던 일인것 같아",
                     ]



    if user_idx == 1:
        print("사용자의 이름을 입력해 주세요.")
        # test = raw_input("-> ")
        # print(test)
        # print(type(test))

        user_name = str(raw_input("-> "))

        print(user_name)
        while not rospy.is_shutdown():
            print("사용자의 발화문을 입력해 주세요.")
            speech = raw_input("-> ")
            # name = str(name)
            send_speech(user_name, str(speech))
            print(str(speech))

            print("Press Enter to continue or Type exit to terminate")
            end_flag = raw_input("")
            if end_flag == "exit":
                break

    if user_idx == 2:
        # 1.5, 4.4, 1.8, 2.0, 2.5
        # L, H, L, L ,L
        for speech in speech_user_1:
            name = "user1"
            send_speech(name, speech)
            rospy.sleep(speed)

    if user_idx == 3:
        # 3.4, 2.5, 3.5, 4.6, 4.8
        # M, M, M, H ,H
        for speech in speech_user_2:
            name = "user2"
            send_speech(name, speech)
            rospy.sleep(speed)

    if user_idx == 4:
        # 3.4, 2.5, 3.5, 4.6, 4.8
        # M, M, M, H ,H
        # for speech in speech_user_3:
        for idx in range(len(speech_user_3)):
            name = "이병현"
            # send_dialog(name, intent_list[idx], info_dict_list[idx])
            # rospy.sleep(speed)
            send_speech(name, speech_user_3[idx])
            rospy.sleep(speed)

    if user_idx == 5:
        # 3.4, 2.5, 3.5, 4.6, 4.8
        # M, M, M, H ,H
        for speech in speech_user_4:
            name = "강준한"
            send_speech(name, speech)
            rospy.sleep(speed)

    if user_idx == 6:
        # 1.5, 4.4, 1.8, 2.0, 2.5
        # L, H, L, L ,L
        for speech in speech_user_5:
            name = "persona1"
            send_speech(name, speech)
            print(speech)
            rospy.sleep(speed)

    if user_idx == 7:
        # 1.5, 4.4, 1.8, 2.0, 2.5
        # L, H, L, L ,L
        for speech in speech_user_6:
            name = "persona1"
            send_speech(name, speech)
            print(speech)
            rospy.sleep(speed)

    if user_idx == 0:
        # 3.4, 2.5, 3.5, 4.6, 4.8
        # M, M, M, H ,H
        PACKAGE_PATH = rospkg.RosPack().get_path("feature_handler") + "/scripts/"
        f = open(PACKAGE_PATH + "data.csv", 'w', encoding='utf-8')
        wr = csv.writer(f)
        wr.writerow(["", "name", "speech_en", "speech_kr"])
        f.close()
        print("Tokens DB Removed")

        # for speech in speech_user_4:
        #     name = "강준한"
        #     send_speech(name, speech)
        #     rospy.sleep(speed)


def terminal_loop():
    while True:
        print("=============================================")
        print("             STT Dummy ROS Node              ")
        print("=============================================")
        print("Mode를 설정해 주세요.                        ")
        print("1. 직접 입력                                 ")
        print("2. User1 [L,H,L,L,L] 재생                    ")
        print("3. User2 [M,M,M,H,H] 재생                    ")
        print("4. Persona : 이병헌 재생                    ")
        print("5. Persona : 강준한 재생                    ")
        print("0. DB 삭제                    ")
        mode = int(raw_input("-> "))

        if mode == 1:
            callback_cmd(1)
        elif mode == 2:
            callback_cmd(2, speed=1)
        elif mode == 3:
            callback_cmd(3, speed=1)
        elif mode == 4:
            callback_cmd(4, speed=1)
        elif mode == 5:
            callback_cmd(5, speed=1)
        elif mode == 6:
            callback_cmd(6, speed=1)
        elif mode == 7:
            callback_cmd(7, speed=1)
        elif mode == 0:
            callback_cmd(0)


def termination_handler(signal_received, frame):
    print('SIGINT or CTRL-C detected. Exiting gracefully')
    exit(0)


def dummy_stt_converter():
    global pub_recog_topic
    global pub_task_topic

    rospy.init_node('dummy_stt_converter', anonymous=False)
    # rospy.Subscriber("simulation_trigger", String, callback_cmd)
    pub_recog_topic = rospy.Publisher("recognitionResult", String, queue_size=100)
    # pub_dialog_topic = rospy.Publisher("dialogResult", String, queue_size=100)
    # pub_task_topic = rospy.Publisher("taskCompletion", String, queue_size=100)
    # pub_recog_topic = rospy.Publisher("recognitionResult", String, queue_size=100)
    # rospy.Subscriber("taskResult", String, callback_task)
    # pub_task_topic = rospy.Publisher("taskExecution", String, queue_size=100)

    terminal_loop()
    rospy.spin()
    # rospy.s


if __name__ == '__main__':
    signal(SIGINT, termination_handler)

    dummy_stt_converter()
