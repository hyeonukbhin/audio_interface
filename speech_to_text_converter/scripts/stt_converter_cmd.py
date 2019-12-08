#!/usr/bin/python3
# -*- coding: utf-8 -*-
import json
import rospy
from std_msgs.msg import String
import rospkg
import csv


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

def send_dialog(name, intent, info_dict):
    current_time = rospy.get_rostime()
    # info_dict = {
    #     "gender": "남성",
    #     "age": "노인",
    #     "sleep_status": "positive",
    #     "disease_status": "positive",
    #     "meal_menu": "설렁탕",
    #     "take_medicine": "negative"
    # }
    msgs_dict = {
        "header": {
            "timestamp": "%i.%i" % (current_time.secs, current_time.nsecs),
            "source": "planning",
            "target": ["dialog"],
            "content": ["dialog_generation"]
        },
        "dialog_generation": {
            "id": random.randint(100, 199),
            "name": name,
            "intent": intent,
            "social_context": {
                **info_dict
            }
        }
    }
    # print(msgs_dict)

    json_string = json.dumps(msgs_dict, ensure_ascii=False, indent=4)
    pub_task_topic.publish(json_string)



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

    # speech_user_2_1 = ["모든 것을 알아 내려고 노력하고 있습니다.",
    #                  "생생한 꽃, 케이티 이온, 넷플릭스 즉석 영화의 작은 것들",
    #                  "그녀의 새로운 블랙 피코트와 사랑에 빠졌습니다. 또 누가 넘어갈까요?",
    #                  "마침내 내일 휴식을 취하기를 바라고있다!",
    #                  "술 크루즈 순회 공연에서 마이클 스콧의 춤이 정말 재미 있어요.",
    #                  "새로운 곳에서 옛 친구와 함께 오래된 학교를 걷어차게되어 기쁘게 생각합니다.",
    #                  "그녀는 일회용 카메라를 다시 사용하기 시작할 것입니다. 그녀는 스테파니에게 무엇이 나타날지 모른다는 게 매우 설렌다는 데 동의합니다",
    #                  "오늘과 이번 주말에 아주 흥분됩니다.",
    #                  "s g-ma는 가장 멋지다",
    #                  "나쁜 하루를 보내고 있습니다. 대상, 신발, 좋은 아보카도 햄버거가 나를 데려 올 수있었습니다.",
    #                  "여름 온도가 높습니다. 제이미, 조나, 미스터 지. 크리스 릴리에 푹 빠졌어",
    #                  "돈, 치아 청소, 버스, 최고 비밀 사용, $ 40의 드레스, 따뜻하고 편안한 테마의 크리스마스 쇼핑, 밖이 얼마나 추운지 잊어 버립니다.",
    #                  "그녀의 사랑을 재발견하고 있다.",
    #                  "왜 netflix에 대기열에 6개 이상의 DVD가 있어야합니까? 나는 5개가 충분하다고 생각합니다.",
    #                  "진정한 피가 할로윈 정신으로 나를 데려오고 있습니다.",
    #                  "재정 문제를 잊어 버리십시오.",
    #                  "제가 사용하는 침대에 문제가 있어요!!",
    #                  "저랑 파리에 함께 가실 분 계신가요?",
    #                  "강아지와 하는 산책이 너무 나를 행복하게 만들어요",
    #                  "개인적으로 화가나는 일이 있어서 며칠동안 연락이 되지 않습니다.",
    #                  "더이상 친구나 드라마에 의존하지 않을거에요.",
    #                  "엄마의 판단에 맞서기 위해 북쪽으로 갑니다.",
    #                  "나의 구세주는 샌들, 드레스, 셔츠입니다."
    #                  ]
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
                    "어 너냐 잘잤다.",
                     "꿈 꿀 새도 없이 곯아 떨어졌지.",
                     "고맙다.",
                     "좋아진 것 같아.",
                     "아 그럼 니가 맨날 말해주잖아.",
                     "오늘 아침 먹고 30분 후에 먹었지 흰 죽이랑 시금치 무침 먹었어.",
                     "싱겁게 먹었어.",
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
                     "아 그게 알고 있는데도 물 마시는걸 자꾸 까먹습니다.",
                     "가만히 있어보자 오늘 아침에 마셨나?",
                     "그래주면 고맙죠.",
                     "네.",
                     "아이쿠 깜빡할뻔 했네. 물은 어디서 마실 수 있나요?",
                     "고마워요."
                     ]

    if user_idx == 1:
        print("사용자의 이름을 입력해 주세요.")
        user_name = str(input("-> "))

        print(user_name)
        while True:
            print("사용자의 발화문을 입력해 주세요.")
            speech = input("-> ")
            # name = str(name)
            send_speech(user_name, str(speech))
            print("Press Enter to continue or Type exit to terminate")
            end_flag = input("")
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
            send_dialog(name, intent_list[idx], info_dict_list[idx])
            rospy.sleep(speed)
            send_speech(name, speech_user_3[idx])
            rospy.sleep(speed)

    if user_idx == 5:
        # 3.4, 2.5, 3.5, 4.6, 4.8
        # M, M, M, H ,H
        for speech in speech_user_4:
            name = "강준한"
            send_speech(name, speech)
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
        mode = int(input("-> "))

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
        elif mode == 0:
            callback_cmd(0)


def scenario_simulator():
    global pub_recog_topic
    global pub_task_topic

    rospy.init_node('dummy_stt_converter', anonymous=False)
    # rospy.Subscriber("simulation_trigger", String, callback_cmd)
    pub_recog_topic = rospy.Publisher("recognitionResult", String, queue_size=100)
    # pub_dialog_topic = rospy.Publisher("dialogResult", String, queue_size=100)
    # pub_task_topic = rospy.Publisher("taskCompletion", String, queue_size=100)
    # pub_recog_topic = rospy.Publisher("recognitionResult", String, queue_size=100)
    # rospy.Subscriber("taskResult", String, callback_task)
    pub_task_topic = rospy.Publisher("taskExecution", String, queue_size=100)


    terminal_loop()
    rospy.spin()
    # rospy.s


if __name__ == '__main__':
    scenario_simulator()
