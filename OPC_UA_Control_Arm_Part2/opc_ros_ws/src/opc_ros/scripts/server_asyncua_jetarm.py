#!/usr/bin/env python3.8

import logging
import netifaces as ni
import asyncio
import traceback
import math
import threading
import ast
import base64
import actionlib
from PIL import Image as PILImage
# import cv2
from io import BytesIO
import numpy as np
import os
# import hiwonder
# import buzzer

from sensor_msgs.msg import Image
import rospy
from std_srvs.srv import Empty, Trigger
from std_srvs.srv import SetBool, SetBoolResponse, SetBoolRequest
from std_msgs.msg import Bool, UInt16, Int32, String
# from opc_ros.data import SetServo, JetMax, SetJetMax
# from opc_ros.srv import SetTarget, SetTargetResponse, SetTargetRequest
# from opc_ros.srv import SetTarget_object, SetTarget_objectResponse, SetTarget_objectRequest
# from opc_ros.srv import ActionSetFileOp, ActionSetFileOpRequest, ActionSetFileOpResponse
# from opc_ros.srv import ActionSetList, ActionSetListRequest, ActionSetListResponse
# from opc_ros.data import ActionSetRawAction, ActionSetRawFeedback, ActionSetRawActionGoal, ActionSetRawResult, \
#     ActionSetRawGoal
# from actionlib_datas.data import GoalID

import asyncua
from asyncua import ua, uamethod, Server

interfaces = ni.interfaces()
interface = None
if "ap0" in interfaces:
    interface = "ap0"
elif "wlan0" in interfaces:
    interface = "wlan0"
else:
    interface = "eth0"
_logger = logging.getLogger(__name__)
IPAddr = ni.ifaddresses(interface)[ni.AF_INET][0]['addr']
# IPAddr = ni.ifaddresses('eth0')[ni.AF_INET][0]['addr']
# IPAddr = ni.ifaddresses('ap0')[ni.AF_INET][0]['addr']

CERTIFICATE_PATH = "/home/hiwonder/opc_ros_ws/src/opc_ros/scripts/CA_Private_key/temp/certificate.der"
PRIVATE_KEY_PATH = "/home/hiwonder/opc_ros_ws/src/opc_ros/scripts/CA_Private_key/temp/privatekey.pem"

MIN_MIDSERVO = -90
MAX_MIDSERVO = 90

MIN_LEFTSERVO = -60
MAX_LEFTSERVO = 87

MIN_RIGHTSERVO = -60
MAX_RIGHTSERVO = 60

MIN_NECK = -90
MAX_NECK = 90

MIN_HEAD = -90
MAX_HEAD = 120

DURATION = 0.45

midServo = 0
leftServo = 0
rightServo = 0
neckServo = 0
headServo = 90
suckcup = False
speed = 5
cordinateSpeed = math.floor(speed / 2)
oldSpeed = speed
x = 0
y = 0
z = 0

oldData = None
oldDataAIEnter = None
oldDataAIRun = None

actionlibClient = None

methodDict = {}
subscriptionList = []

imageQueue = []
imageQueue2 = []
imageTopicDefault = "/isCamOpened"
imageTopic = ""
imageTopic2 = "/usb_cam_1/image_raw"
imageSubscription = ""

taskPool = []

JetMaxControlList = list()
#  0 -> mid
#  1 -> left
#  2 -> right
#  3 -> neck
#  4 -> head
#  5 -> sucker
#  6 -> position command
#  7 -> relative position command
#  8 -> buzzer

AI_SERVICE_NAME = ["palletizing",
                   "waste_classification",
                   "object_tracking",
                   "color_sorting",
                   "fruits_classification"
                   ]
AI_COLOR = ["red", "green", "blue"]

# -1: decrease angle, 0: stop, 1: increase angle
SERVO_LOCK_AUTORUN = {
    "mid": 0,
    "left": 0,
    "right": 0,
    "neck": 0,
    "head": 0
}

CORDINATE_LOCK_AUTORUN = {
    "x": 0,
    "y": 0,
    "z": 0
}


def create_Ua_Argument(Name: str, Datatype: ua.NodeId, Description: str):
    """
    This function create an UA argument used to add more info to the method.
    """
    arg = ua.Argument()
    arg.Name = Name
    arg.DataType = Datatype
    arg.ValueRank = -1
    arg.ArrayDimensions = []
    arg.Description = ua.LocalizedText(Description)
    return arg


def type_conversion(list_of_string: list) -> list:
    """
    This function convert a string object into python type (if possible)
    """
    result = []
    for string in list_of_string:
        try:
            result.append(ast.literal_eval(string))
        except:
            result.append(string)
    return result


def imageConnectTopic(topic):
    global imageTopic, imageSubscription, imageTopicDefault
    imageSubscription.unregister()
    if (topic == imageTopicDefault):
        imageTopic = topic
    else:
        imageTopic = f'/{topic}/image_result'
    imageSubscription = rospy.Subscriber(imageTopic, String, imageCallback)


def imageCallback(img):
    # print(type(img.data))
    imageQueue.append(img.data)
    if (len(imageQueue) > 2):
        imageQueue.clear()
        
def imageCallback2(img):
    # print(type(img.data))
    imageQueue2.append(img.data)
    if (len(imageQueue2) > 2):
        imageQueue2.clear()


"""
================================= uamethod section =================================
"""


@uamethod
def moveServo(parent, servo_name: str, angle_increase: bool):
    global midServo, leftServo, rightServo, neckServo, headServo, speed, JetMaxControlList
    # print(f'{servo_name}, {angle_increase}')
    if servo_name == "mid":
        if not angle_increase and midServo > MIN_MIDSERVO:
            midServoTemp = midServo - speed if (midServo - speed >= MIN_MIDSERVO) else MIN_MIDSERVO
            JetMaxControlList[0].publish(String(data="000002"[::-1]))
        elif angle_increase and midServo < MAX_MIDSERVO:
            midServoTemp = midServo + speed if (midServo + speed <= MAX_MIDSERVO) else MAX_MIDSERVO
            JetMaxControlList[0].publish(String(data="000001"[::-1]))

    elif servo_name == "left":
        if not angle_increase and leftServo > MIN_LEFTSERVO:
            leftServoTemp = leftServo - speed if (leftServo - speed >= MIN_LEFTSERVO) else MIN_LEFTSERVO
            JetMaxControlList[0].publish(String(data="000020"[::-1]))
        elif angle_increase and leftServo < MAX_LEFTSERVO:
            leftServoTemp = leftServo + speed if (leftServo + speed <= MAX_LEFTSERVO) else MAX_LEFTSERVO
            JetMaxControlList[0].publish(String(data="000010"[::-1]))

    elif servo_name == "right":
        if not angle_increase and rightServo > MIN_RIGHTSERVO:
            rightServoTemp = rightServo - speed if (rightServo - speed >= MIN_RIGHTSERVO) else MIN_RIGHTSERVO
            JetMaxControlList[0].publish(String(data="000200"[::-1]))
        elif angle_increase and rightServo < MAX_RIGHTSERVO:
            rightServoTemp = rightServo + speed if (rightServo + speed <= MAX_RIGHTSERVO) else MAX_RIGHTSERVO
            JetMaxControlList[0].publish(String(data="000100"[::-1]))

    elif servo_name == "neck":
        if not angle_increase and neckServo > MIN_NECK:
            neckServoTemp = neckServo - speed if (neckServo - speed >= MIN_NECK) else MIN_NECK
            JetMaxControlList[0].publish(String(data="002000"[::-1]))
        elif angle_increase and neckServo < MAX_NECK:
            neckServoTemp = neckServo + speed if (neckServo + speed <= MAX_NECK) else MAX_NECK
            JetMaxControlList[0].publish(String(data="001000"[::-1]))

    elif servo_name == "head":
        if not angle_increase and headServo > MIN_HEAD:
            headServoTemp = headServo - speed if (headServo - speed >= MIN_HEAD) else MIN_HEAD
            JetMaxControlList[0].publish(String(data="020000"[::-1]))
        elif angle_increase and headServo < MAX_HEAD:
            headServoTemp = headServo + speed if (headServo + speed <= MAX_HEAD) else MAX_HEAD
            JetMaxControlList[0].publish(String(data="010000"[::-1]))
    else:
        print(f"NOTOK")
        return f"NOTOK"
    print(f"{servo_name};{angle_increase};OK")
    return f"{servo_name};{angle_increase};OK"


@uamethod
def autoServo(parent, servo_name: str, direction: int):
    global SERVO_LOCK_AUTORUN
    if direction < -1 or direction > 1:
        return "NOTOK"
    if servo_name in SERVO_LOCK_AUTORUN:
        SERVO_LOCK_AUTORUN[servo_name] = direction
        return "OK"
    else:
        return "NOTOK"


@uamethod
def changeSpeed(parent, spd: int):
    global speed, cordinateSpeed, oldSpeed
    if spd < 1 or spd > 30:
        return "NOTOK"
    oldSpeed = speed
    speed = spd
    cordinateSpeed = math.floor(speed / 2)
    if cordinateSpeed < 1:
        cordinateSpeed = 1
    return "OK"


@uamethod
def sucker(parent, suck: bool):
    JetMaxControlList[5].publish(Bool(data=suck))
    return "OK"


@uamethod
def moveCordinate(parent, cordinate: str, increase: bool):
    global cordinateSpeed
    if cordinate == "x":
        if increase:
            JetMaxControlList[7].publish(SetJetMax(x=cordinateSpeed, y=0, z=0, duration=DURATION))
        else:
            JetMaxControlList[7].publish(SetJetMax(x=-cordinateSpeed, y=0, z=0, duration=DURATION))
    elif cordinate == "y":
        if increase:
            JetMaxControlList[7].publish(SetJetMax(x=0, y=cordinateSpeed, z=0, duration=DURATION))
        else:
            JetMaxControlList[7].publish(SetJetMax(x=0, y=-cordinateSpeed, z=0, duration=DURATION))
    elif cordinate == "z":
        if increase:
            JetMaxControlList[7].publish(SetJetMax(x=0, y=0, z=cordinateSpeed, duration=DURATION))
        else:
            JetMaxControlList[7].publish(SetJetMax(x=0, y=0, z=-cordinateSpeed, duration=DURATION))
    else:
        return f'NOTOK; cordinate can only be x,y,z'
    return f'{cordinate};{increase};OK'


@uamethod
def autoCordinate(parent, cordinate: str, direction: int):
    global CORDINATE_LOCK_AUTORUN
    if direction < -1 or direction > 1:
        return "NOTOK"
    if cordinate in CORDINATE_LOCK_AUTORUN:
        CORDINATE_LOCK_AUTORUN[cordinate] = direction
        return "OK"
    else:
        return "NOTOK"


@uamethod
def setCordinate(parent, xc: float, yc: float, zc: float):
    JetMaxControlList[6].publish(SetJetMax(x=xc, y=yc, z=zc, duration=0.5))
    return "OK"


@uamethod
def goHome(parent):
    JetMaxControlList[0].publish(String(data="gohome"))
    return "OK"

@uamethod
def goFold(parent):
    JetMaxControlList[0].publish(String(data="gofold"))
    return "OK"


@uamethod
def AIservice(parent, service_name: str, enter: bool):
    """
    list of AI service name available:
        palletizing
        ...
    """
    global AI_SERVICE_NAME, imageTopicDefault
    if service_name not in AI_SERVICE_NAME:
        return "NOTOK"
    if enter:
        proxy = rospy.ServiceProxy(f'/{service_name}/enter', Trigger)
        imageConnectTopic(service_name)
    else:
        proxy = rospy.ServiceProxy(f'/{service_name}/exit', Trigger)
        imageConnectTopic(imageTopicDefault)
    proxy()
    return "OK"


@uamethod
def AIserviceRun(parent, service_name: str, run: bool):
    """
    list of AI service name available:
        palletizing
        waste_classification
        object_tracking (need more param)
        color_sorting (need more param)
        ...
    """
    if service_name not in AI_SERVICE_NAME:
        return "NOTOK"
    if service_name == "object_tracking":
        proxy = rospy.ServiceProxy(f'/{service_name}/set_running_color', SetBool)
    else:
        proxy = rospy.ServiceProxy(f'/{service_name}/set_running', SetBool)

    if run:
        proxy(SetBoolRequest(data=True))
    else:
        proxy(SetBoolRequest(data=False))
    return "OK"



@uamethod
def AIsetTarget(parent, service_name: str, color: str, en: bool):
    """
    list of color name available:
        red
        blue
        green
    """
    if service_name not in AI_SERVICE_NAME or color not in AI_COLOR:
        return "NOTOK"
    try:
        if service_name == "object_tracking":
            proxy = rospy.ServiceProxy(f'/{service_name}/set_target', SetTarget_object)
            proxy(SetTarget_objectRequest(color_name=color))
        else:
            proxy = rospy.ServiceProxy(f'/{service_name}/set_target', SetTarget)
            proxy(SetTargetRequest(color_name=color, is_enable=en))
    except Exception:
        traceback.print_exc()
    return "OK"


@uamethod
def waste_class(parent, waste_class: str):
    pub = rospy.Publisher('/waste_classification/set_waste_class', String, queue_size=1)
    pub.publish(String(data=waste_class))
    return "OK"


@uamethod
def set_suck_waste(parent, enter: int):
    pub = rospy.Publisher('/waste_classification/set_suck', UInt16, queue_size=1)
    pub.publish(UInt16(data=enter))
    return "OK"


@uamethod
def change_mode_control(parent, enter: int):
    for i in range(0, enter):
        JetMaxControlList[8].publish(Bool(data=True))
        rospy.sleep(0.1)
        JetMaxControlList[8].publish(Bool(data=False))
        rospy.sleep(0.5)
    return "OK"


"""
================================= /uamethod section =================================
"""


# replace server.start()
async def main():
    """
    ================================= asyncua section =================================
    """
    # server.init()
    
    

    global imageSubscription
    imageSubscription = rospy.Subscriber(imageTopicDefault, String, imageCallback)

    server = Server()
    
    await server.init()
    # endpoint: address to connect to
    server.set_endpoint(f"opc.tcp://{IPAddr}:4841")
    server.set_server_name("Robot Arm Server")

    # set all possible endpoint policies for client to connect through
    server.set_security_policy(
        [
            ua.SecurityPolicyType.NoSecurity,
            ua.SecurityPolicyType.Basic256Sha256_SignAndEncrypt,
            ua.SecurityPolicyType.Basic256Sha256_Sign,
        ]
    )
    # await server.load_certificate(CERTIFICATE_PATH)
    # await server.load_private_key(PRIVATE_KEY_PATH)

    # setup our namespace. An endpoint can have multiple namespace!
    uri = "http://robotarm.asyncua.io"
    idx = await server.register_namespace(uri)

    # add some nodes
    rootFolder = await server.nodes.objects.add_folder(idx, "Robot Arm")

    servo_folder = await rootFolder.add_folder(idx, "Servo")
    midServoNode = await servo_folder.add_variable(idx, "mid", 0, varianttype=ua.VariantType.Double)
    leftServoNode = await servo_folder.add_variable(idx, "left", 0, varianttype=ua.VariantType.Double)
    rightServoNode = await servo_folder.add_variable(idx, "right", 0, varianttype=ua.VariantType.Double)

    add_on_folder = await rootFolder.add_folder(idx, "Add-on")
    neckServoNode = await add_on_folder.add_variable(idx, "neck", 0, varianttype=ua.VariantType.Double)
    headServoNode = await add_on_folder.add_variable(idx, "head", 90.0, varianttype=ua.VariantType.Double)
    suckNode = await add_on_folder.add_variable(idx, "suckcup", False)
    sliderNode = await add_on_folder.add_variable(idx, "slider", "")

    cordinateFolder = await rootFolder.add_folder(idx, "Cordinate")
    xNode = await cordinateFolder.add_variable(idx, "x", 0.0, varianttype=ua.VariantType.Double)
    yNode = await cordinateFolder.add_variable(idx, "y", 0.0, varianttype=ua.VariantType.Double)
    zNode = await cordinateFolder.add_variable(idx, "z", 0.0, varianttype=ua.VariantType.Double)

    speedNode = await rootFolder.add_variable(idx, "speed", 5)

    # ---------------------------------------- Image node------------------------------------------------------
    imageFolder = await rootFolder.add_folder(idx, "Image")
    imageJetmaxNode = await imageFolder.add_variable(idx, "imageJetmax", b"a", datatype=ua.ObjectIds.ImageJPG)
    imageUSBCamNode = await imageFolder.add_variable(idx, "imageUSBCam", b"a", datatype=ua.ObjectIds.ImageJPG)

    # ----------------------------------------------------------------------------------------------------------

    # -----------------------------------------AI current state---------------------------------------

    AIfolderNode = await rootFolder.add_folder(idx, "AICurrent")

    AIEnterCurrentNode = await AIfolderNode.add_variable(idx, "AIEnterCurrent", "")

    AIRunCurrentNode = await AIfolderNode.add_variable(idx, "AIRunCurrent", "")

    AIColorCurrentNode = await AIfolderNode.add_variable(idx, "AIColorCurrent", "")

    AIWasteTypeCurrentNode = await AIfolderNode.add_variable(idx, "AIWasteType", "")

    AIWasteStateCurrentNode = await AIfolderNode.add_variable(idx, "AIWasteState", 1)

    # -----------------------------------------------------------------------------------------------------

    methodFolder = await rootFolder.add_folder(idx, "Method")

    controlMethod = await methodFolder.add_folder(idx, "Control Method")

    inargx = create_Ua_Argument(Name="Servo", Datatype=ua.NodeId(ua.ObjectIds.String),
                                Description="Servo name: mid, left, right, neck, head")
    inargy = create_Ua_Argument(Name="Increase angle", Datatype=ua.NodeId(ua.ObjectIds.Boolean),
                                Description="Increase the angle of servo")
    outarg = create_Ua_Argument(Name="Ack", Datatype=ua.NodeId(ua.ObjectIds.String),
                                Description="Acknowledgement")

    methodDict["moveServo"] = await controlMethod.add_method(idx,
                                                             "moveServo",
                                                             moveServo,
                                                             [inargx, inargy],
                                                             [outarg])

    inargx = create_Ua_Argument(Name="Servo", Datatype=ua.NodeId(ua.ObjectIds.String),
                                Description="Servo name: mid, left, right, neck, head")
    inargy = create_Ua_Argument(Name="Direction", Datatype=ua.NodeId(ua.ObjectIds.Int16),
                                Description="Direction of servo, -1 to decrease, 0 to stop and 1 to increase")

    methodDict["autoServo"] = await controlMethod.add_method(idx,
                                                             "autoServo",
                                                             autoServo,
                                                             [inargx, inargy],
                                                             [outarg])

    inargx = create_Ua_Argument(Name="Speed", Datatype=ua.NodeId(ua.ObjectIds.Int16),
                                Description="Speed of servo, a value from 1 to 20")

    methodDict["changeSpeed"] = await controlMethod.add_method(idx,
                                                               "changeSpeed",
                                                               changeSpeed,
                                                               [inargx],
                                                               [outarg])

    inargx = create_Ua_Argument(Name="Sucker", Datatype=ua.NodeId(ua.ObjectIds.Boolean),
                                Description="Set sucker on or off")

    methodDict["sucker"] = await controlMethod.add_method(idx,
                                                          "sucker",
                                                          sucker,
                                                          [inargx],
                                                          [outarg])

    inargx = create_Ua_Argument(Name="Cordinate", Datatype=ua.NodeId(ua.ObjectIds.String),
                                Description="Cordinate name: x, y, z")
    inargy = create_Ua_Argument(Name="Increase cordinate", Datatype=ua.NodeId(ua.ObjectIds.Boolean),
                                Description="Increase the cordinate of x, y, or z direction")

    methodDict["moveCordinate"] = await controlMethod.add_method(idx,
                                                                 "moveCordinate",
                                                                 moveCordinate,
                                                                 [inargx, inargy],
                                                                 [outarg])

    inargx = create_Ua_Argument(Name="Cordinate", Datatype=ua.NodeId(ua.ObjectIds.String),
                                Description="Cordinate name: x, y, z")
    inargy = create_Ua_Argument(Name="Direction", Datatype=ua.NodeId(ua.ObjectIds.Int16),
                                Description="Direction of cordinate move, -1 to decrease, 0 to stop and 1 to increase")

    methodDict["autoCordinate"] = await controlMethod.add_method(idx,
                                                                 "autoCordinate",
                                                                 autoCordinate,
                                                                 [inargx, inargy],
                                                                 [outarg])

    inargx = create_Ua_Argument(Name="X", Datatype=ua.NodeId(ua.ObjectIds.Float),
                                Description="X axis value")
    inargy = create_Ua_Argument(Name="Y", Datatype=ua.NodeId(ua.ObjectIds.Float),
                                Description="Y axis value")
    inargz = create_Ua_Argument(Name="Z", Datatype=ua.NodeId(ua.ObjectIds.Float),
                                Description="Z axis value")

    methodDict["setCordinate"] = await controlMethod.add_method(idx,
                                                                "setCordinate",
                                                                setCordinate,
                                                                [inargx, inargy, inargz],
                                                                [outarg])

    methodDict["goHome"] = await controlMethod.add_method(idx,
                                                          "goHome",
                                                          goHome,
                                                          [],
                                                          [outarg])
    
    methodDict["goFold"] = await controlMethod.add_method(idx,
                                                          "goFold",
                                                          goFold,
                                                          [],
                                                          [outarg])
    # ----------------------------------------------CHANGE MODE CONTROL------------------------------------------------------------------#
    inargx = create_Ua_Argument(Name="Enter service", Datatype=ua.NodeId(ua.ObjectIds.UInt16),
                                Description="State available: 1 2")

    methodDict["changeModeControl"] = await controlMethod.add_method(idx,
                                                                     "changeModeControl",
                                                                     change_mode_control,
                                                                     [inargx],
                                                                     [outarg])
    # -----------------------------------------------------------------------------------------------------------------------------------#

    inargx = create_Ua_Argument(Name="AI Service Name", Datatype=ua.NodeId(ua.ObjectIds.String),
                                Description="AI service name available: palletizing,\ncolor_sorting\nobject_tracking\nwaste_classification\nfruits_classification\n ...")
    inargy = create_Ua_Argument(Name="Enter service", Datatype=ua.NodeId(ua.ObjectIds.Boolean),
                                Description="Enter or Quit service")

    AIMethod = await methodFolder.add_folder(idx, "AI Method")

    methodDict["AIservice"] = await AIMethod.add_method(idx,
                                                        "AIservice",
                                                        AIservice,
                                                        [inargx, inargy],
                                                        [outarg])

    inargx = create_Ua_Argument(Name="AI Service Name", Datatype=ua.NodeId(ua.ObjectIds.String),
                                Description="AI service name available: palletizing,\ncolor_sorting\nobject_tracking\nwaste_classification\nfruits_classification\n ...")
    inargy = create_Ua_Argument(Name="Start service", Datatype=ua.NodeId(ua.ObjectIds.Boolean),
                                Description="Start or Stop service. Service must be Enter before use")

    methodDict["AIserviceRun"] = await AIMethod.add_method(idx,
                                                           "AIserviceRun",
                                                           AIserviceRun,
                                                           [inargx, inargy],
                                                           [outarg])

    inargx = create_Ua_Argument(Name="AI Service Name", Datatype=ua.NodeId(ua.ObjectIds.String),
                                Description="AI service name available: \ncolor_sorting\nobject_tracking\n ...")
    inargy = create_Ua_Argument(Name="Color name", Datatype=ua.NodeId(ua.ObjectIds.String),
                                Description="Color available: red, green, blue")
    inargz = create_Ua_Argument(Name="Enable color", Datatype=ua.NodeId(ua.ObjectIds.Boolean),
                                Description="Enable or disable color selection.\n"
                                            "This method must be call before running\n"
                                            "color_sorting or object tracking")

    methodDict["AIsetTarget"] = await AIMethod.add_method(idx,
                                                          "AIsetTarget",
                                                          AIsetTarget,
                                                          [inargx, inargy, inargz],
                                                          [outarg])

    # ------------------------------------------------------------------------------------------

    # ActionSetMethod = methodFolder.add_folder(idx, "ActionSetMethod")
    #
    # methodDict["getActionSetList"] = ActionSetMethod.add_method(idx,
    #                                                             "getActionSetList",
    #                                                             getActionSetList,
    #                                                             [],
    #                                                             [outarg])
    #
    # inargx = create_Ua_Argument(Name="Action set name", Datatype=ua.NodeId(ua.ObjectIds.String),
    #                             Description="Run a actionset which name exist from getActionSetList")
    # inargy = create_Ua_Argument(Name="Repeat", Datatype=ua.NodeId(ua.ObjectIds.Int16),
    #                             Description="How many time the action will be repeated")
    #
    # methodDict["runActionSet"] = ActionSetMethod.add_method(idx,
    #                                                         "runActionSet",
    #                                                         runActionSet,
    #                                                         [inargx, inargy],
    #                                                         [outarg])
    # ----------------------------------------WASTE CLASSIFICATION CUSTOM ---------------------------------------------------------------#

    inargx = create_Ua_Argument(Name="Waste_Class", Datatype=ua.NodeId(ua.ObjectIds.String),
                                Description="Waste_Classes available: food_waste,\nhazardous_waste\nrecyclable_waste\nresidual_waste\n")
    inargy = create_Ua_Argument(Name="Enter service", Datatype=ua.NodeId(ua.ObjectIds.UInt16),
                                Description="State available: 1(idle) 2(suck) 3(hold) 4(release)")

    wc_custom = await methodFolder.add_folder(idx, "Waste_classification_custom")

    methodDict["Waste_Classes"] = await wc_custom.add_method(idx,
                                                             "Waste_Class",
                                                             waste_class,
                                                             [inargx],
                                                             [outarg])

    methodDict["suck_waste"] = await wc_custom.add_method(idx,
                                                          "suck_waste",
                                                          set_suck_waste,
                                                          [inargy],
                                                          [outarg])

    # -----------------------------------------------------------------------------------------------------------------------------------#

    async def autoRun():
        """
        This function will run in a loop which control the arm to
        move automatically. To do that, use call method [autoServo]
        to enable the lock.
        :return: None
        """
        global SERVO_LOCK_AUTORUN
        while not rospy.is_shutdown():
            # rospy is shutdown when ctrl - C is pressed!
            # su dung rospy.is_shutdown() nham de stop script khi nhan ctrl - C. Neu
            # khong xai dieu kien nay, script se khong bao gio dung khi bam ctrl - C
            try:
                if SERVO_LOCK_AUTORUN["left"] != 0:
                    moveServo(idx, "left",
                              ua.Variant(True, ua.VariantType.Boolean) if SERVO_LOCK_AUTORUN["left"] == 1 
                              else ua.Variant(False, ua.VariantType.Boolean))
                if SERVO_LOCK_AUTORUN["right"] != 0:
                    moveServo(idx, "right",
                              ua.Variant(True, ua.VariantType.Boolean) if SERVO_LOCK_AUTORUN["right"] == 1 
                              else ua.Variant(False, ua.VariantType.Boolean))
                if SERVO_LOCK_AUTORUN["mid"] != 0:
                    moveServo(idx, "mid",
                              ua.Variant(True, ua.VariantType.Boolean) if SERVO_LOCK_AUTORUN["mid"] == 1 
                              else ua.Variant(False, ua.VariantType.Boolean))
                if SERVO_LOCK_AUTORUN["neck"] != 0:
                    moveServo(idx, "neck",
                              ua.Variant(True, ua.VariantType.Boolean) if SERVO_LOCK_AUTORUN["neck"] == 1 
                              else ua.Variant(False, ua.VariantType.Boolean))
                if SERVO_LOCK_AUTORUN["head"] != 0:
                    moveServo(idx, "head",
                              ua.Variant(True, ua.VariantType.Boolean) if SERVO_LOCK_AUTORUN["head"] == 1 
                              else ua.Variant(False, ua.VariantType.Boolean))
                if CORDINATE_LOCK_AUTORUN["x"] != 0:
                    moveCordinate(idx, "x",
                                  ua.Variant(True, ua.VariantType.Boolean) if CORDINATE_LOCK_AUTORUN["x"] == 1 
                                  else ua.Variant(False, ua.VariantType.Boolean))
                if CORDINATE_LOCK_AUTORUN["y"] != 0:
                    moveCordinate(idx, "y",
                                  ua.Variant(True, ua.VariantType.Boolean) if CORDINATE_LOCK_AUTORUN["y"] == 1 
                                  else ua.Variant(False, ua.VariantType.Boolean))
                if CORDINATE_LOCK_AUTORUN["z"] != 0:
                    moveCordinate(idx, "z",
                                  ua.Variant(True, ua.VariantType.Boolean) if CORDINATE_LOCK_AUTORUN["z"] == 1
                                  else ua.Variant(False, ua.VariantType.Boolean))
                await asyncio.sleep(DURATION)
                # print("sleep 5 seconds")
            except Exception:
                traceback.print_exc()
        print("Exit autoRun")

    async def updateVideoJetMax():
        while not rospy.is_shutdown():
            if len(imageQueue) > 0:
                data = imageQueue.pop(0)
                # imageData = PILImage(base64.b64decode(data))
                # compressed_image = BytesIO()
                # imageData.save(compressed_image, format="JPEG", quality=15, optimize=True)
                # # print(len(compressed_image.getvalue()), flush=True)
                # await imageJetmaxNode.write_value(compressed_image.getvalue())
                await imageJetmaxNode.write_value(base64.b64decode(data))
            await asyncio.sleep(0.05)

    async def updateVideoUSBCam():        
        while not rospy.is_shutdown():
            if len(imageQueue2) > 0:
                data = imageQueue2.pop(0)
                imageData = PILImage.frombytes(mode="RGB",
                                               size=(320, 240),
                                               data=data,
                                               decoder_name="raw")
                compressed_image = BytesIO()
                imageData.save(compressed_image, format="JPEG", quality=30, optimize=True)
                # print(len(compressed_image.getvalue()), flush=True)
                await imageUSBCamNode.write_value(compressed_image.getvalue())
            await asyncio.sleep(0.05)
            
    async def updateAsyncUA():
        global speed, oldSpeed
        while not rospy.is_shutdown():
            if speed != oldSpeed:
                await speedNode.write_value(speed)
            # more update ...
            await asyncio.sleep(0.02)

    async def serverStart():
        print("serveropc", threading.current_thread().getName())
        async with server:
            print(f'OPC server running at {server.endpoint[0]}://{server.endpoint[1]}')
            # subscrition chi chay duoc sau khi Server.start()
            # handler = SubHandler()
            # subscription = server.create_subscription(1, handler)
            # subscriptionList.extend([unityNode])
            # print(subscriptionList)
            # subscription.subscribe_data_change(subscriptionList)
            # asyncio.gather giup chay 2 ham async 1 cach "song song"
            # await asyncio.gather(autoRun(), updateVideoJetMax())
            await asyncio.gather(autoRun(), updateVideoJetMax(),
                                 updateVideoUSBCam(), updateAsyncUA())

        print("Exit server")

    """
    ================================= /asyncua section =================================
    """

    """
    ================================= ros section =================================
    """

    async def updateRobot(data):
        """
        function to read Robot Arm angle data and update to asyncua object model (asyncua server)
        """
        data = data.data
        global midServo, leftServo, rightServo, neckServo, headServo, suckcup, oldData, x, y, z
        # print(data)
        if oldData == data:
            # print("The same data")
            return
        else:
            if (data.__contains__("<") and data.__contains__(">") and data.__contains__("{") and data.__contains__("}")):
                startIndex = data.index("{")
                endIndex = data.index("}")
                firstPart = data[startIndex: endIndex - startIndex].strip('{').strip('}').strip(':');
                data = data[endIndex+1:]
                startIndex = data.index("{")
                endIndex = data.index("}")
                secondPart = data[startIndex: endIndex - startIndex].strip('{').strip('}').strip(':');
                # print(firstPart)
                # print(secondPart)
                firstNumbers = firstPart.split(':');
                # print(firstNumbers)
                secondNumbers = secondPart.split(':');
                # print(secondNumbers)
                if (len(firstNumbers) == len(secondNumbers)):
                    for i in range(0, len(firstNumbers)):
                        # try:
                        # print(f'{secondNumbers[i]} - {firstNumbers[i]}')
                        secondNumber = float(secondNumbers[i])
                        firstNumber = float(firstNumbers[i])
                        if (i == 0):
                            midServo =  secondNumber
                        elif (i == 1):
                            leftServo = secondNumber
                        elif (i == 2):
                            rightServo = secondNumber
                        elif (i == 3):
                            neckServo = secondNumber
                        elif (i == 4):
                            headServo = secondNumber
                        # except Exception:
                        #     print("Failed to parse numbers.")
            asyncioLoop = asyncio.get_running_loop()
            taskSet = set()
            #todo
            # midServo = data.servo1
            # leftServo = data.servo2
            # rightServo = data.servo3
            # neckServo = data.pwm1
            # headServo = data.pwm2
            # suckcup = data.sucker
            # x = data.x
            # y = data.y
            # z = data.z
            # print(midServo, type(midServo))
            # print(f'{midServo} : {leftServo} : {rightServo} : {neckServo} : {headServo}')
            try:
                taskSet.add(asyncioLoop.create_task(midServoNode.write_value(float(midServo))))
                taskSet.add(asyncioLoop.create_task(leftServoNode.write_value(float(leftServo))))
                taskSet.add(asyncioLoop.create_task(rightServoNode.write_value(float(rightServo))))
                taskSet.add(asyncioLoop.create_task(neckServoNode.write_value(float(neckServo))))
                # taskSet.add(asyncioLoop.create_task(suckNode.write_value(suckcup)))
                taskSet.add(asyncioLoop.create_task(headServoNode.write_value(float((headServo)))))
                # taskSet.add(asyncioLoop.create_task(xNode.write_value(x)))
                # taskSet.add(asyncioLoop.create_task(yNode.write_value(y)))
                # taskSet.add(asyncioLoop.create_task(zNode.write_value(z)))
                while len(taskSet) > 0:
                    await taskSet.pop()
            except Exception as e:
                traceback.print_exc()
            finally:
                oldData = data

    async def updateAIEnter(data):
        await AIEnterCurrentNode.write_value(data.data)

    async def updateAIRun(data):
        await AIRunCurrentNode.write_value(data.data)

    async def updateAIColor(data):
        await AIColorCurrentNode.write_value(data.data)

    async def updateAIWasteType(data):
        await AIWasteTypeCurrentNode.write_value(data.data)

    async def updateAIWasteState(data):
        await AIWasteStateCurrentNode.write_value(data.data)
        
    async def updateSlider(data):
        await sliderNode.write_value(data.data)

    async def ros_run():
        global actionlibClient
        print("rosrun", threading.current_thread().name)
        global JetMaxControlList
            
        asyncioLoop = asyncio.get_running_loop()
        # create ONLY 1 NODE (anonymous = False) with the name asyncua. If the same
        # node already exist, old node will be replaced by the new node!
        rospy.init_node("asyncua", anonymous=False, log_level=rospy.DEBUG)
        rospy.Subscriber('serial_data', String, 
                         callback=lambda data: taskPool.append(asyncioLoop.create_task(updateRobot(data))))
        # rospy.Subscriber('/jetmax/AIEnterCurrent', String, 
        #                  callback=lambda data: taskPool.append(asyncioLoop.create_task(updateAIEnter(data))))
        # rospy.Subscriber('/jetmax/AIRunCurrent', String, 
        #                  callback=lambda data: taskPool.append(asyncioLoop.create_task(updateAIRun(data))))
        # rospy.Subscriber('/jetmax/AIColorCurrent', String, 
        #                  callback=lambda data: taskPool.append(asyncioLoop.create_task(updateAIColor(data))))
        # rospy.Subscriber('/waste_classification/set_suck', UInt16, 
        #                  callback=lambda data: taskPool.append(asyncioLoop.create_task(updateAIWasteState(data))))
        # rospy.Subscriber('/waste_classification/set_waste_class', String, 
        #                  callback=lambda data: taskPool.append(asyncioLoop.create_task(updateAIWasteType(data))))
        # rospy.Subscriber('/jetmax/slider', String, 
        #                  callback=lambda data: taskPool.append(asyncioLoop.create_task(updateSlider(data))))
        
        # rospy.Subscriber(imageTopic2, Image, imageCallback2)

        # Append servos to list JetMaxControlList
        # [0 -> 2]
        for i in range(1, 2):
            jetmax_pub_control = rospy.Publisher('servo_vr_ros', String, queue_size=1)
            JetMaxControlList.append(jetmax_pub_control)

        # [3 -> 4]
        # for i in range(1, 3):
        #     jetmax_pub_end_effector = rospy.Publisher('/jetmax/end_effector/servo{}/command'.format(i), SetServo,
        #                                               queue_size=1)
        #     JetMaxControlList.append(jetmax_pub_end_effector)

        # # [5]
        # jetmax_pub_sucker = rospy.Publisher('/jetmax/end_effector/sucker/command', Bool, queue_size=1)

        # # [6]
        # jetmax_pos_cmd = rospy.Publisher('/jetmax/command', SetJetMax, queue_size=1)
        
        # # [7]
        # jetmax_relative_pos_cmd = rospy.Publisher('/jetmax/relative_command', SetJetMax, queue_size=1)
        
        # # [8]
        # jetmax_buzzer = rospy.Publisher('/jetmax/buzzer', Bool, queue_size=1)

        # JetMaxControlList.extend([jetmax_pub_sucker, jetmax_pos_cmd, 
        #                           jetmax_relative_pos_cmd, jetmax_buzzer])
        # actionlibClient = actionlib.SimpleActionClient("/jetmax/actionset_online", ActionSetRawAction)
        # # wait until the action server has started up and started listening for goals
        # actionlibClient.wait_for_server()


        while not rospy.is_shutdown():
            # rospy is shutdown when ctrl - C is pressed!
            while len(taskPool) > 0:
                task = taskPool.pop()
                await task
            await asyncio.sleep(0.05)
        print("Exit rospy")

    """
    ================================= /ros section =================================
    """
    await asyncio.gather(serverStart(), ros_run())


if __name__ == "__main__":

    try:
        logging.basicConfig(level=logging.DEBUG)
        # asyncioLoop = asyncio.get_event_loop()
        print("main", threading.current_thread().name)
        asyncio.run(main())
    except Exception as e:
        traceback.print_exc()
    finally:
        exit()

