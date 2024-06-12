#!/usr/bin/env python3
import sys
import time

sys.path.append('/home/hiwonder/ros_Cat/src/jetmax_buildin_funcs/remote_control/scripts')
sys.path.append('/home/hiwonder/ros_Cat/src/jetmax_buildin_funcs/jetmax_control/scripts')
sys.path.append('/home/hiwonder/ros_Cat/src/jetmax_buildin_funcs/palletizing/scripts')
sys.path.append('/home/hiwonder/ros_Cat/src/jetmax_buildin_funcs/object_tracking/scripts')
sys.path.append('/home/hiwonder/ros_Cat/src/jetmax_buildin_funcs/color_sorting/scripts')
sys.path.append('/home/hiwonder/ros_Cat/src/jetmax_buildin_funcs/waste_classification/scripts')
sys.path.append('/home/hiwonder/ros_Cat/src/jetmax_buildin_funcs/fruits_classification2/scripts')

import hiwonder
import traceback
import rospy
from threading import Thread
import asyncio
import Jetson.GPIO as GPIO


class Singleton(object):
    _instance = None

    def __new__(cls, *args, **kwargs):
        if not cls._instance:
            cls._instance = super(Singleton, cls).__new__(cls, *args, **kwargs)
            cls._instance.jetmax = hiwonder.JetMax()
            cls._instance.sucker = hiwonder.Sucker()
        return cls._instance


async def async_process():
    async def async_waste_classification():
        while True:
            try:
                waste_classification_main.image_proc()
                if rospy.is_shutdown():
                    break
            except Exception as e:
                rospy.logerr(e)
                break
            await asyncio.sleep(0.01)
            
    async def async_fruits_classification():
        # check the fruits_classification name!
        time_out = time.time()
        prev_state = fruits_classification_main.HOME
        while True:
            try:            
                #state_relay = GPIO.input(9)

                if not (prev_state == fruits_classification_main.motor_state) or (fruits_classification_main.motor_state == fruits_classification_main.HOME) or (prev_state == fruits_classification_main.MID) :
                        time_out = time.time()
                        if not (prev_state == fruits_classification_main.MID and fruits_classification_main.motor_state == fruits_classification_main.STOP ):
                            prev_state = fruits_classification_main.motor_state

                if ( time.time() - time_out > 15)  :
                        #flag_ok = False
                        #motor_dir = 1
                        #motor_state = HOME
                        #flag_mid = False
                        #flag_seek_left = None
                        #flag_seek_right = None
                        #flag_target_ok = False
                        #motor_position = 212
                        time_out = time.time()
                        for i in range (3):
                            GPIO.output(4,GPIO.LOW)
                            rospy.sleep(0.1)
                            GPIO.output(4,GPIO.HIGH)
                            rospy.sleep(0.1)
                        #open_gripper(0.5)
                        #motor_cmd("HOME")
                        #move_to_pos((0,0,0),1) # Hung
                        #rospy.sleep(5)
                        print("TIME OUT")              
                
                while fruits_classification_main.control_motor.inWaiting() > 0:
                    data = fruits_classification_main.control_motor.readline().decode(encoding='utf-8')                 

                    if str("STOP:") in data :
                        fruits_classification_main.motor_position = int((str(data).split(":"))[1])
                        print("Pos: ",fruits_classification_main.motor_position)

                    if str("IDLE") in data :
                        time_out = time.time()
                        fruits_classification_main.flag_ok = True
                        print("IDLE")

                    if str("TL") in data or ( str("TR") in data ):
                        fruits_classification_main.flag_target_ok = True
                        print("flag_target_ok")

                fruits_classification_main.image_proc()

                if rospy.is_shutdown():
                    break

            except Exception as e:
                rospy.logerr(e)
                break
        #GPIO.output(4,GPIO.HIGH) # turn off LED

    await asyncio.gather(#async_waste_classification(),
                         async_fruits_classification())


if __name__ == "__main__":

    rospy.init_node("very_main", anonymous=False)

    import remote_control_joystick
    import jetmax_control_main
    #import palletizing_main
    #import object_tracking_main
    #import color_sorting_main
    #import waste_classification_main
    import fruits_classification_main


    def thread_Joystick(js):
        remote_control_joystick.change_mode(0)
        while True:
            try:
                js.update_buttons()
                rospy.sleep(0.05)
                if rospy.is_shutdown():
                    sys.exit(0)
            except KeyboardInterrupt:
                sys.exit(0)


    def thread_object_tracking():
        while True:
            try:
                object_tracking_main.image_proc()
                if rospy.is_shutdown():
                    break
            except KeyboardInterrupt:
                break


    # remote_control_joystick.js.change_mode(0)  # Joint mode as the default mode
    try:
        thread1 = Thread(target=thread_Joystick, args=(remote_control_joystick.js,))
        thread1.start()
        thread2 = Thread(target=jetmax_control_main.jetmax.update)
        thread2.start()
        # thread3 = Thread(target=thread_object_tracking)
        # thread3.start()
        # thread4 = Thread(target = thread_waste_classification)
        # thread4.start()
        loop = asyncio.get_event_loop()
        loop.run_until_complete(async_process())
    except KeyboardInterrupt:
        sys.exit(0)
    except Exception as e:
        traceback.print_exc()
    finally:
        loop.stop()
        loop.close()
        exit()
