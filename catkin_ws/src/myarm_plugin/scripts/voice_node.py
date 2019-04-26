#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from std_msgs.msg import Float32
import speech_recognition as sr


r = sr.Recognizer()


def callback(msg):
    x = msg.data
    gripper_pub = rospy.Publisher('/gripper', Float32, queue_size=1)

    if x == 100:    # key pressed = d
        with sr.Microphone() as source:
            try:
                print("Say something!")
                audio = r.listen(source)
                # recognize speech using Sphinx
                try:
                    rec_phrase = r.recognize_google(audio)
                    rec_phrase = rec_phrase.lower()
                    print("Google thinks you said '" + rec_phrase + "'")
                    if 'half' in rec_phrase:
                        gripper_pub.publish(0.075)
                        # rospy.spinOnce()
                    elif 'close' in rec_phrase:
                        gripper_pub.publish(0.13)
                        # rospy.spinOnce()
                    elif 'open' in rec_phrase:
                        gripper_pub.publish(0)
                        # rospy.spinOnce()
                    elif any(t in rec_phrase for t in ['set', 'angle', 'base']):
                        rospy.Publisher('/GrossesGelenkLink/setAngle', Float32, queue_size=1).publish(45)
                    elif 'stretch arm' in rec_phrase:
                        rospy.Publisher('/GrossesGelenkLink/setAngle', Float32, queue_size=1).publish(0)
                        rospy.Publisher('/MittleresGelenkLink/setAngle', Float32, queue_size=1).publish(0)
                        rospy.Publisher('/KleinesGelenkLink/setAngle', Float32, queue_size=1).publish(0)
                    elif 'go to 90' in rec_phrase:
                        rospy.Publisher('/GrossesGelenkLink/setAngle', Float32, queue_size=1).publish(0)
                        rospy.Publisher('/MittleresGelenkLink/setAngle', Float32, queue_size=1).publish(90)
                    else:
                        print("No viable command")
                except sr.UnknownValueError:
                    print("Google could not understand audio")
                except sr.RequestError as e:
                    print("Google error; {0}".format(e))
            except KeyboardInterrupt:
                pass


def main():
    rospy.init_node('voice_node')
    with sr.Microphone() as source:
        try:
            print("Please wait. Calibrating microphone...")
            # listen for 5 seconds and create the ambient noise energy level
            r.adjust_for_ambient_noise(source, duration=5)
            print("Calibrated")
        except KeyboardInterrupt:
            pass
    rospy.Subscriber("/keyboard_input", Int32, callback)
    rospy.spin()
    pass


if __name__ == '__main__':
    main()
