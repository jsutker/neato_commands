#!/usr/bin/env python
"""speech recognizer. Takes speech from pyaudio and sends rosmsgs based on speech recognised
Say a command, then wait for it to be interpreted. Saying multiple commands will make the end result take longer.
Commands:
"Forward"  = Drive forward
"Back" = Drive Backward
"Left" = Turn to the left
"Right" = Turn to the right
"Stop" = Stop moving
"""


import rospy
import speech_recognition as sr
from std_msgs.msg import String


class Recognize_Speech():

    def __init__(self):
        """ Initialize the speech recognizer """
        rospy.init_node('speech_recognizer')
        self.pub = rospy.Publisher('/speech_cmd', String, queue_size=10)
        self.sleepy = rospy.Rate(2)
        #Stop the robot on shutdown
        rospy.on_shutdown(self.stop)
  

    def recognizespeech(self):
        """Recognizes Speech and publishes to /speech_cmd"""

        # obtain audio from the microphone
        r = sr.Recognizer() 
        with sr.Microphone() as source:
            audio = r.listen(source)


        # recognize speech using Google Speech Recognition
        try:
            # for testing purposes, I'm just using the default API key
            # to use another API key, use `r.recognize_google(audio, key="GOOGLE_SPEECH_RECOGNITION_API_KEY")`
            # instead of `r.recognize_google(audio)`
            audiorecognized = r.recognize_google(audio)

            #print("Google Speech Recognition thinks you said " + audiorecognized ) 

        except:
            pass

        if audiorecognized:
            #If audiorecognized exists, check if it has a command in it
            #Stop comes first so that it is given priority in case multiple commands are interpreted as one

            if "stop" in audiorecognized:
                self.pub.publish(String('k'))

            elif "back" in audiorecognized:
                self.pub.publish(String(',')) 

            elif "forward" in audiorecognized:
                self.pub.publish(String('i'))

            elif "left" in audiorecognized:
                self.pub.publish(String('j'))

            elif "right" in audiorecognized:
                self.pub.publish(String('l'))


    
    ##Main

    def run(self):

        self.recognizespeech()
        self.sleepy.sleep()

recognize = Recognize_Speech()
recognize.run()










