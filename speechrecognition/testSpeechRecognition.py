import speech_recognition as sr
from Drone import Drone, BaseDrone

import time
# URI to the Crazyflie to connect to
uri = 'radio://0/80/2M/E7E7E7E709'

# obtain audio from the microphone
r = sr.Recognizer()
stopCommand = False
with sr.Microphone() as source:
    with Drone(uri) as drone:
        while stopCommand != True:
            print("Say something!")
            audio = r.listen(source)

            # recognize speech using Google Speech Recognition
            try:
                # for testing purposes, we're just using the default API key
                # to use another API key, use `r.recognize_google(audio, key="GOOGLE_SPEECH_RECOGNITION_API_KEY")`
                # instead of `r.recognize_google(audio)`
                
                command = r.recognize_google(audio)
                print("Google Speech Recognition thinks you said " + command)

                #TODO: It just stops after the first call. It stays in the while loop but the drone stops
                if command == "start":
                    drone.flyToPoint(0,0,0.2)
                elif command == "left":
                    drone.flyToPoint(0,-1,0.2)
                elif command == "stop":
                    drone.land()
                    stopCommand = True

                print(stopCommand)

            except sr.UnknownValueError:
                print("Google Speech Recognition could not understand audio")
            except sr.RequestError as e:
                print("Could not request results from Google Speech Recognition service; {0}".format(e))
