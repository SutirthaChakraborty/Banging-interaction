import cv2
import mediapipe as mp
import numpy as np
import traceback
import socket
mp_drawing = mp.solutions.drawing_utils
mp_holistic = mp.solutions.holistic

from shapely.geometry import Point
import pygame
pygame.mixer.init()


import numpy as np
def distance2D(a,b):
    dist = np.linalg.norm(np.array(a)*1000-np.array(b)*1000)
    return dist

# distance2D(Lwrist,lastLWrist)

def avg(lst):
    return sum(lst) / len(lst)

import pygame
pygame.mixer.init()
my_sound = pygame.mixer.Sound('sound/kick.wav')
# my_sound.play()
my_sound.set_volume(0.5)




import threading
from playsound import playsound
def kick(vol=1):
    my_sound = pygame.mixer.Sound('sound/snare.wav')
    my_sound.set_volume(vol)
    my_sound.play()
    
    
def snare(vol=1):   
    my_sound = pygame.mixer.Sound('sound/hihat.wav')
    my_sound.set_volume(vol)
    my_sound.play()
    
def cymbal(vol=1): 
    my_sound = pygame.mixer.Sound('sound/cymbal.wav')
    my_sound.set_volume(vol)
    my_sound.play()
    
def hihat(vol=1):   
    my_sound = pygame.mixer.Sound('sound/kick.wav')
    my_sound.set_volume(vol)
    my_sound.play()




import cv2
import numpy as np
from shapely import geometry
from shapely.geometry import box
from shapely.geometry import Point
import time

import socket


color1 = (150, 150, 0)
color2 = (0, 255, 170)
color3 = (0, 255, 0)
color4 = (0, 150, 255)

thickness = -1
# Center coordinates
center_coordinates = (120, 50)
radius = 10
colorhand = (255, 0, 0)
thickness = -1

lHand=0
rHand=0
size=50


ClientSocket = socket.socket()
host = 'localhost'
# host='192.168.0.241'
port = 1244
 
velocity=0

print('Waiting for connection')

try:
    ClientSocket.connect((host, port))
except socket.error as e:
    print(str(e))


close=True
while close:
    Response = int(ClientSocket.recv(1024).decode('utf-8'))
    channel = Response  # counting from one here
    NOTE_ON = 0x90
    status = NOTE_ON | (channel - 1)
    status=str(status)+" "
    kit=["hihat","snare","cymbal","kick"]
    pmessage1=status+"D cymbal 0"
    pmessage2=status+"D cymbal 0"
    try:
        # cap = cv2.VideoCapture('sample.mp4')
        cap = cv2.VideoCapture(1)
        with mp_holistic.Holistic(min_detection_confidence=0.5, min_tracking_confidence=0.2) as holistic:

            while True:
                ret, background = cap.read()
                if ret==False:
                    print(1)
                    break
                background = cv2.cvtColor(background, cv2.COLOR_BGR2RGB)
                background = cv2.flip(background,1)
                yscale,xscale= background.shape[:-1]
                results = holistic.process(background)
                background = cv2.cvtColor(background, cv2.COLOR_RGB2BGR)
                try:
                    if results.pose_landmarks:

                        RindexFingure= [int(results.pose_landmarks.landmark[mp_holistic.PoseLandmark.RIGHT_INDEX].x*xscale), int(results.pose_landmarks.landmark[mp_holistic.PoseLandmark.RIGHT_INDEX].y*yscale)]
                        LindexFingure= [int(results.pose_landmarks.landmark[mp_holistic.PoseLandmark. LEFT_INDEX].x*xscale), int(results.pose_landmarks.landmark[mp_holistic.PoseLandmark.LEFT_INDEX].y*yscale)]

                        Ref1=[int((results.pose_landmarks.landmark[mp_holistic.PoseLandmark.LEFT_HIP].x*xscale+results.pose_landmarks.landmark[mp_holistic.PoseLandmark.RIGHT_HIP].x*xscale)/2),  
                              int((results.pose_landmarks.landmark[mp_holistic.PoseLandmark.LEFT_HIP].y*yscale+results.pose_landmarks.landmark[mp_holistic.PoseLandmark.RIGHT_HIP].y*yscale)/2)]

                        background = cv2.rectangle(background, (Ref1[0]-size-150,Ref1[1]-50-size), (Ref1[0]+size-150, Ref1[1]-50+size), color1, thickness)
                        background = cv2.rectangle(background, (Ref1[0]-size+150,Ref1[1]-50-size), (Ref1[0]+size+150, Ref1[1]-50+size), color4, thickness)
                        background = cv2.rectangle(background, (Ref1[0]-size-50, Ref1[1]-size), (Ref1[0]+size-50, Ref1[1]+size), color2, thickness)
                        background = cv2.rectangle(background, (Ref1[0]-size+50,Ref1[1]-size), (Ref1[0]+size+50, Ref1[1]+size), color3, thickness)

                        box1 = box(Ref1[0]-size-150,Ref1[1]-50-size, Ref1[0]+size-150, Ref1[1]-50+size)
                        box2 = box(Ref1[0]-size-50, Ref1[1]-size   , Ref1[0]+size-50, Ref1[1]+size)
                        box3 = box(Ref1[0]-size+50,Ref1[1]-size    , Ref1[0]+size+50, Ref1[1]+size)
                        box4 = box(Ref1[0]-size+150,Ref1[1]-50-size, Ref1[0]+size+150, Ref1[1]-50+size)



                        # LEFT HAND

                        if box1.contains(Point(LindexFingure[0],LindexFingure[1]))   and lHand!=1:
                            lHand=1
                            hihat()
                            ClientSocket.send(str.encode(str(pmessage1)))
                            message1=status+"D "+kit[0]+" 100"
                            pmessage1=status+"D "+kit[0]+" 0"
                            ClientSocket.send(str.encode(str(message1)))
                            
                        elif box2.contains(Point(LindexFingure[0],LindexFingure[1])) and lHand!=2:
                            lHand=2
                            cymbal()
                            ClientSocket.send(str.encode(str(pmessage1)))
                            message1=status+"D "+kit[1]+" 100"
                            pmessage1=status+"D "+kit[1]+" 0"
                            ClientSocket.send(str.encode(str(message1)))
                            
                        elif box3.contains(Point(LindexFingure[0],LindexFingure[1])) and lHand!=3:
                            lHand=3
                            snare()
                            ClientSocket.send(str.encode(str(pmessage1)))
                            message1=status+"D "+kit[2]+" 100"
                            pmessage1=status+"D "+kit[2]+" 0"
                            ClientSocket.send(str.encode(str(message1)))
                            
                        elif box4.contains(Point(LindexFingure[0],LindexFingure[1])) and lHand!=4:
                            lHand=4
                            kick()
                            ClientSocket.send(str.encode(str(pmessage1)))
                            message1=status+"D "+kit[3]+" 100"
                            pmessage1=status+"D "+kit[3]+" 0"
                            ClientSocket.send(str.encode(str(message1)))
                            
                        elif box1.contains(Point(LindexFingure[0],LindexFingure[1]))==False and box2.contains(Point(LindexFingure[0],LindexFingure[1]))==False and box3.contains(Point(LindexFingure[0],LindexFingure[1]))==False  and box4.contains(Point(LindexFingure[0],LindexFingure[1]))==False:
                            lHand=0
                            


                        # RIGHT HAND

                        if box1.contains(Point(RindexFingure[0],RindexFingure[1]))   and rHand!=1:
                            rHand=1
                            hihat()
                            ClientSocket.send(str.encode(str(pmessage2)))
                            message2=status+"D "+kit[0]+" 100"
                            pmessage2=status+"D "+kit[0]+" 0"
                            ClientSocket.send(str.encode(str(message2)))

                        elif box2.contains(Point(RindexFingure[0],RindexFingure[1])) and rHand!=2: 
                            rHand=2
                            cymbal()
                            ClientSocket.send(str.encode(str(pmessage2)))
                            message2=status+"D "+kit[1]+" 100"
                            pmessage2=status+"D "+kit[1]+" 0"
                            ClientSocket.send(str.encode(str(message2)))

                        elif box3.contains(Point(RindexFingure[0],RindexFingure[1])) and rHand!=3:
                            rHand=3
                            snare()
                            ClientSocket.send(str.encode(str(pmessage2)))
                            message2=status+"D "+kit[2]+" 100"
                            pmessage2=status+"D "+kit[2]+" 0"
                            ClientSocket.send(str.encode(str(message2)))

                        elif box4.contains(Point(RindexFingure[0],RindexFingure[1])) and rHand!=4:
                            rHand=4
                            kick()
                            ClientSocket.send(str.encode(str(pmessage2)))
                            message2=status+"D "+kit[3]+" 100"
                            pmessage2=status+"D "+kit[3]+" 0"
                            ClientSocket.send(str.encode(str(message2)))

                        elif box1.contains(Point(RindexFingure[0],RindexFingure[1])) == False and  box2.contains(Point(RindexFingure[0],RindexFingure[1]))==False and box3.contains(Point(RindexFingure[0],RindexFingure[1]))==False and box4.contains(Point(RindexFingure[0],RindexFingure[1]))==False:
                            rHand=0
                            
                            

                        cv2.putText(background, str(rHand) , 
                                               tuple(np.multiply(RindexFingure, [1, 1]).astype(int)), 
                                               cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)

                        cv2.putText(background, str(lHand) , 
                                               tuple(np.multiply(LindexFingure, [1, 1]).astype(int)), 
                                               cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)


                        center_coordinates= (RindexFingure[0],RindexFingure[1])
                        background =cv2.circle(background, center_coordinates, radius, colorhand, thickness)
                        center_coordinates= (LindexFingure[0],LindexFingure[1])
                        background =cv2.circle(background, center_coordinates, radius, colorhand, thickness)


                        ######### Draw face landmarks
                        mp_drawing.draw_landmarks(background, results.face_landmarks, mp_holistic.FACE_CONNECTIONS)
                        ##########Right hand
                        mp_drawing.draw_landmarks(background, results.right_hand_landmarks, mp_holistic.HAND_CONNECTIONS)
                        ########## Left Hand
                        mp_drawing.draw_landmarks(background, results.left_hand_landmarks, mp_holistic.HAND_CONNECTIONS)
                        ########## Pose Detections
                        mp_drawing.draw_landmarks(background, results.pose_landmarks, mp_holistic.POSE_CONNECTIONS)

                except Exception as e: 
                    print(e)   

                cv2.imshow('DrumClient',background)
                k = cv2.waitKey(10)
                # Press q to break
                if k == ord('q'):
                    break

        cap.release()
        cv2.destroyAllWindows()
    except:
#         print(e)
        print(traceback.format_exc())