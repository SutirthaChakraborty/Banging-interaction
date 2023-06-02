import sys
import threading
import time
import traceback

import cv2
import mediapipe as mp
import numpy as np
import pygame
from shapely.geometry import Point, box

pygame.mixer.init()
mp_drawing = mp.solutions.drawing_utils
mp_holistic = mp.solutions.holistic

MAX_LEN = 5
BPM = 140

L_timeStamp = [0]*MAX_LEN
R_timeStamp = [0]*MAX_LEN

def calculate_phase_difference(L_timeStamp, R_timeStamp):
    phase_difference = [a_i - b_i for a_i, b_i in zip(L_timeStamp, R_timeStamp)]
    return phase_difference

def kuramoto_bpm(tempo_event):
    global L_timeStamp
    global R_timeStamp
    global BPM
    MIN_BPM = 0.1  # Define a minimum BPM to prevent division by zero or near-zero values.
    print((R_timeStamp)," = ",(L_timeStamp),BPM)
    while True:
        phase_difference = calculate_phase_difference(L_timeStamp, R_timeStamp)
        phase_difference = np.array(phase_difference)
        phase_difference = np.mod(phase_difference, 2 * np.pi)
        phase_difference[phase_difference > np.pi] -= 2 * np.pi
        avg_phase_coherence = np.abs(np.mean(np.exp(1j * phase_difference)))
        estimated_frequency = np.angle(avg_phase_coherence) / (2 * np.pi)
        BPM = max(estimated_frequency * 60, MIN_BPM)  # Ensure BPM never drops below MIN_BPM
        print("BPM= ",BPM)
        # Signal that the tempo has changed
        tempo_event.set()
        time.sleep(1)

def load_sound(filename):
    sound = pygame.mixer.Sound(filename)
    sound.set_volume(0.5)
    return sound

def play_sound(sound, volume=0.5):
    sound.set_volume(volume)
    sound.play()

def distance2D(a, b):
    return np.linalg.norm(np.array(a) - np.array(b))

def avg(lst):
    return sum(lst) / len(lst)

def metronome(tempo_event):
    global BPM
    MAX_SLEEP = sys.float_info.max
    if BPM == 0:
        BPM = 0.01
    beat_interval = 60.0 / BPM  # Interval in seconds
    while True:
        if tempo_event.is_set():
            beat_interval = 60.0 / BPM
            tempo_event.clear()
        play_sound(sounds["click"])
        try:
            time.sleep(min(beat_interval, MAX_SLEEP))  # Ensure sleep duration never exceeds MAX_SLEEP
        except OverflowError:
            time.sleep(MAX_SLEEP)  # sleep for a very long time

sounds = {
    "hihat": load_sound("sound/hihat.wav"),
    "snare": load_sound("sound/snare.wav"),
    "cymbal": load_sound("sound/cymbal.wav"),
    "kick": load_sound("sound/kick.wav"),
    "click": load_sound("sound/click.wav"),
}
# Set up the metronome thread. We start with a default tempo of BPM.
import threading

tempo_event = threading.Event()
metronome_thread = threading.Thread(target=metronome, args=(tempo_event,))
metronome_thread.start()

bpm_thread = threading.Thread(target=kuramoto_bpm, args=(tempo_event,), daemon=True)
bpm_thread.start()


color1 = (150, 150, 0)
color2 = (0, 255, 170)
color3 = (0, 255, 0)
color4 = (0, 150, 255)

# Center coordinates
center_coordinates = (120, 50)
radius = 10
colorhand = (255, 0, 0)
thickness = -1

lHand = 0
rHand = 0

size = 50
Lsize = size
Rsize = size * 3

velocity = 0

speedBuffer = [0] * 400
maxSpeed = minSpeed = 0
LindexFingerPrev = [(0, 0)]
RindexFingerPrev = [(0, 0)]


cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

# # start the thread
# bpm_thread = threading.Thread(target=kuramoto_bpm, daemon=True)
# bpm_thread.start()

try:
    # cap = cv2.VideoCapture('sample.mp4')
    
    with mp_holistic.Holistic(
        min_detection_confidence=0.5, min_tracking_confidence=0.2
    ) as holistic:
        while True:
            ret, background = cap.read()
            if ret == False:
                print(1)
                break
            # background = cv2.cvtColor(background, cv2.COLOR_BGR2RGB)
            background = cv2.flip(background, 1)
            yscale, xscale = background.shape[:-1]
            results = holistic.process(background)
            # background = cv2.cvtColor(background, cv2.COLOR_RGB2BGR)
            try:
                if results.pose_landmarks:
                    RindexFinger = [int(results.pose_landmarks.landmark[mp_holistic.PoseLandmark.RIGHT_INDEX].x* xscale),int(results.pose_landmarks.landmark[mp_holistic.PoseLandmark.RIGHT_INDEX].y* yscale),]
                    LindexFinger = [int(results.pose_landmarks.landmark[mp_holistic.PoseLandmark.LEFT_INDEX].x * xscale),int(results.pose_landmarks.landmark[mp_holistic.PoseLandmark.LEFT_INDEX].y* yscale),]

                    Ref1 = [int((results.pose_landmarks.landmark[mp_holistic.PoseLandmark.LEFT_HIP].x* xscale+ results.pose_landmarks.landmark[mp_holistic.PoseLandmark.RIGHT_HIP].x* xscale)/ 2),int((results.pose_landmarks.landmark[mp_holistic.PoseLandmark.LEFT_HIP].y* yscale+ results.pose_landmarks.landmark[mp_holistic.PoseLandmark.RIGHT_HIP].y* yscale)/ 2),]

                    background = cv2.rectangle(background,(Ref1[0] - size - Rsize, Ref1[1] - Lsize - size),(Ref1[0] + size - Rsize, Ref1[1] - Lsize + size),color1,thickness,)
                    background = cv2.rectangle(background,(Ref1[0] - size + Rsize, Ref1[1] - Lsize - size),(Ref1[0] + size + Rsize, Ref1[1] - Lsize + size),color4,thickness,)
                    background = cv2.rectangle(background,(Ref1[0] - size - Lsize, Ref1[1] - size),(Ref1[0] + size - Lsize, Ref1[1] + size),color2,thickness,)
                    background = cv2.rectangle(background,(Ref1[0] - size + Lsize, Ref1[1] - size),(Ref1[0] + size + Lsize, Ref1[1] + size),color3, thickness,)
                    
                    box1 = box(Ref1[0] - size - Rsize, Ref1[1] - Lsize - size, Ref1[0] + size - Rsize, Ref1[1] - Lsize + size)
                    box2 = box(Ref1[0] - size - Lsize, Ref1[1] - size, Ref1[0] + size - Lsize, Ref1[1] + size)
                    box3 = box(Ref1[0] - size + Lsize, Ref1[1] - size, Ref1[0] + size + Lsize, Ref1[1] + size)
                    box4 = box(Ref1[0] - size + Rsize, Ref1[1] - Lsize - size, Ref1[0] + size + Rsize, Ref1[1] - Lsize + size)

                    RDist = int(distance2D(RindexFingerPrev, RindexFinger))
                    # Calculate distance and volume
                    LDist = int(distance2D(LindexFingerPrev, LindexFinger))

                    speedBuffer.append(RDist)
                    speedBuffer.append(LDist)
                    speedBuffer = speedBuffer[2:]
                    maxSpeed = max(speedBuffer)
                    minSpeed = min(speedBuffer)
                    
                    volR = (RDist - minSpeed) / (maxSpeed - minSpeed)
                    volL = (LDist - minSpeed) / (maxSpeed - minSpeed)
                    
                    # LEFT HAND
                    if ( box1.contains(Point(LindexFinger[0], LindexFinger[1])) and lHand != 1 ):
                        lHand = 1
                        # play_sound(sounds["hihat"], volL)
                        threading.Thread(target=play_sound, args=(sounds["hihat"], volL)).start()
                        L_timeStamp.append(time.time())
                    elif ( box2.contains(Point(LindexFinger[0], LindexFinger[1])) and lHand != 2 ):
                        lHand = 2
                        threading.Thread(target=play_sound, args=(sounds["cymbal"], volL)).start()
                        L_timeStamp.append(time.time())
                    elif ( box3.contains(Point(LindexFinger[0], LindexFinger[1])) and lHand != 3 ):
                        lHand = 3
                        threading.Thread(target=play_sound, args=(sounds["snare"], volL)).start()
                        L_timeStamp.append(time.time())
                    elif ( box4.contains(Point(LindexFinger[0], LindexFinger[1])) and lHand != 4  ):
                        lHand = 4
                        threading.Thread(target=play_sound, args=(sounds["kick"], volL)).start()
                        L_timeStamp.append(time.time())
                    elif (box1.contains(Point(LindexFinger[0], LindexFinger[1])) == False and box2.contains(Point(LindexFinger[0], LindexFinger[1])) == False and box3.contains(Point(LindexFinger[0], LindexFinger[1]))  == False and box4.contains(Point(LindexFinger[0], LindexFinger[1])) == False ):
                        lHand = 0

                    # RIGHT HAND

                    if ( box1.contains(Point(RindexFinger[0], RindexFinger[1])) and rHand != 1 ):
                        rHand = 1
                        # play_sound(sounds["hihat"], volR)
                        threading.Thread(target=play_sound, args=(sounds["hihat"], volR)).start()
                        R_timeStamp.append(time.time())
                    elif ( box2.contains(Point(RindexFinger[0], RindexFinger[1])) and rHand != 2):
                        rHand = 2
                        # play_sound(sounds["cymbal"], volR)
                        threading.Thread(target=play_sound, args=(sounds["cymbal"], volR)).start()
                        R_timeStamp.append(time.time())
                    elif ( box3.contains(Point(RindexFinger[0], RindexFinger[1])) and rHand != 3):
                        rHand = 3
                        # play_sound(sounds["snare"], volR)
                        threading.Thread(target=play_sound, args=(sounds["snare"], volR)).start()
                        R_timeStamp.append(time.time())
                    elif ( box4.contains(Point(RindexFinger[0], RindexFinger[1])) and rHand != 4 ):
                        rHand = 4
                        threading.Thread(target=play_sound, args=(sounds["kick"], volR)).start()
                        R_timeStamp.append(time.time())
                    elif ( box1.contains(Point(RindexFinger[0], RindexFinger[1])) == False and box2.contains(Point(RindexFinger[0], RindexFinger[1])) == False  and box3.contains(Point(RindexFinger[0], RindexFinger[1]))== False  and box4.contains(Point(RindexFinger[0], RindexFinger[1])) == False  ):
                        rHand = 0

                    center_coordinates = (RindexFinger[0], RindexFinger[1])
                    background = cv2.circle(background, center_coordinates, radius, colorhand, thickness)
                    center_coordinates = (LindexFinger[0], LindexFinger[1])
                    background = cv2.circle(background, center_coordinates, radius, colorhand, thickness)

                    LindexFingerPrev = LindexFinger
                    RindexFingerPrev = RindexFinger
                    
                    if len(R_timeStamp)>MAX_LEN:
                        R_timeStamp=R_timeStamp[1:]
                    if len(L_timeStamp)>MAX_LEN:
                        L_timeStamp=L_timeStamp[1:]
                        
                    print(BPM)
                    bpm_thread = threading.Thread(target=kuramoto_bpm, args=(tempo_event,), daemon=True)
                    bpm_thread.start()
                    
            except Exception as e:
                print(traceback.format_exc())
                print(e)

            cv2.imshow("DrumClient", background)
            
            k = cv2.waitKey(10)
            # Press q to break
            if k == ord("q"):
                cap.release()
                cv2.destroyAllWindows()
                break

    cap.release()
    cv2.destroyAllWindows()
except:
    # print(e)
    print(traceback.format_exc())
    cap.release()
    cv2.destroyAllWindows()
    sys.exit()
