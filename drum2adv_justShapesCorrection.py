import cv2
import mediapipe as mp
import numpy as np
from shapely import geometry
import threading
import pygame
import socket
import traceback
from shapely.geometry import Point, box
import time

pygame.mixer.init()
mp_drawing = mp.solutions.drawing_utils
mp_holistic = mp.solutions.holistic


def load_sound(filename):
    sound = pygame.mixer.Sound(filename)
    sound.set_volume(0.5)
    return sound


def play_sound(sound, volume=0.5):
    sound.set_volume(volume)
    sound.play()


def distance2D(a, b):
    return np.linalg.norm(np.array(a) - np.array(b))


def calculate_volume(distance, min_speed, max_speed):
    try:
        return (distance - min_speed) / (max_speed - min_speed)
    except:
        return 0


def avg(lst):
    return sum(lst) / len(lst)


def metronome(tempo, sound):
    beat_interval = 60.0 / tempo  # Interval in seconds
    while True:
        play_sound(sound)
        time.sleep(beat_interval)


sounds = {
    "hihat": load_sound("sound/hihat.wav"),
    "snare": load_sound("sound/snare.wav"),
    "cymbal": load_sound("sound/cymbal.wav"),
    "kick": load_sound("sound/kick.wav"),
    "click": load_sound("sound/click.wav"),
}

# Set up the metronome thread. We start with a default tempo of BPM.
BPM = 100
metronome_thread = threading.Thread(target=metronome, args=(BPM, sounds["click"]))
metronome_thread.start()


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

size = 250
Lsize = size
Rsize = size * 3

ClientSocket = socket.socket()
host = "localhost"
# host='192.168.0.241'
port = 1244

velocity = 0

print("Waiting for connection")

try:
    ClientSocket.connect((host, port))
except socket.error as e:
    print(str(e))


speedBuffer = [0] * 400
maxSpeed = minSpeed = 0
LindexFingerPrev = [(0, 0)]
RindexFingerPrev = [(0, 0)]

close = True
while close:
    Response = int(ClientSocket.recv(1024).decode("utf-8"))
    channel = Response  # counting from one here
    NOTE_ON = 0x90
    status = NOTE_ON | (channel - 1)
    status = str(status) + " "
    kit = ["hihat", "snare", "cymbal", "kick"]
    pmessage1 = status + "D cymbal 0"
    pmessage2 = status + "D cymbal 0"
    try:
        # cap = cv2.VideoCapture('sample.mp4')
        cap = cv2.VideoCapture(0)
        with mp_holistic.Holistic(
            min_detection_confidence=0.5, min_tracking_confidence=0.2
        ) as holistic:
            while True:
                ret, background = cap.read()
                if ret == False:
                    print(1)
                    break
                background = cv2.cvtColor(background, cv2.COLOR_BGR2RGB)
                background = cv2.flip(background, 1)
                yscale, xscale = background.shape[:-1]
                results = holistic.process(background)
                background = cv2.cvtColor(background, cv2.COLOR_RGB2BGR)
                try:
                    if results.pose_landmarks:
                        RindexFinger = [
                            int(
                                results.pose_landmarks.landmark[
                                    mp_holistic.PoseLandmark.RIGHT_INDEX
                                ].x
                                * xscale
                            ),
                            int(
                                results.pose_landmarks.landmark[
                                    mp_holistic.PoseLandmark.RIGHT_INDEX
                                ].y
                                * yscale
                            ),
                        ]
                        LindexFinger = [
                            int(
                                results.pose_landmarks.landmark[
                                    mp_holistic.PoseLandmark.LEFT_INDEX
                                ].x
                                * xscale
                            ),
                            int(
                                results.pose_landmarks.landmark[
                                    mp_holistic.PoseLandmark.LEFT_INDEX
                                ].y
                                * yscale
                            ),
                        ]

                        Ref1 = [
                            int(
                                (
                                    results.pose_landmarks.landmark[
                                        mp_holistic.PoseLandmark.LEFT_HIP
                                    ].x
                                    * xscale
                                    + results.pose_landmarks.landmark[
                                        mp_holistic.PoseLandmark.RIGHT_HIP
                                    ].x
                                    * xscale
                                )
                                / 2
                            ),
                            int(
                                (
                                    results.pose_landmarks.landmark[
                                        mp_holistic.PoseLandmark.LEFT_HIP
                                    ].y
                                    * yscale
                                    + results.pose_landmarks.landmark[
                                        mp_holistic.PoseLandmark.RIGHT_HIP
                                    ].y
                                    * yscale
                                )
                                / 2
                            ),
                        ]

                        background = cv2.rectangle(
                            background,
                            (Ref1[0] - size - Rsize, Ref1[1] - Lsize - size),
                            (Ref1[0] + size - Rsize, Ref1[1] - Lsize + size),
                            color1,
                            thickness,
                        )
                        background = cv2.rectangle(
                            background,
                            (Ref1[0] - size + Rsize, Ref1[1] - Lsize - size),
                            (Ref1[0] + size + Rsize, Ref1[1] - Lsize + size),
                            color4,
                            thickness,
                        )
                        background = cv2.rectangle(
                            background,
                            (Ref1[0] - size - Lsize, Ref1[1] - size),
                            (Ref1[0] + size - Lsize, Ref1[1] + size),
                            color2,
                            thickness,
                        )
                        background = cv2.rectangle(
                            background,
                            (Ref1[0] - size + Lsize, Ref1[1] - size),
                            (Ref1[0] + size + Lsize, Ref1[1] + size),
                            color3,
                            thickness,
                        )

                        box1 = box(
                            Ref1[0] - size - Rsize,
                            Ref1[1] - Lsize - size,
                            Ref1[0] + size - Rsize,
                            Ref1[1] - Lsize + size,
                        )
                        box2 = box(
                            Ref1[0] - size - Lsize,
                            Ref1[1] - size,
                            Ref1[0] + size - Lsize,
                            Ref1[1] + size,
                        )
                        box3 = box(
                            Ref1[0] - size + Lsize,
                            Ref1[1] - size,
                            Ref1[0] + size + Lsize,
                            Ref1[1] + size,
                        )
                        box4 = box(
                            Ref1[0] - size + Rsize,
                            Ref1[1] - Lsize - size,
                            Ref1[0] + size + Rsize,
                            Ref1[1] - Lsize + size,
                        )

                        RDist = int(distance2D(RindexFingerPrev, RindexFinger))
                        # Calculate distance and volume
                        LDist = int(distance2D(LindexFingerPrev, LindexFinger))
                        volL = calculate_volume(LDist, minSpeed, maxSpeed)

                        speedBuffer.append(RDist)
                        speedBuffer.append(LDist)
                        speedBuffer = speedBuffer[2:]
                        maxSpeed = max(speedBuffer)
                        minSpeed = min(speedBuffer)
                        volR = (RDist - minSpeed) / (maxSpeed - minSpeed)
                        volL = (LDist - minSpeed) / (maxSpeed - minSpeed)
                        # LEFT HAND
                        if (
                            box1.contains(Point(LindexFinger[0], LindexFinger[1]))
                            and lHand != 1
                        ):
                            lHand = 1
                            play_sound(sounds["hihat"], volL)
                        elif (
                            box2.contains(Point(LindexFinger[0], LindexFinger[1]))
                            and lHand != 2
                        ):
                            lHand = 2
                            play_sound(sounds["cymbal"], volL)

                        elif (
                            box3.contains(Point(LindexFinger[0], LindexFinger[1]))
                            and lHand != 3
                        ):
                            lHand = 3
                            play_sound(sounds["snare"], volL)
                            # snare(volL)

                        elif (
                            box4.contains(Point(LindexFinger[0], LindexFinger[1]))
                            and lHand != 4
                        ):
                            lHand = 4
                            play_sound(sounds["kick"], volL)
                            # kick(volL)

                        elif (
                            box1.contains(Point(LindexFinger[0], LindexFinger[1]))
                            == False
                            and box2.contains(Point(LindexFinger[0], LindexFinger[1]))
                            == False
                            and box3.contains(Point(LindexFinger[0], LindexFinger[1]))
                            == False
                            and box4.contains(Point(LindexFinger[0], LindexFinger[1]))
                            == False
                        ):
                            lHand = 0

                        # RIGHT HAND

                        if (
                            box1.contains(Point(RindexFinger[0], RindexFinger[1]))
                            and rHand != 1
                        ):
                            rHand = 1
                            play_sound(sounds["hihat"], volR)

                        elif (
                            box2.contains(Point(RindexFinger[0], RindexFinger[1]))
                            and rHand != 2
                        ):
                            rHand = 2
                            play_sound(sounds["cymbal"], volR)

                        elif (
                            box3.contains(Point(RindexFinger[0], RindexFinger[1]))
                            and rHand != 3
                        ):
                            rHand = 3
                            play_sound(sounds["snare"], volR)

                        elif (
                            box4.contains(Point(RindexFinger[0], RindexFinger[1]))
                            and rHand != 4
                        ):
                            rHand = 4
                            play_sound(sounds["kick"], volR)

                        elif (
                            box1.contains(Point(RindexFinger[0], RindexFinger[1]))
                            == False
                            and box2.contains(Point(RindexFinger[0], RindexFinger[1]))
                            == False
                            and box3.contains(Point(RindexFinger[0], RindexFinger[1]))
                            == False
                            and box4.contains(Point(RindexFinger[0], RindexFinger[1]))
                            == False
                        ):
                            rHand = 0

                        cv2.putText(
                            background,
                            str(int(distance2D(LindexFingerPrev, LindexFinger))),
                            tuple(np.multiply(LindexFinger, [1, 1]).astype(int)),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            1,
                            (255, 255, 255),
                            2,
                            cv2.LINE_AA,
                        )

                        cv2.putText(
                            background,
                            str(int(distance2D(RindexFingerPrev, RindexFinger))),
                            tuple(np.multiply(RindexFinger, [1, 1]).astype(int)),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            1,
                            (255, 255, 255),
                            2,
                            cv2.LINE_AA,
                        )

                        center_coordinates = (RindexFinger[0], RindexFinger[1])
                        background = cv2.circle(
                            background, center_coordinates, radius, colorhand, thickness
                        )
                        center_coordinates = (LindexFinger[0], LindexFinger[1])
                        background = cv2.circle(
                            background, center_coordinates, radius, colorhand, thickness
                        )

                        LindexFingerPrev = LindexFinger
                        RindexFingerPrev = RindexFinger

                except Exception as e:
                    print(traceback.format_exc())
                    print(e)

                cv2.imshow("DrumClient", background)
                k = cv2.waitKey(10)
                # Press q to break
                if k == ord("q"):
                    break

        cap.release()
        cv2.destroyAllWindows()
    except:
        #         print(e)
        print(traceback.format_exc())
