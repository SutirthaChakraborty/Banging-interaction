import time
import rtmidi
from pychord import Chord
import traceback
from datetime import datetime


midiout = rtmidi.MidiOut()
available_ports = midiout.get_ports()
if available_ports:
    midiout.open_port(len(available_ports) - 1)
else:
    midiout.open_virtual_port("My virtual output")
available_ports

chord_midi_Number = {
    "C2": "36",
    "Db2": "37",
    "C#2": "37",
    "D2": "38",
    "Eb2": "39",
    "D#2": "39",
    "E2": "40",
    "F2": "41",
    "Gb2": "42",
    "F#2": "42",
    "G2": "43",
    "G#2": "44",
    "Ab2": "44",
    "A2": "45",
    "A#2": "46",
    "Bb2": "46",
    "B2": "47",
    "C3": "48",
    "Db3": "49",
    "C#3": "49",
    "D3": "50",
    "Eb3": "51",
    "D#3": "51",
    "E3": "52",
    "F3": "53",
    "Gb3": "54",
    "F#3": "54",
    "G3": "55",
    "G#3": "56",
    "Ab3": "56",
    "A3": "57",
    "A#3": "58",
    "Bb3": "58",
    "B3": "59",
    "C4": "60",
    "Db4": "61",
    "C#4": "61",
    "D4": "62",
    "Eb4": "63",
    "D#4": "63",
    "E4": "64",
    "F4": "65",
    "Gb4": "66",
    "F#4": "66",
    "G4": "67",
    "G#4": "68",
    "Ab4": "68",
    "A4": "69",
    "A#4": "70",
    "Bb4": "70",
    "B4": "71",
    "C5": "72",
    "Db5": "73",
    "C#5": "73",
    "D5": "74",
    "Eb5": "75",
    "D#5": "75",
    "E5": "76",
    "F5": "77",
    "Gb5": "78",
    "F#5": "78",
    "G5": "79",
    "G#4": "80",
    "Ab5": "80",
    "A5": "81",
    "A#5": "82",
    "Bb5": "82",
    "B5": "83",
}


drumMIDI = {"kick": "69", "snare": "67", "hihat": "65", "cymbal": "62"}


import socket
import os
from _thread import *

ServerSocket = socket.socket()
host = "localhost"
# host = '192.168.4.199'
port = 1244
ThreadCount = 0
SelectedChord = "C"
dummy_off = [144, 69, 0]

try:
    ServerSocket.bind((host, port))
except socket.error as e:
    print(str(e))

print("Waitiing for a Connection..")
ServerSocket.listen(5)


def threaded_client(connection):
    global SelectedChord

    prevNote = "C4"
    c = Chord(SelectedChord)
    k = 1
    velocity = 0
    connection.send(str.encode(str(ThreadCount)))
    while True:
        try:
            data = list(map(str, connection.recv(2048).decode("utf-8").split(" ")))
            # if(data[0]!=''):
            #     print(data)

            # GUITAR
            # message2="XX G 2 0 0"  # port instrument octave note velocity
            if data[1] == "G":
                if data[3] == "all":
                    midion = [int(data[0]), 123, 0]
                    midiout.send_message(midion)
                else:
                    note = str(c.components()[int(data[3])]) + data[2]
                    midion = [int(data[0]), int(chord_midi_Number[note]), int(data[4])]
                    midiout.send_message(midion)

            # message2="XX D snare 100"  # port instrument kit velocity

            if data[1] == "D":
                note = int(drumMIDI[data[2]])
                midion = [int(data[0]), int(note), int(data[3])]
                print(midion)
                midiout.send_message(midion)

            # message=status+"C Am 0"   # port  instrument chord velocity octave
            if data[1] == "C":
                print("hello :: ", data[0], data[1], data[2], data[3], data[4])
                SelectedChord = data[2]
                c = Chord(SelectedChord)
                if int(data[3]) == 0:
                    midiout.send_message(dummy_off)
                    print(dummy_off)
                else:
                    midion = [
                        int(data[0]),
                        int(chord_midi_Number[c.components()[0] + data[4]]),
                        int(data[3]),
                    ]
                    dummy_off = [
                        int(data[0]),
                        int(chord_midi_Number[c.components()[0] + data[4]]),
                        0,
                    ]
                    print(midion)
                    midiout.send_message(midion)

        except Exception as e:
            print(e)
            pass

        if not data:
            break
    connection.close()


a = datetime.now()  # use this
while ThreadCount <= 7:
    Client, address = ServerSocket.accept()
    print("Connected to: " + address[0] + ":" + str(address[1]))
    start_new_thread(threaded_client, (Client,))
    ThreadCount += 1
    print("Thread Number: " + str(ThreadCount))

ServerSocket.close()
midiout.close_port()
