#!/usr/bin/env python3

import struct
from bluepy.btle import Peripheral, DefaultDelegate
import bluepy
import argparse
import time

# parser = argparse.ArgumentParser(description='Print advertisement data from a BLE device')
# parser.add_argument('addr', metavar='A', type=str, help='Address of the form XX:XX:XX:XX:XX:XX')
# args = parser.parse_args()
addr1 = 'C0:98:E5:49:00:44'
addr2 = 'C0:98:E5:49:00:45'
addr3 = 'C0:98:E5:49:00:46'

addresses = [addr1, addr2, addr3]


MAX_PLATOON_PERIOD = 30
INITIAL_CONFIG = [-1, 0, -1]
DEFAULT_SPEED = 70
MAX_SPEED_OFFSET = 125
MIN_SPEED_OFFSET = 30
# addresses = [addr2]

F_VALUE = 0.1 # 1 worked 0.5 too big, 0.05 too small imo
GOAL_DISTANCE = 100 #in mm
GOAL_ENCODER = int((GOAL_DISTANCE + 250)// 0.65)

# if len(addr) != 17:
#     raise ValueError("Invalid address supplied")
DEFAULT_INSTRUCTION = [1, DEFAULT_SPEED, GOAL_DISTANCE] # Lead Toggle, speed, follow_distance

BUCKLER_SERVICE_UUID = "32e61089-2b22-4db5-a914-43ce41986c70"
DATA_CHAR_UUID    = "32e6108a-2b22-4db5-a914-43ce41986c70"
INSTRUCTION_CHAR_UUID    = "32e6108b-2b22-4db5-a914-43ce41986c70"

def connect_to_bucklers(bucklers):
    for i in range(len(bucklers)):
        state = ""
        try:
            state = bucklers[i].getState()
        except bluepy.btle.BTLEInternalError:
            pass
        if state != 'conn':
            print("Connecting to " + addresses[i])
            bucklers[i].connect(addresses[i])
            print("Connected to " + addresses[i])
    return bucklers

def unpack(data):
    return struct.unpack('I', data[0:4]) + struct.unpack('I', data[4:8]) + struct.unpack('I', data[8:12]) + struct.unpack('I', data[12:16])


def format_instructions(instr):
    return struct.pack('B', instr[0]) + struct.pack('B', instr[1]) + struct.pack('B', instr[2])

def run(bucklers):
    print("Start RUN")
    start = time.time_ns()



    svs = [b.getServiceByUUID(BUCKLER_SERVICE_UUID) for b in bucklers]
    # Get characteristic
    # ch = sv.getCharacteristics(LED_CHAR_UUID)[0]
    data_chs = [s.getCharacteristics(DATA_CHAR_UUID)[0] for s in svs]
    instruction_chs = [s.getCharacteristics(INSTRUCTION_CHAR_UUID)[0] for s in svs]

    checkpoint = [(0, 0) for _ in bucklers]
    robot_data = [[] for _ in bucklers]
    lead_follow = INITIAL_CONFIG[:] # = lead, n = following robot n
    send_instructions = [DEFAULT_INSTRUCTION[:] for _ in bucklers]
    intersection = []
    print(GOAL_ENCODER)
    recent_exits = [-1, 0]
    while True:
        recent_exits[1] -= 1
        #input("Press any key to toggle the LED")
        # time.sleep(0.5)
        for num in range(len(data_chs)):
            ch = data_chs[num]
            data = ch.read()
            try:
                char = unpack(data)
                robot_data[num] = char
                if char[3] != checkpoint[num][1]:
                    checkpoint[num] = (char[0], char[3])
                    if char[3] %2 == 1:
                        intersection.append(num)
                        lead_follow[num] = -1
                        print(intersection)

                    if char[3] != 0 and char[3] %2 == 0:
                        if num in intersection:
                            intersection.remove(num)
                        if recent_exits[1] > 0:
                            lead_follow[num] = recent_exits[0]
                        else:
                            recent_exits[0] = num
                            recent_exits[1] = MAX_PLATOON_PERIOD
                            lead_follow[num] = -1
                        send_instructions[num] = DEFAULT_INSTRUCTION[:]
            except struct.error:
                pass

            ## PRINTING OUT DATA
            print("ROBOT READ {0}".format(num))
            print(char[1], char[3])


        # speed_difference = 0
        for i in range(len(intersection)):
            if i == 0:
                send_instructions[intersection[i]] = DEFAULT_INSTRUCTION[:]
            else:
                curr_car = intersection[i]
                prev_car = intersection[i-1]
                print("CHANGING SPEED FOR CAR {0} AND PREV CAR {1}".format(curr_car, prev_car))

                curr_car_encoder = robot_data[curr_car][1]
                prev_car_encoder = robot_data[prev_car][1]
                new_speed_difference = int(((prev_car_encoder - curr_car_encoder) - GOAL_ENCODER)*F_VALUE)
                print(curr_car_encoder, prev_car_encoder, GOAL_ENCODER)

                curr_car_speed = send_instructions[curr_car][1]
                new_speed = new_speed_difference - (send_instructions[prev_car][1] - DEFAULT_SPEED) + DEFAULT_SPEED
                # speed_difference = new_speed_difference
                actual_speed = min(max(new_speed, MIN_SPEED_OFFSET), MAX_SPEED_OFFSET)
                # speed_difference += actual_speed - new_speed
                send_instructions[curr_car][1] = actual_speed

                print("CHANGING SPEED TO {0} DESIRED SPEED {1}".format(actual_speed, new_speed))

        print(send_instructions)
        print(lead_follow)
        print(intersection)
        for num in range(len(instruction_chs)):
            # print("ROBOT {0}".format(num))
            # print(send_instructions[num][1])
            ch = instruction_chs[num]
            curr_time = (time.time_ns() - start)//1000
            curr_time = struct.pack('I', curr_time)
            ch.write(bytearray(format_instructions([1 if lead_follow[num] == -1 else 0] + send_instructions[num][1:]) + curr_time))

def main():

    bucklers = [Peripheral() for a in addresses]
    while True:
        try:
            print("connecting")
            bucklers = connect_to_bucklers(bucklers)
            print("Conencted to all bucklers")
            run(bucklers)

        except bluepy.btle.BTLEDisconnectError:
            print("Disconnected. Trying to Reconnect")

    for b in bucklers:
        b.disconnect()

main()
           #characteristics = buckler.getCharacteristics(startHnd=0x0019, endHnd=0xFFFF, uuid=None)
            #for characteristic in characteristics:
            #    print("{}, hnd={}, supports {}".format(characteristic, hex(characteristic.handle),  characteristic.propertiesToString()))
            # Get service
            # sv = buckler.getServiceByUUID(LED_SERVICE_UUID)
