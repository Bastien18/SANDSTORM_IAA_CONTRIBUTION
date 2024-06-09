import logging
import sys
import time
import threading
import math
import argparse
from threading import Event, Lock
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils import uri_helper

from connect import Connect

# URI to the Crazyflie to connect to
uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E705')
deck_attached_event = Event()

# Variable globale pour stocker les coordonnées des lignes
line_coords = []
coords_lock = threading.Lock()

# ========================== LOGGING PART ==========================
# Not realy usefull to fly the drone but could be handy for debugging features

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)
position_estimate = [0, 0]

def simple_connect():
    print("Yeah, I'm connected! :D")
    time.sleep(3)
    print("Now I will disconnect :'(")

def simple_log(scf, logconf):

    with SyncLogger(scf, logconf) as logger:

        for log_entry in logger:

            timestamp = log_entry[0]
            data = log_entry[1]
            logconf_name = log_entry[2]

            print('[%d][%s]: %s' % (timestamp, logconf_name, data))

            break

def log_pos_callback(timestamp, data, logconf):
    print(data)
    global position_estimate
    position_estimate[0] = data['stateEstimate.x']
    position_estimate[1] = data['stateEstimate.y']

def log_stab_callback(timestamp, data, logconf):
    print('[%d][%s]: %s' % (timestamp, logconf.name, data))

def simple_log_async(scf, logconf):
    cf = scf.cf
    cf.log.add_config(logconf)
    logconf.data_received_cb.add_callback(log_stab_callback)
    logconf.start()
    time.sleep(5)
    logconf.stop()

# ========================== MOTION COMMANDER PART ==========================

# Drone fly at a distance of 10cm from the ground
DEFAULT_HEIGHT = 0.10
# Not used
BOX_LIMIT = 0.5

# Ensur deck flow is attached if not the crazyflie might encounter some troubles flying
def param_deck_flow(_, value_str):
    value = int(value_str)
    print(value)
    if value:
        deck_attached_event.set()
        print('Deck is attached!')
    else:
        print('Deck is NOT attached!')

# Simple mvmt used when developping app
def move_linear_simple(scf):
    with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
        time.sleep(1)
        mc.forward(0.5)
        time.sleep(1)
        mc.turn_left(180)
        time.sleep(1)
        mc.forward(0.5)
        time.sleep(10)

# Comput steering angle and give the direction our drone must turn to in order to follow the line correctly
def compute_steering_angle(steering_vector):
    x1, x2, y2 = steering_vector
    y1 = 244  # y1 is always 244

    # Calculate the angle in radians relative to the vertical axis
    angle_rad = math.atan2(x2 - x1, y1 - y2)
    
    # Convert angle to degrees
    angle_deg = math.degrees(angle_rad)
    
    # Normalize the angle to the range [-180, 180]
    if angle_deg < -180:
        angle_deg += 360
    elif angle_deg > 180:
        angle_deg -= 360
    
    # Determine the direction
    if angle_deg > 0:
        direction = 'right'
    else:
        direction = 'left'

    # Security check, avoid our drone turn into a bayblade
    if(abs(angle_deg) > 90):
        angle_deg = 90
    
    return abs(angle_deg), direction

# Proportionaly adapte distance our drone must go to, we shorten the distance when the steering angle is high
def compute_distance(angle, max_distance=0.3, min_distance=0.02):
    """
    Compute the distance the drone should move based on the angle.
    The further the angle is from 0, the shorter the distance.
    """
    
    # Linear interpolation
    distance = max_distance - (angle / 90) * (max_distance - min_distance)
    return distance

# Correct direction of the drone to follow the line
def correct_direction(mc, angle, direction):
    if(direction == 'left'):
            mc.turn_left(angle)
    else:
        mc.turn_right(angle)

# Auto pilot of our drone
def take_off_simple(scf):
    global line_coords
    with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
        fly_start = time.time()
        time.sleep(5) 

        while((time.time() - fly_start) < 40):
            with coords_lock:
                angle, direction = compute_steering_angle(line_coords)
                print(angle, direction)
            correct_direction(mc, angle, direction)
            mc.forward(compute_distance(angle))

        time.sleep(1)
        mc.stop()

# Ask user before takeoff, sometimes camera streaming is a bit temperamental
def ask_user():
    while True:
        user_input = input("Please enter 'y' or 'n': ").strip().lower()
        if user_input == 'y':
            print("Continuing the program...")
            # Place the rest of your program logic here
            break
        elif user_input == 'n':
            print("Exiting the program...")
            sys.exit(1)
        else:
            print("Invalid input. Please enter 'y' or 'n'.")

# ==================================== IMAGE FETCHING PART ====================================

# Args for setting IP/port of AI-deck. Default settings are for when AI-deck is in AP mode.
parser = argparse.ArgumentParser(description='Connect to AI-deck JPEG streamer example')
parser.add_argument("-n", default="192.168.4.1", metavar="ip", help="AI-deck IP")
parser.add_argument("-p", type=int, default=5000, metavar="port", help="AI-deck port")
parser.add_argument('--save', action='store_true', help="Save streamed images")
args = parser.parse_args()

deck_port = args.p
deck_ip = args.n

fetch_lock = Lock()

# Start the fetching image thread
def start_thread(connect_obj):
    fetch_thread = threading.Thread(target=connect_obj.run)
    fetch_thread.start()
    return fetch_thread

if __name__ == '__main__':
    # Initialize the low-level drivers
    cflib.crtp.init_drivers()

    lg_stab = LogConfig(name='Stabilizer', period_in_ms=10)
    lg_stab.add_variable('stabilizer.roll', 'float')
    lg_stab.add_variable('stabilizer.pitch', 'float')
    lg_stab.add_variable('stabilizer.yaw', 'float')

    # Initialisation de l'image fetcher
    # Utilisation
    connect_obj = Connect(deck_ip, deck_port, args.save, line_coords, coords_lock)
    fetch_thread = start_thread(connect_obj)

    time.sleep(10)

    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        # simple_connect()
        #simple_log_async(scf, lg_stab)

        scf.cf.param.add_update_callback(group="deck", name="bcFlow2",
                                cb=param_deck_flow)
        time.sleep(1)

        logconf = LogConfig(name='Position', period_in_ms=10)
        logconf.add_variable('stateEstimate.x', 'float')
        logconf.add_variable('stateEstimate.y', 'float')
        scf.cf.log.add_config(logconf)
        logconf.data_received_cb.add_callback(log_pos_callback)

        if not deck_attached_event.wait(timeout=5):
            print('No flow deck detected!')
            sys.exit(1)

        ask_user()

        # Uncomment these line and then drone go brrrrrrr
        #commanderThread = threading.Thread(target=take_off_simple, args=(scf,))
        #commanderThread.start()
        #commanderThread.join()
    
    fetch_thread.join()