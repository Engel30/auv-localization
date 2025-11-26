# --- SYSTEM CONFIGURATION ---
# DO NOT CHANGE UNLESS NECESSARY AND YOU KNOW WHAT YOU ARE DOING.
# The things to change are in the USER CONFIGURATION section below

CODEC = "utf-8"
LOCAL_MACHINE_IP = "127.0.0.1"


# --- USER CONFIGURATION ---

# --- Device Mode Configuration ---
# Set to True if this device is the TRANSCEIVER (sends position data)
# Set to False if this device is the TRANSPONDER (receives position data)
IS_TRANSCEIVER = False  # Change this based on which device you're configuring

# --- Water Type Configuration ---
# Affects the speed of sound propagation in a given liquid, in order to estimate the device distance
# Set to True if the liquid is salt water
# Set to False if the liquid is not salt water, and modify according to liquid type
SEA_WATER_CONFIG = False

# --- usbl configuration ---
TRANSCEIVER_IP = "192.168.0.139"
TRANSPONDER_IP = "192.168.0.232"
USBL_PORT = 9200 # Transponder/transceiver: 9200
USBL_CONNECTION_TIMEOUT = 5 # In seconds

INSTANT_MESSAGE_INTERVAL_SEC = 1. # Interval between each instant message

TRANSCEIVER_ID = 2 # This can be assigned by manual configuration of the USBL device
TRANSPONDER_ID = 3 # Same description as above

# --- serial configuration ---
SERIAL_PORT_ID = "/dev/ttys043" # In windows use COM
SERIAL_BAUDRATE = 115200 # The receiver and transmitter devices MUST have the same value
SERIAL_CONNECTION_TIMEOUT = 5


# --- miscellaneous ---
# Used to parse the localization data correctly. Change only if necessary based on type of water

if SEA_WATER_CONFIG is True:
    SPEED_OF_SOUND_MPS = 1500.0
else:
    SPEED_OF_SOUND_MPS = 1480.0 #tap water speed, adjust based on liquid type


# --- Ready-to-go, working configurations ---

#updates IP and ID based on device type
if IS_TRANSCEIVER is True:
    CONFIG = TRANSCEIVER_IP, TRANSCEIVER_ID
else:
    CONFIG = TRANSPONDER_IP, TRANSPONDER_ID

DUMMY_TEST_CONFIG = LOCAL_MACHINE_IP, TRANSPONDER_ID  #dummy config for testing without hardware