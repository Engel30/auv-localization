# --- SYSTEM CONFIGURATION ---
# DO NOT CHANGE UNLESS NECESSARY AND YOU KNOW WHAT YOU ARE DOING.
# The things to change are in the USER CONFIGURATION section below

CODEC = "utf-8"
LOCAL_MACHINE_IP = "127.0.0.1"


# --- USER CONFIGURATION ---

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
SPEED_OF_SOUND_MPS = 1500.0

# --- Ready-to-go, working configurations ---
TRANSPONDER_CONFIG = TRANSPONDER_IP, TRANSCEIVER_ID
TRANSCEIVER_CONFIG = TRANSCEIVER_IP, TRANSPONDER_ID
DUMMY_TEST_CONFIG = LOCAL_MACHINE_IP, TRANSPONDER_ID  # Add this line