#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
USBL Mission Control (Transceiver) - REACTIVE MODE
Cattura automaticamente USBLLONG/USBLANGLES dal modem.
Priorita' a USBLLONG, fallback su USBLANGLES + AT?T per distanza.
Risponde al veicolo quando riceve un messaggio.

Protocollo: struct binario + base64 (max 64 byte payload)
"""

import asyncio
import json
import time
import math
import os
import struct
import base64
import logging
from datetime import datetime

# =============================================================================
#  CONFIGURATION
# =============================================================================
TRANSCEIVER_ID = 2
TRANSPONDER_ID = 3

USBL_IP = "192.168.0.139"
USBL_PORT = 9200
CODEC = "utf-8"
SPEED_OF_SOUND_MPS = 1500.0
MIN_INTEGRITY = 50

# Timeout per AT?T query [s]
PROP_TIME_TIMEOUT = 2.0

# Scadenza dati USBL [s] - dopo questo tempo i dati sono considerati vecchi
USBL_DATA_EXPIRY = 10.0

ENABLE_LOGGING = True
LOG_DIRECTORY = "mission_control_logs"

# =============================================================================
#  DEFAULT VALUES - CONTROL DATA
# =============================================================================
# Questi valori vengono sempre usati per control_data.
# Puoi sostituirli con funzioni o logica dinamica.

CONTROL_ENABLE = True               # <-- MODIFY: abilita/disabilita controllo
CONTROL_BIAS_ANGLE_DEG = 0.0        # <-- MODIFY: angolo bias [gradi]
CONTROL_FREQUENCY_HZ = 0.5          # <-- MODIFY: frequenza [Hz]


# =============================================================================
#  DEFAULT VALUES - USBL (usati se modem non fornisce dati)
# =============================================================================
DEFAULT_USBL_INTEGRITY = 100        # <-- MODIFY: integrity di default
DEFAULT_USBL_RSSI = -40             # <-- MODIFY: RSSI di default [dBm]


# =============================================================================
#  HELPER FUNCTIONS - CUSTOMIZE THESE
# =============================================================================
def get_control_data():
    """
    Ritorna i dati di controllo da inviare al veicolo.
    SOSTITUISCI questa funzione con la tua logica di controllo.
    """
    return {
        'enable': CONTROL_ENABLE,
        'bias_deg': CONTROL_BIAS_ANGLE_DEG,
        'freq_hz': CONTROL_FREQUENCY_HZ
    }


def on_vehicle_telemetry_received(data):
    """
    Callback chiamato quando arriva telemetria dal veicolo.
    SOSTITUISCI questa funzione per processare i dati.
    """
    pos = data['pose']['position']
    ori = data['pose']['orientation']
    bat = data['sensor_data']['battery']
    dep = data['sensor_data']['depth_sensor']
    
    print("\n" + "=" * 60)
    print("[VEHICLE TELEMETRY]")
    print("=" * 60)
    print("  Timestamp: {:.2f}s".format(data['timestamp']))
    print("  Position:  x={:.2f}  y={:.2f}  z={:.2f} m".format(
        pos['x'], pos['y'], pos['z']))
    print("  Orientation: roll={:.1f}  pitch={:.1f}  yaw={:.1f} deg".format(
        ori['roll'], ori['pitch'], ori['yaw']))
    print("  Battery: {:.1f}% @ {:.1f}V".format(
        bat['charge_percent'], bat['voltage_volts']))
    print("  Depth sensor: {:.1f}C, {:.1f}mbar".format(
        dep['temperature'], dep['pressure']))


def on_usbl_data_updated(usbl_state):
    """
    Callback chiamato quando arrivano nuovi dati USBL dal modem.
    SOSTITUISCI per reagire ai dati USBL.
    """
    if usbl_state['mode'] == 1:
        print("\n[USBL UPDATE] LONG mode:")
        print("  ENU: E={:.2f} N={:.2f} U={:.2f} m".format(
            usbl_state['east'], usbl_state['north'], usbl_state['up']))
    else:
        print("\n[USBL UPDATE] ANGLES mode:")
        print("  Bearing: {:.2f} deg, Elevation: {:.2f} deg".format(
            math.degrees(usbl_state['bearing_rad']),
            math.degrees(usbl_state['elevation_rad'])))
    print("  Distance: {:.2f} m".format(usbl_state['distance']))
    print("  Integrity: {}, RSSI: {} dBm".format(
        usbl_state['integrity'], usbl_state['rssi']))


# =============================================================================
#  MESSAGE PROTOCOL
# =============================================================================
MSG_TYPE_VEHICLE = 0x01
MSG_TYPE_MC = 0x02

STRUCT_VEHICLE = '<Bfffffffffff'
STRUCT_MC = '<BfBfffffffBbBff'


def decode_vehicle_message(payload_b64):
    """Decodifica messaggio dal veicolo."""
    try:
        data = base64.b64decode(payload_b64)
        values = struct.unpack(STRUCT_VEHICLE, data)
        
        if values[0] != MSG_TYPE_VEHICLE:
            return None
        
        return {
            'timestamp': values[1],
            'pose': {
                'position': {
                    'x': values[2],
                    'y': values[3],
                    'z': values[4]
                },
                'orientation': {
                    'roll': values[5],
                    'pitch': values[6],
                    'yaw': values[7]
                }
            },
            'sensor_data': {
                'battery': {
                    'charge_percent': values[8],
                    'voltage_volts': values[9]
                },
                'depth_sensor': {
                    'temperature': values[10],
                    'pressure': values[11]
                }
            }
        }
    except Exception as e:
        print("[DECODE] Error: {}".format(e))
        return None


def encode_mc_message(timestamp, usbl, control):
    """
    Codifica messaggio per il veicolo.
    
    Args:
        timestamp: float
        usbl: dict con mode, east, north, up, bearing_rad, elevation_rad, 
              distance, prop_time_us, integrity, rssi
        control: dict con enable, bias_deg, freq_hz
    """
    data = struct.pack(
        STRUCT_MC,
        MSG_TYPE_MC,
        float(timestamp),
        int(usbl['mode']) & 0xFF,
        float(usbl.get('east', 0.0)),
        float(usbl.get('north', 0.0)),
        float(usbl.get('up', 0.0)),
        float(usbl.get('bearing_rad', 0.0)),
        float(usbl.get('elevation_rad', 0.0)),
        float(usbl.get('distance', 0.0)),
        float(usbl.get('prop_time_us', 0.0)),
        int(usbl.get('integrity', DEFAULT_USBL_INTEGRITY)) & 0xFF,
        max(-128, min(127, int(usbl.get('rssi', DEFAULT_USBL_RSSI)))),
        1 if control['enable'] else 0,
        float(control['bias_deg']),
        float(control['freq_hz'])
    )
    return base64.b64encode(data).decode('ascii')


# =============================================================================
#  LOGGER
# =============================================================================
class MissionControlLogger(object):
    def __init__(self, directory, enabled=True):
        self.enabled = enabled
        self.log_file = None
        self.filepath = None
        self.session_start = None
        
        if self.enabled:
            if not os.path.exists(directory):
                os.makedirs(directory)
            
            self.session_start = datetime.now()
            timestamp = self.session_start.strftime("%Y%m%d_%H%M%S")
            self.filepath = os.path.join(directory, "mc_log_{}.jsonl".format(timestamp))
            self.log_file = open(self.filepath, 'w')
            
            self._write({
                "event_type": "SESSION_START",
                "timestamp": self.session_start.isoformat(),
                "config": {
                    "transceiver_id": TRANSCEIVER_ID,
                    "transponder_id": TRANSPONDER_ID,
                    "usbl_ip": USBL_IP
                }
            })
            print("[LOG] File: {}".format(self.filepath))
    
    def _write(self, event):
        if self.log_file and not self.log_file.closed:
            self.log_file.write(json.dumps(event, separators=(',', ':')) + '\n')
            self.log_file.flush()
    
    def log_event(self, event_type, data):
        if not self.enabled:
            return
        self._write({
            "event_type": event_type,
            "timestamp": datetime.now().isoformat(),
            "data": data
        })
    
    def close(self):
        if self.log_file and not self.log_file.closed:
            self.log_event("SESSION_END", {})
            self.log_file.close()


# =============================================================================
#  STATE
# =============================================================================
state = {
    "session_start": None,
    "logger": None,
    "writer": None,
    
    # Dati USBL dal modem (aggiornati automaticamente)
    "usbl": {
        "mode": 0,              # 0=ANGLES, 1=LONG
        "east": 0.0,
        "north": 0.0,
        "up": 0.0,
        "bearing_rad": 0.0,
        "elevation_rad": 0.0,
        "distance": 0.0,
        "prop_time_us": 0.0,
        "integrity": DEFAULT_USBL_INTEGRITY,
        "rssi": DEFAULT_USBL_RSSI,
        "valid": False,
        "timestamp": 0.0,       # Quando sono stati aggiornati
        "source": "none"        # "usbllong", "usblangles", "none"
    },
    
    # Ultimo propagation time da AT?T
    "last_prop_time_ms": None,
    
    # Communication state
    "messages_received": 0,
    "messages_sent": 0,
    "usbllong_count": 0,
    "usblangles_count": 0,
    
    # Delivery
    "waiting_delivery": False,
    "delivery_event": None,
    "delivery_result": None,
    
    # AT?T query
    "waiting_prop_time": False,
    "prop_time_event": None,
    "prop_time_value": None
}


# =============================================================================
#  MODEM COMMUNICATION
# =============================================================================
def decode_modem_response(message):
    """Decodifica risposta dal modem EvoLogics."""
    try:
        str_msg = message.decode(CODEC).strip()
        if not str_msg:
            return "EMPTY", {}
        
        if str_msg == "OK":
            return "OK", {}
        
        if not str_msg.startswith("+++"):
            # Potrebbe essere risposta AT?T
            return "DIRECT_VALUE", {"value": str_msg}
        
        parts = str_msg.split(":")
        
        if "AT" in str_msg and len(parts) >= 3:
            args = parts[2].split(",")
            keyword = args[0]
            
            if keyword == "DELIVEREDIM":
                return "DELIVEREDIM", {"target_id": args[1] if len(args) > 1 else "?"}
            
            elif keyword == "FAILEDIM":
                return "FAILEDIM", {"target_id": args[1] if len(args) > 1 else "?"}
            
            elif keyword == "RECVIM" and len(args) >= 9:
                try:
                    sender_id = args[3]
                    rssi = int(args[7])
                    integrity = int(args[8])
                    
                    payload = None
                    full_msg = parts[2]
                    comma_parts = full_msg.split(",")
                    if len(comma_parts) >= 10:
                        payload = ",".join(comma_parts[9:])
                    
                    return "RECVIM", {
                        "sender_id": sender_id,
                        "integrity": integrity,
                        "valid": integrity > MIN_INTEGRITY,
                        "rssi": rssi,
                        "payload": payload
                    }
                except (ValueError, IndexError):
                    return "RECVIM_ERROR", {}
            
            # USBLLONG - posizione completa (PRIORITARIO)
            elif keyword == "USBLLONG" and len(args) >= 17:
                try:
                    return "USBLLONG", {
                        "target_id": args[3],
                        "east": float(args[7]),
                        "north": float(args[8]),
                        "up": float(args[9]),
                        "propagation_time_us": int(args[13]),
                        "rssi": int(args[14]),
                        "integrity": int(args[15]),
                        "valid": int(args[15]) > MIN_INTEGRITY
                    }
                except (ValueError, IndexError):
                    return "USBLLONG_ERROR", {}
            
            # USBLANGLES - solo direzione (FALLBACK)
            elif keyword == "USBLANGLES" and len(args) >= 13:
                try:
                    return "USBLANGLES", {
                        "target_id": args[3],
                        "bearing_rad": float(args[6]),
                        "elevation_rad": float(args[7]),
                        "rssi": int(args[11]),
                        "integrity": int(args[12]),
                        "valid": int(args[12]) > MIN_INTEGRITY
                    }
                except (ValueError, IndexError):
                    return "USBLANGLES_ERROR", {}
            
            elif keyword in ["SENDSTART", "SENDEND", "RECVSTART", "RECVEND"]:
                return keyword, {}
        
        return "OTHER", {}
    except Exception:
        return "ERROR", {}


def get_relative_timestamp():
    if state["session_start"] is None:
        state["session_start"] = time.time()
    return time.time() - state["session_start"]


def is_usbl_data_valid():
    """Verifica se i dati USBL sono validi e non scaduti."""
    if not state["usbl"]["valid"]:
        return False
    age = time.time() - state["usbl"]["timestamp"]
    return age < USBL_DATA_EXPIRY


def get_usbl_data_for_transmission():
    """
    Prepara i dati USBL da trasmettere.
    Priorita': USBLLONG > USBLANGLES
    """
    usbl = state["usbl"]
    
    return {
        'mode': usbl['mode'],
        'east': usbl['east'],
        'north': usbl['north'],
        'up': usbl['up'],
        'bearing_rad': usbl['bearing_rad'],
        'elevation_rad': usbl['elevation_rad'],
        'distance': usbl['distance'],
        'prop_time_us': usbl['prop_time_us'],
        'integrity': usbl['integrity'],
        'rssi': usbl['rssi']
    }


async def send_command(command):
    """Invia comando AT al modem."""
    if state["writer"]:
        cmd_bytes = "+++{}\n".format(command).encode(CODEC)
        state["writer"].write(cmd_bytes)
        await state["writer"].drain()


async def query_propagation_time():
    """Interroga propagation time con AT?T."""
    state["waiting_prop_time"] = True
    state["prop_time_value"] = None
    state["prop_time_event"].clear()
    
    await send_command("AT?T")
    
    try:
        await asyncio.wait_for(state["prop_time_event"].wait(), timeout=PROP_TIME_TIMEOUT)
        return state["prop_time_value"]
    except asyncio.TimeoutError:
        state["waiting_prop_time"] = False
        return None


async def send_response_to_vehicle():
    """Invia risposta al veicolo con dati USBL + control."""
    
    # Verifica se abbiamo dati USBL validi
    if not is_usbl_data_valid():
        print("[MC] WARNING: No valid USBL data to send")
        # Invia comunque con valori di default/zero
    
    usbl = get_usbl_data_for_transmission()
    control = get_control_data()
    ts = get_relative_timestamp()
    
    payload = encode_mc_message(ts, usbl, control)
    
    print("\n[MC] Sending response to vehicle...")
    if usbl['mode'] == 1:
        print("  Mode: LONG (ENU)")
        print("  E={:.2f} N={:.2f} U={:.2f} m".format(
            usbl['east'], usbl['north'], usbl['up']))
    else:
        print("  Mode: ANGLES")
        print("  Bearing={:.1f}deg Elevation={:.1f}deg".format(
            math.degrees(usbl['bearing_rad']),
            math.degrees(usbl['elevation_rad'])))
    print("  Distance: {:.2f} m".format(usbl['distance']))
    print("  Control: enable={} bias={:.1f}deg freq={:.2f}Hz".format(
        control['enable'], control['bias_deg'], control['freq_hz']))
    
    command = "AT*SENDIM,p0,{},{},ack,{}".format(len(payload), TRANSPONDER_ID, payload)
    encoded = "+++{}\n".format(command).encode(CODEC)
    
    state["waiting_delivery"] = True
    state["delivery_result"] = None
    state["delivery_event"].clear()
    
    state["writer"].write(encoded)
    await state["writer"].drain()
    state["messages_sent"] += 1
    
    if state["logger"]:
        state["logger"].log_event("SENT_TO_VEHICLE", {
            "usbl": usbl,
            "control": control
        })
    
    try:
        await asyncio.wait_for(state["delivery_event"].wait(), timeout=5.0)
        if state["delivery_result"]:
            print("[MC] Response delivered")
        else:
            print("[MC] Response FAILED")
    except asyncio.TimeoutError:
        print("[MC] Response TIMEOUT")
        state["waiting_delivery"] = False


async def handle_usbllong(data):
    """Gestisce dati USBLLONG dal modem - PRIORITARIO."""
    if not data["valid"]:
        return
    
    state["usbllong_count"] += 1
    
    # Calcola distanza da propagation time
    distance = (data["propagation_time_us"] / 1000000.0) * SPEED_OF_SOUND_MPS
    
    # Aggiorna stato USBL (mode=1 = LONG, prioritario)
    state["usbl"]["mode"] = 1
    state["usbl"]["east"] = data["east"]
    state["usbl"]["north"] = data["north"]
    state["usbl"]["up"] = data["up"]
    state["usbl"]["distance"] = distance
    state["usbl"]["prop_time_us"] = data["propagation_time_us"]
    state["usbl"]["integrity"] = data["integrity"]
    state["usbl"]["rssi"] = data["rssi"]
    state["usbl"]["valid"] = True
    state["usbl"]["timestamp"] = time.time()
    state["usbl"]["source"] = "usbllong"
    
    # Log
    if state["logger"]:
        state["logger"].log_event("USBLLONG", data)
    
    # Callback
    on_usbl_data_updated(state["usbl"])


async def handle_usblangles(data):
    """Gestisce dati USBLANGLES dal modem - FALLBACK."""
    if not data["valid"]:
        return
    
    state["usblangles_count"] += 1
    
    # Se abbiamo dati USBLLONG recenti, ignora USBLANGLES
    if state["usbl"]["source"] == "usbllong":
        age = time.time() - state["usbl"]["timestamp"]
        if age < 2.0:  # Ignora se USBLLONG e' recente
            print("[MC] USBLANGLES ignored (recent USBLLONG available)")
            return
    
    # Query AT?T per ottenere distanza
    print("[MC] USBLANGLES received, querying AT?T for distance...")
    prop_time_ms = await query_propagation_time()
    
    if prop_time_ms is not None:
        # Calcola distanza (prop_time e' round-trip in ms)
        distance = (prop_time_ms / 1000.0 / 2.0) * SPEED_OF_SOUND_MPS
        state["usbl"]["distance"] = distance
        state["usbl"]["prop_time_us"] = prop_time_ms * 1000.0  # Converti in us
        state["last_prop_time_ms"] = prop_time_ms
    else:
        print("[MC] WARNING: AT?T timeout, using last known distance")
        # Usa ultimo valore noto o 0
    
    # Aggiorna stato USBL (mode=0 = ANGLES)
    state["usbl"]["mode"] = 0
    state["usbl"]["bearing_rad"] = data["bearing_rad"]
    state["usbl"]["elevation_rad"] = data["elevation_rad"]
    state["usbl"]["integrity"] = data["integrity"]
    state["usbl"]["rssi"] = data["rssi"]
    state["usbl"]["valid"] = True
    state["usbl"]["timestamp"] = time.time()
    state["usbl"]["source"] = "usblangles"
    
    # Calcola ENU da angoli per riferimento (non inviati, ma utili)
    if state["usbl"]["distance"] > 0:
        bearing = data["bearing_rad"]
        elevation = data["elevation_rad"]
        dist = state["usbl"]["distance"]
        state["usbl"]["east"] = dist * math.cos(elevation) * math.sin(bearing)
        state["usbl"]["north"] = dist * math.cos(elevation) * math.cos(bearing)
        state["usbl"]["up"] = dist * math.sin(elevation)
    
    # Log
    if state["logger"]:
        state["logger"].log_event("USBLANGLES", {
            "bearing_rad": data["bearing_rad"],
            "elevation_rad": data["elevation_rad"],
            "distance": state["usbl"]["distance"],
            "integrity": data["integrity"],
            "rssi": data["rssi"]
        })
    
    # Callback
    on_usbl_data_updated(state["usbl"])


async def response_listener(reader):
    """Task principale che ascolta messaggi dal modem."""
    print("[MC] Listening...")
    
    try:
        while True:
            line = await reader.readline()
            if not line:
                print("[MC] Connection closed")
                break
            
            msg_type, data = decode_modem_response(line)
            
            # Risposta AT?T
            if msg_type == "DIRECT_VALUE" and state["waiting_prop_time"]:
                try:
                    state["prop_time_value"] = float(data["value"])
                except (ValueError, KeyError):
                    pass
                state["waiting_prop_time"] = False
                state["prop_time_event"].set()
            
            # Delivery confirmation
            elif msg_type == "DELIVEREDIM":
                if state["waiting_delivery"]:
                    state["delivery_result"] = True
                    state["waiting_delivery"] = False
                    state["delivery_event"].set()
            
            elif msg_type == "FAILEDIM":
                if state["waiting_delivery"]:
                    state["delivery_result"] = False
                    state["waiting_delivery"] = False
                    state["delivery_event"].set()
            
            # *** USBLLONG - PRIORITARIO ***
            elif msg_type == "USBLLONG":
                await handle_usbllong(data)
            
            # *** USBLANGLES - FALLBACK ***
            elif msg_type == "USBLANGLES":
                await handle_usblangles(data)
            
            # *** MESSAGGIO DAL VEICOLO ***
            elif msg_type == "RECVIM":
                state["messages_received"] += 1
                
                if data.get("valid"):
                    payload = data.get("payload")
                    if payload:
                        vehicle_data = decode_vehicle_message(payload.strip())
                        
                        if vehicle_data:
                            if state["logger"]:
                                state["logger"].log_event("VEHICLE_TELEMETRY", vehicle_data)
                            
                            on_vehicle_telemetry_received(vehicle_data)
                            
                            # *** RISPONDI AUTOMATICAMENTE ***
                            await send_response_to_vehicle()
                        else:
                            print("[MC] Failed to decode vehicle message")
                else:
                    print("[MC] Low integrity message: {}".format(
                        data.get('integrity', '?')))
                        
    except asyncio.CancelledError:
        pass
    except Exception as e:
        print("[MC] Listener error: {}".format(e))


# =============================================================================
#  MAIN
# =============================================================================
async def main():
    logging.basicConfig(level=logging.INFO, format="%(message)s")
    
    state["logger"] = MissionControlLogger(LOG_DIRECTORY, ENABLE_LOGGING)
    state["delivery_event"] = asyncio.Event()
    state["prop_time_event"] = asyncio.Event()
    state["session_start"] = time.time()
    
    print("=" * 70)
    print("MISSION CONTROL - AUTO USBL MODE")
    print("=" * 70)
    print("Transceiver ID: {}".format(TRANSCEIVER_ID))
    print("Transponder ID: {} (vehicle)".format(TRANSPONDER_ID))
    print("USBL IP:        {}:{}".format(USBL_IP, USBL_PORT))
    print("Sound Speed:    {} m/s".format(SPEED_OF_SOUND_MPS))
    print("")
    print("USBL MODE PRIORITY:")
    print("  1. USBLLONG (ENU coordinates) - preferred")
    print("  2. USBLANGLES + AT?T (bearing/elevation + distance) - fallback")
    print("")
    print("CONTROL DEFAULTS (modify in script):")
    print("  CONTROL_ENABLE = {}".format(CONTROL_ENABLE))
    print("  CONTROL_BIAS_ANGLE_DEG = {}".format(CONTROL_BIAS_ANGLE_DEG))
    print("  CONTROL_FREQUENCY_HZ = {}".format(CONTROL_FREQUENCY_HZ))
    print("=" * 70)
    
    try:
        reader, writer = await asyncio.open_connection(USBL_IP, USBL_PORT)
        state["writer"] = writer
        print("[MC] Connected to USBL modem")
        print("[MC] Waiting for USBL data and vehicle messages...")
        print("")
    except Exception as e:
        print("[MC] Connection failed: {}".format(e))
        return
    
    listener_task = asyncio.ensure_future(response_listener(reader))
    
    try:
        await listener_task
    except KeyboardInterrupt:
        print("\n[MC] Stopping...")
    finally:
        listener_task.cancel()
        try:
            await listener_task
        except asyncio.CancelledError:
            pass
        
        writer.close()
        
        if state["logger"]:
            state["logger"].close()
        
        print("\n" + "=" * 70)
        print("SUMMARY")
        print("=" * 70)
        print("Messages received: {}".format(state["messages_received"]))
        print("Messages sent:     {}".format(state["messages_sent"]))
        print("USBLLONG count:    {}".format(state["usbllong_count"]))
        print("USBLANGLES count:  {}".format(state["usblangles_count"]))
        print("Last USBL source:  {}".format(state["usbl"]["source"]))
        print("=" * 70)


if __name__ == '__main__':
    try:
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        loop.run_until_complete(main())
    except KeyboardInterrupt:
        pass
    finally:
        loop.close()
