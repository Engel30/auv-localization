#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
USBL Mission Control (Transceiver) - REACTIVE MODE

Pattern corretto:
1. Riceve RECVIM dal transponder -> NON risponde subito
2. Aspetta USBLANGLES o USBLLONG dal modem
3. Se USBLANGLES -> query AT?T per distanza
4. SOLO DOPO risponde al transponder con dati USBL

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

# Timeout per aspettare USBLANGLES/USBLLONG dopo RECVIM [s]
POSITION_CAPTURE_TIMEOUT = 3.0

# Timeout per AT?T query [s]
PROP_TIME_TIMEOUT = 1.0

ENABLE_LOGGING = True
LOG_DIRECTORY = "mission_control_logs"

# =============================================================================
#  DEFAULT VALUES - CONTROL DATA
# =============================================================================
CONTROL_ENABLE = True
CONTROL_BIAS_ANGLE_DEG = 0.0
CONTROL_FREQUENCY_HZ = 0.5
CONTROL_USE_RANGE_ONLY = False

# =============================================================================
#  DEFAULT VALUES - USBL
# =============================================================================
DEFAULT_USBL_INTEGRITY = 100
DEFAULT_USBL_RSSI = -40


# =============================================================================
#  HELPER FUNCTIONS
# =============================================================================
def get_control_data():
    """Ritorna i dati di controllo da inviare al veicolo."""
    return {
        'enable': CONTROL_ENABLE,
        'bias_deg': CONTROL_BIAS_ANGLE_DEG,
        'freq_hz': CONTROL_FREQUENCY_HZ,
        'use_range_only': CONTROL_USE_RANGE_ONLY
    }


def calculate_distance_3d(east, north, up):
    """Calcola distanza 3D da coordinate ENU."""
    return math.sqrt(east**2 + north**2 + up**2)


def angles_to_enu(bearing_rad, elevation_rad, range_m):
    """Converte angoli + distanza in coordinate ENU."""
    horizontal_range = range_m * math.cos(elevation_rad)
    east = horizontal_range * math.sin(bearing_rad)
    north = horizontal_range * math.cos(bearing_rad)
    up = range_m * math.sin(elevation_rad)
    return east, north, up


# =============================================================================
#  MESSAGE PROTOCOL
# =============================================================================
MSG_TYPE_VEHICLE = 0x01
MSG_TYPE_MC = 0x02

STRUCT_VEHICLE = '<Bfffffffffff'
STRUCT_MC = '<BfBfffffffBbBffBf'


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
                'position': {'x': values[2], 'y': values[3], 'z': values[4]},
                'orientation': {'roll': values[5], 'pitch': values[6], 'yaw': values[7]}
            },
            'sensor_data': {
                'battery': {'charge_percent': values[8], 'voltage_volts': values[9]},
                'depth_sensor': {'temperature': values[10], 'pressure': values[11]}
            }
        }
    except Exception as e:
        print("[DECODE] Error: {}".format(e))
        return None


def encode_mc_message(timestamp, usbl, control):
    """Codifica messaggio per il veicolo."""
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
        float(control['freq_hz']),
        1 if control.get('use_range_only', False) else 0,
        float(usbl.get('range_3d', 0.0))
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
        
        if self.enabled:
            if not os.path.exists(directory):
                os.makedirs(directory)
            
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            self.filepath = os.path.join(directory, "mc_log_{}.jsonl".format(timestamp))
            self.log_file = open(self.filepath, 'w')
            
            self._write({
                "event_type": "SESSION_START",
                "timestamp": datetime.now().isoformat(),
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
    
    # Contatori
    "messages_received": 0,
    "messages_sent": 0,
    
    # Stato per il pattern request-response
    "waiting_for_position": False,      # True quando abbiamo ricevuto RECVIM e aspettiamo USBL
    "position_event": None,             # Event per segnalare che abbiamo la posizione
    "position_data": None,              # Dati posizione calcolati
    
    # Stato per AT?T query
    "waiting_for_propagation_time": False,
    "propagation_time_event": None,
    "propagation_time_value": None,
    "pending_angles_data": None,        # USBLANGLES in attesa di AT?T
    
    # Dati USBL correnti
    "current_usbl": {
        "mode": 0,
        "east": 0.0,
        "north": 0.0,
        "up": 0.0,
        "bearing_rad": 0.0,
        "elevation_rad": 0.0,
        "distance": 0.0,
        "range_3d": 0.0,
        "prop_time_us": 0.0,
        "integrity": DEFAULT_USBL_INTEGRITY,
        "rssi": DEFAULT_USBL_RSSI
    },
    
    # Ultimo messaggio ricevuto dal veicolo (per rispondere)
    "pending_vehicle_data": None
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
                    if ',$' in str_msg:
                        _, payload = str_msg.rsplit(',$', 1)
                    
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
                        "uncertainty": float(args[16]) if len(args) > 16 else 0.0,
                        "valid": int(args[15]) > MIN_INTEGRITY
                    }
                except (ValueError, IndexError):
                    return "USBLLONG_ERROR", {}
            
            # USBLANGLES - solo direzione (FALLBACK)
            elif keyword == "USBLANGLES" and len(args) >= 13:
                try:
                    return "USBLANGLES", {
                        "target_id": args[3],
                        "lbearing_rad": float(args[4]),
                        "lelevation_rad": float(args[5]),
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
        
        # Risposta AT?T: formato +++AT?T:<length>:<propagation_time_us>
        if "AT?T" in str_msg and len(parts) >= 3:
            try:
                propagation_time_us = int(parts[2])
                return "PROPAGATION_TIME", {"time_us": propagation_time_us}
            except (ValueError, IndexError):
                return "PROPAGATION_TIME_ERROR", {}
        
        return "OTHER", {}
    except Exception:
        return "ERROR", {}


def get_relative_timestamp():
    if state["session_start"] is None:
        state["session_start"] = time.time()
    return time.time() - state["session_start"]


async def send_at_command(command):
    """Invia comando AT al modem."""
    if state["writer"]:
        cmd_bytes = "+++{}\n".format(command).encode(CODEC)
        state["writer"].write(cmd_bytes)
        await state["writer"].drain()


async def query_propagation_time():
    """
    Query AT?T per ottenere propagation time.
    Ritorna il tempo in microsecondi (one-way).
    """
    state["waiting_for_propagation_time"] = True
    state["propagation_time_value"] = None
    state["propagation_time_event"].clear()
    
    await send_at_command("AT?T")
    
    try:
        await asyncio.wait_for(state["propagation_time_event"].wait(), timeout=PROP_TIME_TIMEOUT)
        return state["propagation_time_value"]
    except asyncio.TimeoutError:
        print("[MC] AT?T timeout")
        state["waiting_for_propagation_time"] = False
        return None


async def send_response_to_vehicle():
    """Invia risposta al veicolo con dati USBL + control."""
    
    usbl = state["current_usbl"]
    control = get_control_data()
    ts = get_relative_timestamp()
    
    payload = encode_mc_message(ts, usbl, control)
    
    state["messages_sent"] += 1
    
    # Log formattato
    print("\n" + "=" * 70)
    print("[MSG #{}] TRANSCEIVER -> TRANSPONDER".format(state["messages_sent"]))
    print("=" * 70)
    print("  USBL Data (mode={}):".format("LONG" if usbl['mode'] == 1 else "ANGLES"))
    if usbl['mode'] == 1:
        print("    ENU:      E={:.3f}  N={:.3f}  U={:.3f} m".format(
            usbl['east'], usbl['north'], usbl['up']))
    else:
        print("    Angles:   bearing={:.2f}deg  elev={:.2f}deg".format(
            math.degrees(usbl['bearing_rad']),
            math.degrees(usbl['elevation_rad'])))
    print("    Range 3D: {:.3f} m (prop time: {:.0f} us)".format(
        usbl['range_3d'], usbl['prop_time_us']))
    print("    Quality:  integrity={}  RSSI={} dBm".format(usbl['integrity'], usbl['rssi']))
    print("  Control Data:")
    print("    enable={}  bias={:.2f}deg  freq={:.2f}Hz  use_range_only={}".format(
        control['enable'], control['bias_deg'], control['freq_hz'],
        control.get('use_range_only', False)))
    print("-" * 70)
    
    # Invia messaggio
    payload_with_marker = "$" + payload
    command = "AT*SENDIM,p0,{},{},ack,{}".format(len(payload_with_marker), TRANSPONDER_ID, payload_with_marker)
    encoded = "+++{}\n".format(command).encode(CODEC)
    
    state["writer"].write(encoded)
    await state["writer"].drain()
    
    if state["logger"]:
        state["logger"].log_event("SENT_TO_VEHICLE", {
            "msg_num": state["messages_sent"],
            "usbl": usbl,
            "control": control
        })


def on_vehicle_telemetry_received(data, msg_num):
    """Callback per telemetria ricevuta."""
    pos = data['pose']['position']
    ori = data['pose']['orientation']
    bat = data['sensor_data']['battery']
    dep = data['sensor_data']['depth_sensor']
    
    print("\n" + "=" * 70)
    print("[MSG #{}] TRANSPONDER -> TRANSCEIVER".format(msg_num))
    print("=" * 70)
    print("  Timestamp: {:.2f}s".format(data['timestamp']))
    print("  Telemetry:")
    print("    Position: x={:.3f}  y={:.3f}  z={:.3f} m".format(
        pos['x'], pos['y'], pos['z']))
    print("    Orient:   roll={:.2f}  pitch={:.2f}  yaw={:.2f} deg".format(
        ori['roll'], ori['pitch'], ori['yaw']))
    print("    Battery:  {:.1f}% @ {:.2f}V".format(
        bat['charge_percent'], bat['voltage_volts']))
    print("    Sensors:  temp={:.1f}C  press={:.1f}mbar".format(
        dep['temperature'], dep['pressure']))
    print("  Waiting for USBL position data...")
    print("-" * 70)


async def handle_usbllong(data):
    """
    Gestisce USBLLONG - posizione completa.
    Se stavamo aspettando una posizione, segnala l'evento.
    """
    if not data["valid"]:
        return
    
    # Calcola range 3D dal propagation time (one-way in microsecondi)
    prop_time_us = data["propagation_time_us"]
    range_3d = (prop_time_us / 1000000.0) * SPEED_OF_SOUND_MPS
    
    # Calcola anche da ENU per confronto
    distance_from_enu = calculate_distance_3d(data["east"], data["north"], data["up"])
    
    # Check se posizione valida (non 0,0,0)
    if abs(data["east"]) < 0.01 and abs(data["north"]) < 0.01 and abs(data["up"]) < 0.01:
        print("[MC] USBLLONG: invalid position (0,0,0) - ignoring")
        return
    
    # Aggiorna stato USBL
    state["current_usbl"]["mode"] = 1  # LONG
    state["current_usbl"]["east"] = data["east"]
    state["current_usbl"]["north"] = data["north"]
    state["current_usbl"]["up"] = data["up"]
    state["current_usbl"]["distance"] = distance_from_enu
    state["current_usbl"]["range_3d"] = range_3d
    state["current_usbl"]["prop_time_us"] = prop_time_us
    state["current_usbl"]["integrity"] = data["integrity"]
    state["current_usbl"]["rssi"] = data["rssi"]
    
    print("[MC] USBLLONG: E={:.2f} N={:.2f} U={:.2f} Range3D={:.2f}m".format(
        data["east"], data["north"], data["up"], range_3d))
    
    # Se stavamo aspettando la posizione, segnala!
    if state["waiting_for_position"]:
        state["position_event"].set()
    
    if state["logger"]:
        state["logger"].log_event("USBLLONG", {
            "data": data,
            "range_3d": range_3d
        })


async def handle_usblangles(data):
    """
    Gestisce USBLANGLES - solo angoli.
    Richiede AT?T per ottenere la distanza.
    """
    if not data["valid"]:
        return
    
    print("[MC] USBLANGLES: bearing={:.2f}deg elev={:.2f}deg".format(
        math.degrees(data["bearing_rad"]),
        math.degrees(data["elevation_rad"])))
    
    # Se non stavamo aspettando una posizione, ignora
    if not state["waiting_for_position"]:
        print("[MC] USBLANGLES received but not waiting for position - ignoring")
        return
    
    # Salva gli angoli e richiedi AT?T
    state["pending_angles_data"] = data
    
    print("[MC] Querying AT?T for distance...")
    prop_time_us = await query_propagation_time()
    
    if prop_time_us is not None and prop_time_us > 0:
        # Calcola range 3D dal propagation time
        range_3d = (prop_time_us / 1000000.0) * SPEED_OF_SOUND_MPS
        
        print("[MC] AT?T: prop_time={} us -> range_3d={:.2f}m".format(prop_time_us, range_3d))
        
        # Calcola ENU da angoli + range
        e, n, u = angles_to_enu(data["bearing_rad"], data["elevation_rad"], range_3d)
        
        # Aggiorna stato USBL
        state["current_usbl"]["mode"] = 0  # ANGLES
        state["current_usbl"]["east"] = e
        state["current_usbl"]["north"] = n
        state["current_usbl"]["up"] = u
        state["current_usbl"]["bearing_rad"] = data["bearing_rad"]
        state["current_usbl"]["elevation_rad"] = data["elevation_rad"]
        state["current_usbl"]["distance"] = range_3d
        state["current_usbl"]["range_3d"] = range_3d
        state["current_usbl"]["prop_time_us"] = prop_time_us
        state["current_usbl"]["integrity"] = data["integrity"]
        state["current_usbl"]["rssi"] = data["rssi"]
        
        print("[MC] Calculated ENU: E={:.2f} N={:.2f} U={:.2f}".format(e, n, u))
        
        # Segnala che abbiamo la posizione
        state["position_event"].set()
        
        if state["logger"]:
            state["logger"].log_event("USBLANGLES_WITH_DISTANCE", {
                "angles": data,
                "prop_time_us": prop_time_us,
                "range_3d": range_3d,
                "enu": {"e": e, "n": n, "u": u}
            })
    else:
        print("[MC] AT?T failed - cannot calculate position")
        # Comunque segnala (con dati zero) per non bloccare il sistema
        state["current_usbl"]["mode"] = 0
        state["current_usbl"]["bearing_rad"] = data["bearing_rad"]
        state["current_usbl"]["elevation_rad"] = data["elevation_rad"]
        state["current_usbl"]["integrity"] = data["integrity"]
        state["current_usbl"]["rssi"] = data["rssi"]
        state["position_event"].set()


async def response_listener(reader):
    """Task principale che ascolta messaggi dal modem."""
    print("[MC] Listening for messages...")
    
    try:
        while True:
            line = await reader.readline()
            if not line:
                print("[MC] Connection closed")
                break
            
            msg_type, data = decode_modem_response(line)
            
            # Risposte AT?T (PROPAGATION_TIME)
            if msg_type == "PROPAGATION_TIME":
                if state["waiting_for_propagation_time"]:
                    state["propagation_time_value"] = data["time_us"]
                    state["waiting_for_propagation_time"] = False
                    state["propagation_time_event"].set()
            
            # DELIVEREDIM
            elif msg_type == "DELIVEREDIM":
                print("[MC] Message delivered to {}".format(data.get("target_id", "?")))
            
            # FAILEDIM
            elif msg_type == "FAILEDIM":
                print("[MC] Message delivery FAILED to {}".format(data.get("target_id", "?")))
            
            # *** USBLLONG - PRIORITARIO ***
            elif msg_type == "USBLLONG":
                await handle_usbllong(data)
            
            # *** USBLANGLES ***
            elif msg_type == "USBLANGLES":
                await handle_usblangles(data)
            
            # *** MESSAGGIO DAL VEICOLO ***
            elif msg_type == "RECVIM":
                if not data.get("valid"):
                    print("[MC] Low integrity message ({}), ignoring".format(
                        data.get('integrity', '?')))
                    continue
                
                state["messages_received"] += 1
                
                payload = data.get("payload")
                if payload:
                    vehicle_data = decode_vehicle_message(payload.strip())
                    
                    if vehicle_data:
                        if state["logger"]:
                            state["logger"].log_event("VEHICLE_TELEMETRY", {
                                "msg_num": state["messages_received"],
                                "data": vehicle_data
                            })
                        
                        on_vehicle_telemetry_received(vehicle_data, state["messages_received"])
                        
                        # *** PATTERN CORRETTO: NON RISPONDERE SUBITO ***
                        # Segnala che stiamo aspettando dati USBL
                        state["waiting_for_position"] = True
                        state["position_event"].clear()
                        state["pending_vehicle_data"] = vehicle_data
                        
                        # Aspetta USBLLONG o USBLANGLES
                        try:
                            await asyncio.wait_for(
                                state["position_event"].wait(), 
                                timeout=POSITION_CAPTURE_TIMEOUT
                            )
                            print("[MC] Position data received!")
                        except asyncio.TimeoutError:
                            print("[MC] Timeout waiting for USBL position data")
                        
                        # Ora rispondi al veicolo (con o senza dati USBL validi)
                        await send_response_to_vehicle()
                        
                        # Reset stato
                        state["waiting_for_position"] = False
                        state["pending_vehicle_data"] = None
                        state["pending_angles_data"] = None
                    else:
                        print("[MC] Failed to decode vehicle message")
                        
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
    state["position_event"] = asyncio.Event()
    state["propagation_time_event"] = asyncio.Event()
    state["session_start"] = time.time()
    
    print("=" * 70)
    print("MISSION CONTROL - REACTIVE MODE (CORRECT TIMING)")
    print("=" * 70)
    print("Transceiver ID: {}".format(TRANSCEIVER_ID))
    print("Transponder ID: {} (vehicle)".format(TRANSPONDER_ID))
    print("USBL IP:        {}:{}".format(USBL_IP, USBL_PORT))
    print("Sound Speed:    {} m/s".format(SPEED_OF_SOUND_MPS))
    print("")
    print("COMMUNICATION PATTERN:")
    print("  1. Receive RECVIM from transponder")
    print("  2. Wait for USBLLONG or USBLANGLES ({:.1f}s timeout)".format(POSITION_CAPTURE_TIMEOUT))
    print("  3. If USBLANGLES, query AT?T for distance")
    print("  4. Send response with USBL + control data")
    print("")
    print("CONTROL DEFAULTS:")
    print("  enable={}  bias={}deg  freq={}Hz  range_only={}".format(
        CONTROL_ENABLE, CONTROL_BIAS_ANGLE_DEG, 
        CONTROL_FREQUENCY_HZ, CONTROL_USE_RANGE_ONLY))
    print("=" * 70)
    
    try:
        reader, writer = await asyncio.open_connection(USBL_IP, USBL_PORT)
        state["writer"] = writer
        print("[MC] Connected to USBL modem")
        print("[MC] Waiting for vehicle messages...")
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