#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
USBL Mission Control (Transceiver)
Riceve telemetria completa dal veicolo, calcola posizione USBL, invia dati al veicolo

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

USBL_POLL_INTERVAL = 2.0
RESPONSE_TIMEOUT = 5.0

ENABLE_LOGGING = True
LOG_DIRECTORY = "mission_control_logs"

# =============================================================================
#  MESSAGE PROTOCOL - BINARY COMPRESSED
# =============================================================================
MSG_TYPE_VEHICLE = 0x01
MSG_TYPE_MC = 0x02

STRUCT_VEHICLE = '<Bfffffffffff'  # 45 bytes
STRUCT_MC = '<BfBfffffBbBff'       # 37 bytes


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
    except Exception:
        return None


def encode_mc_message(timestamp, mode, east, north, up, distance,
                      prop_time_us, integrity, rssi, enable, bias_deg, freq_hz):
    """Codifica messaggio per il veicolo."""
    data = struct.pack(
        STRUCT_MC,
        MSG_TYPE_MC,
        float(timestamp),
        int(mode) & 0xFF,
        float(east), float(north), float(up),
        float(distance),
        float(prop_time_us),
        int(integrity) & 0xFF,
        max(-128, min(127, int(rssi))),
        1 if enable else 0,
        float(bias_deg),
        float(freq_hz)
    )
    return base64.b64encode(data).decode('ascii')


# =============================================================================
#  LOGGER
# =============================================================================
class MissionControlLogger(object):
    def __init__(self, directory, enabled=True):
        self.enabled = enabled
        self.directory = directory
        self.log_file = None
        self.filepath = None
        self.session_start = None
        
        if self.enabled:
            self._init_log_file()
    
    def _init_log_file(self):
        if not os.path.exists(self.directory):
            os.makedirs(self.directory)
        
        self.session_start = datetime.now()
        timestamp = self.session_start.strftime("%Y%m%d_%H%M%S")
        self.filepath = os.path.join(self.directory, "mc_log_{}.jsonl".format(timestamp))
        
        self.log_file = open(self.filepath, 'w')
        
        session_meta = {
            "event_type": "SESSION_START",
            "timestamp": self.session_start.isoformat(),
            "config": {
                "transceiver_id": TRANSCEIVER_ID,
                "transponder_id": TRANSPONDER_ID,
                "usbl_ip": USBL_IP,
                "sound_speed_mps": SPEED_OF_SOUND_MPS
            }
        }
        self._write(session_meta)
        print("[LOG] File: {}".format(self.filepath))
    
    def _write(self, event):
        if self.log_file and not self.log_file.closed:
            self.log_file.write(json.dumps(event, separators=(',', ':')) + '\n')
            self.log_file.flush()
    
    def log_event(self, event_type, data):
        if not self.enabled:
            return
        event = {
            "event_type": event_type,
            "timestamp": datetime.now().isoformat(),
            "data": data
        }
        self._write(event)
    
    def log_vehicle_telemetry(self, data):
        self.log_event("VEHICLE_TELEMETRY", data)
    
    def log_usbllong(self, data):
        self.log_event("USBLLONG", data)
    
    def log_usblangles(self, data):
        self.log_event("USBLANGLES", data)
    
    def log_sent_to_vehicle(self, data):
        self.log_event("SENT_TO_VEHICLE", data)
    
    def close(self):
        if self.log_file and not self.log_file.closed:
            duration = (datetime.now() - self.session_start).total_seconds() if self.session_start else 0
            self.log_event("SESSION_END", {"duration_s": duration})
            self.log_file.close()
            print("[LOG] Closed: {}".format(self.filepath))


# =============================================================================
#  STATE
# =============================================================================
state = {
    # Vehicle telemetry received
    "vehicle": {
        "timestamp": 0.0,
        "pose": {
            "position": {"x": 0.0, "y": 0.0, "z": 0.0},
            "orientation": {"roll": 0.0, "pitch": 0.0, "yaw": 0.0}
        },
        "sensor_data": {
            "battery": {"charge_percent": 0.0, "voltage_volts": 0.0},
            "depth_sensor": {"temperature": 0.0, "pressure": 0.0}
        },
        "valid": False,
        "received_at": 0.0
    },
    
    # USBL calculated position
    "usbl": {
        "mode": 0,
        "east": 0.0,
        "north": 0.0,
        "up": 0.0,
        "distance": 0.0,
        "propagation_time_us": 0.0,
        "integrity": 0,
        "rssi": 0,
        "valid": False,
        "timestamp": 0.0
    },
    
    # Control to send
    "control": {
        "enable": True,
        "bias_deg": 0.0,
        "freq_hz": 0.5
    },
    
    # Communication
    "waiting_delivery": False,
    "delivery_event": None,
    "delivery_result": None,
    "waiting_prop_time": False,
    "prop_time_event": None,
    "prop_time_value": None,
    
    # Stats
    "messages_sent": 0,
    "messages_received": 0,
    "usbllong_count": 0,
    "usblangles_count": 0,
    
    # Session
    "session_start": None,
    "logger": None
}


# =============================================================================
#  PROTOCOL FUNCTIONS
# =============================================================================
def decode_response(message):
    """Decodifica risposta dal modem."""
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
                    full_msg = parts[2]
                    comma_parts = full_msg.split(",")
                    if len(comma_parts) >= 10:
                        payload = ",".join(comma_parts[10:])
                    
                    return "RECVIM", {
                        "sender_id": sender_id,
                        "integrity": integrity,
                        "valid": integrity > MIN_INTEGRITY,
                        "rssi": rssi,
                        "payload": payload
                    }
                except (ValueError, IndexError):
                    return "RECVIM_ERROR", {}
            
            elif keyword == "USBLLONG" and len(args) >= 17:
                try:
                    target_id = args[3]
                    east = float(args[7])
                    north = float(args[8])
                    up = float(args[9])
                    propagation_time_us = int(args[13])
                    rssi = int(args[14])
                    integrity = int(args[15])
                    
                    slant_range = (propagation_time_us / 1000000.0) * SPEED_OF_SOUND_MPS
                    
                    return "USBLLONG", {
                        "target_id": target_id,
                        "east": east,
                        "north": north,
                        "up": up,
                        "range": slant_range,
                        "rssi": rssi,
                        "integrity": integrity,
                        "propagation_time_us": propagation_time_us,
                        "valid": integrity > MIN_INTEGRITY
                    }
                except (ValueError, IndexError):
                    return "USBLLONG_ERROR", {}
            
            elif keyword == "USBLANGLES" and len(args) >= 13:
                try:
                    target_id = args[3]
                    bearing = float(args[6])
                    elevation = float(args[7])
                    rssi = int(args[11])
                    integrity = int(args[12])
                    
                    return "USBLANGLES", {
                        "target_id": target_id,
                        "bearing": bearing,
                        "elevation": elevation,
                        "bearing_deg": math.degrees(bearing),
                        "elevation_deg": math.degrees(elevation),
                        "rssi": rssi,
                        "integrity": integrity,
                        "valid": integrity > MIN_INTEGRITY
                    }
                except (ValueError, IndexError):
                    return "USBLANGLES_ERROR", {}
            
            elif keyword in ["SENDSTART", "SENDEND", "RECVSTART", "RECVEND"]:
                return keyword, {"args": args}
        
        return "OTHER", {}
    except Exception:
        return "ERROR", {}


def get_relative_timestamp():
    """Timestamp relativo dall'inizio sessione."""
    if state["session_start"] is None:
        state["session_start"] = time.time()
    return time.time() - state["session_start"]


# =============================================================================
#  ASYNC TASKS
# =============================================================================
async def send_command(writer, command):
    cmd_bytes = "+++{}\n".format(command).encode(CODEC)
    writer.write(cmd_bytes)
    await writer.drain()


async def query_propagation_time(writer):
    """Query AT?T per propagation time."""
    state["waiting_prop_time"] = True
    state["prop_time_value"] = None
    state["prop_time_event"].clear()
    
    await send_command(writer, "AT?T")
    
    try:
        await asyncio.wait_for(state["prop_time_event"].wait(), timeout=RESPONSE_TIMEOUT)
        return state["prop_time_value"]
    except asyncio.TimeoutError:
        state["waiting_prop_time"] = False
        return None


async def send_to_vehicle(writer, timeout=5.0):
    """Invia dati USBL + control al veicolo (messaggio completo)."""
    usbl = state["usbl"]
    ctrl = state["control"]
    
    if not usbl["valid"]:
        return False
    
    ts = get_relative_timestamp()
    
    payload = encode_mc_message(
        ts,
        usbl["mode"],
        usbl["east"],
        usbl["north"],
        usbl["up"],
        usbl["distance"],
        usbl["propagation_time_us"],
        usbl["integrity"],
        usbl["rssi"],
        ctrl["enable"],
        ctrl["bias_deg"],
        ctrl["freq_hz"]
    )
    
    print("[MC] Payload size: {} bytes".format(len(payload)))
    
    if len(payload) > 63:
        print("[MC] ERROR: Payload too long!")
        return False
    
    command = "AT*SENDIM,p0,{},{},ack,{}".format(len(payload), TRANSPONDER_ID, payload)
    encoded = "+++{}\n".format(command).encode(CODEC)
    
    state["waiting_delivery"] = True
    state["delivery_result"] = None
    state["delivery_event"].clear()
    
    writer.write(encoded)
    await writer.drain()
    state["messages_sent"] += 1
    
    if state["logger"]:
        state["logger"].log_sent_to_vehicle({
            "timestamp": ts,
            "usbl": {
                "mode": "LONG" if usbl["mode"] == 1 else "ANGLES",
                "east": usbl["east"],
                "north": usbl["north"],
                "up": usbl["up"],
                "distance": usbl["distance"]
            },
            "control": ctrl
        })
    
    try:
        await asyncio.wait_for(state["delivery_event"].wait(), timeout=timeout)
        return state["delivery_result"] == True
    except asyncio.TimeoutError:
        state["waiting_delivery"] = False
        return False


async def response_listener(reader):
    """Task che ascolta risposte dal modem."""
    try:
        while True:
            line = await reader.readline()
            if not line:
                print("[MC] Connection closed")
                break
            
            print("[RAW]", line)
            msg_type, data = decode_response(line)
            
            # Propagation time
            if msg_type == "DIRECT_VALUE" and state["waiting_prop_time"]:
                try:
                    state["prop_time_value"] = float(data["value"])
                except (ValueError, KeyError):
                    pass
                state["waiting_prop_time"] = False
                state["prop_time_event"].set()
            
            # Delivery
            elif msg_type == "DELIVEREDIM":
                if state["waiting_delivery"]:
                    state["delivery_result"] = True
                    state["waiting_delivery"] = False
                    state["delivery_event"].set()
                    print("[MC] Message delivered")
            
            elif msg_type == "FAILEDIM":
                if state["waiting_delivery"]:
                    state["delivery_result"] = False
                    state["waiting_delivery"] = False
                    state["delivery_event"].set()
                    print("[MC] Message FAILED")
            
            # Received from vehicle
            elif msg_type == "RECVIM":
                state["messages_received"] += 1
                if data.get("valid"):
                    payload = data.get("payload")
                    if payload:
                        decoded = decode_vehicle_message(payload.strip())
                        if decoded:
                            #print("\n[VEHICLE MESSAGE DECODED]")
                            #print(json.dumps(decoded, indent=2))
                            state["vehicle"]["timestamp"] = decoded["timestamp"]
                            state["vehicle"]["pose"] = decoded["pose"]
                            state["vehicle"]["sensor_data"] = decoded["sensor_data"]
                            state["vehicle"]["valid"] = True
                            state["vehicle"]["received_at"] = time.time()
                            
                            if state["logger"]:
                                state["logger"].log_vehicle_telemetry(decoded)
                            
                            pos = decoded["pose"]["position"]
                            ori = decoded["pose"]["orientation"]
                            bat = decoded["sensor_data"]["battery"]
                            dep = decoded["sensor_data"]["depth_sensor"]
                            
                            print("\n[VEHICLE] Full Telemetry Received:")
                            print("  Position: [{:.2f}, {:.2f}, {:.2f}] m".format(
                                pos["x"], pos["y"], pos["z"]))
                            print("  Orientation: [{:.1f}, {:.1f}, {:.1f}] deg".format(
                                ori["roll"], ori["pitch"], ori["yaw"]))
                            print("  Battery: {:.1f}% @ {:.1f}V".format(
                                bat["charge_percent"], bat["voltage_volts"]))
                            print("  Depth sensor: {:.1f}C, {:.1f}mbar".format(
                                dep["temperature"], dep["pressure"]))
            
            # USBLLONG
            elif msg_type == "USBLLONG":
                if data["valid"]:
                    state["usbllong_count"] += 1
                    state["usbl"]["mode"] = 1
                    state["usbl"]["east"] = data["east"]
                    state["usbl"]["north"] = data["north"]
                    state["usbl"]["up"] = data["up"]
                    state["usbl"]["distance"] = data["range"]
                    state["usbl"]["propagation_time_us"] = data["propagation_time_us"]
                    state["usbl"]["integrity"] = data["integrity"]
                    state["usbl"]["rssi"] = data["rssi"]
                    state["usbl"]["valid"] = True
                    state["usbl"]["timestamp"] = time.time()
                    
                    if state["logger"]:
                        state["logger"].log_usbllong(data)
                    
                    dist_3d = math.sqrt(data["east"]**2 + data["north"]**2 + data["up"]**2)
                    print("\n[USBL] LONG: E={:.2f} N={:.2f} U={:.2f} D={:.2f}m int={}".format(
                        data["east"], data["north"], data["up"], dist_3d, data["integrity"]))
            
            # USBLANGLES
            elif msg_type == "USBLANGLES":
                if data["valid"]:
                    state["usblangles_count"] += 1
                    
                    # Only use if no recent USBLLONG
                    if not state["usbl"]["valid"] or \
                       (time.time() - state["usbl"]["timestamp"]) > 5.0:
                        state["usbl"]["mode"] = 0
                        state["usbl"]["integrity"] = data["integrity"]
                        state["usbl"]["rssi"] = data["rssi"]
                    
                    if state["logger"]:
                        state["logger"].log_usblangles(data)
                    
                    print("\n[USBL] ANGLES: bearing={:.1f}deg elev={:.1f}deg".format(
                        data["bearing_deg"], data["elevation_deg"]))
                        
    except asyncio.CancelledError:
        pass
    except Exception as e:
        print("[MC] Listener error: {}".format(e))


async def polling_task(writer):
    """Task per polling periodico e invio dati al veicolo."""
    await asyncio.sleep(2.0)
    
    poll_count = 0
    
    while True:
        try:
            poll_count += 1
            print("\n" + "=" * 60)
            print("[MC] Poll #{} - {}".format(poll_count, 
                datetime.now().strftime("%H:%M:%S")))
            print("=" * 60)
            
            # Se abbiamo dati USBL validi, inviali al veicolo
            if state["usbl"]["valid"]:
                print("[MC] Sending USBL+Control to vehicle...")
                success = await send_to_vehicle(writer)
                if success:
                    print("[MC] Data sent successfully")
                else:
                    print("[MC] Failed to send data")
            else:
                print("[MC] No valid USBL data to send")
            
            # Query propagation time
            prop_time = await query_propagation_time(writer)
            if prop_time is not None:
                distance = (prop_time / 1000.0 / 2.0) * SPEED_OF_SOUND_MPS
                print("[MC] Prop time: {:.2f}ms -> Distance: {:.2f}m".format(
                    prop_time, distance))
            
            # Vehicle status
            if state["vehicle"]["valid"]:
                pos = state["vehicle"]["pose"]["position"]
                print("[MC] Vehicle position: [{:.2f}, {:.2f}, {:.2f}]m".format(
                    pos["x"], pos["y"], pos["z"]))
            
            print("[MC] Stats: sent={} recv={} usbllong={} usblangles={}".format(
                state["messages_sent"], state["messages_received"],
                state["usbllong_count"], state["usblangles_count"]))
            
            await asyncio.sleep(USBL_POLL_INTERVAL)
            
        except Exception as e:
            print("[MC] Polling error: {}".format(e))
            await asyncio.sleep(USBL_POLL_INTERVAL)


async def main():
    logging.basicConfig(level=logging.INFO, format="%(message)s")
    
    state["logger"] = MissionControlLogger(LOG_DIRECTORY, ENABLE_LOGGING)
    state["delivery_event"] = asyncio.Event()
    state["prop_time_event"] = asyncio.Event()
    state["session_start"] = time.time()
    
    print("=" * 70)
    print("MISSION CONTROL - USBL Transceiver")
    print("=" * 70)
    print("Transceiver ID: {}".format(TRANSCEIVER_ID))
    print("Transponder ID: {} (vehicle)".format(TRANSPONDER_ID))
    print("USBL IP:        {}:{}".format(USBL_IP, USBL_PORT))
    print("Sound Speed:    {} m/s".format(SPEED_OF_SOUND_MPS))
    print("Poll Interval:  {} s".format(USBL_POLL_INTERVAL))
    print("Logging:        {}".format("ENABLED" if ENABLE_LOGGING else "DISABLED"))
    print("")
    print("Message Protocol: struct binary + base64")
    print("  Vehicle->MC: 45 bytes -> 60 chars base64")
    print("  MC->Vehicle: 37 bytes -> 52 chars base64")
    print("=" * 70)
    
    try:
        reader, writer = await asyncio.open_connection(USBL_IP, USBL_PORT)
        print("[MC] Connected to USBL modem")
    except Exception as e:
        print("[MC] Connection failed: {}".format(e))
        return
    
    listener_task = asyncio.ensure_future(response_listener(reader))
    poll_task = asyncio.ensure_future(polling_task(writer))
    
    try:
        while True:
            await asyncio.sleep(0.1)
    except KeyboardInterrupt:
        print("\n[MC] Stopping...")
    finally:
        listener_task.cancel()
        poll_task.cancel()
        
        try:
            await asyncio.gather(listener_task, poll_task)
        except asyncio.CancelledError:
            pass
        
        writer.close()
        
        if state["logger"]:
            state["logger"].close()
        
        print("\n" + "=" * 70)
        print("MISSION CONTROL - SUMMARY")
        print("=" * 70)
        print("Messages sent:     {}".format(state["messages_sent"]))
        print("Messages received: {}".format(state["messages_received"]))
        print("USBLLONG count:    {}".format(state["usbllong_count"]))
        print("USBLANGLES count:  {}".format(state["usblangles_count"]))
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