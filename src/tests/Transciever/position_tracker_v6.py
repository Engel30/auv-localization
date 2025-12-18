#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
USBL Transceiver (Mission Control)
Riceve telemetria dal transponder e risponde con distanza 3D calcolata via AT?T
"""

import asyncio
import struct
import base64
import time
import math
import os
import csv
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

# Timeout per query AT?T [s]
AT_T_TIMEOUT = 3.0

# Default values
DEFAULT_INTEGRITY = 100
DEFAULT_RSSI = -40

# Control data (modifica questi valori)
CONTROL_ENABLE = True
CONTROL_BIAS_DEG = 0.0
CONTROL_FREQ_HZ = 0.5
CONTROL_USE_RANGE_ONLY = False

# LOGGING CONFIGURATION
ENABLE_LOGGING = True
LOG_DIRECTORY = "Transciever/log_usbl_messages"

# =============================================================================
#  MESSAGE PROTOCOL
# =============================================================================
MSG_TYPE_VEHICLE = 0x01
MSG_TYPE_MC = 0x02

STRUCT_VEHICLE = '<Bfffffffffff'       # 45 bytes
STRUCT_MC = '<BfBffffffBBffBf'       # 45 bytes (no prop_time, no rssi)       # 50 bytes


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


def encode_mc_message(timestamp, mode, east, north, up, bearing_rad, elevation_rad,
                      distance, integrity, enable, bias_deg, freq_hz,
                      use_range_only, range_3d):
    """Codifica messaggio per il veicolo."""
    data = struct.pack(
        STRUCT_MC,
        MSG_TYPE_MC,
        float(timestamp),
        int(mode) & 0xFF,
        float(east), float(north), float(up),
        float(bearing_rad), float(elevation_rad),
        float(distance),
        int(integrity) & 0xFF,
        1 if enable else 0,
        float(bias_deg),
        float(freq_hz),
        1 if use_range_only else 0,
        float(range_3d)
    )
    return base64.b64encode(data).decode('ascii')


# =============================================================================
#  CSV LOGGER CLASS
# =============================================================================
class TransceiverLogger(object):
    """Logger per messaggi TX e RX del transceiver."""
    
    def __init__(self, log_dir=LOG_DIRECTORY, enabled=ENABLE_LOGGING):
        self.enabled = enabled
        if not self.enabled:
            print("[LOG] Logging disabilitato")
            return
        
        # Crea directory se non esiste
        if not os.path.exists(log_dir):
            os.makedirs(log_dir)
        
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # File separati per TX e RX
        self.rx_file = os.path.join(log_dir, "tc_rx_tp_{}.csv".format(timestamp))
        self.tx_file = os.path.join(log_dir, "tc_tx_tp_{}.csv".format(timestamp))
        
        self.t0 = None
        self.rx_count = 0
        self.tx_count = 0
        
        # Apri file RX (Transponder -> Transceiver)
        self.rx_fp = open(self.rx_file, 'w')
        self.rx_writer = csv.writer(self.rx_fp)
        self.rx_writer.writerow([
            'timestamp_abs', 'timestamp_rel', 'msg_num',
            'vehicle_timestamp', 'x', 'y', 'z',
            'roll', 'pitch', 'yaw',
            'battery_pct', 'voltage',
            'temperature', 'pressure',
            'integrity', 'rssi'
        ])
        self.rx_fp.flush()
        
        # Apri file TX (Transceiver -> Transponder)
        self.tx_fp = open(self.tx_file, 'w')
        self.tx_writer = csv.writer(self.tx_fp)
        self.tx_writer.writerow([
            'timestamp_abs', 'timestamp_rel', 'msg_num',
            'mode', 'east', 'north', 'up',
            'bearing_deg', 'elevation_deg',
            'distance', 'range_3d', 'integrity',
            'enable', 'bias_deg', 'freq_hz', 'use_range_only'
        ])
        self.tx_fp.flush()
        
        print("[LOG] Logging abilitato")
        print("[LOG] RX: {}".format(self.rx_file))
        print("[LOG] TX: {}".format(self.tx_file))
    
    def _get_timestamps(self):
        now = time.time()
        if self.t0 is None:
            self.t0 = now
        return now, now - self.t0
    
    def log_rx(self, vehicle_data, integrity=0, rssi=0):
        """Log messaggio ricevuto dal transponder."""
        if not self.enabled:
            return
        
        t_abs, t_rel = self._get_timestamps()
        self.rx_count += 1
        
        pos = vehicle_data['pose']['position']
        ori = vehicle_data['pose']['orientation']
        bat = vehicle_data['sensor_data']['battery']
        dep = vehicle_data['sensor_data']['depth_sensor']
        
        self.rx_writer.writerow([
            t_abs, t_rel, self.rx_count,
            vehicle_data['timestamp'],
            pos['x'], pos['y'], pos['z'],
            ori['roll'], ori['pitch'], ori['yaw'],
            bat['charge_percent'], bat['voltage_volts'],
            dep['temperature'], dep['pressure'],
            integrity, rssi
        ])
        self.rx_fp.flush()
    
    def log_tx(self, mode, east, north, up, bearing_deg, elevation_deg,
               distance, range_3d, integrity, enable, bias_deg, freq_hz,
               use_range_only):
        """Log messaggio inviato al transponder."""
        if not self.enabled:
            return
        
        t_abs, t_rel = self._get_timestamps()
        self.tx_count += 1
        
        self.tx_writer.writerow([
            t_abs, t_rel, self.tx_count,
            mode, east, north, up,
            bearing_deg, elevation_deg,
            distance, range_3d, integrity,
            enable, bias_deg, freq_hz, use_range_only
        ])
        self.tx_fp.flush()
    
    def close(self):
        if not self.enabled:
            return
        
        print("[LOG] Messaggi RX: {}".format(self.rx_count))
        print("[LOG] Messaggi TX: {}".format(self.tx_count))
        
        self.rx_fp.close()
        self.tx_fp.close()


# =============================================================================
#  STATE
# =============================================================================
state = {
    "session_start": None,
    "writer": None,
    "messages_received": 0,
    "messages_sent": 0,
    
    # AT?T query state
    "waiting_at_t": False,
    "at_t_event": None,
    "at_t_value_us": None,
    "pending_vehicle_data": None,  # Vehicle data in attesa di risposta AT?T
    
    # Delivery state
    "waiting_delivery": False,
    "delivery_event": None,
    "delivery_success": False,
    
    # Logger
    "logger": None
}


def get_relative_timestamp():
    if state["session_start"] is None:
        state["session_start"] = time.time()
    return time.time() - state["session_start"]


def calculate_range_3d_from_prop_time(prop_time_us):
    """
    Calcola distanza 3D dal propagation time.
    prop_time_us è in microsecondi (one-way per il nostro sistema).
    """
    prop_time_s = prop_time_us / 1000000.0
    return prop_time_s * SPEED_OF_SOUND_MPS


def decode_modem_response(message):
    """Decodifica risposta dal modem."""
    try:
        str_msg = message.decode(CODEC).strip()
        if not str_msg:
            return "EMPTY", {}
        
        if str_msg == "OK":
            return "OK", {}
        
        if not str_msg.startswith("+++"):
            return "OTHER", {}
        
        parts = str_msg.split(":")
        
        if "AT" in str_msg and len(parts) >= 3:
            # Check for AT?T response first (format: +++AT?T:<length>:<propagation_time_us>)
            if "AT?T" in str_msg:
                try:
                    propagation_time_us = int(parts[2])
                    return "AT_T_RESPONSE", {"time_us": propagation_time_us}
                except (ValueError, IndexError):
                    print("[DECODE] Error parsing AT?T response")
                    return "AT_T_RESPONSE_ERROR", {}
            
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
            
            elif keyword in ["SENDSTART", "SENDEND", "RECVSTART", "RECVEND"]:
                return keyword, {}
        
        return "OTHER", {}
    except Exception:
        return "ERROR", {}


async def send_command(command):
    """Invia comando AT al modem."""
    if state["writer"]:
        cmd_bytes = "+++{}\n".format(command).encode(CODEC)
        state["writer"].write(cmd_bytes)
        await state["writer"].drain()


async def handle_at_t_timeout(vehicle_data):
    """Gestisce timeout per risposta AT?T."""
    await asyncio.sleep(AT_T_TIMEOUT)
    
    # Se dopo il timeout siamo ancora in attesa, invia risposta con 0 range
    if state["waiting_at_t"] and state["pending_vehicle_data"] is vehicle_data:
        print("[TC] AT?T timeout - sending response with 0 range")
        state["waiting_at_t"] = False
        state["pending_vehicle_data"] = None
        await send_response_to_transponder(vehicle_data, 0.0)


async def query_propagation_time():
    """Interroga propagation time con AT?T."""
    state["waiting_at_t"] = True
    state["at_t_value_us"] = None
    state["at_t_event"].clear()
    
    await send_command("AT?T")
    
    try:
        await asyncio.wait_for(state["at_t_event"].wait(), timeout=AT_T_TIMEOUT)
        return state["at_t_value_us"]
    except asyncio.TimeoutError:
        print("[TC] AT?T timeout")
        state["waiting_at_t"] = False
        return None


async def send_response_to_transponder(vehicle_data, range_3d):
    """
    Invia risposta al transponder con dati USBL e controllo.
    """
    timestamp = get_relative_timestamp()
    
    # Per ora usiamo solo range_3d, mode = 0 (ANGLES)
    payload = encode_mc_message(
        timestamp=timestamp,
        mode=0,  # ANGLES
        east=0.0,
        north=0.0,
        up=0.0,
        bearing_rad=0.0,
        elevation_rad=0.0,
        distance=0.0,  # Non usato con range_3d
        integrity=DEFAULT_INTEGRITY,
        enable=CONTROL_ENABLE,
        bias_deg=CONTROL_BIAS_DEG,
        freq_hz=CONTROL_FREQ_HZ,
        use_range_only=CONTROL_USE_RANGE_ONLY,
        range_3d=range_3d  # Distanza 3D calcolata
    )
    
    payload_with_marker = "$" + payload
    
    if len(payload_with_marker) > 64:
        print("[TC] ERROR: Payload too long ({} bytes)".format(len(payload_with_marker)))
        return False
    
    command = "AT*SENDIM,p0,{},{},ack,{}".format(
        len(payload_with_marker), TRANSPONDER_ID, payload_with_marker
    )
    encoded = "+++{}\n".format(command).encode(CODEC)
    
    state["waiting_delivery"] = True
    state["delivery_success"] = False
    state["delivery_event"].clear()
    
    state["writer"].write(encoded)
    await state["writer"].drain()
    
    state["messages_sent"] += 1
    
    # Log messaggio inviato su CSV
    if state["logger"]:
        state["logger"].log_tx(
            mode='ANGLES',
            east=0.0, north=0.0, up=0.0,
            bearing_deg=0.0, elevation_deg=0.0,
            distance=0.0,
            range_3d=range_3d,
            integrity=DEFAULT_INTEGRITY,
            enable=CONTROL_ENABLE,
            bias_deg=CONTROL_BIAS_DEG,
            freq_hz=CONTROL_FREQ_HZ,
            use_range_only=CONTROL_USE_RANGE_ONLY
        )
    
    # Log messaggio inviato su console
    pos = vehicle_data['pose']['position']
    ori = vehicle_data['pose']['orientation']
    
    print("\n" + "=" * 70)
    print("[MSG #{}] TRANSCEIVER -> TRANSPONDER".format(state["messages_sent"]))
    print("=" * 70)
    print("  USBL Data:")
    print("    Range 3D: {:.3f} m".format(range_3d))
    print("  Control Data:")
    print("    enable={}  bias={:.2f}deg  freq={:.2f}Hz  use_range_only={}".format(
        CONTROL_ENABLE, CONTROL_BIAS_DEG, CONTROL_FREQ_HZ, CONTROL_USE_RANGE_ONLY))
    print("  Vehicle position (from telemetry):")
    print("    x={:.3f}  y={:.3f}  z={:.3f} m".format(pos['x'], pos['y'], pos['z']))
    print("    roll={:.2f}  pitch={:.2f}  yaw={:.2f} deg".format(
        ori['roll'], ori['pitch'], ori['yaw']))
    print("-" * 70)
    
    # Aspetta conferma delivery
    try:
        await asyncio.wait_for(state["delivery_event"].wait(), timeout=5.0)
        if state["delivery_success"]:
            print("[TC] Response delivered OK\n")
            return True
        else:
            print("[TC] Response delivery FAILED\n")
            return False
    except asyncio.TimeoutError:
        print("[TC] Response delivery TIMEOUT\n")
        state["waiting_delivery"] = False
        return False


async def response_listener(reader):
    """Task principale che ascolta messaggi dal modem."""
    print("[TC] Listening for transponder messages...")
    print("")
    
    try:
        while True:
            line = await reader.readline()
            if not line:
                print("[TC] Connection closed")
                break
            
            msg_type, data = decode_modem_response(line)
            
            # Risposta AT?T
            if msg_type == "AT_T_RESPONSE" and state["waiting_at_t"]:
                prop_time_us = data["time_us"]
                state["at_t_value_us"] = prop_time_us
                state["waiting_at_t"] = False
                
                print("[TC] AT?T response: {:.0f} us".format(prop_time_us))
                
                # Calcola range e invia risposta al transponder
                # Solo se abbiamo ancora vehicle_data (non è già stato inviato dal timeout)
                if state["pending_vehicle_data"] is not None:
                    vehicle_data = state["pending_vehicle_data"]
                    state["pending_vehicle_data"] = None  # Pulisci prima di inviare
                    
                    range_3d = calculate_range_3d_from_prop_time(prop_time_us)
                    print("[TC] Propagation time: {:.0f} us -> Range 3D: {:.3f} m".format(
                        prop_time_us, range_3d))
                    
                    await send_response_to_transponder(vehicle_data, range_3d)
            
            # Delivery confirmation
            elif msg_type == "DELIVEREDIM":
                if state["waiting_delivery"]:
                    state["delivery_success"] = True
                    state["waiting_delivery"] = False
                    state["delivery_event"].set()
            
            elif msg_type == "FAILEDIM":
                if state["waiting_delivery"]:
                    state["delivery_success"] = False
                    state["waiting_delivery"] = False
                    state["delivery_event"].set()
            
            # Messaggio dal transponder
            elif msg_type == "RECVIM":
                state["messages_received"] += 1
                
                if data.get("valid"):
                    payload = data.get("payload")
                    if payload:
                        vehicle_data = decode_vehicle_message(payload.strip())
                        
                        if vehicle_data:
                            pos = vehicle_data['pose']['position']
                            ori = vehicle_data['pose']['orientation']
                            bat = vehicle_data['sensor_data']['battery']
                            dep = vehicle_data['sensor_data']['depth_sensor']
                            
                            # Log messaggio ricevuto su CSV
                            if state["logger"]:
                                state["logger"].log_rx(
                                    vehicle_data,
                                    integrity=data.get('integrity', 0),
                                    rssi=data.get('rssi', 0)
                                )
                            
                            # Log messaggio ricevuto su console
                            print("\n" + "=" * 70)
                            print("[MSG #{}] TRANSPONDER -> TRANSCEIVER".format(
                                state["messages_received"]))
                            print("=" * 70)
                            print("  Timestamp: {:.2f}s".format(vehicle_data['timestamp']))
                            print("  Telemetry:")
                            print("    Position: x={:.3f}  y={:.3f}  z={:.3f} m".format(
                                pos['x'], pos['y'], pos['z']))
                            print("    Orient:   roll={:.2f}  pitch={:.2f}  yaw={:.2f} deg".format(
                                ori['roll'], ori['pitch'], ori['yaw']))
                            print("    Battery:  {:.1f}% @ {:.2f}V".format(
                                bat['charge_percent'], bat['voltage_volts']))
                            print("    Sensors:  temp={:.1f}C  press={:.1f}mbar".format(
                                dep['temperature'], dep['pressure']))
                            print("-" * 70)
                            
                            # Salva vehicle_data e invia AT?T direttamente (NON bloccare il listener)
                            print("[TC] Querying AT?T for propagation time...")
                            state["pending_vehicle_data"] = vehicle_data
                            state["waiting_at_t"] = True
                            state["at_t_value_us"] = None
                            
                            # Invia AT?T direttamente come in USBL_v6
                            at_command = "+++AT?T\n".encode(CODEC)
                            state["writer"].write(at_command)
                            await state["writer"].drain()
                            
                            # Avvia timeout handler in background
                            asyncio.ensure_future(handle_at_t_timeout(vehicle_data))
                        else:
                            print("[TC] Failed to decode vehicle message")
                else:
                    print("[TC] Low integrity message (integrity={})".format(
                        data.get('integrity', '?')))
    
    except asyncio.CancelledError:
        pass
    except Exception as e:
        print("[TC] Listener error: {}".format(e))


async def main():
    state["at_t_event"] = asyncio.Event()
    state["delivery_event"] = asyncio.Event()
    state["session_start"] = time.time()
    
    # Inizializza logger
    state["logger"] = TransceiverLogger()
    
    print("=" * 70)
    print("USBL TRANSCEIVER (Mission Control)")
    print("=" * 70)
    print("Transceiver ID: {}".format(TRANSCEIVER_ID))
    print("Transponder ID: {} (vehicle)".format(TRANSPONDER_ID))
    print("USBL IP:        {}:{}".format(USBL_IP, USBL_PORT))
    print("Sound Speed:    {} m/s".format(SPEED_OF_SOUND_MPS))
    print("")
    print("Control defaults:")
    print("  ENABLE: {}".format(CONTROL_ENABLE))
    print("  BIAS: {} deg".format(CONTROL_BIAS_DEG))
    print("  FREQ: {} Hz".format(CONTROL_FREQ_HZ))
    print("  USE_RANGE_ONLY: {}".format(CONTROL_USE_RANGE_ONLY))
    print("=" * 70)
    print("")
    
    try:
        reader, writer = await asyncio.open_connection(USBL_IP, USBL_PORT)
        state["writer"] = writer
        print("[TC] Connected to USBL modem")
        print("")
    except Exception as e:
        print("[TC] Connection failed: {}".format(e))
        return
    
    listener_task = asyncio.ensure_future(response_listener(reader))
    
    try:
        await listener_task
    except KeyboardInterrupt:
        print("\n[TC] Stopping...")
    finally:
        listener_task.cancel()
        try:
            await listener_task
        except asyncio.CancelledError:
            pass
        
        writer.close()
        
        # Chiudi logger
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