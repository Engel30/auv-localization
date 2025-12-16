#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
USBL Transponder Library
Gestisce la comunicazione acustica dal lato veicolo (transponder)
Riceve dati USBL dal mission control e invia telemetria

Pattern REQUEST-RESPONSE:
- Transponder invia telemetria
- Aspetta risposta dal transceiver (con timeout)
- Solo dopo procede con il prossimo invio

Protocollo messaggi compresso (struct binario + base64):
- Max 64 byte payload
"""

import asyncio
import json
import time
import math
import struct
import base64
import os
import csv
from datetime import datetime

# =============================================================================
#  MESSAGE PROTOCOL - BINARY COMPRESSED
# =============================================================================
MSG_TYPE_VEHICLE = 0x01
MSG_TYPE_MC = 0x02

STRUCT_VEHICLE = '<Bfffffffffff'       # 45 bytes
STRUCT_MC = '<BfBffffffBBffBf'       # 45 bytes (no prop_time, no rssi)


def encode_vehicle_message(timestamp, x, y, z, roll, pitch, yaw,
                           battery_pct, voltage, temperature, pressure):
    """Codifica messaggio veicolo -> mission control."""
    data = struct.pack(
        STRUCT_VEHICLE,
        MSG_TYPE_VEHICLE,
        float(timestamp),
        float(x), float(y), float(z),
        float(roll), float(pitch), float(yaw),
        float(battery_pct), float(voltage),
        float(temperature), float(pressure)
    )
    return base64.b64encode(data).decode('ascii')


def decode_vehicle_message(payload_b64):
    """Decodifica messaggio veicolo."""
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
    except Exception:
        return None


def encode_mc_message(timestamp, mode, east, north, up, bearing_rad, elevation_rad,
                      distance, integrity, enable, bias_deg, freq_hz,
                      use_range_only, range_3d):
    """Codifica messaggio mission control -> veicolo."""
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


def decode_mc_message(payload_b64):
    """Decodifica messaggio mission control."""
    try:
        data = base64.b64decode(payload_b64)
        values = struct.unpack(STRUCT_MC, data)
        
        if values[0] != MSG_TYPE_MC:
            return None
        
        mode = values[2]
        
        return {
            'timestamp': values[1],
            'usbl_data': {
                'mode': 'LONG' if mode == 1 else 'ANGLES',
                'enu': {'east_m': values[3], 'north_m': values[4], 'up_m': values[5]},
                'angles': {
                    'bearing_rad': values[6],
                    'elevation_rad': values[7],
                    'bearing_deg': math.degrees(values[6]),
                    'elevation_deg': math.degrees(values[7])
                },
                'estimated_distance_m': values[8],
                                'integrity': values[9],
                                'range_3d_m': values[14]
            },
            'control_data': {
                'enable': values[10] == 1,
                'bias_angle_deg': values[11],
                'frequency_hz': values[12],
                'use_range_only': values[13] == 1
            }
        }
    except Exception:
        return None


class USBLMessageLogger(object):
    """Logger per messaggi USBL inviati e ricevuti."""
    
    def __init__(self, log_dir="sensor_logs", enabled=True):
        self.enabled = enabled
        if not self.enabled:
            return
        
        if not os.path.exists(log_dir):
            os.makedirs(log_dir)
        
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.filepath = os.path.join(log_dir, "usbl_messages_{}.csv".format(timestamp))
        
        self.fp = open(self.filepath, 'w')
        self.writer = csv.writer(self.fp)
        
        self.writer.writerow([
            'timestamp_abs', 'timestamp_rel', 'msg_num', 'direction',
            'mode', 'east', 'north', 'up', 'bearing_deg', 'elevation_deg',
            'distance', 'range_3d', 'integrity', 'rssi',
            'enable', 'bias_deg', 'freq_hz', 'use_range_only'
        ])
        self.fp.flush()
        
        self.t0 = None
        self.msg_count_rx = 0
        self.msg_count_tx = 0
    
    def _get_timestamps(self):
        now = time.time()
        if self.t0 is None:
            self.t0 = now
        return now, now - self.t0
    
    def log_received(self, data):
        if not self.enabled:
            return
        
        t_abs, t_rel = self._get_timestamps()
        self.msg_count_rx += 1
        
        usbl = data.get('usbl_data', {})
        ctrl = data.get('control_data', {})
        
        self.writer.writerow([
            t_abs, t_rel, self.msg_count_rx, 'RX (TC->TP)',
            usbl.get('mode', ''),
            usbl.get('enu', {}).get('east_m', 0),
            usbl.get('enu', {}).get('north_m', 0),
            usbl.get('enu', {}).get('up_m', 0),
            usbl.get('angles', {}).get('bearing_deg', 0),
            usbl.get('angles', {}).get('elevation_deg', 0),
            usbl.get('estimated_distance_m', 0),
            usbl.get('range_3d_m', 0),
            usbl.get('integrity', 0),
            usbl.get('RSSI', 0),
            ctrl.get('enable', False),
            ctrl.get('bias_angle_deg', 0),
            ctrl.get('frequency_hz', 0),
            ctrl.get('use_range_only', False)
        ])
        self.fp.flush()
    
    def log_sent(self, t):
        if not self.enabled:
            return
        
        t_abs, t_rel = self._get_timestamps()
        self.msg_count_tx += 1
        
        self.writer.writerow([
            t_abs, t_rel, self.msg_count_tx, 'TX (TP->TC)',
            '', t['x'], t['y'], t['z'], t['roll'], t['pitch'], t['yaw'], '',
            '', '', '', '', '', ''
        ])
        self.fp.flush()
    
    def close(self):
        if self.enabled and self.fp:
            self.fp.close()


class USBLTransponder(object):
    """
    Gestisce la comunicazione USBL lato transponder (veicolo).
    
    Pattern REQUEST-RESPONSE:
    - Invia telemetria
    - Aspetta risposta dal transceiver (MC data)
    - Solo dopo procede
    """
    
    def __init__(self, ip, port, transceiver_id, transponder_id,
                 codec="utf-8", min_integrity=50, speed_of_sound=1500.0):
        self.ip = ip
        self.port = port
        self.transceiver_id = transceiver_id
        self.transponder_id = transponder_id
        self.codec = codec
        self.min_integrity = min_integrity
        self.speed_of_sound = speed_of_sound
        
        self.reader = None
        self.writer = None
        self.connected = False
        self.running = False
        self.session_start = None
        
        self.last_mc_data = {
            'valid': False,
            'timestamp': 0.0,
            'usbl_data': {
                'mode': 'ANGLES',
                'enu': {'east_m': 0.0, 'north_m': 0.0, 'up_m': 0.0},
                'angles': {'bearing_rad': 0.0, 'elevation_rad': 0.0,
                          'bearing_deg': 0.0, 'elevation_deg': 0.0},
                'estimated_distance_m': 0.0,
                'propagation_time_us': 0.0,
                'integrity': 0,
                'RSSI': 0,
                'range_3d_m': 0.0
            },
            'control_data': {
                'enable': False,
                'bias_angle_deg': 0.0,
                'frequency_hz': 0.1,
                'use_range_only': False
            },
            'received_at': 0.0
        }
        
        self.on_mc_data_received = None
        self.on_message_received = None
        
        self.stats = {
            'messages_sent': 0,
            'messages_received': 0,
            'delivery_success': 0,
            'delivery_failed': 0,
            'responses_received': 0,
            'responses_timeout': 0
        }
        
        # Eventi per sincronizzazione request-response
        self._delivery_event = None
        self._delivery_result = None
        self._response_event = None  # Evento per aspettare risposta MC
        self._response_received = False
    
    def _get_relative_timestamp(self):
        if self.session_start is None:
            self.session_start = time.time()
        return time.time() - self.session_start
    
    def _encode_at_sendim(self, target_id, payload_str):
        payload_with_marker = "$" + payload_str
        if len(payload_with_marker) > 64:
            raise ValueError("Payload troppo lungo: {} bytes (max 64)".format(len(payload_with_marker)))
        command = "AT*SENDIM,p0,{},{},ack,{}".format(len(payload_with_marker), target_id, payload_with_marker)
        return "+++{}\n".format(command).encode(self.codec)
    
    def _decode_response(self, message):
        try:
            str_msg = message.decode(self.codec).strip()
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
                            "valid": integrity > self.min_integrity,
                            "rssi": rssi,
                            "payload": payload
                        }
                    except (ValueError, IndexError):
                        return "RECVIM_ERROR", {}
                
                elif keyword in ["SENDSTART", "SENDEND", "RECVSTART", "RECVEND",
                                "USBLLONG", "USBLANGLES", "USBLPHYD", "USBLPHYP"]:
                    return keyword, {"args": args}
            
            return "OTHER", {}
        except Exception:
            return "ERROR", {}
    
    def _parse_mc_message(self, payload_str):
        if not payload_str:
            return False
        
        decoded = decode_mc_message(payload_str.strip())
        if decoded:
            self.last_mc_data['valid'] = True
            self.last_mc_data['timestamp'] = decoded['timestamp']
            self.last_mc_data['usbl_data'] = decoded['usbl_data']
            self.last_mc_data['control_data'] = decoded['control_data']
            self.last_mc_data['received_at'] = time.time()
            
            if self.on_mc_data_received:
                self.on_mc_data_received(self.last_mc_data)
            
            return True
        return False
    
    async def _reader_task(self):
        try:
            while self.running:
                line = await self.reader.readline()
                if not line:
                    print("[TRANSPONDER] Connection closed")
                    break
                
                msg_type, data = self._decode_response(line)
                
                if msg_type == "DELIVEREDIM":
                    self._delivery_result = True
                    if self._delivery_event:
                        self._delivery_event.set()
                    self.stats['delivery_success'] += 1
                
                elif msg_type == "FAILEDIM":
                    self._delivery_result = False
                    if self._delivery_event:
                        self._delivery_event.set()
                    self.stats['delivery_failed'] += 1
                
                elif msg_type == "RECVIM":
                    self.stats['messages_received'] += 1
                    if data.get("valid"):
                        payload = data.get("payload")
                        if payload:
                            # Prova a parsare come messaggio MC
                            if self._parse_mc_message(payload):
                                # Risposta MC ricevuta! Segnala l'evento
                                self._response_received = True
                                self.stats['responses_received'] += 1
                                if self._response_event:
                                    self._response_event.set()
                        
                        if self.on_message_received:
                            self.on_message_received(data)
                    
        except asyncio.CancelledError:
            pass
        except Exception as e:
            print("[TRANSPONDER] Reader error: {}".format(e))
    
    async def connect(self):
        try:
            self.reader, self.writer = await asyncio.open_connection(self.ip, self.port)
            self.connected = True
            self.running = True
            self.session_start = time.time()
            print("[TRANSPONDER] Connected to {}:{}".format(self.ip, self.port))
            return True
        except Exception as e:
            print("[TRANSPONDER] Connection failed: {}".format(e))
            return False
    
    async def start(self):
        if not self.connected:
            success = await self.connect()
            if not success:
                return None
        
        self._delivery_event = asyncio.Event()
        self._response_event = asyncio.Event()
        reader_task = asyncio.ensure_future(self._reader_task())
        return reader_task
    
    async def stop(self, reader_task=None):
        self.running = False
        if reader_task:
            reader_task.cancel()
            try:
                await reader_task
            except asyncio.CancelledError:
                pass
        if self.writer:
            self.writer.close()
        self.connected = False
    
    async def send_and_wait_response(self, x, y, z, roll, pitch, yaw,
                                      battery_pct, voltage, temperature, pressure,
                                      delivery_timeout=5.0, response_timeout=8.0):
        """
        Invia telemetria e ASPETTA la risposta dal transceiver.
        
        Args:
            delivery_timeout: timeout per conferma invio (DELIVEREDIM)
            response_timeout: timeout per risposta dal transceiver (RECVIM con MC data)
        
        Returns:
            tuple (delivery_success, response_received, mc_data or None)
        """
        if not self.connected:
            return (False, False, None)
        
        ts = self._get_relative_timestamp()
        payload = encode_vehicle_message(
            ts, x, y, z, roll, pitch, yaw,
            battery_pct, voltage, temperature, pressure
        )
        
        try:
            encoded = self._encode_at_sendim(self.transceiver_id, payload)
        except ValueError as e:
            print("[TRANSPONDER] Encode error: {}".format(e))
            return (False, False, None)
        
        # Reset eventi
        self._delivery_event.clear()
        self._response_event.clear()
        self._delivery_result = None
        self._response_received = False
        
        # Invia messaggio
        self.writer.write(encoded)
        await self.writer.drain()
        self.stats['messages_sent'] += 1
        
        # 1. Aspetta conferma delivery
        delivery_success = False
        try:
            await asyncio.wait_for(self._delivery_event.wait(), timeout=delivery_timeout)
            delivery_success = (self._delivery_result == True)
        except asyncio.TimeoutError:
            print("[TRANSPONDER] Delivery timeout")
            delivery_success = False
        
        if not delivery_success:
            return (False, False, None)
        
        # 2. Aspetta risposta dal transceiver
        response_received = False
        mc_data = None
        try:
            await asyncio.wait_for(self._response_event.wait(), timeout=response_timeout)
            response_received = self._response_received
            if response_received:
                mc_data = self.last_mc_data.copy()
        except asyncio.TimeoutError:
            print("[TRANSPONDER] Response timeout (no MC data received)")
            self.stats['responses_timeout'] += 1
            response_received = False
        
        return (delivery_success, response_received, mc_data)
    
    def get_mc_data(self):
        return self.last_mc_data.copy()
    
    def get_usbl_position(self):
        if not self.last_mc_data['valid']:
            return (0.0, 0.0, 0.0)
        
        usbl = self.last_mc_data['usbl_data']
        
        if usbl['mode'] == 'LONG':
            return (usbl['enu']['east_m'], usbl['enu']['north_m'], usbl['enu']['up_m'])
        else:
            bearing = usbl['angles']['bearing_rad']
            elevation = usbl['angles']['elevation_rad']
            distance = usbl['range_3d_m']
            
            e = distance * math.cos(elevation) * math.sin(bearing)
            n = distance * math.cos(elevation) * math.cos(bearing)
            u = distance * math.sin(elevation)
            
            return (e, n, u)
    
    def get_range_3d(self):
        if not self.last_mc_data['valid']:
            return 0.0
        return self.last_mc_data['usbl_data'].get('range_3d_m', 0.0)
    
    def get_control_data(self):
        return self.last_mc_data['control_data'].copy()
    
    def get_stats(self):
        return self.stats.copy()


class USBLTransponderThread(object):
    """
    Wrapper per eseguire USBLTransponder in un thread separato.
    
    Pattern REQUEST-RESPONSE:
    - Invia telemetria
    - Aspetta risposta
    - Pausa
    - Ripete
    """
    
    def __init__(self, ip, port, transceiver_id, transponder_id,
                 data_lock=None, sensor_data_ref=None, logger=None,
                 send_interval=2.0, response_timeout=8.0,
                 log_directory="sensor_logs", **kwargs):
        self.transponder = USBLTransponder(
            ip, port, transceiver_id, transponder_id, **kwargs
        )
        self.data_lock = data_lock
        self.sensor_data_ref = sensor_data_ref
        self.logger = logger
        self.send_interval = send_interval
        self.response_timeout = response_timeout
        self.running = False
        self._loop = None
        self._reader_task = None
        
        self.message_logger = USBLMessageLogger(log_directory)
        
        self.telemetry = {
            'x': 0.0, 'y': 0.0, 'z': 0.0,
            'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
            'battery_pct': 100.0, 'voltage': 12.0,
            'temperature': 20.0, 'pressure': 1013.0
        }
        self.telemetry_lock = None
        
        # Callbacks
        self.on_control_data_received = None
        self.on_mc_data_callback = None
        self.on_telemetry_sent_callback = None
        
        self.transponder.on_mc_data_received = self._on_mc_data
    
    def _on_mc_data(self, data):
        """Callback quando arrivano dati dal mission control."""
        usbl = data['usbl_data']
        ctrl = data['control_data']
        
        if usbl['mode'] == 'LONG':
            e = usbl['enu']['east_m']
            n = usbl['enu']['north_m']
            u = usbl['enu']['up_m']
        else:
            bearing = usbl['angles']['bearing_rad']
            elevation = usbl['angles']['elevation_rad']
            distance = usbl.get('range_3d_m', 0)
            if distance > 0:
                e = distance * math.cos(elevation) * math.sin(bearing)
                n = distance * math.cos(elevation) * math.cos(bearing)
                u = distance * math.sin(elevation)
            else:
                e, n, u = 0, 0, 0
        
        range_m = usbl['estimated_distance_m']
        range_3d = usbl.get('range_3d_m', 0.0)
        
        if self.data_lock and self.sensor_data_ref:
            with self.data_lock:
                self.sensor_data_ref['usbl']['range'] = range_m
                self.sensor_data_ref['usbl']['range_3d'] = range_3d
                self.sensor_data_ref['usbl']['valid'] = True
                self.sensor_data_ref['usbl']['timestamp'] = time.time()
                self.sensor_data_ref['usbl']['new_data'] = True
                self.sensor_data_ref['usbl']['integrity'] = usbl['integrity']
                self.sensor_data_ref['usbl']['rssi'] = usbl.get('RSSI', 0)
                self.sensor_data_ref['usbl']['east'] = e
                self.sensor_data_ref['usbl']['north'] = n
                self.sensor_data_ref['usbl']['up'] = u
                self.sensor_data_ref['usbl']['mode'] = usbl['mode']
                self.sensor_data_ref['usbl']['use_range_only'] = ctrl['use_range_only']
        
        if self.logger:
            self.logger.log_usbl(range_m, usbl['integrity'], usbl.get('RSSI', 0), e, n, u,
                                 range_3d, usbl['mode'], ctrl['use_range_only'])
        
        self.message_logger.log_received(data)
        
        # Callback per log nel main
        if self.on_mc_data_callback:
            self.on_mc_data_callback(data)
        
        # Callback per control data (seriale)
        if self.on_control_data_received:
            self.on_control_data_received(ctrl)
    
    async def _communication_loop(self):
        """
        Loop principale di comunicazione REQUEST-RESPONSE.
        """
        await asyncio.sleep(2.0)  # Attesa iniziale
        
        while self.running:
            try:
                # Ottieni telemetria corrente
                if self.telemetry_lock:
                    with self.telemetry_lock:
                        t = self.telemetry.copy()
                else:
                    t = self.telemetry.copy()
                
                # Callback per log nel main (prima dell'invio)
                if self.on_telemetry_sent_callback:
                    self.on_telemetry_sent_callback(t)
                
                self.message_logger.log_sent(t)
                
                # INVIA E ASPETTA RISPOSTA
                delivery_ok, response_ok, mc_data = await self.transponder.send_and_wait_response(
                    t['x'], t['y'], t['z'],
                    t['roll'], t['pitch'], t['yaw'],
                    t['battery_pct'], t['voltage'],
                    t['temperature'], t['pressure'],
                    delivery_timeout=5.0,
                    response_timeout=self.response_timeout
                )
                
                if delivery_ok and response_ok:
                    print("[TRANSPONDER] Exchange complete: sent + received response")
                elif delivery_ok:
                    print("[TRANSPONDER] Sent OK, but no response from transceiver")
                else:
                    print("[TRANSPONDER] Delivery FAILED")
                
                # Pausa prima del prossimo ciclo
                await asyncio.sleep(self.send_interval)
                
            except Exception as e:
                print("[TRANSPONDER] Communication error: {}".format(e))
                await asyncio.sleep(self.send_interval)
    
    async def _main_async(self):
        self._reader_task = await self.transponder.start()
        if not self._reader_task:
            return
        
        comm_task = asyncio.ensure_future(self._communication_loop())
        
        try:
            while self.running:
                await asyncio.sleep(0.1)
        except asyncio.CancelledError:
            pass
        finally:
            comm_task.cancel()
            try:
                await comm_task
            except asyncio.CancelledError:
                pass
            await self.transponder.stop(self._reader_task)
    
    def run(self):
        self.running = True
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        try:
            self._loop.run_until_complete(self._main_async())
        except Exception as e:
            print("[TRANSPONDER] Thread error: {}".format(e))
        finally:
            self.message_logger.close()
            self._loop.close()
    
    def stop(self):
        self.running = False
    
    def update_telemetry(self, x, y, z, roll, pitch, yaw,
                         battery_pct=None, voltage=None,
                         temperature=None, pressure=None):
        if self.telemetry_lock:
            with self.telemetry_lock:
                self._update_telemetry_internal(x, y, z, roll, pitch, yaw,
                                                battery_pct, voltage, temperature, pressure)
        else:
            self._update_telemetry_internal(x, y, z, roll, pitch, yaw,
                                            battery_pct, voltage, temperature, pressure)
    
    def _update_telemetry_internal(self, x, y, z, roll, pitch, yaw,
                                    battery_pct, voltage, temperature, pressure):
        self.telemetry['x'] = x
        self.telemetry['y'] = y
        self.telemetry['z'] = z
        self.telemetry['roll'] = roll
        self.telemetry['pitch'] = pitch
        self.telemetry['yaw'] = yaw
        if battery_pct is not None:
            self.telemetry['battery_pct'] = battery_pct
        if voltage is not None:
            self.telemetry['voltage'] = voltage
        if temperature is not None:
            self.telemetry['temperature'] = temperature
        if pressure is not None:
            self.telemetry['pressure'] = pressure