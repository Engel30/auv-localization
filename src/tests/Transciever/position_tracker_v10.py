#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
USBL Transceiver (Mission Control) with GUI
Riceve telemetria dal transponder e risponde con distanza 3D calcolata via AT?T
GUI opzionale per visualizzare traiettoria in tempo reale

FIXED: matplotlib in main thread, asyncio in separate thread
"""

import asyncio
import struct
import base64
import time
import math
import threading
from collections import deque

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

# GUI Configuration
ENABLE_GUI = True  # Set to False to disable GUI
GUI_HISTORY_LENGTH = 600  # Number of points to keep in plots
GUI_UPDATE_INTERVAL = 200  # ms between GUI updates

# =============================================================================
#  GUI IMPORTS (optional)
# =============================================================================
if ENABLE_GUI:
    try:
        import matplotlib
        matplotlib.use('TkAgg')
        import matplotlib.pyplot as plt
        MATPLOTLIB_AVAILABLE = True
    except ImportError:
        print("[WARNING] matplotlib not available - GUI disabled")
        MATPLOTLIB_AVAILABLE = False
        ENABLE_GUI = False
else:
    MATPLOTLIB_AVAILABLE = False

# =============================================================================
#  MESSAGE PROTOCOL
# =============================================================================
MSG_TYPE_VEHICLE = 0x01
MSG_TYPE_MC = 0x02

STRUCT_VEHICLE = '<Bfffffffffff'
STRUCT_MC = '<BfBffffffBBffBf'


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
#  GLOBAL STATE
# =============================================================================
state = {
    "session_start": None,
    "writer": None,
    "messages_received": 0,
    "messages_sent": 0,
    "waiting_at_t": False,
    "at_t_event": None,
    "at_t_value_us": None,
    "pending_vehicle_data": None,
    "waiting_delivery": False,
    "delivery_event": None,
    "delivery_success": False,
    "running": True
}

# GUI data (thread-safe)
gui_data = {
    'lock': threading.Lock(),
    'time': deque(maxlen=GUI_HISTORY_LENGTH),
    'x': deque(maxlen=GUI_HISTORY_LENGTH),
    'y': deque(maxlen=GUI_HISTORY_LENGTH),
    'z': deque(maxlen=GUI_HISTORY_LENGTH),
    'yaw': deque(maxlen=GUI_HISTORY_LENGTH),
    'current_pos': None,
    'current_ori': None
}


def get_relative_timestamp():
    if state["session_start"] is None:
        state["session_start"] = time.time()
    return time.time() - state["session_start"]


def update_gui_data(vehicle_data):
    """Update GUI data with new vehicle position."""
    if not ENABLE_GUI or not MATPLOTLIB_AVAILABLE:
        return
    
    pos = vehicle_data['pose']['position']
    ori = vehicle_data['pose']['orientation']
    timestamp = vehicle_data['timestamp']
    
    with gui_data['lock']:
        gui_data['time'].append(timestamp)
        gui_data['x'].append(pos['x'])
        gui_data['y'].append(pos['y'])
        gui_data['z'].append(pos['z'])
        gui_data['yaw'].append(ori['yaw'])
        gui_data['current_pos'] = pos
        gui_data['current_ori'] = ori


def calculate_range_3d_from_prop_time(prop_time_us):
    """Calcola distanza 3D dal propagation time."""
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
    
    if state["waiting_at_t"] and state["pending_vehicle_data"] is vehicle_data:
        print("[TC] AT?T timeout - sending response with 0 range")
        state["waiting_at_t"] = False
        state["pending_vehicle_data"] = None
        await send_response_to_transponder(vehicle_data, 0.0)


async def send_response_to_transponder(vehicle_data, range_3d):
    """Invia risposta al transponder con range 3D calcolato."""
    
    timestamp = get_relative_timestamp()
    mode = 1
    east, north, up = 0.0, 0.0, 0.0
    bearing_rad, elevation_rad = 0.0, 0.0
    distance = range_3d
    integrity = DEFAULT_INTEGRITY
    
    payload = encode_mc_message(
        timestamp=timestamp,
        mode=mode,
        east=east, north=north, up=up,
        bearing_rad=bearing_rad, elevation_rad=elevation_rad,
        distance=distance, integrity=integrity,
        enable=CONTROL_ENABLE,
        bias_deg=CONTROL_BIAS_DEG,
        freq_hz=CONTROL_FREQ_HZ,
        use_range_only=CONTROL_USE_RANGE_ONLY,
        range_3d=range_3d
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
        while state["running"]:
            line = await reader.readline()
            if not line:
                print("[TC] Connection closed")
                break
            
            msg_type, data = decode_modem_response(line)
            
            if msg_type == "AT_T_RESPONSE" and state["waiting_at_t"]:
                prop_time_us = data["time_us"]
                state["at_t_value_us"] = prop_time_us
                state["waiting_at_t"] = False
                
                print("[TC] AT?T response: {:.0f} us".format(prop_time_us))
                
                if state["pending_vehicle_data"] is not None:
                    vehicle_data = state["pending_vehicle_data"]
                    state["pending_vehicle_data"] = None
                    
                    range_3d = calculate_range_3d_from_prop_time(prop_time_us)
                    print("[TC] Propagation time: {:.0f} us -> Range 3D: {:.3f} m".format(
                        prop_time_us, range_3d))
                    
                    await send_response_to_transponder(vehicle_data, range_3d)
            
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
                            
                            update_gui_data(vehicle_data)
                            
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
                            
                            print("[TC] Querying AT?T for propagation time...")
                            state["pending_vehicle_data"] = vehicle_data
                            state["waiting_at_t"] = True
                            state["at_t_value_us"] = None
                            
                            at_command = "+++AT?T\n".encode(CODEC)
                            state["writer"].write(at_command)
                            await state["writer"].drain()
                            
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


async def async_main():
    """Main asyncio coroutine."""
    state["at_t_event"] = asyncio.Event()
    state["delivery_event"] = asyncio.Event()
    state["session_start"] = time.time()
    
    try:
        reader, writer = await asyncio.open_connection(USBL_IP, USBL_PORT)
        state["writer"] = writer
        print("[TC] Connected to USBL modem")
        print("")
    except Exception as e:
        print("[TC] Connection failed: {}".format(e))
        print("[TC] Make sure USBL modem is connected at {}:{}".format(USBL_IP, USBL_PORT))
        state["running"] = False
        return
    
    listener_task = asyncio.ensure_future(response_listener(reader))
    
    try:
        await listener_task
    except asyncio.CancelledError:
        pass
    except Exception as e:
        print("[TC] Error: {}".format(e))
    finally:
        listener_task.cancel()
        try:
            await listener_task
        except asyncio.CancelledError:
            pass
        
        writer.close()
        
        print("\n" + "=" * 70)
        print("SUMMARY")
        print("=" * 70)
        print("Messages received: {}".format(state["messages_received"]))
        print("Messages sent:     {}".format(state["messages_sent"]))
        print("=" * 70)


def asyncio_thread():
    """Thread per asyncio event loop."""
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    try:
        loop.run_until_complete(async_main())
    except KeyboardInterrupt:
        pass
    finally:
        loop.close()


# =============================================================================
#  MAIN with GUI in main thread
# =============================================================================
def main():
    """Main function - runs GUI in main thread, asyncio in separate thread."""
    
    print("=" * 70)
    print("USBL TRANSCEIVER (Mission Control)")
    print("=" * 70)
    print("Transceiver ID: {}".format(TRANSCEIVER_ID))
    print("Transponder ID: {} (vehicle)".format(TRANSPONDER_ID))
    print("USBL IP:        {}:{}".format(USBL_IP, USBL_PORT))
    print("Sound Speed:    {} m/s".format(SPEED_OF_SOUND_MPS))
    print("GUI:            {}".format("ENABLED" if ENABLE_GUI else "DISABLED"))
    print("")
    print("Control defaults:")
    print("  ENABLE: {}".format(CONTROL_ENABLE))
    print("  BIAS: {} deg".format(CONTROL_BIAS_DEG))
    print("  FREQ: {} Hz".format(CONTROL_FREQ_HZ))
    print("  USE_RANGE_ONLY: {}".format(CONTROL_USE_RANGE_ONLY))
    print("=" * 70)
    print("")
    
    # Start asyncio in separate thread
    asyncio_t = threading.Thread(target=asyncio_thread, daemon=True)
    asyncio_t.start()
    
    if ENABLE_GUI and MATPLOTLIB_AVAILABLE:
        # Run GUI in main thread
        print("[GUI] Starting visualization...")
        
        plt.ion()
        fig = plt.figure(figsize=(14, 8))
        fig.canvas.manager.set_window_title('USBL Transceiver - Vehicle Position')
        
        # XY trajectory
        ax_xy = fig.add_subplot(2, 2, 1)
        line_xy, = ax_xy.plot([], [], 'b-', linewidth=2, label='Trajectory')
        point_current, = ax_xy.plot([], [], 'ro', markersize=10, label='Current')
        ax_xy.scatter([0], [0], c='orange', s=200, marker='^', label='Transceiver', zorder=5)
        ax_xy.set_xlabel('X [m]')
        ax_xy.set_ylabel('Y [m]')
        ax_xy.set_title('XY Trajectory (Top View)')
        ax_xy.grid(True)
        ax_xy.legend(loc='upper right')
        ax_xy.axis('equal')
        
        # XZ profile
        ax_xz = fig.add_subplot(2, 2, 2)
        line_xz, = ax_xz.plot([], [], 'b-', linewidth=2)
        point_current_xz, = ax_xz.plot([], [], 'ro', markersize=10)
        ax_xz.scatter([0], [0], c='orange', s=200, marker='^', zorder=5)
        ax_xz.set_xlabel('X [m]')
        ax_xz.set_ylabel('Z (Depth) [m]')
        ax_xz.set_title('XZ Profile')
        ax_xz.grid(True)
        ax_xz.invert_yaxis()
        
        # Time series XY
        ax_time_xy = fig.add_subplot(2, 2, 3)
        line_x, = ax_time_xy.plot([], [], 'r-', linewidth=2, label='X')
        line_y, = ax_time_xy.plot([], [], 'b-', linewidth=2, label='Y')
        ax_time_xy.set_xlabel('Time [s]')
        ax_time_xy.set_ylabel('Position [m]')
        ax_time_xy.set_title('Position vs Time')
        ax_time_xy.grid(True)
        ax_time_xy.legend()
        
        # Time series Yaw
        ax_time_yaw = fig.add_subplot(2, 2, 4)
        line_yaw, = ax_time_yaw.plot([], [], 'g-', linewidth=2)
        ax_time_yaw.set_xlabel('Time [s]')
        ax_time_yaw.set_ylabel('Yaw [deg]')
        ax_time_yaw.set_title('Heading vs Time')
        ax_time_yaw.grid(True)
        
        # Info text
        info_text = fig.text(0.02, 0.98, 'Waiting for data...', fontsize=10,
                             verticalalignment='top', family='monospace',
                             bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
        
        plt.tight_layout()
        
        try:
            while state["running"]:
                with gui_data['lock']:
                    if len(gui_data['x']) > 0:
                        x_data = list(gui_data['x'])
                        y_data = list(gui_data['y'])
                        z_data = list(gui_data['z'])
                        time_data = list(gui_data['time'])
                        yaw_data = list(gui_data['yaw'])
                        current_pos = gui_data['current_pos']
                        current_ori = gui_data['current_ori']
                        
                        # Update XY
                        line_xy.set_data(x_data, y_data)
                        if current_pos:
                            point_current.set_data([current_pos['x']], [current_pos['y']])
                        
                        if len(x_data) > 0:
                            x_min, x_max = min(x_data), max(x_data)
                            y_min, y_max = min(y_data), max(y_data)
                            x_margin = max(0.5, (x_max - x_min) * 0.1)
                            y_margin = max(0.5, (y_max - y_min) * 0.1)
                            ax_xy.set_xlim(x_min - x_margin, x_max + x_margin)
                            ax_xy.set_ylim(y_min - y_margin, y_max + y_margin)
                        
                        # Update XZ
                        line_xz.set_data(x_data, z_data)
                        if current_pos:
                            point_current_xz.set_data([current_pos['x']], [current_pos['z']])
                        
                        if len(z_data) > 0:
                            z_min, z_max = min(z_data), max(z_data)
                            z_margin = max(0.5, (z_max - z_min) * 0.1)
                            ax_xz.set_ylim(z_min - z_margin, z_max + z_margin)
                        
                        # Update time series
                        line_x.set_data(time_data, x_data)
                        line_y.set_data(time_data, y_data)
                        line_yaw.set_data(time_data, yaw_data)
                        
                        if len(time_data) > 0:
                            ax_time_xy.set_xlim(min(time_data), max(time_data))
                            y_vals = x_data + y_data
                            ax_time_xy.set_ylim(min(y_vals) - 0.5, max(y_vals) + 0.5)
                            
                            ax_time_yaw.set_xlim(min(time_data), max(time_data))
                            ax_time_yaw.set_ylim(min(yaw_data) - 10, max(yaw_data) + 10)
                        
                        # Update info
                        if current_pos and current_ori:
                            info_str = "Current Position:\n"
                            info_str += "  X: {:.3f} m\n".format(current_pos['x'])
                            info_str += "  Y: {:.3f} m\n".format(current_pos['y'])
                            info_str += "  Z: {:.3f} m\n".format(current_pos['z'])
                            info_str += "Orientation:\n"
                            info_str += "  Yaw: {:.1f} deg\n".format(current_ori['yaw'])
                            info_str += "Messages: {}".format(state["messages_received"])
                            info_text.set_text(info_str)
                
                plt.pause(GUI_UPDATE_INTERVAL / 1000.0)
                fig.canvas.draw()
                fig.canvas.flush_events()
                
        except KeyboardInterrupt:
            print("\n[GUI] Closing...")
            state["running"] = False
        finally:
            plt.close('all')
    
    else:
        # No GUI - just wait for asyncio thread
        try:
            asyncio_t.join()
        except KeyboardInterrupt:
            print("\n[TC] Stopping...")
            state["running"] = False


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        state["running"] = False
        print("\n[TC] Shutdown complete")