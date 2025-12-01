import asyncio
import json
import logging
import math
import os
from datetime import datetime
from typing import Tuple, Dict, Any

# ==================== CONFIGURATION ====================
TRANSCEIVER_ID = 2
TRANSPONDER_ID = 3
USBL_IP = "192.168.0.139"
USBL_PORT = 9200
CODEC = "utf-8"
SPEED_OF_SOUND_MPS = 1500.0
MIN_INTEGRITY = 50

# Position capture timeout - how long to wait for USBLANGLES/USBLLONG after RECVIM
POSITION_CAPTURE_TIMEOUT = 2.0  # seconds

# Logging configuration
LOG_DIR = "logs"
LOG_LEVEL = logging.DEBUG  # File logs everything including DEBUG
CONSOLE_LOG_LEVEL = logging.INFO  # Console shows INFO and above

# ========================================================

INSTANT_MESSAGE_RESPONSE_KEYWORDS = {
    "SENDSTART", "SENDEND", "RECVSTART", "RECVEND", "DELIVEREDIM", "USBLLONG", 
    "USBLANGLES", "USBLPHYD", "USBLPHYP", "FAILEDIM", "RECVIM", "CANCELEDIM", 
    "ERROR INTERNAL"
}

# State
state = {
    "request_count": 0,
    "position_sent_count": 0,
    "waiting_for_position": False,
    "position_data": None,
    "position_event": None  # Will be initialized in main()
}


def setup_logging():
    # type: () -> str
    """Setup logging to both file and console. Returns the log file path."""
    
    # Create logs directory if it doesn't exist
    if not os.path.exists(LOG_DIR):
        os.makedirs(LOG_DIR)
    
    # Generate timestamped log filename
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_filename = "USBL_RXTX_{}.log".format(timestamp)
    log_filepath = os.path.join(LOG_DIR, log_filename)
    
    # Get root logger
    logger = logging.getLogger()
    logger.setLevel(logging.DEBUG)  # Capture all levels
    
    # Clear any existing handlers
    logger.handlers = []
    
    # File handler - logs everything with timestamps
    file_handler = logging.FileHandler(log_filepath, encoding='utf-8')
    file_handler.setLevel(LOG_LEVEL)
    file_format = logging.Formatter(
        '%(asctime)s | %(levelname)-8s | %(message)s',
        datefmt='%Y-%m-%d %H:%M:%S'
    )
    file_handler.setFormatter(file_format)
    logger.addHandler(file_handler)
    
    # Console handler - cleaner output
    console_handler = logging.StreamHandler()
    console_handler.setLevel(CONSOLE_LOG_LEVEL)
    console_format = logging.Formatter('%(message)s')
    console_handler.setFormatter(console_format)
    logger.addHandler(console_handler)
    
    return log_filepath


def calculate_distance_3d(east, north, up):
    # type: (float, float, float) -> float
    return math.sqrt(east**2 + north**2 + up**2)


def angles_to_enu(azimuth_deg, elevation_deg, range_m):
    # type: (float, float, float) -> Tuple[float, float, float]
    azimuth_rad = math.radians(azimuth_deg)
    elevation_rad = math.radians(elevation_deg)
    horizontal_range = range_m * math.cos(elevation_rad)
    east = horizontal_range * math.sin(azimuth_rad)
    north = horizontal_range * math.cos(azimuth_rad)
    up = range_m * math.sin(elevation_rad)
    return east, north, up


def encode_message(target_id, payload):
    # type: (int, Dict[str, Any]) -> bytes
    payload_str = json.dumps(payload, separators=(',', ':'))
    if len(payload_str) > 63:
        raise ValueError("Message too long: {} bytes".format(len(payload_str)))
    payload_str = "[{}".format(payload_str)
    command = "AT*SENDIM,p0,{},{},ack,{}".format(len(payload_str), target_id, payload_str)
    encoded = "+++{}\n".format(command).encode(CODEC)
    logging.debug("Encoded message: {}".format(encoded))
    return encoded


def decode_response(message):
    # type: (bytes) -> Tuple[str, Dict[str, Any]]
    try:
        str_msg = message.decode(CODEC).strip()
        logging.debug("Raw received: {}".format(str_msg))
        
        if not str_msg or not str_msg.startswith("+++"):
            return "UNKNOWN", {}
        
        parts = str_msg.split(":")
        
        if "AT" in str_msg:
            if len(parts) >= 3:
                args = parts[2].split(",")
                keyword = args[0]
                
                if keyword == "USBLANGLES" and len(args) >= 7:
                    try:
                        target_id = args[3]
                        azimuth = float(args[4])
                        elevation = float(args[5])
                        range_m = float(args[6])
                        east, north, up = angles_to_enu(azimuth, elevation, range_m)
                        distance_3d = calculate_distance_3d(east, north, up)
                        
                        result = {
                            "target_id": target_id, "azimuth": azimuth, "elevation": elevation,
                            "range": range_m, "east": east, "north": north, "up": up,
                            "distance_3d": distance_3d, "valid": True
                        }
                        logging.debug("Parsed USBLANGLES: {}".format(result))
                        return "USBLANGLES", result
                    except (ValueError, IndexError) as e:
                        logging.error("Error parsing USBLANGLES: {}".format(e))
                        return "USBLANGLES_ERROR", {}
                
                elif keyword == "USBLLONG" and len(args) == 17:
                    try:
                        target_id = args[3]
                        east = float(args[7])
                        north = float(args[8])
                        up = float(args[9])
                        propagation_time_us = int(args[13])
                        position_uncertainty_m = float(args[16])
                        slant_range_m = (propagation_time_us / 1_000_000.0) * SPEED_OF_SOUND_MPS
                        distance_3d = calculate_distance_3d(east, north, up)
                        
                        result = {
                            "target_id": target_id, "east": east, "north": north, "up": up,
                            "range": slant_range_m, "distance_3d": distance_3d,
                            "uncertainty": position_uncertainty_m,
                            "valid": position_uncertainty_m <= (0.95 * slant_range_m)
                        }
                        logging.debug("Parsed USBLLONG: {}".format(result))
                        return "USBLLONG", result
                    except (ValueError, IndexError) as e:
                        logging.error("Error parsing USBLLONG: {}".format(e))
                        return "USBLLONG_ERROR", {}
                
                elif keyword == "DELIVEREDIM":
                    return "DELIVEREDIM", {"target_id": args[1] if len(args) > 1 else "?"}
                
                elif keyword == "FAILEDIM":
                    return "FAILEDIM", {"target_id": args[1] if len(args) > 1 else "?"}
                
                elif keyword == "RECVIM" and len(args) >= 9:
                    try:
                        sender_id = args[3]
                        rssi = int(args[7])
                        integrity = int(args[8])
                        payload = None
                        if ',[' in str_msg:
                            _, payload = str_msg.rsplit(',[', 1)
                            payload = payload.strip()
                            if payload.startswith("{"):
                                try:
                                    payload = json.loads(payload)
                                except:
                                    pass
                        
                        result = {
                            "sender_id": sender_id, "integrity": integrity,
                            "valid": integrity > MIN_INTEGRITY, "rssi": rssi, "message": payload
                        }
                        logging.debug("Parsed RECVIM: {}".format(result))
                        return "RECVIM", result
                    except (ValueError, IndexError) as e:
                        logging.error("Error parsing RECVIM: {}".format(e))
                        return "RECVIM_ERROR", {}
                
                elif keyword in INSTANT_MESSAGE_RESPONSE_KEYWORDS:
                    logging.debug("Received {}: {}".format(keyword, args))
                    return keyword, {"args": args}
        
        else:
            cmd = parts[0].replace("+", "")
            if len(parts) >= 3:
                return "{}_RESPONSE".format(cmd), {"status": parts[2]}
        
        return "OTHER", {}
        
    except Exception as e:
        logging.error("Parse error: {}".format(e))
        return "ERROR", {}


@asyncio.coroutine
def response_reader(reader):
    logging.info("=" * 70)
    logging.info("TRANSCEIVER STARTED - Event-Driven Position Response")
    logging.info("=" * 70)
    logging.info("Listening for position requests...")
    logging.info("")
    
    try:
        while True:
            line = yield from reader.readline()
            if not line:
                logging.warning("Connection closed")
                break
            
            msg_type, data = decode_response(line)
            
            if msg_type == "AT*SENDIM_RESPONSE":
                if data.get("status") == "OK":
                    logging.debug("  OK USBL accepted send command")
            
            elif msg_type == "DELIVEREDIM":
                logging.info("  [OK] Position response delivered to transponder")
            
            elif msg_type == "FAILEDIM":
                logging.error("  [FAIL] Position response FAILED")
            
            elif msg_type == "RECVIM":
                if data["valid"]:
                    msg = data["message"]
                    state["request_count"] += 1
                    
                    logging.info("")
                    logging.info("=" * 70)
                    logging.info("[RECV] POSITION REQUEST RECEIVED #{}".format(state['request_count']))
                    logging.info("=" * 70)
                    logging.info("  From: Transponder ID {}".format(data['sender_id']))
                    logging.info("  RSSI: {} dBm | Integrity: {}".format(data['rssi'], data['integrity']))
                    logging.info("  Request: {}".format(msg))
                    logging.info("  Waiting for position data...")
                    
                    # Start waiting for position
                    state["waiting_for_position"] = True
                    state["position_data"] = None
                    state["position_event"].clear()
                else:
                    logging.warning("  [WARN] Low integrity request (integrity={})".format(data['integrity']))
            
            elif msg_type == "USBLANGLES":
                if state["waiting_for_position"]:
                    e, n, u = data["east"], data["north"], data["up"]
                    dist = data["distance_3d"]
                    
                    # Check if position is valid (not 0,0,0 which indicates no position fix)
                    if abs(e) < 0.01 and abs(n) < 0.01 and abs(u) < 0.01:
                        logging.warning("  [WARN] Invalid position (0,0,0) - ignoring")
                    else:
                        logging.info("  [POS] Position calculated (from angles):")
                        logging.info("     Azimuth: {:.2f} deg | Elevation: {:.2f} deg | Range: {:.2f}m".format(
                            data['azimuth'], data['elevation'], data['range']))
                        logging.info("     E={:.2f}m, N={:.2f}m, U={:.2f}m, Dist={:.2f}m".format(e, n, u, dist))
                        
                        state["position_data"] = {"e": round(e, 2), "n": round(n, 2), "u": round(u, 2)}
                        state["position_event"].set()
            
            elif msg_type == "USBLLONG":
                if state["waiting_for_position"] and data["valid"]:
                    e, n, u = data["east"], data["north"], data["up"]
                    dist = data["distance_3d"]
                    
                    # Check if position is valid (not 0,0,0 which indicates no position fix)
                    if abs(e) < 0.01 and abs(n) < 0.01 and abs(u) < 0.01:
                        logging.warning("  [WARN] Invalid position (0,0,0) - ignoring")
                    else:
                        logging.info("  [POS] Position calculated (motion-compensated):")
                        logging.info("     E={:.2f}m, N={:.2f}m, U={:.2f}m, Dist={:.2f}m".format(e, n, u, dist))
                        logging.info("     Uncertainty: +/-{:.2f}m".format(data['uncertainty']))
                        
                        state["position_data"] = {"e": round(e, 2), "n": round(n, 2), "u": round(u, 2)}
                        state["position_event"].set()
                elif state["waiting_for_position"] and not data["valid"]:
                    logging.warning("  [WARN] Invalid USBLLONG (high uncertainty)")
                
    except asyncio.CancelledError:
        pass
    except Exception as e:
        logging.error("Response reader error: {}".format(e))


@asyncio.coroutine
def position_responder(writer):
    try:
        while True:
            # Wait for position data to be captured (triggered by USBLANGLES/USBLLONG)
            try:
                yield from asyncio.wait_for(state["position_event"].wait(), timeout=5.0)
                state["position_event"].clear()
                
                if state["position_data"] and state["waiting_for_position"]:
                    state["position_sent_count"] += 1
                    
                    logging.info("  [SEND] Sending position back to transponder...")
                    
                    try:
                        encoded = encode_message(TRANSPONDER_ID, state["position_data"])
                        writer.write(encoded)
                        yield from writer.drain()
                        
                        logging.info("  [OK] Position sent: {}".format(state['position_data']))
                    except ValueError as e:
                        logging.error("  [ERROR] {}".format(e))
                    
                    logging.info("=" * 70)
                    logging.info("")
                    
                    # Reset state
                    state["waiting_for_position"] = False
                    state["position_data"] = None
                    
            except asyncio.TimeoutError:
                # Timeout - reset state if we were waiting
                if state["waiting_for_position"]:
                    logging.warning("  [TIMEOUT] Timeout waiting for valid position data")
                    logging.info("=" * 70)
                    logging.info("")
                    state["waiting_for_position"] = False
                    state["position_data"] = None
            
    except asyncio.CancelledError:
        pass
    except Exception as e:
        logging.error("Position responder error: {}".format(e))


@asyncio.coroutine
def main():
    # Setup logging first
    log_filepath = setup_logging()
    
    logging.info("=" * 70)
    logging.info("USBL RXTX v5 - With File Logging")
    logging.info("=" * 70)
    logging.info("Log file: {}".format(os.path.abspath(log_filepath)))
    logging.info("Started at: {}".format(datetime.now().strftime("%Y-%m-%d %H:%M:%S")))
    logging.info("")
    
    # Initialize the asyncio Event
    state["position_event"] = asyncio.Event()
    
    reader, writer = yield from asyncio.open_connection(USBL_IP, USBL_PORT)
    logging.info("[OK] Connected to USBL at {}:{}".format(USBL_IP, USBL_PORT))
    logging.info("Transceiver ID: {} <- Transponder ID: {}".format(TRANSCEIVER_ID, TRANSPONDER_ID))
    logging.info("")
    
    reader_task = asyncio.ensure_future(response_reader(reader))
    responder_task = asyncio.ensure_future(position_responder(writer))
    
    try:
        yield from asyncio.gather(reader_task, responder_task)
    except KeyboardInterrupt:
        logging.info("\n[STOP] Stopped by user")
    finally:
        reader_task.cancel()
        responder_task.cancel()
        try:
            yield from asyncio.gather(reader_task, responder_task)
        except asyncio.CancelledError:
            pass
        
        writer.close()
        # Note: wait_closed() not available in Python 3.6
        
        # Final summary
        logging.info("")
        logging.info("=" * 70)
        logging.info("FINAL SUMMARY")
        logging.info("=" * 70)
        logging.info("Ended at: {}".format(datetime.now().strftime("%Y-%m-%d %H:%M:%S")))
        logging.info("Position requests received: {}".format(state['request_count']))
        logging.info("Position responses sent: {}".format(state['position_sent_count']))
        logging.info("Log saved to: {}".format(os.path.abspath(log_filepath)))
        logging.info("=" * 70)


if __name__ == '__main__':
    try:
        loop = asyncio.get_event_loop()
        loop.run_until_complete(main())
    except KeyboardInterrupt:
        pass