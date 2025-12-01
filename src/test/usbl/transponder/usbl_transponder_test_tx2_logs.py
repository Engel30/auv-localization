import asyncio
import json
import logging
import math
import os
from datetime import datetime
from typing import Tuple, Dict, Any

# ==================== CONFIGURATION ====================
TRANSPONDER_ID = 3
TRANSCEIVER_ID = 2
USBL_IP = "192.168.0.232"
USBL_PORT = 9200
CODEC = "utf-8"
SPEED_OF_SOUND_MPS = 1500.0
MIN_INTEGRITY = 50

# REQUEST FREQUENCY - Time to wait after receiving position before next request
REQUEST_INTERVAL = 2.0  # seconds

# LOGGING
LOG_DIR = "logs"
LOG_FILENAME = "usbl_transponder_{}.log".format(datetime.now().strftime('%Y%m%d_%H%M%S'))

# ========================================================

INSTANT_MESSAGE_RESPONSE_KEYWORDS = {
    "SENDSTART", "SENDEND", "RECVSTART", "RECVEND", "DELIVEREDIM", "USBLLONG", 
    "USBLANGLES", "USBLPHYD", "USBLPHYP", "FAILEDIM", "RECVIM", "CANCELEDIM", 
    "ERROR INTERNAL"
}

# State
state = {
    "waiting_for_response": False,
    "request_count": 0,
    "position_count": 0,
    "last_position": None,
    "response_event": None  # Will be initialized in main()
}


def setup_logging():
    # type: () -> str
    """Setup logging to both console and file. Returns log file path."""
    if not os.path.exists(LOG_DIR):
        os.makedirs(LOG_DIR)
    
    log_path = os.path.join(LOG_DIR, LOG_FILENAME)
    
    # Create logger
    logger = logging.getLogger()
    logger.setLevel(logging.DEBUG)
    
    # Clear any existing handlers
    logger.handlers = []
    
    # Console handler (INFO level)
    console_handler = logging.StreamHandler()
    console_handler.setLevel(logging.INFO)
    console_handler.setFormatter(logging.Formatter("%(message)s"))
    
    # File handler (DEBUG level for more detail)
    file_handler = logging.FileHandler(log_path)
    file_handler.setLevel(logging.DEBUG)
    file_handler.setFormatter(logging.Formatter("%(asctime)s | %(levelname)s | %(message)s"))
    
    logger.addHandler(console_handler)
    logger.addHandler(file_handler)
    
    return log_path


def calculate_distance_3d(east, north, up):
    return math.sqrt(east**2 + north**2 + up**2)


def encode_message(target_id, payload):
    # type: (int, Dict[str, Any]) -> bytes
    payload_str = json.dumps(payload, separators=(',', ':'))
    if len(payload_str) > 63:
        raise ValueError("Message too long: {} bytes".format(len(payload_str)))
    payload_str = "[{}".format(payload_str)
    command = "AT*SENDIM,p0,{},{},ack,{}".format(len(payload_str), target_id, payload_str)
    return "+++{}\n".format(command).encode(CODEC)


def decode_response(message):
    # type: (bytes) -> Tuple[str, Dict[str, Any]]
    try:
        str_msg = message.decode(CODEC).strip()
        if not str_msg or not str_msg.startswith("+++"):
            return "UNKNOWN", {}
        
        parts = str_msg.split(":")
        
        if "AT" in str_msg:
            if len(parts) >= 3:
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
                        if ',[' in str_msg:
                            _, payload = str_msg.rsplit(',[', 1)
                            payload = payload.strip()
                            if payload.startswith("{"):
                                try:
                                    payload = json.loads(payload)
                                except:
                                    pass
                        
                        return "RECVIM", {
                            "sender_id": sender_id, "integrity": integrity,
                            "valid": integrity > MIN_INTEGRITY, "rssi": rssi, "message": payload
                        }
                    except (ValueError, IndexError) as e:
                        logging.error("Error parsing RECVIM: {}".format(e))
                        return "RECVIM_ERROR", {}
                
                elif keyword in INSTANT_MESSAGE_RESPONSE_KEYWORDS:
                    return keyword, {"args": args}
        
        else:
            cmd = parts[0].replace("+", "")
            if len(parts) >= 3:
                return "{}_RESPONSE".format(cmd), {"status": parts[2]}
        
        return "OTHER", {}
        
    except Exception as e:
        logging.error("Parse error: {}".format(e))
        return "ERROR", {}


async def response_reader(reader):
    logging.info("=" * 70)
    logging.info("TRANSPONDER STARTED - Event-Driven Position Request")
    logging.info("=" * 70)
    logging.info("")
    
    try:
        while True:
            line = await reader.readline()
            if not line:
                logging.warning("Connection closed")
                break
            
            msg_type, data = decode_response(line)
            
            if msg_type == "AT*SENDIM_RESPONSE":
                if data.get("status") == "OK":
                    logging.debug("  [OK] USBL accepted send command")
            
            elif msg_type == "DELIVEREDIM":
                logging.info("  [OK] Position request delivered to transceiver")
            
            elif msg_type == "FAILEDIM":
                logging.error("  [FAIL] Position request FAILED - will retry")
                state["waiting_for_response"] = False
                state["response_event"].set()
            
            elif msg_type == "RECVIM":
                if data["valid"]:
                    msg = data["message"]
                    
                    if msg and "e" in msg and "n" in msg and "u" in msg:
                        state["position_count"] += 1
                        e, n, u = msg["e"], msg["n"], msg["u"]
                        dist = calculate_distance_3d(e, n, u)
                        state["last_position"] = (e, n, u, dist)
                        
                        logging.info("")
                        logging.info("=" * 70)
                        logging.info("POSITION RECEIVED #{}".format(state['position_count']))
                        logging.info("=" * 70)
                        logging.info("  East:     {:>8.2f} m {}".format(e, '(west)' if e < 0 else '(east)'))
                        logging.info("  North:    {:>8.2f} m {}".format(n, '(south)' if n < 0 else '(north)'))
                        logging.info("  Up:       {:>8.2f} m {}".format(u, '(below)' if u < 0 else '(above)'))
                        logging.info("  Distance: {:>8.2f} m (3D)".format(dist))
                        logging.info("  RSSI:     {} dBm | Integrity: {}".format(data['rssi'], data['integrity']))
                        logging.info("=" * 70)
                        
                        # Signal that response was received
                        state["waiting_for_response"] = False
                        state["response_event"].set()
                    else:
                        logging.warning("  [WARN] Received message without position data: {}".format(msg))
                else:
                    logging.warning("  [WARN] Low integrity message (integrity={})".format(data['integrity']))
                
    except asyncio.CancelledError:
        pass
    except Exception as e:
        logging.error("Response reader error: {}".format(e))


async def position_requester(writer):
    logging.info("Starting position request loop...")
    logging.info("Request interval: {}s after each position received".format(REQUEST_INTERVAL))
    logging.info("")
    
    # Initial delay
    await asyncio.sleep(1)
    
    try:
        while True:
            # Send position request
            state["request_count"] += 1
            state["waiting_for_response"] = True
            state["response_event"].clear()
            
            request_msg = {
                "r": state["request_count"]  # request ID
            }
            
            logging.info("--> [{}] Sending position request #{}".format(
                datetime.now().strftime('%H:%M:%S'), state['request_count']))
            
            try:
                encoded = encode_message(TRANSCEIVER_ID, request_msg)
                writer.write(encoded)
                await writer.drain()
            except ValueError as e:
                logging.error("[ERROR] {}".format(e))
                continue
            
            # Wait for response (with timeout)
            try:
                await asyncio.wait_for(state["response_event"].wait(), timeout=10.0)
                logging.info("  <-- Response received in time")
            except asyncio.TimeoutError:
                logging.warning("  [TIMEOUT] Waiting for position response")
                state["waiting_for_response"] = False
            
            # Wait before next request
            logging.info("  ... Waiting {}s before next request...".format(REQUEST_INTERVAL))
            logging.info("")
            await asyncio.sleep(REQUEST_INTERVAL)
            
    except asyncio.CancelledError:
        pass
    except Exception as e:
        logging.error("Position requester error: {}".format(e))


async def main():
    # Initialize event (required for Python 3.6)
    state["response_event"] = asyncio.Event()
    
    reader, writer = await asyncio.open_connection(USBL_IP, USBL_PORT)
    logging.info("[OK] Connected to USBL at {}:{}".format(USBL_IP, USBL_PORT))
    logging.info("Transponder ID: {} --> Transceiver ID: {}".format(TRANSPONDER_ID, TRANSCEIVER_ID))
    logging.info("")
    
    reader_task = asyncio.ensure_future(response_reader(reader))
    requester_task = asyncio.ensure_future(position_requester(writer))
    
    try:
        await asyncio.gather(reader_task, requester_task)
    except KeyboardInterrupt:
        logging.info("\nStopped by user")
    finally:
        reader_task.cancel()
        requester_task.cancel()
        try:
            await asyncio.gather(reader_task, requester_task)
        except asyncio.CancelledError:
            pass
        
        writer.close()
        await writer.wait_closed()
        
        # Final summary
        logging.info("")
        logging.info("=" * 70)
        logging.info("FINAL SUMMARY")
        logging.info("=" * 70)
        logging.info("Position requests sent: {}".format(state['request_count']))
        logging.info("Position responses received: {}".format(state['position_count']))
        if state["last_position"]:
            e, n, u, dist = state["last_position"]
            logging.info("Last position: E={:.2f}m, N={:.2f}m, U={:.2f}m, Dist={:.2f}m".format(e, n, u, dist))
        logging.info("=" * 70)


if __name__ == '__main__':
    log_file = setup_logging()
    logging.info("Logging to: {}".format(log_file))
    
    # Python 3.6 compatible event loop
    loop = asyncio.get_event_loop()
    try:
        loop.run_until_complete(main())
    except KeyboardInterrupt:
        pass
    finally:
        loop.close()