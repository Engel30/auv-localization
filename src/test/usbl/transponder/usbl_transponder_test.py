import asyncio
import json
import logging
import math
from datetime import datetime

# ==================== CONFIGURATION ====================
TRANSPONDER_ID = 3
TRANSCEIVER_ID = 2
USBL_IP = "192.168.0.232"
USBL_PORT = 9200
CODEC = "utf-8"
SPEED_OF_SOUND_MPS = 1500.0
MIN_INTEGRITY = 50

# REQUEST FREQUENCY - Time to wait after receiving position before next request
REQUEST_INTERVAL = 1.0  # seconds

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
    "response_event": asyncio.Event()
}


def calculate_distance_3d(east, north, up):
    return math.sqrt(east**2 + north**2 + up**2)


def encode_message(target_id: int, payload: dict) -> bytes:
    payload_str = json.dumps(payload, separators=(',', ':'))
    if len(payload_str) > 63:
        raise ValueError(f"Message too long: {len(payload_str)} bytes")
    payload_str = f"[{payload_str}"
    command = f"AT*SENDIM,p0,{len(payload_str)},{target_id},ack,{payload_str}"
    return f"+++{command}\n".encode(CODEC)


def decode_response(message: bytes) -> tuple[str, dict]:
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
                        logging.error(f"Error parsing RECVIM: {e}")
                        return "RECVIM_ERROR", {}
                
                elif keyword in INSTANT_MESSAGE_RESPONSE_KEYWORDS:
                    return keyword, {"args": args}
        
        else:
            cmd = parts[0].replace("+", "")
            if len(parts) >= 3:
                return f"{cmd}_RESPONSE", {"status": parts[2]}
        
        return "OTHER", {}
        
    except Exception as e:
        logging.error(f"Parse error: {e}")
        return "ERROR", {}


async def response_reader(reader: asyncio.StreamReader):
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
                    logging.debug("  ✓ USBL accepted send command")
            
            elif msg_type == "DELIVEREDIM":
                logging.info(f"  ✓ Position request delivered to transceiver")
            
            elif msg_type == "FAILEDIM":
                logging.error(f"  ✗ Position request FAILED - will retry")
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
                        logging.info(f"📍 POSITION RECEIVED #{state['position_count']}")
                        logging.info("=" * 70)
                        logging.info(f"  East:     {e:>8.2f} m {'(west)' if e < 0 else '(east)'}")
                        logging.info(f"  North:    {n:>8.2f} m {'(south)' if n < 0 else '(north)'}")
                        logging.info(f"  Up:       {u:>8.2f} m {'(below)' if u < 0 else '(above)'}")
                        logging.info(f"  Distance: {dist:>8.2f} m (3D)")
                        logging.info(f"  RSSI:     {data['rssi']} dBm | Integrity: {data['integrity']}")
                        logging.info("=" * 70)
                        
                        # Signal that response was received
                        state["waiting_for_response"] = False
                        state["response_event"].set()
                    else:
                        logging.warning(f"  ⚠️  Received message without position data: {msg}")
                else:
                    logging.warning(f"  ⚠️  Low integrity message (integrity={data['integrity']})")
                
    except asyncio.CancelledError:
        pass
    except Exception as e:
        logging.error(f"Response reader error: {e}")


async def position_requester(writer: asyncio.StreamWriter):
    logging.info("Starting position request loop...")
    logging.info(f"Request interval: {REQUEST_INTERVAL}s after each position received")
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
            
            logging.info(f"→ [{datetime.now().strftime('%H:%M:%S')}] Sending position request #{state['request_count']}")
            
            try:
                encoded = encode_message(TRANSCEIVER_ID, request_msg)
                writer.write(encoded)
                await writer.drain()
            except ValueError as e:
                logging.error(f"❌ {e}")
                continue
            
            # Wait for response (with timeout)
            try:
                await asyncio.wait_for(state["response_event"].wait(), timeout=10.0)
                logging.info(f"  ← Response received in time")
            except asyncio.TimeoutError:
                logging.warning(f"  ⏱️  TIMEOUT waiting for position response")
                state["waiting_for_response"] = False
            
            # Wait before next request
            logging.info(f"  ⏳ Waiting {REQUEST_INTERVAL}s before next request...")
            logging.info("")
            await asyncio.sleep(REQUEST_INTERVAL)
            
    except asyncio.CancelledError:
        pass
    except Exception as e:
        logging.error(f"Position requester error: {e}")


async def main():
    reader, writer = await asyncio.open_connection(USBL_IP, USBL_PORT)
    logging.info(f"✓ Connected to USBL at {USBL_IP}:{USBL_PORT}")
    logging.info(f"📋 Transponder ID: {TRANSPONDER_ID} → Transceiver ID: {TRANSCEIVER_ID}")
    logging.info("")
    
    reader_task = asyncio.create_task(response_reader(reader))
    requester_task = asyncio.create_task(position_requester(writer))
    
    try:
        await asyncio.gather(reader_task, requester_task)
    except KeyboardInterrupt:
        logging.info("\n⏹️  Stopped by user")
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
        logging.info(f"Position requests sent: {state['request_count']}")
        logging.info(f"Position responses received: {state['position_count']}")
        if state["last_position"]:
            e, n, u, dist = state["last_position"]
            logging.info(f"Last position: E={e:.2f}m, N={n:.2f}m, U={u:.2f}m, Dist={dist:.2f}m")
        logging.info("=" * 70)


if __name__ == '__main__':
    logging.basicConfig(level=logging.INFO, format="%(message)s")
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass