import asyncio
import json
import logging
import math
from datetime import datetime

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
    "position_event": asyncio.Event(),
    "waiting_for_propagation_time": False,
    "pending_angles_data": None,
    "propagation_time_event": asyncio.Event()
}


def calculate_distance_3d(east, north, up):
    return math.sqrt(east**2 + north**2 + up**2)


def angles_to_enu(azimuth_deg, elevation_deg, range_m):
    azimuth_rad = math.radians(azimuth_deg)
    elevation_rad = math.radians(elevation_deg)
    horizontal_range = range_m * math.cos(elevation_rad)
    east = horizontal_range * math.sin(azimuth_rad)
    north = horizontal_range * math.cos(azimuth_rad)
    up = range_m * math.sin(elevation_rad)
    return east, north, up


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
                
                if keyword == "USBLANGLES" and len(args) >= 14:
                    try:
                        target_id = args[3]
                        lbearing_rad = float(args[4])  # local bearing in radians
                        lelevation_rad = float(args[5])  # local elevation in radians
                        bearing_rad = float(args[6])  # motion-compensated bearing in radians
                        elevation_rad = float(args[7])  # motion-compensated elevation in radians
                        rssi = int(args[11])
                        integrity = int(args[12])
                        
                        # USBLANGLES does NOT contain range/distance
                        # We need to query AT?T to get propagation time and estimate distance
                        return "USBLANGLES", {
                            "target_id": target_id,
                            "lbearing_rad": lbearing_rad,
                            "lelevation_rad": lelevation_rad,
                            "bearing_rad": bearing_rad,
                            "elevation_rad": elevation_rad,
                            "bearing_deg": math.degrees(bearing_rad),
                            "elevation_deg": math.degrees(elevation_rad),
                            "rssi": rssi,
                            "integrity": integrity,
                            "valid": True
                        }
                    except (ValueError, IndexError) as e:
                        logging.error(f"Error parsing USBLANGLES: {e}")
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
                        
                        return "USBLLONG", {
                            "target_id": target_id, "east": east, "north": north, "up": up,
                            "range": slant_range_m, "distance_3d": distance_3d,
                            "uncertainty": position_uncertainty_m,
                            "valid": position_uncertainty_m <= (0.95 * slant_range_m)
                        }
                    except (ValueError, IndexError) as e:
                        logging.error(f"Error parsing USBLLONG: {e}")
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
                        
                        return "RECVIM", {
                            "sender_id": sender_id, "integrity": integrity,
                            "valid": integrity > MIN_INTEGRITY, "rssi": rssi, "message": payload
                        }
                    except (ValueError, IndexError) as e:
                        logging.error(f"Error parsing RECVIM: {e}")
                        return "RECVIM_ERROR", {}
                
                elif "AT?T" in str_msg:
                    # Format: +++AT?T:<length>:<propagation_time_us>
                    # parts[2] contains the propagation time value
                    try:
                        propagation_time_us = int(parts[2])
                        return "PROPAGATION_TIME", {"time_us": propagation_time_us}
                    except (ValueError, IndexError) as e:
                        logging.error(f"Error parsing AT?T response: {e}")
                        return "PROPAGATION_TIME_ERROR", {}
                
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


async def response_reader(reader: asyncio.StreamReader, writer: asyncio.StreamWriter):
    logging.info("=" * 70)
    logging.info("TRANSCEIVER STARTED - Event-Driven Position Response")
    logging.info("=" * 70)
    logging.info("Listening for position requests...")
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
                logging.info(f"  ✓ Position response delivered to transponder")
            
            elif msg_type == "FAILEDIM":
                logging.error(f"  ✗ Position response FAILED")
            
            elif msg_type == "RECVIM":
                if data["valid"]:
                    msg = data["message"]
                    state["request_count"] += 1
                    
                    logging.info("")
                    logging.info("=" * 70)
                    logging.info(f"📨 POSITION REQUEST RECEIVED #{state['request_count']}")
                    logging.info("=" * 70)
                    logging.info(f"  From: Transponder ID {data['sender_id']}")
                    logging.info(f"  RSSI: {data['rssi']} dBm | Integrity: {data['integrity']}")
                    logging.info(f"  Request: {msg}")
                    logging.info(f"  ⏳ Waiting for position data...")
                    
                    # Start waiting for position
                    state["waiting_for_position"] = True
                    state["position_data"] = None
                    state["position_event"].clear()
                else:
                    logging.warning(f"  ⚠️  Low integrity request (integrity={data['integrity']})")
            
            elif msg_type == "USBLANGLES":
                if state["waiting_for_position"]:
                    logging.info(f"  📐 Angles received:")
                    logging.info(f"     Bearing: {data['bearing_deg']:.2f}° | Elevation: {data['elevation_deg']:.2f}°")
                    logging.info(f"     RSSI: {data['rssi']} dBm | Integrity: {data['integrity']}")
                    logging.info(f"  📏 Requesting propagation time to estimate distance...")
                    
                    # Save angles data and request propagation time
                    state["pending_angles_data"] = data
                    state["waiting_for_propagation_time"] = True
                    state["propagation_time_event"].clear()
                    
                    # Send AT?T command to get propagation time
                    at_command = "+++AT?T\n".encode(CODEC)
                    writer.write(at_command)
                    await writer.drain()
                    logging.debug("  → Sent AT?T command")
            
            elif msg_type == "PROPAGATION_TIME":
                if state["waiting_for_propagation_time"] and state["pending_angles_data"]:
                    time_us = data["time_us"]
                    # Calculate distance from propagation time
                    # distance = (time_us / 1_000_000) * speed_of_sound
                    estimated_distance_m = (time_us / 1_000_000.0) * SPEED_OF_SOUND_MPS
                    
                    logging.info(f"  ⏱️  Propagation time: {time_us} µs")
                    logging.info(f"  📏 Estimated distance: {estimated_distance_m:.2f}m")
                    
                    # Now calculate ENU coordinates
                    angles_data = state["pending_angles_data"]
                    bearing_rad = angles_data["bearing_rad"]
                    elevation_rad = angles_data["elevation_rad"]
                    
                    # Use angles_to_enu with degrees (convert from radians)
                    e, n, u = angles_to_enu(
                        math.degrees(bearing_rad),
                        math.degrees(elevation_rad),
                        estimated_distance_m
                    )
                    dist = calculate_distance_3d(e, n, u)
                    
                    logging.info(f"  📍 Position calculated (from angles + AT?T):")
                    logging.info(f"     E={e:.2f}m, N={n:.2f}m, U={u:.2f}m, Dist={dist:.2f}m")
                    
                    state["position_data"] = {"e": round(e, 2), "n": round(n, 2), "u": round(u, 2)}
                    state["position_event"].set()
                    
                    # Reset state
                    state["waiting_for_propagation_time"] = False
                    state["pending_angles_data"] = None
            
            elif msg_type == "USBLLONG":
                if state["waiting_for_position"] and data["valid"]:
                    e, n, u = data["east"], data["north"], data["up"]
                    dist = data["distance_3d"]
                    
                    # Check if position is valid (not 0,0,0 which indicates no position fix)
                    if abs(e) < 0.01 and abs(n) < 0.01 and abs(u) < 0.01:
                        logging.warning(f"  ⚠️  Invalid position (0,0,0) - ignoring")
                    else:
                        logging.info(f"  📍 Position calculated (motion-compensated):")
                        logging.info(f"     E={e:.2f}m, N={n:.2f}m, U={u:.2f}m, Dist={dist:.2f}m")
                        logging.info(f"     Uncertainty: ±{data['uncertainty']:.2f}m")
                        
                        state["position_data"] = {"e": round(e, 2), "n": round(n, 2), "u": round(u, 2)}
                        state["position_event"].set()
                elif state["waiting_for_position"] and not data["valid"]:
                    logging.warning(f"  ⚠️  Invalid USBLLONG (high uncertainty)")
                
    except asyncio.CancelledError:
        pass
    except Exception as e:
        logging.error(f"Response reader error: {e}")


async def position_responder(writer: asyncio.StreamWriter):
    try:
        while True:
            # Wait for position data to be captured (triggered by USBLANGLES/USBLLONG)
            try:
                await asyncio.wait_for(state["position_event"].wait(), timeout=5.0)
                state["position_event"].clear()
                
                if state["position_data"] and state["waiting_for_position"]:
                    state["position_sent_count"] += 1
                    
                    logging.info(f"  → Sending position back to transponder...")
                    
                    try:
                        encoded = encode_message(TRANSPONDER_ID, state["position_data"])
                        writer.write(encoded)
                        await writer.drain()
                        
                        logging.info(f"  ✓ Position sent: {state['position_data']}")
                    except ValueError as e:
                        logging.error(f"  ❌ {e}")
                    
                    logging.info("=" * 70)
                    logging.info("")
                    
                    # Reset state
                    state["waiting_for_position"] = False
                    state["position_data"] = None
                    
            except asyncio.TimeoutError:
                # Timeout - reset state if we were waiting
                if state["waiting_for_position"]:
                    logging.warning(f"  ⏱️  Timeout waiting for valid position data")
                    logging.info("=" * 70)
                    logging.info("")
                    state["waiting_for_position"] = False
                    state["position_data"] = None
                    state["waiting_for_propagation_time"] = False
                    state["pending_angles_data"] = None
            
    except asyncio.CancelledError:
        pass
    except Exception as e:
        logging.error(f"Position responder error: {e}")


async def main():
    reader, writer = await asyncio.open_connection(USBL_IP, USBL_PORT)
    logging.info(f"✓ Connected to USBL at {USBL_IP}:{USBL_PORT}")
    logging.info(f"📋 Transceiver ID: {TRANSCEIVER_ID} ← Transponder ID: {TRANSPONDER_ID}")
    logging.info("")
    
    reader_task = asyncio.create_task(response_reader(reader, writer))
    responder_task = asyncio.create_task(position_responder(writer))
    
    try:
        await asyncio.gather(reader_task, responder_task)
    except KeyboardInterrupt:
        logging.info("\n⏹️  Stopped by user")
    finally:
        reader_task.cancel()
        responder_task.cancel()
        try:
            await asyncio.gather(reader_task, responder_task)
        except asyncio.CancelledError:
            pass
        
        writer.close()
        await writer.wait_closed()
        
        # Final summary
        logging.info("")
        logging.info("=" * 70)
        logging.info("FINAL SUMMARY")
        logging.info("=" * 70)
        logging.info(f"Position requests received: {state['request_count']}")
        logging.info(f"Position responses sent: {state['position_sent_count']}")
        logging.info("=" * 70)


if __name__ == '__main__':
    logging.basicConfig(level=logging.INFO, format="%(message)s")
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass