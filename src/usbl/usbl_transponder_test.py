import asyncio
import json
import logging
import math
from datetime import datetime

# ==================== CONFIGURATION ====================
TRANSCEIVER_ID = 2
TRANSPONDER_ID = 3
USBL_IP = "192.168.0.139"  # Transceiver IP
USBL_PORT = 9200
CODEC = "utf-8"
SPEED_OF_SOUND_MPS = 1500.0

# TX/RX Control
ENABLE_TX = True   # Enable sending messages
ENABLE_RX = True   # Enable receiving messages
MESSAGE_INTERVAL = 3.0  # seconds between messages (if TX enabled)
MIN_INTEGRITY = 50  # Minimum integrity for received messages

# Message to send (if TX enabled) - Keep under 63 bytes!
TX_MESSAGE = {
    "dev": "R",
    "cmd": "ping"
}

# ========================================================

INSTANT_MESSAGE_RESPONSE_KEYWORDS = {
    "SENDSTART", "SENDEND", "RECVSTART", "RECVEND", "DELIVEREDIM", "USBLLONG", 
    "USBLANGLES", "USBLPHYD", "USBLPHYP", "FAILEDIM", "RECVIM", "CANCELEDIM", 
    "ERROR INTERNAL"
}

# Statistics
message_stats = {"sent": 0, "received": 0, "valid_rx": 0, "discarded_rx": 0}
position_stats = {"usbllong": 0, "usblangles": 0, "valid": 0, "invalid": 0, "last_position": None, "last_time": None}


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
        raise ValueError("Message too long (max 63 bytes)")
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
                
                if keyword == "USBLANGLES" and len(args) >= 7:
                    try:
                        target_id = args[3]
                        azimuth = float(args[4])
                        elevation = float(args[5])
                        range_m = float(args[6])
                        east, north, up = angles_to_enu(azimuth, elevation, range_m)
                        distance_3d = calculate_distance_3d(east, north, up)
                        
                        return "USBLANGLES", {
                            "target_id": target_id, "azimuth": azimuth, "elevation": elevation,
                            "range": range_m, "east": east, "north": north, "up": up,
                            "distance_3d": distance_3d, "valid": True
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
    logging.info("📡 Response reader started\n")
    
    try:
        while True:
            line = await reader.readline()
            if not line:
                logging.warning("Connection closed")
                break
            
            msg_type, data = decode_response(line)
            
            if msg_type == "AT*SENDIM_RESPONSE":
                if data.get("status") == "OK":
                    logging.info("✓ Send command accepted")
            
            elif msg_type == "DELIVEREDIM":
                logging.info(f"✓ Message delivered to transponder (ID {data['target_id']})")
            
            elif msg_type == "FAILEDIM":
                logging.warning(f"✗ Message delivery failed")
            
            elif msg_type == "USBLANGLES":
                timestamp = datetime.now()
                position_stats["usblangles"] += 1
                position_stats["valid"] += 1
                position_stats["last_position"] = (data["east"], data["north"], data["up"])
                position_stats["last_time"] = timestamp
                
                logging.info(
                    f"\n{'='*70}\n"
                    f"📍 TRANSPONDER POSITION (from angles) #{position_stats['usblangles']}\n"
                    f"{'='*70}\n"
                    f"  Target ID:  {data['target_id']}\n"
                    f"  Azimuth:    {data['azimuth']:>7.2f}° | Elevation: {data['elevation']:>7.2f}° | Range: {data['range']:>7.2f}m\n"
                    f"  East:       {data['east']:>7.2f} m {'(west)' if data['east'] < 0 else '(east)'}\n"
                    f"  North:      {data['north']:>7.2f} m {'(south)' if data['north'] < 0 else '(north)'}\n"
                    f"  Up:         {data['up']:>7.2f} m {'(below)' if data['up'] < 0 else '(above)'}\n"
                    f"  Distance:   {data['distance_3d']:>7.2f} m (3D)\n"
                    f"{'='*70}"
                )
            
            elif msg_type == "USBLLONG":
                if data["valid"]:
                    timestamp = datetime.now()
                    position_stats["usbllong"] += 1
                    position_stats["valid"] += 1
                    position_stats["last_position"] = (data["east"], data["north"], data["up"])
                    position_stats["last_time"] = timestamp
                    
                    logging.info(
                        f"\n{'='*70}\n"
                        f"📍 TRANSPONDER POSITION (motion-compensated) #{position_stats['usbllong']}\n"
                        f"{'='*70}\n"
                        f"  Target ID: {data['target_id']}\n"
                        f"  East:  {data['east']:>7.2f} m | North: {data['north']:>7.2f} m | Up: {data['up']:>7.2f} m\n"
                        f"  Distance: {data['distance_3d']:>7.2f} m | Uncertainty: ±{data['uncertainty']:.2f} m\n"
                        f"{'='*70}"
                    )
                else:
                    position_stats["invalid"] += 1
                    logging.warning(f"📍 Invalid position fix (uncertainty too high)")
            
            elif msg_type == "RECVIM" and ENABLE_RX:
                message_stats["received"] += 1
                
                if data["valid"]:
                    message_stats["valid_rx"] += 1
                    logging.info(
                        f"\n{'='*70}\n"
                        f"📨 RECEIVED MESSAGE #{message_stats['received']}\n"
                        f"{'='*70}\n"
                        f"  From:      Transponder ID {data['sender_id']}\n"
                        f"  RSSI:      {data['rssi']} dBm\n"
                        f"  Integrity: {data['integrity']} (threshold: >{MIN_INTEGRITY}) ✓\n"
                        f"  Content:   {data['message']}\n"
                        f"{'='*70}"
                    )
                else:
                    message_stats["discarded_rx"] += 1
                    logging.warning(
                        f"📨 Message #{message_stats['received']} discarded (integrity {data['integrity']} too low)"
                    )
                
    except asyncio.CancelledError:
        pass
    except Exception as e:
        logging.error(f"Response reader error: {e}")


async def message_sender(writer: asyncio.StreamWriter):
    if not ENABLE_TX:
        logging.info("📤 TX disabled - not sending messages")
        while True:
            await asyncio.sleep(3600)  # Sleep indefinitely
        return
    
    logging.info(f"📤 TX enabled - sending messages every {MESSAGE_INTERVAL}s\n")
    
    try:
        while True:
            await asyncio.sleep(MESSAGE_INTERVAL)
            
            message_stats["sent"] += 1
            msg = TX_MESSAGE.copy()
            msg["c"] = message_stats["sent"]  # Just add count
            
            encoded = encode_message(TRANSPONDER_ID, msg)
            writer.write(encoded)
            await writer.drain()
            
            logging.info(f"→ Sent message #{message_stats['sent']}: {msg}")
            
    except asyncio.CancelledError:
        pass
    except Exception as e:
        logging.error(f"Sender error: {e}")


async def main():
    reader, writer = await asyncio.open_connection(USBL_IP, USBL_PORT)
    logging.info(f"✓ Connected to USBL at {USBL_IP}:{USBL_PORT}")
    logging.info(f"📋 Transceiver ID: {TRANSCEIVER_ID} | Target: Transponder ID {TRANSPONDER_ID}")
    logging.info(f"📊 TX: {'ON' if ENABLE_TX else 'OFF'} | RX: {'ON' if ENABLE_RX else 'OFF'}\n")
    
    reader_task = asyncio.create_task(response_reader(reader))
    sender_task = asyncio.create_task(message_sender(writer))
    
    try:
        await asyncio.gather(reader_task, sender_task)
    except KeyboardInterrupt:
        logging.info("\n⏹️  Stopped by user")
    finally:
        reader_task.cancel()
        sender_task.cancel()
        try:
            await asyncio.gather(reader_task, sender_task)
        except asyncio.CancelledError:
            pass
        
        writer.close()
        await writer.wait_closed()
        
        # Final summary
        logging.info("\n" + "="*70)
        logging.info("FINAL SUMMARY")
        logging.info("="*70)
        if ENABLE_TX:
            logging.info(f"Messages sent: {message_stats['sent']}")
        if ENABLE_RX:
            logging.info(f"Messages received: {message_stats['received']} ({message_stats['valid_rx']} valid, {message_stats['discarded_rx']} discarded)")
        if position_stats["valid"] > 0:
            logging.info(f"Position fixes: {position_stats['valid']} valid ({position_stats['usbllong']} USBLLONG, {position_stats['usblangles']} USBLANGLES)")
            if position_stats["last_position"]:
                e, n, u = position_stats["last_position"]
                logging.info(f"Last position: E={e:.2f}m, N={n:.2f}m, U={u:.2f}m")
        logging.info("="*70)


if __name__ == '__main__':
    logging.basicConfig(level=logging.INFO, format="%(asctime)s | %(levelname)s | %(message)s")
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass