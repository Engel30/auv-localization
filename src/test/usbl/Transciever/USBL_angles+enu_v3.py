import asyncio
import json
import logging
import math
from datetime import datetime

# Configuration
TRANSCEIVER_ID = 2
TRANSPONDER_ID = 3
USBL_IP = "192.168.0.139"  # Transceiver IP
USBL_PORT = 9200
CODEC = "utf-8"
SPEED_OF_SOUND_MPS = 1500.0  # Sea water
MIN_INTEGRITY = 50  # Lowered to accept more messages

INSTANT_MESSAGE_RESPONSE_KEYWORDS = {
    "SENDSTART", "SENDEND", "RECVSTART", "RECVEND", "DELIVEREDIM", "USBLLONG", 
    "USBLANGLES", "USBLPHYD", "USBLPHYP", "FAILEDIM", "RECVIM", "CANCELEDIM", 
    "ERROR INTERNAL"
}

# Tracking
message_stats = {
    "received": 0,
    "valid": 0,
    "discarded": 0
}

position_stats = {
    "usbllong_fixes": 0,
    "usblangles_fixes": 0,
    "valid_fixes": 0,
    "invalid_fixes": 0,
    "last_valid_position": None,
    "last_valid_time": None
}


def calculate_distance_3d(east, north, up):
    """Calculate 3D distance from origin"""
    return math.sqrt(east**2 + north**2 + up**2)


def angles_to_enu(azimuth_deg, elevation_deg, range_m):
    """
    Convert azimuth, elevation, and range to ENU coordinates
    
    Azimuth: 0° = North, 90° = East, -90° = West, 180° = South
    Elevation: positive = above, negative = below
    """
    azimuth_rad = math.radians(azimuth_deg)
    elevation_rad = math.radians(elevation_deg)
    
    # Project range onto horizontal plane
    horizontal_range = range_m * math.cos(elevation_rad)
    
    # Calculate ENU
    east = horizontal_range * math.sin(azimuth_rad)
    north = horizontal_range * math.cos(azimuth_rad)
    up = range_m * math.sin(elevation_rad)
    
    return east, north, up


def decode_response(message: bytes) -> tuple[str, dict]:
    """Parse USBL response"""
    try:
        str_msg = message.decode(CODEC).strip()
        
        if not str_msg or not str_msg.startswith("+++"):
            return "UNKNOWN", {}
        
        parts = str_msg.split(":")
        
        if "AT" in str_msg:
            # Async response
            if len(parts) >= 3:
                args = parts[2].split(",")
                keyword = args[0]
                
                if keyword == "USBLANGLES":
                    # USBLANGLES format:
                    # +++AT:XX:USBLANGLES,timestamp1,timestamp2,target_id,azimuth,elevation,range,...
                    
                    if len(args) < 7:
                        logging.warning(f"USBLANGLES has only {len(args)} args, expected at least 7")
                        return "USBLANGLES_INVALID", {"raw": str_msg, "arg_count": len(args)}
                    
                    try:
                        target_id = args[3]
                        azimuth = float(args[4])  # degrees
                        elevation = float(args[5])  # degrees
                        range_m = float(args[6])  # meters
                        
                        # Convert angles to ENU coordinates
                        east, north, up = angles_to_enu(azimuth, elevation, range_m)
                        distance_3d = calculate_distance_3d(east, north, up)
                        
                        return "USBLANGLES", {
                            "target_id": target_id,
                            "azimuth": azimuth,
                            "elevation": elevation,
                            "range": range_m,
                            "east": east,
                            "north": north,
                            "up": up,
                            "distance_3d": distance_3d,
                            "valid": True
                        }
                        
                    except (ValueError, IndexError) as e:
                        logging.error(f"Error parsing USBLANGLES values: {e}")
                        logging.error(f"Args: {args}")
                        return "USBLANGLES_PARSE_ERROR", {"error": str(e), "args": args}
                
                elif keyword == "USBLLONG" and len(args) == 17:
                    target_id = args[3]
                    east = float(args[7])
                    north = float(args[8])
                    up = float(args[9])
                    propagation_time_us = int(args[13])
                    position_uncertainty_m = float(args[16])
                    
                    slant_range_m = (propagation_time_us / 1_000_000.0) * SPEED_OF_SOUND_MPS
                    distance_3d = calculate_distance_3d(east, north, up)
                    
                    return "USBLLONG", {
                        "target_id": target_id,
                        "east": east,
                        "north": north,
                        "up": up,
                        "range": slant_range_m,
                        "distance_3d": distance_3d,
                        "uncertainty": position_uncertainty_m,
                        "valid": position_uncertainty_m <= (0.95 * slant_range_m)
                    }
                
                elif keyword == "DELIVEREDIM":
                    return "DELIVEREDIM", {"target_id": args[1] if len(args) > 1 else "?"}
                
                elif keyword == "FAILEDIM":
                    return "FAILEDIM", {"target_id": args[1] if len(args) > 1 else "?"}
                
                elif keyword == "RECVIM":
                    # RECVIM format:
                    # +++AT:XX:RECVIM,p0,length,sender_id,receiver_id,ack/noack,timestamp,rssi,integrity,accuracy,[payload
                    
                    if len(args) < 9:
                        logging.error(f"RECVIM has {len(args)} args, expected at least 9")
                        return "RECVIM_INVALID", {"raw": str_msg, "arg_count": len(args)}
                    
                    try:
                        sender_id = args[3]
                        rssi = int(args[7])  # RSSI is at index 7
                        integrity = int(args[8])  # Integrity is at index 8
                        
                        # Extract payload
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
                            "sender_id": sender_id,
                            "integrity": integrity,
                            "valid": integrity > MIN_INTEGRITY,
                            "rssi": rssi,
                            "message": payload
                        }
                    except (ValueError, IndexError) as e:
                        logging.error(f"Error parsing RECVIM: {e}")
                        logging.error(f"Args: {args}")
                        return "RECVIM_ERROR", {"raw": str_msg}
                
                elif keyword in INSTANT_MESSAGE_RESPONSE_KEYWORDS:
                    return keyword, {"args": args}
        
        else:
            # Direct command response
            cmd = parts[0].replace("+", "")
            if len(parts) >= 3:
                status = parts[2]
                return f"{cmd}_RESPONSE", {"status": status}
        
        return "OTHER", {"raw": str_msg}
        
    except Exception as e:
        logging.error(f"Parse error: {e}")
        return "ERROR", {"error": str(e), "raw": message}


async def response_reader(reader: asyncio.StreamReader):
    """Continuously read and parse USBL responses"""
    logging.info("🎧 Transceiver listening (handles USBLLONG and USBLANGLES)...\n")
    
    try:
        while True:
            line = await reader.readline()
            
            if not line:
                logging.warning("Connection closed by USBL")
                break
            
            msg_type, data = decode_response(line)
            
            if msg_type == "RECVSTART":
                logging.debug("📡 Starting to receive transmission...")
            
            elif msg_type == "RECVEND":
                logging.debug(f"📡 Transmission complete")
            
            elif msg_type == "USBLANGLES":
                timestamp = datetime.now()
                position_stats["usblangles_fixes"] += 1
                position_stats["valid_fixes"] += 1
                position_stats["last_valid_position"] = (data["east"], data["north"], data["up"])
                position_stats["last_valid_time"] = timestamp
                
                logging.info(
                    f"\n{'='*70}\n"
                    f"📍 TRANSPONDER POSITION (from ANGLES) #{position_stats['usblangles_fixes']}\n"
                    f"{'='*70}\n"
                    f"  Target ID: {data['target_id']}\n"
                    f"  Polar Coordinates:\n"
                    f"    Azimuth:    {data['azimuth']:>7.2f}° (0°=N, 90°=E, -90°=W, ±180°=S)\n"
                    f"    Elevation:  {data['elevation']:>7.2f}° (positive=above, negative=below)\n"
                    f"    Range:      {data['range']:>7.2f} m\n"
                    f"  Cartesian (ENU) Coordinates:\n"
                    f"    East:       {data['east']:>7.2f} m {'(west)' if data['east'] < 0 else '(east)'}\n"
                    f"    North:      {data['north']:>7.2f} m {'(south)' if data['north'] < 0 else '(north)'}\n"
                    f"    Up:         {data['up']:>7.2f} m {'(below/deeper)' if data['up'] < 0 else '(above/shallower)'}\n"
                    f"  Distance:\n"
                    f"    3D:         {data['distance_3d']:>7.2f} m\n"
                    f"  Status: VALID ✓ (angles-based positioning)\n"
                    f"{'='*70}"
                )
                
                logging.info(
                    f"📊 Position Statistics: {position_stats['valid_fixes']} total "
                    f"({position_stats['usbllong_fixes']} USBLLONG, "
                    f"{position_stats['usblangles_fixes']} USBLANGLES)"
                )
            
            elif msg_type == "USBLLONG":
                timestamp = datetime.now()
                
                if data["valid"]:
                    position_stats["usbllong_fixes"] += 1
                    position_stats["valid_fixes"] += 1
                    position_stats["last_valid_position"] = (data["east"], data["north"], data["up"])
                    position_stats["last_valid_time"] = timestamp
                    
                    logging.info(
                        f"\n{'='*70}\n"
                        f"📍 TRANSPONDER POSITION (MOTION-COMPENSATED) #{position_stats['usbllong_fixes']}\n"
                        f"{'='*70}\n"
                        f"  Target ID: {data['target_id']}\n"
                        f"  Coordinates (relative to this transceiver):\n"
                        f"    East:     {data['east']:>8.2f} m {'(west)' if data['east'] < 0 else '(east)'}\n"
                        f"    North:    {data['north']:>8.2f} m {'(south)' if data['north'] < 0 else '(north)'}\n"
                        f"    Up:       {data['up']:>8.2f} m {'(below/deeper)' if data['up'] < 0 else '(above/shallower)'}\n"
                        f"  Distance:\n"
                        f"    3D:       {data['distance_3d']:>8.2f} m\n"
                        f"    Range:    {data['range']:>8.2f} m (acoustic)\n"
                        f"  Quality:\n"
                        f"    Uncertainty: ±{data['uncertainty']:.2f} m\n"
                        f"    Status:   VALID ✓\n"
                        f"{'='*70}"
                    )
                else:
                    position_stats["invalid_fixes"] += 1
                    logging.warning(
                        f"\n📍 POSITION FIX FAILED (Invalid)\n"
                        f"    Target ID:   {data['target_id']}\n"
                        f"    Range:       {data['range']:.2f} m (acoustic distance only)\n"
                        f"    Uncertainty: {data['uncertainty']:.2f} m (too high)\n"
                    )
                
                total = position_stats["valid_fixes"] + position_stats["invalid_fixes"]
                logging.info(
                    f"📊 Position Statistics: {total} total "
                    f"({position_stats['valid_fixes']} valid, {position_stats['invalid_fixes']} invalid)"
                )
            
            elif msg_type == "USBLANGLES_INVALID":
                logging.error(f"❌ USBLANGLES has {data.get('arg_count', '?')} args, expected at least 7")
            
            elif msg_type == "USBLANGLES_PARSE_ERROR":
                logging.error(f"❌ USBLANGLES parsing error: {data['error']}")
            
            elif msg_type == "RECVIM":
                message_stats["received"] += 1
                
                if data["valid"]:
                    message_stats["valid"] += 1
                    logging.info(
                        f"\n{'='*70}\n"
                        f"✉️  MESSAGE #{message_stats['received']} RECEIVED (VALID)\n"
                        f"{'='*70}\n"
                        f"  From:      Transponder ID {data['sender_id']}\n"
                        f"  RSSI:      {data['rssi']} dBm\n"
                        f"  Integrity: {data['integrity']} (threshold: >{MIN_INTEGRITY}) ✓\n"
                        f"  Content:   {data['message']}\n"
                        f"{'='*70}"
                    )
                else:
                    message_stats["discarded"] += 1
                    logging.warning(
                        f"\n{'='*70}\n"
                        f"❌ MESSAGE #{message_stats['received']} DISCARDED (LOW INTEGRITY)\n"
                        f"{'='*70}\n"
                        f"  From:      Transponder ID {data['sender_id']}\n"
                        f"  RSSI:      {data['rssi']} dBm\n"
                        f"  Integrity: {data['integrity']} (threshold: >{MIN_INTEGRITY}) ✗\n"
                        f"  Content:   {data['message']}\n"
                        f"{'='*70}"
                    )
                
                logging.info(
                    f"📊 Message Statistics: {message_stats['valid']} valid, "
                    f"{message_stats['discarded']} discarded, {message_stats['received']} total"
                )
            
            elif msg_type == "RECVIM_ERROR":
                logging.error(f"Failed to parse received message: {data}")
            
            elif msg_type == "DELIVEREDIM":
                logging.info(f"✓ Message delivery confirmed to target {data['target_id']}")
            
            elif msg_type == "FAILEDIM":
                logging.warning(f"✗ Message delivery failed to target {data['target_id']}")
            
            elif msg_type in ["SENDSTART", "SENDEND"]:
                logging.debug(f"Protocol: {msg_type}")
            
            elif msg_type == "OTHER":
                logging.debug(f"Other: {data.get('raw', '')}")
            
            elif msg_type != "UNKNOWN":
                logging.debug(f"{msg_type}: {data}")
                
    except asyncio.CancelledError:
        logging.info("Response reader cancelled")
    except Exception as e:
        logging.error(f"Response reader error: {e}", exc_info=True)


async def status_summary():
    """Periodic status summary"""
    try:
        await asyncio.sleep(30)
        
        while True:
            await asyncio.sleep(30)
            
            logging.info("\n" + "="*70)
            logging.info("STATUS SUMMARY")
            logging.info("="*70)
            
            if message_stats["received"] > 0:
                valid_pct = (message_stats["valid"] / message_stats["received"] * 100)
                logging.info(
                    f"Messages: {message_stats['valid']} valid, "
                    f"{message_stats['discarded']} discarded, "
                    f"{message_stats['received']} total ({valid_pct:.1f}% success)"
                )
            else:
                logging.info("Messages: None received yet")
            
            if position_stats["last_valid_position"]:
                e, n, u = position_stats["last_valid_position"]
                time_ago = (datetime.now() - position_stats["last_valid_time"]).total_seconds()
                dist_3d = calculate_distance_3d(e, n, u)
                
                logging.info(
                    f"Position: {position_stats['valid_fixes']} valid fixes "
                    f"({position_stats['usbllong_fixes']} USBLLONG, "
                    f"{position_stats['usblangles_fixes']} USBLANGLES)"
                )
                logging.info(
                    f"Last known position ({time_ago:.0f}s ago):\n"
                    f"  E={e:.2f}m, N={n:.2f}m, U={u:.2f}m, Distance={dist_3d:.2f}m"
                )
            else:
                logging.info("Position: No valid fix yet")
            
            logging.info("="*70 + "\n")
                
    except asyncio.CancelledError:
        pass


async def main():
    """Main function"""
    try:
        reader, writer = await asyncio.open_connection(USBL_IP, USBL_PORT)
        logging.info(f"✓ Connected to USBL at {USBL_IP}:{USBL_PORT}")
        logging.info(f"📋 Transceiver ID: {TRANSCEIVER_ID}, Listening for Transponder ID: {TRANSPONDER_ID}")
        logging.info(f"📊 Min Message Integrity: {MIN_INTEGRITY}")
        logging.info(f"📐 Position modes: USBLLONG (motion-compensated) and USBLANGLES (raw angles)\n")
        
        reader_task = asyncio.create_task(response_reader(reader), name="Reader")
        summary_task = asyncio.create_task(status_summary(), name="Summary")
        
        try:
            await asyncio.gather(reader_task, summary_task)
        except KeyboardInterrupt:
            logging.info("\nStopped by user")
        finally:
            reader_task.cancel()
            summary_task.cancel()
            
            try:
                await asyncio.gather(reader_task, summary_task)
            except asyncio.CancelledError:
                pass
            
            writer.close()
            await writer.wait_closed()
            
            # Final summary
            logging.info("\n" + "="*70)
            logging.info("FINAL SUMMARY")
            logging.info("="*70)
            
            if message_stats["received"] > 0:
                valid_pct = (message_stats["valid"] / message_stats["received"] * 100)
                logging.info(
                    f"Total messages: {message_stats['received']} "
                    f"({message_stats['valid']} valid, {message_stats['discarded']} discarded, "
                    f"{valid_pct:.1f}% success)"
                )
            
            total_fixes = position_stats["valid_fixes"] + position_stats["invalid_fixes"]
            if total_fixes > 0:
                logging.info(
                    f"Total position fixes: {total_fixes} "
                    f"({position_stats['usbllong_fixes']} USBLLONG, "
                    f"{position_stats['usblangles_fixes']} USBLANGLES, "
                    f"{position_stats['invalid_fixes']} invalid)"
                )
                
                if position_stats["last_valid_position"]:
                    e, n, u = position_stats["last_valid_position"]
                    dist_3d = calculate_distance_3d(e, n, u)
                    logging.info(f"Last position: E={e:.2f}m, N={n:.2f}m, U={u:.2f}m, Distance={dist_3d:.2f}m")
            
            logging.info("="*70)
            logging.info("Connection closed")
            
    except ConnectionRefusedError:
        logging.error(f"Could not connect to USBL at {USBL_IP}:{USBL_PORT}")
        logging.error("Make sure the USBL device is powered on and the IP address is correct")
    except Exception as e:
        logging.error(f"Connection error: {e}", exc_info=True)


if __name__ == '__main__':
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s | %(levelname)s | %(message)s"
    )
    
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        logging.info("\nShutdown requested")