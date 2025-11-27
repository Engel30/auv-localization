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
MIN_INTEGRITY = 50

# DEBUG MODE - Set to True for detailed diagnostics
DEBUG_MODE = False

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
    "valid_fixes": 0,
    "invalid_fixes": 0,
    "last_valid_position": None,
    "last_valid_time": None
}

# Debug tracking
debug_stats = {
    "total_responses": 0,
    "parse_errors": 0,
    "unknown_messages": 0,
    "protocol_sequence": [],
    "last_recvim_time": None,
    "last_usbllong_time": None
}


def calculate_distance_3d(east, north, up):
    """Calculate 3D distance from origin"""
    return math.sqrt(east**2 + north**2 + up**2)


def decode_response(message: bytes) -> tuple[str, dict]:
    """Parse USBL response with detailed error tracking"""
    debug_stats["total_responses"] += 1
    
    try:
        str_msg = message.decode(CODEC).strip()
        
        if DEBUG_MODE:
            logging.debug(f"RAW RESPONSE #{debug_stats['total_responses']}: {str_msg}")
        
        if not str_msg:
            logging.warning("⚠️  Received empty response")
            return "EMPTY", {}
        
        if not str_msg.startswith("+++"):
            logging.warning(f"⚠️  Response doesn't start with '+++': {str_msg[:50]}")
            debug_stats["unknown_messages"] += 1
            return "MALFORMED", {"raw": str_msg}
        
        parts = str_msg.split(":")
        
        if DEBUG_MODE:
            logging.debug(f"   Parsed into {len(parts)} parts: {parts}")
        
        if "AT" in str_msg:
            # Async response
            if len(parts) < 3:
                logging.warning(f"⚠️  AT response has insufficient parts: {parts}")
                return "INCOMPLETE", {"raw": str_msg}
            
            args = parts[2].split(",")
            keyword = args[0]
            
            if DEBUG_MODE:
                logging.debug(f"   Keyword: '{keyword}', Args count: {len(args)}")
            
            if keyword == "USBLLONG":
                timestamp = datetime.now()
                debug_stats["last_usbllong_time"] = timestamp
                
                if len(args) != 17:
                    logging.error(f"❌ USBLLONG has {len(args)} args, expected 17")
                    logging.error(f"   Raw: {str_msg}")
                    return "USBLLONG_INVALID", {"raw": str_msg, "arg_count": len(args)}
                
                try:
                    target_id = args[3]
                    east = float(args[7])
                    north = float(args[8])
                    up = float(args[9])
                    propagation_time_us = int(args[13])
                    position_uncertainty_m = float(args[16])
                    
                    slant_range_m = (propagation_time_us / 1_000_000.0) * SPEED_OF_SOUND_MPS
                    distance_3d = calculate_distance_3d(east, north, up)
                    
                    is_valid = position_uncertainty_m <= (0.95 * slant_range_m)
                    
                    if DEBUG_MODE:
                        logging.debug(
                            f"   USBLLONG PARSED:\n"
                            f"      Target: {target_id}\n"
                            f"      E={east:.3f}, N={north:.3f}, U={up:.3f}\n"
                            f"      Propagation: {propagation_time_us} µs\n"
                            f"      Range: {slant_range_m:.3f} m\n"
                            f"      Uncertainty: {position_uncertainty_m:.3f} m\n"
                            f"      Valid: {is_valid} (uncertainty/range ratio: {position_uncertainty_m/slant_range_m:.2%})"
                        )
                    
                    return "USBLLONG", {
                        "target_id": target_id,
                        "east": east,
                        "north": north,
                        "up": up,
                        "range": slant_range_m,
                        "distance_3d": distance_3d,
                        "uncertainty": position_uncertainty_m,
                        "valid": is_valid,
                        "propagation_us": propagation_time_us
                    }
                    
                except (ValueError, IndexError) as e:
                    logging.error(f"❌ Error parsing USBLLONG values: {e}")
                    logging.error(f"   Args: {args}")
                    debug_stats["parse_errors"] += 1
                    return "USBLLONG_PARSE_ERROR", {"error": str(e), "args": args}
            
            elif keyword == "DELIVEREDIM":
                return "DELIVEREDIM", {"target_id": args[1] if len(args) > 1 else "?"}
            
            elif keyword == "FAILEDIM":
                return "FAILEDIM", {"target_id": args[1] if len(args) > 1 else "?"}
            
            elif keyword == "RECVIM":
                timestamp = datetime.now()
                debug_stats["last_recvim_time"] = timestamp
                
                if len(args) < 9:
                    logging.error(f"❌ RECVIM has {len(args)} args, expected at least 9")
                    return "RECVIM_INVALID", {"raw": str_msg, "arg_count": len(args)}
                
                try:
                    integrity = int(args[8])
                    sender_id = args[3]
                    rssi = int(args[6])
                    
                    # Extract payload
                    payload = None
                    if ',[' in str_msg:
                        _, payload = str_msg.rsplit(',[', 1)
                        payload = payload.strip()
                        
                        if DEBUG_MODE:
                            logging.debug(f"   Extracted payload: {payload[:100]}")
                        
                        if payload.startswith("{"):
                            try:
                                payload = json.loads(payload)
                            except json.JSONDecodeError as e:
                                logging.warning(f"⚠️  JSON parse failed: {e}")
                    else:
                        logging.warning("⚠️  No payload delimiter ',[' found in RECVIM")
                    
                    is_valid = integrity > MIN_INTEGRITY
                    
                    if DEBUG_MODE:
                        logging.debug(
                            f"   RECVIM PARSED:\n"
                            f"      Sender: {sender_id}\n"
                            f"      RSSI: {rssi} dBm\n"
                            f"      Integrity: {integrity} (threshold: >{MIN_INTEGRITY})\n"
                            f"      Valid: {is_valid}\n"
                            f"      Payload: {payload}"
                        )
                    
                    return "RECVIM", {
                        "sender_id": sender_id,
                        "integrity": integrity,
                        "valid": is_valid,
                        "rssi": rssi,
                        "message": payload
                    }
                    
                except (ValueError, IndexError) as e:
                    logging.error(f"❌ Error parsing RECVIM: {e}")
                    logging.error(f"   Args: {args}")
                    debug_stats["parse_errors"] += 1
                    return "RECVIM_ERROR", {"error": str(e), "raw": str_msg}
            
            elif keyword in ["RECVSTART", "RECVEND", "SENDSTART", "SENDEND"]:
                # Track protocol sequence
                debug_stats["protocol_sequence"].append((keyword, datetime.now()))
                if len(debug_stats["protocol_sequence"]) > 20:
                    debug_stats["protocol_sequence"].pop(0)
                
                if keyword == "RECVEND" and len(args) >= 5:
                    if DEBUG_MODE:
                        logging.debug(
                            f"   RECVEND details:\n"
                            f"      Timestamp1: {args[1]}\n"
                            f"      Timestamp2: {args[2]}\n"
                            f"      RSSI: {args[3]} dBm\n"
                            f"      Integrity: {args[4]}"
                        )
                
                return keyword, {"args": args}
            
            elif keyword in INSTANT_MESSAGE_RESPONSE_KEYWORDS:
                return keyword, {"args": args}
            else:
                logging.warning(f"⚠️  Unknown keyword: {keyword}")
                debug_stats["unknown_messages"] += 1
        
        else:
            # Direct command response
            cmd = parts[0].replace("+", "")
            if len(parts) >= 3:
                status = parts[2]
                if DEBUG_MODE:
                    logging.debug(f"   Direct response: {cmd} -> {status}")
                return f"{cmd}_RESPONSE", {"status": status}
        
        debug_stats["unknown_messages"] += 1
        return "OTHER", {"raw": str_msg}
        
    except Exception as e:
        logging.error(f"❌ CRITICAL PARSE ERROR: {e}", exc_info=True)
        debug_stats["parse_errors"] += 1
        return "ERROR", {"error": str(e), "raw": message}


async def response_reader(reader: asyncio.StreamReader):
    """Continuously read and parse USBL responses"""
    logging.info("🎧 DEBUG MODE: Transceiver listening with enhanced diagnostics...\n")
    
    try:
        while True:
            line = await reader.readline()
            
            if not line:
                logging.error("❌ Connection closed by USBL")
                break
            
            msg_type, data = decode_response(line)
            
            if msg_type == "RECVSTART":
                logging.info("📡 ▶️  Starting to receive transmission...")
            
            elif msg_type == "RECVEND":
                logging.info("📡 ⏸️  Transmission receive complete")
                if DEBUG_MODE and "args" in data and len(data["args"]) >= 5:
                    logging.debug(f"   RSSI: {data['args'][3]} dBm, Integrity: {data['args'][4]}")
            
            elif msg_type == "SENDSTART":
                logging.info("📡 ▶️  Starting to send response...")
            
            elif msg_type == "SENDEND":
                logging.info("📡 ⏸️  Send complete")
            
            elif msg_type == "USBLLONG":
                timestamp = datetime.now()
                
                # Check timing from RECVIM
                if debug_stats["last_recvim_time"]:
                    time_diff = (timestamp - debug_stats["last_recvim_time"]).total_seconds()
                    logging.debug(f"   ⏱️  Time from RECVIM to USBLLONG: {time_diff:.3f}s")
                
                if data["valid"]:
                    position_stats["valid_fixes"] += 1
                    position_stats["last_valid_position"] = (data["east"], data["north"], data["up"])
                    position_stats["last_valid_time"] = timestamp
                    
                    logging.info(
                        f"\n{'='*70}\n"
                        f"✅ VALID POSITION FIX #{position_stats['valid_fixes']}\n"
                        f"{'='*70}\n"
                        f"  Target ID: {data['target_id']}\n"
                        f"  Coordinates (relative to this transceiver):\n"
                        f"    East:     {data['east']:>8.2f} m {'(west)' if data['east'] < 0 else '(east)'}\n"
                        f"    North:    {data['north']:>8.2f} m {'(south)' if data['north'] < 0 else '(north)'}\n"
                        f"    Up:       {data['up']:>8.2f} m {'(below/deeper)' if data['up'] < 0 else '(above/shallower)'}\n"
                        f"  Distance:\n"
                        f"    3D:       {data['distance_3d']:>8.2f} m\n"
                        f"    Range:    {data['range']:>8.2f} m (from time-of-flight: {data['propagation_us']}µs)\n"
                        f"  Quality:\n"
                        f"    Uncertainty: ±{data['uncertainty']:.2f} m ({data['uncertainty']/data['range']*100:.1f}% of range)\n"
                        f"    Status:   VALID ✓\n"
                        f"{'='*70}"
                    )
                else:
                    position_stats["invalid_fixes"] += 1
                    logging.warning(
                        f"\n{'='*70}\n"
                        f"❌ INVALID POSITION FIX\n"
                        f"{'='*70}\n"
                        f"  Target ID:   {data['target_id']}\n"
                        f"  Range:       {data['range']:.2f} m (time-of-flight: {data['propagation_us']}µs)\n"
                        f"  Uncertainty: {data['uncertainty']:.2f} m ({data['uncertainty']/data['range']*100:.1f}% of range)\n"
                        f"  ENU coords:  E={data['east']:.2f}, N={data['north']:.2f}, U={data['up']:.2f}\n"
                        f"  ⚠️  Reason: Uncertainty too high - could not determine accurate bearing\n"
                        f"      Only distance is reliable, discard directional data\n"
                        f"  💡 Tips to improve:\n"
                        f"      - Reduce acoustic noise\n"
                        f"      - Improve line of sight\n"
                        f"      - Check transducer placement\n"
                        f"      - Verify device orientations\n"
                        f"{'='*70}"
                    )
                
                # Show statistics
                total = position_stats["valid_fixes"] + position_stats["invalid_fixes"]
                valid_pct = (position_stats["valid_fixes"] / total * 100) if total > 0 else 0
                
                logging.info(
                    f"📊 Position Statistics: {position_stats['valid_fixes']} valid, "
                    f"{position_stats['invalid_fixes']} invalid ({valid_pct:.1f}% success rate)"
                )
            
            elif msg_type == "USBLLONG_INVALID":
                logging.error(
                    f"❌ USBLLONG MESSAGE FORMAT ERROR\n"
                    f"   Expected 17 arguments, got {data.get('arg_count', '?')}\n"
                    f"   This indicates a protocol parsing issue"
                )
            
            elif msg_type == "USBLLONG_PARSE_ERROR":
                logging.error(
                    f"❌ USBLLONG PARSING ERROR: {data['error']}\n"
                    f"   Could not extract numerical values from USBLLONG message\n"
                    f"   This may indicate data corruption"
                )
            
            elif msg_type == "RECVIM":
                message_stats["received"] += 1
                
                if data["valid"]:
                    message_stats["valid"] += 1
                    logging.info(
                        f"\n{'='*70}\n"
                        f"✅ MESSAGE #{message_stats['received']} RECEIVED (VALID)\n"
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
                        f"  ⚠️  Message quality too low - likely corrupted\n"
                        f"  💡 Tips to improve:\n"
                        f"      - Reduce distance between devices\n"
                        f"      - Minimize acoustic interference\n"
                        f"      - Check water conditions\n"
                        f"{'='*70}"
                    )
                
                logging.info(
                    f"📊 Message Statistics: {message_stats['valid']} valid, "
                    f"{message_stats['discarded']} discarded, {message_stats['received']} total"
                )
            
            elif msg_type == "RECVIM_INVALID":
                logging.error(
                    f"❌ RECVIM MESSAGE FORMAT ERROR\n"
                    f"   Expected at least 9 arguments, got {data.get('arg_count', '?')}"
                )
            
            elif msg_type == "RECVIM_ERROR":
                logging.error(f"❌ RECVIM PARSING ERROR: {data['error']}")
            
            elif msg_type == "MALFORMED":
                logging.error(f"❌ Malformed message: {data['raw'][:100]}")
            
            elif msg_type == "INCOMPLETE":
                logging.warning(f"⚠️  Incomplete message: {data['raw'][:100]}")
            
            elif msg_type == "EMPTY":
                logging.warning("⚠️  Empty message received")
            
            elif msg_type == "OTHER":
                if DEBUG_MODE:
                    logging.debug(f"   Other message: {data.get('raw', '')[:100]}")
            
            elif msg_type != "UNKNOWN":
                if DEBUG_MODE:
                    logging.debug(f"   {msg_type}: {data}")
                
    except asyncio.CancelledError:
        logging.info("Response reader cancelled")
    except Exception as e:
        logging.error(f"❌ Response reader error: {e}", exc_info=True)


async def diagnostic_report():
    """Periodic diagnostic report"""
    try:
        await asyncio.sleep(30)
        
        while True:
            await asyncio.sleep(30)
            
            logging.info("\n" + "="*70)
            logging.info("🔍 DIAGNOSTIC REPORT")
            logging.info("="*70)
            
            # Protocol sequence
            if debug_stats["protocol_sequence"]:
                logging.info("Recent protocol sequence:")
                for keyword, timestamp in debug_stats["protocol_sequence"][-10:]:
                    logging.info(f"  {timestamp.strftime('%H:%M:%S.%f')[:-3]} - {keyword}")
            
            # Parse statistics
            logging.info(f"\nParsing statistics:")
            logging.info(f"  Total responses: {debug_stats['total_responses']}")
            logging.info(f"  Parse errors: {debug_stats['parse_errors']}")
            logging.info(f"  Unknown messages: {debug_stats['unknown_messages']}")
            
            # Message timing
            if debug_stats["last_recvim_time"] and debug_stats["last_usbllong_time"]:
                time_diff = (debug_stats["last_usbllong_time"] - debug_stats["last_recvim_time"]).total_seconds()
                logging.info(f"\nLast RECVIM to USBLLONG delay: {time_diff:.3f}s")
            
            # Message stats
            if message_stats["received"] > 0:
                valid_pct = (message_stats["valid"] / message_stats["received"] * 100)
                logging.info(
                    f"\nMessages: {message_stats['valid']} valid, "
                    f"{message_stats['discarded']} discarded, "
                    f"{message_stats['received']} total ({valid_pct:.1f}% success)"
                )
            else:
                logging.warning("\n⚠️  No messages received yet")
                logging.info("   Possible issues:")
                logging.info("   - Transponder not sending")
                logging.info("   - Acoustic link blocked")
                logging.info("   - IP configuration incorrect")
            
            # Position stats
            total_fixes = position_stats["valid_fixes"] + position_stats["invalid_fixes"]
            if total_fixes > 0:
                fix_pct = (position_stats["valid_fixes"] / total_fixes * 100)
                logging.info(
                    f"\nPosition fixes: {position_stats['valid_fixes']} valid, "
                    f"{position_stats['invalid_fixes']} invalid ({fix_pct:.1f}% success)"
                )
                
                if fix_pct < 50:
                    logging.warning("   ⚠️  Low position fix success rate!")
                    logging.info("   Recommendations:")
                    logging.info("   - Check device geometry (spread hydrophones)")
                    logging.info("   - Verify heading/orientation sensors")
                    logging.info("   - Reduce multipath reflections")
            else:
                logging.warning("\n⚠️  No position fixes yet")
                if message_stats["received"] > 0:
                    logging.info("   Messages received but no USBLLONG - this is unusual")
                    logging.info("   Check if position calculation is enabled on USBL device")
            
            logging.info("="*70 + "\n")
                
    except asyncio.CancelledError:
        pass


async def main():
    """Main function"""
    try:
        logging.info("🔧 DEBUG MODE ENABLED - Enhanced diagnostics active")
        logging.info(f"   Connecting to {USBL_IP}:{USBL_PORT}...\n")
        
        reader, writer = await asyncio.open_connection(USBL_IP, USBL_PORT)
        
        logging.info(f"✅ Connected to USBL at {USBL_IP}:{USBL_PORT}")
        logging.info(f"📋 Configuration:")
        logging.info(f"   - Transceiver ID: {TRANSCEIVER_ID}")
        logging.info(f"   - Listening for Transponder ID: {TRANSPONDER_ID}")
        logging.info(f"   - Min Message Integrity: {MIN_INTEGRITY}")
        logging.info(f"   - Speed of Sound: {SPEED_OF_SOUND_MPS} m/s")
        logging.info(f"   - Debug Mode: {DEBUG_MODE}\n")
        
        reader_task = asyncio.create_task(response_reader(reader), name="Reader")
        diagnostic_task = asyncio.create_task(diagnostic_report(), name="Diagnostics")
        
        try:
            await asyncio.gather(reader_task, diagnostic_task)
        except KeyboardInterrupt:
            logging.info("\n⏹️  Stopped by user")
        finally:
            reader_task.cancel()
            diagnostic_task.cancel()
            
            try:
                await asyncio.gather(reader_task, diagnostic_task)
            except asyncio.CancelledError:
                pass
            
            writer.close()
            await writer.wait_closed()
            
            # Final debug report
            logging.info("\n" + "="*70)
            logging.info("FINAL DEBUG REPORT")
            logging.info("="*70)
            logging.info(f"Total responses processed: {debug_stats['total_responses']}")
            logging.info(f"Parse errors: {debug_stats['parse_errors']}")
            logging.info(f"Unknown messages: {debug_stats['unknown_messages']}")
            
            if message_stats["received"] > 0:
                valid_pct = (message_stats["valid"] / message_stats["received"] * 100)
                logging.info(
                    f"\nMessages: {message_stats['received']} total "
                    f"({message_stats['valid']} valid, {message_stats['discarded']} discarded, "
                    f"{valid_pct:.1f}% success)"
                )
            
            if position_stats["valid_fixes"] + position_stats["invalid_fixes"] > 0:
                total_fixes = position_stats["valid_fixes"] + position_stats["invalid_fixes"]
                fix_pct = (position_stats["valid_fixes"] / total_fixes * 100)
                
                logging.info(
                    f"Position fixes: {total_fixes} total "
                    f"({position_stats['valid_fixes']} valid, {position_stats['invalid_fixes']} invalid, "
                    f"{fix_pct:.1f}% success)"
                )
                
                if position_stats["last_valid_position"]:
                    e, n, u = position_stats["last_valid_position"]
                    dist_3d = calculate_distance_3d(e, n, u)
                    logging.info(f"Last position: E={e:.2f}m, N={n:.2f}m, U={u:.2f}m, Distance={dist_3d:.2f}m")
            
            logging.info("="*70)
            logging.info("Connection closed")
            
    except ConnectionRefusedError:
        logging.error(f"❌ Could not connect to USBL at {USBL_IP}:{USBL_PORT}")
        logging.error("   Troubleshooting:")
        logging.error("   - Is the USBL device powered on?")
        logging.error("   - Is the IP address correct?")
        logging.error("   - Is there a network connection?")
        logging.error("   - Check firewall settings")
    except Exception as e:
        logging.error(f"❌ Connection error: {e}", exc_info=True)


if __name__ == '__main__':
    # Set to DEBUG level for maximum detail
    log_level = logging.DEBUG if DEBUG_MODE else logging.INFO
    
    logging.basicConfig(
        level=log_level,
        format="%(asctime)s | %(levelname)-7s | %(message)s"
    )
    
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        logging.info("\n⏹️  Shutdown requested")