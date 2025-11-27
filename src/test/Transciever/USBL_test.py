import asyncio
import json
import logging

# Configuration
TRANSCEIVER_ID = 2
TRANSPONDER_ID = 3
USBL_IP = "192.168.0.139"  # Transceiver IP
USBL_PORT = 9200
CODEC = "utf-8"
SPEED_OF_SOUND_MPS = 1500.0  # Sea water
MIN_INTEGRITY = 50  # Minimum integrity threshold for valid messages

INSTANT_MESSAGE_RESPONSE_KEYWORDS = {
    "SENDSTART", "SENDEND", "RECVSTART", "RECVEND", "DELIVEREDIM", "USBLLONG", 
    "USBLANGLES", "USBLPHYD", "USBLPHYP", "FAILEDIM", "RECVIM", "CANCELEDIM", 
    "ERROR INTERNAL"
}


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
                
                if keyword == "USBLLONG" and len(args) == 17:
                    target_id = args[3]
                    east = float(args[7])
                    north = float(args[8])
                    up = float(args[9])
                    propagation_time_us = int(args[13])
                    position_uncertainty_m = float(args[16])
                    
                    slant_range_m = (propagation_time_us / 1_000_000.0) * SPEED_OF_SOUND_MPS
                    
                    return "USBLLONG", {
                        "target_id": target_id,
                        "east": east,
                        "north": north,
                        "up": up,
                        "range": slant_range_m,
                        "uncertainty": position_uncertainty_m,
                        "valid": position_uncertainty_m <= (0.95 * slant_range_m)
                    }
                
                elif keyword == "DELIVEREDIM":
                    return "DELIVEREDIM", {"target_id": args[1]}
                
                elif keyword == "FAILEDIM":
                    return "FAILEDIM", {"target_id": args[1]}
                
                elif keyword == "RECVIM":
                    if len(args) >= 9:
                        try:
                            integrity = int(args[8])
                            
                            # Extract payload - find the last occurrence of ,[
                            if ',[' in str_msg:
                                _, payload = str_msg.rsplit(',[', 1)
                                payload = payload.strip()
                                
                                # Try to parse as JSON
                                if payload.startswith("{"):
                                    try:
                                        payload = json.loads(payload)
                                    except:
                                        pass
                                
                                return "RECVIM", {
                                    "sender_id": args[3],
                                    "integrity": integrity,
                                    "valid": integrity > MIN_INTEGRITY,
                                    "rssi": int(args[6]),
                                    "message": payload
                                }
                            else:
                                # No payload found
                                return "RECVIM", {
                                    "sender_id": args[3],
                                    "integrity": integrity,
                                    "valid": integrity > MIN_INTEGRITY,
                                    "rssi": int(args[6]),
                                    "message": None
                                }
                        except (ValueError, IndexError) as e:
                            logging.error(f"Error parsing RECVIM: {e}")
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
    logging.info("🎧 Transceiver listening for messages...")
    
    received_count = 0
    discarded_count = 0
    
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
                logging.debug(f"📡 Transmission complete: {data}")
            
            elif msg_type == "USBLLONG":
                if data["valid"]:
                    logging.info(
                        f"📍 Target {data['target_id']} position: "
                        f"E={data['east']:.2f}m N={data['north']:.2f}m U={data['up']:.2f}m "
                        f"Range={data['range']:.2f}m ±{data['uncertainty']:.2f}m"
                    )
                else:
                    logging.warning(
                        f"📍 Invalid position fix for target {data['target_id']} "
                        f"(uncertainty {data['uncertainty']:.2f}m too high for range {data['range']:.2f}m)"
                    )
            
            elif msg_type == "RECVIM":
                received_count += 1
                
                if data["valid"]:
                    logging.info(
                        f"✉️  MESSAGE #{received_count} RECEIVED ✉️\n"
                        f"    From: Transponder ID {data['sender_id']}\n"
                        f"    RSSI: {data['rssi']} dBm\n"
                        f"    Integrity: {data['integrity']} (valid)\n"
                        f"    Content: {data['message']}"
                    )
                else:
                    discarded_count += 1
                    logging.warning(
                        f"❌ MESSAGE #{received_count} DISCARDED (low integrity)\n"
                        f"    From: Transponder ID {data['sender_id']}\n"
                        f"    RSSI: {data['rssi']} dBm\n"
                        f"    Integrity: {data['integrity']} (threshold: >{MIN_INTEGRITY})\n"
                        f"    Content: {data['message']}"
                    )
                
                # Show statistics
                valid_count = received_count - discarded_count
                logging.info(
                    f"📊 Statistics: {valid_count} valid, {discarded_count} discarded, "
                    f"{received_count} total"
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
                # Log raw messages at debug level
                logging.debug(f"Raw: {data.get('raw', '')}")
            
            elif msg_type != "UNKNOWN":
                logging.debug(f"{msg_type}: {data}")
                
    except asyncio.CancelledError:
        logging.info("Response reader cancelled")
    except Exception as e:
        logging.error(f"Response reader error: {e}", exc_info=True)


async def heartbeat():
    """Periodic status indicator"""
    try:
        while True:
            await asyncio.sleep(30)
            logging.info("💓 Transceiver still listening...")
    except asyncio.CancelledError:
        pass


async def main():
    """Main function"""
    try:
        reader, writer = await asyncio.open_connection(USBL_IP, USBL_PORT)
        logging.info(f"✓ Connected to USBL at {USBL_IP}:{USBL_PORT}")
        logging.info(f"📋 Configuration: Transceiver ID={TRANSCEIVER_ID}, Min Integrity={MIN_INTEGRITY}")
        
        reader_task = asyncio.create_task(response_reader(reader), name="Reader")
        heartbeat_task = asyncio.create_task(heartbeat(), name="Heartbeat")
        
        try:
            await asyncio.gather(reader_task, heartbeat_task)
        except KeyboardInterrupt:
            logging.info("Stopped by user")
        finally:
            reader_task.cancel()
            heartbeat_task.cancel()
            
            try:
                await asyncio.gather(reader_task, heartbeat_task)
            except asyncio.CancelledError:
                pass
            
            writer.close()
            await writer.wait_closed()
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