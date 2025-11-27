import asyncio
import json
import logging

# Configuration
TRANSCEIVER_ID = 2
TRANSPONDER_ID = 3
USBL_IP = "192.168.0.232"  # Transponder IP
USBL_PORT = 9200
MESSAGE_INTERVAL = 1.0  # seconds
CODEC = "utf-8"
SPEED_OF_SOUND_MPS = 1500.0  # Sea water

# Test message to send
TEST_MESSAGE = {"status": "test", "value": 123}

INSTANT_MESSAGE_RESPONSE_KEYWORDS = {
    "SENDSTART", "SENDEND", "RECVSTART", "RECVEND", "DELIVEREDIM", "USBLLONG", 
    "USBLANGLES", "USBLPHYD", "USBLPHYP", "FAILEDIM", "RECVIM", "CANCELEDIM", 
    "ERROR INTERNAL"
}


def encode_message(target_id: int, payload: dict) -> bytes:
    """Encode message for USBL"""
    payload_str = json.dumps(payload, separators=(',', ':'))
    
    if len(payload_str) > 63:
        raise ValueError("Message too long (max 63 bytes)")
    
    payload_str = f"[{payload_str}"
    command = f"AT*SENDIM,p0,{len(payload_str)},{target_id},ack,{payload_str}"
    
    return f"+++{command}\n".encode(CODEC)


def decode_response(message: bytes) -> tuple[str, dict]:
    """Parse USBL response"""
    try:
        str_msg = message.decode(CODEC).strip()
        
        if not str_msg or not str_msg.startswith("+++"):
            return "UNKNOWN", {}
        
        parts = str_msg.split(":")
        
        if "AT" in str_msg:
            # Async response (SENDSTART, DELIVEREDIM, USBLLONG, etc.)
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
                        integrity = int(args[8])
                        _, payload = str_msg.rsplit(',[', 1)
                        payload = payload.strip()
                        
                        if payload.startswith("{"):
                            try:
                                payload = json.loads(payload)
                            except:
                                pass
                        
                        return "RECVIM", {
                            "sender_id": args[3],
                            "integrity": integrity,
                            "valid": integrity > 100,
                            "message": payload
                        }
                
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
        return "ERROR", {"error": str(e)}


async def response_reader(reader: asyncio.StreamReader):
    """Continuously read and parse USBL responses"""
    logging.info("Response reader started")
    
    try:
        while True:
            line = await reader.readline()
            
            if not line:
                logging.warning("Connection closed by USBL")
                break
            
            msg_type, data = decode_response(line)
            
            if msg_type == "AT*SENDIM_RESPONSE":
                if data.get("status") == "OK":
                    logging.info("✓ Send command accepted")
                else:
                    logging.warning(f"✗ Send command failed: {data}")
            
            elif msg_type == "DELIVEREDIM":
                logging.info(f"✓ Message delivered to target {data['target_id']}")
            
            elif msg_type == "FAILEDIM":
                logging.warning(f"✗ Message failed to target {data['target_id']}")
            
            elif msg_type == "USBLLONG":
                if data["valid"]:
                    logging.info(
                        f"📍 Position: E={data['east']:.2f}m N={data['north']:.2f}m "
                        f"U={data['up']:.2f}m Range={data['range']:.2f}m "
                        f"Uncertainty={data['uncertainty']:.2f}m"
                    )
                else:
                    logging.warning(f"📍 Invalid position fix (uncertainty too high)")
            
            elif msg_type == "RECVIM":
                if data["valid"]:
                    logging.info(
                        f"📨 Received from {data['sender_id']} "
                        f"(integrity={data['integrity']}): {data['message']}"
                    )
                else:
                    logging.warning(
                        f"📨 Low integrity message discarded (integrity={data['integrity']})"
                    )
            
            elif msg_type in ["SENDSTART", "SENDEND", "RECVSTART", "RECVEND"]:
                logging.debug(f"Protocol: {msg_type}")
            
            elif msg_type != "OTHER":
                logging.debug(f"{msg_type}: {data}")
                
    except asyncio.CancelledError:
        logging.info("Response reader cancelled")
    except Exception as e:
        logging.error(f"Response reader error: {e}")


async def message_sender(writer: asyncio.StreamWriter):
    """Send messages periodically"""
    message_count = 0
    
    try:
        while True:
            await asyncio.sleep(MESSAGE_INTERVAL)
            
            message_count += 1
            test_msg = TEST_MESSAGE.copy()
            test_msg["count"] = message_count
            
            encoded = encode_message(TRANSCEIVER_ID, test_msg)
            writer.write(encoded)
            await writer.drain()
            
            logging.info(f"→ Sent message #{message_count}: {test_msg}")
            
    except asyncio.CancelledError:
        logging.info("Message sender cancelled")
    except Exception as e:
        logging.error(f"Sender error: {e}")


async def main():
    """Main function"""
    reader, writer = await asyncio.open_connection(USBL_IP, USBL_PORT)
    logging.info(f"Connected to USBL at {USBL_IP}:{USBL_PORT}")
    
    reader_task = asyncio.create_task(response_reader(reader), name="Reader")
    sender_task = asyncio.create_task(message_sender(writer), name="Sender")
    
    try:
        await asyncio.gather(reader_task, sender_task)
    except KeyboardInterrupt:
        logging.info("Stopped by user")
    finally:
        reader_task.cancel()
        sender_task.cancel()
        
        try:
            await asyncio.gather(reader_task, sender_task)
        except asyncio.CancelledError:
            pass
        
        writer.close()
        await writer.wait_closed()
        logging.info("Connection closed")


if __name__ == '__main__':
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s | %(levelname)s | %(message)s"
    )
    
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        logging.info("Shutdown requested")