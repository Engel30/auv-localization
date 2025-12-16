import asyncio
import json
import logging
from datetime import datetime

# ==================== CONFIGURATION ====================
TRANSPONDER_ID = 3
USBL_IP = "192.168.0.232"
USBL_PORT = 9200
CODEC = "utf-8"
MIN_INTEGRITY = 50

# Display settings
SHOW_RAW_MESSAGES = True  # Set True to see raw messages too
SHOW_TIMESTAMP = True      # Show timestamp for each message

# ========================================================

# State
state = {
    "message_count": 0,
    "valid_message_count": 0,
    "invalid_message_count": 0
}


def format_timestamp():
    """Ritorna timestamp corrente formattato"""
    return datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]


def decode_recvim(message: bytes) -> tuple[bool, dict]:
    """
    Decodifica un messaggio RECVIM e estrae il payload
    
    Returns:
        (success, data_dict)
        data_dict contiene: sender_id, rssi, integrity, valid, message
    """
    try:
        str_msg = message.decode(CODEC).strip()
        
        if not str_msg or not str_msg.startswith("+++"):
            return False, {}
        
        # Parse RECVIM format: +++AT:<length>:RECVIM,...
        if "RECVIM" not in str_msg:
            return False, {}
        
        parts = str_msg.split(":")
        if len(parts) < 3:
            return False, {}
        
        args = parts[2].split(",")
        keyword = args[0]
        
        if keyword == "RECVIM" and len(args) >= 9:
            try:
                sender_id = args[3]
                rssi = int(args[7])
                integrity = int(args[8])
                
                # Estrai payload JSON
                payload = None
                if ',[' in str_msg:
                    _, payload_str = str_msg.rsplit(',[', 1)
                    payload_str = payload_str.strip()
                    if payload_str.startswith("{"):
                        try:
                            payload = json.loads(payload_str)
                        except json.JSONDecodeError:
                            payload = payload_str  # Mantieni stringa se non è JSON valido
                
                return True, {
                    "sender_id": sender_id,
                    "rssi": rssi,
                    "integrity": integrity,
                    "valid": integrity > MIN_INTEGRITY,
                    "message": payload
                }
            except (ValueError, IndexError) as e:
                logging.debug(f"Error parsing RECVIM: {e}")
                return False, {}
        
        return False, {}
        
    except Exception as e:
        logging.debug(f"Decode error: {e}")
        return False, {}


def display_message(data: dict, raw_msg: bytes = None):
    """Visualizza il messaggio ricevuto in modo formattato"""
    
    state["message_count"] += 1
    
    # Header
    print("\n" + "="*70)
    if SHOW_TIMESTAMP:
        print(f"[{format_timestamp()}] MESSAGE #{state['message_count']}")
    else:
        print(f"MESSAGE #{state['message_count']}")
    print("="*70)
    
    # Info trasmissione
    print(f"From:       Transceiver ID {data['sender_id']}")
    print(f"RSSI:       {data['rssi']} dBm")
    print(f"Integrity:  {data['integrity']}")
    
    if not data['valid']:
        print(f"⚠️  WARNING: Low integrity (< {MIN_INTEGRITY})")
        state["invalid_message_count"] += 1
    else:
        state["valid_message_count"] += 1
    
    print("-"*70)
    
    # Payload
    payload = data.get('message')
    
    if payload is None:
        print("Payload:    (empty)")
    
    elif isinstance(payload, dict):
        # JSON ricevuto - controlla se è il formato atteso (timestamp + counter)
        if 'timestamp' in payload and 'counter' in payload:
            print("Type:       PING MESSAGE (from distance_tracker)")
            print(f"Counter:    {payload['counter']}")
            print(f"Timestamp:  {payload['timestamp']}")
            
            # Converti timestamp in datetime leggibile
            try:
                dt = datetime.fromtimestamp(payload['timestamp'])
                print(f"Time:       {dt.strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}")
            except:
                pass
        else:
            # JSON ma formato diverso
            print("Type:       JSON MESSAGE (unknown format)")
            print("Content:")
            print(json.dumps(payload, indent=2))
    
    else:
        # Non è JSON o è una stringa
        print("Type:       RAW MESSAGE")
        print(f"Content:    {payload}")
    
    print("="*70)
    
    # Mostra messaggio raw se richiesto
    if SHOW_RAW_MESSAGES and raw_msg:
        print("\nRAW MESSAGE:")
        try:
            print(raw_msg.decode(CODEC).strip())
        except:
            print(raw_msg)
        print("="*70)


async def message_listener(reader: asyncio.StreamReader):
    """Task che ascolta continuamente i messaggi in arrivo"""
    
    print("="*70)
    print("TRANSPONDER LISTENER - Message Receiver")
    print("="*70)
    print(f"Transponder ID: {TRANSPONDER_ID}")
    print(f"USBL IP:        {USBL_IP}:{USBL_PORT}")
    print(f"Min Integrity:  {MIN_INTEGRITY}")
    print("="*70)
    print()
    print("Listening for messages...")
    print("(Press Ctrl+C to stop)")
    print()
    
    try:
        while True:
            line = await reader.readline()
            
            if not line:
                logging.warning("Connection closed by remote")
                break
            
            # Prova a decodificare come RECVIM
            success, data = decode_recvim(line)
            
            if success:
                # Messaggio RECVIM valido
                display_message(data, line if SHOW_RAW_MESSAGES else None)
            else:
                # Messaggio non riconosciuto o altro tipo
                try:
                    msg_str = line.decode(CODEC).strip()
                    if msg_str:  # Ignora righe vuote
                        print(f"\n[{format_timestamp()}] OTHER MESSAGE:")
                        print(msg_str)
                        print()
                except:
                    print(f"\n[{format_timestamp()}] BINARY MESSAGE:")
                    print(line)
                    print()
    
    except asyncio.CancelledError:
        pass
    except Exception as e:
        logging.error(f"Listener error: {e}")


async def main():
    # Setup logging
    logging.basicConfig(
        level=logging.WARNING,  # Solo warning ed errori, non info
        format="%(message)s"
    )
    
    # Connetti al modem
    try:
        reader, writer = await asyncio.open_connection(USBL_IP, USBL_PORT)
        print(f"✓ Connected to USBL modem at {USBL_IP}:{USBL_PORT}")
        print()
    except Exception as e:
        print(f"❌ Failed to connect: {e}")
        return
    
    # Avvia listener
    listener_task = asyncio.create_task(message_listener(reader))
    
    try:
        await listener_task
    except KeyboardInterrupt:
        print("\n\n⏹️  Stopped by user")
    finally:
        listener_task.cancel()
        
        try:
            await listener_task
        except asyncio.CancelledError:
            pass
        
        writer.close()
        await writer.wait_closed()
        
        # Summary finale
        print("\n" + "="*70)
        print("SESSION SUMMARY")
        print("="*70)
        print(f"Total messages received: {state['message_count']}")
        print(f"Valid messages:          {state['valid_message_count']}")
        print(f"Invalid messages:        {state['invalid_message_count']}")
        print("="*70)


if __name__ == '__main__':
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass