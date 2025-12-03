import asyncio
import json
import logging
import time
import os
from datetime import datetime

# ==================== CONFIGURATION ====================
TRANSCEIVER_ID = 2
TRANSPONDER_ID = 3
USBL_IP = "192.168.0.139"
USBL_PORT = 9200
CODEC = "utf-8"
SPEED_OF_SOUND_MPS = 1500.0  # Modifica qui per acqua dolce (es. 1440)
MIN_INTEGRITY = 50

# Intervallo tra invii messaggi (secondi)
MESSAGE_INTERVAL = 5.0

# Timeout per aspettare risposte
RESPONSE_TIMEOUT = 3.0

# ==================== LOGGING CONFIGURATION ====================
ENABLE_JSON_LOGGING = True                          # Abilita/disabilita logging su file
LOG_DIRECTORY = "src/test_USBL/usbl/logs"           # Directory dove salvare i log
LOG_FILENAME_PREFIX = "tracker_log"            # Prefisso nome file

# ========================================================

class JSONLogger:
    """Gestisce il logging degli eventi in formato JSON"""
    
    def __init__(self, directory, prefix, enabled=True):
        self.enabled = enabled
        self.directory = directory
        self.prefix = prefix
        self.log_file = None
        self.log_filepath = None
        self.session_start = None
        
        if self.enabled:
            self._init_log_file()
    
    def _init_log_file(self):
        """Inizializza il file di log"""
        # Crea directory se non esiste
        if not os.path.exists(self.directory):
            os.makedirs(self.directory)
            print(f"✓ Created log directory: {self.directory}")
        
        # Genera nome file con timestamp
        self.session_start = datetime.now()
        timestamp = self.session_start.strftime("%Y%m%d_%H%M%S")
        self.log_filepath = os.path.join(
            self.directory, 
            f"{self.prefix}_{timestamp}.jsonl"
        )
        
        # Apri file
        self.log_file = open(self.log_filepath, 'w')
        
        # Scrivi header con metadata sessione
        session_metadata = {
            "event_type": "SESSION_START",
            "timestamp": self.session_start.isoformat(),
            "configuration": {
                "transceiver_id": TRANSCEIVER_ID,
                "transponder_id": TRANSPONDER_ID,
                "usbl_ip": USBL_IP,
                "usbl_port": USBL_PORT,
                "sound_speed_mps": SPEED_OF_SOUND_MPS,
                "message_interval": MESSAGE_INTERVAL,
                "min_integrity": MIN_INTEGRITY
            }
        }
        self._write_event(session_metadata)
        
        print(f"✓ Logging to: {self.log_filepath}")
    
    def _write_event(self, event_dict):
        """Scrivi un evento nel log (JSON Lines format)"""
        if self.log_file and not self.log_file.closed:
            json_line = json.dumps(event_dict, separators=(',', ':'))
            self.log_file.write(json_line + '\n')
            self.log_file.flush()  # Flush immediato per non perdere dati
    
    def log_event(self, event_type, data):
        """Logga un evento generico"""
        if not self.enabled:
            return
        
        event = {
            "event_type": event_type,
            "timestamp": datetime.now().isoformat(),
            "data": data
        }
        self._write_event(event)
    
    def log_ping_sent(self, counter, payload):
        """Logga invio ping"""
        self.log_event("PING_SENT", {
            "counter": counter,
            "payload": payload,
            "target_id": TRANSPONDER_ID
        })
    
    def log_delivery(self, success, target_id, counter):
        """Logga risultato delivery"""
        self.log_event("DELIVERY_RESULT", {
            "success": success,
            "target_id": target_id,
            "counter": counter
        })
    
    def log_distance(self, prop_time_ms, distance_m, counter):
        """Logga stima distanza da AT?T"""
        self.log_event("DISTANCE_ESTIMATION", {
            "propagation_time_ms": prop_time_ms,
            "one_way_time_ms": prop_time_ms / 2,
            "distance_m": distance_m,
            "sound_speed_mps": SPEED_OF_SOUND_MPS,
            "counter": counter
        })
    
    def log_usbllong(self, data, counter):
        """Logga dati USBLLONG"""
        self.log_event("USBLLONG", {
            "counter": counter,
            "target_id": data.get("target_id"),
            "east_m": data.get("east"),
            "north_m": data.get("north"),
            "up_m": data.get("up"),
            "range_m": data.get("range"),
            "uncertainty_m": data.get("uncertainty"),
            "rssi_dbm": data.get("rssi"),
            "integrity": data.get("integrity"),
            "propagation_time_us": data.get("propagation_time_us"),
            "valid": data.get("valid")
        })
    
    def log_usblangles(self, data, counter):
        """Logga dati USBLANGLES"""
        self.log_event("USBLANGLES", {
            "counter": counter,
            "target_id": data.get("target_id"),
            "bearing_rad": data.get("bearing"),
            "elevation_rad": data.get("elevation"),
            "bearing_deg": data.get("bearing_deg"),
            "elevation_deg": data.get("elevation_deg"),
            "rssi_dbm": data.get("rssi"),
            "integrity": data.get("integrity"),
            "accuracy_m": data.get("accuracy"),
            "valid": data.get("valid")
        })
    
    def log_session_end(self, total_messages, last_distance, last_prop_time):
        """Logga fine sessione con summary"""
        session_end = datetime.now()
        duration = (session_end - self.session_start).total_seconds() if self.session_start else 0
        
        self.log_event("SESSION_END", {
            "session_duration_s": duration,
            "total_messages_sent": total_messages,
            "last_distance_m": last_distance,
            "last_propagation_time_ms": last_prop_time
        })
    
    def close(self):
        """Chiudi il file di log"""
        if self.log_file and not self.log_file.closed:
            self.log_file.close()
            print(f"✓ Log saved to: {self.log_filepath}")


# State
state = {
    "message_counter": 0,
    "last_distance": None,
    "last_propagation_time": None,
    "waiting_for_delivery": False,
    "delivery_event": asyncio.Event(),
    "logger": None,  # Inizializzato in main()
    "waiting_for_prop_time": False,
    "prop_time_value": None,
    "prop_time_event": asyncio.Event()
}


def format_timestamp():
    """Ritorna timestamp corrente in formato leggibile"""
    return datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]


def calculate_distance(propagation_time_ms):
    """
    Calcola distanza da propagation time
    
    Args:
        propagation_time_ms: Tempo di propagazione in millisecondi
        
    Returns:
        Distanza in metri
    """
    # Propagation time è round-trip, quindi dividi per 2
    one_way_time_s = (propagation_time_ms / 1000.0) / 2.0
    distance = one_way_time_s * SPEED_OF_SOUND_MPS
    return distance


def decode_response(message: bytes) -> tuple[str, dict]:
    """Decodifica i messaggi dal modem"""
    try:
        str_msg = message.decode(CODEC).strip()
        if not str_msg:
            return "EMPTY", {}
        
        # Gestione risposte semplici
        if str_msg == "OK":
            return "OK", {}
        
        if not str_msg.startswith("+++"):
            # Potrebbe essere una risposta diretta (es. numero per AT?T)
            return "DIRECT_VALUE", {"value": str_msg}
        
        parts = str_msg.split(":")
        
        if "AT" in str_msg:
            if len(parts) >= 3:
                args = parts[2].split(",")
                keyword = args[0]
                
                # DELIVEREDIM
                if keyword == "DELIVEREDIM":
                    return "DELIVEREDIM", {"target_id": args[1] if len(args) > 1 else "?"}
                
                # FAILEDIM
                elif keyword == "FAILEDIM":
                    return "FAILEDIM", {"target_id": args[1] if len(args) > 1 else "?"}
                
                # USBLLONG (formato completo come da manuale)
                elif keyword == "USBLLONG" and len(args) >= 17:
                    try:
                        current_time = float(args[1].split(",")[0])
                        measurement_time = args[1].split(",")[1]
                        target_id = args[3]
                        east = float(args[7])
                        north = float(args[8])
                        up = float(args[9])
                        propagation_time_us = int(args[13])
                        rssi = int(args[14])
                        integrity = int(args[15])
                        uncertainty = float(args[16])
                        
                        # Calcola range da propagation time
                        slant_range = (propagation_time_us / 1_000_000.0) * SPEED_OF_SOUND_MPS
                        
                        return "USBLLONG", {
                            "target_id": target_id,
                            "east": east,
                            "north": north,
                            "up": up,
                            "range": slant_range,
                            "uncertainty": uncertainty,
                            "rssi": rssi,
                            "integrity": integrity,
                            "propagation_time_us": propagation_time_us,
                            "valid": integrity > MIN_INTEGRITY
                        }
                    except (ValueError, IndexError) as e:
                        logging.debug(f"Error parsing USBLLONG: {e}")
                        return "USBLLONG_ERROR", {}
                
                # USBLANGLES (formato corretto da manuale)
                elif keyword == "USBLANGLES" and len(args) >= 13:
                    try:
                        target_id = args[3]
                        lbearing = float(args[4])
                        lelevation = float(args[5])
                        bearing = float(args[6])
                        elevation = float(args[7])
                        rssi = int(args[11])
                        integrity = int(args[12])
                        accuracy = float(args[13])
                        
                        # Converti angoli in gradi per visualizzazione
                        import math
                        bearing_deg = math.degrees(bearing)
                        elevation_deg = math.degrees(elevation)
                        
                        return "USBLANGLES", {
                            "target_id": target_id,
                            "bearing": bearing,
                            "elevation": elevation,
                            "bearing_deg": bearing_deg,
                            "elevation_deg": elevation_deg,
                            "rssi": rssi,
                            "integrity": integrity,
                            "accuracy": accuracy,
                            "valid": integrity > MIN_INTEGRITY
                        }
                    except (ValueError, IndexError) as e:
                        logging.debug(f"Error parsing USBLANGLES: {e}")
                        return "USBLANGLES_ERROR", {}
                
                elif keyword in ["RECVIM", "SENDSTART", "SENDEND", "RECVSTART", "RECVEND"]:
                    return keyword, {"args": args}
        
        else:
            # Risposta AT command (tipo AT*SENDIM -> OK o ERROR)
            cmd = parts[0].replace("+", "")
            if len(parts) >= 3:
                return f"{cmd}_RESPONSE", {"status": parts[2]}
        
        return "OTHER", {"message": str_msg}
        
    except Exception as e:
        logging.debug(f"Parse error: {e}")
        return "ERROR", {}


async def send_command(writer: asyncio.StreamWriter, command: str):
    """
    Invia un comando AT senza aspettare risposta
    La risposta sarà gestita da response_listener
    """
    cmd_bytes = f"+++{command}\n".encode(CODEC)
    writer.write(cmd_bytes)
    await writer.drain()


async def query_propagation_time(writer: asyncio.StreamWriter) -> float:
    """
    Interroga il modem per il propagation time usando AT?T
    
    Returns:
        Propagation time in millisecondi, o None se non disponibile
    """
    state["waiting_for_prop_time"] = True
    state["prop_time_value"] = None
    state["prop_time_event"].clear()
    
    # Invia comando
    await send_command(writer, "AT?T")
    
    # Aspetta risposta (gestita da response_listener)
    try:
        await asyncio.wait_for(state["prop_time_event"].wait(), timeout=RESPONSE_TIMEOUT)
        return state["prop_time_value"]
    except asyncio.TimeoutError:
        logging.warning("Timeout waiting for AT?T response")
        state["waiting_for_prop_time"] = False
        return None


async def send_ping_message(writer: asyncio.StreamWriter) -> bool:
    """
    Invia un messaggio ping con ACK contenente timestamp e counter
    
    Returns:
        True se il messaggio è stato consegnato, False altrimenti
    """
    state["message_counter"] += 1
    
    # Crea payload JSON
    payload = {
        "timestamp": time.time(),
        "counter": state["message_counter"]
    }
    payload_str = json.dumps(payload, separators=(',', ':'))
    
    # Comando AT*SENDIM con ACK
    command = f"AT*SENDIM,{len(payload_str)},{TRANSPONDER_ID},ack,{payload_str}"
    
    print(f"\n{'='*70}")
    print(f"[{format_timestamp()}] PING #{state['message_counter']}")
    print(f"{'='*70}")
    print(f"Sending message to ID {TRANSPONDER_ID}: {payload}")
    
    # Log evento
    if state["logger"]:
        state["logger"].log_ping_sent(state["message_counter"], payload)
    
    # Invia comando (senza aspettare risposta qui)
    await send_command(writer, command)
    
    print(f"✓ Message sent")
    
    # Aspetta DELIVEREDIM
    state["waiting_for_delivery"] = True
    state["delivery_event"].clear()
    
    try:
        await asyncio.wait_for(state["delivery_event"].wait(), timeout=10.0)
        return True
    except asyncio.TimeoutError:
        print(f"⏱️ Timeout waiting for delivery confirmation")
        state["waiting_for_delivery"] = False
        return False


async def message_sender(writer: asyncio.StreamWriter):
    """Task che invia messaggi periodicamente"""
    
    # Aspetta un po' prima di iniziare
    await asyncio.sleep(2.0)
    
    while True:
        try:
            # Invia messaggio
            delivered = await send_ping_message(writer)
            
            if delivered:
                # Interroga propagation time
                print(f"→ Querying propagation time (AT?T)...")
                prop_time_ms = await query_propagation_time(writer)
                
                if prop_time_ms is not None:
                    distance = calculate_distance(prop_time_ms)
                    
                    state["last_propagation_time"] = prop_time_ms
                    state["last_distance"] = distance
                    
                    # Log evento
                    if state["logger"]:
                        state["logger"].log_distance(
                            prop_time_ms, 
                            distance, 
                            state["message_counter"]
                        )
                    
                    print(f"\n📏 DISTANCE ESTIMATION:")
                    print(f"   Propagation time: {prop_time_ms:.2f} ms")
                    print(f"   One-way time:     {prop_time_ms/2:.2f} ms")
                    print(f"   Distance:         {distance:.2f} m")
                    print(f"   (using sound speed: {SPEED_OF_SOUND_MPS} m/s)")
                else:
                    print(f"⚠️ Could not retrieve propagation time")
            
            print(f"\n⏳ Waiting {MESSAGE_INTERVAL}s until next ping...")
            print(f"{'='*70}\n")
            
            # Aspetta prima del prossimo invio
            await asyncio.sleep(MESSAGE_INTERVAL)
            
        except Exception as e:
            logging.error(f"Error in message sender: {e}")
            await asyncio.sleep(MESSAGE_INTERVAL)


async def response_listener(reader: asyncio.StreamReader):
    """Task che ascolta continuamente le risposte del modem"""
    
    try:
        while True:
            line = await reader.readline()
            if not line:
                logging.warning("Connection closed")
                break
            
            msg_type, data = decode_response(line)
            
            # Risposta diretta AT?T (propagation time)
            if msg_type == "DIRECT_VALUE" and state["waiting_for_prop_time"]:
                try:
                    prop_time = float(data["value"])
                    state["prop_time_value"] = prop_time
                    state["waiting_for_prop_time"] = False
                    state["prop_time_event"].set()
                except (ValueError, KeyError):
                    logging.warning(f"Could not parse propagation time: {data}")
                    state["waiting_for_prop_time"] = False
                    state["prop_time_event"].set()
            
            # DELIVEREDIM - messaggio consegnato
            elif msg_type == "DELIVEREDIM":
                if state["waiting_for_delivery"]:
                    print(f"✓ Message delivered to transponder ID {data['target_id']}")
                    
                    # Log evento
                    if state["logger"]:
                        state["logger"].log_delivery(
                            success=True,
                            target_id=data['target_id'],
                            counter=state["message_counter"]
                        )
                    
                    state["waiting_for_delivery"] = False
                    state["delivery_event"].set()
            
            # FAILEDIM - messaggio fallito
            elif msg_type == "FAILEDIM":
                if state["waiting_for_delivery"]:
                    print(f"❌ Message delivery FAILED to ID {data['target_id']}")
                    
                    # Log evento
                    if state["logger"]:
                        state["logger"].log_delivery(
                            success=False,
                            target_id=data['target_id'],
                            counter=state["message_counter"]
                        )
                    
                    state["waiting_for_delivery"] = False
                    state["delivery_event"].set()
            
            # USBLLONG - dati di posizionamento completi
            elif msg_type == "USBLLONG":
                if data["valid"]:
                    # Log evento
                    if state["logger"]:
                        state["logger"].log_usbllong(data, state["message_counter"])
                    
                    print(f"\n📍 USBL POSITIONING (USBLLONG):")
                    print(f"   Target ID:        {data['target_id']}")
                    print(f"   East (E):         {data['east']:>8.2f} m")
                    print(f"   North (N):        {data['north']:>8.2f} m")
                    print(f"   Up (U):           {data['up']:>8.2f} m")
                    print(f"   Slant Range:      {data['range']:>8.2f} m")
                    print(f"   Uncertainty:      ±{data['uncertainty']:.2f} m")
                    print(f"   RSSI:             {data['rssi']} dBm")
                    print(f"   Integrity:        {data['integrity']}")
                    
                    # Calcola distanza 3D
                    import math
                    dist_3d = math.sqrt(data['east']**2 + data['north']**2 + data['up']**2)
                    print(f"   3D Distance:      {dist_3d:.2f} m")
                else:
                    print(f"\n⚠️ USBLLONG received but INVALID (low integrity: {data.get('integrity', 'N/A')})")
            
            # USBLANGLES - solo direzione (nessuna distanza)
            elif msg_type == "USBLANGLES":
                if data["valid"]:
                    # Log evento
                    if state["logger"]:
                        state["logger"].log_usblangles(data, state["message_counter"])
                    
                    print(f"\n📐 USBL DIRECTION (USBLANGLES):")
                    print(f"   Target ID:        {data['target_id']}")
                    print(f"   Bearing:          {data['bearing_deg']:>8.2f}°")
                    print(f"   Elevation:        {data['elevation_deg']:>8.2f}°")
                    print(f"   RSSI:             {data['rssi']} dBm")
                    print(f"   Integrity:        {data['integrity']}")
                    print(f"   Accuracy:         ±{data['accuracy']:.2f} m")
                    print(f"   Note: No range available (direction only)")
                else:
                    print(f"\n⚠️ USBLANGLES received but INVALID (low integrity: {data.get('integrity', 'N/A')})")
            
            # Altri messaggi (debug)
            elif msg_type not in ["OK", "EMPTY", "OTHER"]:
                logging.debug(f"Received: {msg_type} - {data}")
                
    except asyncio.CancelledError:
        pass
    except Exception as e:
        logging.error(f"Response listener error: {e}")


async def main():
    # Setup logging
    logging.basicConfig(
        level=logging.INFO,
        format="%(message)s"
    )
    
    # Inizializza JSON logger
    state["logger"] = JSONLogger(
        directory=LOG_DIRECTORY,
        prefix=LOG_FILENAME_PREFIX,
        enabled=ENABLE_JSON_LOGGING
    )
    
    print("="*70)
    print("DISTANCE TRACKER - Ping with ACK + AT?T + USBL")
    print("="*70)
    print(f"Transceiver ID: {TRANSCEIVER_ID}")
    print(f"Transponder ID: {TRANSPONDER_ID}")
    print(f"USBL IP:        {USBL_IP}:{USBL_PORT}")
    print(f"Sound Speed:    {SPEED_OF_SOUND_MPS} m/s")
    print(f"Ping Interval:  {MESSAGE_INTERVAL} seconds")
    print(f"JSON Logging:   {'ENABLED' if ENABLE_JSON_LOGGING else 'DISABLED'}")
    if ENABLE_JSON_LOGGING:
        print(f"Log Directory:  {LOG_DIRECTORY}")
    print("="*70)
    
    # Connetti al modem
    try:
        reader, writer = await asyncio.open_connection(USBL_IP, USBL_PORT)
        print(f"✓ Connected to USBL modem")
        print()
    except Exception as e:
        print(f"❌ Failed to connect: {e}")
        return
    
    # Avvia tasks
    listener_task = asyncio.create_task(response_listener(reader))
    sender_task = asyncio.create_task(message_sender(writer))
    
    try:
        await asyncio.gather(listener_task, sender_task)
    except KeyboardInterrupt:
        print("\n\n⏹️ Stopped by user")
    finally:
        listener_task.cancel()
        sender_task.cancel()
        
        try:
            await asyncio.gather(listener_task, sender_task)
        except asyncio.CancelledError:
            pass
        
        writer.close()
        await writer.wait_closed()
        
        # Log fine sessione
        if state["logger"]:
            state["logger"].log_session_end(
                total_messages=state['message_counter'],
                last_distance=state['last_distance'],
                last_prop_time=state['last_propagation_time']
            )
            state["logger"].close()
        
        # Summary finale
        print("\n" + "="*70)
        print("FINAL SUMMARY")
        print("="*70)
        print(f"Total messages sent:    {state['message_counter']}")
        if state['last_distance'] is not None:
            print(f"Last distance:          {state['last_distance']:.2f} m")
            print(f"Last propagation time:  {state['last_propagation_time']:.2f} ms")
        print("="*70)


if __name__ == '__main__':
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass