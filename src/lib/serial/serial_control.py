#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Serial Control Library
Gestisce una porta seriale per inviare i dati di controllo ricevuti dal transceiver.
Invia messaggi JSON con enable, bias e frequenza.

Uso:
    serial_ctrl = SerialControlThread(
        port="/dev/ttyS0",
        baudrate=115200
    )
    serial_ctrl.start()
    
    # Aggiorna i dati da inviare
    serial_ctrl.update_control_data(enable=True, bias_deg=10.0, freq_hz=0.5)
    
    # Oppure usa il callback
    transponder.on_control_data_received = serial_ctrl.on_control_data_received
"""

import threading
import json
import time
import queue

try:
    import serial
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False


class SerialControl(object):
    """
    Gestisce la comunicazione seriale per inviare control data in JSON.
    """
    
    def __init__(self, port="/dev/ttyS0", baudrate=115200, timeout=1.0):
        """
        Args:
            port: Porta seriale (es. "/dev/ttyS0", "COM3")
            baudrate: Baud rate (default 115200)
            timeout: Timeout per operazioni seriali [s]
        """
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        
        self.serial = None
        self.connected = False
        
        # Ultimo control data ricevuto
        self.last_control_data = {
            'enable': False,
            'bias_angle_deg': 0.0,
            'frequency_hz': 0.1,
            'use_range_only': False,
            'timestamp': 0.0
        }
        
        # Stats
        self.messages_sent = 0
        self.errors = 0
    
    def connect(self):
        """Apre la connessione seriale."""
        if not SERIAL_AVAILABLE:
            print("[SERIAL] pyserial non disponibile")
            return False
        
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE
            )
            self.connected = True
            print("[SERIAL] Connesso a {} @ {} baud".format(self.port, self.baudrate))
            return True
        except Exception as e:
            print("[SERIAL] Errore connessione: {}".format(e))
            self.connected = False
            return False
    
    def disconnect(self):
        """Chiude la connessione seriale."""
        if self.serial and self.serial.is_open:
            self.serial.close()
        self.connected = False
        print("[SERIAL] Disconnesso")
    
    def send_control_data(self, enable, bias_deg, freq_hz, use_range_only=False):
        """
        Invia i dati di controllo via seriale in formato JSON.
        
        Args:
            enable: Booleano abilitazione
            bias_deg: Angolo bias in gradi
            freq_hz: Frequenza in Hz
            use_range_only: Booleano per modalita' filtro
        
        Returns:
            True se inviato con successo, False altrimenti
        """
        if not self.connected or not self.serial or not self.serial.is_open:
            return False
        
        # Aggiorna stato interno
        self.last_control_data = {
            'enable': enable,
            'bias_angle_deg': bias_deg,
            'frequency_hz': freq_hz,
            'use_range_only': use_range_only,
            'timestamp': time.time()
        }
        
        # Costruisci messaggio JSON
        message = {
            'enable': enable,
            'bias_deg': bias_deg,
            'freq_hz': freq_hz,
            'use_range_only': use_range_only
        }
        
        try:
            # Serializza e invia con newline come terminatore
            json_str = json.dumps(message, separators=(',', ':'))
            data = (json_str + '\n').encode('utf-8')
            
            self.serial.write(data)
            self.serial.flush()
            self.messages_sent += 1
            
            print("[SERIAL TX] {}".format(json_str))
            return True
            
        except Exception as e:
            print("[SERIAL] Errore invio: {}".format(e))
            self.errors += 1
            return False
    
    def get_stats(self):
        """Ritorna statistiche."""
        return {
            'connected': self.connected,
            'messages_sent': self.messages_sent,
            'errors': self.errors
        }


class SerialControlThread(object):
    """
    Thread wrapper per SerialControl.
    Gestisce l'invio asincrono dei control data via seriale.
    """
    
    def __init__(self, port="/dev/ttyS0", baudrate=115200, timeout=1.0):
        """
        Args:
            port: Porta seriale
            baudrate: Baud rate
            timeout: Timeout operazioni
        """
        self.serial_ctrl = SerialControl(port, baudrate, timeout)
        
        # Queue per i messaggi da inviare
        self.send_queue = queue.Queue()
        
        # Thread
        self.thread = None
        self.running = False
        
        # Lock per accesso thread-safe
        self.lock = threading.Lock()
    
    def start(self):
        """Avvia il thread seriale."""
        if not self.serial_ctrl.connect():
            print("[SERIAL THREAD] Connessione fallita, thread non avviato")
            return False
        
        self.running = True
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()
        print("[SERIAL THREAD] Avviato")
        return True
    
    def stop(self):
        """Ferma il thread seriale."""
        self.running = False
        
        # Sblocca la queue se in attesa
        self.send_queue.put(None)
        
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=2.0)
        
        self.serial_ctrl.disconnect()
        print("[SERIAL THREAD] Fermato - Messaggi inviati: {}".format(
            self.serial_ctrl.messages_sent))
    
    def _run(self):
        """Loop principale del thread."""
        while self.running:
            try:
                # Attendi un messaggio dalla queue (con timeout per poter uscire)
                try:
                    item = self.send_queue.get(timeout=1.0)
                except queue.Empty:
                    continue
                
                if item is None:
                    # Segnale di stop
                    break
                
                # Invia il messaggio
                enable = item.get('enable', False)
                bias_deg = item.get('bias_angle_deg', 0.0)
                freq_hz = item.get('frequency_hz', 0.1)
                use_range_only = item.get('use_range_only', False)
                
                self.serial_ctrl.send_control_data(
                    enable, bias_deg, freq_hz, use_range_only
                )
                
            except Exception as e:
                print("[SERIAL THREAD] Errore: {}".format(e))
    
    def update_control_data(self, enable, bias_deg, freq_hz, use_range_only=False):
        """
        Aggiorna i dati di controllo da inviare (thread-safe).
        
        Args:
            enable: Booleano abilitazione
            bias_deg: Angolo bias in gradi
            freq_hz: Frequenza in Hz
            use_range_only: Booleano per modalita' filtro
        """
        control_data = {
            'enable': enable,
            'bias_angle_deg': bias_deg,
            'frequency_hz': freq_hz,
            'use_range_only': use_range_only
        }
        
        self.send_queue.put(control_data)
    
    def on_control_data_received(self, control_data):
        """
        Callback da usare con USBLTransponderThread.
        Quando arrivano nuovi control data dal transceiver, li invia via seriale.
        
        Args:
            control_data: dict con enable, bias_angle_deg, frequency_hz, use_range_only
        """
        self.send_queue.put(control_data)
        
        print("[SERIAL THREAD] Control data ricevuto -> in coda per invio")
        print("  enable={}, bias={:.2f}deg, freq={:.2f}Hz, use_range_only={}".format(
            control_data.get('enable', False),
            control_data.get('bias_angle_deg', 0.0),
            control_data.get('frequency_hz', 0.1),
            control_data.get('use_range_only', False)
        ))
    
    def get_stats(self):
        """Ritorna statistiche."""
        return self.serial_ctrl.get_stats()


# =============================================================================
#  TEST / ESEMPIO STANDALONE
# =============================================================================
if __name__ == "__main__":
    print("=" * 50)
    print("Serial Control Test")
    print("=" * 50)
    
    # Configurazione - modifica questi parametri
    SERIAL_PORT = "/dev/ttyS0"
    SERIAL_BAUDRATE = 115200
    
    # Crea e avvia il thread seriale
    serial_thread = SerialControlThread(
        port=SERIAL_PORT,
        baudrate=SERIAL_BAUDRATE
    )
    
    if serial_thread.start():
        print("\nTest invio messaggi...")
        
        try:
            # Simula ricezione control data
            test_data = [
                {'enable': True, 'bias_angle_deg': 0.0, 'frequency_hz': 0.5, 'use_range_only': False},
                {'enable': True, 'bias_angle_deg': 5.0, 'frequency_hz': 1.0, 'use_range_only': True},
                {'enable': False, 'bias_angle_deg': 0.0, 'frequency_hz': 0.5, 'use_range_only': False},
            ]
            
            for data in test_data:
                serial_thread.on_control_data_received(data)
                time.sleep(1.0)
            
            # Aspetta che i messaggi vengano inviati
            time.sleep(2.0)
            
        except KeyboardInterrupt:
            print("\nInterrotto")
        finally:
            serial_thread.stop()
    else:
        print("Impossibile avviare - verifica la porta seriale")
    
    print("\nStats: {}".format(serial_thread.get_stats()))