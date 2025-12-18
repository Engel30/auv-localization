#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Serial Control Library
Gestisce una porta seriale per inviare i dati di controllo ricevuti dal transceiver.
Invia messaggi in formato CSV: "freq_hz,bias_deg"

Uso:
    serial_ctrl = SerialControlThread(
        port="/dev/ttyS0",
        baudrate=115200
    )
    serial_ctrl.start()
    
    # Aggiorna i dati da inviare
    serial_ctrl.update_control_data(bias_deg=10.0, freq_hz=0.5)
    
    # Imposta valori fissi
    serial_ctrl.set_fixed_values(bias_deg=15.0, freq_hz=None)  # None = usa il valore ricevuto
    
    # Oppure usa il callback
    transponder.on_control_data_received = serial_ctrl.on_control_data_received
"""

import threading
import time
import queue

try:
    import serial
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False


class SerialControl(object):
    """
    Gestisce la comunicazione seriale per inviare control data in formato CSV.
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
        
        # Valori fissi (None = usa il valore ricevuto)
        self.fixed_bias_deg = None
        self.fixed_freq_hz = None
        
        # Ultimo control data ricevuto
        self.last_control_data = {
            'bias_angle_deg': 0.0,
            'frequency_hz': 0.1,
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
    
    def set_fixed_values(self, bias_deg=None, freq_hz=None):
        """
        Imposta valori fissi da usare al posto di quelli ricevuti.
        
        Args:
            bias_deg: Valore fisso per bias (None = usa valore ricevuto)
            freq_hz: Valore fisso per frequenza (None = usa valore ricevuto)
        """
        self.fixed_bias_deg = bias_deg
        self.fixed_freq_hz = freq_hz
        
        print("[SERIAL] Valori fissi impostati: bias={}, freq={}".format(
            bias_deg if bias_deg is not None else "AUTO",
            freq_hz if freq_hz is not None else "AUTO"
        ))
    
    def send_control_data(self, bias_deg, freq_hz):
        """
        Invia i dati di controllo via seriale in formato CSV: "freq_hz,bias_deg"
        
        Args:
            bias_deg: Angolo bias in gradi
            freq_hz: Frequenza in Hz
        
        Returns:
            True se inviato con successo, False altrimenti
        """
        if not self.connected or not self.serial or not self.serial.is_open:
            return False
        
        # Applica valori fissi se impostati
        final_bias = self.fixed_bias_deg if self.fixed_bias_deg is not None else bias_deg
        final_freq = self.fixed_freq_hz if self.fixed_freq_hz is not None else freq_hz
        
        # Aggiorna stato interno
        self.last_control_data = {
            'bias_angle_deg': final_bias,
            'frequency_hz': final_freq,
            'timestamp': time.time()
        }
        
        try:
            # Costruisci stringa CSV: "freq_hz,bias_deg\n"
            csv_str = "{:.2f},{:.2f}".format(final_freq, final_bias)
            data = (csv_str + '\n').encode('utf-8')
            
            self.serial.write(data)
            self.serial.flush()
            self.messages_sent += 1
            
            print("[SERIAL TX] {}".format(csv_str))
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
            'errors': self.errors,
            'fixed_bias_deg': self.fixed_bias_deg,
            'fixed_freq_hz': self.fixed_freq_hz
        }


class SerialControlThread(object):
    """
    Thread wrapper per SerialControl.
    Gestisce l'invio asincrono dei control data via seriale.
    """
    
    def __init__(self, port="/dev/ttyS0", baudrate=115200, timeout=1.0,
                 initial_bias=0.0, initial_freq=2.0, periodic_send=False, send_interval=1.0):
        """
        Args:
            port: Porta seriale
            baudrate: Baud rate
            timeout: Timeout operazioni
            initial_bias: Valore iniziale bias da inviare all'avvio [deg]
            initial_freq: Valore iniziale frequenza da inviare all'avvio [Hz]
            periodic_send: Se True, invia periodicamente l'ultimo valore
            send_interval: Intervallo di invio periodico [s]
        """
        self.serial_ctrl = SerialControl(port, baudrate, timeout)
        
        # Queue per i messaggi da inviare
        self.send_queue = queue.Queue()
        
        # Thread
        self.thread = None
        self.running = False
        
        # Lock per accesso thread-safe
        self.lock = threading.Lock()
        
        # Valori iniziali e modalità
        self.initial_bias = initial_bias
        self.initial_freq = initial_freq
        self.periodic_send = periodic_send
        self.send_interval = send_interval
        
        # Ultimo valore ricevuto
        self.last_bias = initial_bias
        self.last_freq = initial_freq
    
    def start(self):
        """Avvia il thread seriale."""
        if not self.serial_ctrl.connect():
            print("[SERIAL THREAD] Connessione fallita, thread non avviato")
            return False
        
        self.running = True
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()
        
        # Invia immediatamente i valori iniziali
        self.send_now(self.initial_bias, self.initial_freq)
        
        print("[SERIAL THREAD] Avviato - Valori iniziali inviati: bias={:.2f}deg, freq={:.2f}Hz".format(
            self.initial_bias, self.initial_freq))
        
        if self.periodic_send:
            print("[SERIAL THREAD] Modalita' periodic send attiva (intervallo={:.1f}s)".format(
                self.send_interval))
        
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
        last_send_time = time.time()
        
        while self.running:
            try:
                # Calcola timeout dinamico per periodic send
                if self.periodic_send:
                    time_since_last = time.time() - last_send_time
                    timeout = max(0.1, self.send_interval - time_since_last)
                else:
                    timeout = 1.0
                
                # Attendi un messaggio dalla queue
                try:
                    item = self.send_queue.get(timeout=timeout)
                except queue.Empty:
                    # Se in modalità periodic send, invia l'ultimo valore
                    if self.periodic_send and (time.time() - last_send_time) >= self.send_interval:
                        with self.lock:
                            self.serial_ctrl.send_control_data(self.last_bias, self.last_freq)
                        last_send_time = time.time()
                    continue
                
                if item is None:
                    # Segnale di stop
                    break
                
                # Aggiorna e invia il messaggio
                bias_deg = item.get('bias_angle_deg', 0.0)
                freq_hz = item.get('frequency_hz', 0.1)
                
                with self.lock:
                    self.last_bias = bias_deg
                    self.last_freq = freq_hz
                
                self.serial_ctrl.send_control_data(bias_deg, freq_hz)
                last_send_time = time.time()
                
            except Exception as e:
                print("[SERIAL THREAD] Errore: {}".format(e))
    
    def set_fixed_values(self, bias_deg=None, freq_hz=None):
        """
        Imposta valori fissi da usare al posto di quelli ricevuti.
        Thread-safe.
        
        Args:
            bias_deg: Valore fisso per bias (None = usa valore ricevuto)
            freq_hz: Valore fisso per frequenza (None = usa valore ricevuto)
        """
        with self.lock:
            self.serial_ctrl.set_fixed_values(bias_deg, freq_hz)
    
    def send_now(self, bias_deg=None, freq_hz=None):
        """
        Forza invio immediato di un messaggio (bypassa la queue).
        Se i parametri non sono specificati, usa gli ultimi valori.
        
        Args:
            bias_deg: Angolo bias in gradi (None = usa ultimo valore)
            freq_hz: Frequenza in Hz (None = usa ultimo valore)
        """
        if bias_deg is None:
            bias_deg = self.last_bias
        if freq_hz is None:
            freq_hz = self.last_freq
        
        with self.lock:
            self.last_bias = bias_deg
            self.last_freq = freq_hz
            self.serial_ctrl.send_control_data(bias_deg, freq_hz)
    
    def update_control_data(self, bias_deg, freq_hz):
        """
        Aggiorna i dati di controllo da inviare (thread-safe).
        
        Args:
            bias_deg: Angolo bias in gradi
            freq_hz: Frequenza in Hz
        """
        control_data = {
            'bias_angle_deg': bias_deg,
            'frequency_hz': freq_hz
        }
        
        self.send_queue.put(control_data)
    
    def on_control_data_received(self, control_data):
        """
        Callback da usare con USBLTransponderThread.
        Quando arrivano nuovi control data dal transceiver, li invia via seriale.
        
        Args:
            control_data: dict con bias_angle_deg e frequency_hz
        """
        self.send_queue.put(control_data)
        
        print("[SERIAL THREAD] Control data ricevuto -> in coda per invio")
        print("  bias={:.2f}deg, freq={:.2f}Hz".format(
            control_data.get('bias_angle_deg', 0.0),
            control_data.get('frequency_hz', 0.1)
        ))
    
    def get_stats(self):
        """Ritorna statistiche."""
        return self.serial_ctrl.get_stats()


# =============================================================================
#  TEST / ESEMPIO STANDALONE
# =============================================================================
if __name__ == "__main__":
    print("=" * 50)
    print("Serial Control Test - Formato CSV")
    print("=" * 50)
    
    # Configurazione - modifica questi parametri
    SERIAL_PORT = "/dev/ttyS0"
    SERIAL_BAUDRATE = 115200
    
    # Crea il thread seriale con valori iniziali
    serial_thread = SerialControlThread(
        port=SERIAL_PORT,
        baudrate=SERIAL_BAUDRATE,
        initial_bias=0.0,      # Valore iniziale bias
        initial_freq=0.5,      # Valore iniziale frequenza
        periodic_send=False,   # True per invio continuo
        send_interval=2.0      # Intervallo se periodic_send=True
    )
    
    # Esempio: imposta valori fissi (opzionale)
    # serial_thread.set_fixed_values(bias_deg=10.0, freq_hz=None)
    
    if serial_thread.start():
        print("\nThread avviato - valore iniziale gia' inviato automaticamente!")
        print("Test invio messaggi aggiuntivi...")
        
        try:
            # Simula ricezione control data
            test_data = [
                {'bias_angle_deg': 5.0, 'frequency_hz': 1.0},
                {'bias_angle_deg': -10.0, 'frequency_hz': 0.75},
                {'bias_angle_deg': 0.0, 'frequency_hz': 0.5},
            ]
            
            time.sleep(1.0)
            
            for data in test_data:
                serial_thread.on_control_data_received(data)
                time.sleep(1.5)
            
            # Test invio forzato immediato
            print("\n--- Test send_now (invio immediato) ---")
            serial_thread.send_now(bias_deg=20.0, freq_hz=2.0)
            time.sleep(0.5)
            serial_thread.send_now(bias_deg=-5.0, freq_hz=0.3)
            
            # Test con valori fissi
            print("\n--- Test con bias fisso a 15.0 ---")
            serial_thread.set_fixed_values(bias_deg=15.0, freq_hz=None)
            time.sleep(0.5)
            
            for data in test_data[:2]:
                serial_thread.on_control_data_received(data)
                time.sleep(1.0)
            
            # Aspetta che i messaggi vengano inviati
            time.sleep(2.0)
            
        except KeyboardInterrupt:
            print("\nInterrotto")
        finally:
            serial_thread.stop()
    else:
        print("Impossibile avviare - verifica la porta seriale e i permessi")
        print("Esegui: sudo usermod -a -G dialout $USER")
    
    print("\nStats: {}".format(serial_thread.get_stats()))