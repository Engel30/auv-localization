#!/usr/bin/env python3
import time
import datetime
from openUPS2_smbus2 import openups2

def test_ups(tipo="LIFEP04", intervallo=5, durata=60):
    """
    Test acquisizione dati openUPS2 con smbus2
    
    Args:
        tipo: tipo batteria ("LIFEP04" o "LI-ON")
        intervallo: secondi tra acquisizioni
        durata: durata test in secondi (0 = infinito)
    """
    
    # Inizializza UPS
    ups = openups2(bus=1, tipo=tipo)
    
    if ups._bus is None:
        print("Errore: impossibile inizializzare il bus I2C")
        return
    
    # Crea file di log
    log_file = f"openups2_log_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.txt"
    
    print(f"Inizio test - Log: {log_file}")
    print(f"Tipo batteria: {tipo}")
    print(f"Intervallo acquisizione: {intervallo}s")
    print(f"Durata: {durata}s" if durata > 0 else "Durata: continuo (CTRL+C per terminare)")
    print("-" * 70)
    
    start_time = time.time()
    
    with open(log_file, 'w') as f:
        # Header
        f.write("# openUPS2 Log (smbus2)\n")
        f.write(f"# Tipo batteria: {tipo}\n")
        f.write(f"# Timestamp, Voltage(V), Current(A), Carica_Rel(%), Carica_Abs(%), Errore\n")
        
        try:
            while True:
                # Acquisizione dati
                voltage = ups.read_volt()
                current = ups.read_curr()
                rel_charge, abs_charge, err = ups.read_charge()
                
                timestamp = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')
                
                # Scrittura log
                log_line = f"{timestamp}, {voltage}, {current}, {rel_charge}, {abs_charge}, {err}\n"
                f.write(log_line)
                f.flush()
                
                # Stampa a video
                print(f"{timestamp} | V:{voltage:6.2f} | I:{current:6.2f} | Rel:{rel_charge:3d}% | Abs:{abs_charge:3d}% | Err:{err}")
                
                # Verifica durata
                if durata > 0 and (time.time() - start_time) >= durata:
                    break
                
                time.sleep(intervallo)
                
        except KeyboardInterrupt:
            print("\n\nTest interrotto dall'utente")
        except Exception as e:
            print(f"\n\nErrore durante l'acquisizione: {e}")
        finally:
            ups.close()
    
    print(f"\nTest completato. Log salvato in: {log_file}")


if __name__ == "__main__":
    # Test con batteria LIFEP04, acquisizione ogni 5 secondi per 60 secondi
    # Per test continuo impostare durata=0
    test_ups(tipo="LIFEP04", intervallo=5, durata=60)