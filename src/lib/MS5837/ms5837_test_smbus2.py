#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import ms5837_smbus2
import time
import logging
from datetime import datetime

# Configurazione logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler('ms5837_smbus2_data.log'),
        logging.StreamHandler()
    ]
)

def test_sensor(num_readings=10, interval=1.0, model='30BA'):
    """
    Test del sensore MS5837 con smbus2
    
    Args:
        num_readings: numero di letture da effettuare
        interval: intervallo tra le letture (secondi)
        model: '30BA' o '02BA'
    """
    
    # Inizializza sensore
    logging.info("=== Inizializzazione sensore MS5837-%s (smbus2) ===" % model)
    
    if model == '30BA':
        sensor = ms5837_smbus2.MS5837_30BA()
    elif model == '02BA':
        sensor = ms5837_smbus2.MS5837_02BA()
    else:
        logging.error("Modello non valido: %s" % model)
        return False
    
    # Init sensore
    if not sensor.init():
        logging.error("Inizializzazione sensore fallita!")
        return False
    
    logging.info("Sensore inizializzato correttamente")
    
    # Imposta densità fluido (opzionale)
    sensor.setFluidDensity(ms5837_smbus2.DENSITY_FRESHWATER)
    logging.info("Densità fluido: %d kg/m³" % ms5837_smbus2.DENSITY_FRESHWATER)
    
    # Loop di acquisizione
    logging.info("=== Inizio acquisizione dati ===")
    
    for i in range(num_readings):
        try:
            if sensor.read():
                # Lettura temperatura
                temp_c = sensor.temperature(ms5837_smbus2.UNITS_Centigrade)
                
                # Lettura pressione
                pressure_mbar = sensor.pressure(ms5837_smbus2.UNITS_mbar)
                pressure_pa = sensor.pressure(ms5837_smbus2.UNITS_Pa)
                
                # Profondità
                depth_m = sensor.depth()
                
                # Log dei dati
                logging.info(
                    "Lettura #%d | Temp: %.2f°C | Pressione: %.2f mbar (%.0f Pa) | "
                    "Profondità: %.2f m" % 
                    (i+1, temp_c, pressure_mbar, pressure_pa, depth_m)
                )
                
            else:
                logging.warning("Lettura #%d fallita" % (i+1))
            
            # Attendi prima della prossima lettura
            if i < num_readings - 1:
                time.sleep(interval)
                
        except KeyboardInterrupt:
            logging.info("Acquisizione interrotta dall'utente")
            break
        except Exception as e:
            logging.error("Errore durante la lettura #%d: %s" % (i+1, str(e)))
    
    logging.info("=== Acquisizione completata ===")
    return True

if __name__ == "__main__":
    # Test con 500 letture ogni 0.1 secondi
    test_sensor(num_readings=500, interval=0.1, model='30BA')