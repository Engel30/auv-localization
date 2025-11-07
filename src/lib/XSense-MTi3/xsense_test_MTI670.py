#!/usr/bin/env python3
import time
import sys
from datetime import datetime
from xsense import MTI670

# Configurazione
PORT = '/dev/ttyUSB0'  # Modifica con la tua porta seriale
LOG_FILE = 'xsens_data.log'
SAMPLE_RATE = 0.2  # secondi tra letture
NUM_SAMPLES = 50  # numero di campioni da acquisire (0 = infinito)


def main():
    print(f"Inizializzazione IMU sulla porta {PORT}...")
    try:
        ahrs = MTI670(PORT)
    except Exception as e:
        print(f"Errore apertura porta: {e}")
        sys.exit(1)

    print(f"Logging su {LOG_FILE}")
    print("CTRL+C per terminare\n")

    with open(LOG_FILE, 'w') as f:
        # Header
        f.write("timestamp,roll,pitch,yaw,accx,accy,accz,gyrox,gyroy,gyroz,magx,magy,magz,faccx,faccy,faccz\n")

        sample_count = 0
        try:
            while NUM_SAMPLES == 0 or sample_count < NUM_SAMPLES:
                ahrs.read_data()

                timestamp = datetime.now().isoformat()
                eul = ahrs.getEul()
                acc = ahrs.getAcc()
                gyro = ahrs.getGyro()
                mag = ahrs.getMag()
                facc = ahrs.getFacc()

                # Log su file
                f.write(f"{timestamp},{eul[0]},{eul[1]},{eul[2]},")
                f.write(f"{acc[0]},{acc[1]},{acc[2]},")
                f.write(f"{gyro[0]},{gyro[1]},{gyro[2]},")
                f.write(f"{mag[0]},{mag[1]},{mag[2]},")
                f.write(f"{facc[0]},{facc[1]},{facc[2]}\n")
                f.flush()

                # Stampa su console
                print(f"[{sample_count+1}] Roll: {eul[0]:7.2f} Pitch: {eul[1]:7.2f} Yaw: {eul[2]:7.2f}")
                print(f"    Acc: [{acc[0]:7.3f}, {acc[1]:7.3f}, {acc[2]:7.3f}]")

                sample_count += 1
                time.sleep(SAMPLE_RATE)

        except KeyboardInterrupt:
            print("\n\nInterrotto dall'utente")
        except Exception as e:
            print(f"\nErrore durante acquisizione: {e}")

    print(f"\nAcquisiti {sample_count} campioni")
    print(f"Dati salvati in {LOG_FILE}")


if __name__ == '__main__':
    main()