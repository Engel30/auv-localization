#!/usr/bin/env python3
import time
import threading
import random
from datetime import datetime

try:
    from lib.MS5837.ms5837 import MS5837_30BA
    MS5837_AVAILABLE = True
except ImportError:
    MS5837_AVAILABLE = False

try:
    from lib.xsens.xsense import MTI670
    XSENSE_AVAILABLE = True
except ImportError:
    XSENSE_AVAILABLE = False

# CONFIG
CONTROL_LOOP_MS = 20
IMU_PORT = '/dev/ttyUSB0'
DEPTH_I2C_BUS = 0

# SENSOR DATA
sensor_data = {
    'depth': {'pressure': 0.0, 'temperature': 0.0, 'depth': 0.0, 'valid': False},
    'imu': {'euler': [0.0, 0.0, 0.0], 'acc': [0.0, 0.0, 0.0], 'gyro': [0.0, 0.0, 0.0], 
            'mag': [0.0, 0.0, 0.0], 'facc': [0.0, 0.0, 0.0], 'valid': False},
    'usbl': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'valid': False}
}
data_lock = threading.Lock()
running = True

# DEPTH SENSOR THREAD
def depth_thread():
    global running
    sensor = None
    use_simulation = False
    
    if MS5837_AVAILABLE:
        try:
            sensor = MS5837_30BA(bus=DEPTH_I2C_BUS)
            if sensor.init():
                print("[DEPTH] Sensor initialized")
            else:
                print("[DEPTH] Init failed, using simulation")
                use_simulation = True
        except Exception as e:
            print(f"[DEPTH] Error: {e}, using simulation")
            use_simulation = True
    else:
        print("[DEPTH] Library not available, using simulation")
        use_simulation = True
    
    while running:
        try:
            if not use_simulation and sensor:
                if sensor.read():
                    with data_lock:
                        sensor_data['depth']['pressure'] = sensor.pressure()
                        sensor_data['depth']['temperature'] = sensor.temperature()
                        sensor_data['depth']['depth'] = sensor.depth()
                        sensor_data['depth']['valid'] = True
                else:
                    with data_lock:
                        sensor_data['depth']['valid'] = False
            else:
                with data_lock:
                    sensor_data['depth']['pressure'] = 1000 + random.uniform(-5, 5)
                    sensor_data['depth']['temperature'] = 20 + random.uniform(-2, 2)
                    sensor_data['depth']['depth'] = random.uniform(0, 10)
                    sensor_data['depth']['valid'] = True
            
            time.sleep(0.1)
        except Exception as e:
            print(f"[DEPTH] Exception: {e}")
            time.sleep(0.1)

# IMU SENSOR THREAD
def imu_thread():
    global running
    sensor = None
    use_simulation = False
    
    if XSENSE_AVAILABLE:
        try:
            sensor = MTI670(IMU_PORT)
            print("[IMU] Sensor initialized")
        except Exception as e:
            print(f"[IMU] Error: {e}, using simulation")
            use_simulation = True
    else:
        print("[IMU] Library not available, using simulation")
        use_simulation = True
    
    while running:
        try:
            if not use_simulation and sensor:
                sensor.read_data()
                with data_lock:
                    sensor_data['imu']['euler'] = sensor.getEul()
                    sensor_data['imu']['acc'] = sensor.getAcc()
                    sensor_data['imu']['gyro'] = sensor.getGyro()
                    sensor_data['imu']['mag'] = sensor.getMag()
                    sensor_data['imu']['facc'] = sensor.getFacc()
                    sensor_data['imu']['valid'] = True
            else:
                with data_lock:
                    sensor_data['imu']['euler'] = [random.uniform(-180, 180) for _ in range(3)]
                    sensor_data['imu']['acc'] = [random.uniform(-2, 2) for _ in range(3)]
                    sensor_data['imu']['gyro'] = [random.uniform(-0.5, 0.5) for _ in range(3)]
                    sensor_data['imu']['mag'] = [random.uniform(-1, 1) for _ in range(3)]
                    sensor_data['imu']['facc'] = [random.uniform(-1, 1) for _ in range(3)]
                    sensor_data['imu']['valid'] = True
            
            time.sleep(0.01)
        except Exception as e:
            print(f"[IMU] Exception: {e}")
            time.sleep(0.01)

# USBL THREAD (SIMULATION)
def usbl_thread():
    global running
    print("[USBL] Using simulation")
    
    while running:
        try:
            with data_lock:
                sensor_data['usbl']['x'] = random.uniform(-10, 10)
                sensor_data['usbl']['y'] = random.uniform(-10, 10)
                sensor_data['usbl']['z'] = random.uniform(-5, 0)
                sensor_data['usbl']['valid'] = True
            time.sleep(0.5)
        except Exception as e:
            print(f"[USBL] Exception: {e}")
            time.sleep(0.5)

# MAIN CONTROL LOOP
def main():
    global running
    
    print("=" * 80)
    print("SENSOR FUSION SYSTEM")
    print(f"Control Loop: {CONTROL_LOOP_MS}ms")
    print("=" * 80)
    
    # Start sensor threads
    threads = [
        threading.Thread(target=depth_thread, daemon=True),
        threading.Thread(target=imu_thread, daemon=True),
        threading.Thread(target=usbl_thread, daemon=True)
    ]
    
    for t in threads:
        t.start()
    
    time.sleep(1)  # Wait for sensors to initialize
    
    loop_time = CONTROL_LOOP_MS / 1000.0
    
    try:
        while True:
            start = time.time()
            timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
            
            with data_lock:
                d = sensor_data['depth']
                i = sensor_data['imu']
                u = sensor_data['usbl']
            
            print(f"\n{'='*80}")
            print(f"[{timestamp}]")
            print(f"{'='*80}")
            
            # Depth sensor
            status = "OK" if d['valid'] else "FAIL"
            print(f"DEPTH  [{status:4s}] P:{d['pressure']:8.2f} mbar  T:{d['temperature']:6.2f} °C  D:{d['depth']:6.2f} m")
            
            # IMU sensor
            status = "OK" if i['valid'] else "FAIL"
            print(f"IMU    [{status:4s}] Euler: [{i['euler'][0]:7.2f}, {i['euler'][1]:7.2f}, {i['euler'][2]:7.2f}]")
            print(f"              Acc:   [{i['acc'][0]:7.3f}, {i['acc'][1]:7.3f}, {i['acc'][2]:7.3f}]")
            print(f"              Gyro:  [{i['gyro'][0]:7.3f}, {i['gyro'][1]:7.3f}, {i['gyro'][2]:7.3f}]")
            print(f"              Mag:   [{i['mag'][0]:7.3f}, {i['mag'][1]:7.3f}, {i['mag'][2]:7.3f}]")
            print(f"              FAcc:  [{i['facc'][0]:7.3f}, {i['facc'][1]:7.3f}, {i['facc'][2]:7.3f}]")
            
            # USBL sensor
            status = "OK" if u['valid'] else "FAIL"
            print(f"USBL   [{status:4s}] X:{u['x']:7.2f} m  Y:{u['y']:7.2f} m  Z:{u['z']:7.2f} m")
            
            # Timing
            elapsed = time.time() - start
            sleep_time = max(0, loop_time - elapsed)
            time.sleep(sleep_time)
            
    except KeyboardInterrupt:
        print("\n\nShutting down...")
        running = False
        time.sleep(0.5)

if __name__ == "__main__":
    main()