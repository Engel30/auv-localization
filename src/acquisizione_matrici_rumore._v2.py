#!/usr/bin/env python3
"""
Sensor Calibration System for Kalman Filter
Collects data from sensors and computes noise covariance matrices R
"""
import time
import threading
import numpy as np
from datetime import datetime
import json

try:
    from lib.MS5837.ms5837 import MS5837_30BA
    MS5837_AVAILABLE = True
except ImportError:
    MS5837_AVAILABLE = False
    print("[WARNING] MS5837 library not available")

try:
    from lib.xsens.xsense import MTI670
    XSENSE_AVAILABLE = True
except ImportError:
    XSENSE_AVAILABLE = False
    print("[WARNING] Xsense library not available")

# CONFIG
IMU_PORT = '/dev/ttyUSB0'
DEPTH_I2C_BUS = 1
CALIBRATION_TIME = 60  # seconds of data collection

# Data storage
imu_data = {'acc': [], 'gyro': [], 'euler': [], 'facc': [], 'mag': []}
depth_data = {'pressure': [], 'depth': [], 'temperature': []}
data_lock = threading.Lock()
running = True

# Sensor status
sensors_active = {'imu': False, 'depth': False}

def check_sensor_availability():
    """Check which sensors are available and working"""
    print("\n" + "="*80)
    print("SENSOR AVAILABILITY CHECK")
    print("="*80)
    
    # Check IMU
    if XSENSE_AVAILABLE:
        try:
            sensor = MTI670(IMU_PORT)
            sensor.read_data()  # Try one read
            sensors_active['imu'] = True
            print("[IMU] ✓ Connected and working on", IMU_PORT)
        except Exception as e:
            print(f"[IMU] ✗ Error: {e}")
            sensors_active['imu'] = False
    else:
        print("[IMU] ✗ Library not available")
    
    # Check Depth sensor
    if MS5837_AVAILABLE:
        try:
            sensor = MS5837_30BA(DEPTH_I2C_BUS)
            if sensor.init() and sensor.read():
                sensors_active['depth'] = True
                print(f"[DEPTH] ✓ Connected and working on I2C bus {DEPTH_I2C_BUS}")
            else:
                print("[DEPTH] ✗ Initialization or read failed")
                sensors_active['depth'] = False
        except Exception as e:
            print(f"[DEPTH] ✗ Error: {e}")
            sensors_active['depth'] = False
    else:
        print("[DEPTH] ✗ Library not available")
    
    print("="*80 + "\n")
    
    if not any(sensors_active.values()):
        print("[ERROR] No sensors available! Cannot perform calibration.")
        return False
    
    return True

def imu_calibration_thread():
    """Collect IMU data for calibration"""
    global running
    
    if not sensors_active['imu']:
        return
    
    try:
        sensor = MTI670(IMU_PORT)
        print("[IMU] Calibration thread started")
        
        while running:
            try:
                sensor.read_data()
                with data_lock:
                    imu_data['acc'].append(sensor.getAcc())
                    imu_data['gyro'].append(sensor.getGyro())
                    imu_data['euler'].append(sensor.getEul())
                    imu_data['facc'].append(sensor.getFacc())
                    imu_data['mag'].append(sensor.getMag())
                time.sleep(0.01)  # 100 Hz sampling
            except Exception as e:
                print(f"[IMU] Read error: {e}")
                time.sleep(0.1)
                
    except Exception as e:
        print(f"[IMU] Thread error: {e}")
        sensors_active['imu'] = False

def depth_calibration_thread():
    """Collect depth sensor data for calibration"""
    global running
    
    if not sensors_active['depth']:
        return
    
    try:
        sensor = MS5837_30BA(DEPTH_I2C_BUS)
        if not sensor.init():
            print("[DEPTH] Initialization failed")
            sensors_active['depth'] = False
            return
            
        print("[DEPTH] Calibration thread started")
        
        while running:
            try:
                if sensor.read():
                    with data_lock:
                        depth_data['pressure'].append(sensor.pressure())
                        depth_data['depth'].append(sensor.depth())
                        depth_data['temperature'].append(sensor.temperature())
                time.sleep(0.1)  # 10 Hz sampling
            except Exception as e:
                print(f"[DEPTH] Read error: {e}")
                time.sleep(0.1)
                
    except Exception as e:
        print(f"[DEPTH] Thread error: {e}")
        sensors_active['depth'] = False

def compute_statistics(data_array, name):
    """Compute mean, std, and variance for a data array"""
    if len(data_array) == 0:
        return None
    
    data = np.array(data_array)
    stats = {
        'mean': np.mean(data, axis=0).tolist(),
        'std': np.std(data, axis=0).tolist(),
        'var': np.var(data, axis=0).tolist(),
        'samples': len(data)
    }
    
    print(f"\n{name}:")
    print(f"  Samples: {stats['samples']}")
    print(f"  Mean:    {stats['mean']}")
    print(f"  Std:     {stats['std']}")
    print(f"  Var:     {stats['var']}")
    
    return stats

def compute_covariance_matrices():
    """Compute R matrices for Kalman filter"""
    print("\n" + "="*80)
    print("COMPUTING NOISE COVARIANCE MATRICES (R)")
    print("="*80)
    
    results = {}
    
    # IMU Statistics
    if sensors_active['imu']:
        print("\n--- IMU SENSOR ---")
        with data_lock:
            acc_stats = compute_statistics(imu_data['acc'], "Accelerometer")
            gyro_stats = compute_statistics(imu_data['gyro'], "Gyroscope (Angular Velocity)")
            facc_stats = compute_statistics(imu_data['facc'], "Free Acceleration")
            euler_stats = compute_statistics(imu_data['euler'], "Euler Angles (Orientation)")
            mag_stats = compute_statistics(imu_data['mag'], "Magnetometer")
        
        # R_acc: Accelerometer noise
        if acc_stats:
            R_acc_diag = acc_stats['var']
            R_acc = np.diag(R_acc_diag)
            results['R_accelerometer'] = {
                'matrix': R_acc.tolist(),
                'description': 'Accelerometer noise covariance (3x3) [ax, ay, az]',
                'diagonal': R_acc_diag,
                'std': acc_stats['std']
            }
            
            print(f"\n*** R_accelerometer (Linear Acceleration) ***")
            print(f"Diagonal (variance): {R_acc_diag}")
            print(f"Std deviation: {acc_stats['std']}")
            print(f"Matrix:\n{R_acc}")
        
        # R_gyro: Gyroscope noise (angular velocity)
        if gyro_stats:
            R_gyro_diag = gyro_stats['var']
            R_gyro = np.diag(R_gyro_diag)
            results['R_gyroscope'] = {
                'matrix': R_gyro.tolist(),
                'description': 'Gyroscope noise covariance (3x3) [wx, wy, wz]',
                'diagonal': R_gyro_diag,
                'std': gyro_stats['std']
            }
            
            print(f"\n*** R_gyroscope (Angular Velocity) ***")
            print(f"Diagonal (variance): {R_gyro_diag}")
            print(f"Std deviation: {gyro_stats['std']}")
            print(f"Matrix:\n{R_gyro}")
        
        # R_euler: Orientation noise (Euler angles)
        if euler_stats:
            R_euler_diag = euler_stats['var']
            R_euler = np.diag(R_euler_diag)
            results['R_orientation_euler'] = {
                'matrix': R_euler.tolist(),
                'description': 'Orientation noise covariance (3x3) [roll, pitch, yaw]',
                'diagonal': R_euler_diag,
                'std': euler_stats['std']
            }
            
            print(f"\n*** R_orientation (Euler Angles) ***")
            print(f"Diagonal (variance): {R_euler_diag}")
            print(f"Std deviation: {euler_stats['std']}")
            print(f"Matrix:\n{R_euler}")
        
        # R_facc: Free acceleration noise
        if facc_stats:
            R_facc_diag = facc_stats['var']
            R_facc = np.diag(R_facc_diag)
            results['R_free_acceleration'] = {
                'matrix': R_facc.tolist(),
                'description': 'Free acceleration noise covariance (3x3)',
                'diagonal': R_facc_diag,
                'std': facc_stats['std']
            }
            
            print(f"\n*** R_free_acceleration (Gravity compensated) ***")
            print(f"Diagonal (variance): {R_facc_diag}")
            print(f"Std deviation: {facc_stats['std']}")
            print(f"Matrix:\n{R_facc}")
        
        # R_mag: Magnetometer noise
        if mag_stats:
            R_mag_diag = mag_stats['var']
            R_mag = np.diag(R_mag_diag)
            results['R_magnetometer'] = {
                'matrix': R_mag.tolist(),
                'description': 'Magnetometer noise covariance (3x3)',
                'diagonal': R_mag_diag,
                'std': mag_stats['std']
            }
            
            print(f"\n*** R_magnetometer ***")
            print(f"Diagonal (variance): {R_mag_diag}")
            print(f"Std deviation: {mag_stats['std']}")
            print(f"Matrix:\n{R_mag}")
    
    # Depth sensor statistics
    if sensors_active['depth']:
        print("\n--- DEPTH SENSOR ---")
        with data_lock:
            depth_stats = compute_statistics(depth_data['depth'], "Depth")
            pressure_stats = compute_statistics(depth_data['pressure'], "Pressure")
            temp_stats = compute_statistics(depth_data['temperature'], "Temperature")
        
        # R3: Depth measurement noise
        if depth_stats:
            R3_var = depth_stats['var']
            R3 = np.array([[R3_var]])
            results['R3_depth'] = {
                'matrix': R3.tolist(),
                'description': 'Depth sensor noise (1x1)',
                'variance': R3_var
            }
            
            print(f"\n*** R3 (Depth Sensor) ***")
            print(f"Variance: {R3_var}")
            print(f"Matrix:\n{R3}")
    
    # Process noise Q (estimated from acceleration variance)
    if sensors_active['imu'] and acc_stats:
        Q_diag = acc_stats['var']
        Q = np.diag(Q_diag)
        results['Q_process_acceleration'] = {
            'matrix': Q.tolist(),
            'description': 'Process noise from acceleration (3x3)',
            'diagonal': Q_diag,
            'std': acc_stats['std']
        }
        
        print(f"\n*** Q_process (Acceleration) ***")
        print(f"Diagonal (variance): {Q_diag}")
        print(f"Std deviation: {acc_stats['std']}")
        print(f"Matrix:\n{Q}")
    
    # Process noise for angular velocity (from gyroscope)
    if sensors_active['imu'] and gyro_stats:
        Q_angular_diag = gyro_stats['var']
        Q_angular = np.diag(Q_angular_diag)
        results['Q_process_angular'] = {
            'matrix': Q_angular.tolist(),
            'description': 'Process noise from angular velocity (3x3)',
            'diagonal': Q_angular_diag,
            'std': gyro_stats['std']
        }
        
        print(f"\n*** Q_process (Angular Velocity) ***")
        print(f"Diagonal (variance): {Q_angular_diag}")
        print(f"Std deviation: {gyro_stats['std']}")
        print(f"Matrix:\n{Q_angular}")
    
    # R2: USBL (not measured, provide typical values)
    R2_typical = np.diag([1.0, 1.0, 0.5])  # typical USBL noise [x, y, z]
    results['R2_usbl_suggested'] = {
        'matrix': R2_typical.tolist(),
        'description': 'USBL noise (suggested typical values) (3x3)',
        'diagonal': [1.0, 1.0, 0.5],
        'note': 'These are typical values. Calibrate with real USBL data when available.'
    }
    
    print(f"\n*** R2 (USBL - SUGGESTED VALUES) ***")
    print(f"Diagonal values: [1.0, 1.0, 0.5]")
    print(f"Matrix:\n{R2_typical}")
    print("NOTE: These are typical values. Calibrate with real USBL when available.")
    
    return results

def save_results(results, filename='calibration_results.json'):
    """Save calibration results to JSON file"""
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"calibration_{timestamp}.json"
    
    with open(filename, 'w') as f:
        json.dump(results, f, indent=2)
    
    print(f"\n[SAVED] Results saved to: {filename}")

def generate_kalman_code(results):
    """Generate Python code snippet for Kalman filter initialization"""
    print("\n" + "="*80)
    print("KALMAN FILTER INITIALIZATION CODE")
    print("="*80)
    
    code = "# Calibrated matrices for Kalman Filter\nimport numpy as np\n\n"
    code += "# "+"="*70 + "\n"
    code += "# MEASUREMENT NOISE MATRICES (R)\n"
    code += "# "+"="*70 + "\n\n"
    
    if 'R_accelerometer' in results:
        R = results['R_accelerometer']['diagonal']
        code += f"# Accelerometer measurement noise [ax, ay, az]\n"
        code += f"R_accelerometer = np.diag({R})\n\n"
    
    if 'R_gyroscope' in results:
        R = results['R_gyroscope']['diagonal']
        code += f"# Gyroscope measurement noise [wx, wy, wz] (angular velocity)\n"
        code += f"R_gyroscope = np.diag({R})\n\n"
    
    if 'R_orientation_euler' in results:
        R = results['R_orientation_euler']['diagonal']
        code += f"# Orientation measurement noise [roll, pitch, yaw]\n"
        code += f"R_orientation = np.diag({R})\n\n"
    
    if 'R_free_acceleration' in results:
        R = results['R_free_acceleration']['diagonal']
        code += f"# Free acceleration measurement noise (gravity compensated)\n"
        code += f"R_free_acceleration = np.diag({R})\n\n"
    
    if 'R_magnetometer' in results:
        R = results['R_magnetometer']['diagonal']
        code += f"# Magnetometer measurement noise\n"
        code += f"R_magnetometer = np.diag({R})\n\n"
    
    if 'R3_depth' in results:
        R3 = results['R3_depth']['variance']
        code += f"# Depth sensor measurement noise\n"
        code += f"R_depth = np.array([[{R3}]])\n\n"
    
    if 'R2_usbl_suggested' in results:
        R2 = results['R2_usbl_suggested']['diagonal']
        code += f"# USBL measurement noise (typical values - calibrate with real data)\n"
        code += f"R_usbl = np.diag({R2})\n\n"
    
    code += "# "+"="*70 + "\n"
    code += "# PROCESS NOISE MATRICES (Q)\n"
    code += "# "+"="*70 + "\n\n"
    
    if 'Q_process_acceleration' in results:
        Q = results['Q_process_acceleration']['diagonal']
        code += f"# Process noise from acceleration [ax, ay, az]\n"
        code += f"Q_acceleration = np.diag({Q})\n\n"
    
    if 'Q_process_angular' in results:
        Q = results['Q_process_angular']['diagonal']
        code += f"# Process noise from angular velocity [wx, wy, wz]\n"
        code += f"Q_angular = np.diag({Q})\n\n"
    
    code += "# "+"="*70 + "\n"
    code += "# KALMAN FILTER MATRICES (for your existing KF)\n"
    code += "# "+"="*70 + "\n\n"
    
    if 'Q_process_acceleration' in results:
        Q = results['Q_process_acceleration']['diagonal']
        code += f"# Process noise (use acceleration-based Q)\n"
        code += f"Q = np.diag({Q})\n\n"
    
    if 'R_free_acceleration' in results:
        R1 = results['R_free_acceleration']['diagonal']
        code += f"# R1: IMU measurement noise (free acceleration)\n"
        code += f"R1 = np.diag({R1})\n\n"
    
    if 'R2_usbl_suggested' in results:
        R2 = results['R2_usbl_suggested']['diagonal']
        code += f"# R2: USBL measurement noise\n"
        code += f"R2 = np.diag({R2})\n\n"
    
    if 'R3_depth' in results:
        R3 = results['R3_depth']['variance']
        code += f"# R3: Depth sensor measurement noise\n"
        code += f"R3 = np.array([[{R3}]])\n\n"
    
    # Observation matrices
    code += "# "+"="*70 + "\n"
    code += "# OBSERVATION MATRICES (C)\n"
    code += "# "+"="*70 + "\n\n"
    code += "# C1: IMU observes acceleration (state vector indices 6:9)\n"
    code += "C1 = np.zeros((3, 9))\n"
    code += "C1[0:3, 6:9] = np.eye(3)\n\n"
    code += "# C2: USBL observes position (state vector indices 0:3)\n"
    code += "C2 = np.zeros((3, 9))\n"
    code += "C2[0:3, 0:3] = np.eye(3)\n\n"
    code += "# C3: Depth sensor observes Z position (state vector index 2)\n"
    code += "C3 = np.zeros((1, 9))\n"
    code += "C3[0, 2] = 1.0\n\n"
    
    code += "# "+"="*70 + "\n"
    code += "# EXAMPLE: Initialize KF_profond.py Kalman Filter\n"
    code += "# "+"="*70 + "\n\n"
    code += "# from KF_profond import KF\n"
    code += "# kf = KF(Q=Q, R1=R1, R2=R2, R3=R3, C1=C1, C2=C2, C3=C3, T=0.05)\n"
    
    print(code)
    
    # Save to file
    with open('kalman_matrices.py', 'w') as f:
        f.write(code)
    print("\n[SAVED] Code saved to: kalman_matrices.py")

def main():
    global running
    
    print("\n" + "="*80)
    print("KALMAN FILTER SENSOR CALIBRATION")
    print("="*80)
    print(f"This will collect sensor data for {CALIBRATION_TIME} seconds")
    print("Keep sensors STATIONARY and at CONSTANT DEPTH during calibration!")
    print("="*80)
    
    # Check sensor availability
    if not check_sensor_availability():
        return
    
    print(f"\nStarting calibration in 3 seconds...")
    for i in range(3, 0, -1):
        print(f"{i}...")
        time.sleep(1)
    
    print("\n*** CALIBRATION STARTED ***\n")
    
    # Start collection threads
    threads = []
    if sensors_active['imu']:
        t = threading.Thread(target=imu_calibration_thread, daemon=True)
        t.start()
        threads.append(t)
    
    if sensors_active['depth']:
        t = threading.Thread(target=depth_calibration_thread, daemon=True)
        t.start()
        threads.append(t)
    
    # Progress bar
    start_time = time.time()
    try:
        while time.time() - start_time < CALIBRATION_TIME:
            elapsed = time.time() - start_time
            remaining = CALIBRATION_TIME - elapsed
            progress = int((elapsed / CALIBRATION_TIME) * 40)
            bar = "█" * progress + "░" * (40 - progress)
            
            with data_lock:
                imu_samples = len(imu_data['acc']) if sensors_active['imu'] else 0
                depth_samples = len(depth_data['depth']) if sensors_active['depth'] else 0
            
            print(f"\r[{bar}] {elapsed:.1f}s / {CALIBRATION_TIME}s | "
                  f"IMU: {imu_samples} samples | Depth: {depth_samples} samples", 
                  end='', flush=True)
            
            time.sleep(0.1)
    
    except KeyboardInterrupt:
        print("\n\nCalibration interrupted by user!")
    
    print("\n\n*** CALIBRATION COMPLETE ***\n")
    running = False
    time.sleep(0.5)
    
    # Compute statistics and matrices
    results = compute_covariance_matrices()
    
    # Save results
    save_results(results)
    
    # Generate code
    generate_kalman_code(results)
    
    print("\n" + "="*80)
    print("CALIBRATION FINISHED")
    print("="*80)
    print("You can now use the matrices in your Kalman filter!")
    print("Check 'kalman_matrices.py' for ready-to-use code.")

if __name__ == "__main__":
    main()