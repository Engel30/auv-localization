#!/usr/bin/env python3
import time
import threading
import numpy as np
import math
from datetime import datetime
from collections import deque

from lib.KF.KF import KF
from lib.KF.RMatrix import Rxyz

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

try:
    import matplotlib.pyplot as plt
    # Import necessario per registrare la proiezione 3D su versioni vecchie di Matplotlib
    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
    MATPLOTLIB_AVAILABLE = True
except ImportError:
    MATPLOTLIB_AVAILABLE = False
    print("[WARNING] matplotlib not available, plotting disabled")

# ==================== CONFIG ====================
CONTROL_LOOP_MS = 50  # 50ms = 20Hz (era 20ms nella simulazione)
ENABLE_PLOTTING = True and MATPLOTLIB_AVAILABLE
USE_REAL_SENSORS = False  # Set True per sensori reali

IMU_PORT = '/dev/ttyUSB0'
DEPTH_I2C_BUS = 0

# Circular trajectory params
RADIUS = 10.0
OMEGA = 0.15

# Sensor noise
SIGMA_ACC = 0.03
SIGMA_POS = 0.15
SIGMA_YAW = np.deg2rad(1.0)

# USBL position fix every N iterations
USBL_FIX_EVERY = 5

# Plot history length
PLOT_HISTORY = 150

# *** Controlli per performance ***
PLOT_UPDATE_DT = 0.20   # [s] refresh del plot ~5 Hz
PRINT_EVERY = 5         # stampa a terminale ogni N iterazioni

# ==================== GLOBALS ====================
sensor_data = {
    'imu': {'euler': [0.0, 0.0, 0.0], 'acc_body': [0.0, 0.0, 0.0], 'valid': False},
    'usbl': {'pos_world': [0.0, 0.0, 0.0], 'valid': False},
    'depth': {'depth': 0.0, 'valid': False}
}
data_lock = threading.Lock()
running = True

# Simulation time (shared)
sim_time = {'t': 0.0}
sim_lock = threading.Lock()

# KF state
kf_state = {
    'pos': [0.0, 0.0, 0.0],
    'vel': [0.0, 0.0, 0.0],
    'acc': [0.0, 0.0, 0.0]
}
kf_lock = threading.Lock()

# Plot data
plot_data = {
    'time': deque(maxlen=PLOT_HISTORY),
    'true_pos': deque(maxlen=PLOT_HISTORY),
    'est_pos': deque(maxlen=PLOT_HISTORY),
    'meas_pos': [],  # Store only valid fixes (not a deque)
    'error': deque(maxlen=PLOT_HISTORY)
}
plot_lock = threading.Lock()

# ==================== SIMULATION / SENSOR THREADS ====================
def imu_simulation_thread():
    """Gestisce i dati IMU: usa il sensore reale (XSens MTI670) se disponibile e USE_REAL_SENSORS=True,
    altrimenti genera dati simulati sulla traiettoria circolare originale."""
    global running

    use_simulation = not USE_REAL_SENSORS
    sensor = None

    if USE_REAL_SENSORS and XSENSE_AVAILABLE:
        try:
            sensor = MTI670(IMU_PORT)
            print("[IMU] Sensore XSens inizializzato")
            use_simulation = False
        except Exception as e:
            print(f"[IMU] Errore inizializzazione IMU: {e}, passo alla simulazione")
            use_simulation = True
    elif USE_REAL_SENSORS:
        print("[IMU] Libreria XSens non disponibile, uso la simulazione IMU")
        use_simulation = True
    else:
        print("[IMU] USE_REAL_SENSORS=False, uso la simulazione IMU")
        use_simulation = True

    while running:
        if not use_simulation and sensor is not None:
            # Lettura reale da IMU
            try:
                sensor.read_data()
                eul_deg = sensor.getEul()   # [roll, pitch, yaw] in gradi
                roll = math.radians(eul_deg[0])
                pitch = math.radians(eul_deg[1])
                yaw = math.radians(eul_deg[2])

                # Preferisco la free-acceleration (gravity-compensated) se disponibile
                try:
                    acc_body = sensor.getFacc()
                except AttributeError:
                    acc_body = sensor.getAcc()

                with data_lock:
                    sensor_data['imu']['euler'] = [roll, pitch, yaw]
                    sensor_data['imu']['acc_body'] = [acc_body[0], acc_body[1], acc_body[2]]
                    sensor_data['imu']['valid'] = True
            except Exception as e:
                print(f"[IMU] Eccezione in lettura, passo alla simulazione: {e}")
                use_simulation = True
                sensor = None
                continue

            time.sleep(0.01)
        else:
            # === Modalità simulata: traiettoria circolare ===
            with sim_lock:
                t = sim_time['t']

            theta = OMEGA * t

            # Accelerazione vera (frame mondo)
            a_world = np.array([
                [-RADIUS * (OMEGA ** 2) * math.cos(theta)],
                [-RADIUS * (OMEGA ** 2) * math.sin(theta)],
                [0.0]
            ])

            # Orientazione
            roll = 0.0
            pitch = 0.0
            yaw_true = theta + math.pi / 2.0

            # Transform to body frame
            Rwb = Rxyz(roll, pitch, yaw_true)
            a_body_true = Rwb.T @ a_world

            # Add noise
            a_body_meas = a_body_true + np.random.normal(0.0, SIGMA_ACC, size=(3, 1))
            yaw_meas = yaw_true + np.random.normal(0.0, SIGMA_YAW)

            with data_lock:
                sensor_data['imu']['euler'] = [roll, pitch, yaw_meas]
                sensor_data['imu']['acc_body'] = [a_body_meas[0, 0], a_body_meas[1, 0], a_body_meas[2, 0]]
                sensor_data['imu']['valid'] = True

            time.sleep(0.005)


def usbl_simulation_thread():
    """Simula la posizione USBL.
    Per ora non è implementato il driver reale, quindi anche in modalità sensori reali
    si usa comunque la simulazione con rumore."""
    global running

    if USE_REAL_SENSORS:
        print("[USBL] Nessun driver reale implementato, uso comunque la simulazione USBL")
    else:
        print("[USBL] USBL in modalità simulata")

    while running:
        with sim_lock:
            t = sim_time['t']

        theta = OMEGA * t

        # Ground truth position (world frame)
        p_true = np.array([
            [RADIUS * math.cos(theta)],
            [RADIUS * math.sin(theta)],
            [0.0]
        ])

        # Add measurement noise and update
        p_meas = p_true + np.random.normal(0.0, SIGMA_POS, size=(3, 1))
        with data_lock:
            sensor_data['usbl']['pos_world'] = [p_meas[0, 0], p_meas[1, 0], p_meas[2, 0]]
            sensor_data['usbl']['valid'] = True

        time.sleep(0.01)


def depth_simulation_thread():
    """Gestisce il sensore di profondità: usa l'MS5837 reale se disponibile e USE_REAL_SENSORS=True,
    altrimenti genera un valore simulato (profondità costante)."""
    global running

    sensor = None
    use_simulation = not USE_REAL_SENSORS

    if USE_REAL_SENSORS and MS5837_AVAILABLE:
        try:
            sensor = MS5837_30BA(bus=DEPTH_I2C_BUS)
            if sensor.init():
                print("[DEPTH] Sensore MS5837 inizializzato")
                use_simulation = False
            else:
                print("[DEPTH] Init fallita, uso simulazione profondità")
                use_simulation = True
        except Exception as e:
            print(f"[DEPTH] Errore inizializzazione profondimetro: {e}, uso simulazione")
            use_simulation = True
    elif USE_REAL_SENSORS:
        print("[DEPTH] Libreria MS5837 non disponibile, uso simulazione profondità")
        use_simulation = True
    else:
        print("[DEPTH] USE_REAL_SENSORS=False, uso simulazione profondità")
        use_simulation = True

    while running:
        try:
            if not use_simulation and sensor is not None:
                if sensor.read():
                    depth_val = sensor.depth()
                    with data_lock:
                        sensor_data['depth']['depth'] = depth_val
                        sensor_data['depth']['valid'] = True
                else:
                    with data_lock:
                        sensor_data['depth']['valid'] = False
                time.sleep(0.1)
            else:
                # === Modalità simulata: profondità costante ===
                with sim_lock:
                    t = sim_time['t']

                depth_sim = 0.0

                with data_lock:
                    sensor_data['depth']['depth'] = depth_sim
                    sensor_data['depth']['valid'] = True

                time.sleep(0.1)
        except Exception as e:
            print(f"[DEPTH] Eccezione thread profondità: {e}")
            time.sleep(0.1)

# ==================== PLOTTING ====================
def plot_thread():
    """Real-time plotting thread with layout ottimizzato e update rate limitato."""
    if not ENABLE_PLOTTING:
        return

    import matplotlib
    from matplotlib.gridspec import GridSpec

    plt.ion()
    fig = plt.figure(figsize=(16, 9))

    gs = GridSpec(3, 2,
                  width_ratios=[2, 1],
                  height_ratios=[1, 1, 1],
                  left=0.05, right=0.98,
                  top=0.95, bottom=0.06,
                  wspace=0.25, hspace=0.35)

    # 3D Trajectory
    ax1 = fig.add_subplot(gs[:, 0], projection='3d')
    line_true, = ax1.plot([], [], [], 'b-', label='Ground Truth', linewidth=2.0, alpha=0.8)
    line_est, = ax1.plot([], [], [], 'r-', label='KF Estimate', linewidth=2.0, alpha=0.9)
    scatter_meas = ax1.scatter([], [], [], c='lime', s=50, marker='o',
                               edgecolors='darkgreen', linewidths=1.5,
                               label='USBL Fix', alpha=0.9)

    ax1.set_xlabel('X [m]')
    ax1.set_ylabel('Y [m]')
    ax1.set_zlabel('Z [m]')
    ax1.set_title('3D Trajectory - Circular Path')
    ax1.legend(loc='upper right', fontsize=9, framealpha=0.9)
    ax1.grid(True, alpha=0.3)
    ax1.set_facecolor('#f0f0f0')

    ax1.set_xlim(-12, 12)
    ax1.set_ylim(-12, 12)
    ax1.set_zlim(-2, 2)

    # XY top view
    ax2 = fig.add_subplot(gs[0, 1])
    line_true_xy, = ax2.plot([], [], 'b-', label='True', linewidth=1.8, alpha=0.7)
    line_est_xy, = ax2.plot([], [], 'r-', label='KF Est', linewidth=1.8, alpha=0.8)
    scatter_meas_xy = ax2.scatter([], [], c='lime', s=35, marker='o',
                                  edgecolors='darkgreen', linewidths=1.2,
                                  label='USBL', alpha=0.9)
    ax2.set_xlabel('X [m]')
    ax2.set_ylabel('Y [m]')
    ax2.set_title('XY Top View')
    ax2.legend(loc='upper right', fontsize=8, framealpha=0.9)
    ax2.grid(True, alpha=0.3, linestyle='--')
    ax2.set_facecolor('#f8f8f8')
    ax2.axis('equal')

    # Error over time
    ax3 = fig.add_subplot(gs[1, 1])
    line_err, = ax3.plot([], [], 'r-', linewidth=2.0, alpha=0.8)
    ax3.set_xlabel('Time [s]')
    ax3.set_ylabel('Error [m]')
    ax3.set_title('Position Error Norm')
    ax3.grid(True, alpha=0.3, linestyle='--')
    ax3.set_facecolor('#f8f8f8')
    ax3.set_ylim(0, 2)

    # Position components
    ax4 = fig.add_subplot(gs[2, 1])
    line_x, = ax4.plot([], [], 'r-', label='X', linewidth=1.8, alpha=0.8)
    line_y, = ax4.plot([], [], 'g-', label='Y', linewidth=1.8, alpha=0.8)
    line_z, = ax4.plot([], [], 'b-', label='Z', linewidth=1.8, alpha=0.8)
    ax4.set_xlabel('Time [s]')
    ax4.set_ylabel('Position [m]')
    ax4.set_title('Position Components')
    ax4.legend(loc='upper right', fontsize=8, framealpha=0.9)
    ax4.grid(True, alpha=0.3, linestyle='--')
    ax4.set_facecolor('#f8f8f8')

    last_update = 0.0

    while running:
        now = time.time()
        # *** rate limit per performance ***
        if now - last_update < PLOT_UPDATE_DT:
            plt.pause(0.01)
            continue
        last_update = now

        with plot_lock:
            if len(plot_data['time']) == 0:
                plt.pause(0.01)
                continue

            ts = list(plot_data['time'])
            true_pos = list(plot_data['true_pos'])
            est_pos = list(plot_data['est_pos'])
            meas_pos = [p for t, p in plot_data['meas_pos']]
            errors = list(plot_data['error'])

        if len(true_pos) == 0:
            plt.pause(0.01)
            continue

        true_arr = np.asarray(true_pos)
        est_arr = np.asarray(est_pos)

        # 3D trajectory
        line_true.set_data(true_arr[:, 0], true_arr[:, 1])
        line_true.set_3d_properties(true_arr[:, 2])

        line_est.set_data(est_arr[:, 0], est_arr[:, 1])
        line_est.set_3d_properties(est_arr[:, 2])

        if len(meas_pos) > 0:
            meas_arr = np.asarray(meas_pos)
            # Compatibile con Matplotlib 2.x
            scatter_meas._offsets3d = (meas_arr[:, 0], meas_arr[:, 1], meas_arr[:, 2])

        # auto-limiti 3D (leggero)
        all_x = np.concatenate([true_arr[:, 0], est_arr[:, 0]])
        all_y = np.concatenate([true_arr[:, 1], est_arr[:, 1]])
        all_z = np.concatenate([true_arr[:, 2], est_arr[:, 2]])

        x_margin = (all_x.max() - all_x.min()) * 0.1 + 1
        y_margin = (all_y.max() - all_y.min()) * 0.1 + 1
        z_margin = max((all_z.max() - all_z.min()) * 0.2, 1)

        ax1.set_xlim(all_x.min() - x_margin, all_x.max() + x_margin)
        ax1.set_ylim(all_y.min() - y_margin, all_y.max() + y_margin)
        ax1.set_zlim(all_z.min() - z_margin, all_z.max() + z_margin)

        # XY view
        line_true_xy.set_data(true_arr[:, 0], true_arr[:, 1])
        line_est_xy.set_data(est_arr[:, 0], est_arr[:, 1])
        if len(meas_pos) > 0:
            scatter_meas_xy.set_offsets(meas_arr[:, :2])

        ax2.set_xlim(all_x.min() - x_margin, all_x.max() + x_margin)
        ax2.set_ylim(all_y.min() - y_margin, all_y.max() + y_margin)

        # Error
        line_err.set_data(ts, errors)
        ax3.collections[:] = []  # clear vecchi fill
        if len(errors) > 0:
            ax3.fill_between(ts, errors, 0, alpha=0.2, color='red')
            error_max = max(errors)
            ax3.set_ylim(0, error_max * 1.15)
            ax3.set_xlim(ts[0], ts[-1])

        # Position components
        line_x.set_data(ts, est_arr[:, 0])
        line_y.set_data(ts, est_arr[:, 1])
        line_z.set_data(ts, est_arr[:, 2])

        all_pos = np.concatenate([est_arr[:, 0], est_arr[:, 1], est_arr[:, 2]])
        pos_margin = (all_pos.max() - all_pos.min()) * 0.1 + 0.5
        ax4.set_ylim(all_pos.min() - pos_margin, all_pos.max() + pos_margin)
        ax4.set_xlim(ts[0], ts[-1])

        plt.pause(0.01)

    plt.close('all')

# ==================== MAIN CONTROL LOOP ====================
def main():
    global running

    print("=" * 80)
    print("KALMAN FILTER SENSOR FUSION - CIRCULAR TRAJECTORY SIMULATION")
    print(f"Control Loop: {CONTROL_LOOP_MS}ms")
    print(f"Plotting: {'ENABLED' if ENABLE_PLOTTING else 'DISABLED'}")
    print("=" * 80)

    # Initialize KF
    Q = np.eye(3) * 0.05
    R1 = np.eye(3) * 0.02
    R2 = np.eye(6) * 0.5
    R2[3:6, 3:6] *= 0.2

    C1 = np.array([
        [0, 0, 0, 0, 0, 0, 1, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 1, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 1],
    ], dtype=float)

    C2 = np.array([
        [0, 0, 0, 0, 0, 0, 1, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 1, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 1],
        [1, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 1, 0, 0, 0, 0, 0, 0],
    ], dtype=float)

    kf = KF(Q, R1, R2, C1, C2, T=CONTROL_LOOP_MS / 1000.0)
    kf.setX(np.zeros((9, 1)))

    # Start sensor threads
    threads = [
        threading.Thread(target=imu_simulation_thread, daemon=True),
        threading.Thread(target=usbl_simulation_thread, daemon=True),
        threading.Thread(target=depth_simulation_thread, daemon=True),
    ]

    if ENABLE_PLOTTING:
        threads.append(threading.Thread(target=plot_thread, daemon=True))

    for t in threads:
        t.start()

    time.sleep(0.5)

    loop_time = CONTROL_LOOP_MS / 1000.0
    iteration = 0

    try:
        while True:
            start = time.time()
            timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]

            # Update simulation time
            with sim_lock:
                sim_time['t'] = iteration * loop_time
                t_sim = sim_time['t']

            # Get sensor data
            with data_lock:
                imu = sensor_data['imu'].copy()
                usbl = sensor_data['usbl'].copy()
                depth = sensor_data['depth'].copy()

            # USBL fix availability
            usbl_fix_available = (iteration % USBL_FIX_EVERY == 0)

            # Ground truth
            theta = OMEGA * t_sim
            p_true = np.array([
                RADIUS * math.cos(theta),
                RADIUS * math.sin(theta),
                0.0
            ])

            # KF update
            roll, pitch, yaw = imu['euler']
            a_body = np.array(imu['acc_body']).reshape((3, 1))

            if usbl_fix_available and usbl['valid']:
                p_world = np.array(usbl['pos_world']).reshape((3, 1))
                yy = np.vstack([a_body, p_world])
                kf.update(yy, roll=roll, pitch=pitch, yaw=yaw, queue=False, dt_override=loop_time)
            else:
                kf.update(a_body, roll=roll, pitch=pitch, yaw=yaw, queue=True, dt_override=loop_time)

            state = kf.getX()
            pos_est = state[0:3, 0]
            vel_est = state[3:6, 0]
            acc_est = state[6:9, 0]

            with kf_lock:
                kf_state['pos'] = pos_est.tolist()
                kf_state['vel'] = vel_est.tolist()
                kf_state['acc'] = acc_est.tolist()

            error_norm = np.linalg.norm(pos_est - p_true)

            if ENABLE_PLOTTING:
                with plot_lock:
                    plot_data['time'].append(t_sim)
                    plot_data['true_pos'].append(p_true.copy())
                    plot_data['est_pos'].append(pos_est.copy())
                    if usbl_fix_available and usbl['valid']:
                        plot_data['meas_pos'].append((t_sim, np.array(usbl['pos_world'])))
                    if len(plot_data['time']) > 0:
                        t_min = plot_data['time'][0]
                        plot_data['meas_pos'] = [(t, p) for t, p in plot_data['meas_pos'] if t >= t_min]
                    plot_data['error'].append(error_norm)

            # *** Stampa alleggerita ***
            if iteration % PRINT_EVERY == 0:
                if iteration > 0:
                    print('\033[15A\033[J', end='')

                print(f"\n{'='*80}")
                print(f"[{timestamp}] Iteration: {iteration:4d}  Time: {t_sim:6.2f}s")
                print(f"{'='*80}")

                print(f"TRUE   Pos: [{p_true[0]:7.3f}, {p_true[1]:7.3f}, {p_true[2]:7.3f}]")

                status = "OK" if imu['valid'] else "FAIL"
                print(f"IMU    [{status:4s}] Euler: [{roll:7.3f}, {pitch:7.3f}, {yaw:7.3f}]")
                print(f"              Acc_b: [{a_body[0,0]:7.3f}, {a_body[1,0]:7.3f}, {a_body[2,0]:7.3f}]")

                status = "FIX" if usbl_fix_available else "----"
                if usbl['valid']:
                    print(f"USBL   [{status:4s}] Pos: [{usbl['pos_world'][0]:7.3f}, "
                          f"{usbl['pos_world'][1]:7.3f}, {usbl['pos_world'][2]:7.3f}]")
                else:
                    print(f"USBL   [----] No data")

                status = "OK" if depth['valid'] else "FAIL"
                print(f"DEPTH  [{status:4s}] Z: {depth['depth']:7.3f} m (not used in KF)")

                print(f"\nKF EST Pos: [{pos_est[0]:7.3f}, {pos_est[1]:7.3f}, {pos_est[2]:7.3f}]")
                print(f"       Vel: [{vel_est[0]:7.3f}, {vel_est[1]:7.3f}, {vel_est[2]:7.3f}]")
                print(f"       Acc: [{acc_est[0]:7.3f}, {acc_est[1]:7.3f}, {acc_est[2]:7.3f}]")
                print(f"\nERROR: {error_norm:7.4f} m", flush=True)

            iteration += 1

            elapsed = time.time() - start
            sleep_time = max(0, loop_time - elapsed)
            time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\n\nShutting down...")
        running = False
        time.sleep(0.5)

if __name__ == "__main__":
    main()
