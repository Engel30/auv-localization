#!/usr/bin/env python3
"""
Log Analyzer per Distance Tracker

Analizza i file JSON log prodotti da distance_tracker.py
e genera statistiche e grafici.

Configurazione hardcoded - nessun argomento richiesto.
Esegui semplicemente: python3 analyze_log.py
"""

import json
import os
import glob
import sys
from datetime import datetime
from collections import defaultdict

# ==================== CONFIGURATION ====================

# Directory dove cercare i log
LOG_DIRECTORY = "src/test_USBL/usbl/logs"

# Quale file analizzare
ANALYZE_MODE = "LATEST"  # Opzioni: "LATEST", "ALL", "SPECIFIC"

# Se ANALYZE_MODE = "SPECIFIC", specifica il file
SPECIFIC_LOG_FILE = "./logs/distance_tracker_20241202_153000.jsonl"

# Pattern per trovare i file log
LOG_FILE_PATTERN = "tracker_log_*.jsonl"

# Cosa generare
GENERATE_CSV_DISTANCES = True      # Export distances.csv
GENERATE_CSV_USBL = True           # Export usbl_positions.csv
GENERATE_PLOT_DISTANCE = True      # Grafico distanza nel tempo
GENERATE_PLOT_TRAJECTORY = True    # Grafico traiettoria ENU

# Nome file output
OUTPUT_CSV_DISTANCES = "distances.csv"
OUTPUT_CSV_USBL = "usbl_positions.csv"
OUTPUT_PLOT_DISTANCE = "distance_plot.png"
OUTPUT_PLOT_TRAJECTORY = "trajectory_plot.png"

# DPI grafici
PLOT_DPI = 300

# Mostra grafici a schermo (oltre a salvarli)
SHOW_PLOTS = False  # True per visualizzare, False per solo salvare

# ========================================================


# ========================================================


def find_log_files(directory, pattern):
    """Trova tutti i file log nella directory"""
    search_path = os.path.join(directory, pattern)
    files = glob.glob(search_path)
    return sorted(files)  # Ordina per nome (che include timestamp)


def get_log_file_to_analyze():
    """Determina quale file log analizzare in base alla configurazione"""
    
    if ANALYZE_MODE == "SPECIFIC":
        if not os.path.exists(SPECIFIC_LOG_FILE):
            print(f"ERROR: File not found: {SPECIFIC_LOG_FILE}")
            return None
        return SPECIFIC_LOG_FILE
    
    # Trova tutti i file log
    log_files = find_log_files(LOG_DIRECTORY, LOG_FILE_PATTERN)
    
    if not log_files:
        print(f"ERROR: No log files found in {LOG_DIRECTORY} matching {LOG_FILE_PATTERN}")
        return None
    
    if ANALYZE_MODE == "LATEST":
        return log_files[-1]  # L'ultimo (più recente)
    
    elif ANALYZE_MODE == "ALL":
        return log_files  # Ritorna lista
    
    else:
        print(f"ERROR: Invalid ANALYZE_MODE: {ANALYZE_MODE}")
        print("Valid options: LATEST, ALL, SPECIFIC")
        return None


def load_events(filepath):
    """Carica tutti gli eventi dal file JSON Lines"""
    events = []
    try:
        with open(filepath, 'r') as f:
            for line_num, line in enumerate(f, 1):
                try:
                    event = json.loads(line.strip())
                    events.append(event)
                except json.JSONDecodeError as e:
                    print(f"Warning: Invalid JSON at line {line_num}: {e}")
    except FileNotFoundError:
        print(f"Error: File not found: {filepath}")
        sys.exit(1)
    
    return events


def analyze_session(events):
    """Analizza la sessione e genera statistiche"""
    
    # Estrai eventi per tipo
    by_type = defaultdict(list)
    for event in events:
        by_type[event['event_type']].append(event)
    
    print("="*70)
    print("SESSION ANALYSIS")
    print("="*70)
    
    # Session info
    if by_type['SESSION_START']:
        config = by_type['SESSION_START'][0]['configuration']
        print(f"\nCONFIGURATION:")
        print(f"  Transceiver ID:  {config['transceiver_id']}")
        print(f"  Transponder ID:  {config['transponder_id']}")
        print(f"  Sound Speed:     {config['sound_speed_mps']} m/s")
        print(f"  Message Interval: {config['message_interval']} s")
    
    if by_type['SESSION_END']:
        summary = by_type['SESSION_END'][0]['data']
        print(f"\nSESSION SUMMARY:")
        print(f"  Duration:        {summary['session_duration_s']:.1f} seconds ({summary['session_duration_s']/60:.1f} minutes)")
        print(f"  Messages Sent:   {summary['total_messages_sent']}")
        if summary['last_distance_m']:
            print(f"  Last Distance:   {summary['last_distance_m']:.2f} m")
    
    # Message delivery stats
    pings = by_type['PING_SENT']
    deliveries = by_type['DELIVERY_RESULT']
    
    successful = sum(1 for d in deliveries if d['data']['success'])
    failed = sum(1 for d in deliveries if not d['data']['success'])
    
    print(f"\nMESSAGE DELIVERY:")
    print(f"  Total Sent:      {len(pings)}")
    print(f"  Successful:      {successful} ({successful/len(pings)*100:.1f}%)" if pings else "  Successful:      0")
    print(f"  Failed:          {failed} ({failed/len(pings)*100:.1f}%)" if pings else "  Failed:          0")
    
    # Distance statistics
    distances = by_type['DISTANCE_ESTIMATION']
    if distances:
        dist_values = [d['data']['distance_m'] for d in distances]
        prop_times = [d['data']['propagation_time_ms'] for d in distances]
        
        print(f"\nDISTANCE STATISTICS (from AT?T):")
        print(f"  Measurements:    {len(distances)}")
        print(f"  Average:         {sum(dist_values)/len(dist_values):.2f} m")
        print(f"  Min:             {min(dist_values):.2f} m")
        print(f"  Max:             {max(dist_values):.2f} m")
        print(f"  Std Dev:         {std_dev(dist_values):.2f} m")
        print(f"  Avg Prop Time:   {sum(prop_times)/len(prop_times):.2f} ms")
    
    # USBL statistics
    usbllong = by_type['USBLLONG']
    usblangles = by_type['USBLANGLES']
    
    if usbllong:
        valid_usbl = [u for u in usbllong if u['data']['valid']]
        
        print(f"\nUSBL POSITIONING (USBLLONG):")
        print(f"  Total Fixes:     {len(usbllong)}")
        print(f"  Valid Fixes:     {len(valid_usbl)} ({len(valid_usbl)/len(usbllong)*100:.1f}%)")
        
        if valid_usbl:
            ranges = [u['data']['range_m'] for u in valid_usbl]
            uncertainties = [u['data']['uncertainty_m'] for u in valid_usbl]
            rssi_values = [u['data']['rssi_dbm'] for u in valid_usbl]
            integrity_values = [u['data']['integrity'] for u in valid_usbl]
            
            print(f"  Avg Range:       {sum(ranges)/len(ranges):.2f} m")
            print(f"  Avg Uncertainty: ±{sum(uncertainties)/len(uncertainties):.2f} m")
            print(f"  Avg RSSI:        {sum(rssi_values)/len(rssi_values):.1f} dBm")
            print(f"  Avg Integrity:   {sum(integrity_values)/len(integrity_values):.1f}")
            
            # Position stats
            east_values = [u['data']['east_m'] for u in valid_usbl]
            north_values = [u['data']['north_m'] for u in valid_usbl]
            up_values = [u['data']['up_m'] for u in valid_usbl]
            
            print(f"\n  Position Range:")
            print(f"    East:  {min(east_values):.2f} to {max(east_values):.2f} m")
            print(f"    North: {min(north_values):.2f} to {max(north_values):.2f} m")
            print(f"    Up:    {min(up_values):.2f} to {max(up_values):.2f} m")
    
    if usblangles:
        valid_angles = [u for u in usblangles if u['data']['valid']]
        
        print(f"\nUSBL DIRECTION (USBLANGLES):")
        print(f"  Total:           {len(usblangles)}")
        print(f"  Valid:           {len(valid_angles)} ({len(valid_angles)/len(usblangles)*100:.1f}%)")
        
        if valid_angles:
            bearings = [u['data']['bearing_deg'] for u in valid_angles]
            elevations = [u['data']['elevation_deg'] for u in valid_angles]
            
            print(f"  Avg Bearing:     {sum(bearings)/len(bearings):.2f}°")
            print(f"  Avg Elevation:   {sum(elevations)/len(elevations):.2f}°")
    
    print("\n" + "="*70)
    
    return by_type


def std_dev(values):
    """Calcola deviazione standard"""
    if not values:
        return 0.0
    mean = sum(values) / len(values)
    variance = sum((x - mean) ** 2 for x in values) / len(values)
    return variance ** 0.5


def export_distances_csv(events, output_file=None):
    """Esporta le distanze in formato CSV"""
    import csv
    
    if output_file is None:
        output_file = OUTPUT_CSV_DISTANCES
    
    if not GENERATE_CSV_DISTANCES:
        return
    
    distances = [e for e in events if e['event_type'] == 'DISTANCE_ESTIMATION']
    
    if not distances:
        print("No distance data to export")
        return
    
    with open(output_file, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow([
            'Timestamp', 'Counter', 'Distance_m', 
            'Propagation_Time_ms', 'One_Way_Time_ms'
        ])
        
        for event in distances:
            data = event['data']
            writer.writerow([
                event['timestamp'],
                data['counter'],
                data['distance_m'],
                data['propagation_time_ms'],
                data['one_way_time_ms']
            ])
    
    print(f"✓ Distances exported to: {output_file}")


def export_usbl_csv(events, output_file=None):
    """Esporta le posizioni USBL in formato CSV"""
    import csv
    
    if output_file is None:
        output_file = OUTPUT_CSV_USBL
    
    if not GENERATE_CSV_USBL:
        return
    
    usbl_events = [e for e in events if e['event_type'] == 'USBLLONG' and e['data']['valid']]
    
    if not usbl_events:
        print("No USBL data to export")
        return
    
    with open(output_file, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow([
            'Timestamp', 'Counter', 'East_m', 'North_m', 'Up_m',
            'Range_m', 'Uncertainty_m', 'RSSI_dBm', 'Integrity'
        ])
        
        for event in usbl_events:
            data = event['data']
            writer.writerow([
                event['timestamp'],
                data['counter'],
                data['east_m'],
                data['north_m'],
                data['up_m'],
                data['range_m'],
                data['uncertainty_m'],
                data['rssi_dbm'],
                data['integrity']
            ])
    
    print(f"✓ USBL positions exported to: {output_file}")


def plot_distance_timeline(events):
    """Plot distanza nel tempo (richiede matplotlib)"""
    
    if not GENERATE_PLOT_DISTANCE:
        return
    
    try:
        import matplotlib.pyplot as plt
        from datetime import datetime
    except ImportError:
        print("matplotlib not installed - skipping distance plot")
        return
    
    distances = [e for e in events if e['event_type'] == 'DISTANCE_ESTIMATION']
    
    if not distances:
        print("No distance data to plot")
        return
    
    timestamps = [datetime.fromisoformat(e['timestamp']) for e in distances]
    dist_values = [e['data']['distance_m'] for e in distances]
    
    plt.figure(figsize=(12, 6))
    plt.plot(timestamps, dist_values, marker='o', linestyle='-', linewidth=2, markersize=4)
    plt.xlabel('Time')
    plt.ylabel('Distance (m)')
    plt.title('Distance Tracking Over Time')
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    
    plt.savefig(OUTPUT_PLOT_DISTANCE, dpi=PLOT_DPI)
    print(f"✓ Distance plot saved to: {OUTPUT_PLOT_DISTANCE}")
    
    # Mostra grafico se richiesto
    if SHOW_PLOTS:
        try:
            plt.show()
        except:
            pass
    
    plt.close()


def plot_trajectory(events):
    """Plot traiettoria ENU (richiede matplotlib)"""
    
    if not GENERATE_PLOT_TRAJECTORY:
        return
    
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib not installed - skipping trajectory plot")
        return
    
    usbl_events = [e for e in events if e['event_type'] == 'USBLLONG' and e['data']['valid']]
    
    if not usbl_events:
        print("No USBL data to plot")
        return
    
    east = [e['data']['east_m'] for e in usbl_events]
    north = [e['data']['north_m'] for e in usbl_events]
    
    plt.figure(figsize=(10, 10))
    plt.plot(east, north, marker='o', linestyle='-', linewidth=2, markersize=6, alpha=0.7)
    plt.scatter(east[0], north[0], color='green', s=200, marker='o', label='Start', zorder=5)
    plt.scatter(east[-1], north[-1], color='red', s=200, marker='x', label='End', zorder=5)
    plt.xlabel('East (m)')
    plt.ylabel('North (m)')
    plt.title('Transponder Trajectory (ENU Frame)')
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.axis('equal')
    plt.tight_layout()
    
    plt.savefig(OUTPUT_PLOT_TRAJECTORY, dpi=PLOT_DPI)
    print(f"✓ Trajectory plot saved to: {OUTPUT_PLOT_TRAJECTORY}")
    
    # Mostra grafico se richiesto
    if SHOW_PLOTS:
        try:
            plt.show()
        except:
            pass
    
    plt.close()


def main():
    print("="*70)
    print("LOG ANALYZER - Distance Tracker")
    print("="*70)
    print(f"Configuration:")
    print(f"  Log Directory:     {LOG_DIRECTORY}")
    print(f"  Analyze Mode:      {ANALYZE_MODE}")
    if ANALYZE_MODE == "SPECIFIC":
        print(f"  Specific File:     {SPECIFIC_LOG_FILE}")
    print(f"  Generate CSV Dist: {GENERATE_CSV_DISTANCES}")
    print(f"  Generate CSV USBL: {GENERATE_CSV_USBL}")
    print(f"  Generate Plot Dist: {GENERATE_PLOT_DISTANCE}")
    print(f"  Generate Plot Traj: {GENERATE_PLOT_TRAJECTORY}")
    print("="*70)
    print()
    
    # Trova file da analizzare
    log_file = get_log_file_to_analyze()
    
    if log_file is None:
        print("ERROR: Could not determine log file to analyze")
        print("Check configuration at the top of this script")
        return
    
    # Se ANALYZE_MODE = "ALL", processa tutti i file
    if ANALYZE_MODE == "ALL" and isinstance(log_file, list):
        print(f"Found {len(log_file)} log files to analyze:")
        for i, f in enumerate(log_file, 1):
            print(f"  {i}. {f}")
        print()
        
        # Combina tutti gli eventi
        all_events = []
        for f in log_file:
            print(f"Loading: {f}")
            events = load_events(f)
            all_events.extend(events)
        
        print(f"Total events loaded: {len(all_events)}\n")
        events = all_events
        
    else:
        # Analizza singolo file
        print(f"Analyzing: {log_file}\n")
        events = load_events(log_file)
        print(f"Loaded {len(events)} events\n")
    
    # Analizza
    by_type = analyze_session(events)
    
    # Esporta CSV
    if GENERATE_CSV_DISTANCES or GENERATE_CSV_USBL:
        print("\nEXPORTING DATA:")
        export_distances_csv(events)
        export_usbl_csv(events)
    
    # Genera grafici
    if GENERATE_PLOT_DISTANCE or GENERATE_PLOT_TRAJECTORY:
        print("\nGENERATING PLOTS:")
        plot_distance_timeline(events)
        plot_trajectory(events)
    
    print("\n" + "="*70)
    print("Analysis complete!")
    print("="*70)


if __name__ == '__main__':
    main()