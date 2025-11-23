#!/usr/bin/env python3
import time
import math
import utm
from USBL import Usbl, INSTANT_MSG

# Parametri hardcoded
HOST = "192.168.0.138"
PORT = 9200
REMOTE_ADDR = '2'

def test_position_request():
    """Interroga base per posizione GPS e riceve posizioni ENU"""
    
    print(f"Connessione USBL: {HOST}:{PORT}")
    usbl = Usbl(HOST, PORT)
    time.sleep(1)
    
    # Posizione base
    base_wgs84 = None
    base_utm = None
    
    # Posizioni ROV
    enu_positions = []
    
    print(f"\n{'='*60}")
    print("Test richiesta posizioni con conversioni coordinate")
    print(f"{'='*60}\n")
    
    # Richiesta posizione base
    print("[TX] Richiesta posizione base GPS...")
    msg = 'POS'
    im = INSTANT_MSG("0", REMOTE_ADDR, "ack", len(msg), msg, True)
    usbl.write_data(im.message)
    
    print("\nIn attesa risposte...\n")
    
    try:
        while True:
            data = usbl.read_data()
            
            if data and data != "ERROR":
                campi = usbl.parse_msg()
                
                if len(campi) > 0 and campi[0] == "RECVIM":
                    if len(campi) > 10:
                        payload = campi[10].split(';')
                        msg_type = payload[0]
                        
                        # Risposta posizione base GPS (WGS84)
                        if msg_type == '0' and len(payload) >= 3:
                            lat = float(payload[1])
                            
                            # Gestisce sia formato con che senza depth
                            if len(payload) >= 4:
                                lon = float(payload[2])
                                depth = float(payload[3].split('\r')[0])
                            else:
                                lon_depth = payload[2].split('\r')[0]
                                lon = float(lon_depth)
                                depth = 0.0
                            
                            # Salva WGS84
                            base_wgs84 = {'lat': lat, 'lon': lon, 'depth': depth}
                            
                            # Converti in UTM
                            (x_utm, y_utm, zone_number, zone_letter) = utm.from_latlon(lat, lon)
                            base_utm = {
                                'x': x_utm,
                                'y': y_utm,
                                'zone_number': zone_number,
                                'zone_letter': zone_letter
                            }
                            
                            print(f"{'='*60}")
                            print("POSIZIONE BASE RICEVUTA")
                            print(f"{'='*60}")
                            print(f"\nWGS84 (GPS):")
                            print(f"  Latitudine:  {lat:.8f}°")
                            print(f"  Longitudine: {lon:.8f}°")
                            if depth > 0:
                                print(f"  Profondità:  {depth:.2f} m")
                            
                            print(f"\nUTM:")
                            print(f"  X (Easting):  {x_utm:.2f} m")
                            print(f"  Y (Northing): {y_utm:.2f} m")
                            print(f"  Zona: {zone_number}{zone_letter}")
                            print(f"{'='*60}\n")
                        
                        # Posizione ENU del ROV
                        elif msg_type == '2' and len(payload) >= 3:
                            x_enu = float(payload[1])
                            y_raw = payload[2].split('\r')[0]
                            y_enu = float(y_raw)
                            
                            enu_positions.append({
                                'x': x_enu, 
                                'y': y_enu, 
                                'time': time.time()
                            })
                            
                            print(f"{'─'*60}")
                            print(f"POSIZIONE ROV #{len(enu_positions)}")
                            print(f"{'─'*60}")
                            
                            print(f"\nENU (relative alla base):")
                            print(f"  Est (X):   {x_enu:+8.2f} m")
                            print(f"  Nord (Y):  {y_enu:+8.2f} m")
                            
                            if base_utm:
                                # Calcola posizione assoluta ROV in UTM
                                rov_utm_x = base_utm['x'] + x_enu
                                rov_utm_y = base_utm['y'] + y_enu
                                
                                print(f"\nUTM assoluto ROV:")
                                print(f"  X (Easting):  {rov_utm_x:.2f} m")
                                print(f"  Y (Northing): {rov_utm_y:.2f} m")
                                print(f"  Zona: {base_utm['zone_number']}{base_utm['zone_letter']}")
                                
                                # Converti in WGS84
                                (rov_lat, rov_lon) = utm.to_latlon(
                                    rov_utm_x, 
                                    rov_utm_y, 
                                    base_utm['zone_number'], 
                                    base_utm['zone_letter']
                                )
                                
                                print(f"\nWGS84 ROV:")
                                print(f"  Latitudine:  {rov_lat:.8f}°")
                                print(f"  Longitudine: {rov_lon:.8f}°")
                                
                                # Calcola distanza dalla base
                                dist = math.sqrt(x_enu**2 + y_enu**2)
                                bearing = math.degrees(math.atan2(x_enu, y_enu)) % 360
                                
                                print(f"\nDistanza dalla base:")
                                print(f"  Distanza: {dist:.2f} m")
                                print(f"  Direzione: {bearing:.1f}° (0°=Nord, 90°=Est)")
                            else:
                                print("\n[Attendo coordinate base per calcolo posizione assoluta...]")
                            
                            print()
            
            time.sleep(0.1)
    
    except KeyboardInterrupt:
        print(f"\n\n{'='*60}")
        print("RIEPILOGO FINALE")
        print(f"{'='*60}")
        
        if base_wgs84:
            print("\n▸ POSIZIONE BASE:")
            print(f"  WGS84: {base_wgs84['lat']:.8f}°, {base_wgs84['lon']:.8f}°")
            if base_utm:
                print(f"  UTM:   X={base_utm['x']:.2f}m, Y={base_utm['y']:.2f}m ({base_utm['zone_number']}{base_utm['zone_letter']})")
        else:
            print("\n▸ POSIZIONE BASE: NON RICEVUTA")
        
        if enu_positions:
            print(f"\n▸ POSIZIONI ENU RICEVUTE: {len(enu_positions)}")
            
            # Ultimi 3 aggiornamenti
            print("\n  Ultimi 3 aggiornamenti ROV:")
            for i, pos in enumerate(enu_positions[-3:], 1):
                print(f"\n  #{len(enu_positions)-3+i}:")
                print(f"    ENU: X={pos['x']:+.2f}m, Y={pos['y']:+.2f}m")
                
                if base_utm:
                    rov_x = base_utm['x'] + pos['x']
                    rov_y = base_utm['y'] + pos['y']
                    (rov_lat, rov_lon) = utm.to_latlon(
                        rov_x, rov_y,
                        base_utm['zone_number'],
                        base_utm['zone_letter']
                    )
                    print(f"    UTM: X={rov_x:.2f}m, Y={rov_y:.2f}m")
                    print(f"    WGS84: {rov_lat:.8f}°, {rov_lon:.8f}°")
        else:
            print("\n▸ POSIZIONI ENU: NESSUNA RICEVUTA")
        
        print(f"\n{'='*60}\n")
        
        usbl.close()

if __name__ == "__main__":
    test_position_request()