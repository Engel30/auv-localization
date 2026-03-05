#!/usr/bin/env python3
"""
USBL Transponder Library for Jetson TX2 (Python 3.6.9)
Provides non-blocking USBL position tracking with automatic fallback to simulation
"""
import asyncio
import json
import math
import threading
import time
from datetime import datetime
from typing import Dict, Any, Tuple, Optional


class USBLTransponder:
    """USBL Transponder client for position tracking"""
    
    def __init__(self, transponder_id=3, transceiver_id=2, 
                 usbl_ip="192.168.0.232", usbl_port=9200,
                 request_interval=2.0, min_integrity=50,
                 speed_of_sound=1500.0):
        """
        Initialize USBL Transponder
        
        Args:
            transponder_id: ID of this transponder
            transceiver_id: ID of the target transceiver
            usbl_ip: IP address of USBL modem
            usbl_port: Port of USBL modem
            request_interval: Time between position requests (seconds)
            min_integrity: Minimum integrity threshold for valid positions
            speed_of_sound: Speed of sound in water (m/s)
        """
        self.transponder_id = transponder_id
        self.transceiver_id = transceiver_id
        self.usbl_ip = usbl_ip
        self.usbl_port = usbl_port
        self.request_interval = request_interval
        self.min_integrity = min_integrity
        self.speed_of_sound = speed_of_sound
        self.codec = "utf-8"
        
        # State
        self.connected = False
        self.running = False
        self.request_count = 0
        self.position_count = 0
        self.last_position = None  # (e, n, u, distance, timestamp)
        self.last_rssi = None
        self.last_integrity = None
        
        # Thread control
        self._thread = None
        self._loop = None
        self._reader = None
        self._writer = None
        self._response_event = None
        self._data_lock = threading.Lock()
        
        # Message keywords
        self.instant_message_keywords = {
            "SENDSTART", "SENDEND", "RECVSTART", "RECVEND", "DELIVEREDIM", "USBLLONG", 
            "USBLANGLES", "USBLPHYD", "USBLPHYP", "FAILEDIM", "RECVIM", "CANCELEDIM", 
            "ERROR INTERNAL"
        }
    
    def start(self):
        """Start the USBL transponder in a background thread"""
        if self.running:
            return
        
        self.running = True
        self._thread = threading.Thread(target=self._run_async_loop, daemon=True)
        self._thread.start()
        
        # Wait a bit for connection attempt
        time.sleep(0.5)
    
    def stop(self):
        """Stop the USBL transponder"""
        self.running = False
        if self._thread:
            self._thread.join(timeout=2.0)
    
    def get_position(self):
        """
        Get the last received position
        
        Returns:
            dict with keys: 'valid', 'pos_world' [e, n, u], 'distance', 
                           'rssi', 'integrity', 'timestamp'
        """
        with self._data_lock:
            if self.last_position is None:
                return {
                    'valid': False,
                    'pos_world': [0.0, 0.0, 0.0],
                    'distance': 0.0,
                    'rssi': None,
                    'integrity': None,
                    'timestamp': None
                }
            
            e, n, u, dist, ts = self.last_position
            return {
                'valid': True,
                'pos_world': [e, n, u],
                'distance': dist,
                'rssi': self.last_rssi,
                'integrity': self.last_integrity,
                'timestamp': ts
            }
    
    def is_connected(self):
        """Check if USBL is connected"""
        return self.connected
    
    def get_stats(self):
        """Get statistics"""
        with self._data_lock:
            return {
                'connected': self.connected,
                'requests_sent': self.request_count,
                'positions_received': self.position_count
            }
    
    # ==================== INTERNAL METHODS ====================
    
    def _run_async_loop(self):
        """Run the asyncio event loop in this thread"""
        # Create new event loop for this thread (Python 3.6 compatible)
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        
        try:
            self._loop.run_until_complete(self._async_main())
        except Exception as e:
            print("[USBL] Error in async loop: {}".format(e))
        finally:
            self._loop.close()
    
    async def _async_main(self):
        """Main async function"""
        try:
            # Try to connect
            self._reader, self._writer = await asyncio.wait_for(
                asyncio.open_connection(self.usbl_ip, self.usbl_port),
                timeout=3.0
            )
            
            self.connected = True
            print("[USBL] Connected to {}:{}".format(self.usbl_ip, self.usbl_port))
            
            # Create response event
            self._response_event = asyncio.Event()
            
            # Start reader and requester tasks
            reader_task = asyncio.ensure_future(self._response_reader())
            requester_task = asyncio.ensure_future(self._position_requester())
            
            # Wait for both tasks
            await asyncio.gather(reader_task, requester_task)
            
        except asyncio.TimeoutError:
            print("[USBL] Connection timeout - running in simulation mode")
            self.connected = False
        except Exception as e:
            print("[USBL] Connection failed: {} - running in simulation mode".format(e))
            self.connected = False
        finally:
            if self._writer:
                self._writer.close()
                await self._writer.wait_closed()
    
    async def _response_reader(self):
        """Read and parse responses from USBL"""
        try:
            while self.running:
                line = await self._reader.readline()
                if not line:
                    print("[USBL] Connection closed")
                    self.connected = False
                    break
                
                msg_type, data = self._decode_response(line)
                
                if msg_type == "DELIVEREDIM":
                    pass  # Position request delivered
                
                elif msg_type == "FAILEDIM":
                    print("[USBL] Position request FAILED")
                    if self._response_event:
                        self._response_event.set()
                
                elif msg_type == "RECVIM":
                    if data.get("valid", False):
                        msg = data.get("message")
                        
                        if msg and "e" in msg and "n" in msg and "u" in msg:
                            e, n, u = msg["e"], msg["n"], msg["u"]
                            dist = self._calculate_distance_3d(e, n, u)
                            ts = time.time()
                            
                            with self._data_lock:
                                self.position_count += 1
                                self.last_position = (e, n, u, dist, ts)
                                self.last_rssi = data.get('rssi')
                                self.last_integrity = data.get('integrity')
                            
                            # Signal response received
                            if self._response_event:
                                self._response_event.set()
                
        except asyncio.CancelledError:
            pass
        except Exception as e:
            print("[USBL] Response reader error: {}".format(e))
            self.connected = False
    
    async def _position_requester(self):
        """Periodically request positions"""
        await asyncio.sleep(1.0)  # Initial delay
        
        try:
            while self.running:
                # Send position request
                self.request_count += 1
                
                if self._response_event:
                    self._response_event.clear()
                
                request_msg = {"r": self.request_count}
                
                try:
                    encoded = self._encode_message(self.transceiver_id, request_msg)
                    self._writer.write(encoded)
                    await self._writer.drain()
                except Exception as e:
                    print("[USBL] Send error: {}".format(e))
                    self.connected = False
                    break
                
                # Wait for response (with timeout)
                if self._response_event:
                    try:
                        await asyncio.wait_for(
                            self._response_event.wait(), 
                            timeout=10.0
                        )
                    except asyncio.TimeoutError:
                        pass  # Timeout, will retry
                
                # Wait before next request
                await asyncio.sleep(self.request_interval)
                
        except asyncio.CancelledError:
            pass
        except Exception as e:
            print("[USBL] Position requester error: {}".format(e))
            self.connected = False
    
    def _encode_message(self, target_id, payload):
        """Encode message for USBL"""
        payload_str = json.dumps(payload, separators=(',', ':'))
        if len(payload_str) > 63:
            raise ValueError("Message too long: {} bytes".format(len(payload_str)))
        payload_str = "[{}".format(payload_str)
        command = "AT*SENDIM,p0,{},{},ack,{}".format(
            len(payload_str), target_id, payload_str
        )
        return "+++{}\n".format(command).encode(self.codec)
    
    def _decode_response(self, message):
        """Decode response from USBL"""
        try:
            str_msg = message.decode(self.codec).strip()
            if not str_msg or not str_msg.startswith("+++"):
                return "UNKNOWN", {}
            
            parts = str_msg.split(":")
            
            if "AT" in str_msg:
                if len(parts) >= 3:
                    args = parts[2].split(",")
                    keyword = args[0]
                    
                    if keyword == "DELIVEREDIM":
                        return "DELIVEREDIM", {}
                    
                    elif keyword == "FAILEDIM":
                        return "FAILEDIM", {}
                    
                    elif keyword == "RECVIM" and len(args) >= 9:
                        try:
                            sender_id = args[3]
                            rssi = int(args[7])
                            integrity = int(args[8])
                            payload = None
                            
                            if ',[' in str_msg:
                                _, payload = str_msg.rsplit(',[', 1)
                                payload = payload.strip()
                                if payload.startswith("{"):
                                    try:
                                        payload = json.loads(payload)
                                    except:
                                        pass
                            
                            return "RECVIM", {
                                "sender_id": sender_id,
                                "integrity": integrity,
                                "valid": integrity > self.min_integrity,
                                "rssi": rssi,
                                "message": payload
                            }
                        except (ValueError, IndexError):
                            return "RECVIM_ERROR", {}
            
            return "OTHER", {}
            
        except Exception:
            return "ERROR", {}
    
    @staticmethod
    def _calculate_distance_3d(east, north, up):
        """Calculate 3D distance"""
        return math.sqrt(east**2 + north**2 + up**2)


# ==================== HELPER FUNCTION ====================

def create_usbl_transponder(transponder_id=3, transceiver_id=2,
                           usbl_ip="192.168.0.232", usbl_port=9200,
                           request_interval=2.0):
    """
    Create and start a USBL transponder
    
    Returns:
        USBLTransponder instance
    """
    usbl = USBLTransponder(
        transponder_id=transponder_id,
        transceiver_id=transceiver_id,
        usbl_ip=usbl_ip,
        usbl_port=usbl_port,
        request_interval=request_interval
    )
    usbl.start()
    return usbl