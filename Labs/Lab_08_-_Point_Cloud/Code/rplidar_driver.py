import sys
import os
import platform
import time
import json
from rplidar import RPLidar, RPLidarException

class LidarDriver:
    def __init__(self, mode='live', filename='lidar_data.json', baudrate=256000):
        self.mode = mode
        self.filename = filename
        self.baudrate = baudrate
        self.lidar = None
        self.file_handle = None
        self.port_name = None
        self._is_running = False

        if self.mode == 'live':
            self.port_name = self._detect_port()
            self._setup_live_connection()
            if self.filename:
                print(f"[Logger] Data will be saved to: {self.filename}")
                self.file_handle = open(self.filename, 'w')
        
        elif self.mode == 'replay':
            if not os.path.exists(self.filename):
                raise FileNotFoundError(f"Cannot replay. File not found: {self.filename}")
            print(f"[Replay] Reading data from: {self.filename}")
            self.file_handle = open(self.filename, 'r')

    def _detect_port(self):
        os_name = platform.system()
        if os_name == 'Windows':
            return 'COM8'
        elif os_name == 'Darwin':
            return '/dev/tty.SLAB_USBtoUART'
        else:
            return '/dev/ttyUSB0'

    def _setup_live_connection(self):
        try:
            print(f"[Connect] Connecting to LIDAR on {self.port_name}...")
            self.lidar = RPLidar(self.port_name, baudrate=self.baudrate)
            # We skip get_info() sometimes as it can hang on some firmware versions
            print("[Connect] Connected.") 
        except Exception as e:
            print(f"[Error] Could not connect: {e}")
            sys.exit(1)

    def iter_scans(self):
        """
        Safe generator that handles Ctrl+C internally to ensure file saving.
        """
        self._is_running = True
        scan_count = 0
        
        print("[Stream] Starting... Press Ctrl+C to stop.")
        
        try:
            if self.mode == 'live':
                # We use the native iterator but wrap it to catch interruptions
                iterator = self.lidar.iter_scans()
                while self._is_running:
                    try:
                        # Get next scan
                        scan = next(iterator)
                        
                        # Write to file
                        if self.file_handle:
                            json.dump(scan, self.file_handle)
                            self.file_handle.write("\n")
                        
                        yield scan
                        scan_count += 1
                        
                    except StopIteration:
                        break
                    except RPLidarException as e:
                        # Common sensor glitch, ignore and continue
                        continue 

            elif self.mode == 'replay':
                for line in self.file_handle:
                    if not self._is_running: break
                    if not line.strip(): continue
                    
                    scan = json.loads(line)
                    time.sleep(0.1) # Simulate sensor delay
                    yield scan

        except KeyboardInterrupt:
            print("\n[Stop] Ctrl+C detected. Saving data...")
        except Exception as e:
            print(f"\n[Error] Unexpected error: {e}")
        finally:
            self.disconnect()

    def disconnect(self):
        """
        Aggressive cleanup to prevent hanging.
        """
        self._is_running = False
        
        # 1. Close file FIRST to save data
        if self.file_handle:
            try:
                self.file_handle.flush()
                self.file_handle.close()
                print(f"[File] Successfully saved: {self.filename}")
            except Exception as e:
                print(f"[File] Error closing file: {e}")
            self.file_handle = None

        # 2. Close Lidar connection
        if self.lidar:
            print("[Hardware] Stopping Lidar motor...")
            try:
                self.lidar.stop()
                self.lidar.stop_motor()
                # Short pause to let command send
                time.sleep(0.5) 
            except:
                pass # If motor stop fails, ignore it
            
            try:
                self.lidar.disconnect()
                print("[Hardware] Disconnected.")
            except:
                pass
            self.lidar = None