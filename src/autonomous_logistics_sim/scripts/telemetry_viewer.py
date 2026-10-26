#!/usr/bin/env python3
from multiprocessing import shared_memory
import struct, time, sys

def run_viewer():
    print("--- WAITING FOR TELEMETRY ---")
    try:
        # Connect to the memory block created by the node
        shm = shared_memory.SharedMemory(name="telemetry_shm")
        while True:
            # Matches the struct.pack('dddidi', ...)
            data = struct.unpack('dddidi', shm.buf[:40])
            status = "🏁 COMPLETED" if data[5] else f"🎯 TASK #{data[3]}"
            # Use carriage return \r to update the same line
            sys.stdout.write(f"\r{status:<15} | Pos: ({data[0]:5.2f}, {data[1]:5.2f}) | Yaw: {data[2]:6.1f}° | Obs: {data[4]:.2f}m")
            sys.stdout.flush()
            time.sleep(0.3)
    except Exception as e:
        print(f"\nViewer Error: {e}")

if __name__ == '__main__':
    run_viewer()
