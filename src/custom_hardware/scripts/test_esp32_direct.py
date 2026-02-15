#!/usr/bin/env python3
"""
Direct TCP test for ESP32 servo controller.
Use this to test ESP32 communication WITHOUT ROS2.

Usage:
    python3 test_esp32_direct.py --ip 192.168.1.100 --port 5000
"""

import socket
import argparse
import time
import sys


def send_command(sock, cmd):
    """Send command and receive response."""
    print(f"TX: {cmd.strip()}")
    sock.sendall((cmd + "\n").encode())
    
    # Wait for response
    try:
        sock.settimeout(1.0)
        response = sock.recv(256).decode().strip()
        print(f"RX: {response}")
        return response
    except socket.timeout:
        print("RX: (no response)")
        return None


def main():
    parser = argparse.ArgumentParser(description='Test ESP32 servo controller directly')
    parser.add_argument('--ip', default='192.168.1.100', help='ESP32 IP address')
    parser.add_argument('--port', type=int, default=5000, help='ESP32 TCP port')
    args = parser.parse_args()
    
    print("="*50)
    print("ESP32 Servo Controller Direct Test")
    print("="*50)
    print(f"Connecting to {args.ip}:{args.port}...")
    
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(5.0)
        sock.connect((args.ip, args.port))
        print("Connected!\n")
    except Exception as e:
        print(f"Connection failed: {e}")
        print("\nMake sure:")
        print("  1. ESP32 is powered on and connected to WiFi")
        print("  2. ESP32 IP address is correct")
        print("  3. Your computer is on the same network")
        sys.exit(1)
    
    try:
        # Initialize
        print("--- Sending INIT ---")
        send_command(sock, "INIT")
        time.sleep(0.5)
        
        # Get current positions
        print("\n--- Getting current positions ---")
        send_command(sock, "GET")
        time.sleep(0.5)
        
        # Move to home position (all 90 degrees)
        print("\n--- Moving to home position (all 90°) ---")
        send_command(sock, "POS:90,90,90,90,90,90")
        time.sleep(2)
        
        # Get positions
        send_command(sock, "GET")
        time.sleep(0.5)
        
        # Test joint 1 movement
        print("\n--- Moving joint 1 to 60° ---")
        send_command(sock, "POS:60,90,90,90,90,90")
        time.sleep(2)
        
        print("\n--- Moving joint 1 to 120° ---")
        send_command(sock, "POS:120,90,90,90,90,90")
        time.sleep(2)
        
        # Test joint 2 movement
        print("\n--- Moving joint 2 to 60° ---")
        send_command(sock, "POS:90,60,90,90,90,90")
        time.sleep(2)
        
        print("\n--- Moving joint 2 to 120° ---")
        send_command(sock, "POS:90,120,90,90,90,90")
        time.sleep(2)
        
        # Return to home
        print("\n--- Returning to home ---")
        send_command(sock, "POS:90,90,90,90,90,90")
        time.sleep(2)
        
        # Get final positions
        print("\n--- Final positions ---")
        send_command(sock, "GET")
        
        print("\n" + "="*50)
        print("Test completed!")
        print("="*50)
        
    except KeyboardInterrupt:
        print("\n\nTest cancelled")
    finally:
        # Stop and close
        send_command(sock, "STOP")
        sock.close()
        print("\nConnection closed")


if __name__ == '__main__':
    main()
