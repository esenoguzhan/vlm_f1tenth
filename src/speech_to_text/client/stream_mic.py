import socket
import pyaudio
import sys
import time

# Configuration
# ==========================================
# REPLACE THIS WITH YOUR JETSON'S IP ADDRESS
JETSON_IP = '10.183.170.108'  
# ==========================================

PORT = 9000
CHUNK = 1024
FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 16000

def stream_audio():
    p = pyaudio.PyAudio()

    try:
        # Create socket
        client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        print(f"Connecting to Jetson at {JETSON_IP}:{PORT}...")
        try:
            client_socket.connect((JETSON_IP, PORT))
            print("Connected! Streaming microphone...")
        except ConnectionRefusedError:
            print("Failed to connect. Make sure the ROS2 node is running first.")
            return

        # Open Microphone Stream
        stream = p.open(format=FORMAT,
                        channels=CHANNELS,
                        rate=RATE,
                        input=True,
                        frames_per_buffer=CHUNK)

        print("* Recording... (Press Ctrl+C to stop)")
        
        while True:
            try:
                data = stream.read(CHUNK, exception_on_overflow=False)
                client_socket.sendall(data)
            except KeyboardInterrupt:
                break
            except Exception as e:
                print(f"Error sending data: {e}")
                break

    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        if 'stream' in locals():
            stream.stop_stream()
            stream.close()
        p.terminate()
        if 'client_socket' in locals():
            client_socket.close()
        print("Disconnected.")

if __name__ == "__main__":
    if len(sys.argv) > 1:
        JETSON_IP = sys.argv[1]
    
    print(f"Target Jetson IP: {JETSON_IP}")
    stream_audio()
