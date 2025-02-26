import socket
import json
import threading
import time

# Define default port for communication
DEFAULT_PORT = 12345
DEFAULT_HOST = '127.0.0.1'

class PlotSignals:
    """
    Class to handle communication between plot scripts and main GUI
    using a simple UDP socket for messaging.
    """
    def __init__(self, port=DEFAULT_PORT, host=DEFAULT_HOST, is_server=False):
        self.port = port
        self.host = host
        self.is_server = is_server
        self.socket = None
        self.running = False
        self.callbacks = {}
        self._setup_socket()
        
    def _setup_socket(self):
        """Setup the socket for either server or client mode."""
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            if self.is_server:
                # Server binds to the port to receive messages
                self.socket.bind((self.host, self.port))
                self.socket.settimeout(0.1)  # Small timeout for non-blocking behavior
        except Exception as e:
            print(f"Socket setup error: {e}")
            
    def start_server(self):
        """Start the server to listen for messages from plot scripts."""
        if not self.is_server or self.running:
            return
            
        self.running = True
        self.listener_thread = threading.Thread(target=self._listen_for_messages)
        self.listener_thread.daemon = True
        self.listener_thread.start()
        
    def _listen_for_messages(self):
        """Listen for incoming messages in a loop."""
        while self.running:
            try:
                data, addr = self.socket.recvfrom(1024)
                message = json.loads(data.decode('utf-8'))
                
                # Process received message
                message_type = message.get('type')
                message_data = message.get('data')
                
                # Call appropriate callback if registered
                if message_type in self.callbacks:
                    self.callbacks[message_type](message_data)
            except socket.timeout:
                # This is normal, just loop again
                pass
            except Exception as e:
                print(f"Error receiving message: {e}")
                
            time.sleep(0.01)  # Prevent CPU overload
    
    def send_message(self, message_type, message_data):
        """Send a message from the plot script to the main GUI."""
        if self.is_server:
            return
            
        try:
            message = {
                'type': message_type,
                'data': message_data
            }
            self.socket.sendto(json.dumps(message).encode('utf-8'), (self.host, self.port))
        except Exception as e:
            print(f"Error sending message: {e}")
    
    def register_callback(self, message_type, callback):
        """Register a callback function for a specific message type."""
        if self.is_server:
            self.callbacks[message_type] = callback
    
    def stop(self):
        """Stop the server or client."""
        self.running = False
        if self.socket:
            try:
                self.socket.close()
            except:
                pass
