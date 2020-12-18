import socket
import sys
import threading


class TCP_Interface:
    def __init__(self, ip_address = 'localhost', port_number = 65123):
        self.receive_callback = None
        # Create a TCP/IP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        # Connect the socket to the port where the server is listening
        server_address = (ip_address, port_number)
        print('connecting to %s port %s' % server_address)
        self.sock.connect(server_address)
        print("connected successfully")

        # Look for the response
        read_thread = threading.Thread(target=self.reader)
        read_thread.start()

    def bindDataCallback(self, cb):
        self.receive_callback = cb

    def write(self, b):
        self.sock.sendall(b)

    def reader(self):
        while True:
            data = self.sock.recv(1)
            if self.receive_callback:
                self.receive_callback(data)

    def writer(self):
        import time
        while True:
            time.sleep(1)
            message = 'This is the message.  It will be repeated.'
            print('sending "%s"' % message)
            data = bytes(message, 'utf-8')
            self.sock.sendall(data)
    
