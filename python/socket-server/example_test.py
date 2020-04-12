# This example code is in the Public Domain (or CC0 licensed, at your option.)

# Unless required by applicable law or agreed to in writing, this
# software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
# CONDITIONS OF ANY KIND, either express or implied.

# -*- coding: utf-8 -*-

from __future__ import print_function
from __future__ import unicode_literals
from builtins import input
import os
import re
import sys
import netifaces
import socket
from threading import Thread, Event


# -----------  Config  ----------
PORT = 1234

FAMILY = socket.AF_INET
# -------------------------------


class TcpServer:

    def __init__(self, port, family_addr, persist=False):
        self.port = port
        self.socket = socket.socket(family_addr, socket.SOCK_STREAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.settimeout(20.0)
        self.shutdown = Event()
        self.persist = persist
        self.family_addr = family_addr

    def __enter__(self):
        try:
            self.socket.bind(('', self.port))
        except socket.error as e:
            print("Bind failed:{}".format(e))
            raise
        self.socket.listen(1)

        self.server_thread = Thread(target=self.run_server)
        self.server_thread.start()
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        if self.persist:
            sock = socket.socket(self.family_addr, socket.SOCK_STREAM)
            sock.connect(('localhost', self.port))
            sock.sendall(b'Stop', )
            sock.close()
            self.shutdown.set()
        self.shutdown.set()
        self.server_thread.join()
        self.socket.close()

    def run_server(self):
        while not self.shutdown.is_set():
            try:
                conn, address = self.socket.accept()  # accept new connection
                print("Connection from: {}".format(address))
                conn.setblocking(1)
                data = conn.recv(1024)
                if not data:
                    return
                data = data.decode()
                print('Received data: ' + data)
                reply = 'OK: ' + data
                conn.send(reply.encode())
                conn.close()
            except socket.error as e:
                print("Running server failed:{}".format(e))
                raise
            if not self.persist:
                break

def main():
    t = TcpServer(PORT,FAMILY )
    t.run_server()

if __name__ == "__main__":
    main()