import socket
import sys
import traceback
from threading import Thread
from esp_log_parser import ESPLogParser
import queue
import logging
from logging.handlers import QueueHandler
from influxdb_client import InfluxExportServer, InfluxLogHandler

HOST = '192.168.1.252'  # Standard loopback interface address (localhost)
PORT = 1234        # Port to listen on (non-privileged ports are > 1023)

def setup_logger(log_queue):
   logging.basicConfig( level=logging.INFO)
   queue_handler = InfluxLogHandler(log_queue)
   root = logging.getLogger()
   root.addHandler(queue_handler)   

def main():
   
   log_queue = queue.Queue()
   setup_logger(log_queue)
   influx = InfluxExportServer(log_queue)
   influx.run()
   start_server(log_queue)

def start_server(log_queue):
   host = HOST
   port = PORT # arbitrary non-privileged port


   soc = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
   soc.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
   logging.info("Socket created")
   
   try:
      soc.bind((host, port))
   except:
      print("Bind failed. Error : " + str(sys.exc_info()))
      sys.exit()

   soc.listen(6) # queue up to 6 requests
   logging.info("Socket now listening")

   # infinite loop- do not reset for every requests
   while True:
      connection, address = soc.accept()
      ip, port = str(address[0]), str(address[1])
      logging.info("Connected with {0}:{1}".format(ip, port))
      try:
         Thread(target=client_thread, args=(connection, ip, port)).start()
         logging.info("thread started")
      except:
         print("Thread did not start.")
         traceback.print_exc()
   soc.close()

def client_thread(connection, ip, port, max_buffer_size = 5120):
   is_active = True
   while is_active:
      data = connection.recv(1024)
      ESPLogParser().parse(data)

if __name__ == "__main__":
   main()