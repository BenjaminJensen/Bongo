from influxdb import InfluxDBClient
import time
import threading
import logging
from logging.handlers import QueueHandler

class InfluxExporter:
    severity_codes = {'error': 3, 'warning': 4, 'notice': 5, 'info': 6, 'debug': 7}
    def connect(self, host, port, user, password, db):
        self._client = InfluxDBClient(host, port, user, password, db) #('192.168.1.70', 8086, 'python', 'password123', 'syslog')

    def export(self, body, protocol="line"):
        if self._client:
            self._client.write_points(body, protocol=protocol)
        else:
            raise ConnectionError('InfluxDB client not connected')

    def gen_line(self, appname, host, hostname, severity, process, msg):
        # facility & facility_code will default to 14 {console}, for others see https://en.wikipedia.org/wiki/Syslog 
        line = 'syslog,appname={0},facility=console,host={1},hostname={2},severity={3} ' \
            'facility_code=14i,message=\"{4}\",severity_code={5}i,procid=\"{6}\",timestamp={7} {7}\n' \
            ''.format(appname, host, hostname, severity, msg, self.severity_codes[severity], process, time.time_ns())

        return line

class InfluxExportServer:
    def __init__(self, queue):
        self.q = queue

    def run(self):
        t = threading.Thread(target=self.worker)
        t.start()
        logging.info('InfluxExportServer started')
    
    def worker(self):
        influx = InfluxExporter()
        influx.connect('192.168.1.70', 8086, 'python', 'password123', 'esp_logs')

        while True:
            item = self.q.get()
            print(item)
            influx.export(item)
            #print('<{0}> From queue: {1}'.format(type(item),item))

class InfluxLogHandler(QueueHandler):
    def enqueue(self, record):
        line = InfluxExporter().gen_line(record.module, 'python', 'python', record.levelname.lower(), record.funcName, record.message)
        self.queue.put(line)

def main():
 
    #json_body = "syslog,appname=myapp,facility=console,host=myhost,hostname=myhost,severity=warning facility_code=14i,message=\"another warning message here\",severity_code=4i,procid=\"12345\",timestamp={0} {0} \n".format(time.time_ns())
  
    influx = InfluxExporter()
    influx.connect('192.168.1.70', 8086, 'python', 'password123', 'esp_logs')

    l = influx.gen_line('influxExporter', 'localhost', 'localhost', 'warning', 'export', 'Test line')
    #print(l)
    #print(json_body)
    influx.export(l)

if __name__ == "__main__":
   main()