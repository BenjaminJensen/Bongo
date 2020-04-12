import math

class ESPLogParser:

    def parse(self, data):
        for l in self.tokenize(data):
            print(self.decode(l))

    def tokenize(self, data):
        _end = b'\x1b[0m\n'
        cnt = 0
        start = 0
        tokens = []
        while (cnt + 5) <= len(data):
            if data[cnt:(cnt + 5)] == _end:
                tokens.append(data[start:cnt])
                start = cnt + 5
            cnt += 1
        
        return tokens

    def decode(self, l):

        log = {}
        if l[0:4] == b'\x1b[0;':
            log['color'] = self.decode_color(l)
            log['severity'] = self.decode_severity(l)
            spaces = self.find_spaces(l)
            log['time'] = self.decode_timestamp(l, spaces )
            log['msg'] = self.decode_message(l, spaces[2])

        else:
            print('Not a valid line!: "{0}"'.format(l))

        return log

    def find_spaces(self, l):
        #find spaces in line
        cnt = 0
        spaces = []
        for b in l:
            if b == 32: # Space
                spaces.append(cnt)
            cnt += 1
        return spaces

    def decode_color(self, l):
        return int(l[4:6].decode("utf-8"))
    
    def decode_severity(self, l):
        severity = l[7:8].decode("utf-8")
        if severity not in ['E', 'W', 'D', 'I', 'V']:
            raise ValueError('Level "{0}" not supported'.format(severity))
        
        return severity

    def decode_timestamp(self, l, spaces):
        t_start = spaces[0] + 2 # remove " ("
        t_end = spaces[1] - 1 # remove ")"
        t_ms = int( l[t_start:t_end] ) 
        # ---------------------------------
        # Time
        h = math.floor(t_ms / 60 / 60 / 1000)

        # left after hours substracted
        hl = t_ms - h * 60 * 60 * 1000

        m = math.floor(hl / 60 / 1000)

        # left after minutes substracted
        ml = hl - m * 60 * 1000

        s = math.floor(ml / 1000)

        ms = ml - s * 1000

        return '{0}:{1}:{2} {3}'.format(h,m,s,ms)

    def decode_message(self, l, start):
        msg_start = start + 1 # remove = " "

        # find msg end

        end_tag = b'\x1b[0m'

        cnt_s = start # Start after module TAG
        while (cnt_s + 4) <= len(l):
            if l[cnt_s: cnt_s +4] == end_tag:
                break
            cnt_s += 1

        msg_end = cnt_s + 3
        msg = l[msg_start:msg_end].decode("utf-8")

        return msg

def main():
    data = b'\x1b[0;33mW (15337) UART: Data read from Rotor failed. 0 of 20 bytes read\x1b[0m\n\x1b[0;33mW (16337) UART: Data read from Rotor failed. 0 of 20 bytes read\x1b[0m\n\x1b[0;33mW (17337) UART: Data read from Rotor failed. 0 of 20 bytes read\x1b[0m\n\x1b[0;33mW (18337) UART: Data read from Rotor failed. 0 of 20 bytes read\x1b[0m\n\x1b[0;33mW (19337) UART: Data read from Rotor failed. 0 of 20 bytes read\x1b[0m\n\x1b[0;33mW (20337) UART: Data read from Rotor failed. 0 of 20 bytes read\x1b[0m\n\x1b[0;33mW (21337) UART: Data read from Rotor failed. 0 of 20 bytes read\x1b[0m\n\x1b[0;33mW (22337) UART: Data read from Rotor failed. 0 of 20 bytes read\x1b[0m\n\x1b[0;33mW (23337) UART: Data read from Rotor failed. 0 of 20 bytes read\x1b[0m\n'
    ESPLogParser().parse(data)

if __name__ == "__main__":
   main()