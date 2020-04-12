import math

l = b'\x1b[0;32mI (8167207) GENVEX_MQTT: {"t": 25.55, "h": 28.11, "p": 1032.39}\x1b[0m\n'

level = l[7:8].decode("utf-8")
cnt = 0

spaces = []
for b in l:
    if b == 32: # Space
        spaces.append(cnt)
    cnt += 1

# ---------------------------------
# Time
t_start = spaces[0] + 2 # remove " ("
t_end = spaces[1] - 1 # remove ")"
t_ms = int(l[t_start:t_end])

h = math.floor(t_ms / 60 / 60 / 1000)

# left after hours substracted
hl = t_ms - h * 60 * 60 * 1000

m = math.floor(hl / 60 / 1000)

# left after minutes substracted
ml = hl - m * 60 * 1000

s = math.floor(ml / 1000)

ms = ml - s * 1000
#--------------------------------
# Module
m_start = spaces[1] + 1 # remove " "
m_stop = spaces[2] - 1 # remove ":"

module = l[m_start:m_stop].decode("utf-8")

#--------------------------------
# msg

msg_start = spaces[2] + 1 # remove = " "

# find msg end

end_tag = b'\x1b[0m'

cnt_s = spaces[2] # Start after module TAG
while (cnt_s + 4) <= len(l):
    if l[cnt_s: cnt_s +4] == end_tag:
        print('Found {0}'.format(cnt_s))
        break
    cnt_s += 1


msg_end = cnt_s
msg = l[msg_start:msg_end].decode("utf-8")

print('{0} {1}:{2}:{3} {4} | {5}: "{6}"'.format(level,h,m,s,ms, module, msg))