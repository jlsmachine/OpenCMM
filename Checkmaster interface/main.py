import rp2
from machine import Pin, UART
import sys
import uselect
import time

# --- PIO Quadrature Decoder ---
@rp2.asm_pio(fifo_join=rp2.PIO.JOIN_RX)
def quadrature():
    label("start")
    wait(1, pin, 0)
    jmp(pin, "decrement")
    label("increment")
    mov(x, x + 1)
    wait(0, pin, 0)
    jmp("start")
    label("decrement")
    mov(x, x - 1)
    wait(0, pin, 0)
    jmp("start")

# --- Hardware Setup ---
sm_x = rp2.StateMachine(0, quadrature, in_base=Pin(2))
sm_y = rp2.StateMachine(1, quadrature, in_base=Pin(6))
sm_z = rp2.StateMachine(2, quadrature, in_base=Pin(10))

probe = Pin(14, Pin.IN, Pin.PULL_UP)

for sm in [sm_x, sm_y, sm_z]:
    sm.active(1)

hit_counter = 0
stream_dro = True
last_stream_time = 0

def read_raw():
    sm_x.put(0); rx = sm_x.get()
    sm_y.put(0); ry = sm_y.get()
    sm_z.put(0); rz = sm_z.get()
    return rx, ry, rz

def probe_hit_handler(pin):
    global hit_counter
    # Instant latch on trigger
    rx, ry, rz = read_raw()
    hit_counter += 1
    # Send hit instantly out of band
    sys.stdout.write(f'{{"type":"hit","id":{hit_counter},"x":{rx},"y":{ry},"z":{rz}}}\n')

# Renishaw TP-ES breaks contact on touch (NC switch -> Rising Edge to 3.3/5V pull-up)
probe.irq(trigger=Pin.IRQ_RISING, handler=probe_hit_handler)

# Non-blocking USB input listener
poll_stdin = uselect.poll()
poll_stdin.register(sys.stdin, uselect.POLLIN)

while True:
    now = time.ticks_ms()
    
    # 1. Check for incoming commands from OpenCMM
    if poll_stdin.poll(0):
        cmd = sys.stdin.readline().strip()
        if cmd == "PING":
            sys.stdout.write('{"type":"pong"}\n')
        elif cmd == "STREAM_ON":
            stream_dro = True
        elif cmd == "STREAM_OFF":
            stream_dro = False

    # 2. Emit DRO update at ~30Hz (every 33ms)
    if stream_dro and time.ticks_diff(now, last_stream_time) >= 33:
        rx, ry, rz = read_raw()
        sys.stdout.write(f'{{"type":"dro","x":{rx},"y":{ry},"z":{rz}}}\n')
        last_stream_time = now
        
    time.sleep_ms(1)