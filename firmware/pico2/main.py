# Copyright (C) 2026 JLS Machine LLC
# SPDX-License-Identifier: GPL-3.0-or-later
import rp2
from machine import Pin
import sys
import uselect
import time

# --- PIO Quadrature Decoder ---
@rp2.asm_pio()
def quadrature():
    label("start")
    wait(0, pin, 0)
    wait(1, pin, 0)
    jmp(pin, "decrement")
    mov(x, invert(x))
    jmp(x_dec, "inc_done")
    label("inc_done")
    mov(x, invert(x))
    jmp("start")
    label("decrement")
    jmp(x_dec, "start")
    jmp("start")

# X (Pins 2,3), Y (Pins 6,7), Z (Pins 10,11)
for a, b in ((2, 3), (6, 7), (10, 11)):
    Pin(a, Pin.IN, Pin.PULL_UP)
    Pin(b, Pin.IN, Pin.PULL_UP)

sm_x = rp2.StateMachine(0, quadrature, in_base=Pin(2), jmp_pin=Pin(3))
sm_y = rp2.StateMachine(1, quadrature, in_base=Pin(6), jmp_pin=Pin(7))
sm_z = rp2.StateMachine(2, quadrature, in_base=Pin(10), jmp_pin=Pin(11))

for sm in (sm_x, sm_y, sm_z):
    sm.exec("set(x, 0)")
    sm.active(1)

probe = Pin(14, Pin.IN, Pin.PULL_UP)

def _to_signed32(val):
    return val - 0x100000000 if (val & 0x80000000) else val

def read_raw():
    for sm in (sm_x, sm_y, sm_z):
        sm.exec("in_(x, 32)")
        sm.exec("push()")
    return _to_signed32(sm_x.get()), _to_signed32(sm_y.get()), _to_signed32(sm_z.get())

hit_pending = False
hit_counter = 0
last_hit_ticks = 0

def probe_hit_handler(pin):
    global hit_pending, last_hit_ticks
    now = time.ticks_ms()
    if time.ticks_diff(now, last_hit_ticks) >= 60:
        if pin.value() == 1:
            hit_pending = True
            last_hit_ticks = now

probe.irq(trigger=Pin.IRQ_RISING, handler=probe_hit_handler)

poll_obj = uselect.poll()
poll_obj.register(sys.stdin, uselect.POLLIN)

last_stream_time = time.ticks_ms()
cmd_buffer = ""

while True:
    now = time.ticks_ms()

    # 1. Fully non-blocking USB command poll
    if poll_obj.poll(0):
        ch = sys.stdin.read(1)
        if ch:
            if ch == '\n' or ch == '\r':
                cmd = cmd_buffer.strip().upper()
                cmd_buffer = ""
                if cmd == "PING":
                    sys.stdout.write('{"type":"pong"}\n')
                elif cmd == "ZERO" or cmd == "Z":
                    for sm in (sm_x, sm_y, sm_z):
                        sm.exec("set(x, 0)")
            else:
                cmd_buffer += ch

    # 2. Service probe hit instantly
    if hit_pending:
        hit_pending = False
        rx, ry, rz = read_raw()
        hit_counter += 1
        sys.stdout.write(f'{{"type":"hit","event":"hit","id":{hit_counter},"x":{rx},"y":{ry},"z":{rz},"probe":1}}\n')

    # 3. Autonomous 60Hz DRO Stream
    if time.ticks_diff(now, last_stream_time) >= 16:
        rx, ry, rz = read_raw()
        p_val = probe.value()
        sys.stdout.write(f'{{"type":"dro","x":{rx},"y":{ry},"z":{rz},"probe":{p_val}}}\n')
        last_stream_time = now

    time.sleep_ms(1)