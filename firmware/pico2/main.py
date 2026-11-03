import rp2
from machine import Pin
import sys
import uselect
import time

# --- PIO Quadrature Decoder (A = in_base, B = jmp_pin) ---
# Standard RP2350/RP2040 PIO assembly using two's-complement increment
@rp2.asm_pio()
def quadrature():
    label("start")
    wait(0, pin, 0)         # Ensure Channel A is low before waiting for rising edge
    wait(1, pin, 0)         # Wait for Channel A rising edge
    jmp(pin, "decrement")   # Check Channel B (jmp_pin): High -> decrement, Low -> increment

    # Increment X: x = ~(~x - 1)
    mov(x, invert(x))
    jmp(x_dec, "inc_done")
    label("inc_done")
    mov(x, invert(x))
    jmp("start")

    # Decrement X: x = x - 1
    label("decrement")
    jmp(x_dec, "start")
    jmp("start")

# --- Hardware Setup ---
# X Axis: A = GPIO 2, B = GPIO 3
Pin(2, Pin.IN, Pin.PULL_UP)
Pin(3, Pin.IN, Pin.PULL_UP)
sm_x = rp2.StateMachine(0, quadrature, in_base=Pin(2), jmp_pin=Pin(3))

# Y Axis: A = GPIO 6, B = GPIO 7
Pin(6, Pin.IN, Pin.PULL_UP)
Pin(7, Pin.IN, Pin.PULL_UP)
sm_y = rp2.StateMachine(1, quadrature, in_base=Pin(6), jmp_pin=Pin(7))

# Z Axis: A = GPIO 10, B = GPIO 11
Pin(10, Pin.IN, Pin.PULL_UP)
Pin(11, Pin.IN, Pin.PULL_UP)
sm_z = rp2.StateMachine(2, quadrature, in_base=Pin(10), jmp_pin=Pin(11))

# Renishaw TP-ES Probe on GPIO 14 (NC to GND -> opens/HIGH on trigger)
probe = Pin(14, Pin.IN, Pin.PULL_UP)

for sm in (sm_x, sm_y, sm_z):
    sm.exec("set(x, 0)")
    sm.active(1)

def _to_signed32(val):
    return val - 0x100000000 if (val & 0x80000000) else val

def read_raw():
    for sm in (sm_x, sm_y, sm_z):
        sm.exec("in_(x, 32)")
        sm.exec("push()")
    rx = _to_signed32(sm_x.get())
    ry = _to_signed32(sm_y.get())
    rz = _to_signed32(sm_z.get())
    return rx, ry, rz

def zero_pio_counters():
    for sm in (sm_x, sm_y, sm_z):
        sm.exec("set(x, 0)")

# --- Interrupt-Safe Probe Hit Flag & Debounce ---
hit_pending = False
hit_counter = 0
last_hit_ticks = 0
DEBOUNCE_MS = 60

def probe_hit_handler(pin):
    global hit_pending, last_hit_ticks
    now = time.ticks_ms()
    if time.ticks_diff(now, last_hit_ticks) >= DEBOUNCE_MS:
        if pin.value() == 1:  # Confirm rising edge (probe open/triggered)
            hit_pending = True
            last_hit_ticks = now

probe.irq(trigger=Pin.IRQ_RISING, handler=probe_hit_handler)

# Non-blocking USB CDC command listener
poll_stdin = uselect.poll()
poll_stdin.register(sys.stdin, uselect.POLLIN)

stream_dro = True
last_stream_time = time.ticks_ms()

while True:
    now = time.ticks_ms()

    # 1. Immediately service hardware probe trigger on main thread (safe PIO FIFO access)
    if hit_pending:
        hit_pending = False
        rx, ry, rz = read_raw()
        hit_counter += 1
        sys.stdout.write(
            f'{{"type":"hit","event":"hit","id":{hit_counter},"x":{rx},"y":{ry},"z":{rz},"probe":1}}\n'
        )

    # 2. Check for incoming commands from OpenCMM
    if poll_stdin.poll(0):
        cmd = sys.stdin.readline().strip().upper()
        if cmd == "PING":
            sys.stdout.write('{"type":"pong"}\n')
        elif cmd == "ZERO" or cmd == "Z":
            zero_pio_counters()
            rx, ry, rz = read_raw()
            sys.stdout.write(f'{{"type":"dro","x":{rx},"y":{ry},"z":{rz},"probe":0}}\n')
        elif cmd == "STREAM_ON":
            stream_dro = True
        elif cmd == "STREAM_OFF":
            stream_dro = False

    # 3. Emit DRO update at ~60Hz (every 16ms)
    if stream_dro and time.ticks_diff(now, last_stream_time) >= 16:
        rx, ry, rz = read_raw()
        p_state = probe.value()
        sys.stdout.write(f'{{"type":"dro","x":{rx},"y":{ry},"z":{rz},"probe":{p_state}}}\n')
        last_stream_time = now

    time.sleep_ms(1)