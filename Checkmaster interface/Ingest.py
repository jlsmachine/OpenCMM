import serial
import json
import threading

class OpenCMMInterface:
    def __init__(self, port="COM3", baudrate=115200, scale_res_in=0.00001968503937):
        self.port = port
        self.baudrate = baudrate
        self.scale_factor = scale_res_in
        self.ser = None
        self.is_running = False
        self.thread = None
        
        # Callbacks to update OpenCMM UI and measurement engines
        self.on_dro_update = None    # fn(x_in, y_in, z_in)
        self.on_probe_hit = None     # fn(hit_id, x_in, y_in, z_in)

    def connect(self):
        self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
        self.is_running = True
        self.thread = threading.Thread(target=self._listen, daemon=True)
        self.thread.start()
        print(f"OpenCMM connected to Pico 2 on {self.port}")

    def _listen(self):
        while self.is_running:
            try:
                line = self.ser.readline().decode('utf-8').strip()
                if not line:
                    continue
                data = json.loads(line)
                
                msg_type = data.get("type")
                x_in = data["x"] * self.scale_factor
                y_in = data["y"] * self.scale_factor
                z_in = data["z"] * self.scale_factor

                if msg_type == "dro" and self.on_dro_update:
                    self.on_dro_update(x_in, y_in, z_in)
                    
                elif msg_type == "hit" and self.on_probe_hit:
                    self.on_probe_hit(data["id"], x_in, y_in, z_in)
                    
            except (json.JSONDecodeError, UnicodeDecodeError):
                continue
            except Exception as e:
                print(f"Serial communication error: {e}")
                break

    def disconnect(self):
        self.is_running = False
        if self.ser and self.ser.is_open:
            self.ser.close()

# --- Verification & Usage Example inside OpenCMM ---
if __name__ == "__main__":
    cmm = OpenCMMInterface(port="COM3")

    def display_dro(x, y, z):
        print(f"\rDRO -> X: {x:9.5f}\" | Y: {y:9.5f}\" | Z: {z:9.5f}\"", end="")

    def handle_hit(hit_id, x, y, z):
        print(f"\n>>> OPEN_CMM POINT RECORDED #{hit_id}: ({x:.5f}, {y:.5f}, {z:.5f}) <<<")

    cmm.on_dro_update = display_dro
    cmm.on_probe_hit = handle_hit
    cmm.connect()

    try:
        while True:
            pass
    except KeyboardInterrupt:
        cmm.disconnect()