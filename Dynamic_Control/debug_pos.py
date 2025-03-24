from zed_interface import ZEDWrapper
import time

zed = ZEDWrapper()

try:
    while True:
        vitesse = zed.get_velocity()
        print(f"Vitesse: {vitesse:.3f} m/s")
        time.sleep(0.1)
except KeyboardInterrupt:
    zed.close()
