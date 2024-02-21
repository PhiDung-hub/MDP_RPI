import time
import logging
from multiprocessing import Process, Manager
from communication.stm32 import STMLink
import sys
import json
import queue
from typing import Optional
import os
import requests
from consts import SYMBOL_MAP
from logger import prepare_logger
from settings import API_IP, API_PORT

class RaspberryPi:
    """
    Class that represents the Raspberry Pi.
    """

    def __init__(self):
        """
        Initializes the Raspberry Pi.
        """
        self.stm_link = STMLink()
        self.manager = Manager()
        self.movement_lock = self.manager.Value('i', 0)  # Using Value as a flag
        self.current_location = self.manager.dict()

        # Initialize logger
        self.logger = logging.getLogger(__name__)
        self.logger.setLevel(logging.DEBUG)
        formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
        ch = logging.StreamHandler()
        ch.setFormatter(formatter)
        self.logger.addHandler(ch)

    def start(self, t):
        """Starts the RPi orchestrator"""
        try:
            self.stm_link.connect()
            self.proc_recv_stm32 = Process(target=self.recv_stm)
            self.proc_recv_stm32.start()
            self.logger.info("Child Process started")

            ### Start up complete ###
            self.move_forward(t)

        except KeyboardInterrupt:
            self.stop()

    def stop(self):
        """Stops all processes on the RPi and disconnects gracefully with STM32"""
        self.stm_link.disconnect()

    def recv_stm(self) -> None:
        """
        [Child Process] Receive acknowledgement messages from STM32, and release the movement lock
        """
        while True:
            message: str = self.stm_link.recv()
            print(message)
            if message.startswith("ACK"):
                self.movement_lock.value = 0  # Release the lock
                self.logger.debug("ACK from STM32 received, movement lock released.")
            else:
                self.logger.warning(f"Ignored unknown message from STM: {message}")

    def move_forward(self, t):
        """
        Moves the robot forward by sending commands to the STM32
        """
        # Acquire movement lock before sending command
        while True:
            if self.movement_lock.value == 0:  # Check if lock is released
                self.movement_lock.value = 1  # Acquire the lock
                break
            time.sleep(0.1)  # Wait for a short while before checking again
        # Send forward command to STM32
        time.sleep(1)
        self.stm_link.send("FL3xx")
        time.sleep(t)
        self.stm_link.send("SWxxx")
        time.sleep(0.1)
        self.stm_link.send("SSSSS")
        # Wait for acknowledgement from STM32
        while self.movement_lock.value == 1:  # Wait until the lock is released
            time.sleep(0.1)
        # After receiving acknowledgement, update location
        self.current_location['x'] += 1  # Assuming x-coordinate increment by 1 for simplicity
        self.logger.info(f"Robot moved forward. New location: {self.current_location}")

    def snap_and_rec(self, obstacle_id_with_signal: str) -> None:
        """
        RPi snaps an image and calls the API for image-rec.
        The response is then forwarded back to the android
        :param obstacle_id_with_signal: the current obstacle ID followed by underscore followed by signal
        """
        obstacle_id, signal = obstacle_id_with_signal.split("_")
        self.logger.info(f"Capturing image for obstacle id: {obstacle_id}")
        url = f"http://{API_IP}:{API_PORT}/image"
        filename = f"{int(time.time())}_{obstacle_id}_{signal}.jpg"

        con_file = "PiLCConfig9.txt"
        Home_Files = []
        Home_Files.append(os.getlogin())
        config_file = "/home/" + Home_Files[0] + "/" + con_file

        extns = ['jpg', 'png', 'bmp', 'rgb', 'yuv420', 'raw']
        shutters = [-2000, -1600, -1250, -1000, -800, -640, -500, -400, -320, -288, -250, -240, -200, -160, -144, -125, -120, -100, -96, -80, -60, -50, -48, -40, -30, -25, -20, -
                    15, -13, -10, -8, -6, -5, -4, -3, 0.4, 0.5, 0.6, 0.8, 1, 1.1, 1.2, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 15, 20, 25, 30, 40, 50, 60, 75, 100, 112, 120, 150, 200, 220, 230, 239, 435]
        meters = ['centre', 'spot', 'average']
        awbs = ['off', 'auto', 'incandescent', 'tungsten',
                'fluorescent', 'indoor', 'daylight', 'cloudy']
        denoises = ['off', 'cdn_off', 'cdn_fast', 'cdn_hq']

        config = []
        with open(config_file, "r") as file:
            line = file.readline()
            while line:
                config.append(line.strip())
                line = file.readline()
            config = list(map(int, config))
        mode = config[0]
        speed = config[1]
        gain = config[2]
        brightness = config[3]
        contrast = config[4]
        red = config[6]
        blue = config[7]
        ev = config[8]
        extn = config[15]
        saturation = config[19]
        meter = config[20]
        awb = config[21]
        sharpness = config[22]
        denoise = config[23]
        quality = config[24]

        retry_count = 0

        while True:

            retry_count += 1

            shutter = shutters[speed]
            if shutter < 0:
                shutter = abs(1/shutter)
            sspeed = int(shutter * 1000000)
            if (shutter * 1000000) - int(shutter * 1000000) > 0.5:
                sspeed += 1

            rpistr = "libcamera-still -e " + \
                extns[extn] + " -n -t 500 -o " + filename
            rpistr += " --brightness " + \
                str(brightness/100) + " --contrast " + str(contrast/100)
            rpistr += " --shutter " + str(sspeed)
            if ev != 0:
                rpistr += " --ev " + str(ev)
            if sspeed > 1000000 and mode == 0:
                rpistr += " --gain " + str(gain) + " --immediate "
            else:
                rpistr += " --gain " + str(gain)
                if awb == 0:
                    rpistr += " --awbgains " + str(red/10) + "," + str(blue/10)
                else:
                    rpistr += " --awb " + awbs[awb]
            rpistr += " --metering " + meters[meter]
            rpistr += " --saturation " + str(saturation/10)
            rpistr += " --sharpness " + str(sharpness/10)
            rpistr += " --quality " + str(quality)
            rpistr += " --denoise " + denoises[denoise]
            rpistr += " --metadata - --metadata-format txt >> PiLibtext.txt"

            os.system(rpistr)

            self.logger.debug("Requesting from image API")

            response = requests.post(
                url, files={"file": (filename, open(filename, 'rb'))})

            if response.status_code != 200:
                self.logger.error(
                    "Something went wrong when requesting path from image-rec API. Please try again.")
                return

            results = json.loads(response.content)

            # Higher brightness retry

            if results['image_id'] != 'NA' or retry_count > 6:
                break
            elif retry_count > 3:
                self.logger.info(f"Image recognition results: {results}")
                self.logger.info("Recapturing with lower shutter speed...")
                speed -= 1
            elif retry_count <= 3:
                self.logger.info(f"Image recognition results: {results}")
                self.logger.info("Recapturing with higher shutter speed...")
                speed += 1

        # release lock so that bot can continue moving
        self.movement_lock.release()
        try:
            self.retrylock.release()
        except:
            pass

        self.logger.info(f"results: {results}")
        self.logger.info(f"self.obstacles: {self.obstacles}")
        self.logger.info(
            f"Image recognition results: {results} ({SYMBOL_MAP.get(results['image_id'])})")

        if results['image_id'] == 'NA':
            self.failed_obstacles.append(
                self.obstacles[int(results['obstacle_id'])])
            self.logger.info(
                f"Added Obstacle {results['obstacle_id']} to failed obstacles.")
            self.logger.info(f"self.failed_obstacles: {self.failed_obstacles}")
        else:
            self.success_obstacles.append(
                self.obstacles[int(results['obstacle_id'])])
            self.logger.info(
                f"self.success_obstacles: {self.success_obstacles}")
        self.android_queue.put(AndroidMessage("image-rec", results))
if __name__ == "__main__":
    t = float(sys.argv[1])
    rpi = RaspberryPi()
    rpi.start(t)

