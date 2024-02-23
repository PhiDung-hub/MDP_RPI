import time
import logging
from multiprocessing import Process, Manager
from communication.stm32 import STMLink
import sys
import json
import os
import requests
from consts import SYMBOL_MAP
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

        self.rpi_action_queue = self.manager.Queue()
        # Messages that need to be processed by STM32, as well as snap commands
        self.command_queue = self.manager.Queue()
        # X,Y,D coordinates of the robot after execution of a command
        self.path_queue = self.manager.Queue()

        self.proc_recv_stm32 = None
        self.rs_flag = False
        self.success_obstacles = self.manager.list()
        self.failed_obstacles = self.manager.list()
        self.obstacles = self.manager.dict()

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
            message: str = self.stm_link.recv() or ""
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


        awbs = ['off', 'auto', 'incandescent', 'tungsten',
                'fluorescent', 'indoor', 'daylight', 'cloudy']

        awb = 2
        retry_count = 0

        while True:
            retry_count += 1

            rpistr = "libcamera-still -e jpg -n -t 500 -o " + filename
            rpistr += " --awb " + awbs[awb]
            # rpistr += " --metadata - --metadata-format txt >> PiLibtext.txt"

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
            else:
                self.logger.info(f"Image recognition results: {results}")
                self.logger.info("Retrying ...")

        # release lock so that bot can continue moving
        self.movement_lock.value == 0

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

if __name__ == "__main__":
    t = float(sys.argv[1])
    rpi = RaspberryPi()
    rpi.start(t)

