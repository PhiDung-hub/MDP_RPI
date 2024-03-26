# _!/venv/bin/python

import json
import queue
from multiprocessing import Process, Manager, Semaphore
import time
from typing import Optional
import os
import requests
from communication.android import AndroidLink, AndroidMessage
from communication.stm32 import STMLink
from consts import SYMBOL_MAP
from logger import prepare_logger
from settings import API_IP, API_PORT


class PiAction:
    """
    Class that represents an action that the RPi needs to take.
    """

    def __init__(self, cat, value):
        """
        :param cat: The category of the action. Can be 'info', 'mode', 'path', 'snap', 'obstacle', 'location', 'failed', 'success'
        :param value: The value of the action. Can be a string, a list of coordinates, or a list of obstacles.
        """
        self._cat = cat
        self._value = value

    @property
    def cat(self):
        return self._cat

    @property
    def value(self):
        return self._value


class RaspberryPi:
    """
    Class that represents the Raspberry Pi.
    """

    def __init__(self):
        """
        Initializes the Raspberry Pi.
        """
        self.logger = prepare_logger()
        # self.android_link = AndroidLink()
        self.stm_link = STMLink()
        self.android_link = AndroidLink()
        self.manager = Manager()

        self.lock_stm = Semaphore(1)
        self.android_dropped = self.manager.Event()
        self.start_movement = self.manager.Event()

        self.movement_lock = self.manager.Lock()

        # Messages to send to Android
        self.android_queue = self.manager.Queue()
        # Messages that need to be processed by RPi
        self.action_queue = self.manager.Queue()
        # Messages that need to be processed by STM32, as well as snap commands
        self.command_queue = self.manager.Queue()
        self.cached_command_queue = self.manager.Queue()
        # X,Y,D coordinates of the robot after execution of a command
        self.path_queue = self.manager.Queue()

        self.proc_recv_android = None
        self.proc_recv_stm32 = None
        self.proc_android_sender = None
        self.proc_rpi_action = None
        self.proc_command_follower = None

        self.success_obstacles = self.manager.list()
        self.obstacles = self.manager.dict()

    def start(self):
        """Starts the RPi orchestrator"""
        try:
            ### Start up initialization ###

            self.android_queue.put(AndroidMessage("info", "Connected to the RPi!"))
            self.stm_link.connect()
            self.check_api()

            # Define child processes
            self.proc_command_follower = Process(target=self.command_follower)
            self.proc_rpi_action = Process(target=self.rpi_action)
            self.proc_recv_stm32 = Process(target=self.recv_stm)

            # Start child processes
            self.proc_command_follower.start()
            self.proc_rpi_action.start()
            self.proc_recv_stm32.start()

            self.logger.info("Child Processes spawned")

            self.android_link.connect()
            self.proc_android_sender = Process(target=self.android_sender)
            self.proc_android_sender.start()
            self.proc_recv_android = Process(target=self.recv_android)
            self.proc_recv_android.start()

            ### Start up complete ###

            # Send success message to Android
            self.android_queue.put(AndroidMessage("info", "Robot is ready!"))
            self.reconnect_android()

        except KeyboardInterrupt:
            self.stop()

    def reconnect_android(self):
        """Handles the reconnection to Android in the event of a lost connection."""
        self.logger.info("Reconnection handler is watching...")

        while True:
            self.android_dropped.wait()
            # Wait for android connection to drop

            self.logger.error("Android link is down!")

            # Kill child processes
            self.logger.debug("Killing android child processes")
            if self.proc_android_sender:
                self.proc_android_sender.kill()
            if self.proc_android_sender:
                self.proc_android_sender.join()

            if self.proc_recv_android:
                self.proc_recv_android.kill()
            if self.proc_recv_android:
                self.proc_recv_android.join()
            self.logger.debug("Android child processes killed")

            # Clean up old sockets
            self.android_link.disconnect()
            self.android_link.connect()

            # Recreate Android processes
            self.proc_recv_android = Process(target=self.recv_android)
            self.proc_android_sender = Process(target=self.android_sender)

            # Start previously killed processes
            self.proc_recv_android.start()
            self.proc_android_sender.start()

            self.android_queue.put(AndroidMessage("info", "You are reconnected!"))
            self.android_dropped.clear()

    def android_sender(self) -> None:
        """
        [Child process] Responsible for retrieving messages from android_queue and sending them over the Android link.
        """
        while True:
            # Retrieve from queue
            try:
                message: AndroidMessage = self.android_queue.get(timeout=0.5)
            except queue.Empty:
                continue
            try:
                self.android_link.send(message)
            except OSError:
                self.android_dropped.set()
                self.logger.debug("Error Event triggered: Android dropped")

    def recv_android(self) -> None:
        """
        [Child Process] Processes the messages received from Android
        """
        while True:
            msg_str: Optional[str] = None
            try:
                msg_str = self.android_link.recv()
            except OSError:
                self.android_dropped.set()
                self.logger.debug("Event set: Android connection dropped")

            if msg_str is None:
                continue

            try:
                message: dict = json.loads(msg_str)
            except:
                return

            self.logger.debug(f"Receive msg: {message}")
            ## Command: Set obstacles ##
            if message["cat"] == "obstacles":
                self.action_queue.put(PiAction(**message))
                # self.action_queue.put(PiAction(**message)) ## THIS SHOULD BE BELOW
                self.logger.debug(f"Set obstacles PiAction added to queue: {message}")


            ## Command: Start Moving ##
            if message["cat"] == "control" and message["value"].upper() == "START":
                # Commencing path following
                if not self.command_queue.empty():
                    # Main trigger to start movement #
                    time.sleep(3)
                    self.start_movement.set()
                    self.android_queue.put(AndroidMessage("info", "Starting robot!"))
                    # self.request_algo(data=action.value)
                else:
                    self.logger.warning("Empty obstacles")
                    self.android_queue.put(AndroidMessage("error", "Empty obstacles"))

    def rpi_action(self):
        # return
        while True:
            action: PiAction = self.action_queue.get()
            self.logger.debug(
                f"PiAction retrieved from queue: {action.cat} {action.value}"
            )
            if action.cat == "obstacles":
                for obs in action.value["obstacles"]:
                    self.obstacles[obs["id"]] = obs
                print("Acions", action.value)
                self.request_algo(action.value) ## THIS DOESN'T FEEL LIKE IT SHOULD BE HERE
            elif action.cat == "snap":
                self.snap_and_rec(obstacle_id_with_signal=action.value)
            elif action.cat == "stitch":
                self.request_stitch()

    def stop(self):
        """Stops all processes on the RPi and disconnects gracefully with Android and STM32"""
        self.stm_link.disconnect()
        self.logger.info("Program exited!")

    def recv_stm(self) -> None:
        """
        [Child Process] Receive acknowledgement messages from STM32, and release the movement lock
        """
        while True:
            message = self.stm_link.recv()
            if message == None:
                raise Exception("Invalid message")

            self.logger.info(f"STM Callback message {message}")

            if message.startswith("ACK"):
                self.lock_stm.release()
                self.logger.debug("ACK received, movement lock released.")
            else:
                self.logger.warning(f"Ignored unknown message from STM: {message}")

    def command_follower(self) -> None:
        """
        [Child Process] that execute stm/smap commands
        """
        while True:
            # Retrieve next movement command
            if self.lock_stm.get_value() == 1:
                self.lock_stm.acquire()
                command: str = self.command_queue.get()
                # Wait for android start command [Main Trigger]
                # self.start_movement.wait() ### REMOVED THIS BECAUSE THIS IS NOT A VALID FUNCTION
                # self.movement_lock.acquire()
                # STM32 Commands - Send straight to STM32
                stm32_prefixes = ("FW", "BW", "FL", "FR", "BL", "BR", "SS")
                if command.startswith(stm32_prefixes):
                    time.sleep(1)
                    # self.movement_lock.acquire(blocking=True) # commented out this because we can do acquire on top
                    self.stm_link.send(command)
                    self.logger.debug(f"Sending movement command to STM32: {command}")

                # Snap command
                elif command.startswith("SNAP"):
                    obstacle_id_with_signal = command.replace("SNAP", "")
                    self.action_queue.put(
                        PiAction(cat="snap", value=obstacle_id_with_signal)
                    )
                # End of path
                elif command == "SSSSS":
                    self.action_queue.put(PiAction(cat="stitch", value=None))
                    self.logger.info(f"Finished!!!")

    def capture_image(self, filename: str):
        os.system(
            f"libcamera-still -e jpg -n -t 500 -o {filename} --awb auto > /dev/null 2>&1"
        )

    def snap_and_rec(self, obstacle_id_with_signal: str) -> None:
        """
        RPi snaps an image and calls the API for image-rec.
        The response is then forwarded back to the android
        :param obstacle_id_with_signal: the current obstacle ID followed by underscore followed by signal
        """
        obstacle_id, signal = obstacle_id_with_signal.split("_")
        self.logger.info(f"Capturing image for obstacle id: {obstacle_id}")
        filename = f"{obstacle_id}_{signal}.jpg"
        self.capture_image(filename)

        url = f"http://{API_IP}:{API_PORT}/image"
        response = requests.post(url, files={"file": (filename, open(filename, "rb"))})

        if response.status_code != 200:
            self.logger.error(
                "Something went wrong when requesting path from image-rec API. Please try again."
            )
            self.lock_stm.release()
            return

        results = response.json()

        # release lock so that bot can continue moving
        # self.movement_lock.release()
        self.lock_stm.release()

        self.logger.info(f"results: {results}")
        self.logger.info(f"Detected image id: {results['image_id']}")
        self.logger.info(f"self.obstacles: {self.obstacles}")
        self.logger.info(
            f"Image recognition results: {results} ({SYMBOL_MAP.get(results['image_id'])})"
        )

        self.android_queue.put(
            AndroidMessage(
                "image-rec",
                json.dumps(
                    {"image_id": results["image_id"], "obstacle_id": obstacle_id}
                ),
            )
        )

    def request_algo(
        self, data, robot_x=1, robot_y=1, robot_dir=0, retrying=False
    ):
        """
        Requests for a series of commands and the path from the Algo API.
        The received commands and path are then queued in the respective queues
        """
        self.logger.info("Requesting path from algo...")
        self.android_queue.put(AndroidMessage("info", "Requesting from algo..."))
        self.logger.info(f"data: {data}")

        body = {
           **data,
           "big_turn": "0",
           "robot_x": robot_x,
           "robot_y": robot_y,
           "robot_dir": robot_dir,
           "retrying": retrying,
        }
        url = f"http://{API_IP}:{API_PORT}/path"
        response = requests.post(url, json=body)

        # Error encountered at the server, return early
        if response.status_code != 200:
            self.logger.error("Something went wrong when requesting path from Algo API")
            return

        # Parse response
        result = json.loads(response.content)["data"]
        commands = result["commands"]
        path = result["path"]

        # Log commands received
        self.logger.debug(f"Commands received from API: {commands}")

        # Put commands and paths into respective queues
        self.clear_queues()
        for c in commands:
            self.command_queue.put(c)

        # ignore first element as it is the starting position of the robot
        for p in path[1:]:
            self.path_queue.put(p)

        # self.android_queue.put(AndroidMessage("info", "Commands and path received Algo API. Robot is ready to move."))
        self.logger.info("Commands and path received Algo API. Robot is ready to move.")

    def request_stitch(self):
        """Sends a stitch request to the image recognition API to stitch the different images together"""
        url = f"http://{API_IP}:{API_PORT}/stitch"
        response = requests.get(url)

        # If error, then log, and send error to Android
        if response.status_code != 200:
            # Notify android
            self.logger.error(
                "Something went wrong when requesting stitch from the API."
            )
            return

        self.logger.info("Images stitched!")

    def clear_queues(self):
        """Clear both command and path queues"""
        while not self.cached_command_queue.empty():
            self.cached_command_queue.get()
        while not self.command_queue.empty():
            self.command_queue.get()
        while not self.path_queue.empty():
            self.path_queue.get()

    def check_api(self) -> bool:
        """Check whether image recognition and algorithm API server is up and running

        Returns:
            bool: True if running, False if not.
        """
        # Check image recognition API
        url = f"http://{API_IP}:{API_PORT}/status"
        try:
            response = requests.get(url, timeout=1)
            if response.status_code == 200:
                self.logger.debug("API is up!")
                return True
            return False
        # If error, then log, and return False
        except ConnectionError:
            self.logger.warning("API Connection Error")
            return False
        except requests.Timeout:
            self.logger.warning("API Timeout")
            return False
        except Exception as e:
            self.logger.warning(f"API Exception: {e}")
            return False


if __name__ == "__main__":
    rpi = RaspberryPi()
    rpi.start()
