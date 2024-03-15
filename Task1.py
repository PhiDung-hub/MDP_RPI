# _!/venv/bin/python

import json
import queue
import time
from multiprocessing import Process, Manager
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

        self.manager = Manager()

        # self.android_dropped = self.manager.Event()
        self.unpause = self.manager.Event()

        self.movement_lock = self.manager.Lock()

        # Messages to send to Android
        # self.android_queue = self.manager.Queue()
        # Messages that need to be processed by RPi
        self.rpi_action_queue = self.manager.Queue()
        # Messages that need to be processed by STM32, as well as snap commands
        self.command_queue = self.manager.Queue()
        # X,Y,D coordinates of the robot after execution of a command
        self.path_queue = self.manager.Queue()

        self.proc_recv_stm32 = None
        self.proc_android_sender: Process | None = None
        self.proc_rpi_action = None
        self.proc_command_follower = None
        self.success_obstacles = self.manager.list()
        self.failed_obstacles = self.manager.list()
        self.obstacles = self.manager.dict()
        self.current_location = self.manager.dict()
        self.failed_attempt = False

    def start(self):
        """Starts the RPi orchestrator"""
        try:
            ### Start up initialization ###

            # self.android_link.connect()
            # self.android_queue.put(AndroidMessage('info', 'You are connected to the RPi!'))
            self.stm_link.connect()
            self.check_api()

            # Define child processes
            self.proc_recv_stm32 = Process(target=self.recv_stm)
            self.proc_command_follower = Process(target=self.command_follower)
            self.proc_rpi_action = Process(target=self.rpi_action)

            # Start child processes
            self.proc_recv_stm32.start()
            self.proc_command_follower.start()
            self.proc_rpi_action.start()

            self.logger.info("Child Processes spawned")

            ### Start up complete ###

            # Send success message to Android
            # self.android_queue.put(AndroidMessage('info', 'Robot is ready!'))
            # self.android_queue.put(AndroidMessage('mode', 'path'))
            # self.reconnect_android()

        except KeyboardInterrupt:
            self.stop()

    def rpi_action(self):
        while True:
            if self.rpi_action_queue.empty():
                self.logger.debug("Queue empty")
                continue

            action: PiAction = self.rpi_action_queue.get()
            self.logger.debug(
                f"PiAction retrieved from queue: {action.cat} {action.value}"
            )
            print("Action", action)

            if action.cat == "obstacles":
                for obs in action.value["obstacles"]:
                    self.obstacles[obs["id"]] = obs
                self.request_algo(action.value)
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

            if message == "ACK":
                self.movement_lock.release()
                self.logger.debug("ACK received, movement lock released.")

                cur_location = self.path_queue.get_nowait()

                self.current_location["x"] = cur_location["x"]
                self.current_location["y"] = cur_location["y"]
                self.current_location["d"] = cur_location["d"]
                self.logger.info(f"self.current_location = {self.current_location}")
                # self.android_queue.put(AndroidMessage('location', {
                #     "x": cur_location['x'],
                #     "y": cur_location['y'],
                #     "d": cur_location['d'],
                # }))
            else:
                self.logger.warning(f"Ignored unknown message from STM: {message}")

    def command_follower(self) -> None:
        """
        [Child Process] that execute stm/smap commands
        """
        while True:
            # Retrieve next movement command
            command: str = self.command_queue.get()
            # Wait for unpause event to be true [Main Trigger]
            
            # Acquire lock first (needed for both moving, and snapping pictures)
            self.movement_lock.acquire(blocking=True)

            # STM32 Commands - Send straight to STM32
            stm32_prefixes = ("FW", "BW", "FL", "FR", "BL", "BR", "SS")
            if command.startswith(stm32_prefixes):
                self.stm_link.send(command)
                self.logger.debug(f"Sending movement command to STM32: {command}")

            # Snap command
            elif command.startswith("SNAP"):
                obstacle_id_with_signal = command.replace("SNAP", "")
                self.rpi_action_queue.put(
                    PiAction(cat="snap", value=obstacle_id_with_signal)
                )

            # End of path
            elif command == "SSSSS":
                self.logger.info(
                    f"At FIN, self.failed_obstacles: {self.failed_obstacles}"
                )
                self.logger.info(
                    f"At FIN, self.current_location: {self.current_location}"
                )
                if len(self.failed_obstacles) != 0 and self.failed_attempt == False:
                    # NOTE: retrying
                    new_obstacle_list = list(self.failed_obstacles)
                    for obj in list(self.success_obstacles):
                        # {'x': 5, 'y': 11, 'id': 1, 'd': 4}
                        obj["d"] = 8
                        new_obstacle_list.append(obj)

                    self.logger.info("Attempting to go to failed obstacles")
                    self.failed_attempt = True
                    self.request_algo(
                        {"obstacles": new_obstacle_list, "mode": "0"},
                        self.current_location["x"],
                        self.current_location["y"],
                        self.current_location["d"],
                        retrying=True,
                    )
                    self.movement_lock.release()
                    continue

                self.unpause.clear()
                self.movement_lock.release()
                self.logger.info("Commands queue finished.")
                # self.android_queue.put(AndroidMessage(
                #     "info", "Commands queue finished."))
                # self.android_queue.put(AndroidMessage("status", "finished"))
                # self.rpi_action_queue.put(PiAction(cat="stitch", value=""))
            else:
                raise Exception(f"Unknown command: {command}")

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
            return

        results = response.json()

        # release lock so that bot can continue moving
        self.movement_lock.release()

        self.logger.info(f"results: {results}")
        self.logger.info(f"Detected image id: {results['image_id']}")
        self.logger.info(f"self.obstacles: {self.obstacles}")
        self.logger.info(
            f"Image recognition results: {results} ({SYMBOL_MAP.get(results['image_id'])})"
        )

        # if results['image_id'] == 'NA':
        #     self.failed_obstacles.append(
        #         self.obstacles[int(results['obstacle_id'])])
        #     self.logger.info(
        #         f"Added Obstacle {results['obstacle_id']} to failed obstacles.")
        #     self.logger.info(f"self.failed_obstacles: {self.failed_obstacles}")
        # else:
        #     self.success_obstacles.append(
        #         self.obstacles[int(results['obstacle_id'])])
        #     self.logger.info(
        #         f"self.success_obstacles: {self.success_obstacles}")

        # TODO: put queue message to android
        # self.android_queue.put(AndroidMessage("image-rec", results))

    # NOTE: CHANGE THIS TO ADJUST THE MAP
    def request_algo(
        self, data=None, robot_x=1, robot_y=1, robot_dir=0, retrying=False
    ):
        """
        Requests for a series of commands and the path from the Algo API.
        The received commands and path are then queued in the respective queues
        """
        # self.logger.info("Requesting path from algo...")
        # self.android_queue.put(AndroidMessage(
        #     "info", "Requesting path from algo..."))
        # self.logger.info(f"data: {data}")
        # body = {**data, "big_turn": "0", "robot_x": robot_x,
        #         "robot_y": robot_y, "robot_dir": robot_dir, "retrying": retrying}
        body = {
            "obstacles": [
                {"x": 5, "y": 11, "id": 1, "d": 4},
                {"x": 9, "y": 8, "id": 2, "d": 2},
            ],
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
    rpi.request_algo()
    rpi.stm_link.connect()
    rpi.proc_recv_stm32 = Process(target=rpi.recv_stm)
    rpi.proc_recv_stm32.start()
    rpi.command_follower()

    # rpi.proc_rpi_action = Process(target=rpi.rpi_action)
    # rpi.proc_rpi_action.start()
    # rpi.start()
