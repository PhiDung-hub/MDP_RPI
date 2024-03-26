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
        self.stm_link = STMLink()
        self.android_link = AndroidLink()

        self.manager = Manager()

        self.android_dropped = self.manager.Event()
        self.start_movement = self.manager.Event()
        self.lock_stm = Semaphore(1)
        self.lock_stm.acquire()

        # Messages to send to Android
        self.android_queue = self.manager.Queue()
        # Messages that need to be processed by RPi
        # Messages that need to be processed by STM32, as well as snap commands
        self.command_queue = self.manager.Queue()
        self.dists = self.manager.list()

        self.proc_recv_android: Optional[Process] = None
        self.proc_recv_stm32: Optional[Process] = None
        self.proc_android_sender: Optional[Process] = None
        self.proc_command_follower: Optional[Process] = None

    def start(self):
        """Starts the RPi orchestrator"""
        try:
            ### Start up initialization ###
            self.stm_link.connect()
            self.android_link.connect()
            self.android_queue.put(AndroidMessage("info", "Connected to the RPi!"))

            # Define and start child processes
            self.proc_command_follower = Process(target=self.command_follower)
            self.proc_command_follower.start()
            self.proc_recv_stm32 = Process(target=self.recv_stm)
            self.proc_recv_stm32.start()
            self.proc_android_sender = Process(target=self.android_sender)
            self.proc_android_sender.start()
            self.proc_recv_android = Process(target=self.recv_android)
            self.proc_recv_android.start()
            self.logger.info("Child Processes spawned")

            ### Start up complete ###

            # Send success message to Android
            self.android_queue.put(AndroidMessage("info", "Robot is ready!"))
            self.reconnect_android()

        except KeyboardInterrupt:
            self.stop()

    def reconnect_android(self):
        """Handles the reconnection to Android in the event of a lost connection."""
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

            if not (
                message["cat"] == "control" and message["value"].upper() == "START"
            ):
                continue

            ## Command: Start Moving ##
            # Commencing path following
            self.android_queue.put(AndroidMessage("info", "Starting robot!"))

            time.sleep(1)
            self.start_movement.set()

            # NOTE: cache self.dists[1]
            self.lock_stm.release()
            self.command_queue.put("FW150")
            self.command_queue.join()
            image_1 = self.snap_and_rec("Task2_image_1")

            if int(image_1) == 39:  # Left
                self.command_queue.put("FL000")
                self.command_queue.put("FR000")
                self.command_queue.put("FW030")
                self.command_queue.put("FR000")
                self.command_queue.put("FL000")
            else:
                self.command_queue.put("FR000")
                self.command_queue.put("FL000")
                self.command_queue.put("FW030")
                self.command_queue.put("FL000")
                self.command_queue.put("FR000")

            # NOTE: cache self.dists[2]
            self.command_queue.put("FW150")

            # BLock until completes
            self.command_queue.join()
            image_2 = self.snap_and_rec("Task2_image_2")

            if int(image_2) == 39:  # Left
                self.command_queue.put("FL000")
                self.command_queue.put("FW050")
                self.command_queue.put("FR000")
                self.command_queue.put("FW040")
                self.command_queue.put("FR000")
                self.command_queue.put("FW120")
                self.command_queue.put("FR000")

            else:  # Right
                self.command_queue.put("FR000")
                self.command_queue.put("FW050")
                self.command_queue.put("FL000")
                self.command_queue.put("FW040")
                self.command_queue.put("FR000")
                self.command_queue.put("FW120")
                self.command_queue.put("FR000")

            # backward = int(self.dists[0] + self.dists[1]) * 10 + 60
            #
            # CMD_BACK = f"FW{f'{backward}' if backward >= 100 else f'0{backward}'}"
            # self.logger.info(
            #     f"\n\n\n GOT SIGNAL TO WENT BACK HOME {CMD_BACK}!!! \n\n\n!!!"
            # )
            #
            # self.command_queue.put(CMD_BACK)
            self.command_queue.put("FW240")

            if int(image_2) == 38:  # right
                self.command_queue.put("FL000")
                self.command_queue.put("FW030")
                self.command_queue.put("FR000")
            else:
                self.command_queue.put("FR000")
                self.command_queue.put("FW030")
                self.command_queue.put("FL000")

            self.command_queue.put("FW060")
            self.command_queue.put("SSSSS")  # -10
            # self.android_queue.put(AndroidMessage("status", "cooking..."))

    def stop(self):
        """Stops all processes on the RPi and disconnects gracefully with Android and STM32"""
        self.android_link.disconnect()
        self.stm_link.disconnect()
        self.logger.info("Program exited!")
        raise Exception("Program exit!")

    def recv_stm(self) -> None:
        """
        [Child Process] Receive acknowledgement messages from STM32, and release the movement lock
        """
        while True:
            message = self.stm_link.recv()
            self.logger.warning(f"Message from STM: {message}")
            if message == None:
                raise Exception("Invalid message")

            self.logger.info(f"STM Callback message {message}")

            if message.startswith("ACK"):
                self.lock_stm.release()
                self.command_queue.task_done()
                message.strip()

                try:
                    if len(message) == 5:
                        dist = int(message[3:])
                        self.dists.append(dist)
                        self.logger.info(f"Got distance args {dist}")
                        print(dist, "print", self.dists)
                except:
                    print("not distance msg", message)
                finally:
                    self.logger.debug("ACK received, movement lock released.")

    def command_follower(self) -> None:
        """
        [Child Process] that execute stm/smap commands
        """
        while True:
            if self.lock_stm.get_value() != 1:
                print("Locked")
                continue

            self.lock_stm.acquire()
            # Retrieve next movement command
            command: str = self.command_queue.get()

            # STM32 Commands - Send straight to STM32
            stm32_prefixes = ("FW", "BW", "FL", "FR", "BL", "BR", "SS")
            if command.startswith(stm32_prefixes):
                time.sleep(3)  # MUST BE DONE
                self.stm_link.send(command)
                self.logger.debug(f"Sending movement command to STM32: {command}")

            # End of path
            elif command == "SSSSS":
                self.lock_stm.release()
                self.logger.info("Robot cooked")
            else:
                raise Exception(f"Unknown command: {command}")

    def capture_image(self, filename: str):
        os.system(
            f"libcamera-still -e jpg -n -t 500 -o {filename} --awb auto > /dev/null 2>&1"
        )

    def snap_and_rec(self, obstacle_id: str) -> int | str:
        """
        RPi snaps an image and calls the API for image-rec.
        The response is then forwarded back to the android
        :param obstacle_id_with_signal: the current obstacle ID followed by underscore followed by signal
        """
        while True:
            if self.lock_stm.get_value() != 1:
                continue
            self.lock_stm.acquire()
            self.logger.info(f"Capturing image for obstacle id: {obstacle_id}")
            filename = f"{obstacle_id}.jpg"
            self.capture_image(filename)

            url = f"http://{API_IP}:{API_PORT}/image"
            response = requests.post(url, files={"file": (filename, open(filename, "rb"))})

            if response.status_code != 200:
                self.logger.error("Path from image-rec API dead! Please try again.")
                self.lock_stm.release()
                return 0

            results = response.json()
            self.lock_stm.release()

            # release lock so that bot can continue moving
            # self.movement_lock.release()

            self.logger.info(f"Results: {results}")
            self.logger.info(
                f"Image recognition results: {results} ({SYMBOL_MAP.get(results['image_id'])})"
            )

            return results["image_id"]

    def clear_queues(self):
        """Clear both command and path queues"""
        while not self.command_queue.empty():
            self.command_queue.get()


if __name__ == "__main__":
    rpi = RaspberryPi()
    rpi.start()
