import time
import sys
import logging
from multiprocessing import Process, Manager
from communication.stm32 import STMLink

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

    def start(self):
        """Starts the RPi orchestrator"""
        try:
            self.stm_link.connect()
            self.proc_recv_stm32 = Process(target=self.recv_stm)
            self.proc_recv_stm32.start()
            self.logger.info("Child Process started")

            ### Start up complete ###
            self.move_forward()

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

    def move_forward(self):
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
        
        self.stm_link.send("FR180")

        #self.stm_link.send("FW...")

        # time.sleep(float(sys.argv[1]))
        # self.stm_link.send("Sxxxx")
        #time.sleep(3)
        #self.stm_link.send("SSSSS")
        # Wait for acknowledgement from STM32
        while self.movement_lock.value == 1:  # Wait until the lock is released
            time.sleep(0.1)
        # After receiving acknowledgement, update location
        # self.current_location['x'] += 1  # Assuming x-coordinate increment by 1 for simplicity
        self.logger.info(f"Robot moved forward. New location: {self.current_location}")

if __name__ == "__main__":
    rpi = RaspberryPi()
    rpi.start()

