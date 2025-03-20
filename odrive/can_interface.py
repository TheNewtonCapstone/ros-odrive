import can
import threading
import time
from typing import Dict, Callable, Optional, List, Tuple
import asyncio
from enum import IntEnum, unique
import rclpy
import rclpy.logging
from rich import print
from rich.console import Console
from dataclasses import dataclass, field 
import uuid
from .exception import CanException, MessageNotSentException

console = Console()


@unique
class Arbitration(IntEnum):
    """
    Arbitration ID for CAN messages
    Odrive can use 11 bit message Identifiers

    | Node ID | Command ID |
    | 5 bits  | 5 bits     |
    """

    NODE_ID_SIZE = 5
    ARBITRATION_ID_SIZE = 0x1F

    
@dataclass
class ResponseFuture:
    """
    Class to track requests and their responses
    """
    created_at: float
    event: threading.Event
    response: Optional[bytes] = None
    def wait(self, timeout: float) -> Optional[bytes]:
        if self.event.wait(timeout):
            return self.response
        return None

class CanInterface:
    """
    Interface for communicating with ODrives over CAN bus
    """

    def __init__(self, interface: str = "can0", bitrate: int = 1000000):
        self.interface = interface
        self.bitrate = bitrate

        self.bus: Optional[can.Bus] = None
        self.callback: Optional[Callable] = None
        self.receive_thread: Optional[threading.Thread] = None
        self.running: bool = False

        # Request tracking
        self.request_lookup: Dict[int, Tuple[int, int]] = {}
        self.pending_responses: Dict[str, ResponseFuture] = {}
        self.responses_lock = threading.Lock()
        
        

    def start(self, callback: Callable[[int, int, bytes], None]) -> bool:
        """
        Start CAN interface
        Args:
            callback: Function to call for each received message with
            (node_id, cmd_id, data) parameters
        Returns:
            bool: True if successfully started, False otherwise
        """
        if self.running:
            return False
        try:
            self.bus = can.Bus(
                channel=self.interface, bustype="socketcan", bitrate=self.bitrate
            )

            self.callback = callback
            self.running = True
            self.receive_thread = threading.Thread(target=self._receive_loop)
            self.receive_thread.daemon = True
            self.receive_thread.start()

            return True

        except Exception as e:
            console.print(
                f"[red]Can interface: Error starting CAN interface: {e}[/red]"
            )
            if self.bus:
                self.bus.shutdown()
                self.bus = None
            return False

    def stop(self) -> None:
        """
        Stop CAN interface
        """
        self.running = False
        if self.receive_thread and self.receive_thread.is_alive():
            self.receive_thread.join(timeout=10.0)
        if self.bus:
            self.bus.shutdown()
            self.bus = None

    def send_frame(self, arbitration_id: int, data: bytes) -> bool:
        """
        Send a CAN message
        Args:
            arbitration_id: CAN arbitration ID
            data: Data to send
        Returns:
            bool: True if successfully sent, False otherwise
        """
        try:
            # removed the check for running because we might need to send messages before we want to init the loop
            if not self.bus:
                raise ValueError("CAN interface not started")

            msg = can.Message(
                arbitration_id=arbitration_id, data=data, is_extended_id=False
            )
            self.bus.send(msg)
            return True

        except Exception as e:
            console.print(f"[red]Can interface: Error sending CAN message: {e}[/red]")
            return False
        

    def request(self, node_id: int, cmd_id: int, data: bytes, timeout: float = 3.0) -> Optional[bytes]:
        # Generate a unique request ID
        request_id = str(uuid.uuid4())
        
        # Create a future for this response
        future = ResponseFuture(
            created_at=time.time(),
            event=threading.Event()
        )
        
        # Store the future
        with self.responses_lock:
            self.pending_responses[request_id] = future
        
        # Construct and send the message
        arbitration_id = (node_id << Arbitration.NODE_ID_SIZE | cmd_id)
        
        # Store the request_id -> (node_id, cmd_id) mapping for lookup in receive loop
        self.request_lookup[request_id] = (node_id, cmd_id)
        
        try:
            # Send the message
            success = self.send_frame(arbitration_id, data)
            if not success:
                return None
            
            # Wait for the response
            return future.wait(timeout)
        finally:
            # Clean up
            with self.responses_lock:
                if request_id in self.pending_responses:
                    del self.pending_responses[request_id]
                if request_id in self.request_lookup:
                    del self.request_lookup[request_id]
                
                
                
        
    def _receive_loop(self):
        try:
            while self.running:
                msg = self.bus.recv(timeout=0.1)
                if msg and not msg.is_error_frame:
                    node_id = msg.arbitration_id >> Arbitration.NODE_ID_SIZE
                    cmd_id = msg.arbitration_id & Arbitration.ARBITRATION_ID_SIZE
                    
                    # Check if this response matches any pending request
                    with self.responses_lock:
                        # Find matching requests
                        matches = []
                        for req_id, (req_node, req_cmd) in self.request_lookup.items():
                            if req_node == node_id and req_cmd == cmd_id:
                                matches.append(req_id)
                        
                        # If we have matches, fulfill the oldest request first (FIFO)
                        if matches:
                            # Sort by creation time
                            matches.sort(key=lambda x: self.pending_responses[x].created_at)
                            oldest_request = matches[0]
                            
                            # Set the response
                            future = self.pending_responses[oldest_request]
                            future.response = bytes(msg.data)
                            future.event.set()
                    
                    # Call the callback if set
                    if self.callback:
                        self.callback(node_id, cmd_id, msg.data)
        except Exception as e:
            console.print(f"CAN interface: Error in receive loop: {e}")
            time.sleep(0.1)
        


class AsyncCanInterface:
    """
    Interface for communicating with ODrives over CAN bus using asyncio. This class is used to improve perfomance
    """

    def __init__(self, interface: str = "can0", bitrate: int = 1000000):
        self.interface = interface
        self.bitrate = bitrate
        self.bus: Optional[can.Bus] = None
        self.callback: Optional[Callable] = None
        self.reader: Optional[can.AsyncBufferedReader] = None
        self.notifier: Optional[can.Notifier] = None
        self.running = False
        self.initilized = False

        
    async def start(self, callback: Callable[[int, int, bytes], None]) -> bool:
        """
        Start CAN interface
        Args:
            callback: Function to call for each received message with
            (node_id, cmd_id, data) parameters
        Returns:
            bool: True if successfully started, False otherwise
        """
        console.print("Starting CAN interface")
        if self.running:
            return False
        try:
            self.bus = can.Bus(
                channel=self.interface, bustype="socketcan", bitrate=self.bitrate
            )
            self.callback = callback
            self.reader = can.AsyncBufferedReader()
            loop = asyncio.get_event_loop()
            self.notifier = can.Notifier(self.bus, [self.reader], loop=loop)

            self.running = True
            self.initilized = True
            
            asyncio.create_task(self._receive_loop())
            await asyncio.sleep(0.1)
            
            return True

        except Exception as e:
            console.print(
                f"[red]Async Can interface: Error starting CAN interface: {e}[/red]"
            )
            if self.bus:
                self.bus.shutdown()
                self.bus = None
            return False

    def stop(self) -> None:
        """
        Stop async CAN interface
        """
        self.running = False
        self.initilized = False
        if self.notifier:
            self.notifier.stop()
        if self.bus:
            self.bus.shutdown()
            self.bus = None

    def send_frame(self, arbitration_id: int, data: bytes) -> bool:
        """
        Send a CAN message
        Args:
            arbitration_id: CAN arbitration ID
            data: Data to send
        Returns:
            bool: True if successfully sent, False otherwise
        """
        try:
            if not self.bus or not self.initilized:
                raise ValueError("CAN interface not started or initialized")
            msg = can.Message(
                arbitration_id=arbitration_id, data=data, is_extended_id=False
            )
            self.bus.send(msg)
            return True
        except Exception as e:
            console.print(f"[red]Async Can interface: Error sending CAN message: {e}[/red]")
            return False

    async def _receive_loop(self) -> None:
        """
        Async loop for receiving CAN messages
        """
        last_message_time = time.time()
        try:
            while self.running:
                msg = await asyncio.wait_for(self.reader.get_message(), timeout=0.1)
                console.print(f"Received message: {msg}")
                if msg and not msg.is_error_frame:
                    node_id = msg.arbitration_id >> Arbitration.NODE_ID_SIZE
                    cmd_id = msg.arbitration_id & Arbitration.ARBITRATION_ID_SIZE
                    if self.callback:
                        self.callback(node_id, cmd_id, msg.data)
                await asyncio.sleep(0.001)
        except Exception as e:
            console.print(
                f"Async Can interface: Error receiving CAN message in receive loop: {e}"
            )
            await asyncio.sleep(0.1)
            # await self._receive_loop()