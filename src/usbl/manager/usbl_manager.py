import asyncio

import logging
import serial
import serial_asyncio

from manager.usbl_command import UsblCommand
from manager.usbl_translator import encode_usbl_command, decode_usbl_response


class UsblManager:
    """
    Manages concurrent reading and writing for USBL's TCP socket and serial port (UART).
    """

    def __init__(
        self,
        codec: str,
        usbl_ip: str,
        usbl_port: int,
        usbl_connection_timeout: int,
        period_instant_message: float,
        localization_target_id: int,
        serial_port_id: str,
        serial_baudrate: int,
        serial_connection_timeout: int,
    ):
        # System configuration
        self._codec = codec

        # TCP socket (USBL) configuration
        self._usbl_ip = usbl_ip
        self._usbl_port = usbl_port
        self._period_instant_message = period_instant_message
        self._usbl_connection_timeout = usbl_connection_timeout

        # UART configuration
        self._serial_port_id: str = serial_port_id
        self._serial_baudrate: int = serial_baudrate
        self._serial_connection_timeout: int = serial_connection_timeout


        # Stream readers and writers
        self._usbl_stream_reader: asyncio.StreamReader | None = None
        self._usbl_stream_writer: asyncio.StreamWriter | None = None
        self._serial_stream_reader: asyncio.StreamReader | None = None
        self._serial_stream_writer: asyncio.StreamWriter | None = None

        # A event flag to launch firmware command when usbl is connected with tcp
        self.usbl_connection_ready_event = asyncio.Event()

        # A event flag to ensure proper behaviour of usbl once is connected
        self.usbl_communication_ready_event = asyncio.Event()

        self._localization_target_id = localization_target_id

        # Queues for decoupling send logic from writer loops
        self._usbl_send_queue: asyncio.Queue[bytes] = asyncio.Queue()
        self._uart_send_queue: asyncio.Queue[bytes] = asyncio.Queue()

        self._tasks: list[asyncio.Task] = []

    # Context Manager Method (called on init when using `async with Class() as class`)
    # Reference: https://docs.python.org/3/reference/datamodel.html#asynchronous-context-managers
    async def __aenter__(self):
        await self._setup()
        return self

    # Context Manager Method (called on exit when using `async with Class() as class`)
    async def __aexit__(self, exc_type, exc_val, exc_tb):
        await self._stop()

    async def _setup(self):
        logging.info("Manager setup started")
        try:
            logging.info("Connecting to USBL...")
            self._usbl_stream_reader, self._usbl_stream_writer = await asyncio.wait_for(
                asyncio.open_connection(
                    self._usbl_ip, self._usbl_port
                ),
                timeout=self._usbl_connection_timeout
            )
            logging.info(f"USBL connection established at {self._usbl_ip}:{self._usbl_port}")

            logging.info("Connecting to serial port...")
            self._serial_stream_reader, self._serial_stream_writer = await asyncio.wait_for(
                serial_asyncio.open_serial_connection(
                    url=self._serial_port_id, baudrate=self._serial_baudrate
                ),
                timeout=self._serial_connection_timeout
            )
            logging.info(f"Serial connection opened on {self._serial_port_id}")

            logging.info("Manager setup completed")
        except TimeoutError as e:
            logging.error(f"Could not connect within the timeout to TCP or serial. Reason: {e}")
            await self._stop()
            raise
        except ValueError as e:
            logging.error(f"Invalid parameter in some function/s during setup. Reason: {e}")
            await self._stop()
            raise
        except TypeError as e:
            logging.error(f"Invalid parameter's type in some function/s during setup. Reason: {e}")
            await self._stop()
            raise
        except SyntaxError as e:
            logging.error(f"Invalid syntax in some function/s during setup. Reason: {e}")
            await self._stop()
            raise
        except ImportError as e:
            logging.error(f"An import is missing in setup. Reason: {e}")
            await self._stop()
            raise
        except serial.SerialException as e:
            logging.error(f"Serial connection failed. Is the device connected in correct port or is another program using the port? Reason: {e}")
            await self._stop()
            raise
        except ConnectionRefusedError as e:
            logging.error(f"USBL connection over TCP failed. Is the USBL running on {self._usbl_ip}:{self._usbl_port}? Reason: {e}")
            await self._stop()
            raise
        except FileNotFoundError as e:
            logging.error(f"Serial port not found. Is {self._serial_port_id} correct and available? Reason: {e}")
            await self._stop()
            raise
        except Exception as e:
            logging.critical(f"Unexpected error occurred in _setup(): {e}")
            await self._stop()
            raise

    async def run(self):
        logging.info("Manager starting...")
        # Create and store tasks
        self._tasks = [
            asyncio.create_task(
                coro=self._usbl_reader(),
                name="USBL reader"
            ),
            asyncio.create_task(
                coro=self._usbl_writer(),
                name="USBL writer"
            ),
            asyncio.create_task(
                coro=self._serial_reader(),
                name="Serial reader"
            ),
            asyncio.create_task(
                coro=self._serial_writer(),
                name="Serial writer"
            ),
        ]

        logging.info("Manager started")

        # When a task ends for any reason (crash, ...), terminate the program in order to prevent a broken state.
        done, pending = await asyncio.wait(
            self._tasks,
            return_when=asyncio.FIRST_COMPLETED
        )

        for task in done:
            try:
                task.result()  # This will re-raise the exception if the task failed
            except Exception as e:
                logging.error(f"Task '{task.get_name()}' failed, triggering shutdown: {e}")

        # Shutdown gracefully
        await self._stop()

    async def _stop(self):
        logging.debug("Manager stopping...")

        # Cancel all running tasks
        for task in self._tasks:
            if not task.done():
                task.cancel()

            # Ensure task is actually cancelled
            try:
                await task
            except asyncio.CancelledError:
                # If task is actually cancelled, this exception is expected
                pass
            except Exception as e:
                logging.warning(f"A task got an error when it was cancelling: {e}. Ignoring...")

        # Close tcp writer stream
        if self._usbl_stream_writer:
            self._usbl_stream_writer.close()
            await self._usbl_stream_writer.wait_closed()
            logging.debug("USBL writer closed")

        # Close serial writer stream
        if self._serial_stream_writer:
            self._serial_stream_writer.close()
            await self._serial_stream_writer.wait_closed()
            logging.debug("Serial writer closed")


        logging.info("Manager stopped")

        async def _usbl_reader(self):
            """Reads data via TCP from USBL and pass it to serial."""
            logging.debug("Started")
            while True:
                try:
                    line: bytes = await self._usbl_stream_reader.readline()
                    if not line:
                        logging.warning("USBL closed connection")
                        break
                    elif len(line) == 0:
                        continue
                    line = line.strip()

                    # Try to decode for logging - but keep original bytes for processing
                    try:
                        decoded_line = line.decode(self._codec)
                        logging.debug(f"From USBL: {decoded_line}")
                    except UnicodeDecodeError:
                        # Log as hex if decoding fails
                        logging.debug(f"From USBL (binary): {line.hex()}")
                        logging.warning(f"Received binary data that couldn't be decoded as UTF-8: {line[:100]}")

                    if self.usbl_communication_ready_event.is_set():
                        is_good_response, enu_coordinates, message = decode_usbl_response(line,
                                                                                        UsblCommand.SEND_INSTANT_MESSAGE_WITH_ACKNOLEDGEMENT)
                        if is_good_response and message:
                            # Forward message to UART (it should contain the message the other MCU received in UART)
                            await self.send_to_uart(message)
                    else:
                        is_good_response = decode_usbl_response(line, UsblCommand.FIRMWARE_INFORMATION)
                        if is_good_response:
                            self.usbl_communication_ready_event.set()
                            logging.info("USBL is connected and ready to receive commands.")
                except asyncio.CancelledError:
                    break
                except UnicodeDecodeError as e:
                    logging.error(f"Unicode decode error: {e}. Raw data (hex): {line.hex() if 'line' in locals() else 'N/A'}")
                    continue  # Skip this message and continue processing
                except ConnectionResetError:
                    logging.error("TCP connection was forcibly closed by the USBL.")
                    break
                except Exception as e:
                    logging.critical(f"Error: {e}")
                    break
            logging.debug("Finished")

    async def _usbl_writer(self):
        """Waits for data on the queue and writes it to the USBL via TCP socket """
        logging.debug("Started")
        while True:
            try:
                usbl_data_to_send : bytes = await self._usbl_send_queue.get()
                if not usbl_data_to_send or len(usbl_data_to_send) == 0:
                    continue
                usbl_data_to_send = usbl_data_to_send.strip()

                logging.debug(f"[{self._usbl_send_queue.qsize()} {self._uart_send_queue.qsize()}] Data to be sent: {usbl_data_to_send}")
                self._usbl_stream_writer.write(usbl_data_to_send + b'\n')

                # TODO: consider to .task_done() when message is actually sent so when usbl replies with DELIVEREDIM
                # .drain() flushes the write buffer. It's the async version of
                # ensuring data is sent. It waits until the buffer is clear.
                await self._usbl_stream_writer.drain()
                self._usbl_send_queue.task_done()
                logging.info(f"Data sent: {usbl_data_to_send}")

                # TODO: temporary mitigation for queue overflow
                await asyncio.sleep(self._period_instant_message)
            except asyncio.CancelledError:
                # This is thrown when my_task.cancel() and the associated method _my_task() is blocked in `await`.
                # It is a trigger at end of task as soon as requested, so this is the expected behavior
                break
            except (ConnectionResetError, BrokenPipeError) as e:
                logging.error(f"USBL connection lost. Reason: {e}.")
                break
            except Exception as e:
                logging.critical(f"Error: {e}")
                break
        logging.debug("Finished")

    async def _serial_reader(self):
        """Reads data from the serial port and pass it to USBL"""
        logging.debug("Started")
        while True:
            try:
                line: bytes = await self._serial_stream_reader.readline()
                if not line:
                    logging.warning("Serial connection closed by device.")
                    break
                elif len(line) == 0:
                    continue
                line = line.strip()
                logging.info(f"From serial: {line}")

                try:
                    message, has_priority = encode_usbl_command(UsblCommand.SEND_INSTANT_MESSAGE_WITH_ACKNOLEDGEMENT,
                                                                target=self._localization_target_id, payload=line)
                    await self.send_to_usbl(message)
                    if message and (has_priority or self.is_usbl_queue_empty()):
                        pass
                except Exception as e:
                    logging.debug(f"Cannot decode data red from serial: {e}. Skipping...")
            except asyncio.CancelledError:
                # This is thrown when my_task.cancel() and the associated method _my_task() is blocked in `await`.
                # It is a trigger at end of task as soon as requested, so this is the expected behavior
                break
            except (ConnectionResetError, BrokenPipeError, serial.SerialException) as e:
                logging.error(f"Connection lost: {e}")
                break
            except Exception as e:
                logging.critical(f"Error: {e}.")
                break

        logging.debug("Finished")

    async def _serial_writer(self):
        """Waits for data on the queue and writes it to the serial port."""
        logging.debug("Started")
        while True:
            try:
                serial_data_to_send: bytes = await self._uart_send_queue.get()
                if not serial_data_to_send or len(serial_data_to_send) == 0:
                    continue
                serial_data_to_send = serial_data_to_send.strip()

                # Add newline and carriage return
                self._serial_stream_writer.write(serial_data_to_send + b'\r\n')
                logging.debug(f"Data to be sent: {serial_data_to_send}")

                # .drain() flushes the write buffer. It's the async version of
                # ensuring data is sent. It waits until the buffer is clear.
                await self._serial_stream_writer.drain()
                self._uart_send_queue.task_done()
                logging.info(f"Data sent: {serial_data_to_send}")
            except asyncio.CancelledError:
                # This is thrown when my_task.cancel() and the associated method _my_task() is blocked in `await`.
                # It is a trigger at end of task as soon as requested, so this is the expected behavior
                break
            except (ConnectionResetError, BrokenPipeError, serial.SerialException) as e:
                logging.error(f"Connection lost: {e}")
                break
            except Exception as e:
                logging.critical(f"Error: {e}")
                break
        logging.debug("Finished")

    # --- Public functions for sending data to USBL and serial ---
    async def send_to_usbl(self, data: bytes):
        """Queue data for sending over TCP to USBL. The data must follow the AT format in order to be understood by USBL"""
        await self._usbl_send_queue.put(data)

    async def send_to_uart(self, data: bytes):
        """Queue data for sending over serial to UART-connected device."""
        await self._uart_send_queue.put(data)

    def is_usbl_queue_empty(self) -> bool:
        return self._usbl_send_queue.empty()