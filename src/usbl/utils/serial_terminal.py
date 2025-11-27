import asyncio

import serial
import serial_asyncio

"""Serial device emulator for testing this USBL manager in Linux/MacOS 

Simulates a hardware device by connecting to one end of a virtual serial port
pair created by the 'socat' utility.

Usage:
1.  Create virtual ports in a terminal (note the two port names):
    $ socat -d -d pty,raw,echo=0 pty,raw,echo=0

2.  Connect the main application to one port (e.g., /dev/ttys043).

3.  Set the port_name variable below to the *other* port (e.g., /dev/ttys044).

4.  Run your main app and this script in separate terminals.
"""

async def writer(serial_writer: asyncio.StreamWriter):
    """Handles user input and sends data via serial"""
    print("\n" + "="*60)
    print("SERIAL TERMINAL - Type your messages below")
    print("="*60)
    while True:
        try:
            # Run the blocking input() in a separate thread
            line = await asyncio.to_thread(input, "> ")
            
            if not line:
                continue

            serial_writer.write((line + '\n').encode())
            await serial_writer.drain()
            print(f"✓ Sent: {line}")
        except asyncio.CancelledError:
            print("\n[WRITER] Cancelled")
            break
        except serial.SerialException as e:
            print(f"\n[WRITER ERROR] Device unplugged? Error: {e}")
            break
        except EOFError:
            print("\n[WRITER] EOF detected (Ctrl+D pressed)")
            break
        except Exception as e:
            print(f"\n[WRITER ERROR] {e}")
            break


async def reader(serial_reader: asyncio.StreamReader):
    """Reads incoming data from serial"""
    print("[READER] Waiting for incoming messages...\n")
    while True:
        try:
            line = await serial_reader.readline()
            
            if not line:
                # Empty read means connection closed
                print("\n[READER] Connection closed")
                break
            
            decoded = line.decode().strip()
            if decoded:  # Only print non-empty messages
                print(f"\n← RECEIVED: {decoded}")
                print("> ", end='', flush=True)  # Redraw prompt
                
        except asyncio.CancelledError:
            print("\n[READER] Cancelled")
            break
        except serial.SerialException as error:
            print(f"\n[READER ERROR] Device unplugged? Error: {error}")
            break
        except asyncio.IncompleteReadError:
            print("\n[READER ERROR] Stream closed mid-read")
            break
        except UnicodeDecodeError as e:
            print(f"\n[READER ERROR] Could not decode message: {e}")
        except Exception as error:
            print(f"\n[READER ERROR] {error}")
            break


async def main():
    serial_reader = None
    serial_writer = None
    
    try:
        # Configure serial port
        port_name = '/dev/pts/7'  # For Windows use COM ports (listed in device manager)
        baud_rate = 115200

        print(f"Connecting to {port_name} at {baud_rate} bps...")
        
        serial_reader, serial_writer = await asyncio.wait_for(
            serial_asyncio.open_serial_connection(url=port_name, baudrate=baud_rate),
            timeout=5
        )
        print(f"✓ Connected to {port_name}")

        # Create both tasks
        reader_task = asyncio.create_task(reader(serial_reader), name="Reader")
        writer_task = asyncio.create_task(writer(serial_writer), name="Writer")

        # Wait for either task to complete
        done, pending = await asyncio.wait(
            [reader_task, writer_task],
            return_when=asyncio.FIRST_COMPLETED,
        )

        # Cancel remaining tasks
        for task in pending:
            task.cancel()
            try:
                await task
            except asyncio.CancelledError:
                pass

    except serial.SerialException as error:
        print(f"\n[ERROR] Serial error: {error}")
    except TimeoutError as error:
        print(f"\n[ERROR] Connection timeout: {error}")
    except KeyboardInterrupt:
        print("\n\n[SHUTDOWN] Exiting program.")
    except FileNotFoundError:
        print(f"\n[ERROR] Port not found. Is '{port_name}' correct?")
        print("Available ports might be:")
        import glob
        for port in glob.glob('/dev/tty*'):
            print(f"  - {port}")
    except PermissionError:
        print(f"\n[ERROR] Cannot access serial port. Try: sudo chmod 666 {port_name}")
    except Exception as error:
        print(f"\n[ERROR] Unexpected error: {error}")
    finally:
        # Cleanup
        if serial_writer:
            serial_writer.close()
            await serial_writer.wait_closed()
        print("\n[CLEANUP] Serial connection closed")


if __name__ == '__main__':
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n[SHUTDOWN] User requested shutdown.")
    finally:
        print("[EXIT] Program stopped")