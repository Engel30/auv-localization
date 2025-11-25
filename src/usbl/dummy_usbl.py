from settings import CODEC, LOCAL_MACHINE_IP, USBL_PORT
import asyncio
import json

IP = LOCAL_MACHINE_IP

# Simulated message queue - messages to be sent from remote transponder
MESSAGE_QUEUE = [
    "Hello from underwater!",
    json.dumps({"type": "status", "battery": 85, "depth": 15.5}),
    json.dumps({"type": "sensor", "temp": 12.3, "pressure": 2.1}),
    "Test message with special chars: #$%",
    json.dumps({"type": "alert", "message": "Low battery warning"}),
]

async def handle_client(reader: asyncio.StreamReader, writer: asyncio.StreamWriter):
    """
    This coroutine is executed for each client that connects.
    """
    client_addr = writer.get_extra_info('peername')
    print(f"\n{'='*60}")
    print(f"New connection from {client_addr}")
    print(f"{'='*60}\n")

    message_index = 0  # Track which message to send next

    try:
        while True:
            data = await reader.readline()

            if not data:
                print(f"Connection closed by {client_addr}")
                break

            message = data.decode(CODEC).strip()
            print(f"\n[RECEIVED] {message}")

            # Parse the command
            if "+++ATI" in message:
                # Firmware information request
                replies = ["+++ATI:3:1.9"]
                print(f"[ACTION] Sending firmware info")
                
            elif "SENDIM" in message:
                # Instant message send command
                # Extract payload from message
                try:
                    _, payload = message.split(",[")
                    print(f"[ACTION] Instant message received with payload: {payload}")
                except:
                    payload = "unknown"
                
                # Simulate successful message delivery with responses
                replies = [
                    "+++AT*SENDIM:2:OK",
                    "+++AT:23:SENDSTART,3,im,182272,0",
                    "+++AT:30:SENDEND,3,im,1853167139,182272",
                    "+++AT:9:RECVSTART",
                    "+++AT:32:RECVEND,1853461060,139376,-36,77",
                    "+++AT:13:DELIVEREDIM,3",
                ]
                
                # Add localization data (USBLLONG)
                # Format: seconds,microseconds,target_id,x,y,z,east,north,up,roll,pitch,yaw,proptime,rssi,integrity,uncertainty
                localization = (
                    "+++AT:124:USBLLONG,10446.991981,10446.534643,3,"
                    "-0.0604,-1.0888,-0.5851,"  # x, y, z
                    "-0.4233,-1.0020,-0.5901,"  # east, north, up
                    "0.0063,0.0269,-0.3295,"    # roll, pitch, yaw
                    "825,-36,77,1.0375"          # proptime, rssi, integrity, uncertainty
                )
                replies.append(localization)
                
                # Now simulate receiving a message back from the remote transponder
                if message_index < len(MESSAGE_QUEUE):
                    response_message = MESSAGE_QUEUE[message_index]
                    message_index += 1
                    
                    # Format: RECVIM with message payload
                    # Check integrity value (>100 is valid according to docs)
                    integrity = 150
                    recvim = (
                        f"+++AT:61:RECVIM,p0,{len(response_message)},2,3,ack,340015,-32,{integrity},0.0534,"
                        f"[{response_message}"
                    )
                    replies.append(recvim)
                    print(f"[ACTION] Sending queued message #{message_index}: {response_message}")
                else:
                    print(f"[INFO] Message queue empty (sent {len(MESSAGE_QUEUE)} messages)")
                
            else:
                replies = []
                print(f"[WARNING] Unsupported command. This dummy server supports:")
                print(f"  - ATI (firmware info)")
                print(f"  - SENDIM (instant messages)")

            # Send all replies
            for r in replies:
                print(f"[SENDING] {r}")
                writer.write((r + "\n").encode(CODEC))
                await writer.drain()
                
                # Small delay between messages for realism
                await asyncio.sleep(0.1)
            
            print()  # Blank line for readability
            
    except ConnectionResetError:
        print(f"\n[ERROR] Connection reset by {client_addr}")
    except Exception as e:
        print(f"\n[ERROR] Unexpected error: {e}")
    finally:
        print(f"\n[CLEANUP] Closing connection to {client_addr}\n")
        writer.close()
        await writer.wait_closed()


async def send_periodic_messages(writer: asyncio.StreamWriter):
    """
    Optional: Send unsolicited messages periodically (like real USBL alerts)
    """
    await asyncio.sleep(5)  # Wait 5 seconds before starting
    
    while True:
        try:
            # Send an unsolicited RECVIM every 10 seconds
            unsolicited_msg = json.dumps({"type": "periodic", "timestamp": asyncio.get_event_loop().time()})
            recvim = (
                f"+++AT:61:RECVIM,p0,{len(unsolicited_msg)},2,3,ack,340015,-32,150,0.0534,"
                f"[{unsolicited_msg}"
            )
            print(f"\n[UNSOLICITED] Sending periodic message: {unsolicited_msg}")
            writer.write((recvim + "\n").encode(CODEC))
            await writer.drain()
            
            await asyncio.sleep(10)
        except Exception as e:
            print(f"[ERROR] Periodic message failed: {e}")
            break


async def main():
    # Start a TCP server
    server = await asyncio.start_server(handle_client, IP, USBL_PORT)

    addr = server.sockets[0].getsockname()
    print(f"\n{'#'*60}")
    print(f"# DUMMY USBL SERVER")
    print(f"# Serving on {addr}")
    print(f"# Message queue size: {len(MESSAGE_QUEUE)}")
    print(f"{'#'*60}\n")

    async with server:
        await server.serve_forever()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n\n[SHUTDOWN] Server shutting down.\n")
    except Exception as e:
        print(f"\n[ERROR] Server error: {e}\n")