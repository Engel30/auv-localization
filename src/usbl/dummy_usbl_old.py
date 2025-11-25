from settings import CODEC, LOCAL_MACHINE_IP, USBL_PORT

IP = LOCAL_MACHINE_IP # "192.168.0.1"

import asyncio

async def handle_client(reader: asyncio.StreamReader, writer: asyncio.StreamWriter):
    """
    This coroutine is executed for each client that connects.
    'reader' is an asyncio.StreamReader.
    'writer' is an asyncio.StreamWriter.
    """
    client_addr = writer.get_extra_info('peername')
    print(f"New connection from {client_addr}")

    try:
        write_mode = False
        if write_mode:
            while True:
                data = await asyncio.to_thread(input, "> ")
                message = f'+++AT:61:RECVIM,p0,18,2,3,ack,340015,-32,133,0.0534,[{data}\r\n'

                writer.write(message.encode(CODEC))

                await writer.drain()
        else:
            while True:
                data = await reader.readline()

                if not data:
                    # An empty bytes object means the client closed the connection.
                    print(f"Connection closed by {client_addr}")
                    break

                message = data.decode(CODEC).strip()
                print(f"Received: {data}")

                if "+++ATI" in message:
                    replies = ["+++ATI:3:1.9"]
                elif "SENDIM" in message:
                    _, payload = message.split(",[")
                    replies = ["+++AT*SENDIM:2:OK", "+++AT:13:DELIVEREDIM,3",
                               "+++AT:123:USBLLONG,144.369001,144.132576,3,-0.0649,-3.3087,-0.9908,-3.0506,-0.5748,-1.5156,0.1632,-0.0799,-1.4027,2303,-58,142,0.0586"]
                else:
                    replies = []
                    print(
                        "This is a dummy server which support only firmware information and send instant message commands."
                    )

                for r in replies:
                    print(f"Sending: {r}")
                    writer.write((r+"\n").encode(CODEC))

                    # .drain() flushes the write buffer. It's the async version of
                    # ensuring data is sto {client_addr}ent. It waits until the buffer is clear.
                    await writer.drain()
    except ConnectionResetError:
        print(f"Connection reset by {client_addr}")
    finally:
        print(f"Closing connection to {client_addr}")
        writer.close()
        await writer.wait_closed()


async def main():
    # Start a TCP server.
    server = await asyncio.start_server(handle_client, IP, USBL_PORT)

    addr = server.sockets[0].getsockname()
    print(f'Serving on {addr}')

    async with server:
        await server.serve_forever()


# Run the main coroutine
if __name__ == "__main__":
    server: asyncio.Server | None = None
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nServer shutting down.")
    finally:
        if server:
            server.close_clients()
            server.close()