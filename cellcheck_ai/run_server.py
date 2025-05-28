import sys
import os
import asyncio

sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from socket_server import main  # WebSocket entry point

if __name__ == "__main__":
    asyncio.run(main())