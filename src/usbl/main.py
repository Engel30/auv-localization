import asyncio
import logging

from settings import TRANSPONDER_CONFIG, INSTANT_MESSAGE_INTERVAL_SEC, USBL_PORT, CODEC, \
    SERIAL_PORT_ID, SERIAL_BAUDRATE, USBL_CONNECTION_TIMEOUT, SERIAL_CONNECTION_TIMEOUT, DUMMY_TEST_CONFIG
from manager.usbl_manager import UsblManager

# Possible values (check bottom of settings.py): TRANSPONDER_CONFIG, TRANSCEIVER_CONFIG
USBL_IP, LOCALIZATION_TARGET_IP = DUMMY_TEST_CONFIG # CHANGE THIS

class TaskNameFilter(logging.Filter):
    """Add taskName to log records when available"""
    def filter(self, record):
        if not hasattr(record, 'taskName'):
            record.taskName = 'Main'
        return True

async def main():
    # `with` GUARANTEES execution of __aenter__ and __aexit__ of UsblManager (in particular cases) so it ensures its clean up
    async with UsblManager(
        codec=CODEC,
        usbl_ip=USBL_IP,
        usbl_port=USBL_PORT,
        usbl_connection_timeout=USBL_CONNECTION_TIMEOUT,
        period_instant_message=INSTANT_MESSAGE_INTERVAL_SEC,
        localization_target_id=LOCALIZATION_TARGET_IP,
        serial_port_id=SERIAL_PORT_ID,
        serial_baudrate=SERIAL_BAUDRATE,
        serial_connection_timeout=SERIAL_CONNECTION_TIMEOUT
    ) as m:
        manager_task = asyncio.create_task(
            coro=m.run(),
            name="Manager"
        )

        # Wait for the main manager task to complete.
        await manager_task

if __name__ == '__main__':
    logging.basicConfig(
        level=logging.DEBUG,
        format="%(asctime)s | %(levelname)7s | %(taskName)13s | %(message)s",
    )
    
    # Add filter to handle missing taskName
    for handler in logging.root.handlers:
        handler.addFilter(TaskNameFilter())

    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        logging.info(f"User requested shutdown.")
    except Exception as e:
        # It is usually already logged
        logging.critical(f"Error: {e}")
    finally:
        logging.info("Program stopped")