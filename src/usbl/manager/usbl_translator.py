import json
import logging

from settings import CODEC, SPEED_OF_SOUND_MPS
from manager.usbl_command import UsblCommand

INSTANT_MESSAGE_RESPONSE_KEYWORDS: set[str] = {
    "SENDSTART", "SENDEND", "RECVSTART", "RECVEND", "DELIVEREDIM", "USBLLONG", "USBLANGLES", "USBLPHYD", "USBLPHYP",
    "FAILEDIM", "RECVIM", "CANCELEDIM", "ERROR INTERNAL"
}


def decode_usbl_response(decoded_message: bytes, initial_command: str) -> tuple[
    bool,
    tuple[float, float, float, float] | None,
    bytes | None
]:
    """

    :param decoded_message:
    :param initial_command:
    :return: A boolean indicating if the command was successful or not and an eventual tuple containing ENU coordinates.
    """
    str_decoded_message = decoded_message.decode(CODEC)
    split_list: list[str] = str_decoded_message.split(":")


    if "AT" in str_decoded_message:
        # Handle instant messages response
        # Command:
        # +++AT*SENDIM,p0,4,3,ack,test
        #
        # Response:
        # +++AT*SENDIM:2:OK
        # +++AT:23:SENDSTART,3,im,182272,0
        # +++AT:30:SENDEND,3,im,1853167139,182272
        # +++AT:9:RECVSTART
        # +++AT:32:RECVEND,1853461060,139376,-36,77
        # +++AT:13:DELIVEREDIM,3
        # +++AT:124:USBLLONG,10446.991981,10446.534643,3,-0.0604,-1.0888,-0.5851,-0.4233,-1.0020,-0.5901,0.0063,0.0269,-0.3295,825,-36,77,1.2375
        # +++AT:99:USBLPHYD,10446.992626,10446.534642,3,0,-39278,-10591,16777215,-12608,-27941,23818,16777215,16777215
        # +++AT:245:USBLPHYP,10446.992626,10446.534642,3,0,1000000.0000,1000000.0000,1000000.0000,2000000.0000,2000000.0000,2000000.0000,3000000.0000,3000000.0000,3000000.0000,-0.0806,-1.0113,-0.7086,-5000000.0000,-3535534.0000,10606602.0000,-0.0394,-1.1508,-0.4533

        arguments = split_list[2].split(",")
        keyword = arguments[0]

        if keyword not in INSTANT_MESSAGE_RESPONSE_KEYWORDS:
            logging.error(f"The keyword `{keyword}` is not an expected value. Try to reboot the USBL.")
            return False, None, None

        # Get the ENU coordinates from instant message response
        if keyword == "USBLLONG":
            if len(arguments) == 17:
                target_id = arguments[3]

                # x = float(arguments[4])
                # y = float(arguments[5])
                # z = float(arguments[6])

                east = float(arguments[7])
                north = float(arguments[8])
                up = float(arguments[9])

                # USBL Head orientation
                # roll = float(arguments[10])
                # pitch = float(arguments[11])
                # yaw = float(arguments[12])

                propagation_time_us = int(arguments[13])
                # rssi_dbm = int(arguments[14])

                # This is a guess
                # integrity = int(arguments[15])

                # THE EXPLANATION BELOW IS AS ASSUMPTION BASED ON THE LITTLE EVIDENCE FOUND
                # The USBLLONG message provides a 3D position fix for an acoustic target. This field is actually
                # a Position Uncertainty Estimate in meters and has a dual meaning:
                #
                # 1. Valid Position Fix:
                # When the uncertainty value is small compared to the calculated distance, the (East, North, Up)
                # position is reliable, and the uncertainty value represents its estimated error.
                #
                # 2. Invalid Position Fix (Range-Only):
                # When the uncertainty value is nearly identical to the calculated distance, it signals a failed 3D fix.
                # The modem could not determine a reliable bearing but is using this field as a fallback to report the
                # only valid data it has: the  slant range. In this case, the (East, North, Up) position data is meaningless
                # and must be discarded. This situation often correlates with a low value in the integrity (argument 15).
                position_uncertainty_m = float(arguments[16])

                # Calculate slant range from the direct time-of-flight measurement
                slant_range_m = (propagation_time_us / 1_000_000.0) * SPEED_OF_SOUND_MPS

                # --- VALIDATION LOGIC ---
                # If the reported uncertainty is almost the same as the range, the 3D position is invalid.
                # Use a threshold of 95% instead of a direct '==' comparison to handle floating-point inaccuracies.
                if position_uncertainty_m > (0.95 * slant_range_m):
                    # This indicates a failed bearing fix. The position is unreliable.
                    logging.warning(
                        f"Discarding unreliable fix for target {target_id} because uncertainty ({position_uncertainty_m:.2f}m) is too close to range ({slant_range_m:.2f}m)"
                    )
                    return False, None, None

                logging.info(f"Localization data obtained for target {target_id}.")

                return True, (east, north, up, position_uncertainty_m), None
            else:
                logging.warning("Invalid USBLLONG line received. Skipping.")
                return False, None, None
        elif keyword == "DELIVEREDIM":
            logging.info(f"Instant message was delivered to target id: {arguments[1]}")
        elif keyword == "FAILEDIM":
            logging.warning(f"Instant message was NOT delivered to target_id: {arguments[1]}")
        elif keyword == "RECVIM":
            try:
                # Check integrity FIRST before parsing
                integrity = int(arguments[8])
                
                # Value from documentation: integrity > 100 means valid
                if integrity <= 100:
                    logging.warning(f"Received message with LOW integrity ({integrity}). Discarding to avoid corrupted data.")
                    return False, None, None
                
                # Check if there's message payload (as expected), if yes then extract
                _, message = str_decoded_message.rsplit(',[', 1)

            except ValueError as e:
                logging.error(f"Cannot parse RECVIM message: {e}")
                logging.debug(f"Raw message: {str_decoded_message}")
                return False, None, None
            except Exception as e:
                logging.error(f"Error processing RECVIM: {e}")
                return False, None, None

            # Remove terminators
            message = message.strip()

            # Try to parse as JSON
            if message.startswith("{"):
                try:
                    json_message = json.loads(message)
                    message = json.dumps(json_message)
                except Exception:
                    pass

            if integrity > 100:
                integrity_check = "valid"
            else:
                integrity_check = "invalid"
            
            logging.info(f"Message received from target {arguments[3]} with {integrity_check} integrity ({integrity}): {message}")
            return True, None, message.encode(CODEC)
        else:
            logging.debug(f"Keyword received: {keyword}")

        return True, None, None
    else:
        # Check if the response is the right reply for the initial command
        string_response_command: str = split_list[0].replace("+", "")
        response_command = string_response_command

        # All the command's responses contains the initial command
        if response_command != initial_command:
            if response_command == UsblCommand.FIRMWARE_INFORMATION:
                logging.debug(
                    f"Received firmware information reply instead (this is expected). Skipping. Waiting for {initial_command}")
                return True, None, None
            else:
                logging.debug(
                    f"The initial command `{initial_command}` doesn't match the received one `{response_command}`. Try to reboot the USBL."
                )
                return False, None, None

        if response_command == UsblCommand.SEND_INSTANT_MESSAGE_WITH_ACKNOLEDGEMENT:
            # Command:
            # +++AT*SENDIM,p0,4,3,ack,test
            # Raw response:
            # +++AT*SENDIM:2:OK
            return split_list[2] == "OK", None, None
        elif response_command == UsblCommand.SEND_INSTANT_MESSAGE_WITHOUT_ACKNOLEDGEMENT:
            return split_list[2] == "OK", None, None
        elif response_command == UsblCommand.MOTION_COMPENSATED_TRACKING_POSITION_DATA:
            # This command's response estimates the position of the remote target_id in the local coordinate system of the
            # USBL-series device with roll, pitch and heading compensation.
            # The response syntax is: <seconds>,<remote address>,<east>,<north>,<up>
            # The last three are the East, North, Up (ENU) coordinates. East, North, Up are directions
            # of the object-based right-handed reference frame.
            # Raw response example:
            # +++AT?UPX:31:10446,3,-0.4233,-1.0020,-0.5901
            arguments = split_list[2].split(",")

            try:

                # seconds = arguments[0]
                target_id = arguments[1]
                east = float(arguments[2])
                north = float(arguments[3])
                up = float(arguments[4])

                logging.debug(
                    f"The remote target_id `{target_id}` has the following East, North, Up coordinates: {east},{north},{up}"
                )

                return True, (east, north, up, 0), None

            except ValueError as e:
                logging.error(f"Error while parsing value from usbl response (most likely float values): {e}")

                return False, None, None
            except Exception as e:
                logging.error(f"Error while parsing {decoded_message} arguments: {e}")

                return False, None, None

        elif response_command == UsblCommand.FIRMWARE_INFORMATION:
            # Check if the response is correct
            return split_list[2] == "1.9", None, None

        return False, None, None


def encode_usbl_command(
    usbl_command: str,
    target: int | None = None,
    payload: bytes | None = None
) -> tuple[bytes | None, bool]:
    """
    This function takes the usbl commands enum and encodes it in a format which is accepted by the USBL according to the
    S2C acoustic modem manual.

    :param usbl_command: The command to send to USBL
    :param target: The remote address of the target to be passed when sending instant messages. The USBL head has address `2` and the transponder has address `3`. In order to send a broadcast message, you must pass the address `255` and use the USBL command `SEND_INSTANT_MESSAGE_WITHOUT_ACKNOLEDGEMENT`.
    :param payload: The message to be sent at `target`
    :return:
    """
    command = None

    # It has not priority only when payload is json
    has_priority = True

    if usbl_command == UsblCommand.SEND_INSTANT_MESSAGE_WITH_ACKNOLEDGEMENT:
        if target is None or payload is None:
            logging.error("Target and message must be both not null!")
            return None, False

        # Remove terminator
        payload = payload.strip()

        try:
            # Try to make it string
            payload = payload.decode(CODEC)
        except Exception:
            pass

        try:
            json_message = json.loads(payload)
            # This makes the string smaller
            payload = json.dumps(json_message, separators=(',', ':'))

            has_priority = False
        except Exception:
            # It could be not a json, skipping...
            pass

        if len(payload) > 63:
            # The check is 63 because it needs to consider the separator [
            logging.warning(f"USBL allows at most 64 bytes in instant messages. Skipping...")
            return None, False
        else:
            # The separators [ around message is CRUCIAL to avoid parsing conflicts.
            payload = f"[{payload}"
            command = f"{UsblCommand.SEND_INSTANT_MESSAGE_WITH_ACKNOLEDGEMENT},p0,{len(payload)},{target},ack,{payload}"

    elif usbl_command == UsblCommand.SEND_INSTANT_MESSAGE_WITHOUT_ACKNOLEDGEMENT:
        # The separator ,[ around message is CRUCIAL to avoid parsing conflicts.
        command = f"{UsblCommand.SEND_INSTANT_MESSAGE_WITHOUT_ACKNOLEDGEMENT},p0,{len(payload)},{target},noack,{payload}"

    elif usbl_command == UsblCommand.ACOUSTIC_CONNECTION_STATUS:
        command = f"{UsblCommand.ACOUSTIC_CONNECTION_STATUS}"

    elif usbl_command == UsblCommand.MOTION_COMPENSATED_TRACKING_POSITION_DATA:
        command = f"{UsblCommand.MOTION_COMPENSATED_TRACKING_POSITION_DATA}"

    elif usbl_command == UsblCommand.TRACKING_POSITION_DATA:
        command = f"{UsblCommand.TRACKING_POSITION_DATA}"
    elif usbl_command == UsblCommand.FIRMWARE_INFORMATION:
        command = f"{UsblCommand.FIRMWARE_INFORMATION}"

    if command is None:
        logging.critical("Unexpected error: command is None.")
        return None, False

    # OFFICIAL DOCUMENTATION:
    # Which end of line marker - line feed (\n) or carriage return (\r) - ends a command string
    # depends on the interface and the terminal software used to access the device. Ethernet terminals
    # use the line feed (\n), serial terminals - the carriage return (\r). The Ethernet interface of the
    # device is by default configured for a line feed (\n) end of line character, an RS-232 interface –
    # for a carriage return (\r).
    # All messages the device sends back end with a \r\n combination of command characters as
    # the end of line marker.
    return f"+++{command}\n".encode(CODEC), has_priority
