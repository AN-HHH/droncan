"""
CAN reception example (original approach).

Receives raw CAN frames using the python-can library.
This is the old approach - see dronecan_receiver.py for the new DroneCAN-based approach.
"""

import can


def receive_can_messages(channel: str = "vcan0", bustype: str = "socketcan") -> None:
    """
    Receive raw CAN frames.

    :param channel: CAN channel/interface name (e.g. 'vcan0', 'can0').
    :param bustype: python-can bus type (e.g. 'socketcan', 'slcan').
    """
    bus = can.interface.Bus(channel=channel, bustype=bustype)
    print(f"[CAN] Listening on {channel} ({bustype})...")

    try:
        for msg in bus:
            print(
                f"[CAN] id=0x{msg.arbitration_id:08X} "
                f"dlc={msg.dlc} "
                f"data={msg.data.hex()}"
            )
    except KeyboardInterrupt:
        pass
    finally:
        bus.shutdown()
        print("[CAN] Bus closed.")


if __name__ == "__main__":
    receive_can_messages()
