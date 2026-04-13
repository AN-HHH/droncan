"""
DroneCAN reception example.

Receives and decodes DroneCAN messages using the dronecan library.
This replaces the raw CAN reception approach in can_receiver.py.

DroneCAN (formerly UAVCAN v0) is a lightweight protocol built on top of CAN bus
for communication in unmanned aerial vehicles. It provides message framing,
node addressing, data-type encoding, and multi-frame transfer reassembly on top
of raw CAN.
"""

import dronecan


def on_node_status(event: dronecan.Event) -> None:
    """Handle incoming NodeStatus broadcast messages."""
    msg = event.message
    node_id = event.transfer.source_node_id
    print(
        f"[DroneCAN] NodeStatus from node {node_id}: "
        f"uptime={msg.uptime_sec}s "
        f"health={msg.health} "
        f"mode={msg.mode} "
        f"vendor_specific_status_code={msg.vendor_specific_status_code}"
    )


def on_esc_status(event: dronecan.Event) -> None:
    """Handle incoming ESCStatus broadcast messages."""
    msg = event.message
    node_id = event.transfer.source_node_id
    print(
        f"[DroneCAN] ESCStatus from node {node_id}: "
        f"rpm={msg.rpm} "
        f"voltage={msg.voltage:.2f}V "
        f"current={msg.current:.2f}A "
        f"temperature={msg.temperature:.1f}K "
        f"esc_index={msg.esc_index} "
        f"error_count={msg.error_count}"
    )


def receive_dronecan_messages(
    channel: str = "vcan0",
    bustype: str = "socketcan",
    node_id: int = 127,
) -> None:
    """
    Receive and decode DroneCAN messages from the CAN bus.

    :param channel:  CAN channel/interface name (e.g. 'vcan0', 'can0').
    :param bustype:  python-can bus type (e.g. 'socketcan', 'slcan').
    :param node_id:  DroneCAN node ID for this receiver node (1-127).
    """
    node = dronecan.make_node(
        f"{bustype}:{channel}",
        node_id=node_id,
        node_info=dronecan.uavcan.protocol.GetNodeInfo.Response(
            name="dronecan_receiver"
        ),
    )

    # Subscribe to NodeStatus messages (broadcast by every node on the network)
    node.add_handler(dronecan.uavcan.protocol.NodeStatus, on_node_status)

    # Subscribe to ESCStatus messages (sent by ESC nodes)
    node.add_handler(dronecan.uavcan.equipment.esc.Status, on_esc_status)

    print(
        f"[DroneCAN] Listening on {channel} ({bustype}) as node {node_id}..."
    )

    try:
        node.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        print("[DroneCAN] Node closed.")


if __name__ == "__main__":
    receive_dronecan_messages()
