from pymavlink import mavutil

def connect_to_mavlink(port):
    """
    Establish a connection to a MAVLink system.

    Args:
        port (int): The port number to connect to.

    Returns:
        mavutil.mavlink_connection: A MAVLink connection object.

    Example:
        >>> master = connect_to_mavlink(14541)
    """
    master = mavutil.mavlink_connection('udp:127.0.0.1:{}'.format(port))
    master.wait_heartbeat()
    print("Heartbeat from system (system %u component %u)" % (
            master.target_system, master.target_system))
    return master

def receive_attitude_messages(master):
    """
    Receive and print ATTITUDE messages from the MAVLink system.

    Args:
        master (mavutil.mavlink_connection): A MAVLink connection object.

    Example:
        >>> master = connect_to_mavlink(14541)
        >>> receive_attitude_messages(master)
    """
    while True:
        try:
            msg = master.recv_match(type='ATTITUDE', blocking=True)
            if not msg:
                raise ValueError()
            print(msg.to_dict())
        except KeyboardInterrupt:
            print('Key board Interrupt! exit')
            break

def send_trajectory_representation_bezier_message(master, **kwargs):
    """
    Send a TRAJECTORY_REPRESENTATION_BEZIER message to the MAVLink system.

    Args:
        master (mavutil.mavlink_connection): A MAVLink connection object.
        **kwargs: Additional keyword arguments to be included in the message.

    Example:
        >>> master = connect_to_mavlink(14541)
        >>> send_trajectory_representation_bezier_message(master, time_usec=0, bezier_points=[(0, 0, 0), (1, 1, 1)])
    """
    msg = master.MAVLinkMessage(
        mavutil.mavlink.MAVLINK_MSG_ID_TRAJECTORY_REPRESENTATION_BEZIER,
        **kwargs
    )
    master.send(msg)

# Example usage:
master = connect_to_mavlink(14541)
receive_attitude_messages(master)
send_trajectory_representation_bezier_message(master, time_usec=0, bezier_points=[(0, 0, 0), (1, 1, 1)])