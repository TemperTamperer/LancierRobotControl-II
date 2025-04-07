import optirx as rx

def main():
    # Create a socket to listen for data
    multicast_address = "239.255.42.99"  # Default multicast address used by OptiTrack
    port = 1511  # Default port
    sock = rx.mkdatasock(multicast_address, port)

    while True:
        data = sock.recv(rx.MAX_PACKETSIZE)
        packet = rx.unpack(data)
        
        if type(packet) is rx.FrameOfData:
            # Iterate over all rigid bodies
            for rigid_body in packet.rigid_bodies:
                if rigid_body.id == YOUR_RIGID_BODY_ID:
                    position = rigid_body.position
                    print(f"Rigid Body ID: {rigid_body.id}, Position: {position}")

if __name__ == "__main__":
    YOUR_RIGID_BODY_ID = 1  # Replace with your actual rigid body ID
    main()