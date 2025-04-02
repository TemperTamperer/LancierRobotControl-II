import time
from natnet_client import DataDescriptions, DataFrame, NatNetClient

def receive_new_frame(data_frame: DataFrame):
    # Process incoming data frame
    print(f"Received frame: {data_frame}")

def receive_new_desc(desc: DataDescriptions):
    # Process data descriptions
    print("Received data descriptions.")

if __name__ == "__main__":
    # Replace with your OptiTrack server's IP address
    server_ip_address = "192.168.x.x"
    # Replace with your Raspberry Pi's IP address
    local_ip_address = "192.168.x.y"

    streaming_client = NatNetClient(server_ip_address=server_ip_address,
                                    local_ip_address=local_ip_address,
                                    use_multicast=False)
    streaming_client.on_data_description_received_event.handlers.append(receive_new_desc)
    streaming_client.on_data_frame_received_event.handlers.append(receive_new_frame)

    with streaming_client:
        streaming_client.request_modeldef()
        while True:
            time.sleep(1)
            streaming_client.update_sync()
