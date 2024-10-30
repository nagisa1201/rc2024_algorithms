import pyrealsense2 as rs

def list_realsense_devices():
    context = rs.context()
    devices = context.query_devices()
    if len(devices) == 0:
        print("No Realsense devices found")
    else:
        for i, device in enumerate(devices):
            print(f"Device {i}: {device.get_info(rs.camera_info.name)}")
            print(f"  Serial Number: {device.get_info(rs.camera_info.serial_number)}")
            print(f"  Firmware Version: {device.get_info(rs.camera_info.firmware_version)}")
            print(f"  USB Type: {device.get_info(rs.camera_info.usb_type_descriptor)}")
            print("")

if __name__ == "__main__":
    list_realsense_devices()
