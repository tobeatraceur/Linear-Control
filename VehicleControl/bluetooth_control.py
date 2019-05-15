# 先电脑和小车蓝牙配对-密码为1234

import bluetooth
import time
import sys

service_matches = bluetooth.find_service( address = "AB:AF:C2:56:34:02" )

if not service_matches:
    print("No device found, exiting.")
    exit()

device_info = service_matches[0]
port = device_info["port"]
host = device_info["host"]

print("port: {} host: {}".format(port, host))

# Open socket
sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
sock.connect((host, port))

try:
    while True:
        speed = input("input speed: ")

        if not speed:
            command = "{333+00+00}"

        else:
            speed = int(speed)
            command = "{333+" + format(speed, '02d') + "+" +\
                    format(speed, '02d') + "}"

        sock.send(command)
        print(command)

except (KeyboardInterrupt, SystemExit):
    sock.send("{333+00+00}")
    sock.close()
    exit(0)

sock.close()

