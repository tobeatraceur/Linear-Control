# 先电脑和小车蓝牙配对-密码为1234
import bluetooth
import time
service_matches = bluetooth.find_service( address = "AB:AF:C2:56:34:02" )
first_match = service_matches[0]
port = first_match["port"]
name = first_match["name"]
host = first_match["host"]

# nearby_devices = bluetooth.discover_devices(lookup_names=True)
# print("found %d devices" % len(nearby_devices))
# for addr, name in nearby_devices:
#     print("  %s - %s" % (addr, name))
sock=bluetooth.BluetoothSocket(bluetooth.RFCOMM)
sock.connect((host, port))
# 发送内容写在这
# sock.send(chr(2))
sock.send("{333+10+10}")
# sock.send("{900+00+00}")
sock.close()