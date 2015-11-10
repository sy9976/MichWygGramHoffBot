from bluetooth import *

server_sock=BluetoothSocket( RFCOMM )

server_sock.bind(("",PORT_ANY))
server_sock.listen(1)

port = server_sock.getsockname()[1]
uuid = "00001101-0000-1000-8000-00805F9B34FB"
#uuid = "94f39d29-7d6d-437d-973b-fba39e49d4ee"
#uuid = "fb36491d-7c21-40ef-9f67-a63267b5bbea"
advertise_service( server_sock, "AquaPiServer",
                   service_id = uuid,
                   service_classes = [ uuid, SERIAL_PORT_CLASS ],
                   profiles = [ SERIAL_PORT_PROFILE ], 
#                   protocols = [ OBEX_UUID ] 
                    )

client_sock,address = server_sock.accept()
print "Accepted connection from ",address

data = client_sock.recv(1024)
print "received [%s]" % data

client_sock.close()
server_sock.close()

