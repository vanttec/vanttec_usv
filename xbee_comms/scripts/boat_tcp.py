# Echo client program
import socket

HOST = '127.0.0.1'    # The remote host
PORT = 50000              # The same port as used by the server
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((HOST, PORT))
while 1:
    msg = input("Valor a mandar: ")
    print("Sending", msg)
    s.send(bytes(msg, 'utf-8'))
    #data = s.recv(1024)
    #print 'Received', repr(data)
