from socket import AF_INET,SOCK_STREAM,socket


while True:
    message = sobj.recv(2048)
    if(message == "exit"):
        sobj.close()
        break
    elif(message.decode() == ''):
        pass
    else:
        print(message.decode())
class TCPSocket:
    def create(self, ip, port):
        self.socket = socket(AF_INET,SOCK_STREAM)
        self.socket.connect((ip,port))
        print("\n\nCONNECTED TO {}:{}\n\n".format(ip, port))
    def send(self, data):
        self.socket.
