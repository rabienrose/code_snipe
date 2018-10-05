# coding=UTF-8
import socket
import time
import threading
import struct

import urllib2
from PIL import Image
import cStringIO
import matplotlib.pyplot as plt


img=None
aaa=0;
def ImageScale(body):
    print("bodylen:%s" % len(body))
    global img
    global index
    file = cStringIO.StringIO(body)
    im = Image.open(file)
 
#    img.show()
#     plt.figure("images")
#    plt.imshow(img)
#    plt.show()

    if img is None:
        index=0
        #img = plt.imshow(im)
    else:
       index=index+1
	#img.set_data(im)
    

    if index>5:
        index=0 
        plt.pause(.01)
        plt.draw()


s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  #创建socket (AF_INET:IPv4, AF_INET6:IPv6) (SOCK_STREAM:面向流的TCP协议)

s.bind(('192.168.25.197', 10021))                           #绑定本机IP和任意端口(>1024)

s.listen(1)                                            #监听，等待连接的最大数目为1

print('Server is running...')                          

def dataHandle(headPack, body):
#    local=open('/Volumes/chamo/dataset/ipad/online/%s.jpg'%headPack[0],'wb')
#    local.write(body)
#    local.close()
    global aaa
    aaa= aaa+1
    print(aaa)
    #ImageScale(body)

def TCP(sock, addr):                                   #TCP服务器端处理逻辑
    #global dataBuffer
    #global headerSize
    
    dataBuffer=bytes()
    print('Accept new connection from %s:%s.' %addr)   #接受新的连接请求
    headerSize = 8
    while True:
        data = sock.recv(5024*5024)                         #接受其数据
        #time.sleep(1)                                  #延迟
        if not data :        #如果数据为空或者'quit'，则退出
            break
        #sock.send(data.decode('utf-8').upper().encode())  #发送变成大写后的数据,需先解码,再按utf-8编码,  encode()其实就是encode('utf-8')

        dataBuffer += data
        while True:
            if len(dataBuffer) < headerSize:
                #print("数据包（%s Byte）小于消息头部长度，跳出小循环" % len(dataBuffer))
                break
            # 读取包头
            # struct中:!代表Network order，3I代表3个unsigned int数据
            headPack = struct.unpack('!2I', dataBuffer[:headerSize])
            bodySize = headPack[1]
            #print("index:%s, bodySize:%s" % headPack)
            # 分包情况处理，跳出函数继续接收数据
            if len(dataBuffer) < headerSize+bodySize :
                print("数据包（%s Byte）不完整（总共%s Byte），跳出小循环" % (len(dataBuffer), headerSize+bodySize))
                break
            # 读取消息正文的内容
            body = dataBuffer[headerSize:headerSize+bodySize]
                
            # 数据处理
            dataHandle(headPack, body)
                            
            # 粘包情况的处理
            dataBuffer = dataBuffer[headerSize+bodySize:]


    #print('receive:%s\n' %data)
    sock.close()
    dataBuffer=bytes()
    #关闭连接
    print('Connection from %s:%s closed.' %addr)       

while True:
    
    sock, addr = s.accept()                            #接收一个新连接
    TCP(sock, addr)                                    #处理连接
