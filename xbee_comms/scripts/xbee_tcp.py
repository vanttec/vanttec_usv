#!/usr/bin/env python
import rospy
import socket
from std_msgs.msg import String

#Conection
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

#Placeholder para la comunicacion bidireccional
def topic_callback(data):
    hola = "Hola"

def boat():

    rospy.init_node('boat', anonymous=True)
    rospy.Subscriber("BOAT_TO_STATION", String, topic_callback)
    publisher = rospy.Publisher('received_in_boat',String, queue_size = 10)

    rospy.loginfo('Attempting connection')

    #Incrementar el puerto si esta ocupado
    try:
        port = 49999
        bound = False
        while not bound:
            try:
                port = port + 1
                s.bind(('127.0.0.1', port))
        
                #Could bind
                bound = True
            except socket.error as msg:
                pass

        rospy.loginfo('Client at ' + str(port) + ' tcp port')
            
        s.listen(1)
        conn, addr = s.accept()
        rospy.loginfo('Connection accepted')

        while not rospy.is_shutdown():
            data = conn.recv(1024)
            if data != '':
                publisher.publish(data)
            
        s.close()

    finally:
        rospy.spin()
    
if __name__ == '__main__':
    try:
        boat()
    except rospy.ROSInterruptException:
        pass
