import serial
import time
import sys
import threading


def listen():

    #while running:
    if (arduino.inWaiting() > 0):
        msg1 = arduino.read(arduino.inWaiting()).decode("utf-8", "ignore")
        
        #msg1 = msg1.replace("\n", "")
        msg1 = msg1.replace("\r", "")
        if msg1 is not "":
            print(msg1, end = ' ')
			
arduino = serial.Serial('COM11', 9600)
running = True
#thread1 = threading.Thread(target=listen, args=(arduino,))
#thread1.start()

time.sleep(4)


while True:
    listen()
    msg = input()
    # if msg == "exit":
    #     print(msg.encode('ascii'))
    #     thread1.join()
    if msg != "" or msg != "\n":
        arduino.write(msg.encode('ascii'))

        
