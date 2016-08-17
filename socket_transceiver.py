#!/usr/bin/env python3

""" A socket interface designed to take IP payloads and transmit them to another LoRa """
#
# add some licence 
#

import sys
from socket import socket
from time import sleep
from SX127x.LoRa import *
from SX127x.board_config import BOARD

BOARD.setup()

class LoRaSocket(LoRa):

    def __init__(self, verbose=False):
        super(LoRaSocket, self).__init__(verbose)
        self.set_mode(MODE.SLEEP)
        self.set_dio_mapping([0] * 6) #initialise DIO0 for rxdone  
        
    def create_socket(self):
        host = "127.0.0.1"
        port = 20000
         
        mySocket = socket()
        mySocket.bind((host,port))
         
        mySocket.listen(1)
        print("Waiting for socket connection")
        
        conn, addr = mySocket.accept()
        print ("Connection from: " + str(addr))        
        
        return conn

    def on_rx_done(self):
        self.clear_irq_flags(RxDone=1) # clear rxdone IRQ flag
        
        payload = self.read_payload(nocheck=True) #get payload from lora fifo
        print('Payload received:' + str(bytes(payload)))
        
        conn.send(bytes(payload)) #send to socket conn
        
        self.reset_ptr_rx()
        self.set_mode(MODE.RXCONT)


    def on_tx_done(self):
        print('\ntxdone')
        self.clear_irq_flags(TxDone=1) # clear txdone IRQ flag
        self.set_dio_mapping([0] * 6) #DIO0 set for rxdone    
        self.set_mode(MODE.RXCONT)

    def on_cad_done(self):
        print("\non_CadDone")
        print(self.get_irq_flags())

    def on_rx_timeout(self):
        print("\non_RxTimeout")
        print(self.get_irq_flags())

    def on_valid_header(self):
        print("\non_ValidHeader")
        print(self.get_irq_flags())    

    def on_payload_crc_error(self):
        print("\non_PayloadCrcError")
        print(self.get_irq_flags())

    def on_fhss_change_channel(self):
        print("\non_FhssChangeChannel")
        print(self.get_irq_flags())

    def start(self):
        self.set_max_payload_length(128) # set max payload to max fifo buffer length
        self.reset_ptr_rx() 
        self.set_mode(MODE.RXCONT) # stay in RXCONT unless we have data to transmit 
        
        while True:
                data = conn.recv(128) # wait for data on socket
                if not data:
                    break
                        
                # send data to LoRa
                self.set_dio_mapping([1,0,0,0,0,0]) # set DIO0 for txdone
                self.write_payload(list(data)) 
                self.set_mode(MODE.TX)
    

if __name__ == '__main__':
    
    lora = LoRaSocket(verbose=True)

    lora.set_pa_config(pa_select=1)

    print(lora)
    
    assert(lora.get_agc_auto_on() == 1)

    try: input("Press enter to start...")
    except: pass

    try:
        conn = lora.create_socket() # set up socket connection
        lora.start()
    except KeyboardInterrupt:
        sys.stdout.flush()
        print("")
        sys.stderr.write("KeyboardInterrupt\n")
    finally:
        sys.stdout.flush()
        print("")
        lora.set_mode(MODE.SLEEP)
        print(lora)
        print("Closing socket connection")
        conn.close()
        BOARD.teardown()
