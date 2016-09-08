#!/usr/bin/env python3

""" An asynchronous socket <-> LoRa interface """

# MIT License
#
# Copyright (c) 2016 bjcarne
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.


import sys, asyncore
from time import time
from SX127x.LoRa import *
from SX127x.board_config import BOARD

BOARD.setup()

class Server(asyncore.dispatcher):
    def __init__(self, host, port):
        asyncore.dispatcher.__init__(self)
        self.create_socket()
        self.set_reuse_addr()
        self.bind((host, port))
        self.listen(1) 
        
    def handle_accepted(self, sock, addr):
        print("Connection from %s:%s" % sock.getpeername())
        self.conn = Handler(sock)

class Handler(asyncore.dispatcher):
    def __init__(self, sock):
        asyncore.dispatcher.__init__(self, sock)
        self.databuffer = b""
        self.tx_wait = 0
    
    # when data is available on socket send to LoRa
    def handle_read(self):
        if not self.tx_wait:
            data = self.recv(127)
            print('Send:' + str(data))
            lora.write_payload(list(data))
            lora.set_dio_mapping([1,0,0,0,0,0]) # set DIO0 for txdone
            lora.set_mode(MODE.TX)
            self.tx_wait = 1 
    
    # when data for the socket, send
    def handle_write(self):
        if self.databuffer:
            self.send(self.databuffer)
            self.databuffer = b""
            
    def handle_close(self):
        print("Client disconnected")
        self.close()
        
class LoRaSocket(LoRa):
    
    def __init__(self, verbose=False):
        super(LoRaSocket, self).__init__(verbose)
        self.set_mode(MODE.SLEEP)
        self.set_pa_config(pa_select=1)
        self.set_max_payload_length(128) # set max payload to max fifo buffer length
        self.payload = []
        self.set_dio_mapping([0] * 6) #initialise DIO0 for rxdone  
        
    # when LoRa receives data send to socket conn
    def on_rx_done(self):        
        payload = self.read_payload(nocheck=True)
        
        if len(payload) == 127:
            self.payload[len(self.payload):] = payload
        else:
            self.payload[len(self.payload):] = payload
            print('Recv:' + str(bytes(self.payload)))

            server.conn.databuffer = bytes(self.payload) #send to socket conn
            self.payload = []

        self.clear_irq_flags(RxDone=1) # clear rxdone IRQ flag
        self.reset_ptr_rx()
        self.set_mode(MODE.RXCONT)
    
    # after data sent by LoRa reset to receive mode
    def on_tx_done(self):
        self.clear_irq_flags(TxDone=1) # clear txdone IRQ flag
        self.set_dio_mapping([0] * 6)    

        self.set_mode(MODE.RXCONT)
        server.conn.tx_wait = 0
   

if __name__ == '__main__':
    
    server = Server('localhost', 20000)
    
    lora = LoRaSocket(verbose=False)

    print(lora)
    
    try:
        asyncore.loop()

    except KeyboardInterrupt:
        sys.stderr.write("\nKeyboardInterrupt\n")
        
    finally:
        lora.set_mode(MODE.SLEEP)
        print("Closing socket connection")
        server.close()
        BOARD.teardown()
