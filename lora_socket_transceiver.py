#!/usr/bin/env python3

""" A simple beacon transmitter class to send a 1-byte message (0x0f) in regular time intervals. """

# Copyright 2015 Mayer Analytics Ltd.
#
# This file is part of pySX127x.
#
# pySX127x is free software: you can redistribute it and/or modify it under the terms of the GNU Affero General Public
# License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later
# version.
#
# pySX127x is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied
# warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU Affero General Public License for more
# details.
#
# You can be released from the requirements of the license by obtaining a commercial license. Such a license is
# mandatory as soon as you develop commercial activities involving pySX127x without disclosing the source code of your
# own applications, or shipping pySX127x with a closed source product.
#
# You should have received a copy of the GNU General Public License along with pySX127.  If not, see
# <http://www.gnu.org/licenses/>.


import sys
from socket import socket
from time import sleep
from SX127x.LoRa import *
from SX127x.board_config import BOARD

BOARD.setup()

class LoRaSocket(LoRa):

    def __init__(self, verbose=False):
        super(LoRaBeacon, self).__init__(verbose)
        self.set_mode(MODE.SLEEP)
        self.set_dio_mapping([0] * 6)
        
	def create_socket(self):
		host = "127.0.0.1"
		port = 20000
		 
		mySocket = socket.socket()
		mySocket.bind((host,port))
		 
		mySocket.listen(1)
		print("Waiting for socket connection")
		
		conn, addr = mySocket.accept()
		print ("Connection from: " + str(addr))        

    def on_rx_done(self):
        payload = self.read_payload(nocheck=True) #get payload from lora fifo
        print(payload)
        print(bytes(payload))
        conn.send(bytes(payload)) #send to opendenp3 via socket conn
        self.set_mode(MODE.SLEEP)
        self.reset_ptr_rx()
        BOARD.led_off()
        self.set_mode(MODE.RXCONT)


    def on_tx_done(self):
		self.set_dio_mapping([0] * 6) #DIO0 set for rxdone    
        self.clear_irq_flags(TxDone=1) # clear txdone IRQ flag
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
        self.set_max_payload_length(128)
        self.create_socket()
        
		while True:
				data = conn.recv(128).decode()
				if not data:
						break
				print ("from connected  user: " + str(data))
				 
				data = str(data).upper()
				print ("sending: " + str(data))
				


if __name__ == '__main__':
	
	lora = LoRaBeacon(verbose=True)

	lora.set_pa_config(pa_select=1)

	print(lora)
	#assert(lora.get_lna()['lna_gain'] == GAIN.NOT_USED)
	assert(lora.get_agc_auto_on() == 1)

	print("LoRa config:")
	print("")
	try: input("Press enter to start...")
	except: pass

	try:
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
