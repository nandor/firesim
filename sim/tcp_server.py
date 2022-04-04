
import socket
import airsim
import time
import numpy as np
import os
import tempfile
import pprint
import cv2
import sys

import threading
import traceback

CS_GRANT_TOKEN = 0x80 
CS_REQ_CYCLES = 0x81 
CS_RSP_CYCLES = 0x82 
CS_DEFINE_STEP = 0x83
CS_REQ_WAYPOINT = 0x01
CS_RSP_WAYPOINT = 0x02
CS_SEND_IMU = 0x03

#HOST = "127.0.0.1"  # Standard loopback interface address (localhost)
HOST = "192.168.0.47"  # Private aws IP

#PORT = 65432  # Port to listen on (non-privileged ports are > 1023)
SYNC_PORT = 10001  # Port to listen on (non-privileged ports are > 1023)
DATA_PORT = 60002  # Port to listen on (non-privileged ports are > 1023)


class CoSimPacket:
    def __init__(self):
        self.cmd = None
        self.num_bytes = None
        self.data = None

    def __str__(self):
        return "[cmd: 0x{:02X}, num_bytes: {:04d}, data: {}]".format(self.cmd, self.num_bytes, self.data)

    def init(self, cmd, num_bytes, data):
        self.cmd = cmd
        self.num_bytes = num_bytes
        self.data = data

    def decode(self, buffer):
        self.cmd = int.from_bytes(buffer[0:4], "little", signed="False")
        self.num_bytes = int.from_bytes(buffer[4:8], "little", signed="False")
        data_array = [] 
        for i in range(self.num_bytes // 4):
            data_array.append(int.from_bytes(buffer[4 * i + 8 : 4 * i + 12], "little", signed="False"))
        self.data = data_array

    def encode(self):
        buffer = self.cmd.to_bytes(4, 'little') + self.num_bytes.to_bytes(4, 'little')
        if self.num_bytes > 0:
            for datum in self.data:
                buffer = buffer + datum.to_bytes(4, 'little') 
        return buffer

class SocketThread (threading.Thread):
   def __init__(self, syn):
      threading.Thread.__init__(self)
      self.syn = syn

   def read_word(self):
        data = self.sync_conn.recv(1)
        datum = None
        if data:
            for i in range(3):
               while True:
                   datum = self.sync_conn.recv(1)
                   if datum:
                       data = data + datum
                       break
            return data
        return None
   def run(self):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.bind((self.syn.sync_host, self.syn.sync_port))
            s.listen()
            self.sync_conn, self.sync_addr = s.accept()
            count = 0
            with self.sync_conn:
                print(f"sync_connected by {self.sync_addr}")
                txfile = open("/home/centos/synchronizer_txdump.txt", "w")
                rxfile = open("/home/centos/synchronizer_rxdump.txt", "w")
                while True:
                    self.sync_conn.settimeout(0.1)
                    count = count + 1
                    if count > 100:
                        print("Synchronizer heartbeat")
                        count = 0
                    try:
                        # cmd_data = self.sync_conn.recv(4)
                        cmd_data = self.read_word()
                        if cmd_data:
                            # self.sync_conn.setblocking(1)
                            queue = None
                            cmd = int.from_bytes(cmd_data, "little", signed="False")
                            rxfile.write(f"{cmd}\n")
                            print("Got command: 0x{:02X}".format(cmd))
                            if cmd > 0x80:
                                queue = self.syn.sync_rxqueue
                            else:
                                queue = self.syn.data_rxqueue
                            num_bytes = None
                            print("Getting Num Bytes")
                            while True:
                                # num_bytes_data = self.sync_conn.recv(4)
                                num_bytes_data = self.read_word()
                                if num_bytes_data:
                                    num_bytes = int.from_bytes(num_bytes_data, "little", signed="False")
                                    break
                            rxfile.write(f"{num_bytes}\n")
                            data = []
                            for i in range(num_bytes//4):
                                print(f"Getting Data {i}")
                                while True:
                                    if num_bytes:
                                        # datum = self.sync_conn.recv(4)
                                        datum = self.read_word()
                                        data.append(int.from_bytes(datum, "little", signed="False"))
                                        rxfile.write(f"{data[i]}\n")
                                        break
                            packet = CoSimPacket()
                            packet.init(cmd, num_bytes, data)
                            queue.append(packet)
                    except Exception as e:
                        # print(f"recv error: {e}")
                        # traceback.print_exc()
                        pass
                    if len(self.syn.txqueue) > 0:
                        packet = self.syn.txqueue.pop(0)
                        self.sync_conn.sendall(packet.encode())
                        txfile.write(f"{packet.cmd}\n")
                        txfile.write(f"{packet.num_bytes}\n")
                        if packet.num_bytes > 0:
                            for datum in packet.data:
                                txfile.write(f"{datum}\n")



class Synchronizer:
    def __init__(self, host, sync_port, data_port, firesim_step = 1000000):
        self.sync_host = host
        self.data_host = host
        self.sync_port = sync_port
        self.data_port = data_port
        self.firesim_step = firesim_step
        self.packet = CoSimPacket()
        self.txqueue = []
        self.data_rxqueue = []
        self.sync_rxqueue = []

        print("Starting test code")
        if len(sys.argv) > 1:
            airsim_ip = sys.argv[1]
            print("Will connect to {}".format(airsim_ip))
        else:
            airsim_ip = "54.158.237.240"

        # connect to the AirSim simulator
        print("Connecting to server")
        self.client = airsim.MultirotorClient(ip=airsim_ip)
        self.client.confirmConnection()
        self.client.enableApiControl(True)
        print("Connected to server")
        state = self.client.getMultirotorState()

        # print("Taking off...")
        # client.armDisarm(True)
        # client.takeoffAsync().join()

        print("pausing simulator...")
        self.client.simPause(True) 
    
    def run(self):
        socket_thread = SocketThread(self)
        socket_thread.start()
        self.send_firesim_step()
        i = 333
        count = 0
        while True:
            print("Stepping airsim")
            self.client.simContinueForFrames(1)
            print(f"Granting fsim token: {count}")
            count = count + 1
            self.grant_firesim_token()
            print("Polling firesim cycles")
            while True:
                if self.get_firesim_cycles() == 0:
                    break
                time.sleep(0.01)
            while True:
                if (self.client.simIsPause()):
                    break
            print(f"data_rxqueue: {self.data_rxqueue}")
            print("Prossing firesim data data")
            while len(self.data_rxqueue) > 0:
                self.process_fsim_data_packet()
            # packet = CoSimPacket()
            # packet.init(0x02, 4, [i])
            # self.txqueue.append(packet)
        socket_thread.join()
        #with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        #    s.bind((self.sync_host, self.sync_port))
        #    s.listen()
        #    self.sync_conn, self.sync_addr = s.accept()
        #    with self.sync_conn:
        #        print(f"sync_connected by {self.sync_addr}")
        #        # with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s_d:
        #        #     s_d.bind((self.data_host, self.data_port))
        #        #     s_d.listen()
        #        #     self.data_conn, self.data_addr = s_d.accept()
        #        #     with self.data_conn:
        #        #         print(f"data_connected by {self.data_addr}")
        #        self.send_firesim_step()
        #        while True:
        #            time.sleep(1)
        #            self.grant_firesim_token()

        #            while True:
        #                if self.get_firesim_cycles() == 0:
        #                    break
        #                # TODO FIND WORKAROUND TO THROTTLE
        #                time.sleep(0.1)
    
    def send_firesim_step(self):
        packet = CoSimPacket()
        packet.init(CS_DEFINE_STEP, 4, [self.firesim_step])
        print(f"Enqueuing step size: {packet}")
        self.txqueue.append(packet)
        #self.sync_conn.sendall(self.packet.encode())

    def grant_firesim_token(self):
        print("Enqueuing new token")
        packet = CoSimPacket()
        packet.init(CS_GRANT_TOKEN, 0, None)
        self.txqueue.append(packet)
        #self.sync_conn.sendall(self.packet.encode())
    
    def get_firesim_cycles(self):
        packet = CoSimPacket()
        packet.init(CS_REQ_CYCLES, 0, None)
        print(f"Enqueing cycle request: {packet}")
        self.txqueue.append(packet)
        #self.sync_conn.sendall(packet.encode())
        #while True:
        #    data = self.sync_conn.recv(1024)
        #    if data:
        #        break
        #self.packet.decode(data)
        #print(f"Got data: {self.packet}")
        #return self.packet.data[0]
        while len(self.sync_rxqueue) == 0:
            pass
        response = self.sync_rxqueue.pop(0)
        return response.data[0]
    
    def send_test_firesim_data_packet(self):
        packet = CoSimPacket()

    def process_fsim_data_packet(self):
        packet = self.data_rxqueue.pop(0)
        print(f"Dequeued data packet: {packet}")
        



if __name__ == "__main__":

    sync = Synchronizer(HOST, SYNC_PORT, DATA_PORT)
    sync.run()

        

