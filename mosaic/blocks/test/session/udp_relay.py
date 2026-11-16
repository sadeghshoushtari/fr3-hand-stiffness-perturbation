"""Forwards UDP datagrams (standard library only): python udp_relay.py <listen_host:port> <target_host:port>

The session test uses it to connect the pipeline runner (Windows) and the perturbation program (WSL), whose
loopback interfaces are separate.
"""
import socket
import sys

lh, lp = sys.argv[1].rsplit(":", 1)
th, tp = sys.argv[2].rsplit(":", 1)
rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
rx.bind((lh, int(lp)))
tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
while True:
    tx.sendto(rx.recv(65536), (th, int(tp)))
