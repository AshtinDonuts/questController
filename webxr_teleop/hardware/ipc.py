"""Bounded local messages. One datagram is one message; no command backlog."""
import json
import socket


def send(sock, obj):
    try:
        sock.send(json.dumps(obj, allow_nan=False).encode())
        return True
    except OSError:
        return False


def receive(sock):
    messages = []
    for _ in range(64):
        try:
            raw = sock.recv(65536)
            if not raw:
                raise ConnectionError('Peer closed')
            messages.append(json.loads(raw))
        except BlockingIOError:
            break
    return messages


def pair():
    a, b = socket.socketpair(socket.AF_UNIX, socket.SOCK_DGRAM)
    for s in (a, b):
        s.setblocking(False)
    return a, b
