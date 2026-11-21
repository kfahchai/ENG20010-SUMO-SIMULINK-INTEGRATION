
import socket
import time

HOST = "127.0.0.1"   # Listen on all interfaces
PORT = 55000

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind((HOST, PORT))
    s.listen(1)
    print(f"[SERVER] Listening on {HOST}:{PORT} ...")

    conn, addr = s.accept()
    print(f"[SERVER] Client connected from {addr}")

    with conn:
        for i in range(10):
            msg = f"Hello {i}\n"
            conn.sendall(msg.encode("utf-8"))
            print(f"[SERVER] Sent: {msg.strip()}")
            time.sleep(1)

    print("[SERVER] Client disconnected. Server exiting.")
