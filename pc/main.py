import argparse
import json
import os
import queue
import socket
import sys
import threading
import time
import wave
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np
import requests
import whisper

SAMPLE_RATE = 16000
MSG_AUDIO, MSG_END, MSG_TEXT = 0x01, 0x02, 0x03
PING_TYPE = 0x04              # keepalive probe (ESP32 silently ignores unknown types)
PING_INTERVAL = 10.0          # routine ping / reconnect sweep (s)
NODE_LINGER = 30.0            # drop nodes silent on the beacon for this long (s)
DISCOVERY_PORT = 8899
RECV_TIMEOUT = 0.2
JOBS = queue.Queue()  # shared receive queue: (node, audio_bytes)
LOG = deque(maxlen=200)  # chat stream shown in the dashboard
WEB_PORT = 8000


def encode_frame(msg_type, payload=b""):
    return bytes([msg_type]) + len(payload).to_bytes(4, "big") + payload


class FrameParser:
    def __init__(self):
        self.buf = bytearray()

    def feed(self, data):
        self.buf += data
        frames = []
        while len(self.buf) >= 5:
            n = int.from_bytes(self.buf[1:5], "big")
            if len(self.buf) < 5 + n:
                break
            frames.append((self.buf[0], bytes(self.buf[5 : 5 + n])))
            del self.buf[: 5 + n]
        return frames


class Node:
    """One ESP32 node: its own socket, parser, send queue and receive audio buffer."""

    def __init__(self, ip, port):
        self.ip = ip
        self.port = port
        self.sock = None
        self.parser = FrameParser()
        self.audio = bytearray()          # receive accumulation for the current utterance
        self.audio_lock = threading.Lock()
        self.send_q = queue.Queue()       # send queue of (msg_type, payload) or None
        self.send_lock = threading.Lock()
        self.last_seen = time.time()      # last UDP beacon
        self.connected = False
        self.started = False              # sender/receiver threads launched yet?
        self.lights_on = False            # mirror of the LED state (kept on both sides)

    def _set_keepalive(self, s):
        # OS-level keepalive = the routine ping; dead peers surface as OSError fast
        s.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
        if hasattr(socket, "TCP_KEEPIDLE"):
            s.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPIDLE, int(PING_INTERVAL))
            s.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPINTVL, int(PING_INTERVAL))
            s.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPCNT, 3)

    def connect(self):
        with self.send_lock:
            if self.connected:
                return True
            try:
                s = socket.create_connection((self.ip, self.port), timeout=2)
            except OSError as e:
                print(f"[!] Could not connect to {self.ip}: {e}")
                return False
            self._set_keepalive(s)
            s.settimeout(RECV_TIMEOUT)
            self.sock = s
            self.connected = True
            print(f"[+] Connected to node {self.ip}")
            if not self.started:
                self.started = True
                threading.Thread(target=receiver_loop, args=(self,), daemon=True).start()
                threading.Thread(target=sender_loop, args=(self,), daemon=True).start()
            return True

    def close(self):
        with self.send_lock:
            self.connected = False
            if self.sock:
                try:
                    self.sock.close()
                except OSError:
                    pass
            self.sock = None


def receiver_loop(node):
    while True:
        try:
            data = node.sock.recv(4096)
        except socket.timeout:
            continue
        except OSError:
            break
        if not data:
            break
        for t, payload in node.parser.feed(data):
            if t == MSG_AUDIO:
                with node.audio_lock:
                    node.audio.extend(payload)
            elif t == MSG_END:
                with node.audio_lock:
                    audio = bytes(node.audio)
                    node.audio.clear()
                if audio:
                    JOBS.put((node, audio))
    print(f"[!] Lost connection to {node.ip} - will reconnect on next ping.")
    node.close()


def sender_loop(node):
    while True:
        item = node.send_q.get()
        if item is None:
            return
        msg_type, payload = item
        with node.send_lock:
            if node.connected and node.sock:
                try:
                    node.sock.sendall(encode_frame(msg_type, payload))
                except OSError:
                    pass


def worker_loop(jobs, args, model):
    while True:
        node, audio = jobs.get()
        if audio is None:
            return

        if args.debug:
            with wave.open("debug_audio.wav", "wb") as wf:
                wf.setnchannels(1)
                wf.setsampwidth(2)
                wf.setframerate(SAMPLE_RATE)
                wf.writeframes(audio)

        if len(audio) % 2 != 0:
            audio = audio[:-1]  # drop incomplete trailing byte for int16 pairing

        np_audio = np.frombuffer(audio, dtype=np.int16).astype(np.float32) / 32768.0
        print(f"\n[+] Transcribing {node.ip}...")
        text = model.transcribe(np_audio)["text"].strip()
        print(f"Heard: '{text}'")

        if not text:
            payload = json.dumps(
                {"action": "text", "value": "I didn't quite catch that."}
            ).encode("utf-8")
        else:
            schema = {
                "type": "object",
                "properties": {
                    "action": {"type": "string", "enum": ["lights", "text"]},
                    "value": {"type": "string"},
                },
                "required": ["action", "value"],
            }
            res = requests.post(
                args.ollama_url,
                json={
                    "model": args.ollama_model,
                    "prompt": text,
                    "system": "You are a smart home agent. If the user asks to turn lights on/off, output action 'lights' with value '1' for on or '0' for off. For anything else, output action 'text' with your short conversational reply in the value field. Keep answers extremely short, under 2 sentences, no markdown.",
                    "format": schema,
                    "stream": False,
                    "think": False,
                },
            )
            payload = res.json()["response"].strip().encode("utf-8")

        print(f"Reply: {payload.decode(errors='replace')}")
        LOG.append(
            {
                "t": time.strftime("%H:%M:%S"),
                "node": node.ip,
                "heard": text,
                "reply": payload.decode(errors="replace"),
            }
        )
        node.send_q.put((MSG_TEXT, payload))


def discovery_loop(nodes, nodes_lock, port):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    s.bind(("", DISCOVERY_PORT))
    s.settimeout(1.0)
    while True:
        try:
            data, _ = s.recvfrom(256)
        except socket.timeout:
            continue
        except OSError:
            break
        txt = data.decode(errors="ignore").strip()
        if not txt.startswith("VOCEDGE:"):
            continue
        ip = txt.split(":", 1)[1].strip()
        with nodes_lock:
            node = nodes.get(ip)
            if node is None:
                node = Node(ip, port)
                nodes[ip] = node
                print(f"\n[+] New node discovered at {ip}")
            node.last_seen = time.time()
            if not node.connected:
                node.connect()  # beacon from a known node = (re)connect now


def monitor_loop(nodes, nodes_lock):
    while True:
        time.sleep(PING_INTERVAL)
        with nodes_lock:
            now = time.time()
            for ip in list(nodes):
                node = nodes[ip]
                if now - node.last_seen > NODE_LINGER:
                    print(f"[-] Dropping stale node {ip}")
                    node.send_q.put(None)
                    node.close()
                    del nodes[ip]
                    continue
                if not node.connected:
                    node.connect()  # node that used to be connected is gone -> reconnect
                    continue
                # routine ping to verify the link is still alive
                node.send_q.put((PING_TYPE, b""))


def web_server(nodes, nodes_lock, port):
    web_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "web")

    class Handler(BaseHTTPRequestHandler):
        def _send(self, code, body, ctype="application/json"):
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            try:
                if self.path in ("/", "/index.html"):
                    with open(os.path.join(web_dir, "index.html"), "rb") as f:
                        self._send(200, f.read(), "text/html; charset=utf-8")
                elif self.path == "/api/status":
                    with nodes_lock:
                        nodes_out = [
                            {
                                "ip": n.ip,
                                "connected": n.connected,
                                "lights_on": n.lights_on,
                            }
                            for n in nodes.values()
                        ]
                    out = {
                        "nodes": nodes_out,
                        "log": list(LOG),
                        "home": {"lights": [n["lights_on"] for n in nodes_out]},
                    }
                    self._send(200, json.dumps(out).encode())
                else:
                    self._send(404, b'{"error":"not found"}')
            except OSError as e:
                self._send(500, json.dumps({"error": str(e)}).encode())

        def log_message(self, fmt, *args):
            pass  # keep the console clean

    srv = ThreadingHTTPServer(("0.0.0.0", port), Handler)
    srv.serve_forever()


def main():
    ap = argparse.ArgumentParser(description="Multi-node voice assistant host for VocEdge ESP32")
    ap.add_argument("--ip", default="", help="seed node IPs to connect to (comma-separated)")
    ap.add_argument("--port", type=int, default=8888)
    ap.add_argument("--model", default="small.en")
    ap.add_argument("--ollama-url", default="http://localhost:11434/api/generate")
    ap.add_argument("--ollama-model", default="qwen3.5:4b")
    ap.add_argument("--debug", action="store_true", help="save debug_audio.wav per utterance")
    ap.add_argument("--web-port", type=int, default=WEB_PORT, help="dashboard HTTP port")
    args = ap.parse_args()

    print("Loading whisper...")
    model = whisper.load_model(args.model, device="cpu")
    print("Preloading Ollama model...")
    requests.post(
        args.ollama_url,
        json={"model": args.ollama_model, "prompt": "", "stream": False,
              "think": False, "keep_alive": -1},
    )
    print("Models ready.")

    nodes = {}
    nodes_lock = threading.Lock()

    for ip in [x.strip() for x in args.ip.split(",") if x.strip()]:
        with nodes_lock:
            node = Node(ip, args.port)
            nodes[ip] = node
            node.connect()

    threading.Thread(target=discovery_loop, args=(nodes, nodes_lock, args.port), daemon=True).start()
    threading.Thread(target=monitor_loop, args=(nodes, nodes_lock), daemon=True).start()
    threading.Thread(target=web_server, args=(nodes, nodes_lock, args.web_port), daemon=True).start()

    workers = max(1, min(os.cpu_count() or 2, 4))
    for _ in range(workers):
        threading.Thread(target=worker_loop, args=(JOBS, args, model), daemon=True).start()

    print(f"\n[+] Listening for ESP32 nodes (UDP beacon on {DISCOVERY_PORT}).")
    print("    Type text and press Enter to broadcast to all nodes, or 'exit' to quit.")

    try:
        while True:
            line = input()
            if line.lower() == "exit":
                break
            with nodes_lock:
                targets = [n for n in nodes.values() if n.connected]
            if not targets:
                print("[!] No nodes connected.")
                continue
            payload = json.dumps({"action": "text", "value": line}).encode("utf-8")
            for n in targets:
                n.send_q.put((MSG_TEXT, payload))
    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        with nodes_lock:
            for n in nodes.values():
                n.send_q.put(None)
                n.close()


if __name__ == "__main__":
    main()