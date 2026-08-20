import argparse
import queue
import socket
import threading
import wave
import json

import numpy as np
import requests
import whisper

SAMPLE_RATE = 16000
MSG_AUDIO, MSG_END, MSG_TEXT = 0x01, 0x02, 0x03


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


def receiver(sock, parser, audio_buffer, lock, jobs):
    while True:
        try:
            data = sock.recv(4096)
        except socket.timeout:
            continue
        if not data:
            print("\n[!] Connection closed by ESP32.")
            return
        for t, payload in parser.feed(data):
            if t == MSG_AUDIO:
                with lock:
                    audio_buffer.extend(payload)
            elif t == MSG_END:
                with lock:
                    audio = bytes(audio_buffer)
                    audio_buffer.clear()
                if audio:
                    jobs.put(audio)


def worker(jobs, sock, send_lock, args, model):
    while True:
        audio = jobs.get()
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
        print("\n[+] Transcribing...")
        text = model.transcribe(np_audio)["text"].strip()
        print(f"Heard: '{text}'")
        
        # 1. Update the fallback to send valid JSON!
        if not text:
            fallback_json = json.dumps({"action": "text", "value": "I didn't quite catch that."})
            with send_lock:
                sock.sendall(encode_frame(MSG_TEXT, fallback_json.encode("utf-8")))
            continue

        # 2. Define the strict JSON structure
        schema = {
            "type": "object",
            "properties": {
                "action": {"type": "string", "enum": ["lights", "text"]},
                "value": {"type": "string"}
            },
            "required": ["action", "value"]
        }

        # 3. Supercharged System Prompt
        res = requests.post(
            args.ollama_url,
            json={
                "model": args.ollama_model,
                "prompt": text,
                "system": "You are a smart home agent. If the user says anything relating to 'light', 'lights', 'bright' (even just broken single words like 'light'), you MUST output action: 'lights' and value: '1' (for on) or '0' (for off). For any other request, output action: 'text' and put your short conversational reply in the value field.",
                "format": schema, # <--- Forces Ollama to use the schema
                "stream": False,
                "think": False,
            },
        )
        
        # Ollama will now return a perfect JSON string
        reply_json = res.json()["response"].strip()
        print(f"Reply: {reply_json}")
        
        with send_lock:
            sock.sendall(encode_frame(MSG_TEXT, reply_json.encode("utf-8")))


def main():
    ap = argparse.ArgumentParser(description="Voice assistant host for VocEdge ESP32")
    ap.add_argument("--ip", default="192.168.68.161")
    ap.add_argument("--port", type=int, default=8888)
    ap.add_argument("--model", default="small.en")
    ap.add_argument("--ollama-url", default="http://localhost:11434/api/generate")
    ap.add_argument("--ollama-model", default="qwen3.5:4b")
    ap.add_argument("--debug", action="store_true", help="save debug_audio.wav per utterance")
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

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((args.ip, args.port))
    sock.settimeout(0.2)
    print(f"Connected to {args.ip}:{args.port}")

    audio_buffer = bytearray()
    lock = threading.Lock()
    send_lock = threading.Lock()
    jobs = queue.Queue()
    parser = FrameParser()

    threading.Thread(target=worker, args=(jobs, sock, send_lock, args, model), daemon=True).start()
    threading.Thread(target=receiver, args=(sock, parser, audio_buffer, lock, jobs), daemon=True).start()

    try:
        while True:
            line = input("")
            if line.lower() == "exit":
                break
            with send_lock:
                sock.sendall(encode_frame(MSG_TEXT, json.dumps({"action": "text", "value": line}).encode("utf-8")))
    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        sock.close()


if __name__ == "__main__":
    main()
