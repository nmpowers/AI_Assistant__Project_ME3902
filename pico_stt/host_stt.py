import re, time, struct, numpy as np, serial
from difflib import SequenceMatcher
from faster_whisper import WhisperModel

import os, json, subprocess
from openai import OpenAI

# ---------- OpenAI client ----------
_oai = None
def get_client():
    global _oai
    if _oai is None:
        _oai = OpenAI()  # reads OPENAI_API_KEY from env
    return _oai

# ---------- Text generation (Responses API) ----------
# System prompt keeps answers tight and references sensor context if you later add it.
SYS_PROMPT = (
    "You are a concise, factual voice assistant running on a lab bench. "
    "Prefer short, spoken-style answers. If the user asks for measurements, "
    "use the provided 'context' JSON if present (e.g., temperature, proximity)."
)

def ask_chatgpt(prompt_text: str, context: dict | None = None,
                model: str = "gpt-4o-mini",  # fast & cheap; bump later if desired
                max_output_tokens: int = 300) -> str:
    """
    Send the user's question + optional context to the OpenAI Responses API and return text.
    """
    client = get_client()
    # Build a simple input for Responses API: system + user with optional context
    messages = []
    messages.append({"role": "system", "content": SYS_PROMPT})
    if context:
        messages.append({"role": "user", "content": f"Context JSON:\n```json\n{json.dumps(context)}\n```"})
    messages.append({"role": "user", "content": prompt_text})

    resp = client.responses.create(
        model=model,
        input=messages,
        max_output_tokens=max_output_tokens,
    )
    # Extract the first text block
    for item in resp.output or []:
        if item.type == "message":
            for c in item.content or []:
                if c.type == "output_text":
                    return c.text.strip()
    # fallback for older shapes
    try:
        return (resp.output_text or "").strip()
    except Exception:
        return ""  # handle gracefully

def speak_mac_say(text: str, voice: str = "Samantha"):
    try:
        subprocess.run(["say", "-v", voice, text], check=False)
    except Exception:
        pass

# ---- I/O config ----
SERIAL_PORT = "/dev/cu.usbmodem2101"   # <-- put YOUR exact device path
BAUD = 115200
RATE = 16000
FRAMESZ = 1024
MAGIC = b"FRME"

# ---- Wake word config ----
WAKE_PHRASES = [
    "hey pico", "hey friend", "ok pico", "okay pico",
    "hey pico assistant", "pico", "friend",
    # Common Whisper mis-transcriptions for "Hey Pico":
    "hey eco", "hippie co", "hey peacock", "a pico", 
    "hey pickle", "people", "hey be go", "happy go",
    "hey", "hi pico", "eco", "peacock", "pickle", "be go", "people"
]
WAKE_TIMEOUT_S = 12      # active window after wake to grab the question
WAKE_MIN_CONF = 0.60    # fuzzy prefix similarity threshold (0..1)
DEBUG_VAD = True

# ---- Model choices ----
MAIN_MODEL_NAME = "base.en"   # good accuracy on CPU int8
WAKE_MODEL_NAME = "tiny.en"    # faster wake model (optional)
USE_DUAL_MODELS = True          # set False to use one model for everything
COMPUTE = "int8"                # CPU-friendly

# ---------- Serial framing ----------
import json

SENSOR = { "tC": None, "soil_raw": None, "soil_pct": None, "motor_pct": None, "ts": 0, "wake_event" : False }
_buf = bytearray()

# --- ADD THIS FLAG ---
PROXIMITY_TRIGGERED = False

def _min_pos(*vals):
    vals = [v for v in vals if v >= 0]
    return min(vals) if vals else -1

def send_command(s, cmd_dict):
    """Sends a JSON command to the Pico using the CMND protocol."""
    try:
        payload = json.dumps(cmd_dict).encode('utf-8')
        # Protocol: b'CMND' + 2-byte-length + Payload
        header = b'CMND' + struct.pack('<H', len(payload))
        s.write(header + payload)
        # We don't flush immediately if we want to prioritize reading audio, 
        # but for an LED we want it instant:
        s.flush() 
    except Exception as e:
        print(f"Failed to send command: {e}")

def aggressive_flush(s):
    """
    Forcefully clears the serial buffer.
    Mac drivers sometimes ignore reset_input_buffer() if data is streaming fast.
    """
    # 1. Tell OS to drop buffer
    s.reset_input_buffer()
    
    # 2. Manually read any stragglers until empty
    # We give it a tiny timeout to ensure we catch bytes currently on the wire
    t_end = time.time() + 0.05 
    while time.time() < t_end:
        if s.in_waiting > 0:
            s.read(s.in_waiting)

def read_audio_frame_or_update_sensor(s):
    """Return PCM payload when a FRME is parsed; STAT updates SENSOR and continues."""
    global _buf, SENSOR
    while True:
        chunk = s.read(4096)
        if chunk: _buf += chunk
        # find earliest header among FRME/STAT
        iF = _buf.find(b"FRME")
        iS = _buf.find(b"STAT")
        i  = _min_pos(iF, iS)
        if i < 0:
            _buf = _buf[-3:]  # keep tail for split headers
            continue

        # handle STAT first if it occurs earlier
        if iS >= 0 and (iS < iF or iF == -1):
            if len(_buf) < iS + 6:  # need header+len
                continue
            ln = struct.unpack_from("<H", _buf, iS+4)[0]
            end = iS + 6 + ln
            if len(_buf) < end:
                continue
            payload = bytes(memoryview(_buf)[iS+6:end])
            del _buf[:end]
            try:
                data = json.loads(payload.decode("utf-8"))
                SENSOR.update(data)
                SENSOR["ts"] = time.time()
            except Exception:
                pass
            # loop to find next item
            continue

        # otherwise parse FRME
        if len(_buf) < iF + 8:
            continue
        ln, _seq = struct.unpack_from("<HH", _buf, iF+4)
        end = iF + 8 + ln
        if len(_buf) < end:
            continue
        pcm = bytes(memoryview(_buf)[iF+8:end])
        del _buf[:end]
        return pcm

def read_frame_f32(s):
    pay = read_audio_frame_or_update_sensor(s) 
    x = np.frombuffer(pay, dtype="<i2").astype(np.float32)/32768.0
    return x - float(np.mean(x))

def ms_to_frames(ms, rate, framesz):
    return max(1, int(round((ms/1000.0) * (rate/framesz))))

# ---------- Collectors ----------
def collect_utterance_longform(
    s, rate=16000, framesz=1024,
    calib_ms=400, start_min_ms=100, pre_roll_ms=180,
    min_utt_ms=500, end_silence_ms=500, max_pause_ms=400,
    max_total_ms=7000, floor=0.003, thresh_boost=1.2
):
    global PROXIMITY_TRIGGERED 
    # --- CONVERSION TO FRAMES ---
    n_cal = ms_to_frames(calib_ms, rate, framesz)
    need_start = ms_to_frames(start_min_ms, rate, framesz)
    pre_roll = ms_to_frames(pre_roll_ms, rate, framesz)
    min_utt = ms_to_frames(min_utt_ms, rate, framesz)
    end_sil = ms_to_frames(end_silence_ms, rate, framesz)
    max_pause = ms_to_frames(max_pause_ms, rate, framesz)
    max_total = ms_to_frames(max_total_ms, rate, framesz)

    # --- CALIBRATION ---
    # We read n_cal frames to sense the room noise
    amb = []
    for _ in range(n_cal):
        amb.append(read_frame_f32(s))
    
    # Calculate average noise (RMS)
    amb_vals = [float(np.sqrt((a*a).mean())) for a in amb]
    amb_rms = float(np.mean(amb_vals))
    
    # SAFETY CAP: 
    # 1. Floor: Don't let it be too sensitive (0.003)
    # 2. Boost: Multiply noise by 1.5x (was 3.0x, which was too high)
    # 3. Hard Cap: NEVER let threshold go above 0.015, or it goes deaf.
    calc_thresh = amb_rms * thresh_boost
    thresh = max(floor, calc_thresh)
    if thresh > 0.015: 
        thresh = 0.015 # Force it down if it calibrated during a noise

    preroll_buf, buf = [], []
    speech_run = silent_run = total = 0
    started = False

    # --- LISTENING LOOP ---
    while total < max_total:
        # --- PROXIMITY CHECK ---
        if SENSOR.get("wake_event") is True:
            print("\n[Proximity] Hand Detected! Force Waking...")
            SENSOR["wake_event"] = False  # Clear the sensor data
            PROXIMITY_TRIGGERED = True    # <--- SET THE FLAG
            
            # Return dummy audio to break the loop immediately
            return np.array([0.0], dtype=np.float32)
        
        x = read_frame_f32(s)
        rms = float(np.sqrt((x*x).mean()))
        
        # Debug print only occasionally
        # if DEBUG_VAD and (total % 10 == 0):
        #      print(f"[VAD] started={started} rms={rms:.4f} thresh={thresh:.4f} "
        #            f"run={speech_run if not started else -silent_run}")
        total += 1

        if not started:
            preroll_buf.append(x)
            if len(preroll_buf) > pre_roll:
                preroll_buf.pop(0)
            
            if rms >= thresh:
                speech_run += 1
                if speech_run >= need_start:
                    started = True
                    buf.extend(preroll_buf)
                    preroll_buf.clear()
                    buf.append(x)
                    silent_run = 0
            else:
                speech_run = 0
            continue

        # If we are here, we have STARTED recording
        buf.append(x)
        if rms < thresh:
            silent_run += 1
            if silent_run < max_pause:
                continue
            # End of speech detected?
            if len(buf) >= min_utt and silent_run >= end_sil:
                break
        else:
            silent_run = 0

    return np.concatenate(buf) if buf else np.concatenate(amb)

def collect_wake_utterance(s, rate=16000, framesz=1024):
    """Short, snappy collector tuned for hotword phrases."""
    return collect_utterance_longform(
        s, rate=rate, framesz=framesz,
        calib_ms=300, 
        start_min_ms=50,      # Was 80. 
        pre_roll_ms=200,      # Was 120. 
        min_utt_ms=150,       # Was 250. 
        end_silence_ms=400,   # Was 350. 
        max_pause_ms=220,
        max_total_ms=4000, 
        floor=0.003,          # Was 0.006.
        thresh_boost=1.5      # Was 3.0. 
    )

# ---------- STT wrappers ----------
def transcribe(model, audio):
    segs, _ = model.transcribe(
        audio,
        language="en",
        beam_size=5,
        vad_filter=True,
        condition_on_previous_text=False,
        temperature=0.0,
        initial_prompt="Hey Pico, what is the temperature?"
    )
    return "".join(s.text for s in segs).strip()

# ---------- Wake matching ----------
_punct_re = re.compile(r"[^a-z0-9\s]+")

def _normalize(t: str) -> str:
    return _punct_re.sub("", t.lower()).strip()

def _prefix_sim(a: str, b: str) -> float:
    # compare phrase b vs the first len(b)+2 chars of a
    a0 = a[:max(len(b)+2, len(b))]
    return SequenceMatcher(None, a0, b).ratio()

def match_wake(text: str):
    """Return (matched_phrase, remainder) or (None, text)"""
    if not text:
        return None, text
    n = _normalize(text)
    for phrase in WAKE_PHRASES:
        p = _normalize(phrase)
        if n.startswith(p):
            rem = n[len(p):].lstrip()
            return phrase, rem
        if _prefix_sim(n, p) >= WAKE_MIN_CONF:
            # fuzzy hit near the very beginning
            # cut away roughly the phrase length from original, not normalized
            idx = min(len(phrase)+1, len(text))
            return phrase, text[idx:].lstrip()
    return None, text

# ---------- Main loop ----------
def load_models():
    main = WhisperModel(MAIN_MODEL_NAME, device="cpu", compute_type=COMPUTE)
    if USE_DUAL_MODELS:
        try:
            wake = WhisperModel(WAKE_MODEL_NAME, device="cpu", compute_type=COMPUTE)
        except Exception:
            wake = main
    else:
        wake = main
    return wake, main

def listen_for_question(s, model, rate=RATE, framesz=FRAMESZ, max_chain_s=WAKE_TIMEOUT_S):
    pieces, audios = [], []
    t0 = time.time()
    while True:
        audio = collect_utterance_longform(s, rate=rate, framesz=framesz)
        text  = transcribe(model, audio)
        if text:
            pieces.append(text); audios.append(audio)
            joined = " ".join(pieces).strip()
            if joined.endswith("?") or ("?" in text and len(text) > 2):
                return np.concatenate(audios), joined
        if time.time() - t0 > max_chain_s:
            return (np.concatenate(audios) if audios else audio), (" ".join(pieces).strip() if pieces else text)

if __name__ == "__main__":
    print("Loading models…")
    wake_model, main_model = load_models()
    print(f"Wake model: {WAKE_MODEL_NAME if wake_model is not main_model else MAIN_MODEL_NAME} ({COMPUTE})")
    print(f"Main model: {MAIN_MODEL_NAME} ({COMPUTE})")
    with serial.Serial(SERIAL_PORT, BAUD, timeout=1) as s:
        print('Passive: say "Hey Pico" or "Hey Friend"…')
        while True:
            aggressive_flush(s)

            # 1) Wake phase
            wake_audio = collect_wake_utterance(s, rate=RATE, framesz=FRAMESZ)
            # --- UPDATED LOGIC: CHECK THE BOOLEAN ---
            if PROXIMITY_TRIGGERED:
                print("[Wake] Triggered by Proximity Sensor")
                
                # IMPORTANT: Reset the flag immediately so we don't get stuck in a loop
                PROXIMITY_TRIGGERED = False 
                
                # Fake the match to skip wake word detection
                phrase = "PROXIMITY_OVERRIDE" 
                wake_text = ""
                remainder =""
                
            else:
                # Standard Voice Wake logic
                wake_text  = transcribe(wake_model, wake_audio)
                phrase, remainder = match_wake(wake_text)

            if not phrase:
                continue
            
            print(f"[wake] heard: {wake_text!r}  → matched: {phrase!r}")

            # --- ACTION: TURN LED ON ---
            print("  [LED] ON")
            send_command(s, {"led": 2}) # blink mode

            # 2) If there was extra speech after the wake phrase, keep it; else, listen long-form
            if remainder and len(remainder.split()) >= 2:
                # we already have the start of the question; keep it
                pieces = [remainder]
                audios = [wake_audio]
                t0 = time.time()
                # keep collecting until question end or timeout
                while True:
                    aggressive_flush(s)
                    audio = collect_utterance_longform(s, rate=RATE, framesz=FRAMESZ)
                    text  = transcribe(main_model, audio)
                    if text:
                        pieces.append(text); audios.append(audio)
                        joined = " ".join(pieces).strip()
                        if joined.endswith("?") or ("?" in text and len(text) > 2):
                            full_audio = np.concatenate(audios)
                            full_text  = joined
                            break
                    if time.time() - t0 > WAKE_TIMEOUT_S:
                        full_audio = np.concatenate(audios)
                        full_text  = " ".join(pieces).strip()
                        break
            else:
                # --- FIX 1: FLUSH THE BUFFER HERE ---
                # We matched the wake word, but there was no question yet.
                # The user likely paused. Clear the buffer so we listen to "NOW".
                aggressive_flush(s)
                print("Listening for question...")
                t_start = time.time()
                # no remainder; collect a fresh, long question
                full_audio, full_text = listen_for_question(s, main_model, rate=RATE, framesz=FRAMESZ)
                print(f"  [Timing] Recording finished: {time.time() - t_start:.2f}s")

            print("  [LED] OFF")
            send_command(s, {"led": 0})

            # Strip wake phrase from the very beginning if Whisper re-inserted it
            _, full_text_norm = match_wake(full_text)
            cleaned = full_text_norm if full_text_norm else full_text

            print("Q:", cleaned or "(empty)")

            if cleaned:
                # (Optional) include sensor context when you have it; for now, keep it empty.
                print("T", SENSOR.get("tC"))
                context = {
                    "thermistor_c": SENSOR.get("tC"),
                    "soil_raw": SENSOR.get("soil_raw"),
                    "soil_pct": SENSOR.get("soil_pct"),
                    "timestamp": SENSOR.get("ts"),
                }
                t_oai = time.time()
                answer = ask_chatgpt(cleaned, context=context, model="gpt-4o-mini")
                print(f"  [Timing] OpenAI response: {time.time() - t_oai:.2f}s")
                if not answer:
                    answer = "Sorry, I didn’t get that."
                print("A:", answer)

                speak_mac_say(answer, voice="Samantha")
                
                # ADD THIS: Wait 2 seconds for the audio to finish 
                # before we start listening/calibrating again.
                time.sleep(2.0) 
                
                # Clear out the serial buffer so we don't process stale audio
                s.reset_input_buffer()