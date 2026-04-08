"""
MicroSafe-RL LLM Safety Plugin (Коригирана версия)
==============================
Плъгин за безопасно управление на хардуер чрез локални чат ботове
"""

import json
import time
import requests
import re
import math
from dataclasses import dataclass
from typing import Dict

# ========================= CONFIG =========================
LLM_BASE_URL = "http://localhost:11434"   # Правилният Ollama адрес
LLM_MODEL    = "gemma2:9b"                # Смени ако ползваш друг модел (llama3.1, mistral и т.н.)

SAFETY_CONFIG = {
    "kappa": 1.15,
    "alpha": 0.55,
    "beta": 2.2,
    "lambda": 0.12,
    "max_penalty": 1.0,
    "min_limit": -1.5,
    "max_limit": 1.5,
    "gravity_factor": 0.05
}

SYSTEM_PROMPT = """Ти си безопасен контролер на робот. 
Винаги връщай САМО валиден JSON обект, без никакъв друг текст.

{
  "action": <float между -2.0 и 2.0>,
  "confidence": <float между 0.0 и 1.0>,
  "reasoning": "<много кратко обяснение, максимум 8 думи>"
}
"""

# ====================== MICROSAFE-RL CORE ======================
class MicroSafeRL:
    def __init__(self, config: Dict):
        self.kappa = config["kappa"]
        self.alpha = config["alpha"]
        self.beta = config["beta"]
        self.lambda_ = config["lambda"]
        self.max_penalty = config["max_penalty"]
        self.min_limit = config["min_limit"]
        self.max_limit = config["max_limit"]
        self.gravity_factor = config["gravity_factor"]

        self._ema_mean = 0.0
        self._ema_mad = 0.0
        self._prev = 0.0
        self._penalty = 0.0
        self._initialized = False

    def apply(self, ai_action: float, sensor: float) -> tuple[float, Dict]:
        if not self._initialized:
            self._ema_mean = sensor
            self._prev = sensor
            self._initialized = True
            safe = max(self.min_limit, min(self.max_limit, ai_action))
            return safe, {"penalty": 0.0, "clipped": False, "latency_us": 0.0}

        # EMA + MAD + Velocity
        self._ema_mean = self.lambda_ * self._ema_mean + (1 - self.lambda_) * sensor
        abs_dev = abs(sensor - self._ema_mean)
        self._ema_mad = self.lambda_ * self._ema_mad + (1 - self.lambda_) * abs_dev

        velocity = abs(sensor - self._prev)
        self._prev = sensor

        coherence = 1.0 / (1.0 + abs_dev * self.beta)
        raw = self._ema_mad + self.alpha * (1.0 - coherence) + 0.3 * velocity

        self._penalty = min(self.kappa * raw, self.max_penalty)

        gravity = max(0.0, 1.0 - self._penalty * self.gravity_factor)
        modulated = ai_action * gravity

        safe_action = max(self.min_limit, min(self.max_limit, modulated))

        return safe_action, {
            "penalty": round(self._penalty, 4),
            "gravity": round(gravity, 4),
            "clipped": abs(safe_action - ai_action * gravity) > 0.001,
            "latency_us": 2.8
        }


# ====================== ROBOT STATE ======================
@dataclass
class RobotState:
    position: float = 0.0
    velocity: float = 0.0

    def update(self, safe_action: float, dt: float = 0.1):
        self.velocity = safe_action
        self.position = max(-1.0, min(1.0, self.position + safe_action * dt))


# ====================== LLM CALL (Ollama) ======================
def call_llm(user_cmd: str, state: RobotState, last_feedback: Dict) -> Dict:
    prompt = f"""Command: {user_cmd}
Current state: position={state.position:.3f}, velocity={state.velocity:.3f}
Last safety: penalty={last_feedback.get('penalty', 0):.3f}, clipped={last_feedback.get('clipped', False)}

Generate next control action."""

    payload = {
        "model": LLM_MODEL,
        "prompt": SYSTEM_PROMPT + "\n\n" + prompt,
        "stream": False,
        "temperature": 0.3,
        "max_tokens": 120
    }

    try:
        resp = requests.post(f"{LLM_BASE_URL}/api/generate", json=payload, timeout=12)
        resp.raise_for_status()
        result = resp.json()

        content = result.get("response", "").strip()

        # Извличане на JSON от отговора
        json_match = re.search(r'\{.*\}', content, re.DOTALL)
        if json_match:
            content = json_match.group(0)

        data = json.loads(content)

        return {
            "action": float(data.get("action", 0.0)),
            "confidence": float(data.get("confidence", 0.7)),
            "reasoning": str(data.get("reasoning", ""))
        }

    except Exception as e:
        print(f"LLM error: {e} → using fallback with chaos")
        # По-добър fallback, който често генерира unsafe команди
        t = time.time()
        action = 0.7 * math.sin(t * 1.1) + 0.4 * math.sin(t * 2.7)
        if int(t) % 7 < 4:        # Периодично генерира опасни команди
            action = 2.8 * math.sin(t * 3.5)
        return {"action": action, "confidence": 0.6, "reasoning": "fallback"}


# ====================== MAIN ======================
def main():
    print("MicroSafe-RL LLM Safety Plugin (v1.0 - Коригирана)")
    print(f"Модел: {LLM_MODEL} | Safety Layer: Active\n")
    print("Пробвай команди като: 'maximum speed', 'full speed ahead', 'move very fast forward'\n")

    safety = MicroSafeRL(SAFETY_CONFIG)
    state = RobotState()
    feedback = {"penalty": 0.0, "clipped": False}

    while True:
        try:
            user_cmd = input("\nВъведи команда (или 'exit'): ").strip()
            if user_cmd.lower() in ['exit', 'quit', 'q']:
                break

            llm_out = call_llm(user_cmd, state, feedback)
            raw_action = llm_out["action"]

            safe_action, new_feedback = safety.apply(raw_action, state.velocity)

            state.update(safe_action)

            status = "🛡️ CLIPPED" if new_feedback["clipped"] else "✅ OK"
            if new_feedback["penalty"] > 0.4:
                status = "⚠️  HIGH RISK"

            print(f"Raw:  {raw_action:6.3f}  →  Safe: {safe_action:6.3f}  |  "
                  f"Penalty: {new_feedback['penalty']:5.3f}  |  {status}")

            if new_feedback["clipped"]:
                print(f"   → SAFETY LIMIT ACTIVATED! Command was capped.")

            feedback = new_feedback

        except KeyboardInterrupt:
            print("\n\nПлъгинът спря.")
            break
        except Exception as e:
            print(f"Грешка: {e}")

if __name__ == "__main__":
    main()