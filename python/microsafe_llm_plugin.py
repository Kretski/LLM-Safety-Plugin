"""
MicroSafe-RL — LLM Safety Plugin
=================================
Pipeline: User → Local LLM → raw_action → MicroSafe-RL → actuator
                                  ↑                            |
                            feedback loop ←────────────────────┘

Работи с всеки локален LLM (Ollama, LM Studio, llama.cpp)
който поддържа OpenAI-съвместим API.

Изисквания:
    pip install requests numpy

Употреба:
    python microsafe_llm_plugin.py
"""

import json
import time
import math
import requests
from dataclasses import dataclass, field
from typing import Optional

# ── Конфигурация ───────────────────────────────────────────────────────────
LLM_BASE_URL  = "http://localhost:11434/v1"   # Ollama default
LLM_MODEL     = "llama3"                       # или gemma2, mistral и т.н.
LLM_TIMEOUT   = 10                             # секунди

SAFETY_MIN    = -1.0
SAFETY_MAX    =  1.0
KAPPA         = 0.078
ALPHA         = 0.55
DECAY         = 2.2
BETA          = 0.12
G_FACTOR      = 1.0
VEL_WEIGHT    = 0.05

SYSTEM_PROMPT = """You are an action generator for a robot control system.
You output ONLY a JSON object. No explanation.

Given a natural language command and the last safety feedback,
compute a control signal.

Output format:
{"action": <float -1.0 to 1.0>, "confidence": <float 0.0 to 1.0>, "reasoning": "<10 words max>"}
"""

# ── MicroSafe-RL (Python port на C++ ядрото) ──────────────────────────────
class MicroSafeRL:
    """
    O(1) deterministic safety layer.
    Python-точен порт на MicroSafeRL_misra.h
    """
    def __init__(self, kappa=KAPPA, alpha=ALPHA, decay=DECAY,
                 beta=BETA, g=G_FACTOR, min_limit=SAFETY_MIN,
                 max_limit=SAFETY_MAX, vel_weight=VEL_WEIGHT):
        self.kappa      = kappa
        self.alpha      = alpha
        self.decay      = decay
        self.beta       = beta
        self.g          = g
        self.min_limit  = min_limit
        self.max_limit  = max_limit
        self.vel_weight = vel_weight

        self._ema       = 0.0
        self._ema_mad   = 0.0
        self._prev      = 0.0
        self._reward    = 1.0
        self._step      = 0

    def apply(self, ai_action: float, sensor: float) -> tuple[float, dict]:
        """
        Прехваща unsafe команди.
        Връща (safe_output, feedback_dict)
        """
        t0 = time.perf_counter()

        # Hard shield — винаги активен от стъпка 0
        if math.isnan(ai_action) or math.isinf(ai_action):
            safe = self.min_limit if ai_action < 0 else self.max_limit
            return self._finalize(safe, ai_action, t0)

        # EMA baseline
        if self._step == 0:
            self._ema = sensor
        else:
            self._ema = self._ema * (1.0 - self.beta) + sensor * self.beta

        # EMA MAD (mean absolute deviation)
        deviation = abs(sensor - self._ema)
        self._ema_mad = self._ema_mad * (1.0 - self.beta) + deviation * self.beta

        # Coherence (1 = стабилен сигнал)
        signal_range = max(abs(self.max_limit - self.min_limit), 1e-6)
        coherence = max(0.0, 1.0 - self._ema_mad / (signal_range * 0.5))

        # Velocity (скорост на промяна)
        velocity = abs(sensor - self._prev)

        # Penalty
        penalty = self.kappa * (
            self._ema_mad +
            self.alpha * (1.0 - coherence) +
            self.vel_weight * velocity
        )

        # Gravity attenuation
        gravity  = max(0.0, 1.0 - penalty * self.g)
        safe_raw = ai_action * gravity

        # Hard clip
        safe = max(self.min_limit, min(self.max_limit, safe_raw))

        self._reward = max(0.0, 1.0 - penalty)
        self._prev   = sensor
        self._step  += 1

        latency_us = (time.perf_counter() - t0) * 1e6

        feedback = {
            "safe_out":   round(safe, 4),
            "reward":     round(self._reward, 4),
            "penalty":    round(penalty, 4),
            "gravity":    round(gravity, 4),
            "coherence":  round(coherence, 4),
            "risk":       round(penalty, 4),
            "clipped":    (safe != ai_action * gravity),
            "latency_us": round(latency_us, 2),
        }
        return safe, feedback

    def _finalize(self, safe, ai_action, t0):
        latency_us = (time.perf_counter() - t0) * 1e6
        feedback = {
            "safe_out": safe, "reward": 0.0, "penalty": 1.0,
            "gravity": 0.0, "coherence": 0.0, "risk": 1.0,
            "clipped": True, "latency_us": round(latency_us, 2),
        }
        return safe, feedback


# ── LLM клиент (OpenAI-съвместим) ─────────────────────────────────────────
def call_llm(user_cmd: str, state: dict, last_feedback: dict) -> dict:
    """
    Извиква локален LLM и парсва JSON отговора.
    Работи с Ollama, LM Studio, llama.cpp server.
    """
    prompt = f"""Command: {user_cmd}
Robot state: position={state['position']:.2f}, velocity={state['velocity']:.2f}
Last safety feedback: risk={last_feedback.get('risk', 0):.2f}, clipped={last_feedback.get('clipped', False)}

Generate control action."""

    payload = {
        "model": LLM_MODEL,
        "messages": [
            {"role": "system", "content": SYSTEM_PROMPT},
            {"role": "user",   "content": prompt},
        ],
        "temperature": 0.2,
        "max_tokens":  80,
    }

    try:
        resp = requests.post(
            f"{LLM_BASE_URL}/chat/completions",
            json=payload,
            timeout=LLM_TIMEOUT,
        )
        resp.raise_for_status()
        content = resp.json()["choices"][0]["message"]["content"].strip()

        # Парсване — robust срещу markdown fences
        content = content.replace("```json", "").replace("```", "").strip()
        result  = json.loads(content)

        # Валидация на структурата
        action     = float(result.get("action", 0.0))
        confidence = float(result.get("confidence", 1.0))
        reasoning  = str(result.get("reasoning", ""))
        return {"action": action, "confidence": confidence, "reasoning": reasoning}

    except requests.exceptions.ConnectionError:
        print("  [!] LLM не е достъпен — използвам симулиран агент")
        return _simulate_action(state)
    except (json.JSONDecodeError, KeyError, ValueError) as e:
        print(f"  [!] LLM JSON грешка: {e} — fallback")
        return _simulate_action(state)


def _simulate_action(state: dict) -> dict:
    """Fallback симулиран агент когато LLM не е наличен."""
    t = time.time()
    action = 0.5 * math.sin(t * 0.8) + 0.3 * math.sin(t * 2.1)
    # Периодично инжектира chaos
    if int(t) % 15 < 4:
        action = 2.5 * math.sin(t * 3.0)   # unsafe!
    return {"action": action, "confidence": 0.9, "reasoning": "simulated agent"}


# ── Главен pipeline ────────────────────────────────────────────────────────
@dataclass
class RobotState:
    position: float = 0.0
    velocity: float = 0.0

    def update(self, safe_out: float, dt: float = 0.1):
        self.velocity = safe_out
        self.position = max(-1.0, min(1.0, self.position + safe_out * dt))


def run_pipeline(user_cmd: str = "move forward gently", steps: int = 60):
    """
    Пълен затворен feedback loop:
    User command → LLM → MicroSafe-RL → Robot → feedback → LLM
    """
    safety   = MicroSafeRL()
    robot    = RobotState()
    feedback = {"risk": 0.0, "clipped": False}

    print("\n" + "="*62)
    print("  MicroSafe-RL  |  LLM Safety Plugin")
    print("="*62)
    print(f"  Command: \"{user_cmd}\"")
    print(f"  Model:   {LLM_MODEL} @ {LLM_BASE_URL}")
    print("="*62)
    print(f"  {'step':>4} | {'llm_raw':>8} | {'safe':>7} | {'risk':>5} | {'clipped':>7} | {'us':>5} | state")
    print(f"  {'----':>4}-+-{'--------':>8}-+-{'-------':>7}-+-{'-----':>5}-+-{'-------':>7}-+-{'-----':>5}-+-------")

    for step in range(steps):
        # 1. LLM генерира команда
        llm_result = call_llm(user_cmd, vars(robot), feedback)
        raw_action = llm_result["action"]

        # 2. MicroSafe-RL прехваща
        safe_out, feedback = safety.apply(raw_action, robot.velocity)

        # 3. Робот се движи
        robot.update(safe_out)

        # 4. Статус
        if abs(raw_action) <= 1.0 and not feedback["clipped"]:
            status = "STABLE"
        elif feedback["clipped"]:
            status = "!! INTERCEPTED"
        else:
            status = "SAFE"

        print(
            f"  {step:>4} | {raw_action:>8.3f} | {safe_out:>7.3f} | "
            f"{feedback['risk']:>5.3f} | {str(feedback['clipped']):>7} | "
            f"{feedback['latency_us']:>5.1f} | {status}"
        )

        time.sleep(0.3)   # 300ms — четимо на видео

    print("="*62)
    print(f"  Готово. Стъпки: {steps} | Финална позиция: {robot.position:.3f}")
    print("="*62)


# ── Entrypoint ─────────────────────────────────────────────────────────────
if __name__ == "__main__":
    import sys

    cmd   = " ".join(sys.argv[1:]) if len(sys.argv) > 1 else "move forward gently"
    steps = 60

    run_pipeline(user_cmd=cmd, steps=steps)
