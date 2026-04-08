# MicroSafe-RL

Ultra-lightweight AI Safety Layer for Robotics & LLM-driven systems.

## What it does

MicroSafe sits between AI and execution and ensures:

- bounded outputs
- anomaly suppression
- adaptive safety control
- real-time risk scoring

## Architecture

AI → MicroSafeRL → CBF → Safe Output

## Why this matters

AI systems can generate unstable or unsafe actions.
MicroSafe ensures control signals remain within safe operational limits.

## Features

- zero training required
- runs on microcontrollers (Arduino, ESP32, STM32)
- compatible with RL / LLM / control systems
- real-time performance

## Demo

Run:

examples/basic_demo/basic_demo.ino

Output:

ai_input,safe_output,safety_score

## Use cases

- robotics
- drones
- industrial control
- edge AI systems

## License

MIT (research & prototyping)

Commercial licensing available for safety-critical systems