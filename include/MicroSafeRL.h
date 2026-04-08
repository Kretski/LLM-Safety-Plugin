#ifndef MICROSAFE_RL_H
#define MICROSAFE_RL_H

#include <Arduino.h>
#include <math.h>

/**
 * MicroSafe-RL - Ultra-lightweight safety layer for embedded RL/LLM control
 * Подходящ за STM32, ESP32 и други микроконтролери
 */

class MicroSafeRL {
private:
    // Основни параметри
    float kappa;
    float alpha;
    float beta;           // за EMA
    float lambda_;        // decay фактор (използва се вместо beta в някои версии)
    float max_penalty;
    float min_limit;
    float max_limit;
    float gravity_factor;

    // Вътрешно състояние
    float _ema_mean;
    float _ema_mad;
    float _prev_sensor;
    float _penalty;
    bool  _initialized;

public:
    // Конструктор
    MicroSafeRL(
        float k = 1.15f,
        float a = 0.55f,
        float b = 2.2f,
        float lm = 0.12f,
        float max_p = 1.0f,
        float l_min = -1.5f,
        float l_max = 1.5f,
        float g = 0.05f
    ) : kappa(k), alpha(a), beta(b), lambda_(lm), max_penalty(max_p),
        min_limit(l_min), max_limit(l_max), gravity_factor(g),
        _ema_mean(0.0f), _ema_mad(0.0f), _prev_sensor(0.0f),
        _penalty(0.0f), _initialized(false) {}

    // Инициализация
    void init(float initial_sensor = 0.0f) {
        _ema_mean = initial_sensor;
        _prev_sensor = initial_sensor;
        _penalty = 0.0f;
        _initialized = true;
    }

    // Основна функция за безопасно управление
    float apply_safe_control(float ai_action, float sensor) {
        if (!_initialized) {
            init(sensor);
        }

        // EMA + MAD + Velocity
        _ema_mean = lambda_ * _ema_mean + (1.0f - lambda_) * sensor;
        float abs_dev = fabs(sensor - _ema_mean);
        _ema_mad = lambda_ * _ema_mad + (1.0f - lambda_) * abs_dev;

        float velocity = fabs(sensor - _prev_sensor);
        _prev_sensor = sensor;

        float coherence = 1.0f / (1.0f + abs_dev * beta);
        float raw = _ema_mad + alpha * (1.0f - coherence) + 0.3f * velocity;

        _penalty = fmin(kappa * raw, max_penalty);

        float gravity = fmax(0.0f, 1.0f - _penalty * gravity_factor);
        float modulated = ai_action * gravity;

        float safe_action = fmax(min_limit, fmin(max_limit, modulated));

        return safe_action;
    }

    // Getter-и
    float get_penalty() const {
        return _penalty;
    }

    float get_current_reward() const {
        return fmax(0.0f, 1.0f - _penalty);
    }

    // === НОВИ GETTER-И ЗА CBF СИНЕРГИЯТА ===
    float get_min_limit() const { return min_limit; }
    float get_max_limit() const { return max_limit; }

    // За debug
    float get_ema_mean() const { return _ema_mean; }
    float get_ema_mad() const { return _ema_mad; }
};

#endif