#ifndef MICROSRL_CBF_H
#define MICROSRL_CBF_H

#include <Arduino.h>
#include <math.h>

class MicroSafeRL_CBF {
private:
    float cbf_gain  = 0.38f;
    float cbf_alpha = 1.65f;
    float margin    = 0.20f;

    float penalty     = 0.0f;
    float cbf_penalty = 0.0f;

public:
    float apply_safe_control(float cmd) {

        float h = 1.5f - fabsf(cmd);
        cbf_penalty = 0.0f;

        if (h < margin) {
            cbf_penalty = cbf_alpha * (margin - h);
        }

        float dynamic_gain = cbf_gain * (1.0f - penalty);
        if (dynamic_gain < 0.0f) dynamic_gain = 0.0f;

        float attenuation = 1.0f - dynamic_gain * cbf_penalty;

        float safe = cmd * attenuation;

        penalty = 0.95f * penalty + 0.05f * cbf_penalty;

        return constrain(safe, -1.5f, 1.5f);
    }

    float get_penalty() const { return penalty; }
    float get_cbf_penalty() const { return cbf_penalty; }

    float get_safety_index() const {
        return 1.0f - (penalty + cbf_penalty) * 0.5f;
    }
};

#endif