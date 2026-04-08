#ifndef MICROSAFE_CONTROLLER_H
#define MICROSAFE_CONTROLLER_H

#include "MicroSafeRL.h"
#include "MicroSafeRL_CBF.h"

class MicroSafeController {
private:
    MicroSafeRL rl;
    MicroSafeRL_CBF cbf;

public:
    float apply(float ai, float sensor) {
        float rl_safe = rl.apply_safe_control(ai, sensor);
        float final_safe = cbf.apply_safe_control(rl_safe);
        return final_safe;
    }

    // ⭐ ТОВА ЛИПСВА ПРИ ТЕБ
    float get_safety_score() const {
        return 1.0f - (rl.get_penalty() + cbf.get_cbf_penalty()) * 0.5f;
    }

    float get_rl_penalty() const { return rl.get_penalty(); }
    float get_cbf_penalty() const { return cbf.get_cbf_penalty(); }
};

#endif