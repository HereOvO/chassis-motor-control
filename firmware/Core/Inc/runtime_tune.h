#ifndef RUNTIME_TUNE_H
#define RUNTIME_TUNE_H

#include "chassis_config.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    chassis_profile_id_t active_profile_id;
    uint8_t active_profile_dirty;
    chassis_profile_t active_profile;
} runtime_tune_state_t;

void runtime_tune_init(void);
bool runtime_tune_set_param(runtime_param_id_t param_id, float value);
bool runtime_tune_commit(void);
bool runtime_tune_restore_defaults(void);
bool runtime_tune_switch_profile(chassis_profile_id_t profile_id);
const chassis_profile_t *runtime_tune_get_active_profile(void);
const chassis_profile_t *runtime_tune_get_profile(chassis_profile_id_t profile_id);
const runtime_tune_state_t *runtime_tune_get_state(void);

#ifdef __cplusplus
}
#endif

#endif /* RUNTIME_TUNE_H */
