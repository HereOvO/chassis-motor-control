#include "runtime_tune.h"

#include <string.h>

static chassis_profile_t g_profile_committed[CHASSIS_PROFILE_COUNT];
static chassis_profile_t g_profile_working[CHASSIS_PROFILE_COUNT];
static uint8_t g_profile_dirty[CHASSIS_PROFILE_COUNT];
static chassis_profile_id_t g_active_profile_id = CHASSIS_PROFILE_MECANUM;
static runtime_tune_state_t g_state;
static uint8_t g_initialized = 0U;

static bool runtime_tune_profile_id_valid(chassis_profile_id_t profile_id)
{
    return profile_id < CHASSIS_PROFILE_COUNT;
}

static bool runtime_tune_value_is_nan(float value)
{
    return (value != value);
}

static bool runtime_tune_set_sign(int8_t *sign_field, float value)
{
    if (sign_field == NULL || runtime_tune_value_is_nan(value)) {
        return false;
    }

    if (value > 0.0f) {
        *sign_field = 1;
        return true;
    }

    if (value < 0.0f) {
        *sign_field = -1;
        return true;
    }

    return false;
}

static bool runtime_tune_apply_param(chassis_profile_t *profile, runtime_param_id_t param_id, float value)
{
    if (profile == NULL || runtime_tune_value_is_nan(value)) {
        return false;
    }

    switch (param_id) {
    case RUNTIME_PARAM_WHEEL_RADIUS:
        if (value < CHASSIS_WHEEL_RADIUS_MIN_M || value > CHASSIS_WHEEL_RADIUS_MAX_M) {
            return false;
        }
        profile->wheel_radius_m = value;
        return true;

    case RUNTIME_PARAM_TRACK_WIDTH:
        if (value < CHASSIS_TRACK_WIDTH_MIN_M || value > CHASSIS_TRACK_WIDTH_MAX_M) {
            return false;
        }
        profile->track_width_m = value;
        return true;

    case RUNTIME_PARAM_WHEEL_BASE:
        if (profile->drive_mode == CHASSIS_DRIVE_MECANUM) {
            if (value < CHASSIS_WHEEL_BASE_MIN_M || value > CHASSIS_WHEEL_BASE_MAX_M || value == 0.0f) {
                return false;
            }
        } else if (value < 0.0f || value > CHASSIS_WHEEL_BASE_MAX_M) {
            return false;
        }
        profile->wheel_base_m = value;
        return true;

    case RUNTIME_PARAM_REDUCTION_RATIO:
        if (value < CHASSIS_REDUCTION_RATIO_MIN || value > CHASSIS_REDUCTION_RATIO_MAX) {
            return false;
        }
        profile->reduction_ratio = value;
        return true;

    case RUNTIME_PARAM_ENCODER_PPR:
        if (value < (float)CHASSIS_ENCODER_PPR_MIN || value > (float)CHASSIS_ENCODER_PPR_MAX) {
            return false;
        }
        profile->encoder_ppr = (uint16_t)(value + 0.5f);
        return true;

    case RUNTIME_PARAM_DIRECTION_SIGN_0:
        return runtime_tune_set_sign(&profile->direction_sign[0], value);

    case RUNTIME_PARAM_DIRECTION_SIGN_1:
        return runtime_tune_set_sign(&profile->direction_sign[1], value);

    case RUNTIME_PARAM_DIRECTION_SIGN_2:
        return runtime_tune_set_sign(&profile->direction_sign[2], value);

    case RUNTIME_PARAM_DIRECTION_SIGN_3:
        return runtime_tune_set_sign(&profile->direction_sign[3], value);

    case RUNTIME_PARAM_MAX_LINEAR_SPEED:
        if (value < CHASSIS_LINEAR_SPEED_MIN_MPS || value > CHASSIS_LINEAR_SPEED_MAX_MPS) {
            return false;
        }
        profile->max_linear_speed_mps = value;
        return true;

    case RUNTIME_PARAM_MAX_ANGULAR_SPEED:
        if (value < CHASSIS_ANGULAR_SPEED_MIN_RADPS || value > CHASSIS_ANGULAR_SPEED_MAX_RADPS) {
            return false;
        }
        profile->max_angular_speed_radps = value;
        return true;

    case RUNTIME_PARAM_MAX_LINEAR_ACCEL:
        if (value < CHASSIS_LINEAR_ACCEL_MIN_MPS2 || value > CHASSIS_LINEAR_ACCEL_MAX_MPS2) {
            return false;
        }
        profile->max_linear_accel_mps2 = value;
        return true;

    case RUNTIME_PARAM_MAX_ANGULAR_ACCEL:
        if (value < CHASSIS_ANGULAR_ACCEL_MIN_RADPS2 || value > CHASSIS_ANGULAR_ACCEL_MAX_RADPS2) {
            return false;
        }
        profile->max_angular_accel_radps2 = value;
        return true;

    case RUNTIME_PARAM_PWM_LIMIT:
        if (value < (float)CHASSIS_PWM_LIMIT_MIN || value > (float)CHASSIS_PWM_LIMIT_MAX) {
            return false;
        }
        profile->pwm_limit = (uint16_t)(value + 0.5f);
        return true;

    default:
        return false;
    }
}

static void runtime_tune_load_defaults(void)
{
    for (chassis_profile_id_t profile_id = CHASSIS_PROFILE_DIFF;
         profile_id < CHASSIS_PROFILE_COUNT;
         profile_id = (chassis_profile_id_t)(profile_id + 1U)) {
        const chassis_profile_t *defaults = chassis_config_get_profile(profile_id);

        if (defaults != NULL) {
            g_profile_committed[profile_id] = *defaults;
            g_profile_working[profile_id] = *defaults;
            g_profile_dirty[profile_id] = 0U;
        }
    }
}

static void runtime_tune_refresh_state(void)
{
    if (!runtime_tune_profile_id_valid(g_active_profile_id)) {
        g_active_profile_id = chassis_config_default_profile_id();
    }

    g_state.active_profile_id = g_active_profile_id;
    g_state.active_profile_dirty = g_profile_dirty[g_active_profile_id];
    g_state.active_profile = g_profile_working[g_active_profile_id];
}

void runtime_tune_init(void)
{
    if (g_initialized != 0U) {
        runtime_tune_refresh_state();
        return;
    }

    runtime_tune_load_defaults();
    g_active_profile_id = chassis_config_default_profile_id();
    g_initialized = 1U;
    runtime_tune_refresh_state();
}

bool runtime_tune_switch_profile(chassis_profile_id_t profile_id)
{
    if (!runtime_tune_profile_id_valid(profile_id)) {
        return false;
    }

    runtime_tune_init();
    g_active_profile_id = profile_id;
    runtime_tune_refresh_state();
    return true;
}

bool runtime_tune_set_param(runtime_param_id_t param_id, float value)
{
    chassis_profile_t *profile;
    bool updated;

    runtime_tune_init();

    if (param_id >= RUNTIME_PARAM_COUNT) {
        return false;
    }

    profile = &g_profile_working[g_active_profile_id];
    updated = runtime_tune_apply_param(profile, param_id, value);
    if (!updated) {
        return false;
    }

    g_profile_dirty[g_active_profile_id] = 1U;
    runtime_tune_refresh_state();
    return true;
}

bool runtime_tune_commit(void)
{
    chassis_profile_t *working;

    runtime_tune_init();

    working = &g_profile_working[g_active_profile_id];
    if (!chassis_config_validate_profile(working)) {
        return false;
    }

    g_profile_committed[g_active_profile_id] = *working;
    g_profile_dirty[g_active_profile_id] = 0U;
    runtime_tune_refresh_state();
    return true;
}

bool runtime_tune_restore_defaults(void)
{
    const chassis_profile_t *defaults;

    runtime_tune_init();

    defaults = chassis_config_get_profile(g_active_profile_id);
    if (defaults == NULL) {
        return false;
    }

    g_profile_committed[g_active_profile_id] = *defaults;
    g_profile_working[g_active_profile_id] = *defaults;
    g_profile_dirty[g_active_profile_id] = 0U;
    runtime_tune_refresh_state();
    return true;
}

const chassis_profile_t *runtime_tune_get_active_profile(void)
{
    runtime_tune_init();
    return &g_profile_working[g_active_profile_id];
}

const chassis_profile_t *runtime_tune_get_profile(chassis_profile_id_t profile_id)
{
    runtime_tune_init();

    if (!runtime_tune_profile_id_valid(profile_id)) {
        return NULL;
    }

    return &g_profile_committed[profile_id];
}

const runtime_tune_state_t *runtime_tune_get_state(void)
{
    runtime_tune_init();
    return &g_state;
}
