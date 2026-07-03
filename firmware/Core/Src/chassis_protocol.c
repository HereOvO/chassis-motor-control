#include "chassis_protocol.h"

#include "cmsis_os.h"
#include "chassis_mode.h"
#include "runtime_tune.h"

#include <ctype.h>
#include <stdlib.h>
#include <string.h>

#define CHASSIS_PROTOCOL_ASCII_LINE_MAX 128U
#define CHASSIS_PROTOCOL_RX_QUEUE_SIZE 64U
#define CHASSIS_PROTOCOL_MOWEN_FRAME_LEN 12U
#define CHASSIS_PROTOCOL_MOWEN_HEAD0 0xAAU
#define CHASSIS_PROTOCOL_MOWEN_HEAD1 0xBBU
#define CHASSIS_PROTOCOL_MOWEN_CMD0 0x0AU
#define CHASSIS_PROTOCOL_MOWEN_CMD1 0x12U
#define CHASSIS_PROTOCOL_MOWEN_CMD2 0x02U

typedef enum
{
    CHASSIS_PROTOCOL_PARSE_IDLE = 0U,
    CHASSIS_PROTOCOL_PARSE_ASCII = 1U,
    CHASSIS_PROTOCOL_PARSE_MOWEN = 2U
} chassis_protocol_parse_state_t;

typedef enum
{
    CHASSIS_PROTOCOL_FRAME_NONE = 0U,
    CHASSIS_PROTOCOL_FRAME_ASCII = 1U,
    CHASSIS_PROTOCOL_FRAME_MOWEN = 2U
} chassis_protocol_frame_kind_t;

typedef struct
{
    uint8_t data[CHASSIS_PROTOCOL_RX_QUEUE_SIZE];
    volatile uint16_t head;
    volatile uint16_t tail;
} chassis_protocol_rx_ring_t;

static UART_HandleTypeDef *g_protocol_uart = NULL;
static chassis_protocol_mode_t g_protocol_mode = CHASSIS_DEFAULT_PROTOCOL_MODE;
static chassis_protocol_parse_state_t g_parse_state = CHASSIS_PROTOCOL_PARSE_IDLE;
static chassis_protocol_rx_ring_t g_rx_ring;
static char g_ascii_line[CHASSIS_PROTOCOL_ASCII_LINE_MAX];
static size_t g_ascii_length = 0U;
static uint8_t g_ascii_line_ready = 0U;
static chassis_cmd_t g_pending_cmd;
static uint8_t g_pending_valid = 0U;
static uint8_t g_mowen_frame[CHASSIS_PROTOCOL_MOWEN_FRAME_LEN];
static uint8_t g_mowen_index = 0U;

static void chassis_protocol_clear_pending(void)
{
    memset(&g_pending_cmd, 0, sizeof(g_pending_cmd));
    g_pending_valid = 0U;
}

static uint16_t chassis_protocol_ring_next(uint16_t value)
{
    return (uint16_t)((value + 1U) % CHASSIS_PROTOCOL_RX_QUEUE_SIZE);
}

static void chassis_protocol_ring_reset(void)
{
    g_rx_ring.head = 0U;
    g_rx_ring.tail = 0U;
}

static uint8_t chassis_protocol_ring_push(uint8_t byte)
{
    uint16_t next_head;

    next_head = chassis_protocol_ring_next(g_rx_ring.head);
    if (next_head == g_rx_ring.tail) {
        return 0U;
    }

    g_rx_ring.data[g_rx_ring.head] = byte;
    g_rx_ring.head = next_head;
    return 1U;
}

static uint8_t chassis_protocol_ring_pop(uint8_t *byte)
{
    if (byte == NULL || g_rx_ring.head == g_rx_ring.tail) {
        return 0U;
    }

    *byte = g_rx_ring.data[g_rx_ring.tail];
    g_rx_ring.tail = chassis_protocol_ring_next(g_rx_ring.tail);
    return 1U;
}

static char *chassis_protocol_trim_left(char *text)
{
    if (text == NULL) {
        return NULL;
    }

    while (*text != '\0' && isspace((unsigned char)*text) != 0) {
        ++text;
    }

    return text;
}

static void chassis_protocol_trim_right(char *text)
{
    size_t len;

    if (text == NULL) {
        return;
    }

    len = strlen(text);
    while (len > 0U) {
        if (isspace((unsigned char)text[len - 1U]) == 0) {
            break;
        }
        text[len - 1U] = '\0';
        --len;
    }
}

static bool chassis_protocol_parse_float(const char *text, float *value)
{
    char *endptr = NULL;
    const char *tail;

    if (text == NULL || value == NULL) {
        return false;
    }

    *value = strtof(text, &endptr);
    if (endptr == text) {
        return false;
    }

    tail = endptr;
    while (*tail != '\0' && isspace((unsigned char)*tail) != 0) {
        ++tail;
    }

    return (*tail == '\0');
}

static bool chassis_protocol_parse_uint8(const char *text, uint8_t *value)
{
    char *endptr = NULL;
    const char *tail;
    long parsed;

    if (text == NULL || value == NULL) {
        return false;
    }

    parsed = strtol(text, &endptr, 10);
    if (endptr == text || parsed < 0L || parsed > 255L) {
        return false;
    }

    tail = endptr;
    while (*tail != '\0' && isspace((unsigned char)*tail) != 0) {
        ++tail;
    }

    if (*tail != '\0') {
        return false;
    }

    *value = (uint8_t)parsed;
    return true;
}

static bool chassis_protocol_parse_int8(const char *text, int8_t *value)
{
    char *endptr = NULL;
    const char *tail;
    long parsed;

    if (text == NULL || value == NULL) {
        return false;
    }

    parsed = strtol(text, &endptr, 10);
    if (endptr == text || parsed < -128L || parsed > 127L) {
        return false;
    }

    tail = endptr;
    while (*tail != '\0' && isspace((unsigned char)*tail) != 0) {
        ++tail;
    }

    if (*tail != '\0') {
        return false;
    }

    *value = (int8_t)parsed;
    return true;
}

static bool chassis_protocol_parse_mode_token(const char *text, chassis_mode_t *mode)
{
    uint8_t value;

    if (mode == NULL || !chassis_protocol_parse_uint8(text, &value)) {
        return false;
    }

    if (value > (uint8_t)CHASSIS_MODE_FAULT) {
        return false;
    }

    *mode = (chassis_mode_t)value;
    return true;
}

static bool chassis_protocol_parse_profile_token(const char *text, chassis_profile_id_t *profile_id)
{
    uint8_t value;

    if (profile_id == NULL || !chassis_protocol_parse_uint8(text, &value)) {
        return false;
    }

    if (value >= CHASSIS_PROFILE_COUNT) {
        return false;
    }

    *profile_id = (chassis_profile_id_t)value;
    return true;
}

static bool chassis_protocol_parse_param_token(const char *text, runtime_param_id_t *param_id)
{
    uint8_t value;

    if (param_id == NULL || !chassis_protocol_parse_uint8(text, &value)) {
        return false;
    }

    if (value >= RUNTIME_PARAM_COUNT) {
        return false;
    }

    *param_id = (runtime_param_id_t)value;
    return true;
}

static bool chassis_protocol_store_cmd(const chassis_cmd_t *cmd)
{
    if (cmd == NULL) {
        return false;
    }

    g_pending_cmd = *cmd;
    g_pending_valid = 1U;
    return true;
}

static bool chassis_protocol_parse_ascii_line(char *line)
{
    char *cursor;
    char *token;
    chassis_cmd_t cmd;

    if (line == NULL) {
        return false;
    }

    memset(&cmd, 0, sizeof(cmd));
    cursor = chassis_protocol_trim_left(line);
    if (cursor == NULL || *cursor == '\0') {
        return false;
    }
    chassis_protocol_trim_right(cursor);

    token = strtok(cursor, ",");
    if (token == NULL) {
        return false;
    }

    if (strcmp(token, "MODE") == 0) {
        token = strtok(NULL, ",");
        if (!chassis_protocol_parse_mode_token(token, &cmd.mode)) {
            return false;
        }
        cmd.type = CHASSIS_CMD_MODE;
        return chassis_protocol_store_cmd(&cmd);
    }

    if (strcmp(token, "PROFILE") == 0) {
        token = strtok(NULL, ",");
        if (!chassis_protocol_parse_profile_token(token, &cmd.profile_id)) {
            return false;
        }
        cmd.type = CHASSIS_CMD_PROFILE;
        cmd.flags = CHASSIS_CMD_FLAG_PROFILE_SWITCH;
        return chassis_protocol_store_cmd(&cmd);
    }

    if (strcmp(token, "VEL") == 0 || strcmp(token, "CMD_VEL") == 0) {
        token = strtok(NULL, ",");
        if (!chassis_protocol_parse_float(token, &cmd.vx_mps)) {
            return false;
        }
        token = strtok(NULL, ",");
        if (!chassis_protocol_parse_float(token, &cmd.vy_mps)) {
            return false;
        }
        token = strtok(NULL, ",");
        if (!chassis_protocol_parse_float(token, &cmd.wz_radps)) {
            return false;
        }
        cmd.type = CHASSIS_CMD_VELOCITY;
        return chassis_protocol_store_cmd(&cmd);
    }

    if (strcmp(token, "ENABLE") == 0) {
        token = strtok(NULL, ",");
        if (!chassis_protocol_parse_uint8(token, &cmd.enable)) {
            return false;
        }
        cmd.type = CHASSIS_CMD_ENABLE;
        return chassis_protocol_store_cmd(&cmd);
    }

    if (strcmp(token, "PWMTEST") == 0 || strcmp(token, "RAWPWM") == 0) {
        token = strtok(NULL, ",");
        if (!chassis_protocol_parse_uint8(token, &cmd.motor_id)) {
            return false;
        }
        token = strtok(NULL, ",");
        if (!chassis_protocol_parse_int8(token, &cmd.raw_direction)) {
            return false;
        }
        token = strtok(NULL, ",");
        if (!chassis_protocol_parse_float(token, &cmd.duty_norm)) {
            return false;
        }
        cmd.type = CHASSIS_CMD_RAW_PWM;
        return chassis_protocol_store_cmd(&cmd);
    }

    if (strcmp(token, "MSET") == 0 || strcmp(token, "MPARAM") == 0) {
        uint8_t param_raw;

        token = strtok(NULL, ",");
        if (!chassis_protocol_parse_uint8(token, &cmd.motor_id) || cmd.motor_id >= CHASSIS_WHEEL_COUNT) {
            return false;
        }
        token = strtok(NULL, ",");
        if (!chassis_protocol_parse_uint8(token, &param_raw) || param_raw >= MOTOR_PARAM_COUNT) {
            return false;
        }
        token = strtok(NULL, ",");
        if (!chassis_protocol_parse_float(token, &cmd.value)) {
            return false;
        }
        cmd.motor_param_id = (motor_control_param_id_t)param_raw;
        cmd.type = CHASSIS_CMD_MOTOR_PARAM_SET;
        return chassis_protocol_store_cmd(&cmd);
    }
    if (strcmp(token, "SET") == 0) {
        token = strtok(NULL, ",");
        if (!chassis_protocol_parse_param_token(token, &cmd.param_id)) {
            return false;
        }
        token = strtok(NULL, ",");
        if (!chassis_protocol_parse_float(token, &cmd.value)) {
            return false;
        }
        cmd.type = CHASSIS_CMD_PARAM_SET;
        return chassis_protocol_store_cmd(&cmd);
    }

    if (strcmp(token, "COMMIT") == 0) {
        cmd.type = CHASSIS_CMD_PARAM_COMMIT;
        return chassis_protocol_store_cmd(&cmd);
    }

    if (strcmp(token, "RESTORE") == 0) {
        cmd.type = CHASSIS_CMD_PARAM_RESTORE;
        return chassis_protocol_store_cmd(&cmd);
    }

    if (strcmp(token, "ZERO") == 0) {
        cmd.type = CHASSIS_CMD_ZERO;
        cmd.flags = CHASSIS_CMD_FLAG_ZERO_OUTPUT | CHASSIS_CMD_FLAG_RESET_INTEGRATOR;
        return chassis_protocol_store_cmd(&cmd);
    }

    if (strcmp(token, "ASCII") == 0) {
        chassis_protocol_set_mode(CHASSIS_PROTOCOL_MODE_ASCII);
        return true;
    }

    if (strcmp(token, "MOWEN") == 0) {
        chassis_protocol_set_mode(CHASSIS_PROTOCOL_MODE_MOWEN);
        return true;
    }

    return false;
}

static uint16_t chassis_protocol_u16_le(const uint8_t *ptr)
{
    return (uint16_t)((uint16_t)ptr[0] | ((uint16_t)ptr[1] << 8));
}

static int16_t chassis_protocol_s16_le(const uint8_t *ptr)
{
    return (int16_t)chassis_protocol_u16_le(ptr);
}

static bool chassis_protocol_parse_mowen_frame(const uint8_t frame[CHASSIS_PROTOCOL_MOWEN_FRAME_LEN])
{
    chassis_cmd_t cmd;
    int16_t raw_vx;
    int16_t raw_vy;
    int16_t raw_wz;

    if (frame == NULL) {
        return false;
    }

    if (frame[0] != CHASSIS_PROTOCOL_MOWEN_HEAD0 ||
        frame[1] != CHASSIS_PROTOCOL_MOWEN_HEAD1 ||
        frame[2] != CHASSIS_PROTOCOL_MOWEN_CMD0 ||
        frame[3] != CHASSIS_PROTOCOL_MOWEN_CMD1 ||
        frame[4] != CHASSIS_PROTOCOL_MOWEN_CMD2) {
        return false;
    }

    memset(&cmd, 0, sizeof(cmd));
    cmd.type = CHASSIS_CMD_VELOCITY;
    cmd.flags = CHASSIS_CMD_FLAG_AUTO_ENABLE;
    raw_vx = chassis_protocol_s16_le(&frame[5]);
    raw_vy = chassis_protocol_s16_le(&frame[7]);
    raw_wz = chassis_protocol_s16_le(&frame[9]);
    cmd.vx_mps = (float)raw_vx / 1000.0f;
    cmd.vy_mps = (float)raw_vy / 1000.0f;
    cmd.wz_radps = (float)raw_wz / 1000.0f;

    return chassis_protocol_store_cmd(&cmd);
}

static void chassis_protocol_parse_mowen_stream(void)
{
    uint8_t byte;

    while (chassis_protocol_ring_pop(&byte) != 0U) {
        if (g_protocol_mode == CHASSIS_PROTOCOL_MODE_ASCII) {
            if (byte == '\r' || byte == '\n') {
                if (g_ascii_length > 0U) {
                    g_ascii_line[g_ascii_length] = '\0';
                    g_ascii_line_ready = 1U;
                }
                g_ascii_length = 0U;
                g_parse_state = CHASSIS_PROTOCOL_PARSE_IDLE;
            } else if (g_ascii_length < (sizeof(g_ascii_line) - 1U)) {
                g_ascii_line[g_ascii_length++] = (char)byte;
                g_parse_state = CHASSIS_PROTOCOL_PARSE_ASCII;
            } else {
                g_ascii_length = 0U;
                g_parse_state = CHASSIS_PROTOCOL_PARSE_IDLE;
            }
            continue;
        }

        if (g_parse_state == CHASSIS_PROTOCOL_PARSE_IDLE) {
            if (byte == CHASSIS_PROTOCOL_MOWEN_HEAD0) {
                g_mowen_frame[0] = byte;
                g_mowen_index = 1U;
                g_parse_state = CHASSIS_PROTOCOL_PARSE_MOWEN;
            }
            continue;
        }

        if (g_parse_state == CHASSIS_PROTOCOL_PARSE_MOWEN) {
            if (g_mowen_index < CHASSIS_PROTOCOL_MOWEN_FRAME_LEN) {
                g_mowen_frame[g_mowen_index++] = byte;
            }

            if (g_mowen_index >= CHASSIS_PROTOCOL_MOWEN_FRAME_LEN) {
                (void)chassis_protocol_parse_mowen_frame(g_mowen_frame);
                g_mowen_index = 0U;
                g_parse_state = CHASSIS_PROTOCOL_PARSE_IDLE;
            }
            continue;
        }

        g_parse_state = CHASSIS_PROTOCOL_PARSE_IDLE;
        g_mowen_index = 0U;
    }
}

void chassis_protocol_init(UART_HandleTypeDef *huart)
{
    g_protocol_uart = huart;
    g_protocol_mode = CHASSIS_DEFAULT_PROTOCOL_MODE;
    g_parse_state = CHASSIS_PROTOCOL_PARSE_IDLE;
    g_ascii_length = 0U;
    g_ascii_line_ready = 0U;
    g_mowen_index = 0U;
    chassis_protocol_ring_reset();
    chassis_protocol_clear_pending();
}

void chassis_protocol_restart_rx(void)
{
    (void)g_protocol_uart;
}

void chassis_protocol_set_mode(chassis_protocol_mode_t mode)
{
    if (mode != CHASSIS_PROTOCOL_MODE_ASCII && mode != CHASSIS_PROTOCOL_MODE_MOWEN) {
        return;
    }

    g_protocol_mode = mode;
    g_parse_state = CHASSIS_PROTOCOL_PARSE_IDLE;
    g_ascii_length = 0U;
    g_ascii_line_ready = 0U;
    g_mowen_index = 0U;
    chassis_protocol_clear_pending();
    chassis_protocol_ring_reset();
}

chassis_protocol_mode_t chassis_protocol_get_mode(void)
{
    return g_protocol_mode;
}

void chassis_protocol_enqueue_rx_byte_from_isr(uint8_t byte)
{
    (void)chassis_protocol_ring_push(byte);
}

bool chassis_protocol_take_rx_byte(uint8_t *out_byte)
{
    if (out_byte == NULL) {
        return false;
    }

    return chassis_protocol_ring_pop(out_byte) != 0U;
}

void chassis_protocol_on_rx_byte(uint8_t byte)
{
    chassis_protocol_enqueue_rx_byte_from_isr(byte);
}

bool chassis_protocol_poll(chassis_cmd_t *out_cmd)
{
    char line_copy[CHASSIS_PROTOCOL_ASCII_LINE_MAX];

    if (out_cmd == NULL) {
        return false;
    }

    chassis_protocol_parse_mowen_stream();

    if (g_ascii_line_ready != 0U && g_protocol_mode == CHASSIS_PROTOCOL_MODE_ASCII) {
        memcpy(line_copy, g_ascii_line, sizeof(line_copy));
        g_ascii_line_ready = 0U;
        if (!chassis_protocol_parse_ascii_line(line_copy)) {
            return false;
        }
    }

    if (g_pending_valid == 0U) {
        return false;
    }

    *out_cmd = g_pending_cmd;
    chassis_protocol_clear_pending();
    return true;
}

void chassis_protocol_reset(void)
{
    g_parse_state = CHASSIS_PROTOCOL_PARSE_IDLE;
    g_ascii_length = 0U;
    g_ascii_line_ready = 0U;
    g_mowen_index = 0U;
    chassis_protocol_ring_reset();
    chassis_protocol_clear_pending();
}



