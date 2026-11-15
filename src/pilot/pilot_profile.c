#include "pilot/pilot.h"

#include <ctype.h>
#include <linux/input-event-codes.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static const double MAX_RATE = 100.0;

typedef struct {
    const char *name;
    int code;
} code_name_t;

static const code_name_t AXIS_NAMES[] = {
    {"ABS_X", ABS_X},         {"ABS_Y", ABS_Y},         {"ABS_Z", ABS_Z},
    {"ABS_RX", ABS_RX},       {"ABS_RY", ABS_RY},       {"ABS_RZ", ABS_RZ},
    {"ABS_THROTTLE", ABS_THROTTLE}, {"ABS_RUDDER", ABS_RUDDER}, {"ABS_WHEEL", ABS_WHEEL},
    {"ABS_GAS", ABS_GAS},     {"ABS_BRAKE", ABS_BRAKE}, {"ABS_HAT0X", ABS_HAT0X},
    {"ABS_HAT0Y", ABS_HAT0Y}, {"ABS_HAT1X", ABS_HAT1X}, {"ABS_HAT1Y", ABS_HAT1Y},
    {"ABS_HAT2X", ABS_HAT2X}, {"ABS_HAT2Y", ABS_HAT2Y}, {"ABS_HAT3X", ABS_HAT3X},
    {"ABS_HAT3Y", ABS_HAT3Y}, {"ABS_MISC", ABS_MISC},
};

static const code_name_t BUTTON_NAMES[] = {
    {"BTN_SOUTH", BTN_SOUTH}, {"BTN_EAST", BTN_EAST},   {"BTN_NORTH", BTN_NORTH},
    {"BTN_WEST", BTN_WEST},   {"BTN_TL", BTN_TL},       {"BTN_TR", BTN_TR},
    {"BTN_TL2", BTN_TL2},     {"BTN_TR2", BTN_TR2},     {"BTN_SELECT", BTN_SELECT},
    {"BTN_START", BTN_START}, {"BTN_MODE", BTN_MODE},   {"BTN_THUMBL", BTN_THUMBL},
    {"BTN_THUMBR", BTN_THUMBR}, {"BTN_TRIGGER", BTN_TRIGGER}, {"BTN_THUMB", BTN_THUMB},
    {"BTN_TOP", BTN_TOP},     {"BTN_BASE", BTN_BASE},
};

static int lookup(const code_name_t *table, size_t count, const char *name) {
    for (size_t i = 0; i < count; i++) {
        if (strcmp(table[i].name, name) == 0) {
            return table[i].code;
        }
    }
    char *end = NULL;
    long value = strtol(name, &end, 0);
    if (end != name && *end == '\0' && value >= 0 && value < PILOT_BUTTON_SLOTS) {
        return (int)value;
    }
    return -1;
}

int pilot_axis_code(const char *name) {
    int code = lookup(AXIS_NAMES, sizeof(AXIS_NAMES) / sizeof(AXIS_NAMES[0]), name);
    return code < PILOT_AXIS_SLOTS ? code : -1;
}

int pilot_button_code(const char *name) {
    return lookup(BUTTON_NAMES, sizeof(BUTTON_NAMES) / sizeof(BUTTON_NAMES[0]), name);
}

static const char *code_to_name(const code_name_t *table, size_t count, uint16_t code, char *scratch,
                                size_t scratch_len) {
    if (code == PILOT_NONE) {
        return "none";
    }
    for (size_t i = 0; i < count; i++) {
        if (table[i].code == (int)code) {
            return table[i].name;
        }
    }
    (void)snprintf(scratch, scratch_len, "%u", (unsigned)code);
    return scratch;
}

static pilot_profile_t make_gamepad(const char *name, uint16_t vendor, uint16_t product,
                                    const char *hint) {
    pilot_profile_t profile;
    memset(&profile, 0, sizeof(profile));
    (void)snprintf(profile.name, sizeof(profile.name), "%s", name);
    (void)snprintf(profile.name_hint, sizeof(profile.name_hint), "%s", hint);
    profile.vendor = vendor;
    profile.product = product;
    profile.verified = false;
    profile.axis_roll = ABS_RX;
    profile.axis_pitch = ABS_RY;
    profile.axis_yaw = ABS_X;
    profile.axis_throttle = ABS_Y;
    profile.invert_pitch = true;
    profile.invert_throttle = true;
    profile.throttle_mode = PILOT_THROTTLE_INTEGRATE;
    profile.deadzone = 0.08;
    profile.expo = 0.3;
    profile.throttle_rate = 0.6;
    profile.button_arm = BTN_SOUTH;
    profile.button_mode = BTN_EAST;
    profile.axis_arm = PILOT_NONE;
    profile.axis_mode = PILOT_NONE;
    for (int i = 0; i < PILOT_AUX_CHANNELS; i++) {
        profile.axis_aux[i] = PILOT_NONE;
    }
    return profile;
}

static pilot_profile_t make_radio(const char *name, uint16_t vendor, uint16_t product,
                                  const char *hint) {
    pilot_profile_t profile = make_gamepad(name, vendor, product, hint);
    profile.axis_roll = ABS_X;
    profile.axis_pitch = ABS_Y;
    profile.axis_throttle = ABS_Z;
    profile.axis_yaw = ABS_RX;
    profile.invert_pitch = false;
    profile.invert_throttle = false;
    profile.throttle_mode = PILOT_THROTTLE_DIRECT;
    profile.deadzone = 0.02;
    profile.expo = 0.0;
    profile.button_arm = PILOT_NONE;
    profile.button_mode = PILOT_NONE;
    profile.axis_arm = ABS_RY;
    profile.axis_mode = ABS_RZ;
    return profile;
}

typedef struct {
    const char *name;
    uint16_t vendor;
    uint16_t product;
    const char *hint;
    bool radio;
} descriptor_t;

static const descriptor_t DESCRIPTORS[] = {
    {"Sony DualSense", 0x054C, 0x0CE6, "DualSense", false},
    {"Sony DualSense Edge", 0x054C, 0x0DF2, "DualSense Edge", false},
    {"Sony DualShock 4", 0x054C, 0x09CC, "Wireless Controller", false},
    {"Sony DualShock 4 (first generation)", 0x054C, 0x05C4, "Wireless Controller", false},
    {"Xbox One controller", 0x045E, 0x02EA, "Xbox", false},
    {"Xbox One controller (Bluetooth)", 0x045E, 0x02FD, "Xbox", false},
    {"Xbox Series controller", 0x045E, 0x0B12, "Xbox", false},
    {"Xbox Series controller (Bluetooth)", 0x045E, 0x0B13, "Xbox", false},
    {"Logitech F310 (DirectInput)", 0x046D, 0xC216, "F310", false},
    {"Logitech F310 (XInput)", 0x046D, 0xC21D, "F310", false},
    {"Logitech F710 (DirectInput)", 0x046D, 0xC219, "F710", false},
    {"Logitech F710 (XInput)", 0x046D, 0xC21F, "F710", false},
    {"EdgeTX radio (USB joystick)", 0, 0, "EdgeTX", true},
    {"OpenTX radio (USB joystick)", 0, 0, "OpenTX", true},
    {"RadioMaster radio (USB joystick)", 0, 0, "RadioMaster", true},
};

static pilot_profile_t build(const descriptor_t *descriptor) {
    return descriptor->radio ? make_radio(descriptor->name, descriptor->vendor, descriptor->product, descriptor->hint)
                             : make_gamepad(descriptor->name, descriptor->vendor, descriptor->product, descriptor->hint);
}

int pilot_profile_count(void) {
    return (int)(sizeof(DESCRIPTORS) / sizeof(DESCRIPTORS[0]));
}

bool pilot_profile_at(int index, pilot_profile_t *out) {
    if (index < 0 || index >= pilot_profile_count() || out == NULL) {
        return false;
    }
    *out = build(&DESCRIPTORS[index]);
    return true;
}

static bool contains_nocase(const char *haystack, const char *needle) {
    size_t need = strlen(needle);
    size_t have = strlen(haystack);
    if (need == 0 || need > have) {
        return false;
    }
    for (size_t i = 0; i + need <= have; i++) {
        size_t j = 0;
        while (j < need && tolower((unsigned char)haystack[i + j]) == tolower((unsigned char)needle[j])) {
            j++;
        }
        if (j == need) {
            return true;
        }
    }
    return false;
}

bool pilot_profile_for_device(uint16_t vendor, uint16_t product, const char *name, pilot_profile_t *out) {
    if (out == NULL) {
        return false;
    }
    for (int i = 0; i < pilot_profile_count(); i++) {
        if (DESCRIPTORS[i].vendor != 0 && DESCRIPTORS[i].vendor == vendor && DESCRIPTORS[i].product == product) {
            *out = build(&DESCRIPTORS[i]);
            return true;
        }
    }
    if (name != NULL) {
        for (int i = 0; i < pilot_profile_count(); i++) {
            if (DESCRIPTORS[i].vendor == 0 && contains_nocase(name, DESCRIPTORS[i].hint)) {
                *out = build(&DESCRIPTORS[i]);
                return true;
            }
        }
    }
    return false;
}

void pilot_profile_generic(pilot_profile_t *out) {
    *out = make_gamepad("Generic gamepad", 0, 0, "");
}

static int copy_out(char *out, size_t cap, const char *text) {
    size_t len = strlen(text);
    if (out == NULL || cap == 0 || len >= cap) {
        return -1;
    }
    memcpy(out, text, len + 1U);
    return (int)len;
}

static void set_error(char *error, size_t error_len, const char *message, int line) {
    char local[96];
    (void)snprintf(local, sizeof(local), "line %d: %.60s", line, message);
    if (error != NULL && error_len > 0) {
        size_t len = strlen(local);
        size_t keep = len < error_len - 1U ? len : error_len - 1U;
        memcpy(error, local, keep);
        error[keep] = '\0';
    }
}

static bool parse_bool(const char *value, bool *out) {
    if (strcmp(value, "true") == 0 || strcmp(value, "1") == 0 || strcmp(value, "yes") == 0) {
        *out = true;
        return true;
    }
    if (strcmp(value, "false") == 0 || strcmp(value, "0") == 0 || strcmp(value, "no") == 0) {
        *out = false;
        return true;
    }
    return false;
}

static bool parse_number(const char *value, double *out) {
    char *end = NULL;
    double number = strtod(value, &end);
    if (end == value || *end != '\0' || number != number) {
        return false;
    }
    *out = number;
    return true;
}

static int assign_axis(const char *value, uint16_t *out) {
    if (strcmp(value, "none") == 0) {
        *out = PILOT_NONE;
        return 0;
    }
    int code = pilot_axis_code(value);
    if (code < 0) {
        return -1;
    }
    *out = (uint16_t)code;
    return 0;
}

static int assign_button(const char *value, uint16_t *out) {
    if (strcmp(value, "none") == 0) {
        *out = PILOT_NONE;
        return 0;
    }
    int code = pilot_button_code(value);
    if (code < 0) {
        return -1;
    }
    *out = (uint16_t)code;
    return 0;
}

typedef struct {
    const char *key;
    size_t offset;
} field_t;

#define FIELD(name) {#name, offsetof(pilot_profile_t, name)}

static const field_t AXIS_FIELDS[] = {
    FIELD(axis_roll), FIELD(axis_pitch), FIELD(axis_yaw),    FIELD(axis_throttle),
    FIELD(axis_arm),  FIELD(axis_mode),
};
static const field_t BUTTON_FIELDS[] = {FIELD(button_arm), FIELD(button_mode)};
static const field_t BOOL_FIELDS[] = {
    FIELD(invert_roll), FIELD(invert_pitch), FIELD(invert_yaw), FIELD(invert_throttle),
};
static const field_t NUMBER_FIELDS[] = {FIELD(deadzone), FIELD(expo), FIELD(throttle_rate)};

static const field_t *find_field(const field_t *table, size_t count, const char *key) {
    for (size_t i = 0; i < count; i++) {
        if (strcmp(table[i].key, key) == 0) {
            return &table[i];
        }
    }
    return NULL;
}

static int apply_numeric_id(pilot_profile_t *profile, const char *key, const char *value) {
    char *end = NULL;
    long id = strtol(value, &end, 0);
    if (end == value || *end != '\0' || id < 0 || id > 0xFFFF) {
        return -1;
    }
    if (strcmp(key, "vendor") == 0) {
        profile->vendor = (uint16_t)id;
    } else {
        profile->product = (uint16_t)id;
    }
    return 0;
}

static int apply_mode(pilot_profile_t *profile, const char *value) {
    static const char *const MODES[] = {"direct", "integrate", "trigger"};
    for (int i = 0; i < 3; i++) {
        if (strcmp(value, MODES[i]) == 0) {
            profile->throttle_mode = (pilot_throttle_mode_t)i;
            return 0;
        }
    }
    return -1;
}

static int apply_table_key(pilot_profile_t *profile, const char *key, const char *value) {
    char *base = (char *)profile;
    const field_t *field = find_field(AXIS_FIELDS, sizeof(AXIS_FIELDS) / sizeof(AXIS_FIELDS[0]), key);
    if (field != NULL) {
        return assign_axis(value, (uint16_t *)(void *)(base + field->offset));
    }
    field = find_field(BUTTON_FIELDS, sizeof(BUTTON_FIELDS) / sizeof(BUTTON_FIELDS[0]), key);
    if (field != NULL) {
        return assign_button(value, (uint16_t *)(void *)(base + field->offset));
    }
    field = find_field(BOOL_FIELDS, sizeof(BOOL_FIELDS) / sizeof(BOOL_FIELDS[0]), key);
    if (field != NULL) {
        return parse_bool(value, (bool *)(void *)(base + field->offset)) ? 0 : -1;
    }
    field = find_field(NUMBER_FIELDS, sizeof(NUMBER_FIELDS) / sizeof(NUMBER_FIELDS[0]), key);
    if (field != NULL) {
        double number = 0.0;
        double limit = strcmp(key, "throttle_rate") == 0 ? MAX_RATE : 1.0;
        if (!parse_number(value, &number) || number < 0.0 || number > limit) {
            return -1;
        }
        *(double *)(void *)(base + field->offset) = number;
        return 0;
    }
    return 1;
}

static int apply_key(pilot_profile_t *profile, const char *key, const char *value) {
    int status = apply_table_key(profile, key, value);
    if (status <= 0) {
        return status;
    }
    if (strcmp(key, "name") == 0) {
        (void)snprintf(profile->name, sizeof(profile->name), "%s", value);
    } else if (strcmp(key, "name_hint") == 0) {
        (void)snprintf(profile->name_hint, sizeof(profile->name_hint), "%s", value);
    } else if (strcmp(key, "vendor") == 0 || strcmp(key, "product") == 0) {
        return apply_numeric_id(profile, key, value);
    } else if (strcmp(key, "throttle_mode") == 0) {
        return apply_mode(profile, value);
    } else if (strncmp(key, "axis_aux", 8) == 0 && key[8] >= '1' && key[8] < '1' + PILOT_AUX_CHANNELS &&
               key[9] == '\0') {
        return assign_axis(value, &profile->axis_aux[key[8] - '1']);
    } else {
        return -1;
    }
    return 0;
}

int pilot_profile_parse(const char *text, pilot_profile_t *out, char *error, size_t error_len) {
    if (text == NULL || out == NULL) {
        return -1;
    }
    pilot_profile_t profile;
    pilot_profile_generic(&profile);
    (void)snprintf(profile.name, sizeof(profile.name), "%s", "Custom controller");
    profile.verified = false;
    int line_number = 0;
    const char *cursor = text;
    while (*cursor != '\0') {
        line_number++;
        const char *end = strchr(cursor, '\n');
        size_t len = end != NULL ? (size_t)(end - cursor) : strlen(cursor);
        char line[160];
        if (len >= sizeof(line)) {
            set_error(error, error_len, "line too long", line_number);
            return -1;
        }
        memcpy(line, cursor, len);
        line[len] = '\0';
        cursor += len + (end != NULL ? 1U : 0U);
        char *start = line;
        while (*start != '\0' && isspace((unsigned char)*start)) {
            start++;
        }
        if (*start == '\0' || *start == '#' || *start == '[') {
            continue;
        }
        char *equals = strchr(start, '=');
        if (equals == NULL) {
            set_error(error, error_len, "expected key = value", line_number);
            return -1;
        }
        *equals = '\0';
        char *key = start;
        char *value = equals + 1;
        for (char *tail = key + strlen(key); tail > key && isspace((unsigned char)tail[-1]); tail--) {
            tail[-1] = '\0';
        }
        while (*value != '\0' && isspace((unsigned char)*value)) {
            value++;
        }
        for (char *tail = value + strlen(value); tail > value && isspace((unsigned char)tail[-1]); tail--) {
            tail[-1] = '\0';
        }
        if (apply_key(&profile, key, value) != 0) {
            set_error(error, error_len, "unknown key or invalid value", line_number);
            return -1;
        }
    }
    *out = profile;
    return 0;
}

int pilot_profile_format(const pilot_profile_t *profile, char *out, size_t cap) {
    if (profile == NULL || out == NULL || cap == 0) {
        return -1;
    }
    char scratch[8][16];
    char text[1536];
    const size_t axis_count = sizeof(AXIS_NAMES) / sizeof(AXIS_NAMES[0]);
    const size_t button_count = sizeof(BUTTON_NAMES) / sizeof(BUTTON_NAMES[0]);
    static const char *const MODES[] = {"direct", "integrate", "trigger"};
    int written = snprintf(
        text, sizeof(text),
        "name = %s\nname_hint = %s\nvendor = 0x%04X\nproduct = 0x%04X\n"
        "axis_roll = %s\naxis_pitch = %s\naxis_yaw = %s\naxis_throttle = %s\n"
        "invert_roll = %s\ninvert_pitch = %s\ninvert_yaw = %s\ninvert_throttle = %s\n"
        "throttle_mode = %s\ndeadzone = %.3f\nexpo = %.3f\nthrottle_rate = %.3f\n"
        "button_arm = %s\nbutton_mode = %s\naxis_arm = %s\naxis_mode = %s\n",
        profile->name, profile->name_hint, (unsigned)profile->vendor, (unsigned)profile->product,
        code_to_name(AXIS_NAMES, axis_count, profile->axis_roll, scratch[0], sizeof(scratch[0])),
        code_to_name(AXIS_NAMES, axis_count, profile->axis_pitch, scratch[1], sizeof(scratch[1])),
        code_to_name(AXIS_NAMES, axis_count, profile->axis_yaw, scratch[2], sizeof(scratch[2])),
        code_to_name(AXIS_NAMES, axis_count, profile->axis_throttle, scratch[3], sizeof(scratch[3])),
        profile->invert_roll ? "true" : "false", profile->invert_pitch ? "true" : "false",
        profile->invert_yaw ? "true" : "false", profile->invert_throttle ? "true" : "false",
        MODES[profile->throttle_mode], profile->deadzone, profile->expo, profile->throttle_rate,
        code_to_name(BUTTON_NAMES, button_count, profile->button_arm, scratch[4], sizeof(scratch[4])),
        code_to_name(BUTTON_NAMES, button_count, profile->button_mode, scratch[5], sizeof(scratch[5])),
        code_to_name(AXIS_NAMES, axis_count, profile->axis_arm, scratch[6], sizeof(scratch[6])),
        code_to_name(AXIS_NAMES, axis_count, profile->axis_mode, scratch[7], sizeof(scratch[7])));
    if (written <= 0 || (size_t)written >= sizeof(text)) {
        return -1;
    }
    for (int i = 0; i < PILOT_AUX_CHANNELS; i++) {
        char aux_scratch[16];
        int more = snprintf(text + written, sizeof(text) - (size_t)written, "axis_aux%d = %s\n", i + 1,
                            code_to_name(AXIS_NAMES, axis_count, profile->axis_aux[i], aux_scratch,
                                         sizeof(aux_scratch)));
        if (more <= 0 || (size_t)more >= sizeof(text) - (size_t)written) {
            return -1;
        }
        written += more;
    }
    return copy_out(out, cap, text);
}
