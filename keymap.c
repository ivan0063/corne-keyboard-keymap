/*
 * A specialized Corne layout for macOS, heavy terminal use, and IntelliJ IDEA for Java development.
 *
 * Layer Overview (v2):
 * - _BASE:   Standard typing layer with Home Row Mods.
 * - _NAV:    Navigation/editing layer (momentary on hold; toggle on double-tap via TT()).
 * - _SYM:    Programming symbols (momentary only).
 * - _FUN:  macOS commands/window management (momentary).
 * - _NUM:    Numpad for data entry (toggle-friendly).
 * - _FUN: Tri-layer (NAV + SYM) for settings/media.
 */

#include QMK_KEYBOARD_H

// Function-based layer names
enum layers {
  _BASE,
  _NAV,
  _SYM,
  _NUM,
  _FUN
};

// Define custom keycodes for advanced features

/* =========================
 * Tap Dance (optional)
 * If TAP_DANCE_ENABLE is not set in rules.mk, the keymap will still compile,
 * but the tap-dance keys will fall back to their single-tap versions.
 * ========================= */
#ifdef TAP_DANCE_ENABLE
#include "process_tap_dance.h"

enum {
    TD_EQEQ = 0,   // =  -> ==
    TD_NEQ,        // !  -> !=
    TD_ARROW,      // -  -> ->
    TD_FATARROW,   // =  -> =>
    TD_PIPE2,      // |  -> ||
    TD_AND2        // &  -> &&
};

tap_dance_action_t tap_dance_actions[] = {
    [TD_EQEQ]     = ACTION_TAP_DANCE_DOUBLE(KC_EQL, KC_EQL),
    [TD_NEQ]      = ACTION_TAP_DANCE_DOUBLE(KC_EXLM, KC_EQL),
    [TD_ARROW]    = ACTION_TAP_DANCE_DOUBLE(KC_MINS, KC_GT),
    [TD_FATARROW] = ACTION_TAP_DANCE_DOUBLE(KC_EQL, KC_GT),
    [TD_PIPE2]    = ACTION_TAP_DANCE_DOUBLE(KC_PIPE, KC_PIPE),
    [TD_AND2]     = ACTION_TAP_DANCE_DOUBLE(KC_AMPR, KC_AMPR),
};

#    define TD_EQEQ_K     TD(TD_EQEQ)
#    define TD_NEQ_K      TD(TD_NEQ)
#    define TD_ARROW_K    TD(TD_ARROW)
#    define TD_FATARROW_K TD(TD_FATARROW)
#    define TD_PIPE2_K    TD(TD_PIPE2)
#    define TD_AND2_K     TD(TD_AND2)
#else
// Fallbacks when tap-dance is disabled
#    define TD_EQEQ_K     KC_EQL
#    define TD_NEQ_K      KC_EXLM
#    define TD_ARROW_K    KC_MINS
#    define TD_FATARROW_K KC_EQL
#    define TD_PIPE2_K    KC_PIPE
#    define TD_AND2_K     KC_AMPR
#endif

enum custom_keycodes {
    // IntelliJ specific
    IJ_F_ACTION = SAFE_RANGE,
    IJ_GOTO_C,
    IJ_GOTO_F,
    IJ_REFACTOR,
    IJ_RUN,
    IJ_DEBUG,
    IJ_TERM,

    // macOS specific
    MC_TAB,
    MC_SPOT,
    MC_MISSION,
    MC_SS_MENU,
    MC_SS_SEL,   // Screenshot Selection (Cmd+Shift+4)  <-- SOLO UNA VEZ
    MC_HIDE,
    MC_QUIT,
    MC_FORCE_Q,
    MC_AI,
    MC_SS_FULL
};

// Use standard US keyboard layout for macOS
#define US_GRV  KC_GRV
#define US_MINS KC_MINS
#define US_EQL  KC_EQL
#define US_LBRC KC_LBRC
#define US_RBRC KC_RBRC
#define US_BSLS KC_BSLS
#define US_SCLN KC_SCLN
#define US_QUOT KC_QUOT

// Dual-role keys
#define CTL_ESC MT(MOD_LCTL, KC_ESC)
#define SFT_TAB MT(MOD_LSFT, KC_TAB)

// Home row mods (requested): tap = letter, hold = modifier
// Left:  S=Ctrl, D=Option(Alt), F=Command(Gui)
// Right: J=Command(Gui), K=Option(Alt), L=Ctrl
//
// Notes:
// - We keep A and ; as plain keys to avoid overloading too many letters at once.
#define HOME_A    KC_A
#define HOME_S    LCTL_T(KC_S)
#define HOME_D    LALT_T(KC_D)
#define HOME_F    LGUI_T(KC_F)

#define HOME_J    RGUI_T(KC_J)
#define HOME_K    RALT_T(KC_K)
#define HOME_L    RCTL_T(KC_L)
#define HOME_SCLN KC_SCLN

// Thumb row: universal positions, but layer targets can vary per-layer.
// Left:  Caps Lock | Layer Move (varies) | Enter
// Right: Space     | Layer Move (varies) | Option(Alt)
// (same on every layer)
// Left:  Ctrl | Layer Move | Enter
// Right: Space| Layer Move | Option
//
// Layer behavior intent:
// - NAV: makes sense to "stay" sometimes -> TT() (hold = momentary, double-tap = toggle)
// - SYM: usually one-shot/momentary -> MO()
#define TH_CAPS KC_CAPS
#define TH_ENT  KC_ENT
#define TH_SPC  KC_SPC
#define TH_OPT  KC_LALT

// Per-layer thumb "Layer Move" keys
//
// Design intent:
// - Left thumb "Layer Move": primary navigation layer, toggle-friendly (TT) in BASE.
// - Right thumb "Layer Move": primary symbol layer, momentary (MO) in BASE.
// - In NAV/NUM/SYM: provide clear exits back to BASE via TO(_BASE) while keeping a bridge between NAV <-> SYM.
// - NAV + SYM together = FUN (tri-layer) for media/system/RGB.
#define TH_LM_BASE TT(_NAV)
#define TH_RM_BASE MO(_SYM)

#define TH_LM_NAV  TO(_BASE)
#define TH_RM_NAV  TT(_NUM)

#define TH_LM_SYM  MO(_NAV)
#define TH_RM_SYM  TO(_BASE)

#define TH_LM_NUM  TO(_BASE)
#define TH_RM_NUM  MO(_SYM)

#define TH_LM_FUN  TO(_BASE)
#define TH_RM_FUN  TO(_BASE)


// Layer switching helpers

const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {
/*
 * _BASE: Default Typing Layer
 */
[_BASE] = LAYOUT_split_3x6_3(
    SFT_TAB,   KC_Q,    KC_W,    KC_E,    KC_R,    KC_T,       KC_Y,    KC_U,    KC_I,    KC_O,    KC_P,    KC_BSPC,
    CTL_ESC, HOME_A,  HOME_S,  HOME_D,  HOME_F,    KC_G,       KC_H,    HOME_J,  HOME_K,  HOME_L, HOME_SCLN, US_QUOT,
    KC_LSFT,   KC_Z,    KC_X,    KC_C,    KC_V,    KC_B,       KC_N,    KC_M,    KC_COMM, KC_DOT,  KC_SLSH, KC_RSFT,
                    TH_CAPS, TH_LM_BASE, TH_ENT,                       TH_SPC, TH_RM_BASE, TH_OPT
),

/*
 * _NAV: Navigation & Editing (toggle-friendly)
 */
[_NAV] = LAYOUT_split_3x6_3(
KC_ESC,  KC_HOME, KC_UP,   KC_END,  KC_PGUP, KC_DEL,     KC_NO,   KC_HOME, KC_UP,   KC_END,  KC_PGUP, KC_DEL,
    KC_TAB,  KC_LEFT, KC_DOWN, KC_RGHT, KC_PGDN, KC_BSPC,    KC_NO,   LSFT(KC_LEFT), LSFT(KC_DOWN), LSFT(KC_UP), LSFT(KC_RGHT), KC_BSPC,
    KC_LSFT, LALT(KC_LEFT), LALT(KC_RGHT), LGUI(KC_LEFT), LGUI(KC_RGHT), KC_ENT,  LGUI(KC_Z), LGUI(KC_X), LGUI(KC_C), LGUI(KC_V), LGUI(KC_S), KC_RSFT,
                    TH_CAPS, TH_LM_NAV, TH_ENT,                       TH_SPC, TH_RM_NAV, TH_OPT
),

/*
 * _SYM: Programming Symbols (momentary)
 */
[_SYM] = LAYOUT_split_3x6_3(
    // Programming + terminal symbols (with optional tap-dance)
    KC_LPRN, KC_LCBR, KC_LBRC, KC_LT,   TD_EQEQ_K, TD_NEQ_K,     KC_NO,   KC_DLR,  KC_TILD, KC_ASTR, TD_AND2_K, KC_BSPC,
    TD_ARROW_K, KC_UNDS, KC_PLUS, KC_SLSH, KC_BSLS, TD_PIPE2_K,  KC_NO,   KC_QUES, KC_COLN, KC_SCLN, KC_COMM,  TD_FATARROW_K,
    KC_GRV,  KC_EXLM, KC_AT,   KC_HASH, KC_PERC, KC_CIRC,       KC_NO,   KC_DQUO, KC_QUOT, KC_DOT,  KC_MINS,  KC_RPRN,
                        TH_CAPS, TH_LM_SYM, TH_ENT,                       TH_SPC, TH_RM_SYM, TH_OPT
),

/*
 * _NUM: Numpad Layer (toggle-friendly)
 */
[_NUM] = LAYOUT_split_3x6_3(
    KC_ESC,  KC_7,    KC_8,    KC_9,    KC_PMNS, KC_PPLS,    KC_NO,   KC_7,    KC_8,    KC_9,    KC_PMNS, KC_PPLS,
    KC_TAB,  KC_4,    KC_5,    KC_6,    KC_PAST, KC_PSLS,    KC_NO,   KC_4,    KC_5,    KC_6,    KC_PAST, KC_PSLS,
    KC_LSFT, KC_1,    KC_2,    KC_3,    KC_0,    KC_DOT,     KC_NO,   KC_1,    KC_2,    KC_3,    KC_0,    KC_ENT,
                    TH_CAPS, TH_LM_NUM, TH_ENT,                       TH_SPC, TH_RM_NUM, TH_OPT
),

/*
 * _FUN: Keyboard Settings & Media Layer
 */
[_FUN] = LAYOUT_split_3x6_3(
    QK_BOOT, KC_F1,   KC_F2,   KC_F3,   KC_F4,   KC_F5,         KC_F6,   KC_F7,   KC_F8,   KC_F9,   KC_F10,  KC_F11,
    RGB_TOG, RGB_MOD, RGB_HUI, RGB_SAI, RGB_VAI, KC_F12,        KC_MPRV, KC_MPLY, KC_MNXT, KC_VOLD, KC_VOLU, KC_MUTE,
    MC_SPOT, MC_MISSION, MC_SS_MENU, MC_SS_SEL, MC_TAB, KC_NO,  IJ_F_ACTION, IJ_GOTO_C, IJ_GOTO_F, IJ_REFACTOR, IJ_RUN, IJ_DEBUG,
                        TH_CAPS, TH_LM_FUN, TH_ENT,                       TH_SPC, TH_RM_FUN, TH_OPT
)

};

// Handle custom keycodes
bool process_record_user(uint16_t keycode, keyrecord_t *record) {
    if (record->event.pressed) {
        switch (keycode) {
            case IJ_F_ACTION: SEND_STRING(SS_LGUI(SS_LSFT("a"))); return false;
            case IJ_GOTO_C:   SEND_STRING(SS_LGUI("o")); return false;
            case IJ_GOTO_F:   SEND_STRING(SS_LGUI(SS_LSFT("o"))); return false;
            case IJ_REFACTOR: SEND_STRING(SS_LCTL("t")); return false;
            case IJ_RUN:      SEND_STRING(SS_LCTL("r")); return false;
            case IJ_DEBUG:    SEND_STRING(SS_LCTL("d")); return false;
            case IJ_TERM:     SEND_STRING(SS_LALT(SS_TAP(X_F12))); return false;
            case MC_SPOT:     SEND_STRING(SS_LGUI(SS_TAP(X_SPC))); return false;
            case MC_MISSION:  SEND_STRING(SS_LCTL(SS_TAP(X_UP))); return false;
            case MC_SS_MENU:  SEND_STRING(SS_LGUI(SS_LSFT("5"))); return false;
            case MC_SS_FULL:  SEND_STRING(SS_LGUI(SS_LSFT("3"))); return false;
            case MC_SS_SEL:   SEND_STRING(SS_LGUI(SS_LSFT("4"))); return false;
            case MC_HIDE:     SEND_STRING(SS_LGUI("h")); return false;
            case MC_QUIT:     SEND_STRING(SS_LGUI("q")); return false;
            case MC_FORCE_Q:  SEND_STRING(SS_LGUI(SS_LALT(SS_TAP(X_ESC)))); return false;
            case MC_AI:       tap_code(KC_LGUI); tap_code(KC_LGUI); return false;
            case MC_TAB:      register_code(KC_LGUI); tap_code(KC_TAB); return false;
        }
    } else { // on keyup
        switch (keycode) {
            case MC_TAB:
                unregister_code(KC_LGUI);
                return false;
        }
    }
    return true;
}

#ifdef RGB_MATRIX_ENABLE
void set_layer_color(int layer) {
    switch (layer) {
        case _BASE:   rgb_matrix_sethsv_noeeprom(HSV_AZURE); break;
        case _NAV:    rgb_matrix_sethsv_noeeprom(HSV_WHITE); break;
        case _SYM:    rgb_matrix_sethsv_noeeprom(HSV_GOLD); break;
        case _NUM:    rgb_matrix_sethsv_noeeprom(HSV_GREEN); break;
        case _FUN: rgb_matrix_sethsv_noeeprom(HSV_MAGENTA); break;
        default:       rgb_matrix_sethsv_noeeprom(HSV_WHITE); break;
    }
}
layer_state_t layer_state_set_user(layer_state_t state) {
    // Tri-layer: when NAV + SYM are active, turn on ADJUST.
    state = update_tri_layer_state(state, _NAV, _SYM, _FUN);
    set_layer_color(get_highest_layer(state));
    return state;
}
void keyboard_post_init_user(void) {
    set_layer_color(_BASE);
}
#endif // RGB_MATRIX_ENABLE


// --- FIX STARTS HERE ---
// Wrap the OLED and Encoder functions to prevent "multiple definition" errors.
#if defined(OLED_ENABLE) && defined(OLED_DRIVER_SSD1306)

static void render_layer_state(void) {
    oled_set_cursor(0, 0);
    oled_write_P(PSTR("Layer:"), false);
    oled_set_cursor(0, 1);
    switch (get_highest_layer(layer_state | default_layer_state)) {
        case _NAV: oled_write_P(PSTR(" NAV"), false); break;
        case _SYM: oled_write_P(PSTR(" SYM"), false); break;
        case _NUM: oled_write_P(PSTR(" NUM"), false); break;
        case _FUN: oled_write_P(PSTR(" FUN"), false); break;
        default:   oled_write_P(PSTR(" BASE"), false); break;
    }
}

static void render_mods(void) {
    oled_set_cursor(0, 3);
    uint8_t mods = get_mods() | get_oneshot_mods();
    oled_write_P(PSTR("Mods:"), false);
    oled_set_cursor(0, 4);
    oled_write_P((mods & MOD_MASK_GUI)  ? PSTR("G ") : PSTR("- "), false);
    oled_write_P((mods & MOD_MASK_ALT)  ? PSTR("A ") : PSTR("- "), false);
    oled_write_P((mods & MOD_MASK_CTRL) ? PSTR("C ") : PSTR("- "), false);
    oled_write_P((mods & MOD_MASK_SHIFT)? PSTR("S")  : PSTR("-"), false);
}

bool oled_task_kb(void) {
    if (!oled_task_user()) { return false; }

    oled_clear();
    oled_set_cursor(0, 6);
    oled_write_P(PSTR("final_gemini"), false);

    render_layer_state();
    render_mods();
    return false;
}

#endif // OLED_ENABLE



#ifdef ENCODER_ENABLE
bool encoder_update_kb(uint8_t index, bool clockwise) {
    if (!encoder_update_user(index, clockwise)) { return false; }
    if (index == 0) {
        if (clockwise) { tap_code(KC_VOLU); } else { tap_code(KC_VOLD); }
    } else if (index == 1) {
        if (clockwise) { tap_code(KC_PGDN); } else { tap_code(KC_PGUP); }
    }
    return true;
}
#endif // ENCODER_ENABLE

#ifdef COMBO_ENABLE
const uint16_t PROGMEM ss_sel_combo[]  = {KC_E, KC_R, COMBO_END};
const uint16_t PROGMEM ss_full_combo[] = {KC_I, KC_O, COMBO_END};
const uint16_t PROGMEM ss_menu_combo[] = {KC_T, KC_Y, COMBO_END};

combo_t key_combos[] = {
    COMBO(ss_sel_combo,  MC_SS_SEL),
    COMBO(ss_full_combo, MC_SS_FULL),
    COMBO(ss_menu_combo, MC_SS_MENU),
};
#endif
