/*
This is the c configuration file for the keymap

Copyright 2012 Jun Wako <wakojun@gmail.com>
Copyright 2015 Jack Humbert

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

//#define USE_MATRIX_I2C

//#define QUICK_TAP_TERM 0
//#define TAPPING_TERM 100

// For TT(layer): hold = momentary, double-tap = toggle.
#define TAPPING_TOGGLE 2

#ifdef RGBLIGHT_ENABLE
    #define RGBLIGHT_EFFECT_BREATHING
    #define RGBLIGHT_EFFECT_RAINBOW_MOOD
    #define RGBLIGHT_EFFECT_RAINBOW_SWIRL
    #define RGBLIGHT_EFFECT_SNAKE
    #define RGBLIGHT_EFFECT_KNIGHT
    #define RGBLIGHT_EFFECT_CHRISTMAS
    #define RGBLIGHT_EFFECT_STATIC_GRADIENT
    #define RGBLIGHT_EFFECT_RGB_TEST
    #define RGBLIGHT_EFFECT_ALTERNATING
    #define RGBLIGHT_EFFECT_TWINKLE
    #define RGBLIGHT_LIMIT_VAL 120
    #define RGBLIGHT_HUE_STEP 10
    #define RGBLIGHT_SAT_STEP 17
    #define RGBLIGHT_VAL_STEP 17
#endif


// --- Ergonomic timing (added) ---
#ifndef TAPPING_TERM
#    define TAPPING_TERM 180
#endif
#ifndef TAP_DANCE_TERM
#    define TAP_DANCE_TERM 200
#endif
#ifndef TAPPING_TOGGLE
#    define TAPPING_TOGGLE 2
#endif
#define PERMISSIVE_HOLD
#define SPLIT_LAYER_STATE_ENABLE


// --- RGB Matrix idle timeout ---
// Turn off all RGB Matrix LEDs after 10 minutes of inactivity
#ifdef RGB_MATRIX_ENABLE
#    define RGB_MATRIX_TIMEOUT 600000
#    define RGB_MATRIX_SLEEP
#endif

// --- Combos ---
#ifndef COMBO_TERM
#    define COMBO_TERM 35
#endif
