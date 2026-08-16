#!/usr/bin/python3
# -*- coding: utf-8 -*-
# ******************************************************************************
# ZYNTHIAN PROJECT: Zynthian Control Device Driver
#
# Zynthian Control Device Driver for "Akai Fire"
#
# Copyright (C) 2023-2025 Oscar Aceña <oscaracena@gmail.com>
#
# ******************************************************************************
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License as
# published by the Free Software Foundation; either version 2 of
# the License, or any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#
# For a full copy of the GNU General Public License see the LICENSE.txt file.
#
# ******************************************************************************
#
# See zynthian_ctrldev_akai_fire_protocol.md (same directory) for the reverse
# engineered MIDI/SysEx protocol this driver is based on, including what is
# solid vs. what still needs confirming against real hardware.
#
# This first version implements three modes, auto-switched by the current
# zynthian screen: "Device" (generic V5 4-knob navigation plus a full pad-grid
# button matrix - the fallback for any screen not covered by another mode, or
# forced on via Alt+Browser regardless of screen), "Mixer" (audio_mixer
# screen), and "Zynpad" (zynpad screen - sequence/clip launcher on the pad
# grid). The OLED is intentionally untouched for now. Step/Note/Drum are
# unbound, reserved for future modes (step-sequencer, note-pads, drum view).
#
# ******************************************************************************

import time
import logging

from zynlibs.zynseq import zynseq
from zyngine.ctrldev.zynthian_ctrldev_base import zynthian_ctrldev_zynmixer, zynthian_ctrldev_zynpad
from zyngine.ctrldev.zynthian_ctrldev_base_extended import ButtonTimer, CONST
from zyngine.ctrldev.zynthian_ctrldev_base_ui import ModeHandlerBase
from zyngine.zynthian_signal_manager import zynsigman
from zyncoder.zyncore import lib_zyncore
from zyngui import zynthian_gui_config


# MIDI channel events (first 4 bits), next 4 bits is the channel
EV_NOTE_ON = 0x09
EV_NOTE_OFF = 0x08
EV_CC = 0x0B
EV_SYSEX = 0xF0

# Buttons (Note On/Off, channel 0)
# Capacitive touch on the 4 channel-strip knobs - documented but currently
# unused: touch fires on any contact (e.g. just resting a finger on the knob
# to turn it), so it can't reliably stand in for a deliberate press. Solo 1-4
# are used instead (see DeviceHandler.ZYNPOT_SWITCH_BTNS).
BTN_VOLUME_TOUCH = 0x10
BTN_PAN_TOUCH = 0x11
BTN_FILTER_TOUCH = 0x12
BTN_RESONANCE_TOUCH = 0x13
BTN_SELECT_PRESS = 0x19
BTN_BANK = 0x1A		# top-left corner button, labelled "Bank/Mode" on the device
BTN_PAT_UP = 0x1F
BTN_PAT_DOWN = 0x20
BTN_BROWSER = 0x21
BTN_GRID_LEFT = 0x22
BTN_GRID_RIGHT = 0x23
BTN_SOLO_1 = 0x24
BTN_SOLO_2 = 0x25
BTN_SOLO_3 = 0x26
BTN_SOLO_4 = 0x27
BTN_STEP = 0x2C		# unbound for now, reserved for a future step-sequencer mode
BTN_NOTE = 0x2D		# unbound for now, reserved for a future note-pad mode
BTN_DRUM = 0x2E		# unbound for now, reserved for a future drum mode
BTN_PERFORM = 0x2F		# toggles between the Mixer and Zynpad screens
BTN_SHIFT = 0x30
BTN_ALT = 0x31
BTN_PATTERN_SONG = 0x32	# unbound for now
BTN_PLAY = 0x33
BTN_STOP = 0x34
BTN_RECORD = 0x35

# Knobs: relative CC, channel 0, two's-complement encoding
KNOB_VOLUME = 0x10
KNOB_PAN = 0x11
KNOB_FILTER = 0x12
KNOB_RESONANCE = 0x13
KNOB_SELECT = 0x76

# Fixed left-to-right order of the 4 "channel strip" knobs (CC/rotate only) -
# the default zynpot mapping any mode can fall back to via ZynpotRotate.
ZYNPOT_KNOBS = {
    KNOB_VOLUME: 0,
    KNOB_PAN: 1,
    KNOB_FILTER: 2,
    KNOB_RESONANCE: 3,
}

# Output-only LED addresses (channel 0, CC message; value = color, see below)
# For "plain" buttons, LED feedback is CC == the button's own note number.
# Bank/Mode and the Solo strip are exceptions, with their own dedicated addresses.
LED_BANK = 0x1B
LED_SOLO_1 = 0x28
LED_SOLO_2 = 0x29
LED_SOLO_3 = 0x2A
LED_SOLO_4 = 0x2B
LED_BROWSER = BTN_BROWSER
LED_PERFORM = BTN_PERFORM

# Button LED color values, confirmed from the SEGGER blog's decoding series
# (see "Color palette" in zynthian_ctrldev_akai_fire_protocol.md). Each button
# LED is wired to ONE fixed color family - sending a value outside a button's
# own family is unverified/undefined, so these are grouped accordingly.
LED_OFF = 0

# Red-only: Pattern Up/Down, Browser, Grid Left/Right
LED_RED_DULL = 1
LED_RED_HIGH = 2

# Green-only: the Solo-strip addresses (LED_SOLO_1-4)
LED_GREEN_DULL = 1
LED_GREEN_HIGH = 2

# Yellow-red: Step, Note, Drum, Perform, Shift, Record
LED_YR_HIGH_YELLOW = 3
LED_YR_HIGH_RED = 4

# Bank/Mode's own LED (0x1B) isn't in the blog's table - still unverified,
# kept as a generic on/off placeholder (see protocol doc).
LED_ON = 1

# Pad grid: 4 rows x 16 cols = 64 RGB pads, Note On/Off channel 0, notes 0x36-0x75
# (54-117). pad_index = note - 54, row-major, one 16-note block per physical row.
# Which physical row is row 0 (top or bottom) is not confirmed yet - doesn't affect
# correctness, only whether this layout reads top-to-bottom or is visually flipped
# on the real device (swap the row order below if so, once verified).
PAD_NOTE_BASE = 54


def _pad(row, col):
    return PAD_NOTE_BASE + row * 16 + col


# Device-mode button matrix, hosted on the otherwise-idle pad grid - mirrors
# zynthian_ctrldev_akai_apc_key25_mk2.py's DeviceHandler pad layout, adapted to
# Fire's 4 rows (APC's equivalent uses 5):
#   row 0: screen-access (Admin, Mixer, Preset, ZS3), Metronome, Zynseq
#   row 1: Alt, Play, Stop, Record, F1-F4
#   rows 2-3: 2x3 direction block - [Back/No, Up, Sel/Yes] over [Left, Down, Right],
#             same shape as APC's own BACK_NO/UP/SEL_YES over LEFT/DOWN/RIGHT block
PAD_ADMIN = _pad(0, 0)
PAD_MIXER = _pad(0, 1)
PAD_PRESET = _pad(0, 2)
PAD_ZS3 = _pad(0, 3)
PAD_METRONOME = _pad(0, 4)
PAD_ZYNSEQ = _pad(0, 5)

PAD_ALT = _pad(1, 0)
PAD_PLAY = _pad(1, 1)
PAD_STOP = _pad(1, 2)
PAD_RECORD = _pad(1, 3)
PAD_F1 = _pad(1, 4)
PAD_F2 = _pad(1, 5)
PAD_F3 = _pad(1, 6)
PAD_F4 = _pad(1, 7)

PAD_BACK = _pad(2, 0)
PAD_UP = _pad(2, 1)
PAD_SELECT = _pad(2, 2)
PAD_LEFT = _pad(3, 0)
PAD_DOWN = _pad(3, 1)
PAD_RIGHT = _pad(3, 2)


def _alt_mode():
    """Zynthian's persistent, global "alt mode" toggle (Bank/Mode button /
    TOGGLE_ALT_MODE CUIA, see zyngui/zynthian_gui.py's alt_mode attribute) -
    read live from the GUI singleton rather than tracked locally per-handler,
    so it can't go stale if toggled some other way than through this driver.
    NOT the same thing as momentarily holding Fire's own physical Alt button
    (BTN_ALT) - that's a separate, unrelated modifier (see e.g. Alt+Solo-N in
    MixerHandler)."""
    return zynthian_gui_config.zyngui.get_alt_mode()


def _toggle_alt_mode():
    """Flips alt_mode directly instead of through the TOGGLE_ALT_MODE CUIA -
    state_manager.send_cuia() only enqueues the call (self.cuia_queue is
    drained later, elsewhere), so a repaint immediately after sending it would
    still read the pre-toggle value. zynthian_gui.cuia_toggle_alt_mode() is a
    pure attribute flip with no other side effects, so this reproduces it
    exactly, just synchronously - callers can safely repaint right after."""
    zynthian_gui_config.zyngui.alt_mode = not _alt_mode()


# --------------------------------------------------------------------------
# Feedback LEDs controller
# --------------------------------------------------------------------------
class FeedbackLEDs:
    def __init__(self, idev):
        self._idev = idev

    def all_off(self):
        self.control_leds_off()

    def control_leds_off(self):
        for led in (LED_BANK, LED_SOLO_1, LED_SOLO_2, LED_SOLO_3, LED_SOLO_4, LED_BROWSER, LED_PERFORM):
            self.led_off(led)

    def led_off(self, led):
        lib_zyncore.dev_send_ccontrol_change(self._idev, 0, led, LED_OFF)

    def led_on(self, led, value=LED_ON):
        lib_zyncore.dev_send_ccontrol_change(self._idev, 0, led, value)


# --------------------------------------------------------------------------
# Pad grid RGB LEDs (SysEx), see zynthian_ctrldev_akai_fire_protocol.md
# --------------------------------------------------------------------------
class PadLEDs:
    SYSEX_HEADER = (0xF0, 0x47, 0x7F, 0x43, 0x65)

    def __init__(self, idev):
        self._idev = idev

    def set_pad(self, note, r, g, b):
        index = note - PAD_NOTE_BASE
        payload = (index, r, g, b)
        msg = bytes(self.SYSEX_HEADER + (len(payload) // 128, len(payload) % 128) + payload + (0xF7,))
        lib_zyncore.dev_send_midi_event(self._idev, msg, len(msg))

    def all_off(self):
        # Single batched SysEx clearing all 64 pads, rather than tracking which
        # notes each mode happens to use and clearing only those (which breaks
        # the moment a new pad-owning mode is added and something gets missed).
        payload = []
        for index in range(64):
            payload.extend((index, 0, 0, 0))
        payload = tuple(payload)
        msg = bytes(self.SYSEX_HEADER + (len(payload) // 128, len(payload) % 128) + payload + (0xF7,))
        lib_zyncore.dev_send_midi_event(self._idev, msg, len(msg))

    def pad_off(self, note):
        self.set_pad(note, 0, 0, 0)


# --------------------------------------------------------------------------
# Smoothing for the Volume/Pan/Filter/Resonance knobs
# --------------------------------------------------------------------------
#
# The shared KnobSpeedControl (zynthian_ctrldev_base_extended) assumes noisy
# input arrives as occasional +-1 ticks that need several to add up before
# firing, and resets its accumulator to 0 on any direction reversal. Neither
# assumption holds for these knobs: on a light/slow turn they emit individual
# relative ticks already as large as +-4 (confirmed from a real capture), and
# reverse direction on nearly every other message - so a single noisy tick
# alone crosses KnobSpeedControl's threshold and fires (in whatever direction
# that tick happened to be), and its reset-on-reversal throws away what little
# accumulation existed instead of letting +/- ticks cancel out.
#
# This does the opposite: sum raw deltas with NO reset on reversal (so noise
# that's roughly balanced between directions cancels out over time), and use
# a threshold comfortably above the largest single-tick jitter observed, so no
# lone noisy message can fire by itself. Keeps the unused remainder across
# firings rather than zeroing it, so a sustained turn doesn't lose progress.
class KnobJitterFilter:
    def __init__(self, threshold=8):
        self._threshold = threshold
        self._acc = {}

    def feed(self, ccnum, ccval):
        delta = ccval if ccval < 64 else ccval - 128
        acc = self._acc.get(ccnum, 0) + delta
        if abs(acc) < self._threshold:
            self._acc[ccnum] = acc
            return None
        self._acc[ccnum] = acc - self._threshold if acc > 0 else acc + self._threshold
        return 1 if acc > 0 else -1

    def reset(self, ccnum):
        self._acc[ccnum] = 0


# --------------------------------------------------------------------------
# Volume/Pan/Filter/Resonance knob rotate -> zynpot 0-3. The default any mode
# falls back to when it has no more specific use for these 4 knobs (currently
# DeviceHandler and ZynpadHandler) - composed, not inherited, so each handler
# still owns its own note_on/cc_change and just delegates knob handling here.
# --------------------------------------------------------------------------
class ZynpotRotate:
    def __init__(self, state_manager):
        self._state_manager = state_manager
        self._knobs_ease = KnobJitterFilter()

    def cc_change(self, ccnum, ccval):
        zynpot = ZYNPOT_KNOBS.get(ccnum)
        if zynpot is None:
            return
        delta = self._knobs_ease.feed(ccnum, ccval)
        if delta is None:
            return
        self._state_manager.send_cuia("ZYNPOT", [zynpot, delta])
        return True

    def reset(self, ccnum):
        self._knobs_ease.reset(ccnum)


def _select_knob_arrow(state_manager, ccval):
    """Select knob rotate -> ARROW_LEFT/RIGHT, the default left/right
    list-navigation any mode falls back to when it has no more specific use
    for the Select knob (currently DeviceHandler and ZynpadHandler;
    MixerHandler keeps its own specialized raw chain-scroll instead - see
    its cc_change). Select's own encoder isn't noisy like the other 4, so
    no jitter filter here either - same as MixerHandler's."""
    delta = ccval if ccval < 64 else ccval - 128
    state_manager.send_cuia("ARROW_RIGHT" if delta > 0 else "ARROW_LEFT")


# --------------------------------------------------------------------------
# Handle GUI (generic screen navigation, active outside the audio mixer)
# --------------------------------------------------------------------------
class DeviceHandler(ModeHandlerBase):

    # zynpot "switch" (push) actions use the Solo buttons instead of knob touch:
    # touch is capacitive and fires on any contact (including just resting a
    # finger on the knob to turn it), so it can't stand in for a deliberate
    # press - Solo 1-4 give a real momentary button, same physical buttons
    # already doing solo/mute duty in Mixer mode.
    ZYNPOT_SWITCH_BTNS = {
        BTN_SOLO_1: 0,
        BTN_SOLO_2: 1,
        BTN_SOLO_3: 2,
        BTN_SOLO_4: 3,
    }

    # Screen-access pads: each cycles through a tuple of related CUIAs on
    # repeated short press, jumps to the "secondary" one on bold press - same
    # pattern as zynthian_ctrldev_akai_apc_key25_mk2.py's DeviceHandler.
    PAD_ACTIONS = {
        PAD_ADMIN: ("MENU", "SCREEN_ADMIN"),
        PAD_MIXER: ("SCREEN_AUDIO_MIXER", "SCREEN_ALSA_MIXER"),
        PAD_PRESET: ("SCREEN_CONTROL", "PRESET", "SCREEN_BANK"),
        PAD_ZS3: ("SCREEN_ZS3", "SCREEN_SNAPSHOT"),
        PAD_ZYNSEQ: ("SCREEN_ZYNPAD", "SCREEN_PATTERN_EDITOR"),
    }

    # Long-press shortcuts on those same pads, again matching APC's DeviceHandler.
    PAD_LONG_ACTIONS = {
        PAD_ADMIN: "POWER_OFF",
        PAD_PRESET: "PRESET_FAV",
        PAD_ZYNSEQ: "SCREEN_ARRANGER",
    }

    # Which screen lights up which screen-access pad (and at what state index,
    # so a following short press cycles on from there) - kept in sync via
    # on_screen_change(), called even while this handler isn't the active one.
    SCREEN_MAP = {
        "option":         (PAD_ADMIN, 0),
        "main_menu":      (PAD_ADMIN, 0),
        "admin":          (PAD_ADMIN, 1),
        "audio_mixer":    (PAD_MIXER, 0),
        "alsa_mixer":     (PAD_MIXER, 1),
        "control":        (PAD_PRESET, 0),
        "engine":         (PAD_PRESET, 0),
        "preset":         (PAD_PRESET, 1),
        "bank":           (PAD_PRESET, 2),
        "zs3":            (PAD_ZS3, 0),
        "snapshot":       (PAD_ZS3, 1),
        "zynpad":         (PAD_ZYNSEQ, 0),
        "pattern_editor": (PAD_ZYNSEQ, 1),
        "arranger":       (PAD_ZYNSEQ, 1),
    }

    # F1-F4 -> PROGRAM_CHANGE 1-4 (or 5-8 if alt is active), matching APC.
    FUNCTION_PADS = {
        PAD_F1: 1,
        PAD_F2: 2,
        PAD_F3: 3,
        PAD_F4: 4,
    }

    # Pads whose press needs short/bold/long distinction (routed through
    # _btn_timer); everything else is a direct single-action press.
    TIMED_PADS = {PAD_ADMIN, PAD_MIXER, PAD_PRESET, PAD_ZS3, PAD_ZYNSEQ, PAD_PLAY, PAD_STOP}

    # Fixed dim colors for the non-screen-access pads (PAD_ALT excluded - it's
    # painted separately in refresh(), reflecting on/off state). Real RGB, no
    # palette uncertainty here (unlike the button CCs).
    STATIC_PAD_COLORS = {
        PAD_METRONOME: (0, 40, 40),
        PAD_PLAY: (0, 50, 0),
        PAD_STOP: (50, 20, 0),
        PAD_RECORD: (60, 0, 0),
        PAD_F1: (30, 30, 30),
        PAD_F2: (30, 30, 30),
        PAD_F3: (30, 30, 30),
        PAD_F4: (30, 30, 30),
        PAD_BACK: (60, 0, 0),
        PAD_UP: (40, 40, 0),
        PAD_SELECT: (0, 60, 0),
        PAD_LEFT: (40, 40, 0),
        PAD_DOWN: (40, 40, 0),
        PAD_RIGHT: (40, 40, 0),
    }

    # 3-state coloring for screen-access pads, same scheme as APC's DeviceHandler
    # (COLOR_STATE_0/1/2: unselected / primary-selected / secondary-selected),
    # shared across all 5 pads rather than a per-pad hue.
    PAD_STATE_UNSELECTED = (0, 0, 40)   # dim blue, ~ APC's COLOR_STATE_0
    PAD_STATE_PRIMARY = (0, 60, 0)      # green, ~ APC's COLOR_STATE_1
    PAD_STATE_SECONDARY = (60, 30, 0)   # orange, ~ APC's COLOR_STATE_2

    def __init__(self, state_manager, leds: FeedbackLEDs, pads: PadLEDs):
        super().__init__(state_manager)
        self._leds = leds
        self._pads = pads
        self._btn_timer = ButtonTimer(self._handle_timed_button)
        self._zynpot = ZynpotRotate(state_manager)
        self._pad_states = {k: -1 for k in self.PAD_ACTIONS}

    def _pad_state_color(self, state):
        if state < 0:
            return self.PAD_STATE_UNSELECTED
        if state == 0:
            return self.PAD_STATE_PRIMARY
        return self.PAD_STATE_SECONDARY

    def refresh(self):
        self._leds.led_on(LED_BANK) if _alt_mode() else self._leds.led_off(LED_BANK)

        for note, rgb in self.STATIC_PAD_COLORS.items():
            self._pads.set_pad(note, *rgb)
        # PAD_ALT reflects on/off, same as the Bank/Mode LED
        self._pads.set_pad(PAD_ALT, *((40, 0, 40) if _alt_mode() else (0, 0, 40)))

        for note in self.PAD_ACTIONS:
            self._pads.set_pad(note, *self._pad_state_color(self._pad_states[note]))

    def on_screen_change(self, screen):
        super().on_screen_change(screen)
        self._pad_states = {k: -1 for k in self._pad_states}
        pad_state = self.SCREEN_MAP.get(screen)
        if pad_state is not None:
            pad, idx = pad_state
            self._pad_states[pad] = idx

    def note_on(self, note, velocity, shifted_override=None):
        if note in self.ZYNPOT_SWITCH_BTNS or note in self.TIMED_PADS:
            self._btn_timer.is_pressed(note, time.time())
            return True

        # Knob touch note numbers coincide with their CC numbers - use touch
        # only to start each turn with a clean accumulator (no leftover bias
        # from a previous turn), not as a button press (see comment above
        # BTN_VOLUME_TOUCH).
        if note in ZYNPOT_KNOBS:
            self._zynpot.reset(note)
            return True

        if note == BTN_PAT_UP:
            self._state_manager.send_cuia("ARROW_UP")
        elif note == BTN_PAT_DOWN:
            self._state_manager.send_cuia("ARROW_DOWN")
        elif note == BTN_GRID_LEFT:
            self._state_manager.send_cuia("ARROW_LEFT")
        elif note == BTN_GRID_RIGHT:
            self._state_manager.send_cuia("ARROW_RIGHT")
        elif note == PAD_ALT:
            # BTN_BANK (the physical button) does the same thing globally -
            # see midi_event - this is just a second way to reach it while
            # the pad grid already shows Device mode.
            _toggle_alt_mode()
            self.refresh()
        elif note == PAD_METRONOME:
            self._state_manager.send_cuia("TEMPO")
        elif note == PAD_RECORD:
            self._state_manager.send_cuia("TOGGLE_RECORD")
        elif note == PAD_BACK:
            self._state_manager.send_cuia("BACK")
        elif note == PAD_SELECT:
            self._state_manager.send_cuia("V5_ZYNPOT_SWITCH", [3, 'S'])
        elif note == BTN_SELECT_PRESS:
            # Same action as PAD_SELECT above - the physical Select knob's
            # own push is just a second way to reach it.
            self._state_manager.send_cuia("V5_ZYNPOT_SWITCH", [3, 'S'])
        elif note == PAD_UP:
            self._state_manager.send_cuia("ARROW_UP")
        elif note == PAD_DOWN:
            self._state_manager.send_cuia("ARROW_DOWN")
        elif note == PAD_LEFT:
            self._state_manager.send_cuia("ARROW_LEFT")
        elif note == PAD_RIGHT:
            self._state_manager.send_cuia("ARROW_RIGHT")
        elif note in self.FUNCTION_PADS:
            pgm = self.FUNCTION_PADS[note] + (4 if _alt_mode() else 0)
            self._state_manager.send_cuia("PROGRAM_CHANGE", [pgm])
        else:
            return False
        return True

    def note_off(self, note, shifted_override=None):
        self._btn_timer.is_released(note)

    def cc_change(self, ccnum, ccval):
        if ccnum == KNOB_SELECT:
            _select_knob_arrow(self._state_manager, ccval)
            return True
        return self._zynpot.cc_change(ccnum, ccval)

    def _handle_timed_button(self, btn, press_type):
        if press_type == CONST.PT_LONG:
            cuia = self.PAD_LONG_ACTIONS.get(btn)
            if cuia:
                self._state_manager.send_cuia(cuia)
            return True

        zynpot = self.ZYNPOT_SWITCH_BTNS.get(btn)
        if zynpot is not None:
            if press_type == CONST.PT_SHORT:
                self._state_manager.send_cuia("V5_ZYNPOT_SWITCH", [zynpot, 'S'])
            elif press_type == CONST.PT_BOLD:
                self._state_manager.send_cuia("V5_ZYNPOT_SWITCH", [zynpot, 'B'])
            return True

        if btn == PAD_PLAY:
            if press_type == CONST.PT_BOLD:
                self._state_manager.send_cuia("AUDIO_FILE_LIST")
            else:
                self._state_manager.send_cuia("TOGGLE_PLAY")
            return True

        if btn == PAD_STOP:
            if press_type == CONST.PT_BOLD:
                self._state_manager.send_cuia("ALL_SOUNDS_OFF")
            else:
                self._state_manager.send_cuia("STOP")
            return True

        actions = self.PAD_ACTIONS.get(btn)
        if actions is None:
            return

        idx = -1
        if press_type == CONST.PT_SHORT:
            idx = (self._pad_states[btn] + 1) % len(actions)
        elif press_type == CONST.PT_BOLD:
            idx = 1 if len(actions) > 1 else 0
        cuia = actions[idx]

        self._state_manager.send_cuia(cuia)
        return True


# --------------------------------------------------------------------------
# Handle Mixer (active on the audio_mixer screen)
# --------------------------------------------------------------------------
class MixerHandler(ModeHandlerBase):

    # Chains page 4-at-a-time via Grid Left/Right (see _chain_at()). Paging
    # right past the last regular chain lands on a dedicated Master page
    # (_on_master_page) showing the Main/Master chain at position 0 (rows 1-3
    # blank) - Master is otherwise never shown, so it doesn't eat a slot on
    # page 0. Selecting Master (e.g. Solo 1 on that page) makes it the active
    # chain like any other, so the knobs/bar-taps work on it too.

    # Each of the 4 pad rows is one visible chain's 16-pad bar: volume
    # normally (red, filled left-to-right), or - while alt mode is on (the
    # persistent global toggle, see _alt_mode() - NOT momentarily holding
    # BTN_ALT, that's the unrelated Alt+Solo-N modifier below) - balance
    # (green, filled outward from center: right for positive, left for
    # negative, nothing lit at 0).
    BAR_COLOR_VOLUME = (90, 0, 0)
    BAR_COLOR_BALANCE = (0, 90, 0)

    # A single tap jumps to a pad's coarse position (1/16th steps for volume,
    # 1/8th per side for balance) - too coarse for fine adjustment, and for
    # balance specifically no single pad lands exactly on 0. Both solved by
    # subdividing each pad into FINE_STEPS brightness levels:
    #
    # - A tap on a *different* pad than your last one jumps there, landing at
    #   that pad's DIMMEST sub-level (not fully lit) - so the pad under your
    #   finger has room to visibly brighten on the next few taps, rather than
    #   already being maxed out and immediately handing off to its neighbor.
    # - A tap on the SAME pad as last time nudges by one fine step instead of
    #   re-jumping: on the positive/volume side, plain tap = away from center
    #   (brighter/louder), Alt+tap = toward center (dimmer/quieter). On
    #   balance's negative side this is inverted so the same "tap = away from
    #   center, Alt+tap = toward center" mental model holds on both sides
    #   (rather than being mirrored, which incrementing/decrementing a single
    #   signed value symmetrically would otherwise give you). Walking a
    #   balance value toward center this way can land exactly on 0.
    # - Holding both center pads (7 and 8) together while in balance view
    #   jumps straight to 0 - a second-press-while-first-still-held check via
    #   note_off tracking (_center_held), not true MIDI chord detection.
    #
    # Tracking "same pad as last tap" (rather than a "boundary pad" derived
    # fresh from the current value, requiring an exact match) means repeated
    # taps on one physical pad keep working even as the fill visually crosses
    # into a neighboring pad's range.
    FINE_STEPS = 4

    def __init__(self, state_manager, leds: FeedbackLEDs, pads: PadLEDs):
        super().__init__(state_manager)
        self._leds = leds
        self._pads = pads
        self._chains_bank = 0
        self._on_master_page = False  # see _chain_at() / BTN_GRID_RIGHT handling
        self._is_alt = False
        self._knobs_ease = KnobJitterFilter()
        self._last_tapped_col = [None] * 4  # per row, see _on_bar_pad
        self._center_held = [set(), set(), set(), set()]  # per row, see _on_bar_pad

        active_chain = self._chain_manager.get_active_chain()
        self._active_chain = active_chain.chain_id if active_chain else 0

    def refresh(self):
        for pos in range(4):
            self._paint_solo_led(pos)
            self._paint_row(pos)

    def _regular_chain_ids(self):
        """Ordered chain IDs excluding Main (chain_id 0). Main's own slot in
        chain_manager.ordered_chain_ids is NOT fixed at index 0 - new chains
        are inserted *at* Main's current index (see add_chain() in
        zynthian_chain_manager.py), pushing it one slot later each time, so
        it drifts towards the end as chains are added. Filtering it out
        explicitly here (rather than assuming a fixed offset) is what
        actually keeps regular paging and the Master page from overlapping."""
        return [cid for cid in self._chain_manager.ordered_chain_ids if cid != 0]

    def _max_bank(self):
        """Last page of *regular* (non-Main) chains - paging right past this
        lands on the dedicated Master page instead of another regular page."""
        regular_count = len(self._regular_chain_ids())
        return (max(regular_count, 1) - 1) // 4

    def _chain_at(self, pos):
        if self._on_master_page:
            return self._chain_manager.get_chain(0) if pos == 0 else None
        ids = self._regular_chain_ids()
        index = pos + self._chains_bank * 4
        if index >= len(ids):
            return None
        return self._chain_manager.get_chain(ids[index])

    def _paint_solo_led(self, pos):
        # Solo strip is green-only (0=off, 1=dull, 2=high - see protocol doc),
        # only 2 non-off levels available, so solo and mute share the dull
        # level (indistinguishable from each other, but distinct from selected).
        chain = self._chain_at(pos)
        led = LED_SOLO_1 + pos
        if chain is None:
            self._leds.led_off(led)
        elif chain.chain_id == self._active_chain:
            self._leds.led_on(led, LED_GREEN_HIGH)
        elif self._zynmixer.get_solo(chain.mixer_chan) or self._zynmixer.get_mute(chain.mixer_chan):
            self._leds.led_on(led, LED_GREEN_DULL)
        else:
            self._leds.led_off(led)

    def _paint_row(self, pos):
        chain = self._chain_at(pos)
        if chain is None:
            for col in range(16):
                self._pads.pad_off(_pad(pos, col))
            return
        if _alt_mode():
            colors = self._balance_colors(self._zynmixer.get_balance(chain.mixer_chan) * 100)
        else:
            colors = self._volume_colors(self._zynmixer.get_level(chain.mixer_chan) * 100)
        for col in range(16):
            self._pads.set_pad(_pad(pos, col), *colors[col])

    @classmethod
    def _fine_color(cls, fine_pos, lo, hi, color):
        """fine_pos's contribution to one pad spanning fine range [lo, hi):
        full color once fine_pos reaches hi, off below lo, partial brightness
        (proportional) in between - that in-between pad is "the boundary"."""
        if fine_pos >= hi:
            return color
        if fine_pos <= lo:
            return (0, 0, 0)
        frac = (fine_pos - lo) / (hi - lo)
        return tuple(round(c * frac) for c in color)

    @classmethod
    def _volume_fine_pos(cls, level_pct):
        total = 16 * cls.FINE_STEPS
        return round(max(0, min(level_pct, 100)) / 100 * total)

    @classmethod
    def _volume_from_fine_pos(cls, fine_pos):
        return fine_pos / (16 * cls.FINE_STEPS) * 100

    @classmethod
    def _volume_colors(cls, level_pct):
        fine_pos = cls._volume_fine_pos(level_pct)
        return [
            cls._fine_color(fine_pos, col * cls.FINE_STEPS, (col + 1) * cls.FINE_STEPS, cls.BAR_COLOR_VOLUME)
            for col in range(16)
        ]

    @classmethod
    def _balance_fine_pos(cls, balance_pct):
        """Signed fine position: +/- (8 * FINE_STEPS) at full deflection, 0 at
        center - unlike volume this can go negative (left side)."""
        total = 8 * cls.FINE_STEPS
        return round(max(-100, min(balance_pct, 100)) / 100 * total)

    @classmethod
    def _balance_from_fine_pos(cls, fine_pos):
        return fine_pos / (8 * cls.FINE_STEPS) * 100

    @classmethod
    def _balance_colors(cls, balance_pct):
        fine_pos = cls._balance_fine_pos(balance_pct)
        colors = [(0, 0, 0)] * 16
        if fine_pos > 0:
            for i in range(8):
                colors[8 + i] = cls._fine_color(fine_pos, i * cls.FINE_STEPS, (i + 1) * cls.FINE_STEPS, cls.BAR_COLOR_BALANCE)
        elif fine_pos < 0:
            neg = -fine_pos
            for i in range(8):
                colors[7 - i] = cls._fine_color(neg, i * cls.FINE_STEPS, (i + 1) * cls.FINE_STEPS, cls.BAR_COLOR_BALANCE)
        return colors

    def _on_bar_pad(self, note):
        index = note - PAD_NOTE_BASE
        pos, col = index // 16, index % 16
        chain = self._chain_at(pos)
        if chain is None:
            return True

        if _alt_mode() and col in (7, 8):
            self._center_held[pos].add(col)
            if len(self._center_held[pos]) == 2:
                # Both center pads held together: snap straight to 0.
                self._zynmixer.set_balance(chain.mixer_chan, 0)
                self._last_tapped_col[pos] = None
                self._paint_row(pos)
                return True

        # Tapping the same physical pad as last time on this row = fine
        # nudge; any other pad = coarse jump to its position (and it becomes
        # the new "last tapped" pad, so a follow-up tap on it fine-nudges).
        fine_adjust = self._last_tapped_col[pos] == col
        self._last_tapped_col[pos] = col

        if _alt_mode():
            if fine_adjust:
                fine_pos = self._balance_fine_pos(self._zynmixer.get_balance(chain.mixer_chan) * 100)
                if col >= 8:
                    # Positive/right side: tap = away from center (louder
                    # right), Alt+tap = toward center.
                    fine_pos += -1 if self._is_alt else 1
                else:
                    # Negative/left side: inverted relative to a plain signed
                    # increment, so "tap = away from center, Alt+tap = toward
                    # center" holds the same way on both sides.
                    fine_pos += 1 if self._is_alt else -1
                fine_pos = max(-8 * self.FINE_STEPS, min(fine_pos, 8 * self.FINE_STEPS))
            elif col >= 8:
                # Land at this pad's dimmest sub-level, not fully lit - leaves
                # room for subsequent taps to visibly brighten it further.
                fine_pos = (col - 8) * self.FINE_STEPS + 1
            else:
                fine_pos = -((7 - col) * self.FINE_STEPS + 1)
            self._zynmixer.set_balance(chain.mixer_chan, self._balance_from_fine_pos(fine_pos) / 100)
        else:
            if fine_adjust:
                fine_pos = self._volume_fine_pos(self._zynmixer.get_level(chain.mixer_chan) * 100)
                fine_pos += -1 if self._is_alt else 1
                fine_pos = max(0, min(fine_pos, 16 * self.FINE_STEPS))
            else:
                fine_pos = col * self.FINE_STEPS + 1
            self._zynmixer.set_level(chain.mixer_chan, self._volume_from_fine_pos(fine_pos) / 100)

        self._paint_row(pos)
        return True

    def _off_bar_pad(self, note):
        index = note - PAD_NOTE_BASE
        pos, col = index // 16, index % 16
        self._center_held[pos].discard(col)

    def set_alt(self, state):
        # This is momentary-hold BTN_ALT (used as a modifier for Alt+Solo-N =
        # toggle solo below) - unrelated to the persistent "alt mode" that
        # _paint_row() reads via _alt_mode() for volume-vs-balance, so this
        # doesn't need to repaint anything.
        self._is_alt = state

    def note_on(self, note, velocity, shifted_override=None):
        self._on_shifted_override(shifted_override)

        # Same touch-resets-accumulator trick as DeviceHandler (Volume/Pan only
        # here - Filter/Resonance aren't used in Mixer mode, Select has no touch).
        if note in (KNOB_VOLUME, KNOB_PAN):
            self._knobs_ease.reset(note)
            return True

        if PAD_NOTE_BASE <= note < PAD_NOTE_BASE + 64:
            return self._on_bar_pad(note)

        if BTN_SOLO_1 <= note <= BTN_SOLO_4:
            chain = self._chain_at(note - BTN_SOLO_1)
            if chain is None:
                return True

            if self._is_shifted:
                val = self._zynmixer.get_mute(chain.mixer_chan) ^ 1
                self._zynmixer.set_mute(chain.mixer_chan, val, True)
            elif self._is_alt:
                val = self._zynmixer.get_solo(chain.mixer_chan) ^ 1
                self._zynmixer.set_solo(chain.mixer_chan, val, True)
            else:
                self._chain_manager.set_active_chain_by_id(chain.chain_id)
                self._active_chain = chain.chain_id
            self.refresh()
            return True

        if note == BTN_GRID_LEFT:
            # Master page (see _chain_at()) sits one step past the last
            # regular page - Grid Left/Right walk in and out of it too.
            if self._on_master_page:
                self._on_master_page = False
            else:
                self._chains_bank = max(0, self._chains_bank - 1)
            self.refresh()
            return True

        if note == BTN_GRID_RIGHT:
            if self._on_master_page:
                pass
            elif self._chains_bank >= self._max_bank():
                self._on_master_page = True
            else:
                self._chains_bank += 1
            self.refresh()
            return True

        if note == BTN_SELECT_PRESS:
            self._state_manager.send_cuia("V5_ZYNPOT_SWITCH", [3, 'S'])
            return True

        return False

    def note_off(self, note, shifted_override=None):
        if PAD_NOTE_BASE <= note < PAD_NOTE_BASE + 64:
            self._off_bar_pad(note)

    def cc_change(self, ccnum, ccval):
        # Select's own encoder isn't noisy like the other 4 - use it raw.
        if ccnum == KNOB_SELECT:
            delta = ccval if ccval < 64 else ccval - 128
            self._chain_manager.next_chain(delta) if delta > 0 else self._chain_manager.previous_chain(-delta)
            return True

        if ccnum not in (KNOB_VOLUME, KNOB_PAN):
            return

        delta = self._knobs_ease.feed(ccnum, ccval)
        if delta is None:
            return

        chain = self._chain_manager.get_chain(self._active_chain)
        if chain is None:
            return False

        if ccnum == KNOB_VOLUME:
            value = self._zynmixer.get_level(chain.mixer_chan) * 100
            value = max(0, min(value + delta, 100))
            self._zynmixer.set_level(chain.mixer_chan, value / 100)
            self._paint_active_row()
            return True

        if ccnum == KNOB_PAN:
            value = self._zynmixer.get_balance(chain.mixer_chan) * 100
            value = max(-100, min(value + delta, 100))
            self._zynmixer.set_balance(chain.mixer_chan, value / 100)
            self._paint_active_row()
            return True

    def _pos_of_chain_id(self, chain_id):
        """Row position (0-3) chain_id is currently displayed at, or None if
        it's not on the visible page (a different regular page, or the
        Master page isn't showing while chain_id belongs to a regular chain,
        or vice versa)."""
        if chain_id == 0:
            return 0 if self._on_master_page else None
        if self._on_master_page:
            return None
        try:
            index = self._regular_chain_ids().index(chain_id)
        except ValueError:
            return None
        pos = index - self._chains_bank * 4
        return pos if 0 <= pos < 4 else None

    def _paint_active_row(self):
        # Immediate feedback for our own knob-driven changes, rather than
        # waiting on the (queued, and for this would repaint everything)
        # update_mixer_strip signal round-trip.
        pos = self._pos_of_chain_id(self._active_chain)
        if pos is not None:
            self._paint_row(pos)

    def update_mixer_strip(self, chan, symbol, value):
        if symbol not in ("mute", "solo", "level", "balance"):
            return
        chain_id = self._chain_manager.get_chain_id_by_mixer_chan(chan)
        if chain_id is None:
            return
        pos = self._pos_of_chain_id(chain_id)
        if pos is None:
            return
        if symbol in ("mute", "solo"):
            self._paint_solo_led(pos)
        else:
            self._paint_row(pos)

    def set_active_chain(self, chain_id, refresh):
        self._active_chain = chain_id
        if chain_id == 0:
            self._on_master_page = True
        else:
            self._on_master_page = False
            try:
                index = self._regular_chain_ids().index(chain_id)
            except ValueError:
                index = 0
            self._chains_bank = index // 4
        if refresh:
            self.refresh()


def _hex_to_rgb127(hex_color):
    """'#rrggbb' (0-255/channel) -> a 0-127 RGB tuple for the pad SysEx protocol."""
    return tuple(round(int(hex_color[i:i + 2], 16) / 255 * 127) for i in (1, 3, 5))


def _normalize_rgb127(rgb):
    """Scale rgb so its brightest channel hits 127, preserving hue/ratio."""
    peak = max(rgb)
    if peak == 0:
        return rgb
    return tuple(round(c * 127 / peak) for c in rgb)


# --------------------------------------------------------------------------
# Handle Zynpad (sequence/clip launcher, active on the zynpad screen)
# --------------------------------------------------------------------------
#
# Fire has 4 physical pad rows but a zynseq bank can be up to 8x8 - rather than
# add row-paging, this uses the grid's extra width instead: each pair of
# adjacent physical columns (0-1, 2-3, ... 14-15) is one logical column, the
# first of the pair showing logical rows 0-3 and the second rows 4-7 - closer
# to the original 8x8 layout than two separate 8-wide halves would be. See
# _logical_xy()/_physical_xy() for the mapping.
#
# Fire's pads also have no native blink (confirmed in the protocol doc, and
# DrivenByMoss needs the same software workaround) - rather than add a repeat
# timer just for this, state is conveyed by color instead: each sequence is
# colored by its group (dim when stopped, full brightness when playing), while
# starting/stopping (about to play/stop) use a fixed white, dim/bright the
# same way, regardless of group.
class ZynpadHandler(ModeHandlerBase):

    PHYS_COLS = 16
    PHYS_ROWS = 4

    # Group hues mirror zyngui.zynthian_gui_config.PAD_COLOUR_GROUP exactly
    # (same modulo-16 indexing that screen uses), so a sequence's pad color
    # matches what the zynpad touchscreen shows for the same group. Playing
    # uses these at their native (muted-but-saturated) intensity - pushing
    # every hue's peak channel to 127 looked too bright/washed-out. Stopped
    # normalizes to full brightness first, then dims - keeps a clear dim/
    # bright contrast even for hues whose raw value is already quite dark.
    GROUP_COLORS = [_hex_to_rgb127(c) for c in zynthian_gui_config.PAD_COLOUR_GROUP[:16]]
    GROUP_COLORS_DIM = [
        tuple(round(c * 0.15) for c in _normalize_rgb127(rgb)) for rgb in GROUP_COLORS
    ]

    COLOR_EMPTY = (0, 0, 0)
    COLOR_STARTING = (90, 90, 90)   # bright white - SEQ_STARTING / SEQ_RESTARTING
    COLOR_STOPPING = (25, 25, 25)   # dim white - SEQ_STOPPING / SEQ_STOPPINGSYNC

    def __init__(self, state_manager, pads: PadLEDs):
        super().__init__(state_manager)
        self._pads = pads
        self._libseq = self._zynseq.libseq
        # No zynpad-specific use for the 4 knobs yet - default to the same
        # zynpot navigation as Device mode rather than leaving them dead.
        self._zynpot = ZynpotRotate(state_manager)

    @staticmethod
    def _logical_xy(phys_col, phys_row):
        """Physical pad position -> logical zynseq (col, row): each pair of
        adjacent physical columns is one logical column (see class comment)."""
        return phys_col // 2, phys_row + (4 if phys_col % 2 else 0)

    @staticmethod
    def _physical_xy(logical_col, logical_row):
        """Inverse of _logical_xy()."""
        if logical_row >= 4:
            return logical_col * 2 + 1, logical_row - 4
        return logical_col * 2, logical_row

    def refresh(self):
        for row in range(self.PHYS_ROWS):
            for col in range(self.PHYS_COLS):
                self._paint_pad(col, row)

    def _paint_pad(self, col, row):
        note = _pad(row, col)
        lcol, lrow = self._logical_xy(col, row)
        if lcol >= self._zynseq.col_in_bank or lrow >= self._zynseq.col_in_bank:
            self._pads.pad_off(note)
            return
        seq = self._zynseq.get_pad_from_xy(lcol, lrow)
        self._pads.set_pad(note, *self._seq_color(seq))

    def _seq_color(self, seq):
        packed = self._libseq.getSequenceState(self._zynseq.bank, seq)
        state = packed & 0xFF
        group = (packed >> 16) & 0xFF
        return self._state_color(state, seq, group)

    def _group_color(self, group, bright):
        idx = group % len(self.GROUP_COLORS)
        return self.GROUP_COLORS[idx] if bright else self.GROUP_COLORS_DIM[idx]

    def _state_color(self, state, seq, group):
        if state in (zynseq.SEQ_STARTING, zynseq.SEQ_RESTARTING):
            return self.COLOR_STARTING
        if state == zynseq.SEQ_PLAYING:
            return self._group_color(group, bright=True)
        if state in (zynseq.SEQ_STOPPING, zynseq.SEQ_STOPPINGSYNC):
            return self.COLOR_STOPPING
        if self._libseq.isEmpty(self._zynseq.bank, seq):
            return self.COLOR_EMPTY
        return self._group_color(group, bright=False)

    def update_seq_state(self, bank, seq, state=None, mode=None, group=None):
        if bank != self._zynseq.bank:
            return
        lcol, lrow = self._zynseq.get_xy_from_pad(seq)
        if lcol >= 8 or lrow >= 8:
            return
        col, row = self._physical_xy(lcol, lrow)
        self._pads.set_pad(_pad(row, col), *self._state_color(state, seq, group))

    def note_on(self, note, velocity, shifted_override=None):
        if note in ZYNPOT_KNOBS:
            self._zynpot.reset(note)
            return True
        if note == BTN_SELECT_PRESS:
            self._state_manager.send_cuia("V5_ZYNPOT_SWITCH", [3, 'S'])
            return True

        index = note - PAD_NOTE_BASE
        row, col = index // 16, index % 16
        lcol, lrow = self._logical_xy(col, row)
        if lcol >= self._zynseq.col_in_bank or lrow >= self._zynseq.col_in_bank:
            return False
        seq = self._zynseq.get_pad_from_xy(lcol, lrow)
        self._libseq.togglePlayState(self._zynseq.bank, seq)
        return True

    def cc_change(self, ccnum, ccval):
        if ccnum == KNOB_SELECT:
            _select_knob_arrow(self._state_manager, ccval)
            return True
        return self._zynpot.cc_change(ccnum, ccval)


# --------------------------------------------------------------------------
# Main driver
# --------------------------------------------------------------------------
class zynthian_ctrldev_akai_fire(zynthian_ctrldev_zynmixer, zynthian_ctrldev_zynpad):

    # Confirmed on real hardware (see zynthian_ctrldev_akai_fire_protocol.md). Kept the
    # "MIDI 1"/"IN 1" suffixed variants too, in case the exact JACK alias zynautoconnect
    # reports differs from what `aconnect -l` shows (see other drivers' dev_ids for why).
    dev_ids = ["FL STUDIO FIRE", "FL STUDIO FIRE MIDI 1", "FL STUDIO FIRE IN 1"]
    driver_name = 'AKAI Fire'
    driver_description = 'Device + Mixer + Zynpad modes (OLED not yet implemented)'

    def __init__(self, state_manager, idev_in, idev_out=None):
        self._leds = FeedbackLEDs(idev_out)
        self._pads = PadLEDs(idev_out)
        self._device_handler = DeviceHandler(state_manager, self._leds, self._pads)
        self._mixer_handler = MixerHandler(state_manager, self._leds, self._pads)
        self._zynpad_handler = ZynpadHandler(state_manager, self._pads)
        self._current_handler = self._device_handler

        self._is_shifted = False
        self._is_alt = False
        self._btn_timer = ButtonTimer(self._handle_timed_button)

        # The active mode is normally picked automatically from the current
        # screen (see _update_current_handler), but Alt+Browser can unlink
        # it - see midi_event's BTN_BROWSER handling - so it stays on
        # whatever mode is currently showing regardless of screen changes
        # (e.g. keep Mixer on the Fire while navigating the touchscreen to
        # the pattern editor). Re-linking immediately re-syncs to the
        # current screen.
        self._screen_linked = True
        self._last_screen = None

        self._signals = [
            (zynsigman.S_GUI, zynsigman.SS_GUI_SHOW_SCREEN, self._on_gui_show_screen),
        ]

        # NOTE: init() (called by the manager right after this ctor) will call
        # refresh(), which needs _current_handler ready - so this goes last.
        super().__init__(state_manager, idev_in, idev_out)

    def init(self):
        # The pad grid is the device's own SysEx-set memory, independent of
        # this driver's state - it can carry stale colors from a previous
        # session/mode across a reload. _update_current_handler()'s clear
        # only fires on an actual handler *change*, which never happens right
        # at startup (_current_handler is already _device_handler from
        # construction), so clear unconditionally here before anything below
        # (via super().init() -> refresh()) paints the real starting state.
        self._pads.all_off()

        # super().init()/end() cooperatively chain through BOTH mixins here
        # (zynmixer -> zynpad -> base), verified via MRO - no need for the
        # explicit extra zynthian_ctrldev_zynpad.init(self)/end(self) call
        # some other multi-mixin drivers in this codebase carry.
        super().init()
        for signal, subsignal, callback in self._signals:
            zynsigman.register(signal, subsignal, callback)

    def end(self):
        for signal, subsignal, callback in self._signals:
            zynsigman.unregister(signal, subsignal, callback)
        super().end()

    def refresh(self):
        self._current_handler.refresh()
        self._update_mode_leds()

    def _update_mode_leds(self):
        # Browser (red-only): repurposed as a screen-link indicator - lit
        # whenever unlinked, as a reminder the Fire's mode won't follow
        # screen changes until Alt+Browser re-links it.
        self._leds.led_on(LED_BROWSER, LED_RED_HIGH) if not self._screen_linked \
            else self._leds.led_off(LED_BROWSER)

        # Perform (yellow-red): red in Mixer mode, yellow in Zynpad mode, off
        # in Device mode.
        if self._current_handler is self._mixer_handler:
            self._leds.led_on(LED_PERFORM, LED_YR_HIGH_RED)
        elif self._current_handler is self._zynpad_handler:
            self._leds.led_on(LED_PERFORM, LED_YR_HIGH_YELLOW)
        else:
            self._leds.led_off(LED_PERFORM)

    def light_off(self):
        self._leds.all_off()
        self._pads.all_off()

    def midi_event(self, ev):
        evtype = (ev[0] >> 4) & 0x0F

        if evtype == EV_NOTE_ON:
            note = ev[1] & 0x7F
            vel = ev[2] & 0x7F

            if note == BTN_SHIFT:
                self._is_shifted = True
                self._mixer_handler.on_shift_changed(True)
                return True
            if note == BTN_ALT:
                self._is_alt = True
                # Alt+Browser (screen-link toggle) and Alt+Perform (jump to
                # Device) work from any mode, so _is_alt itself is tracked
                # unconditionally above - but only forward into MixerHandler
                # (its Alt+Solo-N = toggle solo modifier) while it's
                # actually the active mode.
                if self._current_handler is self._mixer_handler:
                    self._mixer_handler.set_alt(True)
                return True
            if note == BTN_PERFORM:
                if self._is_alt:
                    # Straight to Device mode, unlinking if needed so it
                    # sticks regardless of subsequent screen changes. Same
                    # caveat as BTN_BROWSER above: _set_current_handler()
                    # only refreshes LEDs on an actual handler change, so
                    # the link-status LED needs an explicit update too.
                    self._screen_linked = False
                    self._set_current_handler(self._device_handler)
                    self._update_mode_leds()
                elif self._screen_linked:
                    # Screen-driven: change the actual screen; screen-follow
                    # (_update_current_handler) then updates the mode to
                    # match. Toggles between the two "performance" screens:
                    # from mixer, go to zynpad; from anywhere else
                    # (including Device mode's screens), go to mixer - so
                    # repeated presses settle into alternating mixer <->
                    # zynpad.
                    if self._last_screen == "audio_mixer":
                        self.state_manager.send_cuia("SCREEN_ZYNPAD")
                    else:
                        self.state_manager.send_cuia("SCREEN_AUDIO_MIXER")
                else:
                    # Unlinked: flip the Fire's own mode directly, without
                    # touching whatever's actually shown on the touchscreen -
                    # same mixer <-> zynpad toggle, just applied straight to
                    # the handler.
                    target = self._zynpad_handler if self._current_handler is self._mixer_handler else self._mixer_handler
                    self._set_current_handler(target)
                return True

            # Transport, Browser and Bank/Mode are global: they work the same
            # regardless of which mode is currently active.
            if note == BTN_BANK:
                _toggle_alt_mode()
                self.refresh()
                return True
            if note == BTN_BROWSER:
                if self._is_alt:
                    self._screen_linked = not self._screen_linked
                    # _update_current_handler() only refreshes LEDs when the
                    # handler itself changes - the link toggle needs its own
                    # LED update even when it doesn't (e.g. toggling while
                    # already on the mode the current screen would pick).
                    self._update_current_handler()
                    self._update_mode_leds()
                else:
                    # MENU is context-aware per current screen (zynthian_gui's
                    # cuia_menu calls the screen's own toggle_menu() if it has
                    # one - e.g. zynpad's own "ZynPad Menu" - else falls back
                    # to the main menu), so this "just works" as a context
                    # menu in whatever mode/screen is active, present and future.
                    self.state_manager.send_cuia("MENU")
                return True
            if note == BTN_RECORD:
                self.state_manager.send_cuia("TOGGLE_RECORD")
                return True
            if note in (BTN_PLAY, BTN_STOP):
                self._btn_timer.is_pressed(note, time.time())
                return True

            return self._current_handler.note_on(note, vel, self._is_shifted)

        if evtype == EV_NOTE_OFF:
            note = ev[1] & 0x7F

            if note == BTN_SHIFT:
                self._is_shifted = False
                self._mixer_handler.on_shift_changed(False)
                return True
            if note == BTN_ALT:
                self._is_alt = False
                if self._current_handler is self._mixer_handler:
                    self._mixer_handler.set_alt(False)
                return True
            if note in (BTN_PLAY, BTN_STOP):
                self._btn_timer.is_released(note)
                return True

            return self._current_handler.note_off(note, self._is_shifted)

        if evtype == EV_CC:
            ccnum = ev[1] & 0x7F
            ccval = ev[2] & 0x7F
            return self._current_handler.cc_change(ccnum, ccval)

        if ev[0] == EV_SYSEX:
            logging.info(f" received SysEx => {ev}")
            return True

        return False

    def update_mixer_strip(self, chan, symbol, value):
        # Guarded: MixerHandler.update_mixer_strip() only repaints (no state
        # to keep fresh for later), so skip it entirely while Mixer isn't the
        # visible mode - otherwise e.g. turning a knob in Device mode (or any
        # other zctrl change reaching here) would paint mixer bars on top of
        # whatever's actually showing on the grid.
        if self._current_handler is self._mixer_handler:
            self._mixer_handler.update_mixer_strip(chan, symbol, value)

    def update_mixer_active_chain(self, active_chain):
        self._mixer_handler.set_active_chain(active_chain, self._current_handler is self._mixer_handler)

    def update_seq_state(self, bank, seq, state=None, mode=None, group=None):
        # Same reasoning as update_mixer_strip above.
        if self._current_handler is self._zynpad_handler:
            self._zynpad_handler.update_seq_state(bank, seq, state, mode, group)

    def _handle_timed_button(self, btn, press_type):
        if btn == BTN_PLAY:
            if press_type == CONST.PT_BOLD:
                self.state_manager.send_cuia("AUDIO_FILE_LIST")
            else:
                self.state_manager.send_cuia("TOGGLE_PLAY")
        elif btn == BTN_STOP:
            if press_type == CONST.PT_BOLD:
                self.state_manager.send_cuia("ALL_SOUNDS_OFF")
            else:
                self.state_manager.send_cuia("STOP")

    def _set_current_handler(self, handler):
        """Switch to the given handler (no-op if it's already current),
        refreshing its pad state and the mode LEDs."""
        if self._current_handler is handler:
            return
        self._current_handler = handler
        if handler is self._mixer_handler:
            # Pick up Alt if it was already held before switching in (its
            # press/release edges only forward to MixerHandler while it's
            # already the active mode - see BTN_ALT handling).
            self._mixer_handler.set_alt(self._is_alt)
        # Unconditional full clear rather than each handler tracking which
        # notes the *other* one used - simpler, and can't miss anything as
        # more pad-owning modes are added.
        self._pads.all_off()
        self._current_handler.refresh()
        self._update_mode_leds()

    def _update_current_handler(self):
        """While screen-linked, re-derive _current_handler from
        _last_screen. While unlinked, do nothing - _current_handler stays
        exactly as-is regardless of screen changes, until re-linked (which
        immediately re-syncs to whatever screen is current at that point).
        Called on screen changes and on the Alt+Browser link toggle."""
        if not self._screen_linked:
            return
        if self._last_screen == "audio_mixer":
            self._set_current_handler(self._mixer_handler)
        elif self._last_screen == "zynpad":
            self._set_current_handler(self._zynpad_handler)
        else:
            self._set_current_handler(self._device_handler)

    def _on_gui_show_screen(self, screen, **kwargs):
        self._last_screen = screen
        was_device = self._current_handler is self._device_handler
        # Keep DeviceHandler's screen-access pad state in sync even while it
        # isn't the active handler, so it's already correct if/when we switch
        # (or force) into Device mode later.
        self._device_handler.on_screen_change(screen)
        self._update_current_handler()
        # _update_current_handler() only refreshes on a handler change; if we
        # were already in Device mode and just the screen changed, still need
        # to repaint the screen-access pads to match.
        if was_device and self._current_handler is self._device_handler:
            self._device_handler.refresh()
