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
# This implements four pad-grid modes, plus a large set of controls that work
# the same regardless of which one is active (or none - see below). Three
# modes are auto-switched by the current zynthian screen: "Mixer" (audio_mixer
# screen), "Zynpad" (zynpad screen - sequence/clip launcher on the pad grid),
# and "StepSeq" (pattern_editor screen, reached via BTN_STEP - a Note
# Sequencer style step editor on the pad grid, see
# zynthian_ctrldev_akai_fire_stepseq_plan.md for the design this implements).
# The fourth, "Play" (reached via BTN_NOTE - a chromatic note-playing keyboard
# on the pad grid), has no screen of its own and is forced on/off directly
# instead. Every other zynthian screen (Admin, Preset, Control, Snapshot, main
# menu, etc.) has no dedicated mode of its own - the pad grid just keeps
# showing whichever of the 4 above was last active (see
# _update_current_handler) - but transport, screen navigation (Perform, Browser,
# Step, Alt+Note/Perform for Snapshot/ZS3, Pattern Up/Down, Grid Left/Right),
# the Volume/Pan/Filter/Resonance/Select knobs, Select's own push, and
# Alt+touch on any of the 4 knobs (a zynpot "switch" push) all keep working
# regardless - see midi_event's global button handling and
# _default_note_on/_default_cc_change for the shared fallbacks every mode
# (including no mode) falls back to. The OLED (see OledDisplay) shows the
# current mode name (blank when there isn't one), refreshed on every mode
# switch. Drum is unbound, reserved for a future mode.
#
# ******************************************************************************

import os
import time
import json
import queue
import logging
import multiprocessing as mp

from PIL import Image, ImageDraw, ImageFont

from zynlibs.zynseq import zynseq
from zynlibs.zynaudioplayer import zynaudioplayer
from zyngine.ctrldev.zynthian_ctrldev_base import zynthian_ctrldev_base, zynthian_ctrldev_zynmixer, zynthian_ctrldev_zynpad
from zyngine.ctrldev.zynthian_ctrldev_base_extended import ButtonTimer, CONST, IntervalTimer
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
# Capacitive touch on the 4 channel-strip knobs - note numbers coincide with
# their own CC numbers. Plain touch just resets that knob's easing
# accumulator (see e.g. ZynpotRotate.reset()); Alt+touch is a deliberate
# zynpot "switch" push instead (see midi_event's ZYNPOT_KNOBS handling) -
# touch alone can't reliably stand in for a press (fires on any contact,
# e.g. just resting a finger while turning the knob), so Alt makes it a
# two-hand gesture instead of incidental.
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
BTN_STEP = 0x2C		# activates StepSeq mode (see StepSeqHandler)
BTN_NOTE = 0x2D		# activates Play mode (see PlayHandler)
BTN_DRUM = 0x2E		# unbound for now, reserved for a future drum mode
BTN_PERFORM = 0x2F		# toggles between the Mixer and Zynpad screens
BTN_SHIFT = 0x30
BTN_ALT = 0x31
BTN_PATTERN_SONG = 0x32	# labelled "Metronome" in DrivenByMoss - see midi_event
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

# MIDI channel (0-indexed) reserved for PlayHandler's live-played notes,
# injected via the driver's midiproc_task - see that method and
# PlayHandler._send_note() for the full story. Distinct from Fire's own raw
# input, always channel 0 (buttons/pads) - see unroute_from_chains below,
# which keeps channel 0 out of chain routing while leaving this one (and
# every other channel) open. Combined with ACTI mode (zmip_set_flag_active_
# chain, set in the driver's init()), any note sent on this channel routes
# to whichever chain is currently active, regardless of that chain's own
# configured MIDI channel - so the exact value here doesn't matter beyond
# "not 0" - EXCEPT it also must avoid zynthian_gui_config.master_midi_channel
# (a completely separate feature - a channel whose incoming notes
# zynthian_state_manager.zynmidi_read() intercepts and maps through
# master_midi_note_cuia straight into CUIA actions, e.g. screen changes,
# transport, recording - confirmed the hard way: every played note here
# triggered a random CUIA when this collided with it). See
# _resolve_play_midi_chan(), which picks around it dynamically rather than
# risk this constant silently colliding with that (env-var-configured,
# so not knowable at import time) channel.
PLAY_MIDI_CHAN = 1


def _resolve_play_midi_chan():
    """PLAY_MIDI_CHAN, unless it collides with master_midi_channel (see
    PLAY_MIDI_CHAN's own comment) - in which case, the first channel in
    1..15 that collides with neither that nor Fire's own raw channel (0).
    Called fresh each time PlayHandler activates, not just once, in case
    the master channel setting changes while zynthian is running."""
    avoid = {0, zynthian_gui_config.master_midi_channel}
    if PLAY_MIDI_CHAN not in avoid:
        return PLAY_MIDI_CHAN
    for chan in range(1, 16):
        if chan not in avoid:
            return chan
    return PLAY_MIDI_CHAN  # unreachable in practice - would need 15 channels all reserved

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
LED_SHIFT = BTN_SHIFT
LED_STEP = BTN_STEP
LED_NOTE = BTN_NOTE
LED_DRUM = BTN_DRUM
LED_ALT = BTN_ALT
LED_PLAY = BTN_PLAY
LED_STOP = BTN_STOP
LED_RECORD = BTN_RECORD
LED_METRONOME = BTN_PATTERN_SONG  # labelled "Metronome" in DrivenByMoss, same yellow-green family as Play

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

# Yellow-red: Step, Note, Drum, Perform, Shift, Record - confirmed against
# DrivenByMoss's own FireColorManager.getColor() (SEQUENCER/NOTE/DRUM/
# SESSION/SHIFT/RECORD case): 1/3 are red (dull/high), 2/4 are
# orange/"yellow" (dull/high) - opposite pairing from an earlier, wrong
# transcription of the SEGGER blog's own table (which had 1=dull yellow,
# swapped with what's actually dull red - found on real hardware: idle
# buttons lit dull red instead of the intended dull yellow).
LED_YR_DULL_RED = 1
LED_YR_DULL_YELLOW = 2
LED_YR_HIGH_RED = 3
LED_YR_HIGH_YELLOW = 4

# Yellow-only: Alt, Stop
LED_Y_DULL = 1
LED_Y_HIGH = 2

# Yellow-green: Pattern/Song ("Metronome" in DrivenByMoss), Play - confirmed
# against FireColorManager.getColor()'s METRONOME/PLAY case, same 1/3=green,
# 2/4=orange("yellow") pairing pattern as yellow-red above.
LED_YG_DULL_GREEN = 1
LED_YG_DULL_YELLOW = 2
LED_YG_HIGH_GREEN = 3
LED_YG_HIGH_YELLOW = 4

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


# Same table as zynthian_gui_patterneditor.py's own NOTE_NAMES - kept as a
# local copy rather than imported (zyngine/ctrldev doesn't otherwise depend
# on zyngui screen modules) - used by PlayHandler for its OLED scale/tonic
# status line.
NOTE_NAMES = ("C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B")


def _alt_mode():
    """Zynthian's persistent, global "alt mode" toggle (Bank/Mode button /
    TOGGLE_ALT_MODE CUIA, see zyngui/zynthian_gui.py's alt_mode attribute) -
    read live from the GUI singleton rather than tracked locally per-handler,
    so it can't go stale if toggled some other way than through this driver.
    Fire's own Shift button (BTN_SHIFT) is wired to the exact same toggle -
    see midi_event's BTN_SHIFT handling - rather than tracking its own
    separate "shifted" boolean, so Shift is naturally sticky (a press flips
    this and it stays flipped, like DrivenByMoss's Fire implementation, as
    opposed to a momentary hold) and Shift/Bank can never disagree about the
    current state, since they're both just alternate ways to read/flip this
    one value. Every "shifted"
    modifier check across all handlers (Mixer's Shift+Solo-N mute, Zynpad's
    Shift+Pad, StepSeq/PlayHandler's Shift+Grid/Pattern combos, etc.) is
    forwarded this same live value on every button press - see midi_event's
    two _current_handler.note_on/note_off calls - not a locally-cached one.
    NOT the same thing as momentarily holding Fire's own physical Alt button
    (BTN_ALT) - that's a separate, unrelated modifier (see e.g. Alt+Solo-N in
    MixerHandler). zyngui can still be None here: driver loading is
    triggered synchronously off midi_autoconnect's device-connect callback,
    which can run before the GUI singleton exists yet - a first refresh()
    reaching this (observed via zynpad.init()'s zynsigman.register_queued()
    apparently replaying an already-queued SS_SEQ_REFRESH straight into the
    new subscriber) would otherwise crash driver.init() with
    'NoneType' object has no attribute 'get_alt_mode', aborting the load -
    which, worse, leaves an orphaned midiproc subprocess behind (it's spawned
    earlier in init(), so it's already running by the time this would crash)
    that wins the JACK client name and gets autoconnected, while the actual
    successfully-loaded retry's own midiproc silently never gets wired up at
    all. Default to alt mode off rather than propagate the crash."""
    zyngui = zynthian_gui_config.zyngui
    return zyngui.get_alt_mode() if zyngui is not None else False


def _toggle_alt_mode():
    """Flips alt_mode directly instead of through the TOGGLE_ALT_MODE CUIA -
    state_manager.send_cuia() only enqueues the call (self.cuia_queue is
    drained later, elsewhere), so a repaint immediately after sending it would
    still read the pre-toggle value. zynthian_gui.cuia_toggle_alt_mode() is a
    pure attribute flip with no other side effects, so this reproduces it
    exactly, just synchronously - callers can safely repaint right after."""
    zyngui = zynthian_gui_config.zyngui
    if zyngui is not None:
        zyngui.alt_mode = not _alt_mode()


# --------------------------------------------------------------------------
# Feedback LEDs controller
# --------------------------------------------------------------------------
class FeedbackLEDs:
    def __init__(self, idev):
        self._idev = idev

    def all_off(self):
        self.control_leds_off()

    def control_leds_off(self):
        for led in (LED_BANK, LED_SOLO_1, LED_SOLO_2, LED_SOLO_3, LED_SOLO_4, LED_BROWSER, LED_PERFORM, LED_SHIFT,
                    LED_STEP, LED_NOTE, LED_DRUM, LED_ALT, LED_PLAY, LED_STOP, LED_RECORD, LED_METRONOME):
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
# OLED display (SysEx, 128x64 monochrome, sent as 8 horizontal 8px-tall
# stripes) - see zynthian_ctrldev_akai_fire_protocol.md for the wire format
# and the BIT_MUTATE remap table (transcribed there from DrivenByMoss's
# FireDisplay.java). This is the first use of the OLED in this driver - the
# SysEx encoding itself is what's unverified against real hardware, so
# _refresh_oled() (top-level driver) deliberately keeps the content trivial
# (mode name only) for the first hardware test; richer content (a scale
# indicator, once scales exist, etc.) is future work once this is confirmed
# to actually paint correctly.
# --------------------------------------------------------------------------
class OledDisplay:
    WIDTH = 128
    HEIGHT = 64
    STRIPES = 8
    STRIPE_HEIGHT = 8
    STRIPE_SIZE = 147   # ceil(128*8/7) - 1 bit/pixel packed 7 bits/byte
    SYSEX_HEADER = (0xF0, 0x47, 0x7F, 0x43, 0x0E)

    # zynthian's own default UI typeface (zynthian_gui_config.font_family),
    # shipped in-repo under fonts/ - reuse it here so the OLED matches the
    # touchscreen's look rather than falling back to Pillow's generic bitmap
    # font. Located relative to this file rather than via an env var
    # (ZYNTHIAN_UI_DIR) so it still resolves in a plain checkout with no
    # zynthian environment sourced.
    FONT_PATH = os.path.join(
        os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
        "fonts", "Audiowide", "Audiowide-Regular.ttf")

    # Remaps an 8-tall x 7-wide pixel block into the packed byte layout the
    # Fire's OLED controller expects - not row-major/page order, see the
    # protocol doc's "Bit packing" section for where this table (and the
    # encode loop in _encode_stripe below) comes from.
    BIT_MUTATE = (
        (13, 19, 25, 31, 37, 43, 49),
        (0, 20, 26, 32, 38, 44, 50),
        (1, 7, 27, 33, 39, 45, 51),
        (2, 8, 14, 34, 40, 46, 52),
        (3, 9, 15, 21, 41, 47, 53),
        (4, 10, 16, 22, 28, 48, 54),
        (5, 11, 17, 23, 29, 35, 55),
        (6, 12, 18, 24, 30, 36, 42),
    )

    # The OLED goes to sleep without periodic traffic - re-send a stripe if
    # this long has passed since it was last sent, even if unchanged
    # (matches DrivenByMoss's own anti-sleep behavior, see the protocol doc).
    ANTI_SLEEP_MS = 3000

    def __init__(self, idev):
        self._idev = idev
        self._image = Image.new('1', (self.WIDTH, self.HEIGHT), 0)
        self._draw = ImageDraw.Draw(self._image)
        self._fonts = {}  # pixel size -> loaded ImageFont, see _font()
        self._stripe_cache = [None] * self.STRIPES
        self._stripe_sent_at = [0.0] * self.STRIPES

    def _font(self, size):
        """TrueType fonts (unlike Pillow's built-in bitmap default) take an
        arbitrary pixel size directly - cache one instance per size actually
        used rather than reloading the file every call. Falls back to the
        built-in font if FONT_PATH is somehow missing (e.g. an install that
        moved/renamed fonts/), rather than crashing the driver over a
        cosmetic-only failure."""
        font = self._fonts.get(size)
        if font is None:
            try:
                font = ImageFont.truetype(self.FONT_PATH, size)
            except OSError:
                font = ImageFont.load_default()
            self._fonts[size] = font
        return font

    def clear(self):
        self._draw.rectangle((0, 0, self.WIDTH, self.HEIGHT), fill=0)

    def text(self, x, y, s, size=16, center_x=False, center_y=False):
        """Draw text with its top-left visual bbox corner at (x, y), in the
        given pixel size. center_x/center_y ignore x/y on their respective
        axis and center the text (by its actual rendered bbox, not the
        font's nominal metrics) on the display instead."""
        font = self._font(size)
        w, h, left, top = self._text_size(font, s)
        if center_x:
            x = (self.WIDTH - w) // 2
        if center_y:
            y = (self.HEIGHT - h) // 2
        self._draw.text((x - left, y - top), s, font=font, fill=1)

    def text_fit(self, y, s, max_size, min_size=8):
        """Center s horizontally at the largest size (<= max_size, >=
        min_size) whose rendered width still fits within WIDTH - for a
        label of unpredictable length (e.g. a scale name) that text()'s
        fixed size can't be pre-sized for without risking it silently
        running off the screen edge."""
        size = max_size
        while size > min_size and self._text_size(self._font(size), s)[0] > self.WIDTH:
            size -= 1
        self.text(0, y, s, size=size, center_x=True)

    def _text_size(self, font, s):
        """(width, height, bbox_left, bbox_top) for s in the given font -
        textbbox() only works for TrueType fonts on some Pillow versions
        (raises ValueError for the legacy bitmap default font _font() falls
        back to on older Pillow), so fall back to the older getsize() API,
        which has no bbox offset to account for."""
        try:
            left, top, right, bottom = self._draw.textbbox((0, 0), s, font=font)
            return max(1, right - left), max(1, bottom - top), left, top
        except (ValueError, AttributeError):
            w, h = font.getsize(s)
            return max(1, w), max(1, h), 0, 0

    def update(self, force=False):
        """Push whatever stripes changed (or are due for an anti-sleep
        refresh) since the last call - cheap to call often; an unchanged,
        not-yet-due stripe costs nothing but the encode-and-compare."""
        now = time.time()
        pixels = self._image.load()
        for stripe in range(self.STRIPES):
            payload = self._encode_stripe(pixels, stripe)
            due = (now - self._stripe_sent_at[stripe]) * 1000 >= self.ANTI_SLEEP_MS
            if not force and payload == self._stripe_cache[stripe] and not due:
                continue
            self._send_stripe(stripe, payload)
            self._stripe_cache[stripe] = payload
            self._stripe_sent_at[stripe] = now

    def _encode_stripe(self, pixels, stripe):
        payload = bytearray(self.STRIPE_SIZE)
        base_y = stripe * self.STRIPE_HEIGHT
        for y in range(self.STRIPE_HEIGHT):
            row = base_y + y
            mutate_row = self.BIT_MUTATE[y]
            for x in range(self.WIDTH):
                if not pixels[x, row]:
                    continue
                remap_bit = mutate_row[x % 7]
                byte_idx = (x // 7) * 8 + remap_bit // 7
                payload[byte_idx] |= 1 << (remap_bit % 7)
        return bytes(payload)

    def _send_stripe(self, stripe, payload):
        # Byte layout: [stripe, stripe, colStart, colEnd, <147 payload bytes>]
        # - see the protocol doc for why the stripe index is duplicated and
        # why colStart/colEnd are always the full-width 0x00/0x7F here.
        body = (stripe, stripe, 0x00, 0x7F) + tuple(payload)
        msg = bytes(self.SYSEX_HEADER + (len(body) // 128, len(body) % 128) + body + (0xF7,))
        lib_zyncore.dev_send_midi_event(self._idev, msg, len(msg))

    def all_off(self):
        self.clear()
        self.update(force=True)


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
# Volume/Pan/Filter/Resonance knob rotate -> zynpot 0-3. A single shared
# instance lives on the top-level driver (see _default_cc_change) and is
# used whenever the active mode has no more specific use for a given knob
# (its own cc_change/note_on decline by returning falsy) - including
# whatever screen has no dedicated mode of its own at all, now that
# DeviceHandler is gone. PlayHandler is the only handler with any bespoke
# use left for one of these 4 (Volume, repurposed for tonic) - the other 3
# fall through to this same shared default there too.
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


def _select_knob_arrow(state_manager, ccval, is_alt=False):
    """Select knob rotate -> ARROW_UP/DOWN by default (most zynthian menus
    are vertical lists), Alt+rotate -> ARROW_LEFT/RIGHT instead - the
    default list-navigation any mode falls back to when it has no more
    specific use for the Select knob - called from the top-level driver's
    own _default_cc_change, same shared-fallback story as ZynpotRotate
    above. Mixer/StepSeq/Play all keep their own specialized use of Select
    instead (chain-scroll, pitch-adjust, octave-shift respectively - see
    each one's own cc_change) and never reach this. Select's own encoder
    isn't noisy like the other 4, so no jitter filter here either."""
    delta = ccval if ccval < 64 else ccval - 128
    if is_alt:
        state_manager.send_cuia("ARROW_RIGHT" if delta > 0 else "ARROW_LEFT")
    else:
        # Inverted relative to Left/Right's own sign convention above -
        # confirmed on real hardware: clockwise (positive delta) reads as
        # "up" through a vertical list, not "down".
        state_manager.send_cuia("ARROW_UP" if delta > 0 else "ARROW_DOWN")


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

        # Touch just resets this knob's own easing accumulator (Volume/Pan
        # only - Filter/Resonance aren't used in Mixer mode, so they decline
        # and fall to the top-level driver's own shared default instead).
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

        # No specific use for Select's own push here - declines to the
        # top-level driver's own default (zynpot switch 3).
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
        # No zynpad-specific use for the 4 knobs/Select/its own push - just
        # decline them (base class no-ops) and let the top-level driver's own
        # shared default handle them, same as every screen with no dedicated
        # mode of its own (see midi_event's _default_cc_change/_default_note_on).

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
        self._on_shifted_override(shifted_override)

        if not (PAD_NOTE_BASE <= note < PAD_NOTE_BASE + 64):
            # Not a pad - knob touch, Select push, etc. - nothing
            # zynpad-specific for any of those, decline and let the
            # top-level driver's own default handle it.
            return False

        index = note - PAD_NOTE_BASE
        row, col = index // 16, index % 16
        lcol, lrow = self._logical_xy(col, row)
        if lcol >= self._zynseq.col_in_bank or lrow >= self._zynseq.col_in_bank:
            return False
        seq = self._zynseq.get_pad_from_xy(lcol, lrow)

        if self._is_shifted:
            # Shift+Pad opens StepSeq directly on that pad's pattern, same
            # shortcut as the APC key25 mk2 driver's own "SHIFT + PAD ->
            # StepSeq". Screen-driven like BTN_STEP (see midi_event): selects
            # the pad, then lets SCREEN_PATTERN_EDITOR (which reads zynpad's
            # now-updated selected_pad) and screen-follow take it from there
            # - so this only actually lands on StepSeq while screen-linked,
            # consistent with every other mode switch in this driver.
            self._select_pad(seq)
            self._state_manager.send_cuia("SCREEN_PATTERN_EDITOR")
            return True

        self._libseq.togglePlayState(self._zynseq.bank, seq)
        return True

    # No cc_change override - nothing zynpad-specific for the 4 knobs or
    # Select either, base class declines everything (see note_on above).


# zynseq keymaps live here on a real box (see zynthian_gui_patterneditor.py's
# own CONFIG_ROOT) - not present in this bare checkout (CLAUDE.md).
ZYNSEQ_CONFIG_ROOT = "/zynthian/zynthian-data/zynseq"


# --------------------------------------------------------------------------
# Handle StepSeq (Note Sequencer style step editor, active on the
# pattern_editor screen, reached via BTN_STEP)
# --------------------------------------------------------------------------
#
# See zynthian_ctrldev_akai_fire_stepseq_plan.md for the full design this
# implements. Short version: pad grid is row=pitch (from a keymap built the
# same way zynthian_gui_patterneditor.py's load_keymap() does - chromatic, or
# a scales.json scale relative to the pattern's tonic; only 4 rows visible at
# once, BTN_PAT_UP/DOWN scroll the window), col=step (only 16 visible at
# once, Grid Left/Right page through the pattern). A pad press is a 4-way
# gesture (see _on_grid_press/_on_grid_release): plain tap toggles the note;
# holding past the long-press threshold toggles it in/out of
# self._selected_notes (a purely local, UI-side "selection" - zynseq has no
# such concept); pressing a second pad in the same row while the first is
# still held is read as an extend-duration drag, matching the touchscreen's
# own drag convention (duration = distance between the two steps). Knobs
# then edit whatever's selected (or the last-tapped note, if selection is
# empty) - see cc_change(). Play/Stop need no special-casing here: the
# existing global TOGGLE_PLAY/STOP CUIAs already target this pattern's own
# sequence whenever the "pattern_editor" screen is showing (see
# zynthian_gui.cuia_toggle_audio_play/cuia_stop_audio_play), which is exactly
# the screen this mode is active on.
class StepSeqHandler(ModeHandlerBase):

    DEFAULT_VELOCITY = 100
    DEFAULT_DURATION = 1

    # Note cell colors: full at a note's start step, dimmer across the rest
    # of its sustain (its "tail") - selected notes get the plan's dark-yellow
    # overlay instead of the normal teal, at both brightness levels.
    COLOR_EMPTY = (0, 0, 0)
    COLOR_NOTE = (0, 60, 60)
    COLOR_NOTE_TAIL = (0, 20, 20)
    COLOR_SELECTED = (70, 60, 0)
    COLOR_SELECTED_TAIL = (35, 30, 0)

    # Playhead cursor: an otherwise-empty cell at the current step gets a dim
    # green tint (rather than staying black) so the column reads as "the
    # cursor" even where there's no note; a cell that already has a note
    # keeps its own color, just with more green mixed in, so the cursor is
    # visible passing through without hiding note/selection state.
    COLOR_PLAYHEAD_EMPTY = (0, 30, 0)
    PLAYHEAD_GREEN_BOOST = 40
    PLAYHEAD_POLL_MS = 200   # matches zynthian_gui's own status-refresh rate

    # Tonic guidance line: every row whose note is the pattern's root note
    # (note%12 == getTonic(), see _load_keymap/_paint_pad) gets a dim white
    # tint spanning the whole row, empty cells included - a constant visual
    # reference for "where the root note is" while scrolling, in Chromatic
    # too (not just an active scale - the tonic field means something
    # either way, see _adjust_tonic). Additive on top of whatever else the
    # cell would show (note/tail/selected/playhead), not a replacement, so
    # it reads as a guide line rather than hiding other state; a small dim
    # white boost stays visibly distinct from the playhead's green and the
    # selection's yellow regardless of which of those the row also has.
    COLOR_TONIC_EMPTY = (18, 18, 18)
    TONIC_ROW_BOOST = 18

    def __init__(self, state_manager, leds: FeedbackLEDs, pads: PadLEDs, oled_refresh_cb=None):
        super().__init__(state_manager)
        self._leds = leds
        self._pads = pads
        self._oled_refresh_cb = oled_refresh_cb  # top-level driver's _refresh_oled, see _on_scale_changed
        self._libseq = self._zynseq.libseq
        self._knobs_ease = KnobJitterFilter()
        self._is_alt = False

        self._scale_label = "Chromatic"  # OLED status line, see _load_keymap()/oled_status()
        self._tonic = 0          # root note (0-11), see _load_keymap()/_paint_pad's tonic-row tint
        self._degree_count = 12  # rows/octave, see _load_keymap()/note_on's Alt+Pattern Up/Down
        self._keymap = [{"note": n} for n in range(128)]
        self._row_offset = 0    # index into _keymap of the topmost visible row
        self._step_page = 0     # which 16-step page of the pattern is visible
        self._steps = 16        # getSteps() of the current pattern, cached in refresh()

        # (step, note) pairs - see class comment. _last_note is the fallback
        # group-edit target when _selected_notes is empty (the most recently
        # tapped-on or added note).
        self._selected_notes = set()
        self._last_note = None

        # Pads currently physically held, note -> (row, col, step, note_val,
        # press_ts) - used to tell a plain tap from a long-press (on release,
        # by elapsed time) and to detect the second-pad-of-an-extend-gesture
        # case (see _on_grid_press). A pad popped from here by that gesture
        # is "consumed" - its own eventual release finds nothing and no-ops,
        # same idea as DrivenByMoss's button.setConsumed().
        self._held = {}

        # Playhead cursor - absolute step index (matching self._step_page's
        # domain) currently playing, or None while stopped/not visible.
        # zynseq has no push signal for this (SS_SEQ_PROGRESS exists but
        # nothing in this codebase actually emits it), so this is polled the
        # same way zynthian_gui_patterneditor.py's own refresh_status() does
        # it - via getPatternPlayhead() - just on our own timer instead of
        # riding the GUI's status-refresh thread.
        self._playhead_step = None
        self._playhead_timer = IntervalTimer()

    def set_alt(self, state):
        # Momentary BTN_ALT, used as a modifier for the Filter knob
        # (Alt+Filter = play chance instead of stutter count), Grid
        # Left/Right (Alt+Grid = cycle scale instead of step-paging - see
        # note_on/_cycle_scale) and Pattern Up/Down (Alt+Pattern = scroll a
        # full octave's worth of rows instead of 1 - see note_on/
        # _scroll_rows; Shift+Pattern scrolls a full 4-row page instead).
        self._is_alt = state

    # ----------------------------------------------------------------------
    # Keymap / view
    # ----------------------------------------------------------------------
    def _load_keymap(self, recenter=True):
        """Rebuild self._keymap from the current pattern's scale/tonic,
        mirroring zynthian_gui_patterneditor.py's load_keymap() (chromatic,
        or a scales.json scale) - custom .midnam keymaps and CC display mode
        are out of scope here.

        This is the pattern's own Scale/Tonic (getScale()/getTonic() - the
        same fields the touchscreen pattern editor's own Scale/Tonic menu
        items set, see _cycle_scale/_adjust_tonic below), not a StepSeq-only
        setting - unlike PlayHandler's scale, which is deliberately
        independent (see its own class comment), a pattern's scale is
        already real, shared, persisted pattern state, so there's no reason
        for the Fire to keep a second copy of it.

        recenter=True (refresh() - a new pattern/mode activation) re-centers
        the row view on middle C, same as before. recenter=False
        (_on_scale_changed() - still the same pattern, just its scale/tonic
        changed) instead keeps showing roughly the same pitch range: the
        note that was at the current row_offset gets relocated in the new
        keymap (its nearest match, since the exact note may no longer be
        in-scale) and the view re-anchors there - changing scale would
        otherwise always yank the view back to middle C, losing whatever
        octave you'd scrolled to."""
        anchor_note = None
        if not recenter and self._keymap and 0 <= self._row_offset < len(self._keymap):
            anchor_note = self._keymap[self._row_offset]["note"]

        scale = self._libseq.getScale()
        tonic = self._libseq.getTonic()
        self._tonic = tonic
        self._degree_count = 12  # rows/octave for Alt+Pattern Up/Down - see note_on
        keymap = []
        self._scale_label = "Chromatic"
        if scale > 1:
            try:
                with open(ZYNSEQ_CONFIG_ROOT + "/scales.json") as f:
                    data = json.load(f)
                if scale <= len(data):
                    entry = data[scale - 1]
                    for octave in range(9):
                        for offset in entry["scale"]:
                            note = tonic + offset + octave * 12
                            if note > 127:
                                break
                            keymap.append({"note": note})
                    self._scale_label = f"{NOTE_NAMES[tonic]} {entry['name']}"
                    self._degree_count = len(entry["scale"])
            except Exception as ex:
                logging.warning(f"StepSeqHandler: can't load scales.json => {ex}")
        if not keymap:
            keymap = [{"note": n} for n in range(128)]
        self._keymap = keymap

        if recenter or anchor_note is None:
            idx = next((i for i, e in enumerate(keymap) if e["note"] >= 60), 0)
        else:
            idx = min(range(len(keymap)), key=lambda i: abs(keymap[i]["note"] - anchor_note))
        self._row_offset = max(0, min(idx - 1, len(keymap) - 4))

    def _cycle_scale(self, delta):
        """Alt+Grid Left/Right - cycles the pattern's Scale param through 0
        (Chromatic) .. len(scales.json) inclusive, wrapping either way -
        same convention (and the same file) PlayHandler's own _cycle_scale
        uses, just applied to zynseq's real per-pattern field instead of a
        StepSeq-local one."""
        try:
            with open(ZYNSEQ_CONFIG_ROOT + "/scales.json") as f:
                count = len(json.load(f))
        except Exception as ex:
            logging.warning(f"StepSeqHandler: can't load scales.json => {ex}")
            count = 0
        scale = (self._libseq.getScale() + (1 if delta > 0 else -1)) % (count + 1)
        self._libseq.setScale(scale)
        self._on_scale_changed()

    def _adjust_tonic(self, delta):
        """Shift+Grid Left/Right - adjusts the pattern's Tonic (root note),
        wrapping 0-11 (C-B)."""
        tonic = (self._libseq.getTonic() + (1 if delta > 0 else -1)) % 12
        self._libseq.setTonic(tonic)
        self._on_scale_changed()

    def _on_scale_changed(self):
        self._load_keymap(recenter=False)
        self._paint_all()
        if self._oled_refresh_cb:
            self._oled_refresh_cb()

    def oled_status(self):
        """Secondary OLED line while StepSeq is active (see the top-level
        driver's _refresh_oled) - the current pattern's scale/tonic, e.g.
        'C Major', or 'Chromatic'."""
        return self._scale_label

    def _keymap_index(self, phys_row):
        """Physical pad row (0=top) -> index into self._keymap, or None if
        that row is past either edge of the keymap right now."""
        idx = self._row_offset + (3 - phys_row)
        return idx if 0 <= idx < len(self._keymap) else None

    def _pad_for(self, step, note_val):
        """Inverse lookup: (step, note) -> (phys_row, phys_col) if currently
        visible, else None. Linear scan over the keymap - fine at
        button-press rates, not called anywhere hot."""
        col = step - self._step_page * 16
        if not (0 <= col < 16):
            return None
        idx = next((i for i, e in enumerate(self._keymap) if e["note"] == note_val), None)
        if idx is None:
            return None
        row = self._row_offset + 3 - idx
        return (row, col) if 0 <= row < 4 else None

    def _page_steps(self, direction):
        max_page = max(0, (self._steps - 1) // 16)
        self._step_page = max(0, min(self._step_page + direction, max_page))
        self._paint_all()

    def _scroll_rows(self, direction, amount=1):
        """Plain Pattern Up/Down scroll by 1 row; Shift+Pattern Up/Down (see
        note_on) passes amount=4 (a full page - the visible window height);
        Alt+Pattern Up/Down passes amount=self._degree_count (one octave's
        worth of rows - 12 for Chromatic, or however many degrees the
        active scale has, see _load_keymap)."""
        max_offset = max(0, len(self._keymap) - 4)
        self._row_offset = max(0, min(self._row_offset + direction * amount, max_offset))
        self._paint_all()

    # ----------------------------------------------------------------------
    # Painting
    # ----------------------------------------------------------------------
    def set_active(self, active):
        super().set_active(active)
        if active:
            self._playhead_timer.add("playhead", self.PLAYHEAD_POLL_MS, self._poll_playhead)
        else:
            self._playhead_timer.remove("playhead")

    def refresh(self):
        self._load_keymap()
        self._steps = self._libseq.getSteps()
        self._step_page = 0
        self._selected_notes = set()
        self._last_note = None
        self._held = {}
        self._playhead_step = None
        self._paint_all()
        self._update_leds()

    def _paint_all(self):
        for row in range(4):
            for col in range(16):
                self._paint_pad(row, col)

    def _paint_column(self, step):
        col = step - self._step_page * 16
        if 0 <= col < 16:
            for row in range(4):
                self._paint_pad(row, col)

    def _poll_playhead(self, name):
        seq = self._get_selected_sequence()
        new_step = None
        if seq is not None:
            state = self._libseq.getPlayState(self._zynseq.bank, seq)
            if state in (zynseq.SEQ_PLAYING, zynseq.SEQ_STARTING, zynseq.SEQ_RESTARTING):
                new_step = self._libseq.getPatternPlayhead()
        if new_step == self._playhead_step:
            return
        old_step = self._playhead_step
        self._playhead_step = new_step
        # Only repaint the columns that actually changed - a full-grid
        # repaint every tick would be needless SysEx traffic.
        if old_step is not None:
            self._paint_column(old_step)
        if new_step is not None:
            self._paint_column(new_step)

    def _paint_pad(self, row, col):
        note_pad = _pad(row, col)
        step = self._step_page * 16 + col
        idx = self._keymap_index(row)
        if idx is None or step >= self._steps:
            self._pads.pad_off(note_pad)
            return
        note_val = self._keymap[idx]["note"]
        is_tonic_row = note_val % 12 == self._tonic
        start = self._libseq.getNoteStart(step, note_val)
        on_playhead = step == self._playhead_step
        if start < 0:
            if on_playhead:
                color = self.COLOR_PLAYHEAD_EMPTY
            elif is_tonic_row:
                color = self.COLOR_TONIC_EMPTY
            else:
                color = self.COLOR_EMPTY
            self._pads.set_pad(note_pad, *color)
            return
        selected = (start, note_val) in self._selected_notes
        if step == start:
            color = self.COLOR_SELECTED if selected else self.COLOR_NOTE
        else:
            color = self.COLOR_SELECTED_TAIL if selected else self.COLOR_NOTE_TAIL
        if on_playhead:
            r, g, b = color
            color = (r, min(127, g + self.PLAYHEAD_GREEN_BOOST), b)
        if is_tonic_row:
            r, g, b = color
            boost = self.TONIC_ROW_BOOST
            color = (min(127, r + boost), min(127, g + boost), min(127, b + boost))
        self._pads.set_pad(note_pad, *color)

    def _paint_pad_at(self, step, note_val):
        pos = self._pad_for(step, note_val)
        if pos is not None:
            self._paint_pad(*pos)

    def _update_leds(self):
        # Solo1 (Stop) is a momentary action with no state, so no LED for it.
        self._leds.led_off(LED_SOLO_1)
        chain = self._get_chain()
        self._leds.led_on(LED_SOLO_2, LED_GREEN_HIGH) if chain is not None and self._zynmixer.get_mute(chain.mixer_chan) \
            else self._leds.led_off(LED_SOLO_2)
        self._leds.led_on(LED_SOLO_3, LED_GREEN_HIGH) if chain is not None and self._zynmixer.get_solo(chain.mixer_chan) \
            else self._leds.led_off(LED_SOLO_3)
        self._leds.led_on(LED_SOLO_4, LED_GREEN_HIGH) if self._libseq.getQuantizeNotes() \
            else self._leds.led_off(LED_SOLO_4)

    # ----------------------------------------------------------------------
    # Note add/remove/select/extend - the pad-press state machine
    # ----------------------------------------------------------------------
    def _on_grid_press(self, note):
        idx = note - PAD_NOTE_BASE
        row, col = idx // 16, idx % 16
        step = self._step_page * 16 + col
        krow = self._keymap_index(row)
        if krow is None or step >= self._steps:
            return True
        note_val = self._keymap[krow]["note"]

        # A second pad in the same row, pressed while another is still held,
        # is the extend-duration gesture - consumes both (see class comment).
        other = next((n for n, info in self._held.items() if info[0] == row and n != note), None)
        if other is not None:
            self._extend_duration(self._held.pop(other), step, note_val)
            return True

        self._held[note] = (row, col, step, note_val, time.time())
        return True

    def _on_grid_release(self, note):
        info = self._held.pop(note, None)
        if info is None:
            return  # consumed by an extend-duration gesture already
        _, _, step, note_val, ts = info
        if time.time() - ts >= CONST.PT_LONG_TIME:
            self._toggle_selection(step, note_val)
        else:
            self._toggle_note(step, note_val)

    def _extend_duration(self, start_info, end_step, note_val):
        _, _, start_step, start_note_val, _ = start_info
        if start_note_val != note_val or start_step == end_step:
            return
        lo, hi = sorted((start_step, end_step))
        if self._libseq.getNoteStart(lo, note_val) != lo:
            return  # nothing to extend - no note actually starts at lo
        # Inclusive: the note should sustain through the second pad you
        # pressed, not end right before it - unlike the touchscreen's own
        # drag gesture (distance, not count), this is two discrete pad
        # presses, so "extend to this pad" reads as "cover this step too".
        self._set_note_duration(lo, note_val, hi - lo + 1)
        self._paint_all()

    def _toggle_note(self, step, note_val):
        start = self._libseq.getNoteStart(step, note_val)
        if start >= 0:
            self._selected_notes.discard((start, note_val))
            if self._last_note == (start, note_val):
                self._last_note = None
            self._libseq.removeNote(start, note_val)
            self._paint_all()
        else:
            self._libseq.addNote(step, note_val, self.DEFAULT_VELOCITY, self.DEFAULT_DURATION, 0)
            self._last_note = (step, note_val)
            self._paint_pad_at(step, note_val)

    def _toggle_selection(self, step, note_val):
        start = self._libseq.getNoteStart(step, note_val)
        if start < 0:
            return  # long-pressing an empty cell is a no-op
        key = (start, note_val)
        if key in self._selected_notes:
            self._selected_notes.discard(key)
        else:
            self._selected_notes.add(key)
        self._paint_pad_at(start, note_val)

    # ----------------------------------------------------------------------
    # Group editing - applies to _selected_notes if non-empty, else to
    # _last_note alone
    # ----------------------------------------------------------------------
    def _current_targets(self):
        if self._selected_notes:
            return set(self._selected_notes)
        return {self._last_note} if self._last_note is not None else set()

    def _adjust_velocity(self, delta):
        for step, note_val in self._current_targets():
            vel = self._libseq.getNoteVelocity(step, note_val)
            if vel <= 0:
                continue
            self._libseq.setNoteVelocity(step, note_val, max(1, min(127, vel + delta)))
        self._paint_all()

    def _adjust_duration(self, delta):
        for step, note_val in self._current_targets():
            dur = self._libseq.getNoteDuration(step, note_val)
            if dur <= 0:
                continue
            self._set_note_duration(step, note_val, max(1, min(self._steps, dur + delta)))
        self._paint_all()

    def _adjust_stutter_count(self, delta):
        for step, note_val in self._current_targets():
            if self._libseq.getNoteDuration(step, note_val) <= 0:
                continue
            val = max(0, self._libseq.getStutterCount(step, note_val) + delta)
            self._libseq.setStutterCount(step, note_val, val)

    def _adjust_stutter_dur(self, delta):
        for step, note_val in self._current_targets():
            if self._libseq.getNoteDuration(step, note_val) <= 0:
                continue
            val = max(1, self._libseq.getStutterDur(step, note_val) + delta)
            self._libseq.setStutterDur(step, note_val, val)

    def _adjust_chance(self, delta):
        for step, note_val in self._current_targets():
            if self._libseq.getNoteDuration(step, note_val) <= 0:
                continue
            val = max(0, min(100, self._libseq.getNotePlayChance(step, note_val) + delta))
            self._libseq.setNotePlayChance(step, note_val, val)

    def _adjust_pitch(self, delta):
        """Rigid transpose of the whole selection (or _last_note alone) by
        `delta` keymap rows - all-or-nothing: if any target would clip past
        either edge of the keymap, the whole turn is refused. Pitch is part
        of a note's zynseq identity, so this is remove-all then add-all
        (not an in-place setter), preserving each note's velocity/duration/
        offset and re-pointing selection at the moved notes."""
        if not delta:
            return
        targets = self._current_targets()
        if not targets:
            return

        note_to_idx = {e["note"]: i for i, e in enumerate(self._keymap)}
        plan = []
        for step, note_val in targets:
            idx = note_to_idx.get(note_val)
            if idx is None:
                return
            new_idx = idx + delta
            if not (0 <= new_idx < len(self._keymap)):
                return
            plan.append((step, note_val, self._keymap[new_idx]["note"]))

        props = {
            (step, note_val): (
                self._libseq.getNoteVelocity(step, note_val),
                self._libseq.getNoteDuration(step, note_val),
                self._libseq.getNoteOffset(step, note_val),
            )
            for step, note_val, _ in plan
        }
        for step, note_val, _ in plan:
            self._libseq.removeNote(step, note_val)

        new_selected, new_last = set(), None
        for step, old_note, new_note in plan:
            vel, dur, off = props[(step, old_note)]
            self._libseq.addNote(step, new_note, vel, dur, off)
            new_selected.add((step, new_note))
            new_last = (step, new_note)

        if self._selected_notes:
            self._selected_notes = new_selected
        else:
            self._last_note = new_last
        self._paint_all()

    # ----------------------------------------------------------------------
    # Solo 1-4: Stop / Mute / Solo / Quantize (see plan doc)
    # ----------------------------------------------------------------------
    def _get_chain(self):
        seq = self._get_selected_sequence()
        if seq is None:
            return None
        chain_id = self._get_chain_id_by_sequence(self._zynseq.bank, seq)
        return self._chain_manager.chains.get(chain_id)

    def _stop_sequence(self):
        seq = self._get_selected_sequence()
        if seq is not None:
            self._libseq.setPlayState(self._zynseq.bank, seq, zynseq.SEQ_STOPPED)

    def _toggle_mute(self):
        chain = self._get_chain()
        if chain is None:
            return
        self._zynmixer.set_mute(chain.mixer_chan, self._zynmixer.get_mute(chain.mixer_chan) ^ 1, True)
        self._update_leds()

    def _toggle_solo(self):
        chain = self._get_chain()
        if chain is None:
            return
        self._zynmixer.set_solo(chain.mixer_chan, self._zynmixer.get_solo(chain.mixer_chan) ^ 1, True)
        self._update_leds()

    def _toggle_quantize(self):
        self._libseq.setQuantizeNotes(not self._libseq.getQuantizeNotes())
        self._update_leds()

    # ----------------------------------------------------------------------
    def note_on(self, note, velocity, shifted_override=None):
        if note in ZYNPOT_KNOBS:
            self._knobs_ease.reset(note)
            return True
        if note == BTN_SELECT_PRESS:
            # Plain press only - Alt+Select-push is BACK globally (see
            # midi_event, checked before dispatch ever reaches here). Select's
            # own plain push is otherwise unbound in this mode, so it clears
            # the selection instead.
            self._selected_notes.clear()
            self._paint_all()
            return True
        if note == BTN_SOLO_1:
            self._stop_sequence()
            return True
        if note == BTN_SOLO_2:
            self._toggle_mute()
            return True
        if note == BTN_SOLO_3:
            self._toggle_solo()
            return True
        if note == BTN_SOLO_4:
            self._toggle_quantize()
            return True
        if note == BTN_GRID_LEFT:
            if self._is_alt:
                self._cycle_scale(-1)
            elif shifted_override:
                self._adjust_tonic(-1)
            else:
                self._page_steps(-1)
            return True
        if note == BTN_GRID_RIGHT:
            if self._is_alt:
                self._cycle_scale(1)
            elif shifted_override:
                self._adjust_tonic(1)
            else:
                self._page_steps(1)
            return True
        if note == BTN_PAT_UP:
            if self._is_alt:
                self._scroll_rows(1, self._degree_count)
            elif shifted_override:
                self._scroll_rows(1, 4)
            else:
                self._scroll_rows(1)
            return True
        if note == BTN_PAT_DOWN:
            if self._is_alt:
                self._scroll_rows(-1, self._degree_count)
            elif shifted_override:
                self._scroll_rows(-1, 4)
            else:
                self._scroll_rows(-1)
            return True
        if PAD_NOTE_BASE <= note < PAD_NOTE_BASE + 64:
            return self._on_grid_press(note)
        return False

    def note_off(self, note, shifted_override=None):
        if PAD_NOTE_BASE <= note < PAD_NOTE_BASE + 64:
            self._on_grid_release(note)

    def cc_change(self, ccnum, ccval):
        if ccnum == KNOB_SELECT:
            delta = ccval if ccval < 64 else ccval - 128
            self._adjust_pitch(delta)
            return True

        if ccnum not in (KNOB_VOLUME, KNOB_PAN, KNOB_FILTER, KNOB_RESONANCE):
            return

        delta = self._knobs_ease.feed(ccnum, ccval)
        if delta is None:
            return True

        if ccnum == KNOB_VOLUME:
            self._adjust_velocity(delta)
        elif ccnum == KNOB_PAN:
            self._adjust_duration(delta)
        elif ccnum == KNOB_FILTER:
            self._adjust_chance(delta) if self._is_alt else self._adjust_stutter_count(delta)
        elif ccnum == KNOB_RESONANCE:
            self._adjust_stutter_dur(delta)
        return True


# --------------------------------------------------------------------------
# Handle Play (live note-playing keyboard on the pad grid, forced active via
# BTN_NOTE from any mode)
# --------------------------------------------------------------------------
#
# v1, deliberately minimal (more from the DrivenByMoss manual excerpt this
# was modeled on - Accent, quantize - is left for later): two layouts,
# toggled by pressing BTN_NOTE again while already in Play mode
# (DrivenByMoss's own convention is a double-press within a short window;
# this driver has no double-press infra elsewhere, and "press again while
# active" is simpler, needs no timing/threshold, and gives BTN_NOTE a reason
# to do something when Play mode is already up, where before it did
# nothing) - see toggle_layout(). Chromatic (the default on every fresh
# activation): the whole grid is one contiguous run through either every
# semitone, or - if a scale is selected, see BTN_PAT_UP/DOWN/KNOB_VOLUME
# below - every in-scale note only, in ascending pitch order; either way
# Select still shifts by a full octave. Piano: two independent 2-octave-apart
# "bands" (rows 0-1 = higher, rows 2-3 = lower), each a real
# white-key-row/black-key-row piano layout (DrivenByMoss's actual Piano
# View), deliberately NOT scale-constrained even if one is selected - a
# scale never removes/moves any of Piano's actual piano keys, since that's
# the whole point of it being a literal keyboard layout (unlike Chromatic,
# which has no such fixed physical meaning to preserve). Both layouts
# octave-shift via the same Select knob. Pressing a pad plays that note on
# the currently active chain for as long as it's held, at the velocity the
# pad itself reports.
#
# Scale/tonic (Chromatic layout only, see _midi_note/_apply_scale): BTN_PAT_UP/
# DOWN cycle through scales.json's entries (same file, and the same 1-based
# "scale index" convention, as StepSeqHandler._load_keymap/the pattern
# editor's own per-pattern Scale param - see zynthian_gui_patterneditor.py's
# load_keymap - though this is otherwise entirely independent state: Play
# mode's scale has nothing to do with whatever's set for a specific
# pattern), wrapping back to index 0 = Chromatic (no filter). The Volume
# knob (repurposed here, like StepSeq repurposes all 4 knobs for its own
# note-editing - see its own cc_change) adjusts the tonic. The OLED's second
# line (see oled_status()) shows the current selection, e.g. "C Major", or
# "Chromatic" - also the piece originally motivating adding the OLED at all.
#
# Getting a live note to actually reach an engine turned out to need real
# JACK-level MIDI I/O, not any Python/ctypes call - see the driver's own
# midiproc_task() for the mechanism (a small always-on JACK client, spawned
# in a subprocess by the base class's init_midiproc(), that passes Fire's
# own raw input through unchanged and additionally emits whatever
# PlayHandler queues via _send_note()/self._notes_queue). Two things tried
# and rejected first, see git history for the full story:
# lib_zyncore.write_zynmidi()/write_zynmidi_note_on() - turns out EVERY Note
# On/Off reaching zynthian_state_manager.zynmidi_read() (from real hardware
# or write_zynmidi() alike) only ever fires a zynsigman signal there
# (SS_MIDI_NOTE_ON/OFF - checked every subscriber: zynpad's preview
# highlight, the GUI's keyboard-CUIA emulation, APC's display sync - NOTHING
# forwards it to an engine). That whole function is a control-plane path
# (menus, mixer, MIDI learn, snapshot recall - CC/PC *do* reach chains from
# there) - not an audio path, so no write_zynmidi variant could ever have
# worked. zynseq.libseq.playNote() was tried too - it does reach engines via
# a real JACK output (zynseq:output -> ZynMidiRouter:step_in), just not
# reliably for an arbitrary chain (that output's own routing, not
# necessarily wired to whatever chain happens to be active). The actual
# working reference for real-time note generation from a ctrldev driver
# turned out to be zynthian_ctrldev_akai_mpk_mini_mk3_moder.py (+
# zynthian_ctrldev_base_moder.py) - a shipped driver that remaps incoming
# keyboard notes to a scale entirely within its own midiproc_task, proving
# the pattern this class's own midiproc_task() now follows too.
#
# The MIDI channel question is solved by ACTI mode
# (lib_zyncore.zmip_set_flag_active_chain(), enabled on this driver's own
# zmip in the top-level driver's init()) rather than by resolving which
# chain owns which channel ourselves: with ACTI on, any note arriving on
# this device's zmip routes to whichever chain is currently active,
# regardless of the note's own channel - so PLAY_MIDI_CHAN can be any fixed
# value, as long as it's not Fire's own raw channel (0) - see
# unroute_from_chains on the top-level driver class, which keeps channel 0
# out of chain routing while leaving every other channel (specifically
# PLAY_MIDI_CHAN) open.
#
# Unlike Mixer/Zynpad/StepSeq, this mode has no screen of its own to be
# screen-linked to - see BTN_NOTE in midi_event, it force-activates and
# unlinks instead.
class PlayHandler(ModeHandlerBase):

    BASE_NOTE_DEFAULT = 36   # matches DrivenByMoss PianoView's own default
    TOTAL_NOTES = 64         # 4 rows x 16 cols
    MAX_OCTAVE_STEPS = 5     # Select shifts by a full octave (12 semitones)
                              # per tick, capped at +-5 steps from the default
                              # - refuses (no-op) past that rather than
                              # clamping to a partial, sub-octave shift, which
                              # would misalign the grid pattern (root-note
                              # markers etc. would land in different columns
                              # than every other position) relative to it.

    # DrivenByMoss's own Play mode color scheme (FireColorManager.java +
    # Scales.getColor()): white for a regular playable note, blue for every
    # occurrence of the octave/root note (note%12==0 with no scale selected;
    # the actual tonic, once one is - see _paint_pad), green while held, red
    # while held AND MIDI record is armed.
    COLOR_OFF = (0, 0, 0)
    COLOR_NOTE = (30, 30, 30)
    COLOR_OCTAVE = (0, 0, 70)
    COLOR_PLAYED = (0, 90, 0)
    COLOR_RECORD = (90, 0, 0)
    # Piano layout only: black keys get their own color (DrivenByMoss uses
    # "the selected track's color" - we have no such concept here, so a
    # fixed, visually-distinct hue instead).
    COLOR_BLACK_KEY = (40, 0, 40)

    # Piano layout: standard 7-white-key/5-black-key octave. Each black pad
    # is drawn above its UPPER white neighbor (C# above D, D# above E, F#
    # above G, G# above A, A# above B) rather than its lower one - matches
    # how the black keys visually lean on a real keyboard better than
    # aligning them with the lower neighbor would. Black key "before" white
    # column index c exists unless c%7 is C (0) or F (3) - the two places a
    # real keyboard has no black key immediately below.
    PIANO_WHITE_OFFSETS = (0, 2, 4, 5, 7, 9, 11)   # C D E F G A B
    PIANO_NO_BLACK_BEFORE = (0, 3)                 # C, F
    PIANO_BAND_OFFSET = 24  # semitones between the two bands (2 octaves)

    def __init__(self, state_manager, leds: FeedbackLEDs, pads: PadLEDs, notes_queue: mp.Queue,
                 oled_refresh_cb=None):
        super().__init__(state_manager)
        self._leds = leds
        self._pads = pads
        self._notes_queue = notes_queue  # drained by the driver's midiproc_task
        self._oled_refresh_cb = oled_refresh_cb  # top-level driver's _refresh_oled, see _on_scale_changed
        self._knobs_ease = KnobJitterFilter()  # Volume knob only - see cc_change/_adjust_tonic
        self._base_note = self.BASE_NOTE_DEFAULT
        self._octave_step = 0   # -MAX_OCTAVE_STEPS..+MAX_OCTAVE_STEPS, see _shift_octave
        self._chromatic = True  # False = Piano layout, see toggle_layout()

        # Scale/tonic - Chromatic layout only, see the class comment and
        # _apply_scale(). Deliberately NOT reset in set_active() like
        # _chromatic is - this is a performance setting worth keeping
        # exactly as dialed in across leaving/re-entering Play mode, unlike
        # the layout choice.
        self._scale = 0    # 0 = Chromatic (no filter), else 1-based scales.json index
        self._tonic = 0    # 0-11 = C-B
        self._scale_keymap = None     # see _apply_scale()
        self._scale_degree_count = 0  # notes/octave in the current scale, for _shift_octave
        self._scale_label = "Chromatic"  # OLED status line, see oled_status()
        self._keymap_offset = 0       # index into _scale_keymap of row=3,col=0 (lowest pad)

        self._play_chan = _resolve_play_midi_chan()  # refreshed in set_active(True) too
        # Pad note -> (midi note, channel) actually sounding for it - both
        # fixed at press time and reused as-is on release/panic, rather than
        # recomputed, so a note always gets its note-off on the exact
        # channel it was started on even if the active chain changes (or the
        # octave shifts) while it's held - otherwise the note-off could go
        # to the wrong channel and leave the original note stuck forever.
        # (In practice channel is always None or self._play_chan - kept as a
        # per-note field anyway rather than assumed constant, in case a
        # future change makes it vary again.)
        self._held = {}

    def set_active(self, active):
        super().set_active(active)
        if active:
            self._chromatic = True  # every fresh activation starts in Play (chromatic)
            self._play_chan = _resolve_play_midi_chan()
        else:
            # Notes are sustained (duration=0) until explicitly stopped -
            # leaving this mode with pads still held would otherwise leave
            # them stuck on forever.
            self._all_notes_off()

    def toggle_layout(self):
        """BTN_NOTE pressed again while Play mode is already active (see
        midi_event's BTN_NOTE handling) - switches between Chromatic and
        Piano. Flushes held notes first, same reasoning as _shift_octave:
        the pad->note mapping is about to change completely."""
        self._all_notes_off()
        self._chromatic = not self._chromatic
        self.refresh()
        # oled_status() reads self._chromatic (Piano has no scale line) -
        # keep the OLED in sync with the layout switch.
        if self._oled_refresh_cb:
            self._oled_refresh_cb()

    def _midi_note(self, row, col):
        if not self._chromatic:
            return self._piano_note(row, col)
        idx = (3 - row) * 16 + col
        if self._scale_keymap is not None:
            pos = self._keymap_offset + idx
            return self._scale_keymap[pos] if 0 <= pos < len(self._scale_keymap) else None
        return self._base_note + idx

    def _piano_note(self, row, col, base_note=None):
        """Piano layout: rows 0-1 are the higher band (0=black key row,
        1=white key row), rows 2-3 the lower band (self.PIANO_BAND_OFFSET
        semitones down) laid out the same way - see the class comment and
        PIANO_WHITE_OFFSETS/PIANO_NO_BLACK_BEFORE. Returns None for a
        "no black key here" gap (e.g. between B and C), same as an
        out-of-MIDI-range note - both mean "this pad plays nothing".
        base_note overrides self._base_note - only used by
        _piano_offset_range(), to reuse this exact mapping (rather than a
        second, hand-derived copy of it) for _shift_base_note's bounds
        check too."""
        base_note = self._base_note if base_note is None else base_note
        band_base = base_note if row < 2 else base_note - self.PIANO_BAND_OFFSET
        is_black_row = row in (0, 2)
        octave, deg = divmod(col, 7)
        white_note = band_base + octave * 12 + self.PIANO_WHITE_OFFSETS[deg]
        if not is_black_row:
            return white_note
        if deg in self.PIANO_NO_BLACK_BEFORE:
            return None
        return white_note - 1

    def _active_channel(self):
        """self._play_chan if there's a chain that can actually take notes
        right now, else None - which chain doesn't matter here: ACTI mode
        (see class comment) routes any note on this channel to whichever
        chain is active, so this is just a sanity check, not a resolution."""
        chain = self._chain_manager.get_active_chain()
        if chain is None or not chain.is_midi():
            return None
        return self._play_chan

    def refresh(self):
        self._held = {}
        for row in range(4):
            for col in range(16):
                self._paint_pad(row, col)
        self._update_leds()

    def _paint_pad(self, row, col, pressed=False):
        note_pad = _pad(row, col)
        note_val = self._midi_note(row, col)
        if note_val is None or not (0 <= note_val <= 127):
            self._pads.pad_off(note_pad)
            return
        # Root-note marker: relative to the actual tonic when a scale is
        # active in Chromatic layout, else plain C (0) - Piano stays
        # C-relative even with a scale selected, since it deliberately
        # ignores scale/tonic entirely (see the class comment).
        root = self._tonic if (self._chromatic and self._scale_keymap is not None) else 0
        if pressed:
            color = self.COLOR_RECORD if self._zynseq.libseq.isMidiRecord() else self.COLOR_PLAYED
        elif note_val % 12 == root:
            color = self.COLOR_OCTAVE
        elif not self._chromatic and row in (0, 2):
            color = self.COLOR_BLACK_KEY
        else:
            color = self.COLOR_NOTE
        self._pads.set_pad(note_pad, *color)

    def oled_status(self):
        """Secondary OLED line while Play mode is active (see the top-level
        driver's _refresh_oled) - current scale/tonic, e.g. 'C Major', or
        'Chromatic'. None in Piano layout: it has no scale line to show
        since it ignores scale/tonic entirely (see the class comment)."""
        return self._scale_label if self._chromatic else None

    def _update_leds(self):
        chain = self._chain_manager.get_active_chain()
        self._leds.led_on(LED_SOLO_2, LED_GREEN_HIGH) if chain is not None and self._zynmixer.get_mute(chain.mixer_chan) \
            else self._leds.led_off(LED_SOLO_2)
        self._leds.led_on(LED_SOLO_3, LED_GREEN_HIGH) if chain is not None and self._zynmixer.get_solo(chain.mixer_chan) \
            else self._leds.led_off(LED_SOLO_3)
        self._leds.led_off(LED_SOLO_1)
        self._leds.led_off(LED_SOLO_4)  # unbound for now

    def _send_note(self, note_val, velocity, channel):
        """Queue a Note On (velocity 0 = off, standard MIDI convention - no
        separate note-off call needed) for the driver's midiproc_task to
        actually emit on the real-time JACK port it owns - see the class
        comment for why this, and not any direct Python/ctypes call, is
        what it takes to reach an engine. channel is only accepted for
        symmetry with how _held tracks its per-note channel field - always
        self._play_chan in practice (see _active_channel())."""
        self._notes_queue.put((0x90 | channel, note_val, velocity))

    def _all_notes_off(self):
        for note_pad, (note_val, channel) in self._held.items():
            if channel is not None:
                self._send_note(note_val, 0, channel)
            idx = note_pad - PAD_NOTE_BASE
            self._paint_pad(idx // 16, idx % 16)
        self._held = {}

    def _shift_octave(self, delta):
        """Select knob, both layouts. Piano always shifts self._base_note
        (_piano_note has no notion of _scale_keymap/_keymap_offset at all -
        it's deliberately scale-blind, see the class comment) - only
        Chromatic, and only when a scale is actually active there, shifts
        the scale keymap offset instead. Checking self._chromatic here
        (not just whether a scale happens to be selected) matters: without
        it, picking a scale then switching to Piano would silently start
        moving _keymap_offset - a field Piano never reads - leaving its own
        _base_note-driven octave untouched and the knob looking dead."""
        if self._chromatic and self._scale_keymap is not None:
            self._shift_keymap_offset(delta)
        else:
            self._shift_base_note(delta)

    def _shift_base_note(self, delta):
        new_step = self._octave_step + (1 if delta > 0 else -1)
        if not (-self.MAX_OCTAVE_STEPS <= new_step <= self.MAX_OCTAVE_STEPS):
            return
        new_base = self.BASE_NOTE_DEFAULT + new_step * 12
        # How far new_base can actually reach before some pad falls outside
        # 0-127 depends on the layout - Chromatic's is a flat run
        # (base_note .. base_note+TOTAL_NOTES-1), but Piano's is a
        # completely different, narrower and asymmetric shape (two bands,
        # see _piano_offset_range) - reusing Chromatic's own (0,
        # TOTAL_NOTES-1) span for Piano too (as this used to) made it hit
        # this "shouldn't normally trigger" safety net far sooner than
        # Chromatic, capping Piano's actual reachable range well short of
        # Chromatic's despite sharing the same MAX_OCTAVE_STEPS.
        lo_off, hi_off = (0, self.TOTAL_NOTES - 1) if self._chromatic else self._piano_offset_range()
        if not (0 <= new_base + lo_off and new_base + hi_off <= 127):
            return  # extra safety net, shouldn't trigger given MAX_OCTAVE_STEPS
        self._all_notes_off()  # avoid leaving notes stuck on under the old mapping
        self._octave_step = new_step
        self._base_note = new_base
        self.refresh()

    def _piano_offset_range(self):
        """(min, max) note offset _piano_note can ever produce relative to
        self._base_note (queried via base_note=0, rather than hand-derived,
        so this can't drift out of sync with _piano_note's own mapping) -
        see _shift_base_note's bounds check, the only caller."""
        offsets = [self._piano_note(row, col, base_note=0)
                   for row in range(4) for col in range(16)]
        offsets = [o for o in offsets if o is not None]
        return min(offsets), max(offsets)

    def _shift_keymap_offset(self, delta):
        """Same idea as _shift_base_note, but for a scale-filtered
        Chromatic layout: shifts by one octave's worth of scale degrees
        (self._scale_degree_count) instead of a flat 12 semitones, so the
        grid keeps landing on the same scale degree in each column after
        the shift. Bounded directly by the keymap's own length rather than
        a separate MAX_OCTAVE_STEPS-style cap - it's already finite (built
        from 11 octaves clamped to 0-127, see _apply_scale). Only requires
        the bottom-left pad to stay in range, not the whole 64-pad window -
        same reasoning as _apply_scale's own anchor: a sparse scale (e.g.
        pentatonic) can run out of room well before 64 pads' worth in
        either direction, and a partially-empty window at that edge is a
        perfectly fine state, not something to refuse reaching."""
        step = self._scale_degree_count if delta > 0 else -self._scale_degree_count
        new_offset = self._keymap_offset + step
        if not (0 <= new_offset < len(self._scale_keymap)):
            return
        self._all_notes_off()
        self._keymap_offset = new_offset
        self.refresh()

    def _load_scales_json(self):
        try:
            with open(ZYNSEQ_CONFIG_ROOT + "/scales.json") as f:
                return json.load(f)
        except Exception as ex:
            logging.warning(f"PlayHandler: can't load scales.json => {ex}")
            return []

    def _cycle_scale(self, delta):
        """BTN_PAT_UP/DOWN - cycles self._scale through 0 (Chromatic) ..
        len(scales.json) inclusive, wrapping either way."""
        data = self._load_scales_json()
        self._scale = (self._scale + (1 if delta > 0 else -1)) % (len(data) + 1)
        self._apply_scale(data)
        self._on_scale_changed()

    def _adjust_tonic(self, delta):
        """Volume knob (repurposed here, see the class comment) - only
        rebuilds/repaints when a scale is actually active; harmless to keep
        tracking self._tonic even in Chromatic-no-scale/Piano so it's
        already right if/when a scale gets picked later."""
        self._tonic = (self._tonic + (1 if delta > 0 else -1)) % 12
        if self._scale > 0:
            self._apply_scale()
            self._on_scale_changed()

    def _apply_scale(self, data=None):
        """Rebuild self._scale_keymap/_scale_degree_count/_scale_label from
        self._scale/self._tonic, and re-anchor self._keymap_offset near
        BASE_NOTE_DEFAULT (same idea as StepSeqHandler._load_keymap's own
        row-window centering) - called whenever either changes. data lets
        _cycle_scale pass along the scales.json parse it already had to do
        anyway, to avoid reading the file twice."""
        self._scale_keymap = None
        self._scale_degree_count = 0
        self._scale_label = "Chromatic"
        if self._scale <= 0:
            return
        if data is None:
            data = self._load_scales_json()
        if not (1 <= self._scale <= len(data)):
            self._scale = 0
            return
        entry = data[self._scale - 1]
        offsets = entry["scale"]
        keymap = [note for octave in range(11) for note in
                  (self._tonic + off + octave * 12 for off in offsets)
                  if 0 <= note <= 127]
        if not keymap:
            self._scale = 0
            return
        self._scale_keymap = keymap
        self._scale_degree_count = len(offsets)
        self._scale_label = f"{NOTE_NAMES[self._tonic]} {entry['name']}"
        # No upper clamp against len(keymap)-TOTAL_NOTES here (unlike
        # _shift_keymap_offset's bounds check) - a sparse scale (e.g.
        # pentatonic) times 64 pads can span most of the MIDI range, so
        # requiring a full 64 in-scale notes above the anchor would often
        # force it down several octaves below BASE_NOTE_DEFAULT just to
        # avoid empty pads at the top of the grid. Landing near
        # BASE_NOTE_DEFAULT with some empty (silent, unlit - see
        # _midi_note's own out-of-range handling) pads past the scale's
        # actual top is the better default; _shift_keymap_offset separately
        # stops you from scrolling the window into total emptiness.
        anchor = next((i for i, n in enumerate(keymap) if n >= self.BASE_NOTE_DEFAULT), len(keymap) - 1)
        self._keymap_offset = max(0, anchor)

    def _on_scale_changed(self):
        self._all_notes_off()  # pad->note mapping just changed
        self.refresh()
        if self._oled_refresh_cb:
            self._oled_refresh_cb()

    def _toggle_mute(self):
        chain = self._chain_manager.get_active_chain()
        if chain is None:
            return
        self._zynmixer.set_mute(chain.mixer_chan, self._zynmixer.get_mute(chain.mixer_chan) ^ 1, True)
        self._update_leds()

    def _toggle_solo(self):
        chain = self._chain_manager.get_active_chain()
        if chain is None:
            return
        self._zynmixer.set_solo(chain.mixer_chan, self._zynmixer.get_solo(chain.mixer_chan) ^ 1, True)
        self._update_leds()

    def _switch_chain(self, nudge):
        # Same chain-scroll MixerHandler's own Select knob already uses -
        # switches which chain you're playing into without touching the
        # actual screen/mode. Already-held notes remember their own
        # channel (see class comment), so switching chain mid-chord doesn't
        # affect notes already sounding, only new presses from here on.
        self._chain_manager.next_chain(nudge)
        self._update_leds()

    def note_on(self, note, velocity, shifted_override=None):
        if note == KNOB_VOLUME:
            # Repurposed for tonic (see cc_change) - reset its own jitter
            # filter on touch. Pan/Filter/Resonance have no use here - decline
            # (return False below) and let the top-level driver's shared
            # default reset theirs instead.
            self._knobs_ease.reset(note)
            return True
        if note == BTN_SOLO_1:
            # No specific "stop" target here (unlike StepSeq's Solo1) - a
            # panic button for this handler's own sustained notes instead.
            self._all_notes_off()
            return True
        if note == BTN_SOLO_2:
            self._toggle_mute()
            return True
        if note == BTN_SOLO_3:
            self._toggle_solo()
            return True
        if note == BTN_GRID_LEFT:
            self._switch_chain(-1)
            return True
        if note == BTN_GRID_RIGHT:
            self._switch_chain(1)
            return True
        if note == BTN_PAT_UP:
            self._cycle_scale(1)
            return True
        if note == BTN_PAT_DOWN:
            self._cycle_scale(-1)
            return True

        if PAD_NOTE_BASE <= note < PAD_NOTE_BASE + 64:
            if note in self._held:
                # Already sounding - e.g. a repeated Note On for a pad
                # that's still physically held (seen on real hardware).
                # Ignore rather than re-triggering the note.
                return True
            idx = note - PAD_NOTE_BASE
            row, col = idx // 16, idx % 16
            note_val = self._midi_note(row, col)
            if note_val is None or not (0 <= note_val <= 127):
                return True
            channel = self._active_channel()
            self._held[note] = (note_val, channel)
            if channel is not None:
                self._send_note(note_val, velocity, channel)
            self._paint_pad(row, col, pressed=True)
            return True
        return False

    def note_off(self, note, shifted_override=None):
        if not (PAD_NOTE_BASE <= note < PAD_NOTE_BASE + 64):
            return
        info = self._held.pop(note, None)
        if info is None:
            return
        note_val, channel = info
        if channel is not None:
            self._send_note(note_val, 0, channel)
        idx = note - PAD_NOTE_BASE
        self._paint_pad(idx // 16, idx % 16)

    def cc_change(self, ccnum, ccval):
        if ccnum == KNOB_SELECT:
            delta = ccval if ccval < 64 else ccval - 128
            self._shift_octave(delta)
            return True
        if ccnum == KNOB_VOLUME:
            delta = self._knobs_ease.feed(ccnum, ccval)
            if delta is not None:
                self._adjust_tonic(delta)
            return True
        # Pan/Filter/Resonance have no use here - decline, top-level driver's
        # shared default (ZynpotRotate) picks them up instead.
        return None


# --------------------------------------------------------------------------
# Main driver
# --------------------------------------------------------------------------
class zynthian_ctrldev_akai_fire(zynthian_ctrldev_zynmixer, zynthian_ctrldev_zynpad):

    # Confirmed on real hardware (see zynthian_ctrldev_akai_fire_protocol.md). Kept the
    # "MIDI 1"/"IN 1" suffixed variants too, in case the exact JACK alias zynautoconnect
    # reports differs from what `aconnect -l` shows (see other drivers' dev_ids for why).
    dev_ids = ["FL STUDIO FIRE", "FL STUDIO FIRE MIDI 1", "FL STUDIO FIRE IN 1"]
    driver_name = 'AKAI Fire'
    driver_description = 'Mixer + Zynpad + StepSeq + Play modes, OLED shows current mode'

    # Block only Fire's own raw channel (buttons/pads, always channel 0) from
    # reaching chains as notes - every other channel, specifically
    # PLAY_MIDI_CHAN (used by midiproc_task's own injected notes, see
    # PlayHandler's class comment), passes through normally. A bitmask, not
    # a bool - see zynthian_ctrldev_base.unroute_from_chains's own docstring.
    # (Confirmed via the diagnostic tried while chasing the real bug - an
    # orphaned duplicate midiproc process from a since-fixed early-boot
    # crash, see _alt_mode()'s comment - that this bitmask was never the
    # problem in the first place.)
    unroute_from_chains = 0b0000000000000001

    def __init__(self, state_manager, idev_in, idev_out=None):
        self._leds = FeedbackLEDs(idev_out)
        self._pads = PadLEDs(idev_out)
        self._oled = OledDisplay(idev_out)
        self._oled_timer = IntervalTimer()  # anti-sleep keepalive, see init()/_oled_tick()
        # IPC to midiproc_task (runs in a spawned subprocess, see init_midiproc()
        # in the base class) - PlayHandler puts (status, note, vel) tuples here,
        # midiproc_task drains and emits them on its own real-time JACK port.
        self._play_notes_queue = mp.Queue()
        self._mixer_handler = MixerHandler(state_manager, self._leds, self._pads)
        self._zynpad_handler = ZynpadHandler(state_manager, self._pads)
        self._stepseq_handler = StepSeqHandler(state_manager, self._leds, self._pads, self._refresh_oled)
        self._play_handler = PlayHandler(state_manager, self._leds, self._pads, self._play_notes_queue,
                                          self._refresh_oled)
        # Mixer is the default/fallback mode: whatever screen has no
        # dedicated mode of its own (Admin, Preset, Control, Snapshot, main
        # menu, etc., now that DeviceHandler is gone) just keeps showing
        # whichever of the 4 real modes was already active rather than
        # switching to anything - see _update_current_handler()'s "no match"
        # case. Mixer is only actually picked here as the very first value,
        # before any real screen has been seen yet.
        self._current_handler = self._mixer_handler
        # Knobs/Select/Select-push are a SEPARATE concern from the pad grid
        # above - _current_handler deliberately goes stale on a screen with
        # no dedicated mode (see above), but a stale mode's own knob logic
        # (e.g. Mixer's chain volume) must NOT keep running invisibly while
        # an unrelated screen (Preset, Admin, Snapshot, ...) is actually
        # showing - that looks exactly like "the knobs stopped doing
        # anything" (found on real hardware). self._null_handler - a bare
        # ModeHandlerBase instance that declines everything (every method is
        # a no-op/returns None) - stands in for "no mode-specific knob
        # behavior right now", forcing _default_cc_change/_default_note_on
        # to run instead. Kept in sync with _current_handler by
        # _set_current_handler() whenever pads actually switch to a real
        # mode; _update_current_handler()'s "no match" case is the only
        # place they deliberately diverge (pads stay stale, knobs go null).
        self._null_handler = ModeHandlerBase(state_manager)
        self._knobs_handler = self._mixer_handler
        self._default_zynpot = ZynpotRotate(state_manager)
        # (bank, seq) pairs currently playing, anywhere - see
        # _pattern_is_playing()/update_seq_state().
        self._playing_sequences = set()
        # OLED mode-name label per handler - see _refresh_oled().
        self._mode_labels = {
            self._mixer_handler: "Mixer",
            self._zynpad_handler: "Zynpad",
            self._stepseq_handler: "StepSeq",
            self._play_handler: "Play",
        }

        # Shift itself has no local state to track any more - it's just a
        # second way to flip zynthian's own persistent alt_mode, see
        # _alt_mode()'s docstring - so only Alt (still a momentary hold)
        # needs tracking here.
        self._is_alt = False
        self._btn_timer = ButtonTimer(self._handle_timed_button)
        # Notes currently being timed as an Alt+touch zynpot push (see
        # midi_event's ZYNPOT_KNOBS handling) - tracked by note rather than
        # re-checking self._is_alt on release, since Alt may already have
        # been released while the knob is still held.
        self._zynpot_touch_active = set()

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
            # Transport LEDs (Play/Record - see _update_mode_leds) need to
            # react live to playback/recording starting or stopping, not
            # just to mode/screen changes like everything else here - same
            # signals zynthian_ctrldev_akai_apc_key25_mk2.py's own DeviceHandler
            # uses for the same purpose. state_manager re-exposes the audio
            # ones under its own name (same subsignal ids), so no extra
            # imports (zynthian_engine_audioplayer/zynthian_audio_recorder)
            # are needed. Actual state is read live off state_manager at
            # refresh time (same "don't cache, just re-check" approach as
            # _alt_mode()) rather than tracked from the signal payload.
            (zynsigman.S_AUDIO_PLAYER, state_manager.SS_AUDIO_PLAYER_STATE, self._on_transport_state_changed),
            (zynsigman.S_AUDIO_RECORDER, state_manager.SS_AUDIO_RECORDER_STATE, self._on_transport_state_changed),
            (zynsigman.S_STATE_MAN, state_manager.SS_MIDI_PLAYER_STATE, self._on_transport_state_changed),
            (zynsigman.S_STATE_MAN, state_manager.SS_MIDI_RECORDER_STATE, self._on_transport_state_changed),
        ]

        # NOTE: init() (called by the manager right after this ctor) will call
        # refresh(), which needs _current_handler ready - so this goes last.
        super().__init__(state_manager, idev_in, idev_out)

    def init(self):
        # The pad grid is the device's own SysEx-set memory, independent of
        # this driver's state - it can carry stale colors from a previous
        # session/mode across a reload. _set_current_handler()'s clear only
        # fires on an actual handler *change*, which never happens right at
        # startup (_current_handler is already _mixer_handler from
        # construction), so clear unconditionally here before anything below
        # (via super().init() -> refresh()) paints the real starting state.
        self._pads.all_off()

        # super().init()/end() cooperatively chain through BOTH mixins here
        # (zynmixer -> zynpad -> base), verified via MRO - no need for the
        # explicit extra zynthian_ctrldev_zynpad.init(self)/end(self) call
        # some other multi-mixin drivers in this codebase carry. This is
        # also what spawns midiproc_task (zynthian_ctrldev_base.init() ->
        # init_midiproc()).
        super().init()
        for signal, subsignal, callback in self._signals:
            zynsigman.register(signal, subsignal, callback)

        # Alt's own momentary lighting (see midi_event's BTN_ALT handling)
        # isn't part of _update_mode_leds() - already ran once via
        # super().init() -> refresh() above - so it needs its own initial
        # dull-default here, matching the rest of its button cluster.
        self._leds.led_on(LED_ALT, LED_Y_DULL)

        # ACTI mode ("active chain input"): any note arriving on this
        # device's zmip routes to whichever chain is currently active,
        # regardless of the note's own channel - see PlayHandler's class
        # comment for why this is how PLAY_MIDI_CHAN gets resolved to an
        # actual target, rather than us matching a channel to a chain
        # ourselves.
        lib_zyncore.zmip_set_flag_active_chain(self.idev, 1)

        # zmop_set_route_from(zmop, zmip, enable) is a SEPARATE per-(chain,
        # device) permission matrix - unroute_from_chains/ACTI above only
        # decide what happens to traffic that's already allowed to reach a
        # chain at all; this is the gate that allows it there in the first
        # place. New chains enable every device here by default
        # (zynthian_chain_manager.add_chain()), but zynthian_state_manager
        # also restores a *persisted per-device* "routed_chains" list at
        # config load that overwrites that - and since this device has
        # presumably been known to zynthian as a pure controller (no note
        # output) this whole time, that saved list is quite plausibly empty
        # for it. Force it open ourselves rather than depend on that state.
        for zmop in range(16):
            lib_zyncore.zmop_set_route_from(zmop, self.idev, 1)

        # First OLED paint, plus a periodic keepalive so it doesn't blank
        # itself after 3s of silence (see OledDisplay.ANTI_SLEEP_MS) - the
        # timer just calls update(), which only re-sends what's actually due,
        # so a 1s tick is plenty responsive without being wasteful.
        self._refresh_oled()
        self._oled_timer.add("oled_keepalive", 1000, self._oled_tick)

    def end(self):
        # Safety net: guarantee any still-sounding Play-mode notes get their
        # note-off, and StepSeq's playhead poll timer stops, regardless of
        # which handler is actually current when the driver is torn down
        # (see PlayHandler.set_active / StepSeqHandler.set_active).
        self._play_handler.set_active(False)
        self._stepseq_handler.set_active(False)
        self._oled_timer.remove("oled_keepalive")
        lib_zyncore.zmip_set_flag_active_chain(self.idev, 0)
        for zmop in range(16):
            lib_zyncore.zmop_set_route_from(zmop, self.idev, 0)
        for signal, subsignal, callback in self._signals:
            zynsigman.unregister(signal, subsignal, callback)
        super().end()

    # ------------------------------------------------------------------
    # Real-time MIDI processor (spawned in its own process by the base
    # class's init_midiproc(), see zynthian_ctrldev_base.py) - passes
    # Fire's own raw input through unchanged (so this driver's normal
    # button/pad/knob handling, via dev{N}_in -> zynmidi_read(), sees
    # everything exactly as it would without a midiproc at all) and
    # additionally emits whatever PlayHandler queues onto self._play_notes_queue.
    # See PlayHandler's class comment for why this exists - modeled directly
    # on zynthian_ctrldev_base_moder.py's own midiproc_task (a shipped,
    # working reference for real-time note generation from a ctrldev driver).
    # ------------------------------------------------------------------
    def midiproc_task(self, jackname):
        zynthian_ctrldev_base.midiproc_task_reset_signal_handlers()

        import jack
        from threading import Event

        client = jack.Client(jackname)
        inport = client.midi_inports.register('in_1')
        outport = client.midi_outports.register('out_1')
        event = Event()

        @client.set_process_callback
        def process(frames):
            outport.clear_buffer()
            last_offset = 0
            # Fire's own hardware only ever sends on channel 0 - anything
            # arriving here on the reserved Play-mode channel can only be
            # some kind of loopback of our own injected notes, not real
            # device input (the actual loopback path turned out to be
            # zynthian_state_manager.zynmidi_read()'s software tap on this
            # zmip, guarded against directly in the top-level driver's
            # midi_event() - this is just cheap extra defense in case
            # anything else ever reaches this port on that channel).
            play_chan = _resolve_play_midi_chan()
            for offset, indata in inport.incoming_midi_events():
                # indata isn't plain bytes (some jack-client buffer/cffi
                # type where [0] doesn't give a plain int) - normalize
                # first, same as the reference examples do via
                # struct.unpack.
                raw = bytes(indata)
                if raw and (raw[0] & 0x0F) == play_chan and (raw[0] & 0xF0) in (0x80, 0x90):
                    continue
                outport.write_midi_event(offset, indata)  # pass through, unchanged
                last_offset = offset

            # Drain any pending Play-mode notes - non-blocking, this runs on
            # every JACK cycle so nothing is ever left waiting long even if
            # drained one cycle late. Written at last_offset (not a fixed 0):
            # JACK MIDI output requires events within one process() call to
            # be written in non-decreasing offset order, and the pass-through
            # loop above may already have used offsets > 0 this cycle -
            # writing at a fixed 0 after that violates the ordering and
            # raises. Sample-accurate timing doesn't matter for hand-played
            # notes anyway.
            while True:
                try:
                    event_bytes = self._play_notes_queue.get_nowait()
                except queue.Empty:
                    break
                try:
                    outport.write_midi_event(last_offset, event_bytes)
                except Exception:
                    pass

        @client.set_shutdown_callback
        def shutdown(status, reason):
            event.set()

        with client:
            event.wait()

    def refresh(self):
        self._current_handler.refresh()
        self._update_mode_leds()

    def _update_mode_leds(self):
        # This whole bottom-left cluster (Step/Note/Drum/Perform/Shift, all
        # yellow-red) idles at dull yellow rather than fully off - reads as
        # "alive"/available, brightening or changing color for whatever's
        # actually active - same idea BTN_ALT's own momentary lighting
        # follows (see its note_on/note_off), just state-driven here instead
        # of held-driven. Bank/Mode and Browser aren't part of this cluster
        # (different color families / already-established behavior) and are
        # untouched.

        # Shift mirrors zynthian's own persistent alt_mode toggle - see
        # _alt_mode()'s docstring.
        self._leds.led_on(LED_SHIFT, LED_YR_HIGH_RED if _alt_mode() else LED_YR_DULL_YELLOW)

        # Bank/Mode is one of the alternate ways to flip the very same
        # alt_mode toggle (see _alt_mode()'s docstring) - mirrors it too,
        # same as Shift, though its own LED color family is unconfirmed (see
        # protocol doc) so this stays a plain on/off rather than assuming it
        # also supports yellow-red's dull/high split.
        self._leds.led_on(LED_BANK, LED_ON) if _alt_mode() else self._leds.led_off(LED_BANK)

        # Browser (red-only): repurposed as a screen-link indicator - lit
        # whenever unlinked, as a reminder the Fire's mode won't follow
        # screen changes until Alt+Browser re-links it.
        self._leds.led_on(LED_BROWSER, LED_RED_HIGH) if not self._screen_linked \
            else self._leds.led_off(LED_BROWSER)

        # Perform: red in Mixer mode, yellow in Zynpad mode, dull yellow
        # otherwise (Perform only toggles between the two "performance"
        # screens - see BTN_PERFORM - nothing else is part of that toggle).
        if self._current_handler is self._mixer_handler:
            self._leds.led_on(LED_PERFORM, LED_YR_HIGH_RED)
        elif self._current_handler is self._zynpad_handler:
            self._leds.led_on(LED_PERFORM, LED_YR_HIGH_YELLOW)
        else:
            self._leds.led_on(LED_PERFORM, LED_YR_DULL_YELLOW)

        # Step: yellow while StepSeq is the active mode, dull yellow
        # otherwise - same "which mode is active" convention as Perform.
        self._leds.led_on(LED_STEP, LED_YR_HIGH_YELLOW if self._current_handler is self._stepseq_handler
                           else LED_YR_DULL_YELLOW)

        # Note: yellow while Play is the active mode; red while sitting on
        # the Snapshot screen (Alt+Note's own target - see BTN_NOTE) as a
        # "you're here" hint, same idea as Perform's own red/yellow split
        # for Mixer/Zynpad; dull yellow otherwise. Play-active takes
        # priority in the (unlikely, since Play doesn't touch the
        # touchscreen itself) case both happen to be true at once.
        if self._current_handler is self._play_handler:
            self._leds.led_on(LED_NOTE, LED_YR_HIGH_YELLOW)
        elif self._last_screen == "snapshot":
            self._leds.led_on(LED_NOTE, LED_YR_HIGH_RED)
        else:
            self._leds.led_on(LED_NOTE, LED_YR_DULL_YELLOW)

        # Drum: reserved for a future mode - dull yellow, nothing to react
        # to yet (same "alive but idle" cluster default as everything else
        # here).
        self._leds.led_on(LED_DRUM, LED_YR_DULL_YELLOW)

        # Transport (Play/Stop/Record) - loosely based on zynthian_wsleds_v5.py
        # (Record lights while audio_recorder.rec_proc, Play while
        # status_audio_player) and zynthian_ctrldev_akai_apc_key25_mk2.py's
        # DeviceHandler (same idea, signal-driven), but both of those only
        # ever show one generic "lit" state - extended here to distinguish
        # audio vs MIDI by color, since TOGGLE_RECORD/TOGGLE_PLAY/STOP
        # already target one or the other depending on _alt_mode() (see
        # zynthian_gui.cuia_toggle_record/_play/_stop). Idle itself reflects
        # which one is *currently targeted* rather than a flat neutral color,
        # so toggling alt_mode (Shift/Bank/Alt+touch) visibly changes these
        # two even with nothing actually playing/recording yet - confirmed
        # missing on real hardware (pressing Shift produced no visible
        # change, since idle color didn't depend on alt_mode at all).
        is_midi = _alt_mode()
        # state_manager.status_audio_player is a cached copy, updated by its
        # own cb_status_audio_player() - a SEPARATE subscriber to the same
        # SS_AUDIO_PLAYER_STATE signal we are, racing us for who reads/
        # writes first depending on registration order. Confirmed backwards
        # on real hardware (lit only once playback had already stopped, not
        # while actually playing) - read the live engine state directly
        # instead, sidestepping the race entirely. status_midi_player/
        # status_midi_recorder/audio_recorder.rec_proc don't have this
        # problem (each is written directly, in the same method, before its
        # own signal is sent - no competing cached copy involved).
        audio_player = self.state_manager.audio_player
        is_audio_playing = audio_player is not None and zynaudioplayer.get_playback_state(audio_player.handle)

        # Record only has red+yellow in its own family (no green - it's
        # yellow-red, not yellow-green) - audio uses red (also just the
        # conventional "recording" color on real hardware), MIDI uses
        # yellow, both dimmed to "dull" for idle/targeted-but-not-recording.
        if self.state_manager.audio_recorder.rec_proc:
            self._leds.led_on(LED_RECORD, LED_YR_HIGH_RED)
        elif self.state_manager.status_midi_recorder:
            self._leds.led_on(LED_RECORD, LED_YR_HIGH_YELLOW)
        else:
            self._leds.led_on(LED_RECORD, LED_YR_DULL_YELLOW if is_midi else LED_YR_DULL_RED)

        # Play's own family (yellow-green) has a real green for audio.
        # "MIDI playing" also covers the pattern currently open in
        # StepSeq/the pattern editor - BTN_PLAY/BTN_STOP there drive
        # zynseq's own per-sequence transport directly
        # (zynthian_gui_patterneditor.toggle_playback() ->
        # libseq.setPlayState()), which never touches status_midi_player or
        # emits either transport signal at all - found on real hardware
        # (pattern playback worked, but this LED didn't react to it) -
        # _pattern_is_playing() below covers that gap.
        if is_audio_playing:
            self._leds.led_on(LED_PLAY, LED_YG_HIGH_GREEN)
        elif self.state_manager.status_midi_player or self._pattern_is_playing():
            self._leds.led_on(LED_PLAY, LED_YG_HIGH_YELLOW)
        else:
            self._leds.led_on(LED_PLAY, LED_YG_DULL_YELLOW if is_midi else LED_YG_DULL_GREEN)

        # Stop: DrivenByMoss's own FireColorManager only ever sends 0/1/2 to
        # this button (yellow-only family, confirmed) - values 3/4
        # (LED_YR_*_RED, i.e. reusing yellow-red's own red tier) are
        # unverified for it specifically, requested/tried anyway since
        # DrivenByMoss simply never exercising a value doesn't prove the
        # hardware can't do it. Real hardware (V5/APC) never reacts here at
        # all; this shows "is there anything to stop" instead, same idea as
        # the yellow version this replaces.
        anything_playing = is_audio_playing or self.state_manager.status_midi_player or self._pattern_is_playing()
        self._leds.led_on(LED_STOP, LED_YR_HIGH_RED if anything_playing else LED_YR_DULL_RED)

        # Metronome (BTN_PATTERN_SONG) - same yellow-green family as Play,
        # lit while the Tempo screen it opens (see midi_event's
        # BTN_PATTERN_SONG handling) is actually showing.
        self._leds.led_on(LED_METRONOME, LED_YG_HIGH_YELLOW if self._last_screen == "tempo" else LED_YG_DULL_YELLOW)

    def _pattern_is_playing(self):
        """Whether any sequence, in any bank, is currently playing - see the
        Play LED comment above for why this needs its own check, separate
        from state_manager's status_audio_player/status_midi_player.
        self._playing_sequences is kept up to date incrementally by
        update_seq_state() - not scanned here, and deliberately not scoped
        to only zynpad's own currently-selected pad (a pattern started
        elsewhere and left running should still count)."""
        return bool(self._playing_sequences)

    def _on_transport_state_changed(self, **kwargs):
        self._update_mode_leds()

    def light_off(self):
        self._leds.all_off()
        self._pads.all_off()
        self._oled.all_off()

    def _refresh_oled(self):
        label = self._mode_labels.get(self._current_handler, "")
        self._oled.clear()
        # oled_status() is an optional per-handler hook (currently only
        # PlayHandler has one, for its scale/tonic - see its own
        # oled_status()) - a second, smaller status line under the mode
        # name instead of the single big centered label every other mode
        # gets.
        get_status = getattr(self._current_handler, "oled_status", None)
        status = get_status() if get_status is not None else None
        if status:
            self._oled.text(0, 4, label, size=16, center_x=True)
            # text_fit rather than a fixed size - scale names vary a lot in
            # length (e.g. "C Major" vs. "C# Harmonic Minor") and scales.json
            # is a user-editable file, so there's no fixed upper bound on
            # this string worth hardcoding a size for.
            self._oled.text_fit(34, status, max_size=16, min_size=9)
        else:
            # size=24 is the largest that keeps the widest label ("StepSeq")
            # within the 128px width with some margin - see the size-sweep
            # measurements in this driver's own dev notes if this ever needs
            # revisiting for a longer label.
            self._oled.text(0, 0, label, size=24, center_x=True, center_y=True)
        self._oled.update()

    def _oled_tick(self, name):
        self._oled.update()

    def midi_event(self, ev):
        evtype = (ev[0] >> 4) & 0x0F

        # Loopback guard: our own Play-mode notes (see PlayHandler /
        # midiproc_task) reach this exact callback a second time, not just
        # chains - zynthian_state_manager.zynmidi_read() taps ALL zmip
        # traffic in software (not only JACK-level real-time routing) and
        # hands it to ctrldev_manager.midi_event() for OUR OWN device
        # before anything else happens to it. Every note/button check below
        # matches on note number alone, never channel, because Fire's real
        # hardware only ever sends channel 0 - so without this, our own
        # injected notes get misread as pad presses (cascading phantom
        # presses, since Play mode's note range overlaps the pad range) or
        # as specific buttons whose note number they happen to coincide
        # with (e.g. BTN_SOLO_2/3 = 37/38, right at Play mode's own
        # BASE_NOTE_DEFAULT=36, spuriously toggling mute/solo - confirmed
        # on real hardware). Real Fire input is always channel 0, so
        # anything here on the reserved Play channel can only be our own
        # echo - consume and drop it before it reaches anything else.
        if evtype in (EV_NOTE_ON, EV_NOTE_OFF) and (ev[0] & 0x0F) == _resolve_play_midi_chan():
            return True

        if evtype == EV_NOTE_ON:
            note = ev[1] & 0x7F
            vel = ev[2] & 0x7F

            if note == BTN_SHIFT:
                # Sticky (toggle), not momentary - see _alt_mode()'s
                # docstring. self.refresh() updates Shift's own LED
                # (_update_mode_leds) plus whatever the current handler
                # shows for alt_mode (e.g. Mixer's volume-vs-balance view).
                _toggle_alt_mode()
                self._mixer_handler.on_shift_changed(_alt_mode())
                self.refresh()
                return True
            if note == BTN_ALT:
                self._is_alt = True
                # Momentary - lit while held (yellow-only family: dull by
                # default matching the rest of this button cluster, high
                # while pressed), unlike Shift/Bank/Perform/Step/Note above
                # which are state-driven via _update_mode_leds() instead.
                self._leds.led_on(LED_ALT, LED_Y_HIGH)
                # Alt+Browser (screen-link toggle), Alt+Note (Snapshot) and
                # Alt+Perform (ZS3) work from any mode, so _is_alt itself is
                # tracked unconditionally above - but only forward into MixerHandler
                # (its Alt+Solo-N = toggle solo modifier) or StepSeqHandler
                # (its Alt+Filter = play chance modifier) while one of them
                # is actually the active mode.
                if self._current_handler is self._mixer_handler:
                    self._mixer_handler.set_alt(True)
                elif self._current_handler is self._stepseq_handler:
                    self._stepseq_handler.set_alt(True)
                return True
            if note == BTN_STEP:
                if self._current_handler is self._play_handler:
                    # Play mode's unlink (see BTN_NOTE below) is scoped to
                    # staying in Play mode - leaving it via BTN_STEP should
                    # restore screen-linking like every other way out does,
                    # not leave it stuck unlinked. The CUIA below then drives
                    # screen-follow into StepSeq normally.
                    self._screen_linked = True
                # Screen-driven, like Perform: send the CUIA and let
                # screen-follow (_update_current_handler) pick up StepSeq if
                # linked. SCREEN_PATTERN_EDITOR itself resolves "which
                # pattern" from zynpad's currently selected sequence (same as
                # pressing Shift+Pad in Zynpad mode, or the touchscreen).
                self.state_manager.send_cuia("SCREEN_PATTERN_EDITOR")
                return True
            if note == BTN_NOTE:
                if self._is_alt:
                    # Snapshot access - screen-driven like BTN_STEP just
                    # above, so normally no forcing/unlinking: screen-follow
                    # (_update_current_handler) leaves _current_handler as-is,
                    # since "snapshot" has no dedicated mode of its own - the
                    # pad grid just keeps showing whatever it already did.
                    # Exception: if Play is currently active, leave it first -
                    # relinking alone isn't enough, since neither "snapshot"
                    # nor "zs3" (Alt+Perform's own target) are screens
                    # _update_current_handler() can match, so _current_handler
                    # would stay stuck on Play even after relinking - the very
                    # next Perform press would just re-enter its own "leaving
                    # Play" branch forever instead of ever reaching actual
                    # Mixer/Zynpad navigation (found on real hardware).
                    # Mixer is a reasonable, predictable place to land, same
                    # default this driver already picks at construction.
                    if self._current_handler is self._play_handler:
                        self._screen_linked = True
                        self._set_current_handler(self._mixer_handler)
                    self.state_manager.send_cuia("SCREEN_SNAPSHOT")
                    return True
                if self._current_handler is self._play_handler:
                    # Already active - a second press cycles the layout
                    # (Chromatic <-> Piano) instead of doing nothing.
                    self._play_handler.toggle_layout()
                else:
                    # Unlike the other modes, Play has no screen of its own
                    # to be screen-linked to - force it on directly and
                    # unlink so it sticks regardless of the touchscreen.
                    self._screen_linked = False
                    self._set_current_handler(self._play_handler)
                    self._update_mode_leds()
                return True
            if note == BTN_PERFORM:
                if self._is_alt:
                    # ZS3 access - Alt+Note above is Snapshot, its neighbor in
                    # the same "recall a saved state" family (see the
                    # akai_fire_stepseq_plan.md discussion for how this
                    # pairing settled). This used to jump straight to Device
                    # mode instead; screen-driven now, like everything else
                    # here. Same Play-mode exception as Alt+Note above - Alt
                    # here takes priority over the "leaving Play" elif right
                    # below, so it needs its own relink+handoff.
                    if self._current_handler is self._play_handler:
                        self._screen_linked = True
                        self._set_current_handler(self._mixer_handler)
                    self.state_manager.send_cuia("SCREEN_ZS3")
                elif self._current_handler is self._play_handler:
                    # Same reasoning as BTN_STEP above: Play mode's unlink is
                    # scoped to staying in Play mode, and Perform is the
                    # normal "go back" button - restore screen-linking and
                    # resync immediately to whatever's actually on the
                    # touchscreen (unchanged the whole time, since Play mode
                    # never touches the screen itself), landing back on
                    # StepSeq/Zynpad/Mixer, or nowhere in particular, as
                    # appropriate.
                    self._screen_linked = True
                    self._update_current_handler()
                    if self._current_handler is self._play_handler:
                        # _last_screen didn't match any of the 3 real modes
                        # (e.g. Play was entered from Admin, or some other
                        # screen with no dedicated mode of its own) -
                        # _update_current_handler() alone can never move
                        # pad-grid ownership off Play in that case (same
                        # root cause as the Alt+Note/Alt+Perform fix above),
                        # so fall back to Mixer explicitly rather than
                        # leaving Perform stuck re-entering this same branch
                        # forever on every subsequent press.
                        self._set_current_handler(self._mixer_handler)
                elif self._screen_linked:
                    # Screen-driven: change the actual screen; screen-follow
                    # (_update_current_handler) then updates the mode to
                    # match. Toggles between the two "performance" screens:
                    # from mixer or StepSeq (pattern_editor - you got there
                    # from zynpad in the first place, via BTN_STEP or
                    # Shift+Pad, so this reads as "go back"), go to zynpad;
                    # from anywhere else (including screens with no
                    # dedicated mode of their own), go to mixer - so repeated
                    # presses settle into alternating mixer <-> zynpad.
                    if self._last_screen in ("audio_mixer", "pattern_editor"):
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
            if note == BTN_PATTERN_SONG:
                # Labelled "Metronome" in DrivenByMoss's own Fire mapping
                # (see the protocol doc) - opens the tempo/metronome screen.
                # An Alt+press direct on/off toggle was tried and reverted:
                # no CUIA or state_manager method exists for it anywhere in
                # zynthian (zynthian_gui_tempo's own zctrl - gated behind
                # `if self.shown` for the actual libseq write - is the only
                # thing that ever flips it, even the APC key25 mk2 driver's
                # own Metronome button only ever opens this same screen), so
                # doing it properly meant reaching past that gate and
                # hand-syncing the screen's own zctrl/dirty/replot state by
                # hand - fragile, chasing real bugs each time on hardware.
                # Not worth it for a toggle the screen itself is one press
                # away regardless.
                self.state_manager.send_cuia("TEMPO")
                return True
            if note in (BTN_PLAY, BTN_STOP):
                self._btn_timer.is_pressed(note, time.time())
                return True
            if note == BTN_SELECT_PRESS and self._is_alt:
                # BACK - Fire has no dedicated Back button (DeviceHandler's
                # was a borrowed pad, gone now) - Alt+Select-push stands in
                # for it instead. Global/unconditional (checked before
                # handler dispatch below) so it works the same everywhere,
                # even in StepSeq, which otherwise repurposes a *plain*
                # Select-push for clearing its own selection.
                self.state_manager.send_cuia("BACK")
                return True

            # Alt+touch on one of the 4 knobs (Volume/Pan/Filter/Resonance) =
            # a deliberate zynpot "switch" push (short/bold/long), reusing
            # whatever the current screen's own 4 controllers are. Gated on
            # Alt because touch is capacitive and fires on any contact (e.g.
            # just resting a finger while turning the knob) - requiring Alt
            # held first makes it a deliberate two-hand gesture rather than
            # incidental. Without Alt, touch falls through to the current
            # handler's own use of it (usually just clearing the knob-easing
            # accumulator - see e.g. ZynpotRotate.reset()). Tracked by note
            # (not re-checking self._is_alt) so release still resolves
            # correctly even if Alt was let go before the knob was.
            if note in ZYNPOT_KNOBS and self._is_alt:
                self._zynpot_touch_active.add(note)
                self._btn_timer.is_pressed(note, time.time())
                return True

            # Knob touch/Select-push go through _knobs_handler, not
            # _current_handler - they can disagree (see _knobs_handler's
            # comment in __init__) whenever the pad grid is showing a stale
            # mode. Everything else (pads) always follows _current_handler.
            if note in ZYNPOT_KNOBS or note == BTN_SELECT_PRESS:
                if self._knobs_handler.note_on(note, vel, _alt_mode()):
                    return True
                return self._default_note_on(note)
            if self._current_handler.note_on(note, vel, _alt_mode()):
                return True
            return self._default_note_on(note)

        if evtype == EV_NOTE_OFF:
            note = ev[1] & 0x7F

            if note == BTN_SHIFT:
                # Sticky - release does nothing, state already flipped on
                # the press above.
                return True
            if note == BTN_ALT:
                self._is_alt = False
                self._leds.led_on(LED_ALT, LED_Y_DULL)
                if self._current_handler is self._mixer_handler:
                    self._mixer_handler.set_alt(False)
                elif self._current_handler is self._stepseq_handler:
                    self._stepseq_handler.set_alt(False)
                return True
            if note in (BTN_PLAY, BTN_STOP):
                self._btn_timer.is_released(note)
                return True
            if note in self._zynpot_touch_active:
                self._zynpot_touch_active.discard(note)
                self._btn_timer.is_released(note)
                return True

            return self._current_handler.note_off(note, _alt_mode())

        if evtype == EV_CC:
            ccnum = ev[1] & 0x7F
            ccval = ev[2] & 0x7F
            if self._knobs_handler.cc_change(ccnum, ccval):
                return True
            return self._default_cc_change(ccnum, ccval)

        if ev[0] == EV_SYSEX:
            logging.info(f" received SysEx => {ev}")
            return True

        return False

    def update_mixer_strip(self, chan, symbol, value):
        # Guarded: MixerHandler.update_mixer_strip() only repaints (no state
        # to keep fresh for later), so skip it entirely while Mixer isn't the
        # visible mode - otherwise e.g. turning a knob on some other screen
        # (or any other zctrl change reaching here) would paint mixer bars on
        # top of whatever's actually showing on the grid.
        if self._current_handler is self._mixer_handler:
            self._mixer_handler.update_mixer_strip(chan, symbol, value)

    def update_mixer_active_chain(self, active_chain):
        self._mixer_handler.set_active_chain(active_chain, self._current_handler is self._mixer_handler)

    def update_seq_state(self, bank, seq, state=None, mode=None, group=None):
        # Same reasoning as update_mixer_strip above.
        if self._current_handler is self._zynpad_handler:
            self._zynpad_handler.update_seq_state(bank, seq, state, mode, group)
        # Unconditional, unlike the zynpad forwarding above - the Play/Stop
        # LEDs' own "is any pattern playing" check (see _update_mode_leds)
        # needs to track every sequence, everywhere, regardless of which
        # mode is current or which one is currently selected. Maintained
        # incrementally here rather than scanning all sequences on every LED
        # refresh, since this signal already fires globally for every
        # sequence's play-state change (same one Zynpad's own reactive pad
        # colors are built on) - found on real hardware that checking only
        # zynpad's own selected_pad missed every other pattern still
        # playing in the background.
        if state in (zynseq.SEQ_PLAYING, zynseq.SEQ_STARTING, zynseq.SEQ_RESTARTING):
            self._playing_sequences.add((bank, seq))
        else:
            self._playing_sequences.discard((bank, seq))
        self._update_mode_leds()

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
        else:
            zynpot = ZYNPOT_KNOBS.get(btn)
            if zynpot is not None:
                letter = {CONST.PT_SHORT: 'S', CONST.PT_BOLD: 'B', CONST.PT_LONG: 'L'}[press_type]
                self.state_manager.send_cuia("V5_ZYNPOT_SWITCH", [zynpot, letter])

    def _default_note_on(self, note):
        """Fallback for a note the current handler declined (returned
        falsy) - formerly DeviceHandler/ZynpadHandler's own copy-pasted
        logic, now the one shared place for it. Touch just resets the
        shared default zynpot's easing accumulator (Alt+touch, a deliberate
        push, is already handled earlier in midi_event and never reaches
        here); Select's own push defaults to zynpot switch 3, same action
        every handler with nothing more specific for it already sent."""
        if note in ZYNPOT_KNOBS:
            self._default_zynpot.reset(note)
            return True
        if note == BTN_SELECT_PRESS:
            self.state_manager.send_cuia("V5_ZYNPOT_SWITCH", [3, 'S'])
            return True
        return False

    def _default_cc_change(self, ccnum, ccval):
        """Fallback for a CC the current handler declined (returned falsy) -
        same story as _default_note_on above. Select defaults to list
        navigation (up/down, or left/right while Alt is held - see
        _select_knob_arrow), the other 4 knobs to zynpot rotate."""
        if ccnum == KNOB_SELECT:
            _select_knob_arrow(self.state_manager, ccval, self._is_alt)
            return True
        return self._default_zynpot.cc_change(ccnum, ccval)

    def _set_current_handler(self, handler):
        """Switch to the given handler (no-op on the pad-grid side if it's
        already current), refreshing its pad state and the mode LEDs.
        Always resyncs _knobs_handler to match, even on the pad no-op path -
        needed for e.g. re-linking back onto the screen _current_handler was
        already (silently) showing, where _knobs_handler had since drifted
        to self._null_handler (see _update_current_handler) and needs
        pulling back in line even though the pads never actually changed."""
        self._knobs_handler = handler
        if self._current_handler is handler:
            return
        old_handler = self._current_handler
        self._current_handler = handler
        if handler is self._mixer_handler:
            # Pick up Alt if it was already held before switching in (its
            # press/release edges only forward to MixerHandler/StepSeqHandler
            # while one of them is already the active mode - see BTN_ALT
            # handling).
            self._mixer_handler.set_alt(self._is_alt)
        elif handler is self._stepseq_handler:
            self._stepseq_handler.set_alt(self._is_alt)
        # set_active() is a no-op for every handler except PlayHandler, which
        # uses it to stop any still-sounding notes on the way out - harmless
        # to call unconditionally on both ends of the switch.
        old_handler.set_active(False)
        handler.set_active(True)
        # Unconditional full clear rather than each handler tracking which
        # notes the *other* one used - simpler, and can't miss anything as
        # more pad-owning modes are added.
        self._pads.all_off()
        self._current_handler.refresh()
        self._update_mode_leds()
        self._refresh_oled()

    def _update_current_handler(self):
        """While screen-linked, re-derive _current_handler (pad grid) and
        _knobs_handler from _last_screen. While unlinked, do nothing - both
        stay exactly as-is regardless of screen changes, until re-linked
        (which immediately re-syncs to whatever screen is current at that
        point) - a deliberate pin (see Alt+Browser) applies to knobs too, not
        just pads. Called on screen changes and on the Alt+Browser link
        toggle.

        A screen with no dedicated mode of its own (Admin, Preset, Control,
        Snapshot, main menu, etc., now that DeviceHandler is gone) matches
        none of the cases below: _current_handler - and the pad grid - just
        stays whatever it already was, rather than switching to anything or
        going blank, but _knobs_handler goes to self._null_handler so the 4
        knobs/Select/Select-push fall to the generic default instead of
        running that stale mode's own logic invisibly (see its own comment
        in __init__)."""
        if not self._screen_linked:
            return
        if self._last_screen == "audio_mixer":
            self._set_current_handler(self._mixer_handler)
        elif self._last_screen == "zynpad":
            self._set_current_handler(self._zynpad_handler)
        elif self._last_screen == "pattern_editor":
            self._set_current_handler(self._stepseq_handler)
        else:
            self._knobs_handler = self._null_handler

    def _on_gui_show_screen(self, screen, **kwargs):
        self._last_screen = screen
        self._update_current_handler()
        # _update_current_handler() only refreshes LEDs (via
        # _set_current_handler) on an actual pad-mode change - Note's own
        # "on the Snapshot screen" hint (see _update_mode_leds) needs to
        # react to every screen change regardless, since landing on
        # Snapshot alone never changes which mode the pad grid shows.
        self._update_mode_leds()
