from dataclasses import dataclass
from functools import partial
import logging
import time
from typing import Optional
from zyncoder.zyncore import lib_zyncore

from zyngine.ctrldev.zynthian_ctrldev_akai_apc_key25_mk2 import BTN_RECORD, BTN_PLAY, BTN_STOP_ALL_CLIPS
from zyngine.zynthian_signal_manager import zynsigman

from zyngine.zynthian_engine_audioplayer import zynthian_engine_audioplayer
from zyngine.ctrldev.zynthian_ctrldev_base import zynthian_ctrldev_base
from zyngine.ctrldev.zynthian_ctrldev_base_extended import CONST, KnobSpeedControl, ButtonTimer
from zyngine.ctrldev.zynthian_ctrldev_base_ui import ModeHandlerBase
from zyngui import zynthian_gui_config

class zynthian_ctrldev_mvave_smk37_elite(zynthian_ctrldev_base):

    dev_ids = ["*"]
    driver_name = "M-Vave SMK-37 Elite"
    driver_description = "TODO"
    unroute_from_chains = False
    autoload_flag = False

    def __init__(self, state_manager, idev_in, idev_out=None):

        self.state_manager = state_manager
        self._display_controller = DisplayController(idev_out)
        self._light_controller = LightController(idev_out)

        self._device_handler = DeviceHandler(state_manager, self._display_controller, self._light_controller)
        self._current_handler = self._device_handler

        self._signals = [
            (zynsigman.S_GUI,
             zynsigman.SS_GUI_SHOW_SCREEN,
             self._on_gui_show_screen),

            (zynsigman.S_AUDIO_PLAYER,
             zynthian_engine_audioplayer.SS_AUDIO_PLAYER_STATE,
             lambda handle, state:
             self._on_media_change_state(state, f"audio-{handle}", "player")),

            (zynsigman.S_AUDIO_RECORDER,
             state_manager.audio_recorder.SS_AUDIO_RECORDER_STATE,
             partial(self._on_media_change_state, media="audio", kind="recorder")),

            (zynsigman.S_STATE_MAN,
             state_manager.SS_MIDI_PLAYER_STATE,
             partial(self._on_media_change_state, media="midi", kind="player")),

            (zynsigman.S_STATE_MAN,
             state_manager.SS_MIDI_RECORDER_STATE,
             partial(self._on_media_change_state, media="midi", kind="recorder"))
        ]

        # NOTE: init will call refresh(), so _current_hanlder must be ready!
        super().__init__(state_manager, idev_in, idev_out)
        self._current_handler.set_active(True)

    def init(self):
        logging.info("SMK-37 Elite-Master: init")
        super().init()
        for signal, subsignal, callback in self._signals:
            zynsigman.register(signal, subsignal, callback)
        
    def end(self):
        logging.info("SMK-37 Elite-Master: end")
        for signal, subsignal, callback in self._signals:
            zynsigman.unregister(signal, subsignal, callback)
        self._current_handler.set_active(False)
        self._display_controller.clear_all_strings()
        self._light_controller.restore_all_lights()
        super().end()

    def refresh(self):
        super().refresh()
        self._current_handler.refresh()

    def midi_event(self, ev: bytes):
        evtype = (ev[0] >> 4) & 0x0F

        if evtype == CONST.MIDI_PC:
            program = ev[1] & 0x7F
            return self._current_handler.pg_change(program)

        elif evtype == CONST.MIDI_NOTE_ON:
            note = ev[1] & 0x7F
            velocity = ev[2] & 0x7F
            channel = ev[0] & 0x0F
            return self._current_handler.note_on(note, channel, velocity)

        elif evtype == CONST.MIDI_NOTE_OFF:
            note = ev[1] & 0x7F
            channel = ev[0] & 0x0F
            return self._current_handler.note_off(note, channel)

        elif evtype == CONST.MIDI_CC:
            ccnum = ev[1] & 0x7F
            ccval = ev[2] & 0x7F
            return self._current_handler.cc_change(ccnum, ccval)

        elif ev[0] == CONST.MIDI_SYSEX:
            return self._current_handler.sysex_message(ev[1:-1])

        return False

    # def on_alt_mode(self, alt_mode: bool):
    #     refresh = self._current_handler == self._device_handler
    #     self._device_handler.on_alt_mode(alt_mode, refresh)

    def _on_gui_show_screen(self, screen):
        self._device_handler.on_screen_change(screen)
        if self._current_handler == self._device_handler:
            self._current_handler.refresh()

    def _on_media_change_state(self, state, media, kind):
        self._current_handler.on_media_change(media, kind, state)
        if self._current_handler == self._device_handler:
            self._current_handler.refresh()

    # def _change_handler(self, new_handler):
    #     if new_handler == self._current_handler:
    #         return
    #     if self._current_handler is not None:
    #         self._current_handler.set_active(False)
    #     self._current_handler = new_handler
    #     self._current_handler.set_active(True)


MANUFACTURER_ID = 0x35
BANK_SWITCH_CMD = 0x36
STRING_DISPLAY_CMD = 0x37

def send_midi_message(idev, status: int, data1: int, data2: int):
    """
    Send a standard MIDI message.

    Args:
        status: MIDI status byte (e.g., 0x90 for Note On Ch1)
        data1: First data byte (note number or CC number)
        data2: Second data byte (velocity or CC value)
    """
    msg = bytes((status, data1, data2))
    lib_zyncore.dev_send_midi_event(idev, msg, len(msg))

def send_sysex(idev, data):
    """
    Send a SysEx message.

    Args:
        data: List of bytes including F0 start and F7 end
    """
    logging.info(f"  SysEx out: {' '.join(f'0x{b:02X}' for b in data)}")
    msg = bytes(data)
    lib_zyncore.dev_send_midi_event(idev, msg, len(msg))

PAD_CC = {
    16: 64, 17: 65, 18: 66, 19: 67, 20: 68, 21: 69, 22: 70, 23: 71,
    24: 94, 25: 93, 26: 95, 27: 91, 28: 92, 29: 46, 30: 47, 31: 76
}

CC_PAD = {ccnum: pad for pad, ccnum in PAD_CC.items()}

class Colors:
    OFF = 0
    PURPLE = [1, 12, 13, 25, 26, 27, 38, 39, 40, 53, 54, 55, 67, 68, 69]
    YELLOW = [2, 3, 4, 18, 31, 32, 33, 34, 44, 45, 46]
    GREEN = [5, 6, 19, 20, 21, 35, 36, 47, 48, 49, 61, 62, 63]
    CYAN = [7, 8, 22, 37, 50, 64]
    BLUE = [9, 10, 11, 23, 24, 51, 52, 65, 66]
    WHITE = [14, 28, 41, 42, 56, 70]
    RED = [15, 29, 57]
    ORANGE = [16, 30]
    BROWN = [17, 43, 58, 59, 60]

class ColorEffects:
    """Light effect types."""
    SOLID = 0x5A
    BREATHING = 0x5B
    RESTORE = 0x5C

class LightController:
    """Controls pad LEDs on the SMK-37 Elite."""

    # Pad indices for different sections
    MODE_BUTTONS = list(range(0, 4))  # Pads 0-3
    NAVIGATION_BUTTONS = list(range(4, 8))  # Pads 4-7
    TRANSPORT_BUTTONS = list(range(8, 16))  # Pads 8-15
    MODE_SPECIFIC_PADS = list(range(16, 32))  # Pads 16-31
    PLAY_BUTTON = 32
    RECORD_BUTTON = 33

    # Grid pad note to light index mapping
    GRID_PAD_NOTE_TO_LIGHT = {
        0x28: 0, 0x26: 1, 0x2E: 2, 0x2C: 3,
        0x31: 4, 0x37: 5, 0x33: 6, 0x35: 7,
        0x25: 8, 0x24: 9, 0x2A: 10, 0x36: 11,
        0x30: 12, 0x2F: 13, 0x2D: 14, 0x2B: 15
    }

    def __init__(self, idev):
        """
        Initialize light controller.

        Args:
            idev: midi out device
        """
        self.idev = idev
        self._light_states = {}  # Cache current light states

    def set_pad_light(self, pad_index: int, color: int, effect: int = ColorEffects.SOLID, force: bool = False):
        """
        Set individual pad light.

        Args:
            pad_index: Pad index (0-33)
            color: Color value (see Colors class)
            effect: Effect type (see Effects class)
            force: If True, bypass cache and send message anyway
        """
        if not (0 <= pad_index <= 33):
            print(f"Invalid pad index: {pad_index}")
            return

        # Check cache to avoid redundant messages (unless force=True)
        cache_key = (pad_index, color, effect)
        if not force and self._light_states.get(pad_index) == cache_key:
            return

        sysex = [0xF0, MANUFACTURER_ID, pad_index, color, effect, 0xF7]
        print(f"  Sending light SysEx: pad={pad_index}, color=0x{color:02X}, effect=0x{effect:02X}")
        send_sysex(self.idev, sysex)
        self._light_states[pad_index] = cache_key

    def set_light_solid(self, pad_index: int, color: int):
        """Set pad to solid color."""
        self.set_pad_light(pad_index, color, ColorEffects.SOLID)

    def set_light_breathing(self, pad_index: int, color: int):
        """Set pad to breathing animation."""
        self.set_pad_light(pad_index, color, ColorEffects.BREATHING)

    def set_light_flashing(self, pad_index: int, color1: int, color2: int):
        """
        Set pad to flash between two colors.

        Args:
            pad_index: Pad index (0-33)
            color1: First color
            color2: Second color
        """
        if not (0 <= pad_index <= 33):
            print(f"Invalid pad index: {pad_index}")
            return

        # For flashing, color2 is sent in effect position
        sysex = [0xF0, MANUFACTURER_ID, pad_index, color1, color2, 0xF7]
        send_sysex(self.idev, sysex)
        self._light_states[pad_index] = (pad_index, color1, color2)

    def turn_off_light(self, pad_index: int):
        """Turn off specific pad light."""
        self.set_pad_light(pad_index, Colors.OFF, ColorEffects.RESTORE)

    def turn_off_all_lights(self):
        """Turn off all pad lights."""
        for pad_index in range(34):
            self.turn_off_light(pad_index)
            time.sleep(0.01)

    def restore_all_lights(self):
        """Turn off all pad lights."""
        for pad_index in range(34):
            self.set_pad_light(pad_index, Colors.OFF, ColorEffects.RESTORE)
            time.sleep(0.01)

    def clear_light_cache(self):
        """Clear the light state cache to force refresh on next command."""
        self._light_states.clear()
        print("Light cache cleared")

    def set_mode_buttons_color(self, color: int):
        """Set all mode button lights to a color."""
        for i in self.MODE_BUTTONS:
            self.set_light_solid(i, color)

    def set_navigation_buttons_color(self, color: int):
        """Set all navigation button lights to a color."""
        for i in self.NAVIGATION_BUTTONS:
            self.set_light_solid(i, color)

    def set_transport_buttons_color(self, color: int):
        """Set all transport button lights to a color."""
        for i in self.TRANSPORT_BUTTONS:
            self.set_light_solid(i, color)

    def flash_play_button(self, color1: int, color2: int):
        """Set play button to flash between two colors."""
        self.set_light_flashing(self.PLAY_BUTTON, color1, color2)

    def flash_record_button(self, color1: int, color2: int):
        """Set record button to flash between two colors."""
        self.set_light_flashing(self.RECORD_BUTTON, color1, color2)

    def get_light_state(self, pad_index: int) -> Optional[tuple]:
        """Get cached light state for a pad."""
        return self._light_states.get(pad_index)


class DisplayController:
    """Controls text displays on pads, knobs, and faders."""

    # Target ID ranges for different displays
    PAD_DISPLAY_START = 0x00  # Pads 0-15: 0x00-0x0F
    MODE_PAD_DISPLAY_START = 0x10  # Pads 16-31: 0x10-0x1F
    KNOB_NAME_DISPLAY_START = 0x20  # Knobs 0-15 names: 0x20-0x2F
    KNOB_VALUE_DISPLAY_START = 0x30  # Knobs 0-15 values: 0x30-0x3F
    TITLE_DISPLAY = 0x40  # Startup display
    FADER_NAME_DISPLAY_START = 0x41  # Faders 0-7 names: 0x41-0x48
    FADER_VALUE_DISPLAY_START = 0x49  # Faders 0-7 values: 0x49-0x50

    # Character encoding map for the device display
    CHAR_TO_BYTE = {
        'a': 0x00, 'b': 0x01, 'c': 0x02, 'd': 0x03, 'e': 0x04, 'f': 0x05, 'g': 0x06, 'h': 0x07,
        'i': 0x08, 'j': 0x09, 'k': 0x0A, 'l': 0x0B, 'm': 0x0C,
        'n': 0x0D, 'o': 0x0E, 'p': 0x0F, 'q': 0x10, 'r': 0x11, 's': 0x12, 't': 0x13,
        'u': 0x14, 'v': 0x15, 'w': 0x16, 'x': 0x17, 'y': 0x18, 'z': 0x19,
        'A': 0x1A, 'B': 0x1B, 'C': 0x1C, 'D': 0x1D, 'E': 0x1E, 'F': 0x1F, 'G': 0x20,
        'H': 0x21, 'I': 0x22, 'J': 0x23, 'K': 0x24, 'L': 0x25, 'M': 0x26,
        'N': 0x27, 'O': 0x28, 'P': 0x29, 'Q': 0x2A, 'R': 0x2B, 'S': 0x2C, 'T': 0x2D,
        'U': 0x2E, 'V': 0x2F, 'W': 0x30, 'X': 0x31, 'Y': 0x32, 'Z': 0x33,
        '0': 0x34, '1': 0x35, '2': 0x36, '3': 0x37, '4': 0x38,
        '5': 0x39, '6': 0x3A, '7': 0x3B, '8': 0x3C, '9': 0x3D,
        '+': 0x3E, '-': 0x3F, '*': 0x40, '/': 0x41, ' ': 0x42, '.': 0x43, '#': 0x44
    }

    @classmethod
    def text_to_bytes(cls, text: str) -> list:
        """Convert string to device-specific byte encoding."""
        return [cls.CHAR_TO_BYTE.get(char, 0x20) for char in text]

    MAX_TEXT_LENGTH = 255

    def __init__(self, idev):
        """
        Initialize display controller.

        Args:
            idev: midi out device
        """
        self.idev = idev

    def _send_string_message(self, target_id: int, text: str):
        """
        Send string to a specific target display.

        Args:
            target_id: Target display ID
            text: Text to display
        """
        text_bytes = self.text_to_bytes(text)
        length = len(text_bytes)

        if length > self.MAX_TEXT_LENGTH:
            text_bytes = text_bytes[:self.MAX_TEXT_LENGTH]
            length = self.MAX_TEXT_LENGTH

        sysex = [0xF0, MANUFACTURER_ID, STRING_DISPLAY_CMD,
                 target_id, length] + text_bytes + [0xF7]
        send_sysex(self.idev, sysex)

    def send_title_string(self, text: str):
        """
        Send text to screen title

        Args:
            text: Text to display
        """
        target_id = self.TITLE_DISPLAY
        self._send_string_message(target_id, text)

    def send_pad_string(self, pad_index: int, text: str):
        """
        Send text to a pad display (pads 0-31).

        Args:
            pad_index: Pad index (0-31)
            text: Text to display
        """
        if not (0 <= pad_index <= 31):
            print(f"Invalid pad index: {pad_index}")
            return

        target_id = self.PAD_DISPLAY_START + pad_index
        self._send_string_message(target_id, text)

    def send_knob_name_string(self, knob_index: int, text: str):
        """
        Send text to a knob's name display (knobs 0-15).

        Args:
            knob_index: Knob index (0-15)
            text: Text to display
        """
        if not (0 <= knob_index <= 15):
            print(f"Invalid knob index: {knob_index}")
            return

        target_id = self.KNOB_NAME_DISPLAY_START + knob_index
        self._send_string_message(target_id, text)

    def send_knob_value_string(self, knob_index: int, text: str):
        """
        Send text to a knob's value display (knobs 0-15).

        Args:
            knob_index: Knob index (0-15)
            text: Text to display
        """
        if not (0 <= knob_index <= 15):
            print(f"Invalid knob index: {knob_index}")
            return

        target_id = self.KNOB_VALUE_DISPLAY_START + knob_index
        self._send_string_message(target_id, text)

    def send_knob_volume_with_value(self, knob_index: int, volume_0_125: int,
                                    volume_db_text: str):
        """
        Send volume value (0-125) with dB text to knob display.

        Args:
            knob_index: Knob index (0-15)
            volume_0_125: Volume value (0-125)
            volume_db_text: Text showing dB value (e.g., "0.0db")
        """
        if not (0 <= knob_index <= 15):
            print(f"Invalid knob index: {knob_index}")
            return

        target_id = self.KNOB_VALUE_DISPLAY_START + knob_index
        volume_byte = min(max(volume_0_125, 0), 125)
        db_text_bytes = self.text_to_bytes(volume_db_text)

        combined_bytes = [volume_byte] + db_text_bytes
        length = len(combined_bytes)

        if length > self.MAX_TEXT_LENGTH:
            combined_bytes = combined_bytes[:self.MAX_TEXT_LENGTH]
            length = self.MAX_TEXT_LENGTH

        sysex = [0xF0, MANUFACTURER_ID, STRING_DISPLAY_CMD,
                 target_id, length] + combined_bytes + [0xF7]
        send_sysex(self.idev, sysex)

    def send_fader_name_string(self, fader_index: int, text: str):
        """
        Send text to a fader's name display (faders 0-7).

        Args:
            fader_index: Fader index (0-7)
            text: Text to display
        """
        if not (0 <= fader_index <= 7):
            print(f"Invalid fader index: {fader_index}")
            return

        target_id = self.FADER_NAME_DISPLAY_START + fader_index
        self._send_string_message(target_id, text)

    def send_fader_value_string(self, fader_index: int, text: str):
        """
        Send text to a fader's value display (faders 0-7).

        Args:
            fader_index: Fader index (0-7)
            text: Text to display
        """
        if not (0 <= fader_index <= 7):
            print(f"Invalid fader index: {fader_index}")
            return

        target_id = self.FADER_VALUE_DISPLAY_START + fader_index
        self._send_string_message(target_id, text)

    def clear_all_strings(self):
        """Clear all text displays on the device."""

        # Clear all command: F0 35 37 7F 03 7F 7F F7
        sysex = [0xF0, MANUFACTURER_ID, STRING_DISPLAY_CMD,
                 0x7F, 0x03, 0x7F, 0x7F, 0xF7]
        send_sysex(self.idev, sysex)

    def clear_mode_strings(self):
        """Clear only mode-specific pad strings (pads 16-31)."""
        for i in range(16, 32):
            self.send_pad_string(i, "")

    def clear_global_strings(self):
        """Clear only global pad strings (pads 0-15)."""
        for i in range(16):
            self.send_pad_string(i, "")

    def setup_demo_display(self):
        """Setup a demo display configuration for testing."""
        # Send startup message
        self._send_string_message(self.STARTUP_DISPLAY, "SMK-37 Test")

        # Set pad labels
        self.send_pad_string(0, "Mode 1")
        self.send_pad_string(1, "Mode 2")
        self.send_pad_string(2, "Mode 3")
        self.send_pad_string(3, "Mode 4")

        # Set knob names
        self.send_knob_name_string(0, "Vol")
        self.send_knob_name_string(1, "Pan")
        self.send_knob_name_string(2, "Cut")
        self.send_knob_name_string(3, "Res")

        # Set knob values
        self.send_knob_volume_with_value(0, 100, "0.0db")
        self.send_knob_volume_with_value(1, 64, "C")

        # Set fader names
        self.send_fader_name_string(0, "EQ1")
        self.send_fader_name_string(1, "EQ2")
        self.send_fader_name_string(2, "EQ3")
        self.send_fader_name_string(3, "EQ4")

        # Set fader values
        self.send_fader_value_string(0, "0dB")
        self.send_fader_value_string(1, "+3dB")
        self.send_fader_value_string(2, "-2dB")
        self.send_fader_value_string(3, "+1dB")

class DeviceHandler(ModeHandlerBase):

    PADS_LAYOUT = {
        # 1st row
        16: {"color": Colors.BLUE[1], "name": "ALT"},
        17: {"color": Colors.BLUE[0], "name": "OPT"},
        18: {"color": Colors.BLUE[0], "name": "MIX"},
        19: {"color": Colors.BLUE[0], "name": "CTRL"},
        20: {"color": Colors.BLUE[0], "name": "ZS3"},
        21: {"color": Colors.RED[0], "name": "BACK"},
        22: {"color": Colors.YELLOW[0], "name": "UP"},
        23: {"color": Colors.GREEN[0], "name": "SEL"},
        # 2nd row
        24: {"color": Colors.BLUE[0], "name": "REC"},
        25: {"color": Colors.BLUE[0], "name": "STOP"},
        26: {"color": Colors.BLUE[0], "name": "PLAY"},
        27: {"color": Colors.BLUE[0], "name": "TEMPO"},
        28: {"color": Colors.BLUE[0], "name": "PAD"},
        29: {"color": Colors.YELLOW[0], "name": "LEFT"},
        30: {"color": Colors.YELLOW[0], "name": "DOWN"},
        31: {"color": Colors.YELLOW[0], "name": "RIGHT"}
    }

    BTN_ALT = 16
    BTN_OPT_ADMIN = 17
    BTN_MIX_LEVEL = 18
    BTN_CTRL_PRESET = 19
    BTN_ZS3_SHOT = 20
    BTN_BACK_NO = 21
    BTN_UP = 22
    BTN_SEL_YES = 23
    BTN_RECORD = 24
    BTN_STOP = 25
    BTN_PLAY = 26
    BTN_METRONOME = 27
    BTN_PAD_STEP = 28
    BTN_LEFT = 29
    BTN_DOWN = 30
    BTN_RIGHT = 31

    COLOR_ALT_OFF = Colors.BLUE[1]
    COLOR_ALT_ON = Colors.PURPLE[0]

    COLOR_STATE_0 = Colors.BLUE[0]
    COLOR_STATE_1 = Colors.GREEN[0]
    COLOR_STATE_2 = Colors.ORANGE[0]

    def __init__(self, state_manager, display_controller: DisplayController, light_controller: LightController):
        super().__init__(state_manager)
        self._display_controller = display_controller
        self._light_controller = light_controller

        self._knobs_ease = KnobSpeedControl()
        self._is_alt_active = False
        self._is_playing = set()
        self._is_recording = set()
        self._btn_timer = ButtonTimer(self._handle_timed_button)

        self._btn_actions = {
            self.BTN_OPT_ADMIN: ("MENU", "SCREEN_ADMIN"),
            self.BTN_MIX_LEVEL: ("SCREEN_AUDIO_MIXER", "SCREEN_ALSA_MIXER"),
            self.BTN_CTRL_PRESET: ("SCREEN_CONTROL", "PRESET", "SCREEN_BANK"),
            self.BTN_ZS3_SHOT: ("SCREEN_ZS3", "SCREEN_SNAPSHOT"),
            self.BTN_PAD_STEP: ("SCREEN_ZYNPAD", "SCREEN_PATTERN_EDITOR"),
            self.BTN_METRONOME: ("TEMPO",),
            self.BTN_RECORD: ("TOGGLE_RECORD",),
            self.BTN_PLAY: (
                lambda is_bold: [
                    "AUDIO_FILE_LIST" if is_bold else "TOGGLE_PLAY"
                ]
            ),
            self.BTN_STOP: (
                lambda is_bold: [
                    "ALL_SOUNDS_OFF" if is_bold else "STOP"
                ]
            )
        }

        self._btn_states = {k: -1 for k in self._btn_actions}

    def set_active(self, active):
        super().set_active(active)
        if active:
            self.init()

    def init(self):
        self._display_controller.send_title_string("Device")
        for pad_index, pad_properties in self.PADS_LAYOUT.items():
            self._display_controller.send_pad_string(pad_index, pad_properties["name"])
        self.refresh()

    def refresh(self):
        if self._state_manager.power_save_mode:
            return True

        # Lit up fixed buttons
        for btn in [self.BTN_UP, self.BTN_DOWN, self.BTN_LEFT, self.BTN_RIGHT]:
            self._light_controller.set_light_solid(btn, Colors.YELLOW[0])
        self._light_controller.set_light_solid(self.BTN_SEL_YES, Colors.GREEN[0])
        self._light_controller.set_light_solid(self.BTN_BACK_NO, Colors.RED[0])

        alt_color = self.COLOR_ALT_ON if self._is_alt_active else self.COLOR_ALT_OFF
        self._light_controller.set_light_solid(self.BTN_ALT, alt_color)

        # Lit up state-full control buttons
        for btn, state in self._btn_states.items():
            color = [self.COLOR_STATE_1, self.COLOR_STATE_2, self.COLOR_STATE_0][state]
            self._light_controller.set_light_solid(btn, color)

        # Transport buttons
        if self._btn_states[self.BTN_PAD_STEP] == 1:
            self._light_controller.set_light_solid(self.BTN_RECORD, self.COLOR_STATE_2)
            self._light_controller.set_light_solid(self.BTN_STOP, self.COLOR_STATE_2)
            self._light_controller.set_light_solid(self.BTN_PLAY, self.COLOR_STATE_2)
        else:
            if not self._is_alt_active:
                if self._is_playing:
                    self._light_controller.set_light_breathing(self.BTN_PLAY, Colors.GREEN[0])
                    self._light_controller.set_light_solid(self.BTN_STOP, Colors.RED[0])
                if self._is_recording:
                    self._light_controller.set_light_breathing(self.BTN_RECORD, Colors.RED[0])
            else:
                if self._is_playing:
                    self._light_controller.set_light_breathing(self.BTN_PLAY, Colors.PURPLE[0])
                    self._light_controller.set_light_solid(self.BTN_STOP, Colors.RED[0])
                if self._is_recording:
                    self._light_controller.set_light_breathing(self.BTN_RECORD, Colors.PURPLE[0])

    def note_on(self, note, velocity, shifted_override=None):
        """Overwrite in derived class if needed."""
        logging.info("Note on: {0} {1} {2}".format(note, velocity, shifted_override))
        return True

    def note_off(self, note, shifted_override=None):
        """Overwrite in derived class if needed."""
        logging.info("Note off: {0} {1}".format(note, shifted_override))
        return True

    def cc_change(self, ccnum, ccval):
        """Overwrite in derived class if needed."""
        logging.info("CC Change: {0} {1}".format(ccnum, ccval))

        btn = CC_PAD.get(ccnum, None)
        if btn is None:
            return

        if ccval > 0:
            if btn == self.BTN_UP:
                self._state_manager.send_cuia("ARROW_UP")
            elif btn == self.BTN_DOWN:
                self._state_manager.send_cuia("ARROW_DOWN")
            elif btn == self.BTN_LEFT:
                self._state_manager.send_cuia("ARROW_LEFT")
            elif btn == self.BTN_RIGHT:
                self._state_manager.send_cuia("ARROW_RIGHT")
            elif btn == self.BTN_SEL_YES:
                self._state_manager.send_cuia("V5_ZYNPOT_SWITCH", [3, 'S'])
            elif btn == self.BTN_BACK_NO:
                self._state_manager.send_cuia("BACK")
            elif btn == self.BTN_ALT:
                self._is_alt_active = not self._is_alt_active
                self._state_manager.send_cuia("TOGGLE_ALT_MODE")
                self.refresh()
            else:
                # Buttons that may have bold/long press
                self._btn_timer.is_pressed(btn, time.time())
        else:
            self._btn_timer.is_released(btn)

        return True

    def pg_change(self, program):
        """Overwrite in derived class if needed."""
        logging.info("PG Change: {0}".format(program))

    def sysex_message(self, payload):
        """Overwrite in derived class if needed."""
        logging.info("SYSEX: {0}".format(payload))

    def on_alt_mode(self, alt_mode, refresh=False):
        logging.info("on_alt_mode {0} {1}".format(alt_mode, refresh))
        if refresh:
            self.refresh()

    def on_screen_change(self, screen):
        screen_map = {
            "option": (self.BTN_OPT_ADMIN, 0),
            "main_menu": (self.BTN_OPT_ADMIN, 0),
            "admin": (self.BTN_OPT_ADMIN, 1),
            "audio_mixer": (self.BTN_MIX_LEVEL, 0),
            "alsa_mixer": (self.BTN_MIX_LEVEL, 1),
            "control": (self.BTN_CTRL_PRESET, 0),
            "engine": (self.BTN_CTRL_PRESET, 0),
            "preset": (self.BTN_CTRL_PRESET, 1),
            "bank": (self.BTN_CTRL_PRESET, 1),
            "zs3": (self.BTN_ZS3_SHOT, 0),
            "snapshot": (self.BTN_ZS3_SHOT, 1),
            "zynpad": (self.BTN_PAD_STEP, 0),
            "pattern_editor": (self.BTN_PAD_STEP, 1),
            "arranger": (self.BTN_PAD_STEP, 1),
            "tempo": (self.BTN_METRONOME, 0),
        }

        self._btn_states = {k: -1 for k in self._btn_states}
        try:
            btn, idx = screen_map[screen]
            self._btn_states[btn] = idx
        except KeyError:
            pass

    def on_media_change(self, media, kind, state):
        flags = self._is_playing if kind == "player" else self._is_recording
        flags.add(media) if state else flags.discard(media)

    def _handle_timed_button(self, btn, press_type):
        if press_type == CONST.PT_LONG:
            cuia = {
                self.BTN_OPT_ADMIN: "POWER_OFF",
                self.BTN_CTRL_PRESET: "PRESET_FAV",
                self.BTN_PAD_STEP: "SCREEN_ARRANGER",
            }.get(btn)
            if cuia:
                self._state_manager.send_cuia(cuia)
            return True

        actions = self._btn_actions.get(btn)
        if actions is None:
            return
        if callable(actions):
            actions = actions(press_type == CONST.PT_BOLD)

        idx = -1
        if press_type == CONST.PT_SHORT:
            idx = self._btn_states[btn]
            idx = (idx + 1) % len(actions)
            cuia = actions[idx]
        elif press_type == CONST.PT_BOLD:
            # In buttons with 2 functions, the default on bold press is the second
            idx = 1 if len(actions) > 1 else 0
            cuia = actions[idx]

        # Split params, if given
        params = []
        if ":" in cuia:
            cuia, params = cuia.split(":")
            params = params.split(",")
            params[0] = int(params[0])

        self._state_manager.send_cuia(cuia, params)
        return True
