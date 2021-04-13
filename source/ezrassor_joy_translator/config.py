"""Config manipulation functions and related enumerations."""
import enum


TYPE_KEY = "type"
INDEX_KEY = "index"
ACCEPT_KEY = "accept"
AXIS_TYPE = "axis"
BUTTON_TYPE = "button"


def verify_config(config):
    """Validate the contents of the config map."""
    for signal in Signal:
        if signal not in config:
            raise VerificationError(
                f"signal '{signal.value}' is missing from config",
            )
        if TYPE_KEY not in config[signal]:
            raise VerificationError(
                f"signal '{signal.value}' must include a type",
            )
        if not (
            config[signal][TYPE_KEY] == AXIS_TYPE
            or config[signal][TYPE_KEY] == BUTTON_TYPE
        ):
            raise VerificationError(
                f"signal '{signal.value}' type must be "
                + f"'{BUTTON_TYPE}' or '{AXIS_TYPE}'",
            )
        if INDEX_KEY not in config[signal]:
            raise VerificationError(
                f"signal '{signal.value}' must include an index",
            )
        if not isinstance(config[signal][INDEX_KEY], int):
            raise VerificationError(
                f"signal '{signal.value}' index must be an integer",
            )
        if config[signal][INDEX_KEY] < 0:
            raise VerificationError(
                f"signal '{signal.value}' index must be non-negative",
            )
        if ACCEPT_KEY in config[signal] and not isinstance(
            config[signal][ACCEPT_KEY], float
        ):
            raise VerificationError(
                f"signal '{signal.value}' accept must be a float",
            )


def read_signal(config, message, signal):
    """Read a signal in a given message with respect to a given config."""
    if config[signal][TYPE_KEY] == AXIS_TYPE:
        value = message.axes[config[signal][INDEX_KEY]]
        if ACCEPT_KEY in config[signal]:
            return value == config[signal][ACCEPT_KEY]
        else:
            return value
    else:
        return message.buttons[config[signal][INDEX_KEY]] == 1


class Signal(enum.Enum):
    """Available signals for the EZRASSOR."""

    LEFT_WHEEL = "left_wheel_signal"
    RIGHT_WHEEL = "right_wheel_signal"
    RAISE_FRONT_ARM = "raise_front_arm_signal"
    LOWER_FRONT_ARM = "lower_front_arm_signal"
    RAISE_BACK_ARM = "raise_back_arm_signal"
    LOWER_BACK_ARM = "lower_back_arm_signal"
    DIG_FRONT_DRUM = "dig_front_drum_signal"
    DUMP_FRONT_DRUM = "dump_front_drum_signal"
    DIG_BACK_DRUM = "dig_back_drum_signal"
    DUMP_BACK_DRUM = "dump_back_drum_signal"
    STOP_ROUTINE = "stop_routine_signal"
    AUTO_DRIVE_ROUTINE = "auto_drive_routine_signal"
    AUTO_DIG_ROUTINE = "auto_dig_routine_signal"
    AUTO_DUMP_ROUTINE = "auto_dump_routine_signal"
    AUTO_DOCK_ROUTINE = "auto_dock_routine_signal"
    FULL_AUTONOMY_ROUTINE = "full_autonomy_routine_signal"


class VerificationError(Exception):
    """Encapsulate verification errors."""

    def __init__(self, message):
        """Initialize this error with a message."""
        self.message = message

    def ___str___(self):
        """Create a human-readable representation of this error."""
        return self.message
