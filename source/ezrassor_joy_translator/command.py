"""Data structures that make up an EZRASSOR command."""
import enum
import ezrassor_joy_translator as translator


def create_command(read_signal, message):
    """Create a command from a message with controller data.

    The read_signal() function reads controller data for different signal types
    (like button presses or axis changes).
    """

    # Set a routine action, if available.
    routine_action = None
    if read_signal(message, translator.Signal.STOP_ROUTINE) is True:
        routine_action = RoutineAction.STOP
    elif read_signal(message, translator.Signal.AUTO_DRIVE_ROUTINE) is True:
        routine_action = RoutineAction.AUTO_DRIVE
    elif read_signal(message, translator.Signal.AUTO_DIG_ROUTINE) is True:
        routine_action = RoutineAction.AUTO_DIG
    elif read_signal(message, translator.Signal.AUTO_DUMP_ROUTINE) is True:
        routine_action = RoutineAction.AUTO_DUMP
    elif read_signal(message, translator.Signal.AUTO_DOCK_ROUTINE) is True:
        routine_action = RoutineAction.AUTO_DOCK
    elif read_signal(message, translator.Signal.FULL_AUTONOMY_ROUTINE) is True:
        routine_action = RoutineAction.FULL_AUTONOMY

    # Calculate the wheel action using left and right wheel signals.
    wheel_action = WheelAction(
        linear_x=(
            read_signal(message, translator.Signal.RIGHT_WHEEL)
            + read_signal(message, translator.Signal.LEFT_WHEEL)
        )
        / 2,
        angular_z=(
            read_signal(message, translator.Signal.RIGHT_WHEEL)
            - read_signal(message, translator.Signal.LEFT_WHEEL)
        )
        / 2,
    )

    # Set the front arm action.
    if (
        read_signal(message, translator.Signal.RAISE_FRONT_ARM) is True
        and read_signal(message, translator.Signal.LOWER_FRONT_ARM) is True
    ):
        front_arm_action = ArmAction.STOP
    elif read_signal(message, translator.Signal.RAISE_FRONT_ARM) is True:
        front_arm_action = ArmAction.RAISE
    elif read_signal(message, translator.Signal.LOWER_FRONT_ARM) is True:
        front_arm_action = ArmAction.LOWER
    else:
        front_arm_action = ArmAction.STOP

    # Set the back arm action.
    if (
        read_signal(message, translator.Signal.RAISE_BACK_ARM) is True
        and read_signal(message, translator.Signal.LOWER_BACK_ARM) is True
    ):
        back_arm_action = ArmAction.STOP
    elif read_signal(message, translator.Signal.RAISE_BACK_ARM) is True:
        back_arm_action = ArmAction.RAISE
    elif read_signal(message, translator.Signal.LOWER_BACK_ARM) is True:
        back_arm_action = ArmAction.LOWER
    else:
        back_arm_action = ArmAction.STOP

    # Set the front drum action.
    if (
        read_signal(message, translator.Signal.DIG_FRONT_DRUM) is True
        and read_signal(message, translator.Signal.DUMP_FRONT_DRUM) is True
    ):
        front_drum_action = DrumAction.STOP
    elif read_signal(message, translator.Signal.DIG_FRONT_DRUM) is True:
        front_drum_action = DrumAction.DIG
    elif read_signal(message, translator.Signal.DUMP_FRONT_DRUM) is True:
        front_drum_action = DrumAction.DUMP
    else:
        front_drum_action = DrumAction.STOP

    # Set the back drum action.
    if (
        read_signal(message, translator.Signal.DIG_BACK_DRUM) is True
        and read_signal(message, translator.Signal.DUMP_BACK_DRUM) is True
    ):
        back_drum_action = DrumAction.STOP
    elif read_signal(message, translator.Signal.DIG_BACK_DRUM) is True:
        back_drum_action = DrumAction.DIG
    elif read_signal(message, translator.Signal.DUMP_BACK_DRUM) is True:
        back_drum_action = DrumAction.DUMP
    else:
        back_drum_action = DrumAction.STOP

    return Command(
        wheel_action,
        front_arm_action,
        back_arm_action,
        front_drum_action,
        back_drum_action,
        routine_action,
    )


class Command:
    """A command containing actions for an EZRASSOR."""

    def __init__(
        self,
        wheel_action,
        front_arm_action,
        back_arm_action,
        front_drum_action,
        back_drum_action,
        routine_action,
    ):
        """Initialize this command with actions."""
        self.wheel_action = wheel_action
        self.front_arm_action = front_arm_action
        self.back_arm_action = back_arm_action
        self.front_drum_action = front_drum_action
        self.back_drum_action = back_drum_action
        self.routine_action = routine_action


class WheelAction:
    """This action describes how to move the wheels of an EZRASSOR."""

    def __init__(self, linear_x, angular_z):
        """Initialize this action with movement floats."""
        self.linear_x = linear_x
        self.angular_z = angular_z


class ArmAction(enum.Enum):
    """This action describes how to move the arms of an EZRASSOR."""

    LOWER = -1.0
    STOP = 0.0
    RAISE = 1.0


class DrumAction(enum.Enum):
    """This action describes how to move the drums of an EZRASSOR."""

    DUMP = -1.0
    STOP = 0.0
    DIG = 1.0


class RoutineAction(enum.Enum):
    """This action describes which routine to execute for an EZRASSOR."""

    AUTO_DRIVE = 0b000001
    AUTO_DIG = 0b000010
    AUTO_DUMP = 0b000100
    AUTO_DOCK = 0b001000
    FULL_AUTONOMY = 0b010000
    STOP = 0b100000
