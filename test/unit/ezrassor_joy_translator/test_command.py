"""Tests for the command conversion code in this module."""
import ezrassor_joy_translator as translator


# This map of signal readings is first copied and then tweaked for each test.
# Each copy is used to create a mock read_signal() function that is called by
# the create_command() function. This enables us to control which signals are
# active and which are not.
DEFAULT_READINGS = {
    translator.Signal.LEFT_WHEEL: 0.0,
    translator.Signal.RIGHT_WHEEL: 0.0,
    translator.Signal.RAISE_FRONT_ARM: False,
    translator.Signal.LOWER_FRONT_ARM: False,
    translator.Signal.RAISE_BACK_ARM: False,
    translator.Signal.LOWER_BACK_ARM: False,
    translator.Signal.DIG_FRONT_DRUM: False,
    translator.Signal.DUMP_FRONT_DRUM: False,
    translator.Signal.DIG_BACK_DRUM: False,
    translator.Signal.DUMP_BACK_DRUM: False,
    translator.Signal.STOP_ROUTINE: False,
    translator.Signal.AUTO_DRIVE_ROUTINE: False,
    translator.Signal.AUTO_DIG_ROUTINE: False,
    translator.Signal.AUTO_DUMP_ROUTINE: False,
    translator.Signal.AUTO_DOCK_ROUTINE: False,
    translator.Signal.FULL_AUTONOMY_ROUTINE: False,
}


def test_create_command_with_no_signals():
    """Should create a command that does nothing."""
    readings = DEFAULT_READINGS.copy()

    command = translator.create_command(
        lambda message, signal: readings[signal], None
    )

    assert command.routine_action is None
    assert command.wheel_action.linear_x == 0.0
    assert command.wheel_action.angular_z == 0.0
    assert command.front_arm_action is translator.ArmAction.STOP
    assert command.back_arm_action is translator.ArmAction.STOP
    assert command.front_drum_action is translator.DrumAction.STOP
    assert command.back_drum_action is translator.DrumAction.STOP


def test_create_command_with_stop_routine_signal():
    """Should create a command that stops any executing routines."""
    readings = DEFAULT_READINGS.copy()
    readings[translator.Signal.STOP_ROUTINE] = True

    command = translator.create_command(
        lambda message, signal: readings[signal], None
    )

    assert command.routine_action is translator.RoutineAction.STOP
    assert command.wheel_action.linear_x == 0.0
    assert command.wheel_action.angular_z == 0.0
    assert command.front_arm_action is translator.ArmAction.STOP
    assert command.back_arm_action is translator.ArmAction.STOP
    assert command.front_drum_action is translator.DrumAction.STOP
    assert command.back_drum_action is translator.DrumAction.STOP


def test_create_command_with_auto_drive_routine_signal():
    """Should create a command that starts the auto-drive routine."""
    readings = DEFAULT_READINGS.copy()
    readings[translator.Signal.AUTO_DRIVE_ROUTINE] = True

    command = translator.create_command(
        lambda message, signal: readings[signal], None
    )

    assert command.routine_action is translator.RoutineAction.AUTO_DRIVE
    assert command.wheel_action.linear_x == 0.0
    assert command.wheel_action.angular_z == 0.0
    assert command.front_arm_action is translator.ArmAction.STOP
    assert command.back_arm_action is translator.ArmAction.STOP
    assert command.front_drum_action is translator.DrumAction.STOP
    assert command.back_drum_action is translator.DrumAction.STOP


def test_create_command_with_auto_dig_routine_signal():
    """Should create a command that starts the auto-dig routine."""
    readings = DEFAULT_READINGS.copy()
    readings[translator.Signal.AUTO_DIG_ROUTINE] = True

    command = translator.create_command(
        lambda message, signal: readings[signal], None
    )

    assert command.routine_action is translator.RoutineAction.AUTO_DIG
    assert command.wheel_action.linear_x == 0.0
    assert command.wheel_action.angular_z == 0.0
    assert command.front_arm_action is translator.ArmAction.STOP
    assert command.back_arm_action is translator.ArmAction.STOP
    assert command.front_drum_action is translator.DrumAction.STOP
    assert command.back_drum_action is translator.DrumAction.STOP


def test_create_command_with_auto_dump_routine_signal():
    """Should create a command that starts the auto-dump routine."""
    readings = DEFAULT_READINGS.copy()
    readings[translator.Signal.AUTO_DUMP_ROUTINE] = True

    command = translator.create_command(
        lambda message, signal: readings[signal], None
    )

    assert command.routine_action is translator.RoutineAction.AUTO_DUMP
    assert command.wheel_action.linear_x == 0.0
    assert command.wheel_action.angular_z == 0.0
    assert command.front_arm_action is translator.ArmAction.STOP
    assert command.back_arm_action is translator.ArmAction.STOP
    assert command.front_drum_action is translator.DrumAction.STOP
    assert command.back_drum_action is translator.DrumAction.STOP


def test_create_command_with_auto_dock_routine_signal():
    """Should create a command that starts the auto-dock routine."""
    readings = DEFAULT_READINGS.copy()
    readings[translator.Signal.AUTO_DOCK_ROUTINE] = True

    command = translator.create_command(
        lambda message, signal: readings[signal], None
    )

    assert command.routine_action is translator.RoutineAction.AUTO_DOCK
    assert command.wheel_action.linear_x == 0.0
    assert command.wheel_action.angular_z == 0.0
    assert command.front_arm_action is translator.ArmAction.STOP
    assert command.back_arm_action is translator.ArmAction.STOP
    assert command.front_drum_action is translator.DrumAction.STOP
    assert command.back_drum_action is translator.DrumAction.STOP


def test_create_command_with_full_autonomy_routine_signal():
    """Should create a command that enables full autonomy."""
    readings = DEFAULT_READINGS.copy()
    readings[translator.Signal.FULL_AUTONOMY_ROUTINE] = True

    command = translator.create_command(
        lambda message, signal: readings[signal], None
    )

    assert command.routine_action is translator.RoutineAction.FULL_AUTONOMY
    assert command.wheel_action.linear_x == 0.0
    assert command.wheel_action.angular_z == 0.0
    assert command.front_arm_action is translator.ArmAction.STOP
    assert command.back_arm_action is translator.ArmAction.STOP
    assert command.front_drum_action is translator.DrumAction.STOP
    assert command.back_drum_action is translator.DrumAction.STOP


def test_create_command_with_positive_left_and_right_wheel_signals():
    """Should create a command that moves the EZRASSOR forward."""
    readings = DEFAULT_READINGS.copy()
    readings[translator.Signal.LEFT_WHEEL] = 1.0
    readings[translator.Signal.RIGHT_WHEEL] = 1.0

    command = translator.create_command(
        lambda message, signal: readings[signal], None
    )

    assert command.routine_action is None
    assert command.wheel_action.linear_x == 1.0
    assert command.wheel_action.angular_z == 0.0
    assert command.front_arm_action is translator.ArmAction.STOP
    assert command.back_arm_action is translator.ArmAction.STOP
    assert command.front_drum_action is translator.DrumAction.STOP
    assert command.back_drum_action is translator.DrumAction.STOP


def test_create_command_with_negative_left_and_right_wheel_signals():
    """Should create a command that moves the EZRASSOR backward."""
    readings = DEFAULT_READINGS.copy()
    readings[translator.Signal.LEFT_WHEEL] = -1.0
    readings[translator.Signal.RIGHT_WHEEL] = -1.0

    command = translator.create_command(
        lambda message, signal: readings[signal], None
    )

    assert command.routine_action is None
    assert command.wheel_action.linear_x == -1.0
    assert command.wheel_action.angular_z == 0.0
    assert command.front_arm_action is translator.ArmAction.STOP
    assert command.back_arm_action is translator.ArmAction.STOP
    assert command.front_drum_action is translator.DrumAction.STOP
    assert command.back_drum_action is translator.DrumAction.STOP


def test_create_command_with_positive_left_wheel_and_negative_right_wheel_signals():
    """Should create a command that tank-turns the EZRASSOR to the right."""
    readings = DEFAULT_READINGS.copy()
    readings[translator.Signal.LEFT_WHEEL] = 1.0
    readings[translator.Signal.RIGHT_WHEEL] = -1.0

    command = translator.create_command(
        lambda message, signal: readings[signal], None
    )

    assert command.routine_action is None
    assert command.wheel_action.linear_x == 0.0
    assert command.wheel_action.angular_z == -1.0
    assert command.front_arm_action is translator.ArmAction.STOP
    assert command.back_arm_action is translator.ArmAction.STOP
    assert command.front_drum_action is translator.DrumAction.STOP
    assert command.back_drum_action is translator.DrumAction.STOP


def test_create_command_with_negative_left_wheel_and_positive_right_wheel_signals():
    """Should create a command that tank-turns the EZRASSOR to the left."""
    readings = DEFAULT_READINGS.copy()
    readings[translator.Signal.LEFT_WHEEL] = -1.0
    readings[translator.Signal.RIGHT_WHEEL] = 1.0

    command = translator.create_command(
        lambda message, signal: readings[signal], None
    )

    assert command.routine_action is None
    assert command.wheel_action.linear_x == 0.0
    assert command.wheel_action.angular_z == 1.0
    assert command.front_arm_action is translator.ArmAction.STOP
    assert command.back_arm_action is translator.ArmAction.STOP
    assert command.front_drum_action is translator.DrumAction.STOP
    assert command.back_drum_action is translator.DrumAction.STOP


def test_create_command_with_raise_front_arm_signal():
    """Should create a command that raises the front arm."""
    readings = DEFAULT_READINGS.copy()
    readings[translator.Signal.RAISE_FRONT_ARM] = True

    command = translator.create_command(
        lambda message, signal: readings[signal], None
    )

    assert command.routine_action is None
    assert command.wheel_action.linear_x == 0.0
    assert command.wheel_action.angular_z == 0.0
    assert command.front_arm_action is translator.ArmAction.RAISE
    assert command.back_arm_action is translator.ArmAction.STOP
    assert command.front_drum_action is translator.DrumAction.STOP
    assert command.back_drum_action is translator.DrumAction.STOP


def test_create_command_with_lower_front_arm_signal():
    """Should create a command that lowers the front arm."""
    readings = DEFAULT_READINGS.copy()
    readings[translator.Signal.LOWER_FRONT_ARM] = True

    command = translator.create_command(
        lambda message, signal: readings[signal], None
    )

    assert command.routine_action is None
    assert command.wheel_action.linear_x == 0.0
    assert command.wheel_action.angular_z == 0.0
    assert command.front_arm_action is translator.ArmAction.LOWER
    assert command.back_arm_action is translator.ArmAction.STOP
    assert command.front_drum_action is translator.DrumAction.STOP
    assert command.back_drum_action is translator.DrumAction.STOP


def test_create_command_with_raise_and_lower_front_arm_signals():
    """Should create a command that does nothing.

    The competing raise and lower signals for the front arm cancel each other
    out.
    """
    readings = DEFAULT_READINGS.copy()
    readings[translator.Signal.RAISE_FRONT_ARM] = True
    readings[translator.Signal.LOWER_FRONT_ARM] = True

    command = translator.create_command(
        lambda message, signal: readings[signal], None
    )

    assert command.routine_action is None
    assert command.wheel_action.linear_x == 0.0
    assert command.wheel_action.angular_z == 0.0
    assert command.front_arm_action is translator.ArmAction.STOP
    assert command.back_arm_action is translator.ArmAction.STOP
    assert command.front_drum_action is translator.DrumAction.STOP
    assert command.back_drum_action is translator.DrumAction.STOP


def test_create_command_with_raise_back_arm_signal():
    """Should create a command that raises the back arm."""
    readings = DEFAULT_READINGS.copy()
    readings[translator.Signal.RAISE_BACK_ARM] = True

    command = translator.create_command(
        lambda message, signal: readings[signal], None
    )

    assert command.routine_action is None
    assert command.wheel_action.linear_x == 0.0
    assert command.wheel_action.angular_z == 0.0
    assert command.front_arm_action is translator.ArmAction.STOP
    assert command.back_arm_action is translator.ArmAction.RAISE
    assert command.front_drum_action is translator.DrumAction.STOP
    assert command.back_drum_action is translator.DrumAction.STOP


def test_create_command_with_lower_back_arm_signal():
    """Should create a command that lowers the back arm."""
    readings = DEFAULT_READINGS.copy()
    readings[translator.Signal.LOWER_BACK_ARM] = True

    command = translator.create_command(
        lambda message, signal: readings[signal], None
    )

    assert command.routine_action is None
    assert command.wheel_action.linear_x == 0.0
    assert command.wheel_action.angular_z == 0.0
    assert command.front_arm_action is translator.ArmAction.STOP
    assert command.back_arm_action is translator.ArmAction.LOWER
    assert command.front_drum_action is translator.DrumAction.STOP
    assert command.back_drum_action is translator.DrumAction.STOP


def test_create_command_with_raise_and_lower_back_arm_signals():
    """Should create a command that does nothing.

    The competing raise and lower signals for the back arm cancel each other
    out.
    """
    readings = DEFAULT_READINGS.copy()
    readings[translator.Signal.RAISE_BACK_ARM] = True
    readings[translator.Signal.LOWER_BACK_ARM] = True

    command = translator.create_command(
        lambda message, signal: readings[signal], None
    )

    assert command.routine_action is None
    assert command.wheel_action.linear_x == 0.0
    assert command.wheel_action.angular_z == 0.0
    assert command.front_arm_action is translator.ArmAction.STOP
    assert command.back_arm_action is translator.ArmAction.STOP
    assert command.front_drum_action is translator.DrumAction.STOP
    assert command.back_drum_action is translator.DrumAction.STOP


def test_create_command_with_dig_front_drum_signal():
    """Should create a command that digs the front drum."""
    readings = DEFAULT_READINGS.copy()
    readings[translator.Signal.DIG_FRONT_DRUM] = True

    command = translator.create_command(
        lambda message, signal: readings[signal], None
    )

    assert command.routine_action is None
    assert command.wheel_action.linear_x == 0.0
    assert command.wheel_action.angular_z == 0.0
    assert command.front_arm_action is translator.ArmAction.STOP
    assert command.back_arm_action is translator.ArmAction.STOP
    assert command.front_drum_action is translator.DrumAction.DIG
    assert command.back_drum_action is translator.DrumAction.STOP


def test_create_command_with_dump_front_drum_signal():
    """Should create a command that dumps the front drum."""
    readings = DEFAULT_READINGS.copy()
    readings[translator.Signal.DUMP_FRONT_DRUM] = True

    command = translator.create_command(
        lambda message, signal: readings[signal], None
    )

    assert command.routine_action is None
    assert command.wheel_action.linear_x == 0.0
    assert command.wheel_action.angular_z == 0.0
    assert command.front_arm_action is translator.ArmAction.STOP
    assert command.back_arm_action is translator.ArmAction.STOP
    assert command.front_drum_action is translator.DrumAction.DUMP
    assert command.back_drum_action is translator.DrumAction.STOP


def test_create_command_with_dig_and_dump_front_drum_signals():
    """Should create a command that does nothing.

    The competing dig and dump signals for the front drum cancel each other out.
    """
    readings = DEFAULT_READINGS.copy()
    readings[translator.Signal.DIG_FRONT_DRUM] = True
    readings[translator.Signal.DUMP_FRONT_DRUM] = True

    command = translator.create_command(
        lambda message, signal: readings[signal], None
    )

    assert command.routine_action is None
    assert command.wheel_action.linear_x == 0.0
    assert command.wheel_action.angular_z == 0.0
    assert command.front_arm_action is translator.ArmAction.STOP
    assert command.back_arm_action is translator.ArmAction.STOP
    assert command.front_drum_action is translator.DrumAction.STOP
    assert command.back_drum_action is translator.DrumAction.STOP


def test_create_command_with_dig_back_drum_signal():
    """Should create a command that digs the back drum."""
    readings = DEFAULT_READINGS.copy()
    readings[translator.Signal.DIG_BACK_DRUM] = True

    command = translator.create_command(
        lambda message, signal: readings[signal], None
    )

    assert command.routine_action is None
    assert command.wheel_action.linear_x == 0.0
    assert command.wheel_action.angular_z == 0.0
    assert command.front_arm_action is translator.ArmAction.STOP
    assert command.back_arm_action is translator.ArmAction.STOP
    assert command.front_drum_action is translator.DrumAction.STOP
    assert command.back_drum_action is translator.DrumAction.DIG


def test_create_command_with_dump_back_drum_signal():
    """Should create a command that dumps the back drum."""
    readings = DEFAULT_READINGS.copy()
    readings[translator.Signal.DUMP_BACK_DRUM] = True

    command = translator.create_command(
        lambda message, signal: readings[signal], None
    )

    assert command.routine_action is None
    assert command.wheel_action.linear_x == 0.0
    assert command.wheel_action.angular_z == 0.0
    assert command.front_arm_action is translator.ArmAction.STOP
    assert command.back_arm_action is translator.ArmAction.STOP
    assert command.front_drum_action is translator.DrumAction.STOP
    assert command.back_drum_action is translator.DrumAction.DUMP


def test_create_command_with_dig_and_dump_back_drum_signals():
    """Should create a command that does nothing.

    The competing dig and dump signals for the back drum cancel each other out.
    """
    readings = DEFAULT_READINGS.copy()
    readings[translator.Signal.DIG_BACK_DRUM] = True
    readings[translator.Signal.DUMP_BACK_DRUM] = True

    command = translator.create_command(
        lambda message, signal: readings[signal], None
    )

    assert command.routine_action is None
    assert command.wheel_action.linear_x == 0.0
    assert command.wheel_action.angular_z == 0.0
    assert command.front_arm_action is translator.ArmAction.STOP
    assert command.back_arm_action is translator.ArmAction.STOP
    assert command.front_drum_action is translator.DrumAction.STOP
    assert command.back_drum_action is translator.DrumAction.STOP


def test_create_command_with_several_different_signals():
    """Should create a command that instructs the EZRASSOR to do many things.

    This test ensures that concurrently-active signals do not interfere with
    each other.
    """
    readings = DEFAULT_READINGS.copy()
    readings[translator.Signal.STOP_ROUTINE] = True
    readings[translator.Signal.LEFT_WHEEL] = -1.0
    readings[translator.Signal.RIGHT_WHEEL] = 1.0
    readings[translator.Signal.RAISE_FRONT_ARM] = True
    readings[translator.Signal.LOWER_BACK_ARM] = True
    readings[translator.Signal.DUMP_FRONT_DRUM] = True
    readings[translator.Signal.DIG_BACK_DRUM] = True

    command = translator.create_command(
        lambda message, signal: readings[signal], None
    )

    assert command.routine_action is translator.RoutineAction.STOP
    assert command.wheel_action.linear_x == 0.0
    assert command.wheel_action.angular_z == 1.0
    assert command.front_arm_action is translator.ArmAction.RAISE
    assert command.back_arm_action is translator.ArmAction.LOWER
    assert command.front_drum_action is translator.DrumAction.DUMP
    assert command.back_drum_action is translator.DrumAction.DIG
