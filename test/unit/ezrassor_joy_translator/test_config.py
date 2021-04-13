"""Test the config functions in this module."""
import copy
import ezrassor_joy_translator as translator
import sensor_msgs.msg


# This config map is first deep copied and then tweaked for each test.
VALID_CONFIG = {}
for index, signal in enumerate(translator.Signal):
    VALID_CONFIG[signal] = {}
    VALID_CONFIG[signal][translator.TYPE_KEY] = translator.BUTTON_TYPE
    VALID_CONFIG[signal][translator.INDEX_KEY] = index


def test_verify_config_with_valid_config():
    """Should do nothing because the configuration is valid."""
    config = copy.deepcopy(VALID_CONFIG)

    translator.verify_config(config)


def test_verify_config_with_missing_signal():
    """Should fail because a signal is missing."""
    config = copy.deepcopy(VALID_CONFIG)
    del config[translator.Signal.LEFT_WHEEL]

    try:
        translator.verify_config(config)
        assert False, "verify_config() should have failed"
    except Exception as error:
        assert isinstance(error, translator.VerificationError)
        assert "is missing from config" in error.message


def test_verify_config_with_signal_without_type():
    """Should fail because a signal does not have a type."""
    config = copy.deepcopy(VALID_CONFIG)
    del config[translator.Signal.LEFT_WHEEL][translator.TYPE_KEY]

    try:
        translator.verify_config(config)
        assert False, "verify_config() should have failed"
    except Exception as error:
        assert isinstance(error, translator.VerificationError)
        assert "must include a type" in error.message


def test_verify_config_with_signal_with_invalid_type():
    """Should fail because a signal's type is invalid."""
    config = copy.deepcopy(VALID_CONFIG)
    config[translator.Signal.LEFT_WHEEL][translator.TYPE_KEY] = "invalid"
    print(config)

    try:
        translator.verify_config(config)
        assert False, "verify_config() should have failed"
    except Exception as error:
        assert isinstance(error, translator.VerificationError)
        assert "type must be 'button' or 'axis'" in error.message


def test_verify_config_with_signal_without_index():
    """Should fail because a signal does not have an index."""
    config = copy.deepcopy(VALID_CONFIG)
    del config[translator.Signal.LEFT_WHEEL][translator.INDEX_KEY]

    try:
        translator.verify_config(config)
        assert False, "verify_config() should have failed"
    except Exception as error:
        assert isinstance(error, translator.VerificationError)
        assert "must include an index" in error.message


def test_verify_config_with_signal_with_non_integer_index():
    """Should fail because a signal's index is not an integer."""
    config = copy.deepcopy(VALID_CONFIG)
    config[translator.Signal.LEFT_WHEEL][translator.INDEX_KEY] = "invalid"

    try:
        translator.verify_config(config)
        assert False, "verify_config() should have failed"
    except Exception as error:
        assert isinstance(error, translator.VerificationError)
        assert "index must be an integer" in error.message


def test_verify_config_with_signal_with_negative_index():
    """Should fail because a signal's index is negative."""
    config = copy.deepcopy(VALID_CONFIG)
    config[translator.Signal.LEFT_WHEEL][translator.INDEX_KEY] = -1

    try:
        translator.verify_config(config)
        assert False, "verify_config() should have failed"
    except Exception as error:
        assert isinstance(error, translator.VerificationError)
        assert "index must be non-negative" in error.message


def test_verify_config_with_signal_with_non_float_accept():
    """Should fail because a signal's accept is not a float."""
    config = copy.deepcopy(VALID_CONFIG)
    config[translator.Signal.STOP_ROUTINE][
        translator.TYPE_KEY
    ] = translator.AXIS_TYPE
    config[translator.Signal.STOP_ROUTINE][translator.ACCEPT_KEY] = "invalid"

    try:
        translator.verify_config(config)
        assert False, "verify_config() should have failed"
    except Exception as error:
        assert isinstance(error, translator.VerificationError)
        assert "accept must be a float" in error.message


def test_read_signal_with_axis_signal():
    """Should return the float value of the axis since no accept is present."""
    config = {
        translator.Signal.STOP_ROUTINE: {
            translator.TYPE_KEY: translator.AXIS_TYPE,
            translator.INDEX_KEY: 0,
        },
    }
    message = sensor_msgs.msg.Joy()
    message.axes = [-1.0]

    value = translator.read_signal(
        config,
        message,
        translator.Signal.STOP_ROUTINE,
    )

    assert value == -1.0


def test_read_signal_with_axis_signal_at_accept_value():
    """Should return True since the axis value equals the accept value."""
    config = {
        translator.Signal.STOP_ROUTINE: {
            translator.TYPE_KEY: translator.AXIS_TYPE,
            translator.INDEX_KEY: 0,
            translator.ACCEPT_KEY: -1.0,
        },
    }
    message = sensor_msgs.msg.Joy()
    message.axes = [-1.0]

    value = translator.read_signal(
        config,
        message,
        translator.Signal.STOP_ROUTINE,
    )

    assert value is True


def test_read_signal_with_axis_signal_not_at_accept_value():
    """Should return False since the axis value does not equal the accept value."""
    config = {
        translator.Signal.STOP_ROUTINE: {
            translator.TYPE_KEY: translator.AXIS_TYPE,
            translator.INDEX_KEY: 0,
            translator.ACCEPT_KEY: -1.0,
        },
    }
    message = sensor_msgs.msg.Joy()
    message.axes = [1.0]

    value = translator.read_signal(
        config,
        message,
        translator.Signal.STOP_ROUTINE,
    )

    assert value is False


def test_read_signal_with_pressed_button_signal():
    """Should return True since the button is pressed."""
    config = {
        translator.Signal.STOP_ROUTINE: {
            translator.TYPE_KEY: translator.BUTTON_TYPE,
            translator.INDEX_KEY: 0,
        },
    }
    message = sensor_msgs.msg.Joy()
    message.buttons = [1]

    value = translator.read_signal(
        config,
        message,
        translator.Signal.STOP_ROUTINE,
    )

    assert value is True


def test_read_signal_with_unpressed_button_signal():
    """Should return False since the button is unpressed."""
    config = {
        translator.Signal.STOP_ROUTINE: {
            translator.TYPE_KEY: translator.BUTTON_TYPE,
            translator.INDEX_KEY: 0,
        },
    }
    message = sensor_msgs.msg.Joy()
    message.buttons = [1]

    value = translator.read_signal(
        config,
        message,
        translator.Signal.STOP_ROUTINE,
    )

    assert value is True
