"""Integration tests for the joy_translator using the launch_testing framework."""
import ament_index_python.packages as ament
import concurrent.futures
import geometry_msgs.msg
import launch
import launch.launch_description_sources as description_sources
import launch_testing
import rclpy
import sensor_msgs.msg
import std_msgs.msg
import time
import unittest


NODE = "test_joy_translator"
PACKAGE = "ezrassor_joy_translator"
JOY_TRANSLATOR_LAUNCH_FILE_FORMAT = "{0}/launch/joy_translator.py"
WHEEL_ACTIONS_TOPIC = "wheel_actions"
FRONT_ARM_ACTIONS_TOPIC = "front_arm_actions"
BACK_ARM_ACTIONS_TOPIC = "back_arm_actions"
FRONT_DRUM_ACTIONS_TOPIC = "front_drum_actions"
BACK_DRUM_ACTIONS_TOPIC = "back_drum_actions"
ROUTINE_ACTIONS_TOPIC = "routine_actions"
JOY_TOPIC = "joy"
QUEUE_SIZE = 10
LONG_CATCHUP_TIME = 1.0
SHORT_CATCHUP_TIME = 0.1
TIMEOUT = 2.0


# These pre-defined messages contain the axes (0) and buttons (1) that result
# in a specific command for the EZRASSOR (when using the xbox-360 controller
# config).
DRIVE_FORWARD_MESSAGE = (
    [0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
)
TURN_LEFT_MESSAGE = (
    [0.0, -1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
)
DRIVE_BACKWARD_MESSAGE = (
    [0.0, -1.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
)
TURN_RIGHT_MESSAGE = (
    [0.0, 1.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
)
STOP_DRIVING_MESSAGE = (
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
)
RAISE_FRONT_ARM_MESSAGE = (
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
)
LOWER_FRONT_ARM_MESSAGE = (
    [0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
)
STOP_FRONT_ARM_MESSAGE = (
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
)
RAISE_BACK_ARM_MESSAGE = (
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
)
LOWER_BACK_ARM_MESSAGE = (
    [0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
)
STOP_BACK_ARM_MESSAGE = (
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
)
DIG_FRONT_DRUM_MESSAGE = (
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
)
DUMP_FRONT_DRUM_MESSAGE = (
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
)
STOP_FRONT_DRUM_MESSAGE = (
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
)
DIG_BACK_DRUM_MESSAGE = (
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
)
DUMP_BACK_DRUM_MESSAGE = (
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
)
STOP_BACK_DRUM_MESSAGE = (
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
)
EXECUTE_AUTO_DRIVE_ROUTINE_MESSAGE = (
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
)
EXECUTE_AUTO_DIG_ROUTINE_MESSAGE = (
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
)
EXECUTE_AUTO_DUMP_ROUTINE_MESSAGE = (
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
)
EXECUTE_AUTO_DOCK_ROUTINE_MESSAGE = (
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
)
EXECUTE_FULL_AUTONOMY_ROUTINE_MESSAGE = (
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
)
STOP_ROUTINE_MESSAGE = (
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0],
)


def generate_test_description():
    """Create a test description with the joy_translator launch file."""
    joy_translator_launch_file = JOY_TRANSLATOR_LAUNCH_FILE_FORMAT.format(
        ament.get_package_share_directory(PACKAGE),
    )

    return launch.LaunchDescription(
        [
            launch.actions.IncludeLaunchDescription(
                description_sources.PythonLaunchDescriptionSource(
                    joy_translator_launch_file,
                ),
                launch_arguments={
                    "device": "/dev/fake-device",
                    "controller": "xbox-360-controller.yaml",
                }.items(),
            ),
            launch_testing.actions.ReadyToTest(),
        ]
    )


class JoyTranslatorIntegrationTests(unittest.TestCase):
    """A suite of integration tests for the joy_translator."""

    @classmethod
    def setUpClass(*arguments):
        """Initialize ROS before testing begins.

        This method name is required by unittest.
        """
        rclpy.init()

    def setUp(self):
        """Initialize testing infrastructure before each test.

        This method name is required by unittest.
        """
        self._node = rclpy.create_node(NODE)

        # Save the output of each joy_translator topic to a list.
        self._wheel_actions = []
        self._node.create_subscription(
            geometry_msgs.msg.Twist,
            WHEEL_ACTIONS_TOPIC,
            lambda message: self._wheel_actions.append(
                (message.linear.x, message.angular.z),
            ),
            QUEUE_SIZE,
        )
        self._front_arm_actions = []
        self._node.create_subscription(
            std_msgs.msg.Float32,
            FRONT_ARM_ACTIONS_TOPIC,
            lambda message: self._front_arm_actions.append(message.data),
            QUEUE_SIZE,
        )
        self._back_arm_actions = []
        self._node.create_subscription(
            std_msgs.msg.Float32,
            BACK_ARM_ACTIONS_TOPIC,
            lambda message: self._back_arm_actions.append(message.data),
            QUEUE_SIZE,
        )
        self._front_drum_actions = []
        self._node.create_subscription(
            std_msgs.msg.Float32,
            FRONT_DRUM_ACTIONS_TOPIC,
            lambda message: self._front_drum_actions.append(message.data),
            QUEUE_SIZE,
        )
        self._back_drum_actions = []
        self._node.create_subscription(
            std_msgs.msg.Float32,
            BACK_DRUM_ACTIONS_TOPIC,
            lambda message: self._back_drum_actions.append(message.data),
            QUEUE_SIZE,
        )
        self._routine_actions = []
        self._node.create_subscription(
            std_msgs.msg.Int8,
            ROUTINE_ACTIONS_TOPIC,
            lambda message: self._routine_actions.append(message.data),
            QUEUE_SIZE,
        )

        self._joy_publisher = self._node.create_publisher(
            sensor_msgs.msg.Joy,
            JOY_TOPIC,
            QUEUE_SIZE,
        )

        # Sleep for some time to give ROS a moment to warm up.
        time.sleep(LONG_CATCHUP_TIME)

    def tearDown(self):
        """Destroy testing infrastructure after each test.

        This method name is required by unittest.
        """
        self._node.destroy_node()

    @classmethod
    def tearDownClass(*arguments):
        """Shut down ROS after testing is complete.

        This method name is required by unittest.
        """
        rclpy.shutdown()

    def _publish(self, raw_message):
        """Publish a pre-defined message to the joy topic."""
        message = sensor_msgs.msg.Joy()
        message.axes, message.buttons = raw_message
        self._joy_publisher.publish(message)
        rclpy.spin_once(self._node, timeout_sec=TIMEOUT)
        time.sleep(SHORT_CATCHUP_TIME)

    def _spin_for_subscribers(self):
        """Spin the node for a brief period of time.

        This is necessary to give the testing subscribers enough spins to
        collect all required outputs.
        """
        with concurrent.futures.ThreadPoolExecutor(max_workers=1) as executor:
            future = executor.submit(time.sleep, LONG_CATCHUP_TIME)
            rclpy.spin_until_future_complete(
                self._node,
                future,
                timeout_sec=TIMEOUT,
            )

    def _verify_wheel_actions(self, expected_actions):
        """Verify the received wheel actions are as expected."""
        self.assertEqual(self._wheel_actions, expected_actions)

    def _verify_front_arm_actions(self, expected_actions):
        """Verify the received front arm actions are as expected."""
        self.assertEqual(self._front_arm_actions, expected_actions)

    def _verify_back_arm_actions(self, expected_actions):
        """Verify the received back arm actions are as expected."""
        self.assertEqual(self._back_arm_actions, expected_actions)

    def _verify_front_drum_actions(self, expected_actions):
        """Verify the received front drum actions are as expected."""
        self.assertEqual(self._front_drum_actions, expected_actions)

    def _verify_back_drum_actions(self, expected_actions):
        """Verify the received back drum actions are as expected."""
        self.assertEqual(self._back_drum_actions, expected_actions)

    def _verify_routine_actions(self, expected_actions):
        """Verify the received routine actions are as expected."""
        self.assertEqual(self._routine_actions, expected_actions)

    def test_joy_translator_produces_accurate_wheel_actions(self):
        """Should produce wheel actions when given wheel-related messages."""
        self._publish(DRIVE_FORWARD_MESSAGE)
        self._publish(TURN_LEFT_MESSAGE)
        self._publish(DRIVE_BACKWARD_MESSAGE)
        self._publish(TURN_RIGHT_MESSAGE)
        self._publish(STOP_DRIVING_MESSAGE)

        self._spin_for_subscribers()

        self._verify_wheel_actions(
            [(1.0, 0.0), (0.0, 1.0), (-1.0, 0.0), (0.0, -1.0), (0.0, 0.0)],
        )
        self._verify_front_arm_actions([0.0, 0.0, 0.0, 0.0, 0.0])
        self._verify_back_arm_actions([0.0, 0.0, 0.0, 0.0, 0.0])
        self._verify_front_drum_actions([0.0, 0.0, 0.0, 0.0, 0.0])
        self._verify_back_drum_actions([0.0, 0.0, 0.0, 0.0, 0.0])
        self._verify_routine_actions([])

    def test_joy_translator_produces_accurate_arm_actions(self):
        """Should produce arm actions when given arm-related messages."""
        self._publish(RAISE_FRONT_ARM_MESSAGE)
        self._publish(LOWER_FRONT_ARM_MESSAGE)
        self._publish(STOP_FRONT_ARM_MESSAGE)
        self._publish(RAISE_BACK_ARM_MESSAGE)
        self._publish(LOWER_BACK_ARM_MESSAGE)
        self._publish(STOP_BACK_ARM_MESSAGE)

        self._spin_for_subscribers()

        self._verify_wheel_actions(
            [
                (0.0, 0.0),
                (0.0, 0.0),
                (0.0, 0.0),
                (0.0, 0.0),
                (0.0, 0.0),
                (0.0, 0.0),
            ],
        )
        self._verify_front_arm_actions([1.0, -1.0, 0.0, 0.0, 0.0, 0.0])
        self._verify_back_arm_actions([0.0, 0.0, 0.0, 1.0, -1.0, 0.0])
        self._verify_front_drum_actions([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self._verify_back_drum_actions([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self._verify_routine_actions([])

    def test_joy_translator_produces_accurate_drum_actions(self):
        """Should produce drum actions when given drum-related messages."""
        self._publish(DIG_FRONT_DRUM_MESSAGE)
        self._publish(DUMP_FRONT_DRUM_MESSAGE)
        self._publish(STOP_FRONT_DRUM_MESSAGE)
        self._publish(DIG_BACK_DRUM_MESSAGE)
        self._publish(DUMP_BACK_DRUM_MESSAGE)
        self._publish(STOP_BACK_DRUM_MESSAGE)

        self._spin_for_subscribers()

        self._verify_wheel_actions(
            [
                (0.0, 0.0),
                (0.0, 0.0),
                (0.0, 0.0),
                (0.0, 0.0),
                (0.0, 0.0),
                (0.0, 0.0),
            ],
        )
        self._verify_front_arm_actions([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self._verify_back_arm_actions([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self._verify_front_drum_actions([1.0, -1.0, 0.0, 0.0, 0.0, 0.0])
        self._verify_back_drum_actions([0.0, 0.0, 0.0, 1.0, -1.0, 0.0])
        self._verify_routine_actions([])

    def test_joy_translator_produces_accurate_routine_actions(self):
        """Should produce routine actions when given routine-related messages."""
        self._publish(EXECUTE_AUTO_DRIVE_ROUTINE_MESSAGE)
        self._publish(EXECUTE_AUTO_DIG_ROUTINE_MESSAGE)
        self._publish(EXECUTE_AUTO_DUMP_ROUTINE_MESSAGE)
        self._publish(EXECUTE_AUTO_DOCK_ROUTINE_MESSAGE)
        self._publish(EXECUTE_FULL_AUTONOMY_ROUTINE_MESSAGE)
        self._publish(STOP_ROUTINE_MESSAGE)

        self._spin_for_subscribers()

        self._verify_wheel_actions(
            [
                (0.0, 0.0),
                (0.0, 0.0),
                (0.0, 0.0),
                (0.0, 0.0),
                (0.0, 0.0),
                (0.0, 0.0),
            ],
        )
        self._verify_front_arm_actions([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self._verify_back_arm_actions([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self._verify_front_drum_actions([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self._verify_back_drum_actions([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self._verify_routine_actions(
            [
                0b000001,
                0b000010,
                0b000100,
                0b001000,
                0b010000,
                0b100000,
            ],
        )
