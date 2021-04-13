"""Execute a ROS node using the ezrassor_joy_translator module.

This node reads messages from a joy topic and translates those messages into
commands for the EZRASSOR. These commands are made up of different actions that
control various aspects of the robot.

The node processes joy messages using a pre-loaded controller config (because
different types of controllers have different joy message mappings).
"""
import ezrassor_joy_translator as translator
import functools
import geometry_msgs.msg
import rclpy
import sensor_msgs.msg
import std_msgs.msg


NODE = "joy_translator"
WHEEL_ACTIONS_TOPIC = "wheel_actions"
FRONT_ARM_ACTIONS_TOPIC = "front_arm_actions"
BACK_ARM_ACTIONS_TOPIC = "back_arm_actions"
FRONT_DRUM_ACTIONS_TOPIC = "front_drum_actions"
BACK_DRUM_ACTIONS_TOPIC = "back_drum_actions"
ROUTINE_ACTIONS_TOPIC = "routine_actions"
JOY_TOPIC = "joy"
QUEUE_SIZE = 10
SUBPARAMETER_FORMAT = "{0}.{1}"


def main(passed_args=None):
    """Main entry point for the ROS node."""
    try:
        rclpy.init(args=passed_args)
        node = rclpy.create_node(NODE)

        # Declare "type", "index", and "accept" parameters for each signal.
        # These parameters are usually loaded from files in the config/
        # directory.
        for signal in translator.Signal:
            node.declare_parameter(
                SUBPARAMETER_FORMAT.format(signal.value, translator.TYPE_KEY),
            )
            node.declare_parameter(
                SUBPARAMETER_FORMAT.format(signal.value, translator.INDEX_KEY),
            )
            node.declare_parameter(
                SUBPARAMETER_FORMAT.format(signal.value, translator.ACCEPT_KEY),
            )

        # Convert all available parameters into a config map, then verify that
        # the map is valid.
        config = {}
        for signal in translator.Signal:
            signal_type = node.get_parameter(
                SUBPARAMETER_FORMAT.format(signal.value, translator.TYPE_KEY),
            ).value
            signal_index = node.get_parameter(
                SUBPARAMETER_FORMAT.format(signal.value, translator.INDEX_KEY),
            ).value
            signal_accept = node.get_parameter(
                SUBPARAMETER_FORMAT.format(signal.value, translator.ACCEPT_KEY),
            ).value
            if (
                signal_type is not None
                or signal_index is not None
                or signal_accept is not None
            ):
                config[signal] = {}
                if signal_type is not None:
                    config[signal][translator.TYPE_KEY] = signal_type
                if signal_index is not None:
                    config[signal][translator.INDEX_KEY] = signal_index
                if signal_accept is not None:
                    config[signal][translator.ACCEPT_KEY] = signal_accept
        translator.verify_config(config)

        # Create publishers for each type of action.
        wheel_actions_publisher = node.create_publisher(
            geometry_msgs.msg.Twist,
            WHEEL_ACTIONS_TOPIC,
            QUEUE_SIZE,
        )
        front_arm_actions_publisher = node.create_publisher(
            std_msgs.msg.Float32,
            FRONT_ARM_ACTIONS_TOPIC,
            QUEUE_SIZE,
        )
        back_arm_actions_publisher = node.create_publisher(
            std_msgs.msg.Float32,
            BACK_ARM_ACTIONS_TOPIC,
            QUEUE_SIZE,
        )
        back_front_actions_publisher = node.create_publisher(
            std_msgs.msg.Float32,
            FRONT_DRUM_ACTIONS_TOPIC,
            QUEUE_SIZE,
        )
        back_drum_actions_publisher = node.create_publisher(
            std_msgs.msg.Float32,
            BACK_DRUM_ACTIONS_TOPIC,
            QUEUE_SIZE,
        )
        routine_actions_publisher = node.create_publisher(
            std_msgs.msg.Int8,
            ROUTINE_ACTIONS_TOPIC,
            QUEUE_SIZE,
        )

        def process_message(create_command, message):
            """Callback to create and process a command from a message."""
            command = create_command(message)

            if command.wheel_action is not None:
                wheel_action = geometry_msgs.msg.Twist()
                wheel_action.linear.x = command.wheel_action.linear_x
                wheel_action.angular.z = command.wheel_action.angular_z
                wheel_actions_publisher.publish(wheel_action)

            if command.front_arm_action is not None:
                front_arm_action = std_msgs.msg.Float32()
                front_arm_action.data = command.front_arm_action.value
                front_arm_actions_publisher.publish(front_arm_action)

            if command.back_arm_action is not None:
                back_arm_action = std_msgs.msg.Float32()
                back_arm_action.data = command.back_arm_action.value
                back_arm_actions_publisher.publish(back_arm_action)

            if command.front_drum_action is not None:
                front_drum_action = std_msgs.msg.Float32()
                front_drum_action.data = command.front_drum_action.value
                back_front_actions_publisher.publish(front_drum_action)

            if command.back_drum_action is not None:
                back_drum_action = std_msgs.msg.Float32()
                back_drum_action.data = command.back_drum_action.value
                back_drum_actions_publisher.publish(back_drum_action)

            if command.routine_action is not None:
                routine_action = std_msgs.msg.Int8()
                routine_action.data = command.routine_action.value
                routine_actions_publisher.publish(routine_action)

        # Subscribe to the joy topic and use the process_message() function to
        # handle messages. This function is pre-loaded with a create_command()
        # function (which is pre-loaded with a read_signal() function).
        node.create_subscription(
            sensor_msgs.msg.Joy,
            JOY_TOPIC,
            functools.partial(
                process_message,
                functools.partial(
                    translator.create_command,
                    functools.partial(
                        translator.read_signal,
                        config,
                    ),
                ),
            ),
            QUEUE_SIZE,
        )

        # Spin!
        rclpy.spin(node)
    except translator.VerificationError as error:
        node.get_logger().error(str(error))
        exit(1)
    except KeyboardInterrupt:
        pass
