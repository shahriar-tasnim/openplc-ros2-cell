#!/usr/bin/env python3

"""Modbus TCP to ROS 2 bridge for the simulated pick-and-place cell."""

from threading import Thread

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String

from pymodbus.datastore import (
    ModbusSequentialDataBlock,
    ModbusServerContext,
    ModbusSlaveContext,
)
from pymodbus.server import StartTcpServer


MODBUS_HOST = "0.0.0.0"
MODBUS_PORT = 1502


class ModbusBridgeNode(Node):
    """Exchange OpenPLC Modbus bits with ROS 2 topics."""

    def __init__(self):
        super().__init__("modbus_bridge")

        # ---------------------------------------------------
        # Modbus memory
        # ---------------------------------------------------

        self.modbus_store = ModbusSlaveContext(
            co=ModbusSequentialDataBlock(0, [False] * 64),
            di=ModbusSequentialDataBlock(0, [False] * 64),
            hr=ModbusSequentialDataBlock(0, [0] * 32),
            ir=ModbusSequentialDataBlock(0, [0] * 32),
            zero_mode=True,
        )

        self.modbus_context = ModbusServerContext(
            slaves=self.modbus_store,
            single=True,
        )

        # ---------------------------------------------------
        # OpenPLC command publishers
        # ---------------------------------------------------

        self.conveyor_pub = self.create_publisher(
            Bool,
            "/cell/conveyor_run",
            10,
        )

        self.robot_start_pub = self.create_publisher(
            Bool,
            "/cell/robot_start",
            10,
        )

        self.robot_reset_pub = self.create_publisher(
            Bool,
            "/cell/robot_reset",
            10,
        )

        self.move_home_pub = self.create_publisher(
            Bool,
            "/cell/move_home",
            10,
        )

        self.bridge_status_pub = self.create_publisher(
            String,
            "/cell/modbus_bridge_status",
            10,
        )

        # ---------------------------------------------------
        # Robot feedback subscriptions
        # ---------------------------------------------------

        self.robot_home = True
        self.robot_busy = False
        self.robot_done = False
        self.robot_fault = False

        self.create_subscription(
            Bool,
            "/cell/robot_home",
            self.robot_home_callback,
            10,
        )

        self.create_subscription(
            Bool,
            "/cell/robot_busy",
            self.robot_busy_callback,
            10,
        )

        self.create_subscription(
            Bool,
            "/cell/robot_done",
            self.robot_done_callback,
            10,
        )

        self.create_subscription(
            Bool,
            "/cell/robot_fault",
            self.robot_fault_callback,
            10,
        )

        # ---------------------------------------------------
        # Simulated operator and sensor inputs
        # ---------------------------------------------------

        self.start_pb = False
        self.stop_pb = False
        self.reset_pb = False

        self.auto_mode = True
        self.safety_ok = True
        self.part_at_pick = False
        self.place_area_clear = True

        self.create_subscription(
            Bool,
            "/cell/start_pb",
            self.start_callback,
            10,
        )

        self.create_subscription(
            Bool,
            "/cell/stop_pb",
            self.stop_callback,
            10,
        )

        self.create_subscription(
            Bool,
            "/cell/reset_pb",
            self.reset_callback,
            10,
        )

        self.create_subscription(
            Bool,
            "/cell/auto_mode",
            self.auto_mode_callback,
            10,
        )

        self.create_subscription(
            Bool,
            "/cell/safety_ok",
            self.safety_callback,
            10,
        )

        self.create_subscription(
            Bool,
            "/cell/part_at_pick",
            self.part_callback,
            10,
        )

        self.create_subscription(
            Bool,
            "/cell/place_area_clear",
            self.place_area_callback,
            10,
        )

        self.previous_commands = [False] * 8

        # Exchange data ten times per second.
        self.timer = self.create_timer(
            0.1,
            self.exchange_data,
        )

        # Start the blocking Modbus server in another thread.
        self.modbus_thread = Thread(
            target=self.start_modbus_server,
            daemon=True,
        )
        self.modbus_thread.start()

        self.get_logger().info(
            f"Modbus TCP bridge listening on "
            f"{MODBUS_HOST}:{MODBUS_PORT}"
        )

    # -------------------------------------------------------
    # Modbus server
    # -------------------------------------------------------

    def start_modbus_server(self):
        try:
            StartTcpServer(
                context=self.modbus_context,
                address=(MODBUS_HOST, MODBUS_PORT),
            )
        except Exception as exc:
            self.get_logger().error(
                f"Modbus server stopped: {exc}"
            )

    # -------------------------------------------------------
    # Main cyclic exchange
    # -------------------------------------------------------

    def exchange_data(self):
        # FC1 memory: Modbus coils written by OpenPLC.
        commands = self.modbus_store.getValues(
            1,
            0,
            count=8,
        )

        commands = [bool(value) for value in commands]

        conveyor_run = commands[0]
        robot_start = commands[1]
        robot_reset = commands[2]
        move_home = commands[3]

        self.publish_bool(
            self.conveyor_pub,
            conveyor_run,
        )
        self.publish_bool(
            self.robot_start_pub,
            robot_start,
        )
        self.publish_bool(
            self.robot_reset_pub,
            robot_reset,
        )
        self.publish_bool(
            self.move_home_pub,
            move_home,
        )

        # PLC input mapping:
        # 0  StartPB
        # 1  StopPB
        # 2  ResetPB
        # 3  AutoMode
        # 4  SafetyOK
        # 5  PartAtPick
        # 6  PlaceAreaClear
        # 7  CommunicationOK
        # 8  RobotHome
        # 9  RobotBusy
        # 10 RobotDone
        # 11 RobotFault

        feedback = [
            self.start_pb,
            self.stop_pb,
            self.reset_pb,
            self.auto_mode,
            self.safety_ok,
            self.part_at_pick,
            self.place_area_clear,
            True,  # CommunicationOK while bridge is running
            self.robot_home,
            self.robot_busy,
            self.robot_done,
            self.robot_fault,
        ]

        # FC2 memory: discrete inputs read by OpenPLC.
        self.modbus_store.setValues(
            2,
            0,
            feedback,
        )

        if commands != self.previous_commands:
            status = (
                f"coils={list(map(int, commands))} | "
                f"home={int(self.robot_home)} "
                f"busy={int(self.robot_busy)} "
                f"done={int(self.robot_done)} "
                f"fault={int(self.robot_fault)}"
            )

            self.get_logger().info(status)

            message = String()
            message.data = status
            self.bridge_status_pub.publish(message)

            self.previous_commands = commands

    @staticmethod
    def publish_bool(publisher, value):
        message = Bool()
        message.data = bool(value)
        publisher.publish(message)

    # -------------------------------------------------------
    # Robot feedback callbacks
    # -------------------------------------------------------

    def robot_home_callback(self, message):
        self.robot_home = message.data

    def robot_busy_callback(self, message):
        self.robot_busy = message.data

    def robot_done_callback(self, message):
        self.robot_done = message.data

    def robot_fault_callback(self, message):
        self.robot_fault = message.data

    # -------------------------------------------------------
    # Operator and sensor callbacks
    # -------------------------------------------------------

    def start_callback(self, message):
        self.start_pb = message.data

    def stop_callback(self, message):
        self.stop_pb = message.data

    def reset_callback(self, message):
        self.reset_pb = message.data

    def auto_mode_callback(self, message):
        self.auto_mode = message.data

    def safety_callback(self, message):
        self.safety_ok = message.data

    def part_callback(self, message):
        self.part_at_pick = message.data

    def place_area_callback(self, message):
        self.place_area_clear = message.data


def main(args=None):
    rclpy.init(args=args)
    node = ModbusBridgeNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
