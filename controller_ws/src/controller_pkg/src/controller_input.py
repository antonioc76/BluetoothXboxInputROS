#!/usr/bin/env python3

import rclpy
import evdev
import time
from rclpy.node import Node
from evdev import ecodes
from std_msgs.msg import String
from autonav_msgs.msg import ControllerInput

JOY_MIN = 0
JOY_MAX = 65535

TRIGGER_MIN = 0
TRIGGER_MAX = 1023

# enable UserspaceHID=true in /etc/bluetooth/input.conf

class ControllerInputNode(Node):
    def __init__(self):
        super().__init__('controller_input')
        self.publisher = self.create_publisher(ControllerInput, 'autonav/controller_input', 10)
        self.controller = self.get_controller()

        self.stick_names = ["ABS_X", "ABS_Y", "ABS_Z", "ABS_RZ"]
        self.dpad_names = ["ABS_HAT0Y", "ABS_HAT0X"]
        self.button_names = ["BTN_NORTH", "BTN_EAST", "BTN_SOUTH", "BTN_WEST"]
        self.get_inputs()
        

    def get_controller(self):
        devices = [evdev.InputDevice(path) for path in evdev.list_devices()]
        controller = None

        for device in devices:
            if device.name == 'Xbox Wireless Controller':
                controller = evdev.InputDevice(device.path)
                self.get_logger().info(f"assigned controller: \nName: {device.name} \nPath: {device.path} \nBluetooth MAC address: {device.uniq}")
            
        return controller
    

    def normalize(self, x, min, max, lower_bound):
        return ((-lower_bound + 1) * (x - min) / (max - min) ) + lower_bound


    def get_inputs(self):
        while True:
            try: # handle a disconnection
                for event in self.controller.read_loop():
                    if event.type in ecodes.bytype:
                        codename = ecodes.bytype[event.type][event.code]
                    else:
                        codename = "?"
                    
                    value = 0.0

                    if ecodes.EV[event.type] == "EV_SYN": # we don't care about these
                        continue

                    if ecodes.EV[event.type] == "EV_MSC": # we don't care about these
                        continue

                    if ecodes.EV[event.type] == "EV_ABS": # joysticks, dpad, or triggers
                        if codename in self.stick_names:
                            value = self.normalize(event.value, JOY_MIN, JOY_MAX, -1)
                        elif codename in self.dpad_names:
                            value = float(event.value)
                        else: #trigger
                            value = self.normalize(event.value, TRIGGER_MIN, TRIGGER_MAX, 0)

                    elif ecodes.EV[event.type] == "EV_KEY": # buttons
                        value = float(event.value)
                    
                    msg = ControllerInput()

                    matches = None

                    # codenames for the controller right thumb buttons are an array of possible names, match for one of them
                    if type(codename) == list:
                        matches = list(set(codename) & set(self.button_names))
                        if matches is not None:
                            msg.button_name = self.xbox_swap_N_W(matches[0])
                        else:
                            msg.button_name = "?"
                    else:
                        msg.button_name = codename

                    msg.value = value

                    self.get_logger().info(f"button: {msg.button_name}, value: {msg.value}, type: {ecodes.EV[event.type]}")
                    self.publisher.publish(msg)

            except OSError: # first disconnect
                self.reconnect()

            except AttributeError: # after the controller InputDevice object is destroyed
                self.reconnect()

            else:
                pass


    def reconnect(self):
        time.sleep(2)
        self.get_logger().info("attempting to reconnect...")
        self.controller = None
        self.controller = self.get_controller()

        if self.controller is not None:
            self.get_logger().info("reconnected!")


    def xbox_swap_N_W(self, BTN_DIR):
        if BTN_DIR == "BTN_NORTH":
            return "BTN_WEST"
        elif BTN_DIR == "BTN_WEST":
            return "BTN_NORTH"
        else:
            return BTN_DIR
        

def main():
    rclpy.init()
    controller_input = ControllerInputNode()

    rclpy.spin(controller_input)

    controller_input.destroy_node() 
    rclpy.shutdown()


if __name__ == "__main__":
    main()

