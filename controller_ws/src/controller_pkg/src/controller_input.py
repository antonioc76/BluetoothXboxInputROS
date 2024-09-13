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

        self.timer_period = 0.1
        self.publisher = self.create_publisher(ControllerInput, 'autonav/controller_input', 10)
        
        self.controller = self.get_controller()
        self.stick_names = ["ABS_X", "ABS_Y", "ABS_Z", "ABS_RZ"]
        self.dpad_names = ["ABS_HAT0Y", "ABS_HAT0X"]
        self.button_names = ["BTN_NORTH", "BTN_EAST", "BTN_SOUTH", "BTN_WEST"]\
        
        self.controller_state = {
            "ABS_BRAKE": 0.0,
            "ABS_GAS": 0.0,
            "ABS_HAT0X": 0.0,
            "ABS_HAT0Y": 0.0,
            "ABS_RZ": 0.0,
            "ABS_X": 0.0,
            "ABS_Y": 0.0,
            "ABS_Z": 0.0,
            "BTN_EAST": 0.0,
            "BTN_MODE": 0.0,
            "BTN_NORTH": 0.0,
            "BTN_SELECT": 0.0,
            "BTN_START": 0.0,
            "BTN_SOUTH": 0.0,
            "BTN_TL": 0.0,
            "BTN_TR": 0.0,
            "BTN_WEST": 0.0,
            "KEY_RECORD": 0.0,
            "wildcard": 0.0
        }

        self.get_inputs()


    def get_controller(self):
        devices = [evdev.InputDevice(path) for path in evdev.list_devices()]
        controller = None

        for device in devices:
            if device.name == 'Xbox Wireless Controller':
                controller = evdev.InputDevice(device.path)
                self.get_logger().info(f"assigned controller: \nName: {device.name} \nPath: {device.path} \nBluetooth MAC address: {device.uniq}")
                self.get_logger().info(f"controller state will be published every {self.timer_period} seconds")
            
        return controller
    

    def normalize(self, x, min, max, lower_bound):
        return ((-lower_bound + 1) * (x - min) / (max - min) ) + lower_bound


    def get_inputs(self):
        last_callback_time_s = time.time()
        while True:
            last_callback_time_s = self.clock_routine(last_callback_time_s)

            try: # handle a disconnection
                event = self.controller.read_one()

                if event is None:
                    continue

                if event.type in ecodes.bytype:
                    codename = ecodes.bytype[event.type][event.code]
                else:
                    codename = "wildcard"
                
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

                matches = None

                # codenames for the controller right thumb buttons are an array of possible names, match for one of them
                if type(codename) == list:
                    matches = list(set(codename) & set(self.button_names))
                    if matches is not None:
                        button_name = self.xbox_swap_N_W(matches[0])
                    else:
                        button_name = "wildcard"
                else:
                    button_name = codename

                self.update_controller_state(button_name, value)

                msg = self.construct_controller_state_message()

                self.get_logger().info(f"publishing controller state:\n{str(self.controller_state)}")
                self.publisher.publish(msg)

            except OSError as e: # first disconnect
                #raise(e) # debug
                last_callback_time_s = self.reconnect(last_callback_time_s)

            except AttributeError as e: # after the controller InputDevice object is destroyed
                #raise(e) # debug 
                last_callback_time_s = self.reconnect(last_callback_time_s)

            else:
                pass
            

    def clock_routine(self, last_callback_time_s):
        current_time_s = time.time()
        time_delta_s = current_time_s - last_callback_time_s
        if time_delta_s > self.timer_period:
            self.controller_state_timer_callback()
            last_callback_time_s = time.time()

        return last_callback_time_s


    def construct_controller_state_message(self):
        message = ControllerInput()

        message.btn_north = self.controller_state["BTN_NORTH"]
        message.btn_east = self.controller_state["BTN_EAST"]
        message.btn_south = self.controller_state["BTN_SOUTH"]
        message.btn_west = self.controller_state["BTN_WEST"]
        message.btn_start = self.controller_state["BTN_START"]
        message.btn_select = self.controller_state["BTN_SELECT"]
        message.btn_mode = self.controller_state["BTN_MODE"]        
        message.btn_tr = self.controller_state["BTN_TR"]        
        message.btn_tl = self.controller_state["BTN_TL"]
        message.key_record = self.controller_state["KEY_RECORD"]
        message.abs_hat0x = self.controller_state["ABS_HAT0X"]   
        message.abs_hat0y = self.controller_state["ABS_HAT0Y"]           
        message.abs_x = self.controller_state["ABS_X"]           
        message.abs_y = self.controller_state["ABS_Y"]
        message.abs_z = self.controller_state["ABS_Z"]
        message.abs_rz = self.controller_state["ABS_RZ"]
        message.abs_gas = self.controller_state["ABS_GAS"]      
        message.abs_brake = self.controller_state["ABS_BRAKE"]
        message.wildcard = self.controller_state["wildcard"]
        
        return message


    def update_controller_state(self, key, value):
        self.controller_state[key] = value


    def reconnect(self, last_callback_time_s):
        self.controller_state = dict.fromkeys(self.controller_state, 0.0)
        last_callback_time_s = self.clock_routine(last_callback_time_s)

        time.sleep(self.timer_period)

        self.get_logger().info("attempting to reconnect...")
        self.controller = None
        self.controller = self.get_controller()

        if self.controller is not None:
            self.get_logger().info("reconnected!")
        
        return last_callback_time_s


    def controller_state_timer_callback(self):
        msg = self.construct_controller_state_message()
        # print(f"publishing: {str(self.controller_state)}")
        self.publisher.publish(msg)


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

