#!/usr/bin/env python3
"""
Keyboard-based button controller for testing.
Press keys to simulate wireless controller button presses.

Key Mappings:
  u = UP (stand)
  d = DOWN (sit)
  s = START (start walking)
  x = SELECT (stop walking)
  a = A button (soft abort / damping)
  b = B button (kill / emergency stop)
  q = quit

Note: This node requires terminal input and must be run in a terminal
that supports raw keyboard input.
"""
import sys
import select
import termios
import tty
import rclpy
from rclpy.node import Node
from blind_locomotion.msg import Button


HELP_TEXT = """
╔══════════════════════════════════════════════════════╗
║           Keyboard Button Controller                 ║
╠══════════════════════════════════════════════════════╣
║  Key Mappings:                                       ║
║    u = STAND (up button)                             ║
║    d = SIT (down button)                             ║
║    e = EMERGENCY SIT (slow descent)                  ║
║    s = START WALKING (start button)                  ║
║    x = STOP WALKING (select button)                  ║
║    a = SOFT ABORT / DAMPING (A button)               ║
║    b = KILL / EMERGENCY STOP (B button)              ║
║    h = show this help                                ║
║    q = quit                                          ║
╠══════════════════════════════════════════════════════╣
║  Press a key to send button command...               ║
╚══════════════════════════════════════════════════════╝
"""


class KeyboardButtons(Node):
    def __init__(self):
        super().__init__('keyboard_buttons')
        
        # Parameters
        self.declare_parameter('verbose', True)
        self.verbose = self.get_parameter('verbose').get_parameter_value().bool_value
        
        # Publisher
        self.publisher = self.create_publisher(Button, 'buttons', 10)
        
        # Timer to check keyboard at 50Hz
        self.timer = self.create_timer(0.02, self.check_keyboard)
        
        # Terminal settings
        self.old_settings = None
        self.setup_terminal()
        
        print(HELP_TEXT)
        
    def setup_terminal(self):
        """Set terminal to raw mode for character-by-character input."""
        try:
            self.old_settings = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())
        except Exception as e:
            self.get_logger().error(f'Failed to setup terminal: {e}')
            self.get_logger().error('This node must be run in an interactive terminal')
            
    def restore_terminal(self):
        """Restore terminal to original settings."""
        if self.old_settings:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
            
    def check_keyboard(self):
        """Check for keyboard input and publish button messages."""
        if select.select([sys.stdin], [], [], 0)[0]:
            key = sys.stdin.read(1).lower()
            self.process_key(key)
            
    def process_key(self, key):
        """Process a key press and publish the corresponding button message."""
        msg = Button()
        action_name = None
        
        if key == 'u':
            msg.up = True
            action_name = 'STAND (up)'
        elif key == 'd':
            msg.down = True
            action_name = 'SIT (down)'
        elif key == 'e':
            msg.emergency_sit = True
            action_name = 'EMERGENCY SIT (slow descent)'
        elif key == 's':
            msg.start = True
            action_name = 'START WALKING'
        elif key == 'x':
            msg.select = True
            action_name = 'STOP WALKING (select)'
        elif key == 'a':
            msg.a = True
            action_name = 'SOFT ABORT / DAMPING (A)'
        elif key == 'b':
            msg.b = True
            action_name = 'KILL / EMERGENCY STOP (B)'
        elif key == 'h':
            print(HELP_TEXT)
            return
        elif key == 'q':
            self.get_logger().info('Quit requested, shutting down...')
            self.restore_terminal()
            raise SystemExit(0)
        else:
            # Unknown key, don't publish
            return
            
        # Publish the button message
        self.publisher.publish(msg)
        
        if self.verbose and action_name:
            self.get_logger().info(f'Button pressed: {action_name}')
            
    def destroy_node(self):
        """Clean up terminal settings on shutdown."""
        self.restore_terminal()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardButtons()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.restore_terminal()
        node.destroy_node()
        rclpy.shutdown()
        print('\nKeyboard controller shutdown complete.')


if __name__ == '__main__':
    main()
