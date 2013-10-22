
#!/usr/bin/python

# The MIT License
#
# Copyright (c) 2013 Johannes Mayr
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

from __future__ import with_statement

"""
wheeled_robin serial control Interface (WSCI) is based on Roomba SCI and Create OI and implements a
subset of the opcodes specified there.

wheeled_robin_driver.py uses the SerialCommandInterface of create_driver from ROS.
The Wheeled_Robin class is adopted from the Roomba class in create_driver.
"""
__author__ = "joh.mayr@jku.at (Johannes Mayr)"

import struct
import time
from create_driver import SerialCommandInterface, DriverError

WHEELED_ROBIN_OPCODES = dict(
    start = 128,
    safe = 131,
    full = 132,
    drive = 137,
    leds = 139,
    sensors = 142,
    direct_drive = 145,
    digital_outputs = 147,
    stream = 148,
    query_list = 149,
    pause_resume_stream = 150,
    )

WHEELED_ROBIN_MODES = (
    'off',
    'passive',
    'safe',
    'full')

SENSOR_GROUP_PACKET_LENGTHS = {
    0: 28,
    1: 18,
    7: 2,
    8: 2,
    9: 2,
    10: 2,
    11: 2,
    12: 2,
    13: 2,
    14: 2,
    15: 1,
    16: 2,
    17: 2,
    18: 2,
    19: 2,
    20: 2,
    21: 2,
    22: 1,
    }

# drive constants.
VELOCITY_X_MAX = 500 # mm/s
VELOCITY_X_SLOW = int(VELOCITY_X_MAX * 0.33)
VELOCITY_X_FAST = int(VELOCITY_X_MAX * 0.66)

VELOCITY_G_MAX = 500 # mm/s
VELOCITY_G_SLOW = int(VELOCITY_G_MAX * 0.33)
VELOCITY_G_FAST = int(VELOCITY_G_MAX * 0.66)

MAX_WHEEL_SPEED = 500
WHEEL_SEPARATION = 260 # mm

SERIAL_TIMEOUT = 2 # Number of seconds to wait for reads. 2 is generous.
START_DELAY = 5 # Time it takes the wheeled_robin to boot.

assert struct.calcsize('H') == 2, 'Expecting 2-byte shorts.'

class WheeledRobin(object):

  """Represents a wheeled_robin robot."""

  def __init__(self):
    self.tty = None
    self.sci = None
    self.safe = True

  def start(self, tty='/dev/ttyUSB0'): 
    self.tty = tty
    self.sci = SerialCommandInterface(tty, 38400)# The real baudrate has do be defined with setserial (spd_cust and divisor)
    self.sci.add_opcodes(WHEELED_ROBIN_OPCODES)

  def passive(self):
    """Put the robot in passive mode."""
    self.sci.start()
    time.sleep(0.5)

  def control(self):
    """Start the robot's SCI interface and place it in safe mode."""
    self.passive()
    if not self.safe:
      self.sci.full()
    else:
      self.sci.safe()
# self.sci.control() # Also puts the Roomba in to safe mode. #is deprecated
    time.sleep(0.5)

  def direct_drive(self, velocity_left, velocity_right):
    # Mask integers to 2 bytes.
    vl = int(velocity_left) & 0xffff
    vr = int(velocity_right) & 0xffff
    self.sci.direct_drive(*struct.unpack('4B', struct.pack('>2H', vr, vl)))
    
  def drive(self, velocity_x, velocity_g):
    """controls wheeled_robins's drive wheels.

    NOTE(Johannes Mayr): This commands differs slightly from the SCI and the OI
    as the second argument is not the radius. Instead the angular velocity around 
    z is used as second argument.

    The wheeled_robin takes four data bytes, interpreted as two 16-bit signed values
    using two's complement. The first two bytes specify the average linear velocity
    of the drive wheels in millimeters per second (mm/s), with the high byte
    being sent first. The next two bytes specify the average angular velocity  in millirad per second (mrad/s) at
    which wheeled_robin will turn. 

    Also see drive_straight and turn_in_place convenience methods.
    """
    # Mask integers to 2 bytes.
    velocity_x = int(velocity_x) & 0xffff
    velocity_g = int(velocity_g) & 0xffff

    # Pack as shorts to get 2 x 2 byte integers. Unpack as 4 bytes to send.
    # TODO(damonkohler): The 4 unpacked bytes will just be repacked later,
    # that seems dumb to me.
    bytes = struct.unpack('4B', struct.pack('>2H', velocity_x, velocity_g))
    self.sci.drive(*bytes)

  def stop(self):
    """Set velocities to 0 to stop movement."""
    self.drive(0, 0)

  def slow_stop(self, velocity):
    """Slowly reduce the velocities to 0 to stop movement."""
    velocities = xrange(velocity, VELOCITY_X_SLOW, -25)
    if velocity < 0:
      velocities = xrange(velocity, -VELOCITY_X_SLOW, 25)
    for v in velocities:
      self.drive(v, 0)
      time.sleep(0.05)
    self.stop()

  def drive_straight(self, velocity):
    """drive in a straight line."""
    self.drive(velocity, 0)

  def turn_in_place(self, velocity):
    """Turn in place either clockwise or counter-clockwise."""
    self.drive(0, velocity)  
    
  def set_digital_outputs(self, value):
    """Enable or disable digital outputs."""
    self.sci.digital_outputs(value)
