# coding: utf-8
"""
@namespace maestro

Class library for interfacing the MiniMaestro Servo Controller.
Written using the documentation found here: http://www.pololu.com/docs/0J40/all#5.h.4

@author miannucci
Spring 2014

"""
import serial


class Maestro:
    # Command byte definitions (Private)
    _SET_TARGET_CMD           = 0x84
    _SET_MULTIPLE_TARGET_CMD  = 0x9F
    _SET_SPEED_CMD            = 0x87
    _SET_ACCELERATION_CMD     = 0x89
    _SET_PWM_CMD              = 0x8A
    _GET_POSITION_CMD         = 0x90
    _GET_MOVING_STATE_CMD     = 0x93
    _GET_ERRORS_CMD           = 0xA1
    _GO_HOME_CMD              = 0xA2
    _INIT_CMD                 = 0xAA
    
    # Motor Target Limit Definitions (Public)
    HOME_PULSE                = 6000
    MAX_FROM_HOME             = 2000
    MAX_FORWARD_SPEED         = HOME_PULSE + MAX_FROM_HOME
    MAX_REVERSE_SPEED         = HOME_PULSE - MAX_FROM_HOME
    SERVO_RANGE               = MAX_FROM_HOME * 2
    
    def __init__(self, port="/dev/ttyACM0", baudrate=9600):
        """Class constructor block"""
        self._commandPort = serial.Serial()
        if self._commandPort.isOpen():
            self._commandPort.close()
        try:
            self._commandPort.setPort(port)
            self._commandPort.setBaudrate(baudrate)
            self._commandPort.setTimeout(5)
            self._commandPort.open()
        except serial.serialutil.SerialException, e:
            print e

        # Send a quick initialization byte if the port was opened
        if self._commandPort.isOpen() and self._commandPort.writable():
            self._commandPort.write(chr(self._INIT_CMD))
            self._commandPort.flush()
            print "Command Port initialized successfully"

    def __del__(self):
        """Class destructor block"""
        if self._commandPort.isOpen():
            print "Closing command port"
            self._commandPort.close()

    def close(self):
        """Close the serial port"""
        if self._commandPort.isOpen():
            print 'Closing command port ', self._commandPort.getPort()
            self._commandPort.close()

    def write(self, *data):
        """ Write a message to the command port

        Handles writing the commands to the Maestro device

        """
        if not self._commandPort.isOpen() or not self._commandPort.writable():
            print "Serial port closed. Please open the serial port!"
            return 
        for message in data: 
            self._commandPort.write(chr(message))
        self._commandPort.flush()

    def setTarget(self, channel, target):
        """Sends a set target command to the controller

        Input Arguments:
        channel -- The channel to control by target (integer)
        target  -- The integer value to trasmit as pulse width in units of quarter-microseconds

        The target is a non-negative integer. If the channel is configured as a servo,
        then the target represents the pulse width to transmit in units of quarter-microseconds.
        A target value of 0 tells the Maestro to stop sending pulses to the servo. If the channel
        is configured as a digital output, values less than 6000 tell the Maestro to drive the line
        low, while values of 6000 or greater tell the Maestro to drive the line high.

        """
        outStr = [self._SET_TARGET_CMD, channel, self.LowBits(target), self.HighBits(target)]
        self.write(*outStr)

    def setTargetPercent(self, channel, percent):
        """Sends a set target command to the controller when given a percentage instead of an 
           explicit command

        Input Arguments:
        channel -- The channel to control by target (integer)
        percent -- The integer value from -100 to 100 to transmit as a pulse width 

        The target is a percentage value between 0 and 100. If the percentage is negative the 
        corresponding motion will be in the negative direction (ccw). This function abstracts the
        normal set target command. 

        """
        target = self.PercentToCommand(percent)
        self.setTarget(channel, target)


    def setMultipleTargets(self, numTargets, channel1, target1, channel2,
            target2, channel3=None, target3=None, channel4=None, target4=None):
        """Set multiple servo targets at one time

        Input Arguments:
        numChannels -- The number of channels to be changed with command (min=2, max=4)
        channel(i)  -- The channel to control by target (integer)
        target(i)   -- The integer value to trasmit as pulse width in units of (0.25 microseconds)

        This command simultaneously sets the targets for a contiguous block of channels.
        The first byte specifies how many channels are in the contiguous block; this is the number
        of target values you will need to send. The second byte specifies the lowest channel
        number in the block. The subsequent bytes contain the target values for each of the channels,
        in order by channel number, in the same format as the Set Target command above.

        """
        if (numTargets < 2) or (numTargets > 4):
            return "Invalid number of targets"
        outStr = [self._SET_MULTIPLE_TARGET_CMD, numTargets]
        outStr.append(channel1, self.LowBits(target1), self.HighBits(target1))
        outStr.append(channel2, self.LowBits(target2), self.HighBits(target2))
        if numTargets > 2:
            outStr.append(channel3, self.LowBits(target3), self.HighBits(target3))
        if numTargets > 3:
            outStr.append(channel4, self.LowBits(target4), self.HighBits(target4))
        self.write(*outStr)

    def setSpeed(self, channel, speed):
        """Sends the speed command to the maestro channel

        Input Arguments:
        channel -- The channel of which to set the speed
        speed   -- The speed limit of the rate of output value change in units of (0.25 microseconds)/(10 ms).

        For a speed value of 140, if you send a Set Target command to adjust the
        target from, say, 1000 microseconds to 1350 microseconds, it will take 100 ms to make that adjustment.
        A speed of 0 makes the speed unlimited, so that setting. the target will immediately
        affect the position. Note that the actual speed at which your servo moves is also limited
        by the design of the servo itself, the supply voltage, and mechanical loads;
        this parameter will not help your servo go faster than what it is physically capable of.

        """
        outStr = [self._SET_SPEED_CMD, channel, self.LowBits(speed), self.HighBits(speed)]
        self.write(*outStr)

    def setAcceleration(self, channel, acceleration):
        """Sets the acceleration limit of the servo channel

        Input Arguments:
        channel -- The channel of which to set the speed
        acceleration -- The acceleration limit of the channels
                        output in units of (0.25 microseconds)/(10 ms)/(80 ms)

        This command limits the acceleration of a servo channel’s output. The acceleration limit is a value
        from 0 to 255 in units of (0.25 microseconds)/(10 ms)/(80 ms), except in special cases. A value of 0 corresponds
        to no acceleration limit. An acceleration limit causes the speed of a servo to slowly ramp up until it
        reaches the maximum speed, then to ramp down again as position approaches target, resulting in a relatively
        smooth motion from one point to another. With acceleration and speed limits, only a few target settings are
        required to make natural-looking motions that would otherwise be quite complicated to produce.
        At the minimum acceleration setting of 1, the servo output takes about 3 seconds to move smoothly from a
        target of 1 ms to a target of 2 ms. The acceleration setting has no effect on channels configured
        as inputs or digital outputs.

        """
        outStr = [self._SET_ACCELERATION_CMD, channel, self.LowBits(acceleration), self.HighBits(acceleration)]
        self.write(*outStr)

    def setPWM(self, onTime, period):
        """Sets the PWM to the specified onTime and period

        Input Arguments:
        On Time -- On time value in units of 1/48 microseconds
        period  -- Period value in units of 1/48 microseconds

        This command sets the PWM output to the specified on time and period, in units of 1/48 microseconds.
        A period of 4800, for example, will generate a frequency of 10 kHz. The resolution on these
        values depends on the period as shown in the table below:

        Period range   Period resolution  On-time resolution
           1–1024             4                    1
          1025–4096          16                    4
          4097–16384         64                   16

        The special periods 1024, 4096, and 16384 are not recommended, since 100% duty cycle is not
        available at these values. If the on time is set equal to the period at one of the special values,
        the duty cycle will be 0% (a low output). The periods 1020 (47.1 kHz), 4080 (11.7 kHz), and 16320 (2.9 kHz)
        provide the best possible resolution with 100% and 0% duty cycle options, so you should use one of these
        periods if possible.

        """
        outStr = [self._SET_PWM_CMD, self.LowBits(onTime), self.HighBits(onTime), self.LowBits(period), self.HighBits(period)]
        self.write(*outStr)

    def getPosition(self, channel):
        """Returns the position of the given channel

        Input Arguments:
        channel -- The channel of which to get the position

        This command allows the device communicating with the Maestro to get the position value
        of a channel. If the specified channel is configured as a servo, this position value
        represents the current pulse width that the Maestro is transmitting on the channel,
        reflecting the effects of any previous commands, speed and acceleration limits, or scripts
        running on the Maestro. If the channel is configured as a digital output, a position value
        less than 6000 means the Maestro is driving the line low, while a position value of 6000
        or greater means the Maestro is driving the line high. If the channel is configured as an
        input, the position represents the voltage measured on the channel. The inputs on channels
        0–11 are analog: their values range from 0 to 1023, representing voltages from 0 to 5 V.
        The inputs on channels 12–23 are digital: their values are either exactly 0 or exactly 1023.

        NOTE: The position value returned by this command is equal to four times the number
              displayed in the Position box in the Status tab of the Maestro Control Center.

        """
        self.write(self._GET_POSITION_CMD, channel)
        # Get the return bytes
        lowbits = ord(self._commandPort.read(1))
        highbits = ord(self._commandPort.read(1))
        position = lowbits + (highbits << 8)
        return position

    def getMovingState(self):
        """Returns a 0 if no servos are moving, or a 1 if servos are moving

        This command is used to determine whether the servo outputs have reached their
        targets or are still changing, limited by speed or acceleration settings.
        Using this command together with the Set Target command, you can initiate several
        servo movements and wait for all the movements to finish before moving on to the
        next step of your program.

        """
        self.write(self._GET_MOVING_STATE_CMD)
        # Get the return byte
        return ord(self._commandPort.read(1))

    def goHome(self):
        """Returns all servos back to home position

        This command sends all servos and outputs to their home positions, just
        as if an error had occurred. For servos and outputs set to “Ignore”, the
        position will be unchanged.

        """
        self.write(self._GO_HOME_CMD)

    def getErrors(self):
        """Returns any errors that the Maestro has detected.

        Error Codes:
        Serial Signal Error (bit 0)          -- A hardware-level error that occurs when a byte’s stop bit is not detected at the expected place.
                                                This can occur if you are communicating at a baud rate that differs from the Maestro’s baud rate.
        Serial Overrun Error (bit 1)         -- A hardware-level error that occurs when the UART’s internal buffer fills up.
                                                This should not occur during normal operation.
        Serial RX buffer full (bit 2)        -- A firmware-level error that occurs when the firmware’s buffer for bytes received on the RX
                                                line is full and a byte from RX has been lost as a result. This error should not occur during
                                                normal operation.
        Serial CRC error (bit 3)             -- This error occurs when the Maestro is running in CRC-enabled mode and the cyclic redundancy check (CRC)
                                                byte at the end of the command packet does not match what the Maestro has computed as that packet’s CRC.
                                                In such a case, the Maestro ignores the command packet and generates a CRC error.
        Serial protocol error (bit 4)        -- This error occurs when the Maestro receives an incorrectly formatted or nonsensical command packet.
                                                For example, if the command byte does not match a known command or an unfinished command packet is
                                                interrupted by another command packet, this error occurs.
        Serial timeout error (bit 5)         -- When the serial timeout is enabled, this error occurs whenever the timeout period has elapsed without the
                                                Maestro receiving any valid serial commands. This timeout error can be used to make the servos return to
                                                their home positions in the event that serial communication between the Maestro and its controller is disrupted.
        Script stack error (bit 6)           -- This error occurs when a bug in the user script has caused the stack to overflow or underflow. Any script
                                                command that modifies the stack has the potential to cause this error. The stack depth is 32 on the Micro
                                                Maestro and 126 on the Mini Maestros.
        Script call stack error (bit 7)      -- This error occurs when a bug in the user script has caused the call stack to overflow or underflow.
                                                An overflow can occur if there are too many levels of nested subroutines, or a subroutine calls itself too
                                                many times. The call stack depth is 10 on the Micro Maestro and 126 on the Mini Maestros. An underflow can
                                                occur when there is a return without a corresponding subroutine call. An underflow will occur if you run a
                                                subroutine using the “Restart Script at Subroutine” serial command and the subroutine terminates with a return
                                                command rather than a quit command or an infinite loop.
        Script program counter error (bit 8) -- This error occurs when a bug in the user script has caused the program counter (the address of the next
                                                instruction to be executed) to go out of bounds. This can happen if your program is not terminated by a quit,
                                                return, or infinite loop.

        """
        self.write(self._GET_ERRORS_CMD)
        # Get the response bytes
        lowbits = ord(self._commandPort.read(1))
        highbits = ord(self._commandPort.read(1))
        return lowbits, highbits

    def LowBits(self, value):
        """Returns the lower 7 bits to be passed as a control sequence (bits 0-6)"""
        return value & 0x7F

    def HighBits(self, value):
        """Returns the upper 7 bits to be passed as a control sequence (bits 7-13)"""
        return (value >> 7) & 0x7F

    def PercentToCommand(self, percent):
        """Process the percent value given for the servo control

        Input Arguments:
        percent -- an integer value between -100 and 100.

        Returns:
        target  -- the equivalent value to send the servo a desired pulse-width

        """
        shift = (percent/100)*self.MAX_FROM_HOME
        return self.MAX_FROM_HOME + shift