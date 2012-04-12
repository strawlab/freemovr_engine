# -*- Mode: python; tab-width: 4; indent-tabs-mode: nil; indent-offset: 4 -*-

from __future__ import division
import serial
import glob
import time
import decimal


def _str2num(num, factor=10):
    """Takes a number in the supply's format, which always has one
    decimal place, and returns a decimal number reflecting it.
    """
    return decimal.Decimal(num) / factor


def _num2str(s, length=3, factor=10):
    """Turns a number, which may be a decimal, integer, or float,
    and turns it into a supply-formatted string.
    """
    dec = decimal.Decimal(s)
    return ('%0' + str(length) + 'i') % int(dec * factor)


def _numfields(s, fields, factors):
    """Generates numbers of the given lengths in the string.
    """
    assert len(fields) == len(factors)

    pos = 0
    for i in range(len(fields)):
        field = fields[i]
        factor = factors[i]
        yield _str2num(s[pos:pos + field], factor)
        pos += field

CONSTANT_CURRENT_MODE = "CC"
CONSTANT_VOLTAGE_MODE = "CV"


class Supply1696_1698(object):
    def __init__(self, ident="/dev/ttyUSB0", debug=False):
        self._ident = ident
        self._debug = debug
        try:
            self.ser = serial.Serial(self._ident, 9600, 8, 'N', 1, timeout=2.0)
        except:
            self.ser = None

        self._log("Opened %s (OK: %s)" % (self._ident, True if self.ser else False))

    def _log(self, s):
        if self._debug:
            print s

    def command(self, code, param='', address='00', readresp=True):
        if not (self.ser and self.ser.isOpen()):
            return None

        # Put this communication in an isolated little transaction.
        self.ser.flushInput()
        self.ser.flushOutput()

        self.ser.write(code + address + param + "\r")
        self.ser.flush()

        self._log("---> " + code + address + param)

        if not readresp:
            return None

        out = None
        while True:
            # Read until CR.
            resp = ''
            while True:
                try:
                    char = self.ser.read(1)
                except serial.SerialException:
                    char = ''
                finally:
                    if char == '':
                        self._log("---> !timeout")
                        self.command('ENDS', readresp=False)
                        self.command('ENDS', readresp=False)
                        resp = self.command(code, param, address)
                        break
                    else:
                        resp += char
                if char == '\r':
                    break

            if resp == 'OK\r':
                if out:
                    self._log("<--- " + out)
                return out

            if out is not None:
                self._log("---> !more than one line of output without OK")
                return resp

            out = resp

    def disable_front_panel(self):
        self.command('SESS')

    def enable_front_panel(self):
        self.command('ENDS')

    def voltage(self, val):
        self.command('VOLT', _num2str(val))

    def current(self, val):
        self.command('CURR', _num2str(val, factor=100))

    def reading(self):
        resp = self.command('GETD')
        volt, curr, mode = _numfields(resp, (4, 4, 1), (100, 1000, 1))
        return volt, curr, (CONSTANT_VOLTAGE_MODE, CONSTANT_CURRENT_MODE)[int(mode)]

    def maxima(self):
        resp = self.command('GMAX')
        return tuple(_numfields(resp, (3, 3), (10, 100)))

    def settings(self):
        resp = self.command('GETS')
        return tuple(_numfields(resp, (3, 3), (10, 100)))

    def enable_output(self):
        """Enable output."""
        self.command('SOUT', '0')

    def disable_output(self):
        """Enable output."""
        self.command('SOUT', '1')

    def set_output(self, enable):
        if enable:
            self.enable_output()
        else:
            self.disable_output()

    def open(self):
        if self.ser and not self.ser.isOpen():
            self.ser.open()

    def close(self):
        if self.ser and self.ser.isOpen():
            self.ser.close()
        self._log("Closed %s (OK: %s)" % (self._ident, not self.ser.isOpen() if self.ser else False))

    def __enter__(self):
        self.disable_front_panel()
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.enable_front_panel()
