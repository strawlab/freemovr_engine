# -*- Mode: python; tab-width: 4; indent-tabs-mode: nil; indent-offset: 4 -*-

import urllib
import time
import subprocess
import requests
import sys

INPUT_COMPUTER = "INPUT_COMPUTER"
INPUT_NONE = "NONE"


class ProjectorCommunicationError(Exception):
    pass


class ProjectorPJD6381:

    INPUTS = {
      INPUT_COMPUTER: 0
    }

    def __init__(self, host, username="admin", password="", debug=False):
        self._host = host
        self._username = username
        self._password = password
        self._debug = debug

    def _request_get(self, string, disable_debug=False, **params):
        url = string.format(host=self._host)
        r = requests.get(url, params=params, auth=(self._username, self._password))
        if self._debug and not disable_debug:
            print "-->", r.url
            print "<--", r.content
        if not r.status_code == 200:
            raise ProjectorCommunicationError("Invalid projector command: %s (response: %s)" % (r.url, r.status_code))
        return r

    def _power_smart(self, on):
        status = self._request_get("http://{host}/protect/status.htm", disable_debug=True).content
        if "Power Off" in status:
            if on:
                return self._request_get("http://{host}/protect/execPwr.cgi", PWRCHG=1)
        else:
            if not on:
                return self._request_get("http://{host}/protect/execPwr.cgi", PWRCHG=1)

    def _power(self, on):
        cmd = 2 if on else 3
        return self._request_get("http://{host}/protect/execPwr.cgi", PWRCHG=cmd)

    def _bright(self, increase):
        cmd = "+" if increase else "-"
        return self._request_get("http://{host}/protect/execBrig.cgi", BRIG=cmd)

    def _src(self, inputname):
        try:
            inputid = self.INPUTS[inputname]
        except KeyError:
            inputid = 0
        return self._request_get("http://{host}/protect/execSrc.cgi", SRCSEL=inputid)

    def _get_status(self):
        """ returns (power_on, input) """
        status = self._request_get("http://{host}/protect/status.htm", disable_debug=True).content
        ison = "Power On" in status
        iscomputer = INPUT_COMPUTER if "Computer1" in status else INPUT_NONE
        return ison, iscomputer

    def is_up(self):
        return subprocess.call('ping -q -c 1 %s > /dev/null' % self._host, shell=True) == 0

    def get_status(self):
        return self._get_status()

    def power_on(self):
        self._power(True)

    def power_off(self):
        self._power(False)

    def set_power(self, power):
        self._power(power)

    def get_power(self):
        return self._get_status()[0]

    def set_brightness(self, val):
        """ set brightness in range 0-100. Returns brightness """
        currentval = self.get_brightness()

        while currentval != val:
            increase = True if val > currentval else False
            currentval = int(self._bright(increase).content.strip())
            time.sleep(0.1)

        return currentval

    def get_brightness(self):
        return int(self._request_get("http://{host}/protect/execBrig.cgi").content.strip())

    def set_input(self, inputname):
        return self._src(inputname)

    def get_input(self):
        return self._get_status()[1]
