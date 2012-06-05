#! /usr/bin/env python
# -*- coding: utf-8 -*-
#
## This file may be used under the terms of the GNU General Public
## License version 2.0 as published by the Free Software Foundation
## and appearing in the file LICENSE included in the packaging of
## this file.
##
## This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
## WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
##

import shlex, subprocess
from threading import Thread
#from multiprocessing import Process as Thread, Event
from time import sleep


class WrapCommand (Thread):
    r"""Author : Yves-Gwenael Bourhis

    ==================================================
    Wrap a shell comand into a python threaded object.
    ==================================================

    Usage:
    ======

    You want to launch the following bash commands in a thread::

        [user@localhost ~]$ ls -l | grep pdf | wc -l
        5

    here is how you can do it::

        >>> Ls = WrapCommand( 'ls -l')
        >>> GrepPdf = WrapCommand( 'grep pdf')
        >>> Wc = WrapCommand( 'wc -l')
        >>> Wc.stdin = GrepPdf
        >>> GrepPdf.stdin = Ls
        >>> Wc.start( )
        >>> #Do stuff
        ...
        >>> Wc.join()
        >>> Wc.results
        ('5\n', '')

    the 'results' property is a tuple  (stdoutdata, stderrdata)

    You can also do it this way::

        >>> Ls = WrapCommand( 'ls -l | grep pdf | wc -l', shell=True)
        >>> Ls.start()
        >>> #Do stuff
        >>> Ls.join()
        >>> Ls.results[0]
        '5\n'

    You would need to specify 'shell=True' when the command
    you wish to execute is actually built into the shell.
    i.e.: on Windows if you use built in commands such as 'dir' or 'copy':
    http://docs.python.org/library/subprocess.html#subprocess.Popen

    The purpose of doing it in a thread is when the above commands may
    take a few hours, and that you want to perform other tasks in the
    meanwhile.
    You can check the process is still running with::

        >>> Wc.is_alive( )
        False

    'True' would be returned if still running.
    To terminate it prematurely (i.e. it deadlocked) you have the
    'terminate()', 'kill()' or 'send_signal(signal) methods which are
    self speaking.
    When you want to wait for the thread to end, use the 'join()' method:
    http://docs.python.org/library/threading.html#threading.Thread.join


    You want to launch the following bash commands without threading::

        [user@localhost ~]$ ls -l | grep pdf | wc -l
        5

    here is how you can do it::

        >>> Ls = WrapCommand( 'ls -l')
        >>> GrepPdf = WrapCommand( 'grep pdf')
        >>> Wc = WrapCommand( 'wc -l')
        >>> Wc(GrepPdf(Ls))
        '5\n'

    Avoid doing this for processes where a large amount of data is piped
    between each command.

    instead, do it this way::

        >>> Ls = WrapCommand( 'ls -l | grep pdf | wc -l', shell=True)
        >>> Ls()
        '5\n'

    Prefer the threaded method instead if this may take a long time and
    that you want to perform other tasks in the meanwhile.


    You can specify another shell for running commands::

        >>> Ls = WrapCommand( 'ls', shell=True, executable='C:/windows/System32/WindowsPowerShell/v1.0/powershell.exe')
        >>> print Ls()

            Directory : C:\Users\Yves\python_tests

        Mode                LastWriteTime     Length Name
        ----                -------------     ------ ----
        -a---        27/01/2011     00:14       7006 commandwrapper.py
        -a---        27/01/2011     00:15       7048 commandwrapper.pyc


    You can also use Context Management (with_item):
    http://docs.python.org/reference/compound_stmts.html#grammar-token-with_item

    example::

        >>> with WrapCommand( 'ls -l') as Ls:
        ...     with WrapCommand( 'grep pdf') as GrepPdf:
        ...         with WrapCommand( 'wc -l') as Wc:
        ...             Wc.stdin = GrepPdf
        ...             GrepPdf.stdin = Ls
        ...             Wc.start( )
        ...             #Do stuff
        ...             Wc.join()
        ...
        >>> Wc.results
        ('5\n', '')

    You may also simply want to have a subprocess objet::

        >>> ls = WrapCommand( 'ls -l')
        >>> lscmd = ls.makeCmd()
        >>>

    the returned object (`lscmd` in the example above) is a standard subprocess.Popen object
    """

    pid = None
    returncode = None
    results = [None, None]
    exc_type = None
    exc_value = None
    traceback = None
    sent_signal = None
    dont_auto_communicate = False

    finished_cb = None
    finished_cb_args = tuple()

    cwd = None
    stdin = None
    stdout = subprocess.PIPE
    stderr = subprocess.PIPE

    shell = False
    executable = None
    command = None
    commands = None

    def __init__(self, command, shell=False, executable=None,
                 stdin=None, stdout=subprocess.PIPE, stderr=subprocess.PIPE, cwd=None):
        self.__parent = super(WrapCommand, self)
        self.cmd = None
        self.shell = shell
        self.executable = executable
        self.command = command
        self.commands = shlex.split(command)
        name = self.commands[0]
        self.stdin = stdin
        self.stdout = stdout
        self.stderr = stderr
        self.cwd = cwd
        self.__parent.__init__(name=name)

    def prepareToRun(self, dont_auto_communicate=False):
        """
        returns the subprocess object::

            >>> Ls = WrapCommand( 'ls -l')
            >>> cmd = Ls.prepareToRun( )
            >>> result = cmd.communicate( )

        'cmd' is a subprocess.Popen object
        See http://docs.python.org/library/subprocess.html#subprocess.Popen

        'result' is a tuple (stdoutdata, stderrdata).
        See http://docs.python.org/library/subprocess.html#subprocess.Popen.communicate
        """
        self.dont_auto_communicate = dont_auto_communicate
        if isinstance(self.stdin, str):
            stdin = subprocess.PIPE
        elif isinstance(self.stdin, self.__class__):
            self.stdin.prepareToRun(True)
            stdin = self.stdin.cmd.stdout
        else:
            stdin = self.stdin
        if self.shell:
            command = self.command
        else:
            command = self.commands
        self.cmd = subprocess.Popen(command,
                                    stdin=stdin,
                                    stdout=self.stdout,
                                    stderr=self.stderr,
                                    shell=self.shell,
                                    executable=self.executable,
                                    cwd=self.cwd)

        print "!@#"*5,self.cwd

        self.pid = self.cmd.pid
        return self.cmd

    makeCmd = prepareToRun
    make_cmd = prepareToRun

    def __enter__(self):
        return self

    def __call__(self, stdin=None):
        self.stdin = stdin
        self.run()
        return self.results[0]

    def set_finished_cb(self, cb, *args):
        self.finished_cb = cb
        self.finished_cb_args = args

    def run(self):
        self.prepareToRun()
        if not self.dont_auto_communicate:
            self.results = self.cmd.communicate(self.stdin)
        self.returncode = self.cmd.returncode
        if self.finished_cb:
            self.finished_cb(self, *self.finished_cb_args)

    def send_signal(self, signal):
        if self.is_alive() and self.cmd is not None:
            self.cmd.send_signal(signal)
            self.sent_signal = signal

    def terminate(self):
        if self.is_alive() and self.cmd is not None:
            self.cmd.terminate()
            self.sent_signal = 'SIGTERM'
        if hasattr(self.__parent, 'terminate'):
            self.__parent.terminate()

    def kill(self):
        if self.is_alive() and self.cmd is not None:
            self.cmd.kill()
            self.sent_signal = 'SIGKILL'

    def stop(self):
        if self.is_alive()and self.cmd is not None:
            self.terminate()
            if self.is_alive() and self.cmd is not None:
                sleep(1)
            if self.is_alive() and self.cmd is not None:
                self.kill()

    def __exit__(self, exc_type=None, exc_value=None, traceback=None):
        if self.is_alive() and self.cmd is not None:
            self.stop()
        self.exc_type = exc_type
        self.exc_value = exc_value
        self.traceback = traceback


class WrapOnceCommand(WrapCommand):
    """
    WrapOnceCommand is the same as WrapCommand, but the cmd attribute
    which is a subprocess.Popen object will be created once and for all
    Therefore the run methode (or the object) can only be called once.
    The goal it to launch a command in a thread, and to have this
    command easily start/stopped from elsewhere.
    """
    def run(self):
        if self.cmd is None:
            self.prepareToRun()
        if not self.dont_auto_communicate:
            self.results = self.cmd.communicate(self.stdin)
        self.returncode = self.cmd.returncode
        if self.finished_cb:
            self.finished_cb(self, *self.finished_cb_args)

