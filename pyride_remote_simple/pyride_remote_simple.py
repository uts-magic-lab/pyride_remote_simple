#!/usr/bin/env python

import telnetlib
from ast import literal_eval

"""
Get a remote PyRIDE instance via PyRIDECommander,
no need to install anything, but you can't add
callbacks.

Author: Sammy Pfeiffer <Sammy.Pfeiffer at student.uts.edu.au>
Author: Benjamin Johnston <benjamin.johnston at uts.edu.au>
"""


class PyRIDERemoteInstance(object):
    def __init__(self, host, port=27005):
        """
        Create a remote PyRIDE instance on:
        :param str host: IP or hostname.
        :param int port: Port, defaults to 27005.
        """
        print("Connecting to PyRIDE on host " +
              str(host) + " and port " + str(port) + "...")
        self.prc = PyRIDECommander(host, port)
        self.PyRAIDModuleName = ''
        print("Getting remote instance...")
        self.get_remote_instance()
        print("Ready to make orders!")

    def send_raw_command(self, command):
        """
        Send a raw command to the remote PyRIDE instance.
        """
        return self.prc.send_command(command)

    def make_call(self, method_name):
        def func(*args, **kwargs):
            cmd = self.PyRAIDModuleName + "." + method_name + "("
            for arg in args:
                if type(arg) == str:
                    cmd += "'''" + str(arg) + "'''" + ", "
                else:
                    cmd += str(arg) + ", "
            for name, value in kwargs.items():
                cmd += str(name) + "=" + str(value) + ", "
            if cmd[-2:] == ", ":
                cmd = cmd[:-2]
            cmd += ")"
            # print("Sending command:\n  " + str(cmd))
            return self.prc.send_command(cmd)
        return func

    def get_doc(self, method_name):
        cmd = self.PyRAIDModuleName + "." + method_name + ".__doc__"
        return self.prc.send_command(cmd)

    def get_remote_instance(self):
        # Looks like:
        # ['PyPR2', '__builtins__',
        # '__doc__', '__name__',
        # '__package__', 'py_main']
        dir_list = self.prc.send_command('dir()')
        # To get if it is PyPR2, PyREEM, PyCthulu...
        robot_module_name = ''
        for module in dir_list:
            # This is not really robust...
            if module.startswith('Py'):
                robot_module_name = module
                break
        self.PyRAIDModuleName = robot_module_name

        method_names = self.prc.send_command(robot_module_name +
                                             '.__dict__.keys()')
        # Magic line to get all docs in once (single call each is too slow)
        getalldocs = """{k:%s.__dict__[k].__doc__ for k in %s.__dict__.keys()}""" % (
            self.PyRAIDModuleName, self.PyRAIDModuleName)
        docs_dict = self.prc.send_command(getalldocs)
        for method_name in method_names:
            if not method_name.startswith('__'):
                # print "Adding method: " + method_name
                self.__dict__[method_name] = self.make_call(method_name)
                self.__dict__[method_name].__doc__ = docs_dict[method_name]


class PyRIDECommander(object):
    """
    Send PyRIDE commands from a telnet client
    from any kind of python program and
    get back it's output in native
    python format.

    Author: Sammy Pfeiffer <Sammy.Pfeiffer at student.uts.edu.au>
    """
    prompt = '>>> '

    def __init__(self, hostname, port=27005):
        self.tn = telnetlib.Telnet(hostname, port)
        # So we clean up
        self.tn.read_until(self.prompt)
        self.last_output = None

    def send_command(self, command):
        """
        Send command, add \r\n if needed.

        :param str command: text command
        """
        # Fix needed \r\n in the end
        if not command.endswith('\r\n'):
            command += '\r\n'
        self.last_command = command[:-2]
        self.tn.write(command)
        tmp_output = self.tn.read_until(self.prompt)

        if len(tmp_output) > (len(command) + len(self.prompt)):
            self.last_output = tmp_output[len(command):-len(self.prompt)]
        else:
            self.last_output = None

        if self.last_output is not None:
            try:
                self.last_output = literal_eval(self.last_output)
            except SyntaxError:
                pass
        return self.last_output

    def get_last_output(self):
        return self.last_output

    def get_last_command(self):
        return self.last_command

    def close(self):
        self.tn.close()

    def __del__(self):
        self.tn.close()


if __name__ == '__main__':
    # Example:
    # from pyride_remote_simple import PyRIDERemoteInstance
    PyPR2 = PyRIDERemoteInstance('pr2')
    PyPR2.say('hello')
    PyPR2.moveHeadTo(0., 0.)
    PyPR2.getBatteryStatus()
    PyPR2.getArmJointPositions(True)
    PyPR2.send_raw_command("PyPR2.say('hello')")
    PyPR2.send_raw_command("dir()")
    # Expected output:
    # ['PyPR2', '__builtins__', '__doc__', '__name__', '__package__']

    PyNao = PyRIDERemoteInstance('nao')
    PyNao.say('hello')
    PyNao.send_raw_command("dir()")

    PyREEM = PyRIDERemoteInstance('reemh3-3c')
    PyREEM.getArmJointPositions(True)
    # Expected output:
    # {'arm_left_7_joint': 2.739275735495535e-05, 'arm_left_5_joint': -6.391643382822915e-05, 'arm_left_6_joint': 2.739275735495535e-05, 'arm_left_4_joint': 0.6112902034476808, 'arm_left_1_joint': -0.40065491868780706, 'arm_left_2_joint': 0.2500431928699627, 'arm_left_3_joint': -0.10061569672681259}
    # super hacky way of getting the robot description abusing send_raw_command
    PyREEM.send_raw_command("import rospy;rospy.init_node('hackytest', anonymous=True);robot_desc=rospy.get_param('/robot_description');rospy.signal_shutdown('End hacky test');print(robot_desc)")