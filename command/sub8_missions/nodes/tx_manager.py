#!/usr/bin/env python
import sys
import rospy
import txros
import curses
import psutil

from sub8 import tx_sub
import missions.mission_library as mission_library

from twisted.internet import defer, reactor


class sub8_curses():
    def __init__(self):
        self.main_scr = curses.initscr()

        curses.noecho()
        curses.cbreak()
        curses.curs_set(0)
        curses.start_color()
        curses.use_default_colors()

        self.height, self.width = self.main_scr.getmaxyx()

        self.main_scr.border(0)
        self.main_scr.addstr(0, 2, 'Sub8 Mission Manager', curses.A_BOLD)

        self.tleft_win = curses.newwin(self.height//2-1, (self.width//3)-1, 1, 1)
        self.tleft_win.box()
        self.tleft_win.addstr(0, 2, 'Available Missions:', curses.A_BOLD)

        self.bleft_win = curses.newwin((self.height//2)-1, (self.width//3)-1, self.height//2, 1)
        self.bleft_win.box()
        self.bleft_win.addstr(0, 2,'Mission Status:', curses.A_BOLD)

        self.tright_win = curses.newwin((self.height//3)-2, 2*(self.width//3)-2, 1, (self.width//3)+1)
        self.tright_win.box()
        self.tright_win.addstr(0, 2,'Sub8 Status:', curses.A_BOLD)

        self.mright_win =curses.newwin((self.height//3), 2*(self.width//3)-2, (self.height//3)-1, (self.width//3)+1)
        self.mright_win.box()
        self.mright_win.addstr(0, 2, 'Sub8 Odometry:', curses.A_BOLD)

        self.bright_win = curses.newwin((self.height//3), (2*self.width//3)-2, 2*(self.height//3)-1, (self.width//3)+1)
        self.bright_win.box()
        self.bright_win.addstr(0, 2,'Mission Status:', curses.A_BOLD)

    def refresh(self):
        self.main_scr.refresh()
        self.tleft_win.refresh()
        self.tright_win.refresh()
        self.mright_win.refresh()
        self.bleft_win.refresh()
        self.bright_win.refresh()


class sub8_hardware:
    def __init__(self):
        # CPU
        self.cpu_usage = psutil.cpu_percent(percpu=True)
        self.cpu_times = psutil.cpu_times()
        self.cpu_stats = psutil.cpu_stats()

        #Wprint 'cpu_usage: ', self.cpu_usage
        #print 'cpu_times: ', self.cpu_times
        #print 'cpu_stats: ', self.cpu_stats

        # Memory 
        self.memory = psutil.virtual_memory()
        self.total_memory = self.memory.total
        self.avail_memory = self.memory.available
        self.memory_perc = self.memory.percent

        # Disk Usage
        self.disk_util = psutil.disk_usage('/').percent

        # Networking
        # TODO: pernic = True will enumerate br0 & em1 network devices
        self.net_io = psutil.net_io_counters(pernic=false)

        # Misc. 
        self.users = psutil.user()

    '''
    def __str__(self):
        cpu_str = ''
        for core, percent in enumerate(self.cpu_percent):
            cpu_str = cpu_str + core +
        return 'CPU Cores: {self.cpu_cores}\n'.format(self=self) +
               'CPU Usage:\n' +
    '''


def check_ros_status():
    try:
        rosgraph.Master('/rostopic').getPid()
    except socket.error:
        raise ROSTopicIOException("Unable to communicate with master")

def get_bus_voltage():
    pass

def get_mission_satus():
    pass

def main():
    try:
        #sub8_hw = sub8_hardware()
        sub8_console = sub8_curses()
        sub8_console.refresh()
        sub8_console.main_scr.getch()
    finally:
        curses.endwin

if __name__=='__main__':
    main()
