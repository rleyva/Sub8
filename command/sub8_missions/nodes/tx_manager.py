import sys
import rospy
import txros
import curses

from sub8 import tx_sub
import missions

from twisted.internet import defer, reactor

status_scr = curses.initscr()
curses.noecho()
curses.cbreak()
curses.curs_set(0)
curses.start_color()
curses.use_default_colors()

try:
    status_scr.border(0)
    status_scr.addstr(0, 2, "Sub8 Mission Manager", curses.A_BOLD)

    left_win = curses.newwin(38, 30, 1, 1)
    left_win.addstr(1, 1, 'Available Missions:', curses.A_BOLD)
    left_win.box()

    right_win = curses.newwin(38, 50, 1, 31)
    right_win.addstr(1, 1, 'Mission Status:', curses.A_BOLD)
    right_win.box()

    status_scr.refresh()
    left_win.refresh()
    right_win.refresh()
    status_scr.getch()

finally:
    curses.endwin
