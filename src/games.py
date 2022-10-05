#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import threading
import os
import numpy as np
import time
from subprocess import call, Popen
from functools import partial
import random
import Tkinter
import tkFont
from PIL import Image,ImageTk

# Import ROS modules
import rospy
import tf

# Sending goals etc
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseFeedback, MoveBaseResult

from std_msgs.msg import Empty
from std_srvs.srv import Empty as srv_empty
from std_msgs.msg import Int8
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped
from geometry_msgs.msg import Twist, Vector3

import roslib
roslib.load_manifest("rosparam")
import rosparam

# Class AMTHGame
import game


class AMTHGames():

  ##############################################################################
  # constructor
  ##############################################################################
  def __init__(self):

    self.root = Tkinter.Tk()
    self.root.attributes('-fullscreen',True)

    # Load params
    self.init_params()

    # Let's go: choose a quartet of games to begin with
    self.choose_game_screen()

    # Seems that the mainloop should be placed here; otherwise throws error
    rospy.logwarn('AMTHGAMES root mainloop')
    self.root.mainloop()


  ##############################################################################
  def choose_game_screen(self):

    # new canvas
    canvas = self.new_canvas()
    self.set_canvas(canvas)

    # to frame panw sto opoio 8a einai ta koumpia
    frame = self.new_frame()
    self.set_frame(frame)

    # All the choices for this question
    choices = self.quartet_titles

    # Show Q
    buttonVec = []
    buttonText = []

    QButton = Tkinter.Button(frame,text='???',fg='#E0B548',bg='#343A40',activeforeground='#E0B548',activebackground='#343A40', command=self.kill_root)
    buttonVec.append(QButton)

    # The text of the question
    q_text = 'ΕΠΙΛΕΞΑΤΕ ΣΕΤ ΠΑΙΧΝΙΔΙΩΝ'
    buttonText.append(q_text)

    xNum = len(buttonVec)
    yNum = 1


    xEff = 1.0
    yEff = 0.3
    GP = 0.1

    xWithGuard = xEff/xNum
    xG = GP*xWithGuard
    xB = xWithGuard-xG

    yWithGuard = yEff/yNum
    yG = GP*yWithGuard
    yB = yWithGuard-yG

    counter = 0
    for xx in range(xNum):
      for yy in range(yNum):
        thisX = xG/2+xx*xWithGuard
        thisY = yG/2+yy*yWithGuard+0.1

        buttonVec[counter].place(relx=thisX,rely=thisY,relheight=yB,relwidth=xB)
        buttonVec[counter].config(text=buttonText[counter])
        buttonVec[counter].update()

        thisWidth = buttonVec[counter].winfo_width()
        thisHeight = buttonVec[counter].winfo_height()

        buttonVec[counter].config(font=("Helvetica", 16))
        buttonVec[counter].update()

        counter = counter+1





    # ta koumpia tou para8urou apanthseis
    buttonVec = []
    buttonText = []



    # Show A
    for i in range(len(choices)):
      buttonText.append(choices[i])
      this_butt = Tkinter.Button(frame,text='???',fg='white',bg='#E0B548',activeforeground='white',activebackground='#E0B548',command=partial(self.let_the_games_begin, i))
      buttonVec.append(this_butt)

    xNum,yNum = self.get_x_y_dims(len(buttonVec))

    xEff = 1.0
    yEff = 0.5
    GP = 0.1

    xWithGuard = xEff/xNum
    xG = GP*xWithGuard
    xB = xWithGuard-xG

    yWithGuard = yEff/yNum
    yG = GP*yWithGuard
    yB = yWithGuard-yG

    counter = 0
    for xx in range(xNum):
      for yy in range(yNum):

        if counter < len(buttonVec):
          thisX = xG/2+xx*xWithGuard
          thisY = yG/2+yy*yWithGuard+1-yEff

          buttonVec[counter].place(relx=thisX,rely=thisY,relheight=yB,relwidth=xB)
          buttonVec[counter].config(text=buttonText[counter])
          buttonVec[counter].update()

          thisWidth = buttonVec[counter].winfo_width()
          thisHeight = buttonVec[counter].winfo_height()
          buttonVec[counter].config(font=("Helvetica", 20))
          buttonVec[counter].update()

        counter = counter+1


  ##############################################################################
  def get_x_y_dims(self,desiredNum):

    side1 = int(desiredNum/int(desiredNum**0.5))
    side2 = int(desiredNum/side1)

    sideSmall = np.min([side1,side2])
    sideLarge = np.max([side1,side2])

    if sideSmall*sideLarge < desiredNum:
      sideSmall = sideSmall+1

    side1 = np.min([sideSmall,sideLarge])
    side2 = np.max([sideSmall,sideLarge])

    return side1,side2


  ##############################################################################
  def init_params(self):

    self.pkg_name = rospy.get_param('~pkg_name', '')
    self.pkg_ap = rospy.get_param('~pkg_ap', '')
    self.quartet_titles = rospy.get_param('~quartet_titles', '')
    self.quartet_codes = rospy.get_param('~quartet_codes', '')


    if self.pkg_name == '':
      rospy.logerr('AMTHGames pkg_name not set; aborting')
      return

    if self.pkg_ap == '':
      rospy.logerr('AMTHGames pkg_ap not set; aborting')
      return

    if self.quartet_titles == '':
      rospy.logerr('[%s] quartet_titles not set; aborting', self.pkg_name)
      return

    if self.quartet_codes == '':
      rospy.logerr('[%s] quartet_codes not set; aborting', self.pkg_name)
      return


  ##############################################################################
  def kill_root(self):
    call(['bash', '/home/cultureid_user0/game_desktop_launchers/kill_all.sh'])
    self.root.destroy()
    rospy.logerr('GAMES OVER')
    os._exit(os.EX_OK)


  ##############################################################################
  def let_the_games_begin(self, q):

    rospy.loginfo('[%s] Starting game %s', self.pkg_name, self.quartet_codes[q])

    # Load params for this specific game
    rospy.loginfo('[%s] Loading params', self.pkg_name)
    self.load_game_params(self.quartet_codes[q])
    rospy.loginfo('[%s] Done loading params', self.pkg_name)

    # Play this specific game
    one_game = game.AMTHGame(self.root)

    rospy.logerr('[%s] Game %s over', self.pkg_name, self.quartet_codes[q])


  ##############################################################################
  def load_game_params(self, game_id):

    # Load these variables
    self.tl_variables = rospy.get_param('~tl_variables', '')

    # Load the variables inside these .yaml files
    self.tl_files = rospy.get_param('~tl_files', '')

    # Load the variables inside these .yaml files
    self.tl_dot_yaml = rospy.get_param('~tl_dot_yaml', '')

    if self.tl_variables == '':
      rospy.logerr('[%s] tl_variables not set; aborting', self.pkg_name)
      return

    if self.tl_files == '':
      rospy.logerr('[%s] tl_files not set; aborting', self.pkg_name)
      return

    if self.tl_dot_yaml == '':
      rospy.logerr('[%s] tl_dot_yaml not set; aborting', self.pkg_name)
      return

    ns = self.pkg_name + '_games/'


    for i in range(len(self.tl_variables)):
      rospy.set_param(\
          ns + self.tl_variables[i][0], \
          self.pkg_ap + self.tl_variables[i][1] + game_id)

    for i in range(len(self.tl_dot_yaml)):
      rospy.set_param(\
          ns + self.tl_dot_yaml[i][0], \
          self.pkg_ap + self.tl_dot_yaml[i][1] + game_id + '.yaml')

    for i in range(len(self.tl_files)):
      self.load_params_from_yaml(\
          self.pkg_ap + self.tl_files[i] + game_id + '.yaml', ns)




  ##############################################################################
  def load_params_from_yaml(self, file_path, myns):

    paramlist = \
        rosparam.load_file(file_path,default_namespace=myns)
    for params, ns in paramlist:
      rosparam.upload_params(ns,params)


  ##############################################################################
  def new_canvas(self):
    canvas = Tkinter.Canvas(self.root)
    canvas.configure(bg='white')
    canvas.pack(fill=Tkinter.BOTH,expand=True)
    self.canvas_ = canvas
    return canvas


  ##############################################################################
  def new_frame(self):

    # clean window
    for frames in self.root.winfo_children():
      frames.destroy()

    frame = Tkinter.Frame(self.root,bg='white')
    frame.place(relwidth=0.95,relheight=0.95,relx=0.025,rely=0.025)
    self.frame_ = frame

    return frame


  ##############################################################################
  def set_canvas(self, canvas):
    self.canvas_ = canvas


  ##############################################################################
  def set_frame(self, frame):
    self.frame_ = frame




################################################################################
# main
################################################################################
if __name__ == '__main__':

  rospy.init_node('amth_games_node')

  try:
    ggas = AMTHGames()
    rospy.spin()

  except rospy.ROSInterruptException:
    print("SHUTTING DOWN")
    pass
