#!/usr/bin/env python

import sys
import threading
import os
import time
import rospy
from std_msgs.msg import Empty
from std_msgs.msg import Int8
import Tkinter
import numpy as np
import time
from subprocess import call
from functools import partial
from sys import argv



class GuiGameA():

################################################################################
# constructor
################################################################################
  def __init__(self):

    self.root = Tkinter.Tk()
    self.root.attributes('-fullscreen',True)

    # self.q[0] (scalar) indicates the current group playing;
    # self.q[1] is a list indicating the question index that has not been
    # answered yet (this question is the current question)
    # First group playing assumed to be group 0
    self.q = [0]

    self.total_errors = 0
    self.time_start = time.time()
    self.dir_media = rospy.get_param('~dir_media', '')
    self.dir_scripts = rospy.get_param('~dir_scripts', '')
    self.rfid_java_exec_dir = rospy.get_param('~rfid_java_exec_dir', '')
    self.rfid_file = rospy.get_param('~rfid_file', '')

    # Read plain questions, choices, answers
    self.Q = rospy.get_param('~Q', '')
    self.C = rospy.get_param('~C', '')
    self.A = rospy.get_param('~A', '')



    if self.dir_media == '':
      print('[cultureid_games_N] dir_media not set; aborting')
      return

    if self.dir_scripts == '':
      print('[cultureid_games_N] dir_scripts not set; aborting')
      return

    if self.rfid_file == '':
      print('[cultureid_games_N] rfid_file not set; aborting')
      return

    if self.rfid_java_exec_dir == '':
      print('[cultureid_games_N] rfid_java_exec_dir not set; aborting')
      return
    else:
      self.reset_file(self.rfid_java_exec_dir+ '/' + self.rfid_file)

    # [Q]uestions, [C]hoices, and [A]nswers for game_a

    if self.Q == '':
      print('[cultureid_games_N] Q not set; aborting')
      return

    if self.C == '':
      print('[cultureid_games_N] C not set; aborting')
      return

    if self.A == '':
      print('[cultureid_games_N] A not set; aborting')
      return

    # No answers given yet, for all groups
    self.q.append([0] * len(self.Q))


    # Let's go
    self.init()

    # Seems that the mainloop should be placed here; otherwise throws error
    rospy.logwarn('root mainloop')
    self.root.mainloop()





################################################################################
  def init(self):
    rospy.logwarn('init')

    # clean window
    for frames in self.root.winfo_children():
      frames.destroy()

    # new canvas
    canvas = Tkinter.Canvas(self.root)
    canvas.configure(bg='red')
    canvas.pack(fill=Tkinter.BOTH,expand=True)

    # to frame panw sto opoio 8a einai ta koumpia
    frame = Tkinter.Frame(self.root,bg='grey')
    frame.place(relwidth=0.95,relheight=0.95,relx=0.025,rely=0.025)

    # ta koumpia tou para8urou
    buttonVec = []
    buttonText = []

    buttonText.append('START GAME')
    sg = Tkinter.Button(frame,text='???',fg='black',bg='white',command=self.select_group_init)
    buttonVec.append(sg)

    xNum = 1
    yNum = len(buttonVec)

    xEff = 1
    yEff = 1

    GP = 0.05

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
        thisY = yG/2+yy*yWithGuard

        buttonVec[counter].place(relx=thisX,rely=thisY,relheight=yB,relwidth=xB)
        buttonVec[counter].config(text=buttonText[counter])
        buttonVec[counter].update()

        thisWidth = buttonVec[counter].winfo_width()
        thisHeight = buttonVec[counter].winfo_height()
        buttonVec[counter].update()

        counter = counter+1




################################################################################
  def select_group_init(self):

    # clean window
    for frames in self.root.winfo_children():
      frames.destroy()

    # new canvas
    canvas = Tkinter.Canvas(self.root)
    canvas.configure(bg='red')
    canvas.pack(fill=Tkinter.BOTH,expand=True)

    # to frame panw sto opoio 8a einai ta koumpia
    frame = Tkinter.Frame(self.root,bg='grey')
    frame.place(relwidth=0.95,relheight=0.95,relx=0.025,rely=0.025)

    buttonVec = []
    buttonText = []

    for i in range(0,len(self.Q)):
      this_butt = Tkinter.Button(frame,text='???',fg='black',bg='white', activebackground="green", command=partial(self.game,i))
      buttonVec.append(this_butt)
      this_group_name = 'GROUP %d' %(i+1)
      buttonText.append(this_group_name)

    # Swapped order so that buttons are placed horizontally
    xNum,yNum = self.get_x_y_dims(len(buttonVec))

    xEff = 1.0
    yEff = 1.0

    GP = 0.05

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
          thisX = xG/2+xx*xWithGuard+1-xEff
          thisY = yG/2+yy*yWithGuard

          buttonVec[counter].place(relx=thisX,rely=thisY,relheight=yB,relwidth=xB)
          buttonVec[counter].config(text=buttonText[counter])
          buttonVec[counter].update()

          thisWidth = buttonVec[counter].winfo_width()
          thisHeight = buttonVec[counter].winfo_height()
          buttonVec[counter].update()

        counter = counter+1


################################################################################
  def group_buttons(self, highlight_group):

    # clean window
    for frames in self.root.winfo_children():
      frames.destroy()

    # new canvas
    canvas = Tkinter.Canvas(self.root)
    canvas.configure(bg='red')
    canvas.pack(fill=Tkinter.BOTH,expand=True)

    # to frame panw sto opoio 8a einai ta koumpia
    frame = Tkinter.Frame(self.root,bg='grey')
    frame.place(relwidth=0.95,relheight=0.95,relx=0.025,rely=0.025)

    buttonVec = []
    buttonText = []

    for i in range(0,len(self.Q)):
      if highlight_group == i:
        this_butt = Tkinter.Button(frame,text='???',fg='black',bg='green', command=partial(self.game,i))
      else:
        this_butt = Tkinter.Button(frame,text='???',fg='black',bg='white', command=partial(self.game,i))

      buttonVec.append(this_butt)
      this_group_name = 'GROUP %d' %(i+1)
      buttonText.append(this_group_name)

    # Swapped order so that buttons are placed horizontally
    yNum,xNum = self.get_x_y_dims(len(buttonVec))

    xEff = 0.1
    yEff = 0.05

    GP = 0.05

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
          thisX = xG/2+xx*xWithGuard+1-xEff
          thisY = yG/2+yy*yWithGuard

          buttonVec[counter].place(relx=thisX,rely=thisY,relheight=yB,relwidth=xB)
          buttonVec[counter].config(text=buttonText[counter])
          buttonVec[counter].update()

          thisWidth = buttonVec[counter].winfo_width()
          thisHeight = buttonVec[counter].winfo_height()
          buttonVec[counter].update()

        counter = counter+1

    return frame

################################################################################
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



################################################################################
  def game(self,group):
    self.q[0] = group
    rospy.logwarn('Current group: %d' % self.q[0])
    frame = self.group_buttons(self.q[0])

    if self.q[1][self.q[0]] < len(self.Q[self.q[0]]):
      self.continue_game(frame)
    else:
      self.game_over(self.q[0])



################################################################################
  def continue_game(self, frame):

    current_group = self.q[0]

    # The current unanswered question for this group
    current_q = self.q[1][current_group]

    # Show Q
    buttonVec = []
    buttonText = []

    QButton = Tkinter.Button(frame,text='???',fg='black',bg='white')
    buttonVec.append(QButton)
    buttonText.append(self.Q[current_group][current_q])

    xNum = 1
    yNum = len(buttonVec)

    xEff = 1.0
    yEff = 0.3

    GP = 0.05

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
        buttonVec[counter].update()

        counter = counter+1

    # ta koumpia tou para8urou apanthseis


    buttonVec = []
    buttonText = []

    # All the choices for this question
    choices = self.C[current_group][current_q]

    # The only correct answer
    correct_a = self.A[current_group][current_q]

    # Is this a button question or a rfid-card question?
    do_open_rfid_reader = False
    if isinstance(correct_a, str):
      do_open_rfid_reader = True



    # Show A
    for answer_txt,num in zip(choices,range(len(choices))):
      rospy.logwarn(answer_txt)
      rospy.logwarn(num)
      buttonText.append(answer_txt)
      this_butt = Tkinter.Button(frame,text='???',fg='black',bg='white',activebackground="green", command=partial(self.check_answer_given, num, correct_a, do_open_rfid_reader, frame))
      buttonVec.append(this_butt)

    xNum,yNum = self.get_x_y_dims(len(buttonVec))


    xEff = 1.0
    yEff = 0.6

    GP = 0.05

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
          buttonVec[counter].update()

        counter = counter+1






    # Check the answer given if the answer is of the rfid-card type
    if do_open_rfid_reader == True:
      rospy.logwarn('OPENING RFID READER')
      self.open_rfid_reader()

      rospy.sleep(1.0)

      is_answer_correct = False
      while is_answer_correct == False:
        val = self.check_rfid_answer_validity(correct_a)

        if val == -1:
          rospy.loginfo('-1')
        if val == True:
          rospy.loginfo('true')
          rospy.logwarn('SHUTTING DOWN RFID READER')
          self.close_rfid_reader()
          is_answer_correct = True
        if val == False:
          rospy.loginfo('false')
          self.total_errors = self.total_errors+1
          incorrect_answer(frame)

        # Flush rfid measurements file (less errors and faster gameplay)
        self.reset_file(self.rfid_java_exec_dir + '/' + self.rfid_file)
        rospy.sleep(1.0)

      self.correct_answer(frame)




################################################################################
  def check_answer_given(self, answer_given, correct_a, do_open_rfid_reader, frame):

    if do_open_rfid_reader == False:
      if answer_given == correct_a:
        self.correct_answer(frame)
      else:
        self.incorrect_answer(frame)



################################################################################
  def correct_answer(self, frame):

    # Update answered questions num
    self.q[1][self.q[0]] = self.q[1][self.q[0]] + 1
    rospy.logwarn('this answer is correct')

    if self.q[1][self.q[0]] < len(self.Q[self.q[0]]):
      self.game(self.q[0])
    else:
      self.game_over(self.q[0])

################################################################################
  def incorrect_answer(self, frame_in):
    rospy.logwarn('this answer is incorrect')

    # clean window
    for frames in self.root.winfo_children():
      frames.destroy()

    # new canvas
    canvas = Tkinter.Canvas(self.root)
    canvas.configure(bg='red')
    canvas.pack(fill=Tkinter.BOTH,expand=True)

    # increase total erros
    self.total_errors = self.total_errors+1

    # to frame panw sto opoio 8a einai ta koumpia
    frame = Tkinter.Frame(self.root,bg='grey')
    frame.place(relwidth=0.95,relheight=0.95,relx=0.025,rely=0.025)

    # ta koumpia tou para8urou
    buttonVec = []
    buttonText = []

    playButton = Tkinter.Button(frame,text='???',fg='black',bg='white', command=partial(self.game, self.q[0]))
    buttonVec.append(playButton)
    buttonText.append('wrong! haha loser')

    xNum = 1
    yNum = len(buttonVec)

    xEff = 1.0
    yEff = 1.0

    GP = 0.05

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
        thisY = yG/2+yy*yWithGuard

        buttonVec[counter].place(relx=thisX,rely=thisY,relheight=yB,relwidth=xB)
        buttonVec[counter].config(text=buttonText[counter])
        buttonVec[counter].update()

        thisWidth = buttonVec[counter].winfo_width()
        thisHeight = buttonVec[counter].winfo_height()
        buttonVec[counter].update()

        counter = counter+1

    return


################################################################################
  def game_over(self, group):
    return


  ############################################################################
  def open_rfid_reader(self):

    cmd = 'cd ' + self.rfid_java_exec_dir + ';' + './open_rfid_reader.sh'
    os.system(cmd)

  ############################################################################
  def close_rfid_reader(self):

    cmd = 'cd ' + self.rfid_java_exec_dir + ';' + './close_rfid_reader.sh'
    os.system(cmd)

  ############################################################################
  def check_rfid_answer_validity(self, correct_answer):

    # Get the measurements
    measurements = self.read_file(self.rfid_java_exec_dir + '/' + self.rfid_file)
    if len(measurements) > 0:
      epc_most_measurements, proportion_of_measurements = self.get_epc_with_most_measurements(measurements)

      self.reset_file(self.rfid_java_exec_dir + '/' + self.rfid_file)

      if (proportion_of_measurements > 0.5) and (epc_most_measurements == correct_answer):
        return True
      else:
        return False
    else:
      return -1

  ############################################################################
  def get_epc_with_most_measurements(self, measurements):

    unique_epcs = []
    unique_epc_counter = []

    for m in measurements:
      m = m.split(',')
      m = m[2]
      m = m[1:]

      if (m in unique_epcs) == False:
        unique_epcs.append(m)
        unique_epc_counter.append(0)

      if (m in unique_epcs) == True:
        indx = unique_epcs.index(m)
        unique_epc_counter[indx] = unique_epc_counter[indx] + 1

    return unique_epcs[np.argmax(unique_epc_counter)], np.max(unique_epc_counter) / len(measurements)



  ############################################################################
  def read_file(self, file_str):
    with open(file_str,'r') as f:
      lines  = f.readlines()
      f.close()
      return lines

  ############################################################################
  def reset_file(self, file_str):
    with open(file_str,'w') as f:
      f.close()



################################################################################
# main
################################################################################
if __name__ == '__main__':

  rospy.init_node('gui_game_a_node')

  try:
    gga = GuiGameA()
    rospy.spin()

  except rospy.ROSInterruptException:
    print("SHUTTING DOWN")
    pass
