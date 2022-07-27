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

    # self.state[0] (scalar) indicates the current group playing;
    # self.state[1] is a list indicating the question index that has not been
    # answered yet (this question is the current question)
    # First group playing assumed to be group 0
    self.state = [0]

    # Correct answers [0], incorrect answers [1] and game duration [2] per group
    self.stats = [[],[],[]]

    self.dir_media = rospy.get_param('~dir_media', '')
    self.dir_scripts = rospy.get_param('~dir_scripts', '')
    self.rfid_java_exec_dir = rospy.get_param('~rfid_java_exec_dir', '')
    self.rfid_file = rospy.get_param('~rfid_file', '')

    # Read [Q]uestions, [C]hoices, correct [A]nswers
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
    self.state.append([0] * len(self.Q))

    # Correct answers, incorrect answers, and game duration per group
    self.stats[0] = [0] * len(self.Q)
    self.stats[1] = [0] * len(self.Q)
    self.stats[2] = [0] * len(self.Q)

    # Start the clock
    self.global_time_start = time.time()

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
# group buttons are displayed in each screen for switching between groups
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
      if i == highlight_group:
        this_butt = Tkinter.Button(frame,text='???',fg='black',bg='green', command=partial(self.game,i))
      else:
        this_butt = Tkinter.Button(frame,text='???',fg='black',bg='white', command=partial(self.game,i))

      buttonVec.append(this_butt)
      this_group_name = 'GROUP %d' %(i+1)
      buttonText.append(this_group_name)

    xNum = len(buttonVec)
    yNum = 1

    xEff = 0.25
    yEff = 0.075

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
  def game(self,group):
    self.state[0] = group
    rospy.logwarn('Current group: %d' % self.state[0])
    frame = self.group_buttons(self.state[0])

    if self.state[1][self.state[0]] < len(self.Q[self.state[0]]):
      self.continue_game(frame)
    else:
      self.game_over(self.state[0])



################################################################################
  def continue_game(self, frame):

    current_group = self.state[0]

    # The current unanswered question for this group
    current_q = self.state[1][current_group]

    # All the choices for this question
    choices = self.C[current_group][current_q]

    # The only correct answer
    correct_a = self.A[current_group][current_q]

    # Is this a button question or a rfid-card question?
    do_open_rfid_reader = False
    if isinstance(correct_a, str):
      do_open_rfid_reader = True



    # Show Q
    buttonVec = []
    buttonText = []

    # If this question requires a rfid card as an answer then press the question
    # button and bring the card to the reader;
    # otherwise no need to press the question button
    # The -1 is a workaround because the check_answer_given function is called
    # for all types of questions, but one may find oneself in the situation
    # where the question does not call for a rfid card but pressing the question
    # button opens up the reader, hanging execution
    if do_open_rfid_reader:
      QButton = Tkinter.Button(frame,text='???',fg='black',bg='white', command=partial(self.check_answer_given, -1, correct_a, do_open_rfid_reader))
    else:
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



    # Show A
    for answer_txt,num in zip(choices,range(len(choices))):
      rospy.logwarn(answer_txt)
      rospy.logwarn(num)
      buttonText.append(answer_txt)
      this_butt = Tkinter.Button(frame,text='???',fg='black',bg='white',activebackground="green", command=partial(self.check_answer_given, num, correct_a, do_open_rfid_reader))
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






################################################################################
  def check_answer_given(self, answer_given, correct_a, do_open_rfid_reader):

    if do_open_rfid_reader == False:
      if answer_given == correct_a:
        self.correct_answer()
      else:
        self.incorrect_answer()
    else:
      if answer_given == -1:
        rospy.logwarn('OPENING RFID READER')
        self.open_rfid_reader()

        rospy.sleep(1.0)

        is_answer_correct = False
        val = self.check_rfid_answer_validity(correct_a)

        if val == -1:
          rospy.loginfo('-1')
          self.insufficient_answer()
        if val == True:
          rospy.loginfo('true')
          self.correct_answer()
        if val == False:
          rospy.loginfo('false')
          self.incorrect_answer()

        # Shut down reader
        rospy.logwarn('SHUTTING DOWN RFID READER')
        self.close_rfid_reader()

        # Flush rfid measurements file (less errors and faster gameplay)
        self.reset_file(self.rfid_java_exec_dir + '/' + self.rfid_file)




################################################################################
  def correct_answer(self):

    # Update answered questions num
    self.state[1][self.state[0]] = self.state[1][self.state[0]] + 1
    rospy.logwarn('this answer is correct')

    # Increment correct answers counter for this group
    self.stats[0][self.state[0]] = self.stats[0][self.state[0]] + 1

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

    if self.state[1][self.state[0]] < len(self.Q[self.state[0]]):
      playButton = Tkinter.Button(frame,text='???',fg='black',bg='white',command=partial(self.game, self.state[0]))
      buttonVec.append(playButton)
      buttonText.append('CORRECT! YOU R THE BOMB')
    else:
      playButton = Tkinter.Button(frame,text='???',fg='black',bg='white', command=partial(self.game_over, self.state[0]))
      buttonVec.append(playButton)
      buttonText.append('CORRECT! YOU R THE BOMB')

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







################################################################################
  def incorrect_answer(self):
    rospy.logwarn('this answer is incorrect')

    # Increment incorrect answers counter for this group
    self.stats[1][self.state[0]] = self.stats[1][self.state[0]] + 1

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

    buttonText.append('WRONG! HAHA LOSER')
    playButton = Tkinter.Button(frame,text='???',fg='black',bg='white', command=partial(self.game, self.state[0]))
    buttonVec.append(playButton)

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



################################################################################
  def insufficient_answer(self):
    rospy.logwarn('this answer is insufficient')

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

    buttonText.append('BRING THE CARD CLOSER')
    playButton = Tkinter.Button(frame,text='???',fg='black',bg='white', command=partial(self.game, self.state[0]))
    buttonVec.append(playButton)

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


################################################################################
  def game_over(self, group):

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

    counter = 0
    next_group = -1
    for i in range(0,len(self.state[1])):
      if self.state[1][i] < len(self.Q[i]):
        counter = counter + 1
        next_group = i
        break

    if counter > 0:
      playButton = Tkinter.Button(frame,text='???',fg='black',bg='white', command=partial(self.game, next_group))
      buttonVec.append(playButton)
      buttonText.append('GAME OVER FOR GROUP ' + str(group+1) + '\n\n\nCorrect answers: ' + str(self.stats[0][group]) + '\n\nIncorrect answers: ' + str(self.stats[1][group]))
    else:
      playButton = Tkinter.Button(frame,text='???',fg='black',bg='white')
      buttonVec.append(playButton)
      buttonText.append('GAME OVER FOREVER')

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
