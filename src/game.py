#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import threading
import os
import rospy
import tf
import Tkinter
import numpy as np
import time
from subprocess import call, Popen
from functools import partial
from sys import argv
import random
import tkFont
from PIL import Image,ImageTk

# Sending goals etc
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseFeedback, MoveBaseResult

from std_msgs.msg import Empty
from std_srvs.srv import Empty as srv_empty
from std_msgs.msg import Int8
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped
from geometry_msgs.msg import Twist, Vector3



class GuiGame():

################################################################################
# constructor
################################################################################
  def __init__(self):

    self.root = Tkinter.Tk()
    self.root.attributes('-fullscreen',True)

    # Sets the pose estimate of amcl
    self.amcl_init_pose_pub = \
        rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)

    # Publishes raw velocity commands to the wheels of the base
    self.raw_velocity_commands_pub = \
        rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)

    self.init_params()

    # For measuring group times
    self.previous_group = -1
    self.global_clock = time.time()

    # Let's go
    self.init()

    # Seems that the mainloop should be placed here; otherwise throws error
    rospy.logwarn('root mainloop')
    self.root.mainloop()


  ##############################################################################
  def ask_to_restore_game(self):

    # new canvas
    canvas = self.new_canvas()
    self.set_canvas(canvas)

    # to frame panw sto opoio 8a einai ta koumpia
    frame = self.new_frame()
    self.set_frame(frame)

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
    QButton = Tkinter.Button(frame,text='???',fg='#E0B548',bg='#343A40',activeforeground='#E0B548',activebackground='#343A40')
    buttonVec.append(QButton)
    buttonText.append('Υπάρχει αποθηκευμένο παιχνίδι. Συνέχεια?')

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

        buttonVec[counter].config(font=("Helvetica", 20))
        buttonVec[counter].update()

        counter = counter+1



    #######################################
    # ta koumpia tou para8urou apanthseis
    buttonVec = []
    buttonText = []


    # Show choices
    answer_txt = ['ΒΕΒΑΙΩΣ', 'ΟΧΙ, ΕΥΧΑΡΙΣΤΩ']
    for i in range(2):
      buttonText.append(answer_txt[i])
      this_butt = Tkinter.Button(frame,text='???',fg='white',bg='#E0B548',activeforeground='white',activebackground='#E0B548',command=partial(self.response_to_restore_game,i))
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
          buttonVec[counter].config(font=("Helvetica", 20))
          buttonVec[counter].update()

        counter = counter+1


  ##############################################################################
  # Play end song
  def celebrate(self):
    rospy.logwarn('watch me im twerking')

    # This is synchronous
    #call(['cvlc', '--no-repeat','--play-and-exit', self.dir_media + '/tiff_game_over_ohyeah.mp3'])

    # This is asynchronous
    pn = random.randint(0,6)
    p = Popen(['cvlc', '--no-repeat','--play-and-exit', self.cel_media + '/c' + str(pn) + '.mp3'])

    # Celebrate randomly
    cn = random.randint(0,1)
    self.celebration_motion(cn)

    p.terminate()
    return


  ##############################################################################
  def celebration_motion(self,cn):

    # Rocking from side to side
    if cn == 0:
      l = Vector3(0,0,0)
      ap = Vector3(0,0,+1)
      an = Vector3(0,0,-1)

      twist_msg_p = Twist(l,ap)
      twist_msg_n = Twist(l,an)

      r = 16
      for i in range(r):
        self.raw_velocity_commands_pub.publish(twist_msg_p)
        rospy.sleep(0.5)
        self.raw_velocity_commands_pub.publish(twist_msg_n)
        rospy.sleep(0.5)



    # Rocking back and forth
    if cn == 1:
      lf = Vector3(+0.1,0,0)
      lb = Vector3(-0.1,0,0)
      a = Vector3(0,0,0)

      twist_msg_p = Twist(lf,a)
      twist_msg_n = Twist(lb,a)

      r = 16
      for i in range(r):
        self.raw_velocity_commands_pub.publish(twist_msg_p)
        rospy.sleep(0.5)
        self.raw_velocity_commands_pub.publish(twist_msg_n)
        rospy.sleep(0.5)


    # One full rotation
    if cn == 2:
      t = self.yaw

      l = Vector3(0,0,0)
      a = Vector3(0,0,+1)

      twist_msg = Twist(l,a)

      r = 20
      for i in range(r):
        self.raw_velocity_commands_pub.publish(twist_msg)
        rospy.sleep(0.2)

      da = 0.05
      while(abs(t-self.yaw) > da):
        self.raw_velocity_commands_pub.publish(twist_msg)

    return


  ##############################################################################
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


  ############################################################################
  def check_rfid_answer_validity(self, correct_answer):

    # Get the measurements
    measurements = self.read_file(self.rfid_java_exec_dir + '/' + self.rfid_file)
    if len(measurements) > 0:
      epc_most_measurements, proportion_of_measurements = self.get_epc_with_most_measurements(measurements)

      self.reset_file(self.rfid_java_exec_dir + '/' + self.rfid_file)

      # Antodimi fix 20/09/2022 after tif 22
      #if (proportion_of_measurements > 0.5) and (epc_most_measurements == correct_answer):
      if epc_most_measurements == correct_answer:
        return True
      else:
        return False
    else:
      return -1


  ############################################################################
  def close_rfid_reader(self):

    cmd = 'cd ' + self.rfid_java_exec_dir + ';' + './close_rfid_reader.sh'
    os.system(cmd)


  ##############################################################################
  def compile_statestring(self):

    # Compile self.state
    state_string = 'state: [' + str(self.state[0]) + ","
    state_string = state_string + "["
    for i in range(len(self.Q)):
      state_string = state_string + str(self.state[1][i])
      if i != len(self.Q)-1:
        state_string = state_string + ","
    state_string = state_string + "]"
    state_string = state_string + "]"

    # Compile self.stats
    stats_string = 'stats: ['

    for i in range(len(self.stats)):
      stats_string = stats_string + "["
      for j in range(len(self.stats[i])):
        stats_string = stats_string + str(self.stats[i][j])
        if j != len(self.stats[i])-1:
          stats_string = stats_string + ","

      stats_string = stats_string + "]"
      if i != len(self.stats)-1:
        stats_string = stats_string + ","

    stats_string = stats_string + "]"

    # Compile self.pose_
    pose_string = 'pose: ['
    pose_string = pose_string + str(self.pose_.pose.pose.position.x) + ', '
    pose_string = pose_string + str(self.pose_.pose.pose.position.y) + ', '
    pose_string = pose_string + str(self.pose_.pose.pose.position.z) + ', '
    pose_string = pose_string + str(self.pose_.pose.pose.orientation.x) + ', '
    pose_string = pose_string + str(self.pose_.pose.pose.orientation.y) + ', '
    pose_string = pose_string + str(self.pose_.pose.pose.orientation.z) + ', '
    pose_string = pose_string + str(self.pose_.pose.pose.orientation.w) + ']'

    return state_string + "\n\n" + stats_string + "\n\n" + pose_string


  ##############################################################################
  def continue_game(self, frame):

    current_group = self.state[0]

    # The current unanswered question for this group
    current_q = self.state[1][current_group]

    # All the choices for this question
    choices = self.C[current_group][current_q]

    # The only correct answer
    correct_a = self.A[current_group][current_q]

    # An image to display alongside the question
    current_i = self.I[current_group][current_q]

    # The voice-over of the question
    current_vf = self.V[current_group][current_q]

    #print current_group
    #print current_q
    #print choices
    #print correct_a
    #print current_i

    # Does there have to be an image alongside the question?
    display_image = False
    if current_i != '':
      display_image = True
      img = Tkinter.PhotoImage(\
          file=self.dir_media+'/'+ str(current_group) + str(current_q) + '.png',master=self.canvas_)
      cuid = Tkinter.Label(frame,image=img)
      cuid.image = img

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
      QButton = Tkinter.Button(frame,text='???',fg='#E0B548',bg='#343A40',activeforeground='#E0B548',activebackground='#343A40', command=partial(self.check_answer_given, -1, correct_a, do_open_rfid_reader))
    else:
      QButton = Tkinter.Button(frame,text='???',fg='#E0B548',bg='#343A40',activeforeground='#E0B548',activebackground='#343A40')
    buttonVec.append(QButton)

    # Make text of question fit into `max_line_length` characters

    # The text of the question
    q_text = self.Q[current_group][current_q]

    # Split by space
    q_text_split = q_text.split(" ")
    max_line_length = 80
    if display_image == True:
      max_line_length = max_line_length / 2

    lines = ['']
    line_counter = 0
    for i in range(len(q_text_split)):
      test_line = lines[line_counter] + " " + q_text_split[i]
      if len(test_line) <= max_line_length:
        lines[line_counter] = test_line
      else:
        line_counter = line_counter + 1
        lines.append('\n')
        lines[line_counter] = lines[line_counter] + " " + q_text_split[i]


    disp_qtext = ''
    for i in range(len(lines)):
      disp_qtext = disp_qtext + lines[i]

    buttonText.append(disp_qtext)


    if display_image == True:
      buttonVec.append(cuid)
      buttonText.append('')

      xEff = 1.0
      yEff = 0.6
      GP = 0.1
    else:
      xEff = 1.0
      yEff = 0.3
      GP = 0.1


    xNum = len(buttonVec)
    yNum = 1


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
    for answer_txt,num in zip(choices,range(len(choices))):
      buttonText.append(answer_txt)
      this_butt = Tkinter.Button(frame,text='???',fg='white',bg='#E0B548',activeforeground='white',activebackground='#E0B548',command=partial(self.check_answer_given, num, correct_a, do_open_rfid_reader))
      buttonVec.append(this_butt)

    xNum,yNum = self.get_x_y_dims(len(buttonVec))


    if display_image == True:
      buttonVec.append(cuid)
      buttonText.append('')

      xEff = 1.0
      yEff = 0.3
      GP = 0.1
    else:
      xEff = 1.0
      yEff = 0.6
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

    # Play question in audio form if audio file is supposed to exist
    if current_vf != '':
      if self.V_played[current_group][current_q] == False:
        call(['cvlc', '--no-repeat','--play-and-exit', self.dir_media + '/' + str(current_group) + str(current_q) + '.mp3'])
        self.V_played[current_group][current_q] = True



  ##############################################################################
  def correct_answer(self):

    # Update answered questions num
    self.state[1][self.state[0]] = self.state[1][self.state[0]] + 1
    rospy.logwarn('this answer is correct')

    # Increment correct answers counter for this group
    self.stats[0][self.state[0]] = self.stats[0][self.state[0]] + 1

    # Save state to file
    self.save_state_to_file()

    # to frame panw sto opoio 8a einai ta koumpia
    frame = self.new_frame()
    self.set_frame(frame)

    # ta koumpia tou para8urou
    buttonVec = []
    buttonText = []

    if self.state[1][self.state[0]] < len(self.Q[self.state[0]]):
      playButton = Tkinter.Button(frame,text='???',fg='#E0B548',bg='#343A40',activeforeground='#E0B548',activebackground='#343A40',command=partial(self.game, self.state[0]))
      buttonVec.append(playButton)
      buttonText.append('ΕΥΓΕ!')
    else:
      playButton = Tkinter.Button(frame,text='???',fg='#E0B548',bg='#343A40',activeforeground='#E0B548',activebackground='#343A40', command=partial(self.game_over, self.state[0]))
      buttonVec.append(playButton)
      buttonText.append('Συγχαρητήρια!')

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
        buttonVec[counter].config(font=("Helvetica", 30))
        buttonVec[counter].update()

        counter = counter+1

    # Celebrate mf
    self.celebrate()


  ##############################################################################
  def display_message(self, message):

    frame = self.new_frame()
    self.set_frame(frame)

    # ta koumpia tou para8urou
    buttonVec = []
    buttonText = []

    playButton = Tkinter.Button(frame,text='???',fg='#E0B548',bg='#343A40',activeforeground='#E0B548',activebackground='#343A40')
    buttonVec.append(playButton)
    buttonText.append(message)

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
        buttonVec[counter].config(font=("Helvetica", 20))
        buttonVec[counter].update()

        counter = counter+1


  ##############################################################################
  def display_winner(self, stats):

    correct = stats[0]
    incorrect = stats[1]
    time = stats[2]

    count = []
    for i in range(0,len(correct)):
      count.append((correct[i]-incorrect[i])/(time[i]+0.01))

    max_v = max(count)
    gs = [i for i,j in enumerate(count) if j == max_v]



    # to frame panw sto opoio 8a einai ta koumpia
    frame = self.new_frame()
    self.set_frame(frame)

    # ta koumpia tou para8urou
    buttonVec = []
    buttonText = []


    playButton = Tkinter.Button(frame,text='???',fg='#E0B548',bg='#343A40',activeforeground='#E0B548',activebackground='#343A40', command=self.kill_root)
    buttonVec.append(playButton)

    txt = 'ΣΥΓΧΑΡΗΤΗΡΙΑ ΣΤΗΝ ΟΜΑΔΑ'
    if (len(gs) == 1):
      txt = txt + ' ' + str(gs[0]+1)
    else:
      txt = txt + 's '
      for i in range(0,len(gs)-1):
        txt = txt + str(gs[i]+1) + ', '

      txt = txt + str(gs[len(gs)-1]+1)


    buttonText.append(txt)

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
        buttonVec[counter].config(font=("Helvetica", 30))
        buttonVec[counter].update()

        counter = counter+1

    # Reset the state and save it (no hung-ups here)
    self.reset_state()
    self.save_state_to_file()


  ##############################################################################
  # exit button
  def exit_button(self,frame):

    buttonVec = []
    buttonText = []

    this_butt = Tkinter.Button(frame,text='???',fg='#E0B548',bg='red',activeforeground='#E0B548',activebackground='#343A40', command=self.kill_root)

    buttonVec.append(this_butt)
    this_group_name = 'X'
    buttonText.append(this_group_name)

    xNum = 1
    yNum = 1

    xEff = 0.075
    yEff = 0.075

    GP = 0.175

    xWithGuard = xEff/xNum
    xG = GP*xWithGuard
    xB = xWithGuard-xG

    yWithGuard = yEff/yNum
    yG = GP*yWithGuard
    yB = yWithGuard-yG

    counter = len(buttonVec)-1
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
          buttonVec[counter].config(font=("Helvetica", 20))
          buttonVec[counter].update()

        counter = counter+1

    return frame


  ##############################################################################
  def extract_yaw(self, pose_msg):
    q = pose_msg.pose.pose.orientation
    euler = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
    return euler[2]


  ##############################################################################
  def game(self,group):
    self.state[0] = group
    rospy.logwarn('Current group:  %d' % self.state[0])
    rospy.logwarn('Previous group: %d' % self.previous_group)

    # Start the clock for the new group at every group change
    if self.state[0] != self.previous_group:
      self.clock_start[self.state[0]] = time.time()

    # End the clock for the previous group
    if self.previous_group == -1:
      self.clock_end[self.state[0]] = time.time()
      duration = self.clock_end[self.state[0]]-self.clock_start[self.state[0]]
      self.stats[2][self.state[0]] = self.stats[2][self.state[0]] + duration
      rospy.logwarn('duration 0 = %f', duration)
    else:
      self.clock_end[self.previous_group] = time.time()
      duration = self.clock_end[self.previous_group]-self.clock_start[self.previous_group]
      self.stats[2][self.previous_group] = self.stats[2][self.previous_group] + duration
      rospy.logwarn('duration 1 = %f', duration)

    self.previous_group = self.state[0]

    # Save state to file
    self.save_state_to_file()

    frame = self.new_frame()
    self.set_frame(frame)

    # The first buttons to be drawn
    frame = self.group_buttons(self.state[0], frame)
    frame = self.exit_button(frame)


    if self.state[1][self.state[0]] < len(self.Q[self.state[0]]):
      self.continue_game(frame)
    else:
      self.game_over(self.state[0])


  ##############################################################################
  def game_over(self, group):

    # Save state to file
    self.save_state_to_file()

    # to frame panw sto opoio 8a einai ta koumpia
    frame = self.new_frame()
    self.set_frame(frame)

    # ta koumpia tou para8urou
    buttonVec = []
    buttonText = []

    ccounter = 0
    next_group = -1
    for i in range(0,len(self.state[1])):
      if self.state[1][i] < len(self.Q[i]):
        ccounter = ccounter + 1
        next_group = i
        break

    if ccounter > 0:
      #playButton = Tkinter.Button(frame,text='???',fg='#E0B548',bg='#343A40',activeforeground='#E0B548',activebackground='#343A40', command=partial(self.game, next_group))
      playButton = Tkinter.Button(frame,text='???',fg='#E0B548',bg='#343A40',activeforeground='#E0B548',activebackground='#343A40', command=self.select_group_init)
      buttonVec.append(playButton)
      buttonText.append('GAME OVER ΓΙΑ ΤΗΝ ΟΜΑΔΑ ' + str(group+1) + '\n\n\nΣωστές απαντήσεις: ' + str(self.stats[0][group]) + '\nΛανθασμένες απαντήσεις: ' + str(self.stats[1][group]))
    else:
      playButton = Tkinter.Button(frame,text='???',fg='#E0B548',bg='#343A40',activeforeground='#E0B548',activebackground='#343A40', command=partial(self.display_winner, self.stats))
      buttonVec.append(playButton)
      txt = 'GAME OVER ΔΙΑ ΠΑΝΤΟΣ\n\n\n'
      for i in range(0,len(self.state[1])):
        txt = txt + 'ΟΜΑΔΑ ' + str(i+1) + ':\nΣωστές απαντήσεις: ' + str(self.stats[0][i]) + '\nΛανθασμένες απαντήσεις: ' + str(self.stats[1][i]) + '\nΧρόνος παιχνιδιού: ' + str(self.stats[2][i])[0:4] +  ' δευτερόλεπτα\n\n\n'
      buttonText.append(txt)

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
        if ccounter > 0:
          buttonVec[counter].config(font=("Helvetica", 20))
        else:
          buttonVec[counter].config(font=("Helvetica", 12))
        buttonVec[counter].update()

        counter = counter+1


  ##############################################################################
  def get_canvas(self):
    return self.canvas_


  ##############################################################################
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


  ##############################################################################
  def get_frame(self):
    return self.frame_


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
  def goto_goal_pose(self):

    # Construct goal msg
    goal_msg = self.make_goal_msg(self.goal_pose)

    # Clear and come with me
    self.display_message('ΠΑΡΑΚΑΛΩ ΑΠΟΜΑΚΡΥΝΘΕΙΤΕ')
    rospy.sleep(3.0)
    rospy.wait_for_service('/move_base/clear_costmaps')
    rospy.ServiceProxy('/move_base/clear_costmaps', srv_empty)
    rospy.sleep(2.0)
    self.display_message('ΠΑΡΑΚΑΛΩ ΕΛΑΤΕ ΜΑΖΙ ΜΟΥ')

    rospy.logwarn('sending goal and waiting for result')
    self.action_client.send_goal(goal_msg)
    self.action_client.wait_for_result()

    # TODO unload params here and load new params (?)


  ##############################################################################
  # group buttons are displayed in each screen for switching between groups
  def group_buttons(self, highlight_group, frame):

    # to frame panw sto opoio 8a einai ta koumpia
    #frame = self.new_frame()
    #self.set_frame(frame)

    buttonVec = []
    buttonText = []

    for i in range(0,len(self.Q)):
      if i == highlight_group:
        this_butt = Tkinter.Button(frame,text='???',fg='#E0B548',bg='green',activeforeground='#E0B548',activebackground='green', command=partial(self.show_intro_video_play_button,i))
      else:
        this_butt = Tkinter.Button(frame,text='???',fg='#E0B548',bg='#343A40',activeforeground='#E0B548',activebackground='#343A40', command=partial(self.show_intro_video_play_button,i))

      buttonVec.append(this_butt)
      this_group_name = 'ΟΜΑΔΑ %d' %(i+1)
      buttonText.append(this_group_name)

    xNum = len(buttonVec)
    yNum = 1

    xEff = 0.70
    yEff = 0.1

    GP = 0.2

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
          thisX = xG/2+xx*xWithGuard#+xEff
          thisY = yG/2+yy*yWithGuard

          buttonVec[counter].place(relx=thisX,rely=thisY,relheight=yB,relwidth=xB)
          buttonVec[counter].config(text=buttonText[counter])
          buttonVec[counter].update()

          thisWidth = buttonVec[counter].winfo_width()
          thisHeight = buttonVec[counter].winfo_height()
          buttonVec[counter].config(font=("Helvetica", 20))
          buttonVec[counter].update()

        counter = counter+1

    return frame


  ##############################################################################
  def incorrect_answer(self):
    rospy.logwarn('this answer is incorrect')

    # Increment incorrect answers counter for this group
    self.stats[1][self.state[0]] = self.stats[1][self.state[0]] + 1

    # Save state to file
    self.save_state_to_file()

    # to frame panw sto opoio 8a einai ta koumpia
    frame = self.new_frame()
    self.set_frame(frame)

    # ta koumpia tou para8urou
    buttonVec = []
    buttonText = []

    buttonText.append('Κάνατε λάθος. Πατήστε για επιστροφή')
    playButton = Tkinter.Button(frame,text='???',fg='#E0B548',bg='#343A40',activeforeground='#E0B548',activebackground='#343A40', command=partial(self.game, self.state[0]))
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
        buttonVec[counter].config(font=("Helvetica", 30))
        buttonVec[counter].update()

        counter = counter+1

    # Voice of goddess
    call(['cvlc', '--no-repeat','--play-and-exit', self.dir_media + '/try_again.mp3'])

  ##############################################################################
  def init(self):
    rospy.logwarn('init')

    # new canvas
    canvas = self.new_canvas()
    self.set_canvas(canvas)

    # to frame panw sto opoio 8a einai ta koumpia
    frame = self.new_frame()
    self.set_frame(frame)

    # ta koumpia tou para8urou
    buttonVec = []
    buttonText = []

    buttonText.append('ΕΚΚΙΝΗΣΗ')
    if self.ask_to_restore == True:
      sg = Tkinter.Button(frame,text='???',fg='#E0B548',bg='#343A40',activeforeground='#E0B548',activebackground='#343A40',command=self.ask_to_restore_game)
    else:
      self.reset_state()
      sg = Tkinter.Button(frame,text='???',fg='#E0B548',bg='#343A40',activeforeground='#E0B548',activebackground='#343A40',command=self.select_group_init)
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
        buttonVec[counter].config(font=("Helvetica", 30))
        buttonVec[counter].update()

        counter = counter+1


  ##############################################################################
  def init_params(self):

    # Read params
    self.dir_media = rospy.get_param('~dir_media', '')
    self.cel_media = rospy.get_param('~cel_media', '')
    self.dir_scripts = rospy.get_param('~dir_scripts', '')
    self.rfid_java_exec_dir = rospy.get_param('~rfid_java_exec_dir', '')
    self.rfid_file = rospy.get_param('~rfid_file', '')
    self.statefile = rospy.get_param('~statefile', '')
    self.start_pose = rospy.get_param('~start_pose', '')
    self.goal_pose = rospy.get_param('~goal_pose', '')

    # Read [Q]uestions, [C]hoices, correct [A]nswers, [I]mages
    self.Q = rospy.get_param('~Q', '')
    self.C = rospy.get_param('~C', '')
    self.A = rospy.get_param('~A', '')
    self.I = rospy.get_param('~I', '')
    self.V = rospy.get_param('~V', '')


    # Booleans for playing the voice-over of a question only once;
    # otherwise it breaks balls. Init to false for all.
    # (BTW PYTHON USES copy by reference in `self.V_played = self.V` !!!)
    self.V_played = []
    for i in range(len(self.V)):
      self.V_played.append([])
      for j in range(len(self.V[i])):
        self.V_played[i].append(False)


    # Read saved state
    self.in_state = rospy.get_param('~state', '')
    self.in_stats = rospy.get_param('~stats', '')
    self.in_pose  = rospy.get_param('~pose', '')


    if self.dir_media == '':
      rospy.logerr('[cultureid_games_N] dir_media not set; aborting')
      return

    if self.cel_media == '':
      rospy.logerr('[cultureid_games_N] cel_media not set; aborting')
      return

    if self.dir_scripts == '':
      rospy.logerr('[cultureid_games_N] dir_scripts not set; aborting')
      return

    if self.rfid_file == '':
      rospy.logerr('[cultureid_games_N] rfid_file not set; aborting')
      return

    if self.rfid_java_exec_dir == '':
      rospy.logerr('[cultureid_games_N] rfid_java_exec_dir not set; aborting')
      return
    else:
      self.reset_file(self.rfid_java_exec_dir + '/' + self.rfid_file)

    if self.statefile == '':
      rospy.logerr('[cultureid_games_N] statefile not set; aborting')
      return

    if self.start_pose == '':
      rospy.logerr('[cultureid_games_N] start_pose not set; aborting')
      return

    if self.goal_pose == '':
      rospy.logerr('[cultureid_games_N] goal_pose not set; aborting')
      return

    if self.Q == '':
      rospy.logerr('[cultureid_games_N] Q not set; aborting')
      return

    if self.C == '':
      rospy.logerr('[cultureid_games_N] C not set; aborting')
      return

    if self.A == '':
      rospy.logerr('[cultureid_games_N] A not set; aborting')
      return

    # Set pose; from normal configuration
    self.pose_ = self.make_pose_msg(self.start_pose)
    self.set_amcl_init_pose_msg(self.pose_)

    # Check if the user wants to restore the last game if abruptly hung up
    if self.in_state != '' and self.in_stats != '':
      self.ask_to_restore = False
      if sum(self.in_state[1]) != 0:
        self.ask_to_restore = True

    # If the introduction video has been shown do not show it again
    self.intro_played = [False] * len(self.Q)


  ##############################################################################
  def insufficient_answer(self):
    rospy.logwarn('this answer is insufficient')

    # to frame panw sto opoio 8a einai ta koumpia
    frame = self.new_frame()
    self.set_frame(frame)

    # ta koumpia tou para8urou
    buttonVec = []
    buttonText = []

    buttonText.append('Παρακαλώ πλησιάστε την κάρτα πιο κοντά')
    playButton = Tkinter.Button(frame,text='???',fg='#E0B548',bg='#343A40',activeforeground='#E0B548',activebackground='#343A40', command=partial(self.game, self.state[0]))
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
        buttonVec[counter].config(font=("Helvetica", 30))
        buttonVec[counter].update()

        counter = counter+1


  ##############################################################################
  def kill_root(self):
    self.save_state_to_file()
    call(['bash', '/home/cultureid_user0/game_desktop_launchers/kill_all.sh'])
    self.root.destroy()
    print('game over')
    os._exit(os.EX_OK)


  ##############################################################################
  def make_goal_msg(self, target_pose):
    goal = MoveBaseGoal()
    goal.target_pose.pose.position.x = target_pose[0]
    goal.target_pose.pose.position.y = target_pose[1]
    goal.target_pose.pose.position.z = target_pose[2]
    goal.target_pose.pose.orientation.x = target_pose[3]
    goal.target_pose.pose.orientation.y = target_pose[4]
    goal.target_pose.pose.orientation.z = target_pose[5]
    goal.target_pose.pose.orientation.w = target_pose[6]
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.header.stamp = rospy.Time.now()

    return goal


  ##############################################################################
  def make_pose_msg(self, in_pose):
    out_pose = PoseWithCovarianceStamped()
    out_pose.pose.pose.position.x = in_pose[0]
    out_pose.pose.pose.position.y = in_pose[1]
    out_pose.pose.pose.position.z = in_pose[2]
    out_pose.pose.pose.orientation.x = in_pose[3]
    out_pose.pose.pose.orientation.y = in_pose[4]
    out_pose.pose.pose.orientation.z = in_pose[5]
    out_pose.pose.pose.orientation.w = in_pose[6]
    out_pose.header.frame_id = 'map'
    out_pose.header.stamp = rospy.Time.now()

    return out_pose


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


  ############################################################################
  def open_rfid_reader(self):
    cmd = 'cd ' + self.rfid_java_exec_dir + ';' + './open_rfid_reader.sh'
    os.system(cmd)


  ##############################################################################
  def pose_callback(self, msg):
    rospy.logwarn('Pose callback')
    self.pose_ = PoseWithCovarianceStamped(msg)


  ##############################################################################
  def quit_game(self):
    self.root.destroy()
    rospy.signal_shutdown('game over')
    os._exit(os.EX_OK)


  ##############################################################################
  def response_to_restore_game(self, c):
    if c == 0:
      self.restore_state()

      # Set pose; from saved file
      self.pose_ = self.make_pose_msg(self.in_pose)

    if c == 1:
      self.reset_state()

      # Set pose; from normal configuration
      self.pose_ = self.make_pose_msg(self.start_pose)


    self.set_amcl_init_pose_msg(self.pose_)
    self.save_state_to_file()
    self.select_group_init()


  ############################################################################
  def read_file(self, file_str):
    with open(file_str,'r') as f:
      lines  = f.readlines()
      f.close()
      return lines


  ##############################################################################
  def reset_file(self, file_str):
    with open(file_str,'w') as f:
      f.close()


  ##############################################################################
  def restore_state(self):
    self.reset_state()
    self.state = self.in_state
    self.stats = self.in_stats


  ##############################################################################
  def reset_state(self):

    # self.state[0] (scalar) indicates the current group playing;
    # self.state[1] is a list indicating the question index that has not been
    # answered yet (this question is the current question)
    # First group playing assumed to be group 0
    self.state = [0,[]]
    self.state[1] = [0] * len(self.Q)

    # Correct answers [0], incorrect answers [1] and game duration [2] per group
    self.stats = [[],[],[]]
    self.stats[0] = [0] * len(self.Q)
    self.stats[1] = [0] * len(self.Q)
    self.stats[2] = [0] * len(self.Q)

    self.clock_start = [0] * len(self.Q)
    self.clock_end = [0] * len(self.Q)


  ##############################################################################
  def save_state_to_file(self):
    self.write_file(self.compile_statestring(), self.statefile)


  ##############################################################################
  def select_group_init(self):

    # to frame panw sto opoio 8a einai ta koumpia
    frame = self.new_frame()
    self.set_frame(frame)

    buttonVec = []
    buttonText = []

    for i in range(0,len(self.Q)):
      this_butt = Tkinter.Button(frame,text='???',fg='#E0B548',bg='#343A40',activeforeground='#E0B548',activebackground='#343A40', command=partial(self.show_intro_video_play_button,i))
      buttonVec.append(this_butt)
      this_group_name = 'ΟΜΑΔΑ %d' %(i+1)
      buttonText.append(this_group_name)


    # Swapped order so that buttons are placed horizontally
    xNum,yNum = self.get_x_y_dims(len(buttonVec))

    xEff = 1.0
    yEff = 0.8

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
          print counter, thisX, thisY, yB, xB
          buttonVec[counter].config(font=("Helvetica", 30))
          buttonVec[counter].update()

        counter = counter+1


    # EXIT button
    exit_button = Tkinter.Button(frame,text='???',fg='#E0B548',bg='#343A40',activeforeground='#E0B548',activebackground='#343A40', command=self.kill_root)
    buttonVec.append(exit_button)
    buttonText.append('ΕΞΟΔΟΣ')

    xEff = 1.0
    yEff = 0.2

    GP = 0.05

    xNum = xNum+1
    yNum = yNum+1

    xWithGuard = xEff/xNum
    xG = GP*xWithGuard
    xB = xWithGuard-xG

    yWithGuard = yEff/yNum
    yG = GP*yWithGuard
    yB = yWithGuard-yG

    thisX = xG/2+xx*xWithGuard+1-xEff
    thisY = yG/2+yy*yWithGuard+1-yEff

    buttonVec[counter].place(relx=thisX,rely=thisY,relheight=yB,relwidth=xB)
    buttonVec[counter].config(text=buttonText[counter])
    buttonVec[counter].update()

    thisWidth = buttonVec[counter].winfo_width()
    thisHeight = buttonVec[counter].winfo_height()
    print counter, thisX, thisY, yB, xB
    buttonVec[counter].config(font=("Helvetica", 30))
    buttonVec[counter].update()



  ##############################################################################
  def set_amcl_init_pose_msg(self, pose):
    self.amcl_init_pose_pub.publish(pose)


  ##############################################################################
  def set_canvas(self, canvas):
    self.canvas_ = canvas


  ##############################################################################
  def set_frame(self, frame):
    self.frame_ = frame


  ##############################################################################
  def show_intro_video(self, group):

    call(['vlc', '--no-repeat','--fullscreen','--play-and-exit', \
        self.dir_media + '/intro_' + str(group) + '.mp4'])

    self.game(group)


  ##############################################################################
  def show_intro_video_play_button(self, group):

    # if the introduction video has not been played yet
    if self.intro_played[group] == False and self.state[1][group] == 0:

      # to frame panw sto opoio 8a einai ta koumpia
      frame = self.new_frame()
      self.set_frame(frame)

      # ta koumpia tou para8urou
      buttonVec = []
      buttonText = []


      playButton = Tkinter.Button(frame,text='???',fg='#E0B548',bg='#343A40',activeforeground='#E0B548',activebackground='#343A40', command=partial(self.show_intro_video,group))
      buttonVec.append(playButton)
      buttonText.append('ΑΝΑΠΑΡΑΓΩΓΗ ΟΠΤΙΚΟΑΚΟΥΣΤΙΚΟΥ ΥΛΙΚΟΥ')


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
          buttonVec[counter].config(font=("Helvetica", 20))
          buttonVec[counter].update()

          counter = counter+1
    else:
      self.game(group)


  ##############################################################################
  def write_file(self, content, file_str):
    with open(file_str,'w') as f:
      f.write(content)
      f.close()













################################################################################
# main
################################################################################
if __name__ == '__main__':

  rospy.init_node('gui_game_node')

  try:
    gga = GuiGame()
    rospy.spin()

  except rospy.ROSInterruptException:
    print("SHUTTING DOWN")
    pass
