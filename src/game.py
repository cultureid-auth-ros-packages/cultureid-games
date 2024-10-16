#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import threading
import os
import numpy as np
import time
import copy
from subprocess import call, Popen
from functools import partial
import random
import Tkinter
import tkFont
from PIL import Image,ImageTk

# Import ROS modules
import rospy
import tf

from std_msgs.msg import Empty
from std_srvs.srv import Empty as srv_empty
from std_msgs.msg import Int8
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped
from geometry_msgs.msg import Twist, Vector3


class AMTHGame():

  ##############################################################################
  # constructor
  ##############################################################################
  def __init__(self):
  #def __init__(self,root):

    self.root = Tkinter.Tk()
    self.root.attributes('-fullscreen',True)
    #self.root = root

    # Sets the pose estimate of amcl
    self.amcl_init_pose_pub = \
        rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10, latch=True)

    # Publishes raw velocity commands to the wheels of the base
    # [for celebrations]
    self.raw_velocity_commands_pub = \
        rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)

    # Load and set
    self.init_params()

    # Let's go
    self.init()

    # Seems that the mainloop should be placed here; otherwise throws error
    #rospy.logwarn('AMTHGAME root mainloop')
    self.root.mainloop()


  ##############################################################################
  def append_answer_to_list_of_answers(self, ans):
    self.answers_list.append(ans)

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
  def ask_to_skip_group(self, group):

    # new canvas
    canvas = self.new_canvas()
    self.set_canvas(canvas)

    # to frame panw sto opoio 8a einai ta koumpia
    frame = self.new_frame()
    self.set_frame(frame)

    # Show Q
    buttonVec = []
    buttonText = []

    QButton = Tkinter.Button(frame,text='???',fg='#E0B548',bg='#343A40',activeforeground='#E0B548',activebackground='#343A40')
    buttonVec.append(QButton)
    buttonText.append('Επιθυμείτε να τερματίσετε το παρόν \nπαιχνίδι για την ομάδα ' + str(group+1) + '?')

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
    answer_txt = ['ΝΑΙ, ΠΑΜΕ ΠΑΡΑΚΑΤΩ', 'ΟΧΙ, ΘΕΛΩ ΝΑ ΣΥΝΕΧΙΣΩ ΝΑ ΠΑΙΖΩ']
    for i in range(2):
      buttonText.append(answer_txt[i])
      if i == 0:
        this_butt = Tkinter.Button(frame,text='???',fg='white',bg='#E0B548',activeforeground='white',activebackground='#E0B548',command=partial(self.quit_group,group))
      else:
        this_butt = Tkinter.Button(frame,text='???',fg='white',bg='#E0B548',activeforeground='white',activebackground='#E0B548',command=partial(self.game,group))

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

        counter += 1


  ##############################################################################
  # Play end song
  def celebrate(self, game_notover_flag ):
    rospy.logwarn('watch me im twerking')

     # Celebrate conditionally. If game over: move; if not, don't.
    if game_notover_flag:
      pn = random.randint(0,2)
      call(['cvlc', '--no-repeat','--play-and-exit', self.cel_media + '/cshort' + str(pn) + '.mp3'])
    else:
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

      if isinstance(answer_given, int):
        if answer_given == correct_a:
          self.correct_answer()
        else:
          self.incorrect_answer()

      if isinstance(answer_given, list):

        answer_given_set = set(answer_given)

        # Counts occurences of elements in `answer_given_set`
        counts = [0]*len(answer_given_set)
        i = 0

        for ag in answer_given_set:
          for j in range(len(answer_given)):
            if ag == answer_given[j]:
              counts[i] += 1
          i += 1


        # One might select a false answer, and then deselect it. The incorrect
        # answer must be removed from the answers' list
        i = 0
        to_be_set = set()
        for ag in answer_given_set:
          if counts[i] % 2 == 1:
            to_be_set.add(ag)
          i += 1

        self.answers_list = list(to_be_set)

        if to_be_set == set(correct_a):
          self.correct_answer()
        else:
          self.incorrect_answer()

    else:
      if answer_given == -1:
        rospy.logwarn('OPENING RFID READER')
        self.reset_file(self.rfid_java_exec_dir + '/' + self.rfid_file)
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

    # For correct, incorrect, and points
    for i in range(len(self.stats)-1):
      stats_string = stats_string + "["
      for j in range(len(self.stats[i])):
        stats_string = stats_string + str(self.stats[i][j])
        if j != len(self.stats[i])-1:
          stats_string = stats_string + ","

      stats_string = stats_string + "]"
      if i != len(self.stats)-1:
        stats_string = stats_string + ","

    stats_string = stats_string + str(self.stats[len(self.stats)-1]) + "]"

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

    # When a questions has multiple correct answers this list holds all
    # checked answers
    self.answers_list = []

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
          file=self.dir_media+'/'+ self.I[current_group][current_q],master=self.canvas_)
      cuid = Tkinter.Label(frame,image=img)
      cuid.image = img

    # Is this a button question or a rfid-card question?
    do_open_rfid_reader = False
    if isinstance(correct_a, str):
      do_open_rfid_reader = True



    # --------------------------------------------------------------------------
    # Show Q -------------------------------------------------------------------
    # --------------------------------------------------------------------------
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
      if isinstance(correct_a, int) or isinstance(correct_a, str):
        QButton = Tkinter.Button(frame,text='???',fg='#E0B548',bg='#343A40',activeforeground='#E0B548',activebackground='#343A40')
      elif isinstance(correct_a,list):
        QButton = Tkinter.Button(frame,text='???',fg='#E0B548',bg='#343A40',activeforeground='#E0B548',activebackground='#343A40',command=partial(self.check_answer_given, self.answers_list, correct_a, do_open_rfid_reader))

    buttonVec.append(QButton)

    # Make text of question fit into `max_line_length` characters

    # The text of the question
    q_text = self.Q[current_group][current_q]

    # Fit text into box
    #disp_qtext = self.fit_text(q_text, display_image, 80)
    buttonText.append(q_text)


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
        buttonVec[counter].config(wraplength=thisWidth-10,justify="center")
        buttonVec[counter].update()

        counter = counter+1





    # ta koumpia tou para8urou apanthseis
    buttonVec = []
    buttonText = []

    # All checkboxes must be checked off before execution...
    # https://stackoverflow.com/questions/37595078/tkinter-is-there-a-way-to-check-checkboxes-by-default
    self.check_var = []

    # --------------------------------------------------------------------------
    # Show A -------------------------------------------------------------------
    # --------------------------------------------------------------------------
    for answer_txt,num in zip(choices,range(len(choices))):

      # Fit text into box
      #disp_qtext = self.fit_text(answer_txt, False, 40)

      # If you find a `~` character in an answer this means that a rfid tag is
      # required to be presented to the robot as an answer.
      id_tilda = answer_txt.find('~')
      if id_tilda == -1:
        buttonText.append(answer_txt)
      else:
        buttonText.append(answer_txt[0:id_tilda])

      if isinstance(correct_a, int) or isinstance(correct_a, str):
        this_butt = Tkinter.Button(frame,text='???',fg='white',bg='#E0B548',activeforeground='white',activebackground='#E0B548',command=partial(self.check_answer_given, num, correct_a, do_open_rfid_reader))
      elif isinstance(correct_a, list):
        self.check_var.append(Tkinter.IntVar(value=0))
        this_butt = Tkinter.Checkbutton(frame,text='???',selectcolor="black",fg='white',bg='#E0B548',activeforeground='white',activebackground='#E0B548',  variable=self.check_var[num], command=partial(self.append_answer_to_list_of_answers, num))

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
          buttonVec[counter].config(wraplength=thisWidth-10,justify="center")
          buttonVec[counter].update()

        counter = counter+1

    # Play question in audio form if audio file is supposed to exist
    if current_vf != '':
      if self.V_played[current_group][current_q] == False:
        call(['cvlc', '--no-repeat','--play-and-exit', self.dir_media + '/' + str(self.V[current_group][current_q])])
        self.V_played[current_group][current_q] = True

    """
    # --------------------------------------------------------------------------
    # --------------------------------------------------------------------------
    # Display enter button
    buttonVec = []
    buttonText = []

    o_flag = False

    if do_open_rfid_reader:
      o_flag = True
      QButton = Tkinter.Button(frame,text='???',fg='#E0B548',bg='green', command=partial(self.check_answer_given, -1, correct_a, do_open_rfid_reader))
    else:
      if isinstance(correct_a,list):
        o_flag = True
        QButton = Tkinter.Button(frame,text='???',fg='#E0B548',bg='green',command=partial(self.check_answer_given, self.answers_list, correct_a, do_open_rfid_reader))

    if o_flag == True:
      buttonVec.append(QButton)
      this_group_name = 'O'
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
            thisX = xG/2+xx*xWithGuard+1-2*xEff
            thisY = yG/2+yy*yWithGuard

            buttonVec[counter].place(relx=thisX,rely=thisY,relheight=yB,relwidth=xB)
            buttonVec[counter].config(text=buttonText[counter])
            buttonVec[counter].update()

            thisWidth = buttonVec[counter].winfo_width()
            thisHeight = buttonVec[counter].winfo_height()
            buttonVec[counter].config(font=("Helvetica", 20))
            buttonVec[counter].update()

          counter = counter+1
  """

  ##############################################################################
  def correct_answer(self):

    # Update answered questions num
    self.state[1][self.state[0]] = self.state[1][self.state[0]] + 1
    rospy.logwarn('this answer is correct')

    # Increment correct answers counter for this group
    self.stats[0][self.state[0]] = self.stats[0][self.state[0]] + 1

    # Calculate points
    self.stats[2][self.state[0]] += self.point_pl

    # Save state to file
    self.save_state_to_file()

    # to frame panw sto opoio 8a einai ta koumpia
    frame = self.new_frame()
    self.set_frame(frame)

    # The photo displayed over the label
    photo = Tkinter.PhotoImage(master = self.canvas_, file = self.correct_answer_image)

    game_notover_flag = self.state[1][self.state[0]] < len(self.Q[self.state[0]])



    # Display label, NOT button; see
    # https://github.com/PySimpleGUI/PySimpleGUI/issues/5036#issuecomment-1000423733
    playLabel = Tkinter.Label(frame,text='???',fg='#E0B548',bg='#343A40',activeforeground='#E0B548',activebackground='#343A40', image=photo, compound=Tkinter.TOP)
    if game_notover_flag:
      labelText = 'ΕΥΓΕ! Έχετε %d πόντους!' % self.stats[2][self.state[0]]
    else:
      labelText = 'ΣΥΓΧΑΡΗΤΗΡΙΑ! \nΣυγκεντρώσατε %d πόντους!' % self.stats[2][self.state[0]]

    xNum = 1
    yNum = 1

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

        playLabel.place(relx=thisX,rely=thisY,relheight=yB,relwidth=xB)
        playLabel.config(text=labelText)
        playLabel.update()

        thisWidth = playLabel.winfo_width()
        thisHeight = playLabel.winfo_height()
        playLabel.config(font=("Helvetica", 30))
        playLabel.update()




    # Celebrate mf
    self.celebrate(game_notover_flag )

    # Keep score screen on for a second
    rospy.sleep(1)

    # Go back to the beginning or to the end
    if game_notover_flag:
      self.select_group_init()
    else:
      self.game_over(self.state[0])


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

    # Locate index of max points
    max_v = max(self.stats[2])
    gs = [i for i,j in enumerate(self.stats[2]) if j == max_v]

    # to frame panw sto opoio 8a einai ta koumpia
    frame = self.new_frame()
    self.set_frame(frame)

    # ta koumpia tou para8urou
    buttonVec = []
    buttonText = []


    playButton = Tkinter.Button(frame,text='???',fg='#E0B548',bg='#343A40',activeforeground='#E0B548',activebackground='#343A40', command=self.kill_root)
    buttonVec.append(playButton)

    txt = 'ΣΥΓΧΑΡΗΤΗΡΙΑ'
    if (len(gs) == 1):
      txt = txt + ' ΣΤΗΝ ΟΜΑΔΑ ' + str(gs[0]+1)
    else:
      txt = txt + ' ΣΤΙΣ ΟΜΑΔΕΣ '
      for i in range(0,len(gs)-1):
        txt = txt + str(gs[i]+1) + ', '

      txt = txt + str(gs[len(gs)-1]+1)


    txt += ' \nΣυνολικός χρόνος τάξης: %d δευτερόλεπτα' % int(self.stats[3])

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
    yEff = 0.1

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
  def fit_text(self,in_text,display_image,width):

    q_text_split = in_text.split(" ")
    max_line_length = width
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

    return disp_qtext


  ##############################################################################
  def game(self,group):
    self.state[0] = group
    rospy.logwarn('Current group:  %d' % self.state[0])
    rospy.logwarn('Previous group: %d' % self.previous_group)

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
      buttonText.append('GAME OVER ΓΙΑ ΤΗΝ ΟΜΑΔΑ ' + str(group+1) + '\n\n\nΣωστές απαντήσεις: ' + str(self.stats[0][group]) + '\nΛανθασμένες απαντήσεις: ' + str(self.stats[1][group]) + '\nΠόντοι: ' + str(self.stats[2][group]))
    else:
      playButton = Tkinter.Button(frame,text='???',fg='#E0B548',bg='#343A40',activeforeground='#E0B548',activebackground='#343A40', command=partial(self.display_winner, self.stats))
      buttonVec.append(playButton)
      txt = 'GAME OVER ΔΙΑ ΠΑΝΤΟΣ\n\n\n'
      for i in range(0,len(self.state[1])):
        txt = txt + 'ΟΜΑΔΑ ' + str(i+1) + ':\nΣωστές απαντήσεις: ' + str(self.stats[0][i]) + '\nΛανθασμένες απαντήσεις: ' + str(self.stats[1][i]) + '\nΠόντοι: ' + str(self.stats[2][i])+  ' \n\n\n'
      buttonText.append(txt)

      self.groups_clock_stop = rospy.Time.now()
      self.stats[3] = (self.groups_clock_stop-self.groups_clock_start).to_sec()

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


    # Calculate measurement occurencies for each unique epc / tag
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


    # Group playing
    current_group = self.state[0]

    # The current unanswered question for this group
    current_q = self.state[1][current_group]

    # All the choices for this question
    choices = copy.copy(self.C[current_group][current_q])

    # Get available answers in epc form for the current question
    unique_epcs_limited = {}
    for c in range(len(choices)):
      id_tilda = choices[c].find('~')
      choices[c] = choices[c][id_tilda+1:]

    # `unique_epcs_limited` holds only, and at most, the available answers in
    # epc form and their occurence count in the measurements file
    unique_epcs_limited = {}
    for u in range(len(unique_epcs)):
      if unique_epcs[u] in choices:
        unique_epcs_limited[unique_epcs[u]] = unique_epc_counter[u]


    if len(unique_epcs_limited) > 0:
      return max(unique_epcs_limited, key=unique_epcs_limited.get), np.max(unique_epcs_limited.values()) / len(measurements)
    else:
      return 'NONE', 0


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
  # group buttons are displayed in each screen for switching between groups
  def group_buttons(self, highlight_group, frame):

    # to frame panw sto opoio 8a einai ta koumpia
    #frame = self.new_frame()
    #self.set_frame(frame)

    buttonVec = []
    buttonText = []

    for i in range(0,len(self.Q)):
      if i == highlight_group:
        this_butt = Tkinter.Button(frame,text='???',fg='#E0B548',bg='green',activeforeground='#E0B548',activebackground='green', command=partial(self.ask_to_skip_group,i))
      else:
        this_butt = Tkinter.Button(frame,text='???',fg='#E0B548',bg='#343A40',activeforeground='#E0B548',activebackground='#343A40', command=partial(self.show_intro_video_play_button,i))

      buttonVec.append(this_butt)
      this_group_name = 'ΟΜΑΔΑ %d (%d)' % ((i+1), self.stats[2][i])
      buttonText.append(this_group_name)

    xNum = len(buttonVec)
    yNum = 1

    xEff = 0.85
    yEff = 0.1

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
          thisX = xG/2+xx*xWithGuard#+xEff
          thisY = yG/2+yy*yWithGuard

          buttonVec[counter].place(relx=thisX,rely=thisY,relheight=yB,relwidth=xB)
          buttonVec[counter].config(text=buttonText[counter])
          buttonVec[counter].update()

          thisWidth = buttonVec[counter].winfo_width()
          thisHeight = buttonVec[counter].winfo_height()
          buttonVec[counter].config(font=("Helvetica", 16))
          buttonVec[counter].update()

        counter = counter+1

    return frame


  ##############################################################################
  def incorrect_answer(self):
    rospy.logwarn('this answer is incorrect')

    # Increment incorrect answers counter for this group
    self.stats[1][self.state[0]] = self.stats[1][self.state[0]] + 1

    # Calculate points
    self.stats[2][self.state[0]] -= self.point_mn

    # Save state to file
    self.save_state_to_file()

    # to frame panw sto opoio 8a einai ta koumpia
    frame = self.new_frame()
    self.set_frame(frame)

    # The photo to appear on the label
    photo = Tkinter.PhotoImage(master = self.canvas_, file = self.incorrect_answer_image)


    # Display label, NOT button
    playLabel = Tkinter.Label(frame,text='???',fg='#E0B548',bg='#343A40',activeforeground='#E0B548',activebackground='#343A40', image=photo, compound=Tkinter.TOP)
    labelText = 'Κάνατε λάθος. Έχετε %d πόντους!' % self.stats[2][self.state[0]]

    xNum = 1
    yNum = 1

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

        playLabel.place(relx=thisX,rely=thisY,relheight=yB,relwidth=xB)
        playLabel.config(text=labelText)
        playLabel.update()

        thisWidth = playLabel.winfo_width()
        thisHeight = playLabel.winfo_height()
        playLabel.config(font=("Helvetica", 30))
        playLabel.update()



    # Voice of goddess
    call(['cvlc', '--no-repeat','--play-and-exit', self.dir_media + '/try_again.mp3'])

    # Keep score screen on for a second
    rospy.sleep(1)

    self.game(self.state[0])

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
    self.rules_dir = rospy.get_param('~rules_dir', '')
    self.cel_media = rospy.get_param('~cel_media', '')
    self.rfid_java_exec_dir = rospy.get_param('~rfid_java_exec_dir', '')
    self.rfid_file = rospy.get_param('~rfid_file', '')
    self.statefile = rospy.get_param('~statefile', '')
    self.start_pose = rospy.get_param('~start_pose', '')
    self.correct_answer_image = rospy.get_param('~correct_answer_image', '')
    self.incorrect_answer_image = rospy.get_param('~incorrect_answer_image', '')

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

    if self.correct_answer_image == '':
      rospy.logerr('[cultureid_games_N] correct_answer_image not set; aborting')
      return

    if self.incorrect_answer_image == '':
      rospy.logerr('[cultureid_games_N] incorrect_answer_image not set; aborting')
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

    # All games start from game a. Due to the large map problem set the
    # robot's pose to that of the start pose of game a
    #self.set_amcl_init_pose_msg(self.pose_)

    # Check if the user wants to restore the last game if abruptly hung up
    if self.in_state != '' and self.in_stats != '':
      self.ask_to_restore = False
      if sum(self.in_state[1]) != 0:
        self.ask_to_restore = True

    # If the introduction video has been shown do not show it again
    self.intro_played = [False] * len(self.Q)


    # For measuring group times
    self.previous_group = -1
    self.groups_clock_start = rospy.Time.now()
    self.groups_clock_stop = rospy.Time.now()

    # Weights for correct/incorrect answers
    self.point_pl = 4
    self.point_mn = 1


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
    #call(['bash', '/home/cultureid_user0/game_desktop_launchers/kill_all.sh'])

    # This is necessary so that control is relinquished to class AMTHGames.
    # Who would have thought
    self.root.quit()
    rospy.sleep(0.5)
    self.root.destroy()

    rospy.logerr('This game is over')
    #os._exit(os.EX_OK)


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
    #canvas.pack(fill=Tkinter.BOTH,expand=True)
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
  def quit_group(self, group):

    # How many answered correctly
    ac = self.state[1][group]

    # How many questions in total
    tq = len(self.Q[group])

    # This many questions answered (correct + skipped)
    self.state[1][group] = tq

    # This many incorrect answers (skipped = incorrect)
    self.stats[1][group] += tq-ac

    # These many points
    self.stats[2][group] = self.point_pl*ac-self.point_mn*self.stats[1][group]

    # Game over my friend
    self.game_over(group)


  ##############################################################################
  def response_to_restore_game(self, c):
    if c == 0:
      self.restore_state()

      # Set pose; from saved file
      self.pose_ = self.make_pose_msg(self.in_pose)

      self.groups_clock_start -= rospy.Duration.from_sec(self.in_stats[3])

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

    # Correct answers [0], incorrect answers [1], points [2] and
    # game duration [3] per group

    self.stats = [[],[],[],[]]
    self.stats[0] = [0] * len(self.Q)
    self.stats[1] = [0] * len(self.Q)
    self.stats[2] = [10] * len(self.Q)
    self.stats[3] = 0


  ##############################################################################
  def save_state_to_file(self):

    self.groups_clock_stop = rospy.Time.now()
    self.stats[3] = (self.groups_clock_stop-self.groups_clock_start).to_sec()

    self.write_file(self.compile_statestring(), self.statefile)


  ##############################################################################
  def select_group_init(self):

    # to frame panw sto opoio 8a einai ta koumpia
    frame = self.new_frame()
    self.set_frame(frame)


    # Group buttons ------------------------------------------------------------
    buttonVec = []
    buttonText = []

    for i in range(0,len(self.Q)):
      this_butt = Tkinter.Button(frame,text='???',fg='#E0B548',bg='#343A40',activeforeground='#E0B548',activebackground='#343A40', command=partial(self.show_intro_video_play_button,i))
      buttonVec.append(this_butt)
      this_group_name = 'ΟΜΑΔΑ %d\n%d πόντοι' % ((i+1), self.stats[2][i])
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
          buttonVec[counter].config(font=("Helvetica", 30))
          buttonVec[counter].update()

        counter = counter+1


    # EXIT button --------------------------------------------------------------
    exit_button = Tkinter.Button(frame,text='???',fg='#E0B548',bg='#343A40',activeforeground='#E0B548',activebackground='#343A40', command=self.kill_root)
    buttonVec.append(exit_button)
    buttonText.append('ΕΞΟΔΟΣ')

    xEff = 1.0
    yEff = 0.3

    GP = 0.1

    xNum = 2
    yNum = yNum+1
    xx = 0

    xWithGuard = xEff/xNum
    xG = GP*xWithGuard
    xB = xWithGuard-xG

    yWithGuard = yEff/yNum
    yG = GP*yWithGuard
    yB = yWithGuard-yG

    thisX = xG/2+xx*xWithGuard+1-xEff
    thisY = yG/2+yy*yWithGuard+1-yEff+GP

    buttonVec[counter].place(relx=thisX,rely=thisY,relheight=yB,relwidth=xB)
    buttonVec[counter].config(text=buttonText[counter])
    buttonVec[counter].update()

    thisWidth = buttonVec[counter].winfo_width()
    thisHeight = buttonVec[counter].winfo_height()
    buttonVec[counter].config(font=("Helvetica", 30))
    buttonVec[counter].update()



    # HELP button --------------------------------------------------------------
    help_button = Tkinter.Button(frame,text='???',fg='#E0B548',bg='#343A40',activeforeground='#E0B548',activebackground='#343A40', command=self.show_help)
    buttonVec.append(help_button)
    buttonText.append('ΚΑΝΟΝΕΣ')

    xx = xx + 1
    counter = counter + 1

    xWithGuard = xEff/xNum
    xG = GP*xWithGuard
    xB = xWithGuard-xG

    yWithGuard = yEff/yNum
    yG = GP*yWithGuard
    yB = yWithGuard-yG

    thisX = xG/2+xx*xWithGuard+1-xEff
    thisY = yG/2+yy*yWithGuard+1-yEff+GP

    buttonVec[counter].place(relx=thisX,rely=thisY,relheight=yB,relwidth=xB)
    buttonVec[counter].config(text=buttonText[counter])
    buttonVec[counter].update()

    thisWidth = buttonVec[counter].winfo_width()
    thisHeight = buttonVec[counter].winfo_height()
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
  # show rules
  def show_help(self):

    call(['vlc', '--no-repeat','--fullscreen','--play-and-exit', \
        self.rules_dir + '/rules.mp4'])


  ##############################################################################
  def show_intro_video(self, group):

    call(['vlc', '--no-repeat','--fullscreen','--play-and-exit', \
        self.dir_media + '/intro_' + str(group) + '.mp4'])

    self.intro_played[group] = True
    self.game(group)


  ##############################################################################
  def show_intro_video_play_button(self, group):

    self.previous_group = group


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
