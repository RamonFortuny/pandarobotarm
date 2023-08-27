#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Float64, Int32
import time
import re
import serial

class GripperStaticUnknown(object):
  def __init__(self):
    #Params
    self.loop_rate = rospy.Rate(10)

    self.port_name = "/dev/ttyACM0"
    self.BAUD = 500000
    self.PORT_TIMEOUT = 0.5
    self.PORT_WRITE_TIMEOUT=1

    self.serial_port = serial.Serial(self.port_name,self.BAUD,timeout=self.PORT_TIMEOUT,write_timeout=self.PORT_WRITE_TIMEOUT)
    self.serial_port.reset_input_buffer()

    self.measurements = 0.0 #R This line initializes an empty list named self.measurements to store measurements received from the gripper actuator.
    # self.objAttachedCount = [] #R This line initializes an empty list named self.objAttachedCount to track the attachment status of objects.
    self.initialized = False #R It indicates whether the gripper actuator has been initialized.
    self.gripperOpen = False #R It indicates whether the gripper is currently open or closed.
    # self.getData = False #R It determines whether data should be retrieved from the gripper actuator.
    self.objAttachedSys = False #R It represents the system-level object attachment status.
    self.msg2Controller = 0 #R It holds messages to be published to the controller.
    # "Ready!" - 1
    # "Object Grasped" - 2
    # "Object lost" - 3
    # "Gripper opened" - 4


    # self.last_line = "" #R It stores the last line of data received from the gripper actuator.
    self.action = "" #R It stores the current action or command received from the controller.
    self.msg2Gaussian = 0.0 #R  It represents a value to be published to the Gaussian node.
    # self.tic = 0 #R It serves as a reference time for measuring elapsed time.

    #Subscribers
    rospy.Subscriber("/controller2gripper", String, self.callback,queue_size=10) #R  It listens for commands sent to the gripper actuator and invokes the callback method.
    
    #Publishers
    self.pubGripper2Controller = rospy.Publisher('/gripper2controller', Int32, queue_size=10)
    self.pubGripper2Gaussian = rospy.Publisher('/gripper2gaussian', Float64, queue_size=10)

  
  def send(self, string):
    try:
      self.serial_port.reset_input_buffer()  # Clear the input buffer
      self.serial_port.write(string.encode())
      response = self.serial_port.readline().decode()  # Read a line from the serial port
      return response.strip()  # Remove leading/trailing white space
    except serial.SerialTimeoutException:
      print("Send failed!")
      return False
    except serial.SerialException:
          print("SensorInterface - Lost conenction!")



  '''
  Actual decision making process
  '''
  def switch(self, lang): #R Responsible for making decisions based on the received command (lang).
    toc = time.perf_counter()
    
    #print(self.objAttachedCount)
    
    #Start ActionPlan
    if lang == "Are you ready?" and not self.initialized:
      self.send("init\n")
      time.sleep(0.01)
      # self.send("sub-clear\n")
      # time.sleep(0.01)
      # self.send("sub S1 force all\n") #To subscribe to S1 (force)
      # time.sleep(0.01)
      # self.send("sub S2 force all\n")
      # time.sleep(0.01)
      # self.send("pub-on\n") #R To publish
      self.initialized = True
      self.msg2Controller = 1

    else:
      #Ready to grasp
      if lang == "Ready to grasp" and self.gripperOpen:
        # print("lang:", lang)
        # print("self.gripperOpen:", self.gripperOpen)
        # time.sleep(0.4)
        # self.send("ctrl stop\n")
        self.send("grip 2\n")
        time.sleep(0.2)
        self.send("grip 2\n")
        time.sleep(0.2)
        self.send("grip 2\n")
        time.sleep(2)
        self.gripperOpen = False

        #self.measurements =float(self.send("tact force\n"))
        response = self.send("tact force\n")
        try:
            self.measurements = float(response)
        except ValueError:
            print(f"Received unexpected response: {response}")

        if self.measurements > 0.03:
          self.objAttachedSys = True
          self.msg2Controller = 2
        else:
          self.objAttachedSys = False
          rospy.logwarn("No object found.")
          self.msg2Controller = 3
        rospy.loginfo(self.measurements)

        

      #Open Gripper
      if lang == "Open gripper" and not self.gripperOpen:
        self.openGripper()


     
  '''
  Function to open the gripper
  '''
  def openGripper(self):
    time.sleep(0.4)
    self.send("ctrl stop\n")
    time.sleep(0.2)
    self.send("m open\n")
    time.sleep(1)
    rospy.loginfo("Gripper opened")
    #self.msg2Gaussian = 0.0
    # self.objAttachedCount = [0] * 20
    # self.objAttachedCount = []
    self.objAttachedSys = False
    # self.getData = False
    self.gripperOpen = True
    self.msg2Controller = 4
    

  '''
  Subscribes to the controller topic
  '''
  def callback(self, data):
    self.action = data.data

  '''
  Start function
  '''
  def start(self):
      while not rospy.is_shutdown():
        self.pubGripper2Controller.publish(self.msg2Controller)
        self.pubGripper2Gaussian.publish(self.msg2Gaussian)

        self.switch(self.action)
        self.loop_rate.sleep()

if __name__ == '__main__':
    rospy.init_node("gripper_static_unknown", anonymous=True)
    my_node = GripperStaticUnknown()
    my_node.start()
