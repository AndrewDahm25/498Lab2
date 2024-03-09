import numpy as np
import math
import PyKDL as kdl
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from typing import Tuple, List
from copy import deepcopy

import general_utility as general
from robot_components import Brush, Link, Joint


def dh_tf(alpha: float, a: float, d: float, theta: float) -> np.ndarray:
  """Create a DH transform using alpha, a, d, and theta

  TODO -- you will need to fill this function in! 

  Args:
      alpha (float): alpha angle in radians
      a (float): a in mm
      d (float): d in mm
      theta (float): theta in radians

  Returns:
      np.ndarray: (4,4) numpy array of the transformation matrix
  """
  return 


def make_frame(rotation: np.ndarray, translation: np.ndarray) -> np.ndarray:
  """Create a tool transform using rotation and translation via ZYX Euler Trans.

  Args:
      rotation (np.ndarray): numpy 3x1 array of rotations
      translate (np.ndarray): numpy 3x1 array of translations

  Returns:
      np.ndarray: 4x4 numpy transformation array
  """
  rot_mat = kdl.Rotation()
  rot_mat.DoRotZ(rotation[2])
  rot_mat.DoRotY(rotation[1])
  rot_mat.DoRotX(rotation[0])
  frame = kdl.Frame(rot_mat, kdl.Vector(*translation))

  return general.kdl_frame_to_np(frame)


class Workspace(object):
  def __init__(self, x_min: float, x_max: float, y_min: float, y_max: float,
               z_min: float, z_max: float):
    """Initialize the workspace 

    Args:
        x_min (float): min x of the worksapce in mm
        x_max (float): max x of the workspace in mm 
        y_min (float): min y of the workspace in mm
        y_max (float): max y of the workspace in mm
        z_min (float): min z of the workspace in mm
        z_max (float): max z of the workspace in mm
    """
    self.x_min = x_min
    self.x_max = x_max
    self.y_min = y_min
    self.y_max = y_max
    self.z_min = z_min
    self.z_max = z_max


class Fanuc(object):
  """Fanuc class to hold the information about the Fanuc Arm """
  @property
  def joints(self) -> List[Joint]:
    return [
        self._joint_1, self._joint_2, self._joint_3, self._joint_4,
        self._joint_5, self._joint_6
    ]

  @property
  def ee_frame(self) -> np.ndarray:
    """Returns the position of the end effector given the joint transforms 

    Note, this is an inefficient call as it recalculates each time. If you want to 
    be fancy and make your code run faster, you can re-write this. 

    Returns:
        np.ndarray: (4,4) of the final location of the end effector. 
    """
    ee_frame = self._joint_1.dh_transform @ self._joint_2.dh_transform @ self._joint_3.dh_transform @ self._joint_4.dh_transform @ self._joint_5.dh_transform @ self._joint_6.dh_transform
    return ee_frame

  def __init__(self):
    """Initialize the class """

    self._joints = []
    self._drawn_once = False

    ## TODO -- fill in fanuc physical parameters
    ## Fanuc link lengths in millimeters
    self.a_1 = ...
    self.l_1_z = ...

    ## TODO -- create the workspace boundaries. 
    self.workspace = Workspace(...)

    # Initialzie the colors for drawing links. Feel free to change if you'd like.
    self.colors = [
        [0, 0, 0],
        [1, 0, 0],
        [0, 1, 0],
        [0, 0, 1],
        [0, 1, 1],
        [1, 1, 0],
        [1, 0, 1],
    ]

    ## Create the base figure to pass to the joints and links
    self._create_plot()

    ## Create the brush
    self.brush = Brush(self.ax)

    ## Make the base frame
    base_frame = make_frame([0, 0, 0], [0, 0, self.l_1_z])  #self.l_1_z
    self._base_frame = Joint(self.ax)
    self._base_frame.set_final_transform(base_frame)
    self._base_frame.set_dh_transform(base_frame)

    self._zero_frame = Joint(self.ax)
    self._zero_frame.set_dh_transform(np.eye(4))
    self._zero_frame.set_final_transform(np.eye(4))

    # Initialize and create the joints
    self._joint_1: Joint = None
    self._joint_2: Joint = None
    self._joint_3: Joint = None
    self._joint_4: Joint = None
    self._joint_5: Joint = None
    self._joint_6: Joint = None

    self._setup_joints()

    self._links = [Link(self.ax, self.colors[index]) for index in range(6)]

  def _setup_joints(self):
    """Use this area to initialize the joints.
    TODO 
    I have provided the start of the first joint. You need to
    - fill in the limits for the first joint. 
    - fill in all of the information for the remaining joints. 

    This will also help you get comfortable with initializing classes. 
    """

    self._joint_1 = Joint(self.ax, self.colors[0])
    self._joint_1.set_joint_limits(math.radians(...), math.radians(...))
    ...

    for joint in self.joints:
      joint.draw()

  def calculate_fk(self, joint_angles: np.ndarray):
    """Calculate the forward kinematics of the fanuc. 

    TODO 

    In this function, you need to load the joint DH parameters into each joint. 
    This will then allow you to call self.ee_frame to get the final output 
    frame of the robot, or do any math using joint_X.dh_transform. 
    Hint: This will use your created dh_tf() 
    Hint: You should be setting the transform here for all 6 joints. 

    Args:
        joint_angles (np.ndarray): (1,6) array of the joint angles 
    """

    self._joint_1.set_dh_transform(...)
    ...


  def calculate_ik(self, ee_frame: np.ndarray,
                    prev_joint_angles: np.ndarray) -> Tuple[bool, np.ndarray]:
    """calculate the inverse kinematics of the fanuc
    TODO
    This is the hard part of the whole thing. If you get this working, it's all downhill.
    Be careful, the devil is in the details. Be sure to include all solution cases. 
    Be sure to not make large leaps in configuration. 
    Be sure to test your code well. The auto-grader will certainly test many configurations of 
    your robot to ensure it can match them. 

    Args:
        ee_frame (np.ndarray): The desired location of the end effector in space as a 4x4 frame
        prev_joint_angles (np.ndarray): The previous joint angles (hint minimize the difference)

    Returns:
        Tuple[bool, np.ndarray]: bool -- whether or not a solutio exists
                                 np.ndarray -- the 6x1 array of that solution if it exists 
    """
    return False, []

  def get_ee_pose_from_brush(self, rotation: np.ndarray,
                             brush_pose: np.ndarray) -> np.ndarray:
    """Get the end effector pose from the desired pose of the brush. 

    TODO
    Note: If you are confused why this blank function is here. Remember that the 
    test drawing files, and your drawing files, are the location of the Brush tip, NOT the 
    end effector frame. Therefore, you need to calculate that EE frame from the brush frame 
    to put into your IK solution. 
    (If your tetra or prism isn't continuous, it's because this isn't working right)

    Args:
        rotation (np.ndarray): (3,3) rotation matrix for the brush
        brush_pose (np.ndarray): 1x3 position in space of the brush (x,y,z)

    Returns:
        np.ndarray: (4,4) location of the end effector to get the brush at the desired location
    """
    general.check_proper_numpy_format(rotation, (3, 3))
    general.check_proper_numpy_format(brush_pose, (3, ))

    return ...

  def draw_fanuc_path(self, starting_angles: np.ndarray, path: str) -> None:
    """Draw the fanuc moving through a desired path
    TODO -- you must fill this out to draw the fanuc from the start point 
    to the end point of the path. 

    Args:
        starting_angles (np.ndarray): 1x6 starting angles of the robot 
        path (str): full length path from your home directory to the file 
                    eg "/home/lcfarrell/ME_498/lab2/test_paths/prism.yaml"
    """
    data = general.get_data_from_yaml(path)

    

  def _create_plot(self):
    """Initialize the plot to use throughout
        No change necessary, setup function provided"""
    self.fig = plt.figure(figsize=(8, 8), facecolor='w')
    self.ax = self.fig.add_subplot(111, projection='3d')
    plt.xlim([self.workspace.x_min, self.workspace.x_max])
    plt.ylim([self.workspace.y_min, self.workspace.y_max])
    self.ax.set_zlim([self.workspace.z_min, self.workspace.z_max])
    self.ax.set_xlabel('X (mm)', fontsize=16)
    self.ax.set_ylabel('Y (mm)', fontsize=16)
    self.ax.set_zlabel('Z (mm)', fontsize=16)
    # self.ax.view_init(elev=22.8, azim=147.3)
    plt.grid(True)

  def initialize_fanuc_drawing(self, joint_angles: np.ndarray) -> None:
    """Initialize the drawing to ensure it draws properly
    No change needed, used to setup the drawing capability

    Args:
        joint_angles (np.ndarray): 6x1 array of joint angles desired. 
    """
    self.calculate_fk(joint_angles)
    plt.show(block=False)
    plt.pause(0.5)

  def draw_fanuc(self, joint_angles: np.ndarray) -> None:
    """Draw the fanuc in the provided configuration. 
    Note, this addes the distance from frame 0 to the mounting frame. 
    No change needed, provided drawing capability 

    Args:
        joint_angles (np.ndarray): 6x1 array of the desired joint angles. 
    """
    general.check_proper_numpy_format(joint_angles, (6, ))

    self.calculate_fk(joint_angles)
    if not self._drawn_once:
      self.initialize_fanuc_drawing(joint_angles)
      self._drawn_once = True

    self._base_frame.draw()
    self._zero_frame.draw()

    current_transform = self._base_frame.dh_transform

    for index, joint in enumerate(self.joints):
      prev_transform = current_transform
      current_transform = current_transform @ joint.dh_transform
      joint.set_final_transform(current_transform)
      joint.draw()
      self._links[index].update_frames(prev_transform, current_transform)
      self._links[index].draw()

    # draw the brush at the end
    self.brush.update_tool_frame(current_transform)
    self.brush.show_enabled()
    self.brush.paint()

    plt.pause(0.0001)