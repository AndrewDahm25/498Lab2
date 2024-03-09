import numpy as np
import math
import PyKDL as kdl
import yaml


def check_proper_numpy_format(value: np.ndarray, shape: tuple) -> bool:
  """Send in a numpy array to make sure it is the right shape 

  Args:
      value (np.ndarray): Input value to test
      shape (tuple): desired shape as a tuple ex (4,4)

  Returns:
      bool: If True, the array is well formed 
  """
  if not isinstance(value, np.ndarray):
    return False

  if value.shape != shape:
    return False

  return True


def np_frame_to_kdl(np_frame: np.ndarray) -> kdl.Frame:
  """Turn a numpy 4x4 array into a KDL frame 
  NOTE: It does NOT check that the 4x4 array is a proper transformation

  Args:
      np_frame (np.ndarray): Input numpy array of shape 4x4

  Returns:
      kdl.Frame: KDL version of that transformation
  """

  if not check_proper_numpy_format(np_frame, (4, 4)):
    raise TypeError("Incorret type or size to translate into a KDL Frame")

  kdl_frame = kdl.Frame()
  for i in range(3):
    for j in range(3):
      kdl_frame.M[i, j] = np_frame[i, j]
    kdl_frame.p[i] = np_frame[i, 3]

  return kdl_frame


def kdl_rotation_to_np(rotation: kdl.Rotation) -> np.ndarray:
  """Convert a KDL rotation to a 3x3 numpy aray 

  Args:
      rotation (kdl.Rotation): KDL Rotation to convert 

  Raises:
      TypeError: if an invalid type is sent in as input

  Returns:
      np.ndarray: numpy array shape 3x3
  """
  if not isinstance(rotation, kdl.Rotation):
    raise TypeError("Must send in a rotation")

  np_matrix = np.eye(3)
  np_matrix[0:3, 0] = [value for value in rotation.UnitX()]
  np_matrix[0:3, 1] = [value for value in rotation.UnitY()]
  np_matrix[0:3, 2] = [value for value in rotation.UnitZ()]

  return np_matrix


def kdl_frame_to_np(frame: kdl.Frame) -> np.ndarray:
  """Convert a KDL frame to numpy frame

  Args:
      frame (kdl.Frame): Frame to convert to numpy

  Raises:
      TypeError: If the wrong type is sent in

  Returns:
      np.ndarray: 4x4 Transformation matrix in numpy
  """

  if not isinstance(frame, kdl.Frame):
    raise TypeError("Must send in KDL Frame")

  translation_vector = np.array([frame.p.x(), frame.p.y(), frame.p.z()])

  np_matrix = np.eye(4)

  np_matrix[0:3, 0:3] = kdl_rotation_to_np(frame.M)
  np_matrix[:3, 3] = translation_vector

  return np_matrix


def get_data_from_yaml(filepath: str) -> dict:
  """Get the data out of a yaml file in dictionary form

  Args:
      filepath (str): full filepath to the location on your computer
                      eg: "/home/lcfarrell/ME_498/Lab2/pahts/path_name.yaml"

  Returns:
      dict: dictionary in the format {"color": int, "x": float, "y": float, "z": float}
  """

  with open(filepath, "r") as yaml_file:
    data = yaml.safe_load(yaml_file)
  return data
