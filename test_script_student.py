#!/usr/bin/env python3
import numpy as np
import math
import time

from fanuc_solution import Fanuc


def test_draw_faunc():
  robot = Fanuc()

  input_values = np.array(
      [-math.pi / 6, math.pi / 5, -math.pi / 4, 1.25, -1.56, 7.25])
  robot.calculate_fk(input_values)


  starting_values = np.array([0, 0, 0, 0, 0, 0])

  test_path = "/home/lcfarrell/ME_498_Python/lab2/src/prism.yaml"

  robot.draw_fanuc_path(starting_values, test_path)


if __name__ == "__main__":
  # load_data()

  robot = Fanuc()
  robot.draw_fanuc(np.array([math.pi/3, 0, 0, 0, 0, 0]))

  time.sleep(1.0)

  robot.brush.selection = 2
  robot.draw_fanuc(np.array([math.pi/3, 0, 0, 0, 0, 0]))

  time.sleep(1.0)

  robot.brush.selection = 1
  robot.draw_fanuc(np.array([math.pi/3, 0, 0, 0, 0, 0]))

  time.sleep(1.0)

  robot.brush.selection = 3
  robot.draw_fanuc(np.array([math.pi/3, -math.pi/4, 0, 0, 0, 0]))

  time.sleep(1.0)

  test_draw_faunc()
  