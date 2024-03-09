#!/usr/bin/env python3
import numpy as np
import math

from fanuc import Fanuc


def test_draw_faunc():
  robot = Fanuc()

  input_values = np.array(
      [-math.pi / 6, math.pi / 5, -math.pi / 4, 1.25, -1.56, 7.25])
  robot.calculate_fk(input_values)


  starting_values = np.array([0, 0, 0, 0, 0, 0])

  test_path = "/home/lcfarrell/devel/ME_498_Staff/2024/labs/lab2/src/prism.yaml"

  robot.draw_fanuc_path(starting_values, test_path)


if __name__ == "__main__":
  # load_data()

  robot = Fanuc()
  robot.draw_fanuc(np.array([0, 0, 0, 0, 0, 0]))

  test_draw_faunc()
  