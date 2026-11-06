import unittest
import numpy as np
from extremebench.agents import PlanningAgent,ConstantVelocityAgent
class AgentTests(unittest.TestCase):
 def test_interface_is_abstract(self):
  with self.assertRaises(TypeError):PlanningAgent()
 def test_constant_velocity_units_and_shape(self):
  p=ConstantVelocityAgent().predict({'language':'Speed: 10.00 m/s. Instruction: keep lane.'})
  self.assertEqual(p.shape,(8,3));np.testing.assert_array_equal(p[:,0],np.arange(5,41,5));np.testing.assert_array_equal(p[:,1:],np.zeros((8,2)))
