# tests/test_state_machine.py
import unittest
from ast_work.statemachine import MonitorBatteryAndCollision

class TestStateMachine(unittest.TestCase):
    def setUp(self):
        self.sm = MonitorBatteryAndCollision()

    def test_initial_state(self):
        self.assertEqual(self.sm.current_state, 'initial')

    def test_state_transition(self):
        self.sm.transition_to('next_state')
        self.assertEqual(self.sm.current_state, 'next_state')

    def test_invalid_transition(self):
        with self.assertRaises(ValueError):
            self.sm.transition_to('invalid_state')

if __name__ == '__main__':
        unittest.main()
