def add(a: float, b: float) -> float:
    """Calculates the sum of two numbers."""
    return a + b

import unittest

class TestAdd(unittest.TestCase):
    def test_add(self):
        self.assertEqual(add(1, 2), 3)
        self.assertEqual(add(-1, 1), 0)
        self.assertAlmostEqual(add(0.1, 0.2), 0.3)

if __name__ == '__main__':
    unittest.main()
