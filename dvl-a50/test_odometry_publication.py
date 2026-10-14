"""Exercise odometry publication timing without starting threads or sending MAVLink."""

import unittest
from unittest.mock import Mock, patch

from test_dvl import dvl
from test_range_filter import frame


class OdometryPublicationTest(unittest.TestCase):
    def setUp(self):
        self.driver = dvl.DvlDriver()
        self.driver.mav = Mock()
        self.driver.rangefinder = False

    def publish(self, arrival, valid=True):
        sample = dict(
            frame([2] * 4),
            vx=0.1,
            vy=0,
            vz=0,
            velocity_valid=valid,
            fom=0,
            time=100,
            time_of_transmission=arrival * 1e6,
        )
        with patch("dvl.time.time", return_value=arrival), patch("dvl.time.monotonic", return_value=arrival):
            self.driver.handle_velocity(sample)

    def test_normal_rates_and_jitter_publish_every_sample(self):
        for rate in (5, 9, 10, 15):
            with self.subTest(rate=rate):
                self.setUp()
                for i in range(40):
                    self.publish(100 + i / rate + (0.008 if i % 2 else 0))
                self.assertEqual(self.driver.mav.send_odometry.call_count, 40)

    def test_reports_closer_than_100_ms_are_not_skipped(self):
        # Recorded A50 report intervals around 9 Hz include ~95 ms spacing.
        arrival = 100.0
        for interval in (0.095, 0.105, 0.095, 0.120, 0.095):
            self.publish(arrival)
            arrival += interval
        self.assertEqual(self.driver.mav.send_odometry.call_count, 5)

    def test_fast_input_remains_rate_limited(self):
        sent_at = []
        for i in range(200):
            arrival = 100 + i / 1000
            before = self.driver.mav.send_odometry.call_count
            self.publish(arrival)
            if self.driver.mav.send_odometry.call_count > before:
                sent_at.append(arrival)
        self.assertGreater(len(sent_at), 1)
        self.assertTrue(all(b - a >= 0.05 for a, b in zip(sent_at, sent_at[1:])))

    def test_invalid_velocity_is_not_published(self):
        self.publish(100, valid=False)
        self.driver.mav.send_odometry.assert_not_called()


if __name__ == "__main__":
    unittest.main()
