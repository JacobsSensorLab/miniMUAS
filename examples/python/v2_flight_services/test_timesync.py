"""Dashboard clock offsets: relative to the GCS, with or without absolute time.

Run: python3 -m unittest test_timesync  (from this directory)
"""

from __future__ import annotations

import unittest

from timesync import ClockReading, UNKNOWN, offset_vs_local_ms, parse_tracking_csv

# `chronyc -n -c tracking` on a GCS serving its own clock (`local stratum 10`,
# no reachable upstream): the isolated-field case of 2026-10-13. Reference ID
# 7F7F0101 has no printable name, so field 1 is empty.
GCS_LOCAL_REF = (
    "7F7F0101,,10,1791923727.000000000,0.000000000,0.000000000,0.000000000,"
    "-51.862,0.000,0.000,0.000000000,0.000000000,0.0,Normal\n"
)
GCS_POOL_REF = (
    "2D21675E,45.33.103.94,4,1791842734.000000000,-0.000106398,0.000047718,"
    "0.000285626,-51.862,0.038,0.303,0.029524863,0.006082010,256.1,Normal\n"
)
LOCAL = "127.0.0.1"      # an address of this host, like the GCS's 192.168.1.13
NOT_LOCAL = "192.0.2.1"  # TEST-NET-1: never assigned here


class OffsetVsLocalTest(unittest.TestCase):
    def test_gcs_local_reference_parses_as_unknown(self):
        self.assertFalse(parse_tracking_csv(GCS_LOCAL_REF).known)

    def test_drone_synced_to_us_needs_no_absolute_time(self):
        # The field: drones on the GCS, GCS on its hand-set clock.
        ours = parse_tracking_csv(GCS_LOCAL_REF)
        self.assertEqual(offset_vs_local_ms(LOCAL, 0.05, ours), 0.05)

    def test_drone_synced_to_us_ignores_our_upstream_offset(self):
        # node - us is the drone's own reading; our offset from the pool
        # must not be subtracted from it.
        ours = parse_tracking_csv(GCS_POOL_REF)
        self.assertAlmostEqual(ours.offset_ms, 0.106398)
        self.assertEqual(offset_vs_local_ms(LOCAL, -0.2, ours), -0.2)

    def test_common_upstream_differences_the_offsets(self):
        ours = ClockReading("45.33.103.94", 0.3, 0.1)
        self.assertAlmostEqual(offset_vs_local_ms("45.33.103.94", 1.0, ours), 0.7)

    def test_unrelated_or_unknown_reference_is_none(self):
        ours = ClockReading("45.33.103.94", 0.3, 0.1)
        self.assertIsNone(offset_vs_local_ms(NOT_LOCAL, 1.0, ours))
        self.assertIsNone(offset_vs_local_ms(NOT_LOCAL, 1.0, UNKNOWN))
        self.assertIsNone(offset_vs_local_ms("", 0.0, ours))
        self.assertIsNone(offset_vs_local_ms("not-an-ip", 0.0, ours))


if __name__ == "__main__":
    unittest.main()
