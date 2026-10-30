"""Uniform-density LiDAR enclosed-area CG tests."""
from __future__ import annotations

import math
import unittest

from nd_sociomind.experiment.autonomous_navigation.follow_scenario.data.processing.polygon_area_centroid import polygon_area_centroid


class PolygonAreaCentroidTest(unittest.TestCase):
    def test_square(self):
        square = [(0., 0.), (4., 0.), (4., 2.), (0., 2.)]
        self.assertEqual(polygon_area_centroid(square), (2., 1.))
        self.assertEqual(polygon_area_centroid(square[::-1]), (2., 1.))

    def test_concave_uniform_mass_not_vertex_average(self):
        # An L shape with total area 5 m^2 has centroid (1.1, 1.1).
        polygon = [(0., 0.), (3., 0.), (3., 1.),
                   (1., 1.), (1., 3.), (0., 3.)]
        x, y = polygon_area_centroid(polygon)
        self.assertAlmostEqual(x, 1.1)
        self.assertAlmostEqual(y, 1.1)
        self.assertNotAlmostEqual(x, sum(p[0] for p in polygon) / len(polygon))

    def test_degenerate_and_invalid(self):
        self.assertIsNone(polygon_area_centroid([]))
        self.assertIsNone(polygon_area_centroid([(0., 0.), (1., 1.), (2., 2.)]))
        self.assertIsNone(polygon_area_centroid([(0., 0.), (1., 0.), (math.nan, 1.)]))

    def test_remote_translated_polygon(self):
        corners = [(1000000., 4000000.), (1000002., 4000000.),
                   (1000002., 4000002.), (1000000., 4000002.)]
        self.assertEqual(polygon_area_centroid(corners), (1000001., 4000001.))


if __name__ == "__main__":
    unittest.main()
