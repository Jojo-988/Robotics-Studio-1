import unittest
import numpy as np
from landmark_fit import fit_round, spatial_clusters


class FitTests(unittest.TestCase):
    def test_clusters_do_not_merge_separate_objects(self):
        p=np.array([[0,0,0],[.05,0,1],[1,1,0],[1.05,1,0]])
        self.assertEqual(sorted(map(len,spatial_clusters(p))),[2,2])

    def test_partial_circle(self):
        a=np.linspace(.2,2.8,60)
        p=np.column_stack((4+.18*np.cos(a),-4+.18*np.sin(a)))
        r=fit_round(p)
        np.testing.assert_allclose(r['centre'],[4,-4],atol=1e-8)
        self.assertAlmostEqual(r['radius'],.18)

    def test_partial_sphere(self):
        a,b=np.meshgrid(np.linspace(.1,2.9,15),np.linspace(.4,2.3,12))
        p=np.column_stack((np.sin(b).ravel()*np.cos(a).ravel(),np.sin(b).ravel()*np.sin(a).ravel(),np.cos(b).ravel()))*.35+[-4,4,.25]
        np.testing.assert_allclose(fit_round(p)['centre'],[-4,4,.25],atol=1e-8)

    def test_line_rejected(self):
        with self.assertRaises(ValueError):fit_round(np.column_stack((np.arange(20),np.zeros(20))))

    def test_short_arc_rejected(self):
        a=np.linspace(0,.2,30)
        with self.assertRaises(ValueError):fit_round(np.column_stack((np.cos(a),np.sin(a))))

    def test_nan_rejected(self):
        with self.assertRaises(ValueError):fit_round(np.full((20,2),np.nan))
