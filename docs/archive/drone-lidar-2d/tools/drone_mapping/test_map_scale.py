import unittest
import numpy as np
from check_map_scale import fit_wall, cell_centres

class ScaleTests(unittest.TestCase):
    def test_vertical_line(self):
        y=np.linspace(-2,8,100)
        fit=fit_wall(np.column_stack([12.5+.01*y,y]),0)
        self.assertAlmostEqual(fit['slope'],.01)
        self.assertAlmostEqual(fit['intercept_m'],12.5)

    def test_horizontal_line(self):
        x=np.linspace(-10,10,200)
        fit=fit_wall(np.column_stack([x,np.full(200,12.5)]),1)
        self.assertAlmostEqual(fit['intercept_m'],12.5)

    def test_insufficient_support(self):
        with self.assertRaises(ValueError): fit_wall(np.zeros((20,2)),0)

    def test_cell_centres_with_rotated_origin(self):
        m={'info':{'width':2,'height':1,'resolution':1,
           'origin':{'position':{'x':10,'y':20},'orientation':{'x':0,'y':0,'z':np.sqrt(.5),'w':np.sqrt(.5)}}},'data':[0,100]}
        _,x,y=cell_centres(m)
        np.testing.assert_allclose(x,[[9.5,9.5]])
        np.testing.assert_allclose(y,[[20.5,21.5]])

if __name__=='__main__':unittest.main()
