import importlib.util
from pathlib import Path
from types import ModuleType
import sys
from unittest.mock import patch
import unittest
import torch
ROOT=Path(__file__).resolve().parents[1]
manager=ModuleType('isaaclab.managers');manager.ManagerTermBase=object
foot=ModuleType('ground_test.foot_geometry');foot.FOOT_NAMES=('lf_shin','rf_shin','lb_shin','rb_shin');foot.load_sole_points=lambda *a,**k:None
spec=importlib.util.spec_from_file_location('ground_test.ground_contact',ROOT/'ground_contact.py')
module=importlib.util.module_from_spec(spec)
with patch.dict(sys.modules,{'isaaclab.managers':manager,'ground_test.foot_geometry':foot}):spec.loader.exec_module(module)

class GroundContactTests(unittest.TestCase):
    def setUp(self):
        self.triangle=torch.tensor([[[0.,0.,0.],[1.,0.,0.],[0.,1.,0.]]])
    def test_distance_surface_edges_vertices_and_degenerate(self):
        p=torch.tensor([[.2,.2,.001],[.6,.6,0.],[-.1,0,0.]])
        torch.testing.assert_close(module.surface_distance(p,self.triangle),torch.tensor([.001,2**.5*.1,.1]),atol=1e-6,rtol=1e-5)
        torch.testing.assert_close(module.surface_distance(torch.tensor([[1.,0,0]]),torch.zeros(1,3,3)),torch.tensor([1.]))
    def flags(self,counts,starts,points,gaps,triangles=None,position=None,q=None):
        n=len(counts)
        return module.forbidden_flags(torch.tensor(counts),torch.tensor(starts),torch.tensor(points,dtype=torch.float32).reshape(-1,3),
            torch.tensor(gaps),torch.zeros(n,3) if position is None else position,
            torch.tensor([[1.,0,0,0]]*n) if q is None else q,triangles)
    def test_zero_force_not_needed_and_gap_threshold(self):
        actual=self.flags([1,1,1,0],[0,1,2,0],[[0,0,0]]*3,[-.001,.0005,.002])
        self.assertEqual(actual.tolist(),[True,True,False,False])
    def test_mixed_sole_and_side_contacts_are_not_averaged(self):
        actual=self.flags([2,1],[1,0],[[.2,.2,0],[.2,.2,0],[.2,.2,.02]],[0,0,0],self.triangle)
        self.assertEqual(actual.tolist(),[True,False])
    def test_empty_buffer(self):
        self.assertEqual(self.flags([0,0],[0,0],[],[],self.triangle).tolist(),[False,False])
    def test_rigid_transform_does_not_change_classification(self):
        position=torch.tensor([[1.,2.,3.]])
        q=torch.tensor([[2**-.5,0,0,2**-.5]])
        actual=self.flags([1],[0],[[.8,2.2,3]],[0],self.triangle,position,q)
        self.assertEqual(actual.tolist(),[False])
    def test_invalid_points_do_not_create_false_contact(self):
        self.assertEqual(self.flags([1],[0],[[float('nan'),0,0]],[0]).tolist(),[False])

if __name__=='__main__':unittest.main()
