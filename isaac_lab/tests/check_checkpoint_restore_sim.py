"""Run train CLI while checking restored buffers BEFORE any resumed physics step."""
from pathlib import Path
import runpy
import sys
import torch

root=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(root/'isaac_lab'));sys.path.insert(0,str(root))
ns=runpy.run_path(str(root/'isaac_lab/train.py'),run_name='checkpoint_restore_test')
from isaac_lab.checkpoint_state import FlatEnvState, EPHEMERAL
original=FlatEnvState.restore


def equal_tree(a,b,path):
    if isinstance(b,torch.Tensor):
        torch.testing.assert_close(a.cpu(),b.cpu(),rtol=0,atol=1e-6,equal_nan=True,msg=path)
    elif isinstance(b,dict):
        for key in b:
            if key not in EPHEMERAL:equal_tree(a[key],b[key],path+'.'+key)
    else:
        for i,v in enumerate(b):equal_tree(a[i],v,path+'.'+str(i))


def checked_restore(self,state):
    result=original(self,state)
    current=self.capture()
    for key in ('scene','physics','objects','joint_targets','robot_previous_joint_vel'):
        equal_tree(current[key],state[key],key)
    assert current['common_step_counter']==state['common_step_counter']
    assert current['sim_step_counter']==state['sim_step_counter']
    print('CHECKPOINT_RESTORE_BUFFERS_PASS poses velocities targets physics commands episode action_history reward_sums contact_history counters',flush=True)
    return result


FlatEnvState.restore=checked_restore
try:
    ns['main']()
    print('CHECKPOINT_RESTORE_SIM_PASS',flush=True)
except BaseException:
    import traceback;traceback.print_exc();print('CHECKPOINT_RESTORE_SIM_FAIL',flush=True);raise
