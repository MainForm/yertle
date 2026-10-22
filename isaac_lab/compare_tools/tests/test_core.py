import ast
import json
import math
from pathlib import Path
import sys
import tempfile
import time
import unittest
from unittest.mock import patch
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from compare_tools.core import (Actor,CONTRACTS,Decoder,Feed,JsonlTail,REFERENCE,
                               RealFrame,gravity_quaternion,independent_observation)
from compare_tools.physics import advance

def obs(step=0,q=None,cmd=None):
    return independent_observation([.2,.3,.4],[.1,.2,.3],[0,0,-1],cmd or [0,0,0],
                                  REFERENCE if q is None else q,np.zeros(12),np.zeros(12),step)
def record(t=0,**kw):
    r=dict(kind='tick',t_s=t,mode='POLICY',cmd=[-.1,0,0],obs55=obs().tolist(),sample=1,
           body_velocity_forced_zero_test=True);r.update(kw);return r

class CompareTests(unittest.TestCase):
    def test_layout_and_reference(self):
        a=obs();self.assertEqual(a.shape,(55,));self.assertEqual(a.dtype,np.float32)
        np.testing.assert_array_equal(a[:3],[0,0,0]);np.testing.assert_array_equal(a[12:24],0)
        np.testing.assert_allclose(a[3:6],[.1,.2,.3]);np.testing.assert_array_equal(a[50:53],[0,1,0])
        np.testing.assert_allclose(a[53:],[1.7,.6])
    def test_independent_state_is_not_real_feedback(self):
        own_q=REFERENCE+np.arange(12,dtype=np.float32)*.01
        before=obs(q=own_q)
        real=RealFrame(0,'POLICY',np.zeros(3),obs(),0,True)
        real.obs[12:48]=100;real.obs[3:9]=200
        after=obs(q=own_q)
        np.testing.assert_array_equal(before,after)
    def test_measured_velocity_mode_preserves_gyro_and_own_linear_velocity(self):
        a=independent_observation([.2,.3,.4],[1,2,3],[0,0,-1],[0,0,0],REFERENCE,np.zeros(12),np.zeros(12),0,False)
        np.testing.assert_allclose(a[:6],[.2,.3,.4,1,2,3])
    def test_action_history_and_phase_are_independent(self):
        a=independent_observation([0]*3,[0]*3,[0,0,-1],[0]*3,REFERENCE,[0]*12,np.arange(12),25)
        np.testing.assert_array_equal(a[36:48],np.arange(12));self.assertNotEqual(float(a[48]),0)
    def test_relative_to_absolute_angles(self):
        a=obs();a[12:24]=np.arange(12)*.01
        f=RealFrame(0,'POLICY',np.zeros(3),a,0,True)
        np.testing.assert_allclose(f.q,REFERENCE+np.arange(12)*.01,atol=1e-7)
    def test_roll_pitch_inverse_has_no_fabricated_yaw(self):
        np.testing.assert_allclose(gravity_quaternion([0,0,-1]),[1,0,0,0])
        r=.2;p=-.3;g=[math.sin(p),-math.cos(p)*math.sin(r),-math.cos(p)*math.cos(r)]
        q=gravity_quaternion(g)
        expected=[math.cos(r/2)*math.cos(p/2),math.sin(r/2)*math.cos(p/2),math.cos(r/2)*math.sin(p/2),-math.sin(r/2)*math.sin(p/2)]
        np.testing.assert_allclose(q,expected,atol=1e-7)
    def test_missing_observation_does_not_invent_zero_pose(self):
        d=Decoder();d.accept(record(obs55=None));self.assertIsNone(d.frame.q)
    def test_invalid_record_erases_valid_frame(self):
        for bad in ([float('nan')]*55,[0]*54):
            d=Decoder();d.accept(record());d.accept(record(obs55=bad));self.assertIsNone(d.frame)
    def test_wrong_gait_and_command_are_rejected(self):
        d=Decoder();bad=obs();bad[53]=2.2;d.accept(record(obs55=bad.tolist()));self.assertIsNone(d.frame)
        d.accept(record(cmd=[1,0,0]));self.assertIsNone(d.frame)
    def test_metadata_and_session(self):
        d=Decoder();d.accept(dict(kind='policy_model_config',checkpoint_iteration='2400'))
        d.accept(dict(kind='policy_action',policy_session=3,episode_step=7));d.accept(record())
        self.assertEqual(d.frame.session,3);self.assertEqual(d.frame.episode_step,7)
    def test_model_mismatch_is_blocked(self):
        actor=Actor.__new__(Actor);actor.metadata={'yertle.checkpoint_sha256':'abc'}
        with self.assertRaises(ValueError):actor.check_cli(None)
        with self.assertRaises(ValueError):actor.check_cli({'contract':next(iter(CONTRACTS)),'checkpoint_sha256':'other'})
        actor.check_cli({'contract':next(iter(CONTRACTS)),'checkpoint_sha256':'abc'})
    def test_partial_json_and_utf8_tail(self):
        with tempfile.TemporaryDirectory() as t:
            p=Path(t)/'log.jsonl';data=(json.dumps({'kind':'event','message':'한글'},ensure_ascii=False)+'\n').encode()
            p.write_bytes(data[:-4]);tail=JsonlTail(p);self.assertEqual(tail.poll(),[])
            with p.open('ab') as f:f.write(data[-4:])
            self.assertEqual(tail.poll()[0]['message'],'한글');self.assertEqual(tail.poll(),[])
    def test_tail_truncation(self):
        with tempfile.TemporaryDirectory() as t:
            p=Path(t)/'log.jsonl';p.write_text(json.dumps(record())+'\n');tail=JsonlTail(p);tail.poll()
            p.write_text('{}\n');self.assertEqual(tail.poll(),[{}])
    def test_replay_recorded_time(self):
        with tempfile.TemporaryDirectory() as t:
            p=Path(t)/'log.jsonl';p.write_text(json.dumps(record(0))+'\n'+json.dumps(record(2,sample=2))+'\n')
            f=Feed(p,replay=True);f.poll();self.assertEqual(f.decoder.frame.sample,1)
            f.started-=3;f.poll();self.assertEqual(f.decoder.frame.sample,2)
    def test_live_age_includes_tick_age_not_just_other_log_events(self):
        with tempfile.TemporaryDirectory() as t:
            p=Path(t)/'log.jsonl';p.write_text(json.dumps(record())+'\n');f=Feed(p);f.poll()
            f.last_tick_at-=2
            with p.open('a') as s:s.write('{"kind":"event"}\n')
            f.poll();self.assertGreater(f.age(),1)
    def test_replay_simulation_clock_does_not_skip_on_slow_render(self):
        with tempfile.TemporaryDirectory() as t:
            p=Path(t)/'log.jsonl';p.write_text(json.dumps(record(0))+'\n'+json.dumps(record(2,sample=2))+'\n')
            f=Feed(p,replay=True);f.started-=100
            f.poll(.02);self.assertEqual(f.decoder.frame.sample,1)
            f.poll(2.);self.assertEqual(f.decoder.frame.sample,2)
    def test_latest_log_switch_resets_metadata(self):
        with tempfile.TemporaryDirectory() as t:
            p=Path(t)/'robot_1.jsonl';p.write_text(json.dumps(record())+'\n')
            f=Feed(logs_dir=t);f.poll();f.decoder.info={'old':True}
            p2=Path(t)/'robot_2.jsonl';p2.write_text(json.dumps(record(sample=2))+'\n')
            import os;os.utime(p2,(time.time()+1,time.time()+1));f.poll()
            self.assertEqual(f.decoder.frame.sample,2);self.assertIsNone(f.decoder.info)
    def test_usb_etw_sidecar_is_never_selected_as_robot_feedback(self):
        with tempfile.TemporaryDirectory() as t:
            p=Path(t)/'robot_1.jsonl';p.write_text(json.dumps(record(sample=7))+'\n')
            sidecar=Path(t)/'robot_1.usb_etw.jsonl';sidecar.write_text('{"kind":"usb_event"}\n')
            import os;os.utime(sidecar,(time.time()+1,time.time()+1))
            f=Feed(logs_dir=t);f.poll();self.assertEqual(f.path,p);self.assertEqual(f.decoder.frame.sample,7)
    def test_policy_call_uses_only_sim_state(self):
        tree=ast.parse((Path(__file__).resolve().parents[2]/'compare_real_sim.py').read_text(encoding='utf-8'))
        call=next(n for n in ast.walk(tree) if isinstance(n,ast.Call) and isinstance(n.func,ast.Name) and n.func.id=='independent_observation')
        self.assertNotIn('current',ast.unparse(call));self.assertNotIn('frame',ast.unparse(call))
    def test_physics_uses_same_four_substeps_and_independent_clock(self):
        from types import SimpleNamespace as NS
        from unittest.mock import Mock
        env=NS(action_manager=Mock(),scene=Mock(),sim=Mock(),cfg=NS(decimation=4),
               physics_dt=.005,_sim_step_counter=0,episode_length_buf=np.array([3]),common_step_counter=0)
        action=object();advance(env,action)
        env.action_manager.process_action.assert_called_once_with(action)
        self.assertEqual(env.action_manager.apply_action.call_count,4)
        self.assertEqual(env.scene.write_data_to_sim.call_count,4)
        self.assertEqual(env.scene.update.call_count,4)
        env.sim.step.assert_called_with(render=False)
        self.assertEqual(env._sim_step_counter,4);self.assertEqual(int(env.episode_length_buf[0]),4)

if __name__=='__main__':unittest.main()
