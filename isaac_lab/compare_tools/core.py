"""Policy contract, observation assembly and bounded read-only CLI log input.

Independent observations have no real-frame parameter by design. The real robot
supplies command/run intent and an optional initial synchronization only.
"""
from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
import time
import numpy as np

JOINTS=tuple(f'{leg}_{kind}' for kind in ('shoulder','thigh','shin') for leg in ('lb','lf','rb','rf'))
REFERENCE=np.array([0.]*4+[math.radians(34)]*4+[math.radians(-85)]*4,dtype=np.float32)
PROFILE='55:12:0,34,-85:scale0.25:vx-0.3,0.5:trot1.7,0.6'
CONTRACTS={'yertle.position.v1:'+PROFILE,'v6.21.6:'+PROFILE}

def vector(value,n):
    a=np.asarray(value,dtype=np.float32)
    if a.shape!=(n,) or not np.isfinite(a).all(): raise ValueError(f'Expected {n} finite values')
    return a.copy()

def command(value):
    a=vector(value,3)
    if np.any(a<[-.30001,-.20001,-1.00001]) or np.any(a>[.50001,.20001,1.00001]):
        raise ValueError('Command outside the supported training profile')
    return a

def independent_observation(lin_vel,gyro,gravity,cmd,q,qd,last_action,episode_step,zero_velocity=True):
    """Same 55D layout as CLI; all state arguments must come from PhysX/SIM."""
    if episode_step<0: raise ValueError('Invalid episode step')
    out=np.empty(55,dtype=np.float32)
    out[:3]=0 if zero_velocity else vector(lin_vel,3)
    out[3:6]=vector(gyro,3);out[6:9]=vector(gravity,3);out[9:12]=command(cmd)
    out[12:24]=vector(q,12)-REFERENCE;out[24:36]=vector(qd,12)
    out[36:48]=vector(last_action,12)
    phase=np.remainder(np.float32(episode_step)*np.float32(.02)*np.float32(1.7),np.float32(1))
    angle=np.float32(2*math.pi)*phase
    out[48:50]=[np.sin(angle),np.cos(angle)];out[50:53]=[0,1,0];out[53:]=[1.7,.6]
    return out

def gravity_quaternion(gravity):
    """Roll/pitch from body gravity; yaw=0 is a display assumption, not measured."""
    g=vector(gravity,3).astype(float)
    if not .5<float(np.linalg.norm(g))<1.5: raise ValueError('Invalid gravity direction')
    roll=math.atan2(-g[1],-g[2]);pitch=math.atan2(g[0],math.hypot(g[1],g[2]))
    cr,sr=math.cos(roll/2),math.sin(roll/2);cp,sp=math.cos(pitch/2),math.sin(pitch/2)
    return np.array([cr*cp,sr*cp,cr*sp,-sr*sp],dtype=np.float32)

class Actor:
    def __init__(self,path):
        import onnxruntime as ort
        self.path=Path(path).resolve();self.sha256=hashlib.sha256(self.path.read_bytes()).hexdigest()
        opts=ort.SessionOptions();opts.intra_op_num_threads=1;opts.inter_op_num_threads=1
        self.session=ort.InferenceSession(str(self.path),sess_options=opts,providers=['CPUExecutionProvider'])
        self.metadata=self.session.get_modelmeta().custom_metadata_map
        if self.metadata.get('yertle.contract') not in CONTRACTS: raise ValueError('Incompatible ONNX profile/metadata')
        if 'yertle.joint_names' in self.metadata and self.metadata['yertle.joint_names']!=','.join(JOINTS):
            raise ValueError('ONNX joint order mismatch')
        inputs=self.session.get_inputs();outputs=self.session.get_outputs()
        if len(inputs)!=1 or len(outputs)!=1 or inputs[0].name!='obs' or outputs[0].name!='actions':
            raise ValueError('Expected obs/actions tensors')
        if inputs[0].shape!=[1,55] or outputs[0].shape!=[1,12] or inputs[0].type!='tensor(float)' or outputs[0].type!='tensor(float)':
            raise ValueError('Expected fixed float32 [1,55] -> [1,12]')
    def infer(self,obs):
        return vector(self.session.run(['actions'],{'obs':vector(obs,55)[None,:]})[0][0],12)
    def check_cli(self,info):
        if not info: raise ValueError('Waiting for CLI policy_model_config; use the current external-model CLI')
        if info.get('contract') not in CONTRACTS: raise ValueError('CLI profile differs from SIM')
        expected=self.metadata.get('yertle.checkpoint_sha256')
        if not expected or info.get('checkpoint_sha256')!=expected: raise ValueError('CLI and viewer checkpoint hashes differ')

@dataclass
class RealFrame:
    time_s:float
    mode:str
    cmd:np.ndarray
    obs:object
    sample:int
    zero_velocity:object
    episode_step:int=0
    session:int=0

    @property
    def q(self):
        if self.obs is None: return None
        return self.obs[12:24]+REFERENCE
    @property
    def quaternion(self):
        return None if self.obs is None else gravity_quaternion(self.obs[6:9])

class Decoder:
    def __init__(self):
        self.info=None;self.frame=None;self.session=0;self.episode_step=0;self.errors=0
    def accept(self,record):
        if record.get('kind')=='policy_model_config': self.info=record;return
        if record.get('kind')=='policy_action':
            self.session=int(record.get('policy_session',0));self.episode_step=int(record.get('episode_step',0))
            return
        if record.get('kind')!='tick': return
        try:
            timestamp=float(record['t_s'])
            if not math.isfinite(timestamp) or timestamp<0: raise ValueError('Invalid timestamp')
            obs=vector(record['obs55'],55) if record.get('obs55') is not None else None
            if obs is not None:
                if not np.allclose(obs[50:53],[0,1,0],atol=1e-5) or not np.allclose(obs[53:],[1.7,.6],atol=1e-5):
                    raise ValueError('Unsupported gait in CLI input')
                gravity_quaternion(obs[6:9])
            self.frame=RealFrame(timestamp,str(record.get('mode','UNKNOWN')),command(record['cmd']),obs,
                int(record.get('sample',0)),record.get('body_velocity_forced_zero_test'),self.episode_step,self.session)
        except (ValueError,TypeError,KeyError,OverflowError):
            self.errors+=1
            # Invalid data must be visibly invalid, never replace it with zero joints.
            self.frame=None

class JsonlTail:
    """Bounded binary reads; partial UTF-8/JSON lines are not parsed early."""
    def __init__(self,path): self.path=Path(path);self.offset=0;self.partial=b''
    def poll(self):
        try:
            if self.path.stat().st_size<self.offset: self.offset=0;self.partial=b''
            with self.path.open('rb') as f:
                f.seek(self.offset);chunk=f.read(262144);self.offset=f.tell()
        except FileNotFoundError: return []
        lines=(self.partial+chunk).split(b'\n');self.partial=lines.pop()
        if len(self.partial)>1048576: raise ValueError('Unterminated oversized JSONL record')
        result=[]
        for line in lines:
            if not line.strip(): continue
            record=json.loads(line.decode('utf-8-sig'))
            if isinstance(record,dict): result.append(record)
        return result

class Feed:
    def __init__(self,log=None,logs_dir=None,replay=False,speed=1.):
        self.explicit=Path(log) if log else None;self.logs_dir=Path(logs_dir) if logs_dir else None
        self.replay=replay;self.speed=speed;self.path=None;self.tail=None;self.decoder=Decoder()
        self.pending=[];self.started=time.monotonic();self.eof=False
        self.last_frame=None;self.last_tick_at=None
    def poll(self,replay_time=None):
        path=self.explicit
        if path is None and self.logs_dir:
            paths=list(self.logs_dir.glob('robot_*.jsonl'))
            path=max(paths,key=lambda p:p.stat().st_mtime_ns) if paths else None
        if path is not None and path!=self.path:
            self.path=path;self.tail=JsonlTail(path);self.decoder=Decoder();self.started=time.monotonic();self.pending=[]
            self.last_frame=None;self.last_tick_at=None
            if not self.replay and path.exists() and path.stat().st_size>524288:
                for record in self.tail.poll():self.decoder.accept(record)
                with path.open('rb') as f:
                    f.seek(max(0,path.stat().st_size-262144));f.readline();self.tail.offset=f.tell()
                self.tail.partial=b''
        if self.tail is None: return None
        if self.replay:
            if not self.pending: self.pending.extend(self.tail.poll())
            elapsed=(time.monotonic()-self.started)*self.speed if replay_time is None else replay_time
            while self.pending:
                record=self.pending[0]
                if float(record.get('t_s',0))>elapsed: break
                self.decoder.accept(self.pending.pop(0))
            self.eof=not self.pending and self.tail.offset==self.path.stat().st_size
        else:
            for record in self.tail.poll(): self.decoder.accept(record)
        if self.decoder.frame is not self.last_frame:
            self.last_frame=self.decoder.frame;self.last_tick_at=time.monotonic()
        return self.decoder.frame
    def age(self):
        if self.replay: return 0.
        if self.path is None: return float('inf')
        try:return max(0.,time.time()-self.path.stat().st_mtime,
                       time.monotonic()-self.last_tick_at if self.last_tick_at is not None else float('inf'))
        except FileNotFoundError:return float('inf')
