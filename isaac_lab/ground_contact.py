"""Force-independent ground contact classification for rewards and termination.

Do not mutate collider geometry, joint frames or policy observations. Inspect
individual PhysX contacts, never averaged contact positions or net forces.
"""
import torch
from isaaclab.managers import ManagerTermBase
from .foot_geometry import FOOT_NAMES, load_sole_points

BODY_NAMES = ('base_link',) + tuple(f'{leg}_{joint}' for leg in ('lf','rf','lb','rb')
                                   for joint in ('shoulder','thigh','shin'))


def surface_distance(points, triangles):
    """Exact nearest triangle-surface distance [m], including edges/vertices."""
    a,b,c=triangles.unbind(1)
    ab=b-a;ac=c-a;ap=points[:,None]-a
    n=torch.linalg.cross(ab,ac,dim=-1)
    nn=n.square().sum(-1)
    pn=(ap*n).sum(-1)
    projected=ap-pn[...,None]*n/nn.clamp_min(1e-20)[...,None]
    aa=ab.square().sum(-1);bb=ac.square().sum(-1);cross=(ab*ac).sum(-1)
    det=aa*bb-cross.square()
    u=((projected*ab).sum(-1)*bb-(projected*ac).sum(-1)*cross)/det.clamp_min(1e-20)
    v=((projected*ac).sum(-1)*aa-(projected*ab).sum(-1)*cross)/det.clamp_min(1e-20)
    inside=(nn>1e-20)&(u>=0)&(v>=0)&(u+v<=1)
    edges=[]
    for start,end in ((a,b),(b,c),(c,a)):
        direction=end-start;relative=points[:,None]-start
        t=((relative*direction).sum(-1)/direction.square().sum(-1).clamp_min(1e-20)).clamp(0,1)
        edges.append((relative-t[...,None]*direction).square().sum(-1))
    squared=torch.where(inside,pn.square()/nn.clamp_min(1e-20),torch.stack(edges).amin(0))
    return squared.amin(-1).clamp_min(0).sqrt()


def inverse_rotate(q, vector):
    xyz=-q[...,1:];cross=torch.linalg.cross(xyz,vector,dim=-1)
    return vector+2*(q[...,:1]*cross+torch.linalg.cross(xyz,cross,dim=-1))


def forbidden_flags(counts, starts, points, separation, position, quaternion,
                    triangles=None, separation_tolerance_m=.001, sole_tolerance_m=.005):
    """One flag per body/environment if ANY qualifying contact is forbidden.

    Counts and starts are unaggregated sensor/ground-pair buffers. No force
    threshold. Multiple simultaneous contacts never hide each other.
    """
    counts=counts.reshape(-1).long();starts=starts.reshape(-1).long()
    owner=torch.repeat_interleave(torch.arange(len(counts),device=counts.device),counts)
    begin=torch.repeat_interleave(counts.cumsum(0)-counts,counts)
    indices=torch.repeat_interleave(starts,counts)+torch.arange(len(owner),device=counts.device)-begin
    p=points.reshape(-1,3)[indices]
    gap=separation.reshape(-1)[indices]
    valid=torch.isfinite(p).all(-1)&torch.isfinite(gap)&(gap<=separation_tolerance_m)
    if triangles is not None:
        local=inverse_rotate(quaternion[owner],p-position[owner])
        distance=surface_distance(local,triangles)
        valid &= distance>sole_tolerance_m
    result=torch.zeros(len(counts),dtype=torch.long,device=counts.device)
    result.scatter_add_(0,owner,valid.long())
    return result>0


class GroundContacts:
    """Shared per-control-step contact snapshot for the three penalty terms."""
    def __init__(self,env):
        self.robot=env.scene['robot']
        self.ids=[self.robot.body_names.index(n) for n in BODY_NAMES]
        self.views=[]
        sensor=env.scene.sensors['contact_forces']
        root=self.robot.cfg.prim_path.replace('.*','*')
        ground=env.scene.terrain.cfg.prim_path
        from pxr import Usd, UsdPhysics
        import omni.usd
        ground_prim=omni.usd.get_context().get_stage().GetPrimAtPath(ground)
        colliders=[str(p.GetPath()) for p in Usd.PrimRange(ground_prim) if p.HasAPI(UsdPhysics.CollisionAPI)]
        if len(colliders)!=1:
            raise RuntimeError(f'Expected one flat ground collider, found {colliders}')
        ground=colliders[0]
        # One exact body per environment per view: avoid multi-body filtered
        # ContactSensor aggregation limitations and expensive point averaging.
        for name in BODY_NAMES:
            view=sensor._physics_sim_view.create_rigid_contact_view(
                root+'/'+name,filter_patterns=[ground],max_contact_data_count=64*env.num_envs)
            if view.sensor_count!=env.num_envs or view.filter_count!=1:
                raise RuntimeError(f'Unexpected contact layout for {name}: {view.sensor_count}, {view.filter_count}')
            self.views.append(view)
        self.triangles={name:torch.tensor(p,device=env.device,dtype=self.robot.data.body_pos_w.dtype).reshape(-1,3,3)
                        for name,p in zip(FOOT_NAMES,load_sole_points(self.robot.cfg.spawn.usd_path,triangles=True))}
        self.last_step=None
        self.flags=None

    def read(self,env,separation_tolerance_m,sole_tolerance_m):
        key=(env.common_step_counter,separation_tolerance_m,sole_tolerance_m)
        if self.last_step==key:return self.flags
        flags=[]
        for name,index,view in zip(BODY_NAMES,self.ids,self.views):
            _,points,_,gap,counts,starts=view.get_contact_data(dt=env.physics_dt)
            if int(counts.sum().item())>=points.reshape(-1,3).shape[0]:
                raise RuntimeError(f'Contact buffer saturated for {name}; increase max_contact_data_count')
            flags.append(forbidden_flags(counts,starts,points,gap,
                self.robot.data.body_pos_w[:,index],self.robot.data.body_quat_w[:,index],
                self.triangles.get(name),separation_tolerance_m,sole_tolerance_m))
        self.flags=torch.stack(flags,dim=1);self.last_step=key
        return self.flags


class GroundContactPenalty(ManagerTermBase):
    def __init__(self,cfg,env):
        super().__init__(cfg,env)
        if not hasattr(env,'_yertle_ground_contacts'):
            env._yertle_ground_contacts=GroundContacts(env)
        self.contacts=env._yertle_ground_contacts
        self.indices=[BODY_NAMES.index(n) for n in cfg.params['body_names']]

    def reset(self,env_ids=None):
        self.contacts.last_step=None

    def __call__(self,env,body_names,separation_tolerance_m=.001,sole_tolerance_m=.005):
        if not 0<=separation_tolerance_m<float('inf') or not 0<=sole_tolerance_m<float('inf'):
            raise ValueError('Contact distance tolerances must be finite and nonnegative')
        return self.contacts.read(env,separation_tolerance_m,sole_tolerance_m)[:,self.indices].float().sum(-1)
