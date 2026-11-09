"""Score logged replay separately on fitting and held-out straight-line segments."""
import json
from pathlib import Path
import os
import sys
import numpy as np

stage=Path(os.environ['YERTLE_CONTACT_REPLAY_DIR'])
variant=sys.argv[1] if len(sys.argv)>1 else 'coarse'
post='' if variant=='coarse' else '-'+variant
cases=json.loads((stage/f'config{post}.json').read_text())['cases']
real=json.loads((stage/'inputs.json').read_text())['windows']
sim=[json.loads(l) for l in (stage/f'trials{post}.jsonl').open()]
out={}
for w in real:
    z=[r for r in sim if r['window']==w['name']]
    assert len(z)>300,(w['name'],len(z))
    st=np.array([r['t'] for r in z])
    t=np.array([r['t'] for r in w['feedback']])
    assert all(np.diff(t)>0) and all(np.diff(st)>0)
    valid=(st>=.5)&(st<=t[-1])
    st=st[valid]
    raw=np.array([r['raw'] for r in w['feedback']])
    raw_ref=np.stack([np.interp(st,t,x) for x in raw.T],axis=-1)
    wxyz=np.array([r['w'] for r in w['feedback']])
    gravity=np.array([r['g'] for r in w['feedback']])
    yaw_ref=np.interp(st,t,-np.sum(wxyz*gravity,axis=-1))
    tilt_ref=np.interp(st,t,np.rad2deg(np.arccos(np.clip(-gravity[:,2],-1,1))))
    def rollup(rate):
        return np.r_[0,np.cumsum((rate[1:]+rate[:-1])*.5*np.diff(st))]
    cum_ref=rollup(yaw_ref)
    actual=np.array([r['raw'] for r in z])[valid]
    rates=np.array([r['yaw_world'] for r in z])[valid]
    angles=np.array([r['tilt'] for r in z])[valid]
    height=np.array([r['z'] for r in z])[valid]
    guard=np.array([r['pid_guard'] for r in z])[valid]
    joint_err=np.abs(actual-raw_ref[:,None,:])
    rows=[]
    for i,(mode,name,friction) in enumerate(cases):
        cumulative=rollup(rates[:,i])
        rows.append(dict(mode=mode,candidate=name,friction=friction,final_yaw_deg=float(np.rad2deg(cumulative[-1])),
            yaw_rmse_rad_s=float(np.sqrt(np.mean((rates[:,i]-yaw_ref)**2))),
            cumulative_yaw_mae_deg=float(np.rad2deg(np.mean(abs(cumulative-cum_ref)))),
            final_yaw_error_deg=float(np.rad2deg(abs(cumulative[-1]-cum_ref[-1]))),
            raw_position_mae=float(joint_err[:,i].mean()),
            raw_mae_per_leg=joint_err[:,i].reshape(-1,4,3).mean(axis=(0,2)).tolist(),
            tilt_mae_deg=float(np.mean(abs(angles[:,i]-tilt_ref))),
            max_tilt_deg=float(angles[:,i].max()),min_height_m=float(height[:,i].min()),
            pid_guard=bool(guard[:,i].any())))
    out[w['name']]=dict(real_yaw_deg=float(np.rad2deg(cum_ref[-1])),real_yaw_abs_mean=float(abs(yaw_ref).mean()),
                        real_max_tilt=float(tilt_ref.max()),duration_s=float(st[-1]-st[0]),samples=len(st),
                        results=sorted(rows,key=lambda a:a['cumulative_yaw_mae_deg']))
(stage/f'summary{post}.json').write_text(json.dumps(out,indent=2))
for k,d in out.items():
    print(k,'real yaw',round(d['real_yaw_deg'],2),'real tilt max',round(d['real_max_tilt'],2))
    for a in d['results']:
        print(a['mode'],a['candidate'],*[round(a[key],3) for key in ('final_yaw_deg','cumulative_yaw_mae_deg','yaw_rmse_rad_s','raw_position_mae','max_tilt_deg')],a['pid_guard'])
