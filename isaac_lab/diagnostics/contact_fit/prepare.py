import json,sys
from pathlib import Path
from bisect import bisect_right
source=Path(sys.argv[1])
R=[json.loads(l) for l in source.open(encoding='utf-8')]
targets=[r for r in R if r['kind']=='policy_result' and r['sent']]
actions={(r['policy_session'],r['episode_step']):r for r in R if r['kind']=='policy_action'}
ticks={r['sample']:r for r in R if r['kind']=='tick' and r['mode']=='POLICY' and (r.get('imu') or {}).get('valid') and (r.get('imu_flow') or {}).get('gravity_direction_body') and r.get('mapped_body_gyro_rad_s') and all(m['valid'] for m in r['motors'])}
pid=[r for r in R if r['kind']=='f103_pid_feedback' and r.get('schema')==2 and all(m['valid'] for m in r['motors'])]
goal_pairs=[(t,actions[(t['policy_session'],t['episode_step'])]) for t in targets]
last=targets[-1]['t_s']
cases=[]
for name,lo,hi in [('fit_vx025',29.103,36.3829),('check_vx020',81.1629,last)]:
    ts=[p for p in goal_pairs if lo<=p[0]['t_s']<hi and p[1]['sample'] in ticks]
    first=ts[0][1]
    start=first['t_s']
    a=ticks[first['sample']]
    start_mcu=first['mcu_ms']
    # Align PWM with firmware time, using the latest tick's relation between host and MCU clocks.
    # Reference joint targets use the observed ACK time on the PC.
    raw_goals=[(round(t['t_s']-start,6),t['raw_positions']) for t,_ in ts]
    initial_target=targets[bisect_right([t['t_s'] for t in targets],start)-1]['raw_positions']
    raw_goals.insert(0,(0.,initial_target))
    pwm=sorted(((x['mcu_ms']-start_mcu)/1000,[m['pwm_raw'] for m in x['motors']])
               for x in pid if -.15<=(x['mcu_ms']-start_mcu)/1000<=(hi-start)+.1)
    assert pwm and pwm[0][0]<.1
    feedback=[]
    for _,r in ts:
        z=ticks[r['sample']]
        feedback.append(dict(t=r['t_s']-start,raw=[m['position'] for m in z['motors']],w=z['mapped_body_gyro_rad_s'],g=z['imu_flow']['gravity_direction_body']))
    cases.append(dict(name=name,start=start,end=hi,goals=raw_goals,pwm=pwm,feedback=feedback,raw=feedback[0]['raw'],
                      qd=first['obs55'][24:36],g=feedback[0]['g'],w=feedback[0]['w'],v=first['obs55'][:3]))
Path(sys.argv[2]).write_text(json.dumps(dict(log=source.name,windows=cases),separators=(',',':')),encoding='utf-8')
for c in cases:print(c['name'],c['start'],c['end'],len(c['goals']),len(c['pwm']),len(c['feedback']))
