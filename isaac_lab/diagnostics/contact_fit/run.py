"""Offline replay of real acknowledged motor goals with candidate contact materials."""
from pathlib import Path
import sys

source=Path(__file__).resolve().parents[2] / 'play_xyz_sequential_sweep.py'
sys.path[:0]=[str(Path(__file__).parent),str(source.parent)]
sys.argv[0]=str(source)
code=source.read_text()
anchor='    env_cfg.episode_length_s = 30.0'
code=code.replace(anchor,anchor+'''
    env_cfg.seed=0
    env_cfg.events.physics_material=None
    env_cfg.events.add_base_mass=None
    for key in ('pose_range','velocity_range'):
        env_cfg.events.reset_base.params[key]={}
''')
start=code.index('    sweep_trace = []')
end=code.index('    clear_manual_push(pulse)',start)
code=code[:start]+'''    from contact_fit import run_trial
    run_trial(base_env)
'''+code[end:]
exec(compile(code,str(source),'exec'),dict(__file__=str(source),__name__='__main__',__package__=None))
