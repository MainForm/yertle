"""Offline Python-viewer vs deployed C++ ONNX Runtime numerical comparison."""
import argparse
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from compare_tools.core import Actor,REFERENCE,independent_observation

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--model',type=Path,required=True);p.add_argument('--cpp-infer',type=Path,required=True)
    p.add_argument('--report',type=Path,required=True);a=p.parse_args()
    actor=Actor(a.model);rng=np.random.default_rng(20261022);observations=[]
    for i in range(512):
        g=rng.normal(size=3);g/=np.linalg.norm(g)
        observations.append(independent_observation(rng.normal(size=3),rng.normal(size=3),g,
            [rng.uniform(-.3,.5),rng.uniform(-.2,.2),rng.uniform(-1,1)],REFERENCE+rng.normal(size=12)*.3,
            rng.normal(size=12)*3,rng.normal(size=12),i,True))
    observations=np.array(observations,dtype=np.float32)
    t=Path(tempfile.mkdtemp(prefix='ycv-infer-'));observations.tofile(t/'obs.f32')
    result=subprocess.run([str(a.cpp_infer.resolve()),'--batch',str(a.model.resolve()),str(t/'obs.f32'),str(t/'out.f32')],capture_output=True,check=True)
    cpp=np.fromfile(t/'out.f32',dtype=np.float32).reshape(-1,12)
    py=np.array([actor.infer(row) for row in observations]);np.testing.assert_allclose(py,cpp,atol=2e-5,rtol=2e-5)
    report=dict(passed=True,samples=len(observations),max_abs_error=float(np.max(abs(py-cpp))),
                model_sha256=actor.sha256,checkpoint_iteration=actor.metadata.get('yertle.checkpoint_iteration'),
                cpp_backend=json.loads(result.stdout),scope='numerical only; no COM/hardware')
    a.report.parent.mkdir(parents=True,exist_ok=True);a.report.write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps(report,indent=2))
if __name__=='__main__':main()
