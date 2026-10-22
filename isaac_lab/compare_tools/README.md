# 실물 재현 + 독립 ONNX/PhysX 비교

## 분리 원칙

- 기존 C++ CLI는 COM 수신, 센서/입력 매핑, 실제 모터 제어를 그대로 담당한다.
- `compare_real_sim.py`는 CLI가 이미 기록하는 JSONL을 읽기만 한다. 포트를 열거나
  CLI/모터에 명령을 전송하지 않는다. 종료/지연되어도 CLI 동작에는 영향을 주지 않는다.
- A / `Robot`: 자유 몸체 + 중력/접촉/implicit drive. 자신의 상태로55D 입력을 만들고
  같은 ONNX를 추론해12D action을 적용한다. 이전 action과 phase도 자체 관리한다.
  play와 같은 action 처리 및5ms×4 물리 서브스텝을 사용하며, 표시와 무관한 PPO
  reward/종료 판정/자동 reset 계산은 실행하지 않는다.
- B / `Measured`: CLI의 obs55에서 기준각을 복원해 측정 관절 자세와 중력 방향의
  roll/pitch를 직접 표시한다. 중력/충돌/drive가 꺼진 별도 모델이며 물리 예측이 아니다.
- 두 모델이 계속 공유하는 것은 vx/vy/yaw 명령과 시작/정지 의도뿐이다.
  POLICY 진입/새 세션 또는 명시적 Sync 버튼에서만 측정 초기 자세로 A를 한 번 reset한다.
  이후 실물 q/qd/IMU/이전 action을 A 입력에 복사하지 않는다.
- 실물 CLI 입력이 없어도 센서값을 0으로 만들어 표시하지 않는다. 마지막 자세를 유지하고
  STALE/WAITING을 표시한다. 기본 0.5초 무갱신이면 A도 일시 정지한다.
- 새 프로그램/테스트/launcher 소스는 이 시뮬레이션 저장소에만 둔다.
  기존 Git 무시 대상인 v6.21.6 CLI와 외부 ONNX exporter를 수정하거나 커밋하지 않는다.

## 현재 실행 (Windows)

ONNX Runtime CPU 1.23.2를 기존 Isaac Python에 설치한다 (현재 PC는 설치 완료).

```powershell
D:\tools\isg\e\Scripts\python.exe -m pip install -r "C:\Users\ik\Documents\ChatGPT\yertle 기반 4족보행 로봇 제작\yertle-isaaclab-host\source\isaac_lab\compare_tools\requirements.txt"
```

1. 기존 실물 CLI는 기존 명령으로 실행한다. `--zero-velocity-test` 조건을 유지한다.
2. 별도 PowerShell에서 아래 뷰어를 실행한다. 기본적으로 기존 번들의 가장 최근
   `logs/robot_*.jsonl`을 따라가므로 CLI에 새 옵션을 추가할 필요가 없다.
3. 기존 CLI에서 F3/H/U/P를 조작한다. 뷰어는 P 진입 시 초기 자세를 한 번 맞추고
   독립 시뮬레이션을 시작한다. 실물의 POLICY 종료는 A를 일시 정지시킨다.
4. 연결/모니터링만 한 상태에서도 유효한 obs55가 있으면 뷰어의
   `Sync initial pose / start SIM` 버튼으로 **시뮬레이션만** 시작할 수 있다.
   이 버튼은 실제 로봇을 움직이거나 arm하지 않는다.

```powershell
D:\tools\isg\e\Scripts\python.exe "C:\Users\ik\Documents\ChatGPT\yertle 기반 4족보행 로봇 제작\yertle-isaaclab-host\source\isaac_lab\compare_real_sim.py" --experience isaaclab.python.yertle_min_d3d12.kit
```

기본 ONNX는 `C:\Users\ik\Desktop\yertle-RTOS\policy_onnx\models\active\policy.onnx`.
뷰어 시작 시 해당 ONNX를 고정해서 읽고 CLI 로그의 checkpoint 해시와 비교한다.
다른 모델이면 독립 실행을 차단한다. CLI와 뷰어를 종료한 후 ONNX를 교체하고 다시 시작한다.
`--model`로 같은 모델의 다른 파일 경로를 지정할 수도 있다.

특정 실시간 로그: `--log "로그 절대경로.jsonl"`.
다른 CLI 로그 폴더: `--logs-dir "폴더"`.
이전 로그 재생: `--replay "로그.jsonl" --model "해당 로그와 같은 ONNX"`.
기록된 센서/명령을 재생하되 A는 여전히 자신의 상태로 ONNX를 실행한다.
POLICY 시작 후 기록 재생 시간은 A의 시뮬레이션 시간에 맞추므로 느린 렌더링 때문에
실물 프레임만 먼저 진행하지 않는다. 실시간 비교가 느리면 로그 재생으로 비교한다.
`--replay-speed 0.5`는 기록의 재생 속도를 바꾸므로 시간 축 비교에 유의한다.
`--seconds 10`은 뷰어 실행시간을 제한한다. `--capture`는 종료 시 viewport PNG를 저장한다.

실물 없이 화면만 확인:

```powershell
D:\tools\isg\e\Scripts\python.exe "C:\Users\ik\Documents\ChatGPT\yertle 기반 4족보행 로봇 제작\yertle-isaaclab-host\source\isaac_lab\compare_real_sim.py" --demo --experience isaaclab.python.yertle_min_d3d12.kit
```

데모 B는 오른쪽 앞다리를 합성 파형으로 움직인다. 실측 데이터인 것처럼 해석하면 안 된다.

## 비교 조건과 한계

- 기본 A 입력 선속도 XYZ는0: 기존 실물 `--zero-velocity-test`와 조건을 맞춘다.
  A의 물리 몸통 속도를 고정하거나 실제 IMU 각속도를 지우는 것이 아니다.
  `--linear-velocity measured`는 A의 실제 PhysX 선속도를 입력하고,
  실물 CLI의 forced-zero 설정과 다르면 비교를 차단한다.
- 55D/12D, 관절 순서, 기준0/+34/-85°, scale.25, 50Hz, trot1.7Hz/duty.6을 확인한다.
  좌표축/모터 ID/부호/raw 보정 및 기존 입력 API는 바꾸지 않는다.
- 기본 물리 상한은 관절별1.9Nm/4.6rad/s이다. 다른 학습 설정은
  `--effort-limit-sim`/`--velocity-limit-sim`으로 명시해야 한다.
  실제 모터 연결 토크/전류/펌웨어 PID/Speed/ACC가 보정된 모델은 아니다.
- B의 절대 XY·높이·yaw는55D 입력만으로 측정할 수 없다. A 옆에 표시하도록
  XY를 정렬하고, 높이0.25m/yaw0을 사용한다. B의 화면상 위치/지면 이탈/몸통 높이를
  실측 이동으로 해석하지 않는다. 동적 가속도는 중력 기반 roll/pitch에도 영향을 줄 수 있다.
- 자동 fall reset은 없다. A가 넘어지는 것도 관찰 대상이다. Sync만 초기 상태를 쓴다.
- A는 명목20ms, PhysX5ms/4서브스텝이다. 렌더링은 최대30Hz이고 하드 실시간이 아니다.
  CLI와의 시간/phase가 계속 강제 동기화되는 것은 아니며, 느린 PC에서는 차이가 누적될 수 있다.
- 기본 물리는1env CPU, 렌더링은D3D12/performance다. 학습이 GPU를 사용하는 동안
  불필요한 GPU 물리 메모리와 중복 렌더링을 줄이기 위한 선택이다.
  GPU 학습 backend와 직접 맞추려면 `--device cuda:0`을 명시할 수 있다.
- `runs/real_sim_compare/<시각>/`에 ONNX 해시/설정 report, A의 입력/action/실제 관절,
  B의 측정 관절 및 각 시간 축을 timeline.jsonl로 남긴다. 원본 CLI 로그는 덮어쓰지 않는다.

## 테스트 / 빌드

```powershell
D:\tools\isg\e\Scripts\python.exe -m unittest discover -s "C:\Users\ik\Documents\ChatGPT\yertle 기반 4족보행 로봇 제작\yertle-isaaclab-host\source\isaac_lab\compare_tools\tests" -v
```

`verify_runtime.py --model MODEL --cpp-infer policy_infer.exe --report report.json`은
뷰어 Python과 기존 C++ ONNX Runtime 출력을512개 입력으로 비교하며 COM을 열지 않는다.

선택적 Windows launcher는 simulator/Python을 새로 설치하거나 묶지 않는다.
기존 Isaac Python과 이 checkout의 viewer 경로를 사용해 실행한다.

```powershell
cmake -S "C:\Users\ik\Documents\ChatGPT\yertle 기반 4족보행 로봇 제작\yertle-isaaclab-host\source\isaac_lab\compare_tools\launcher" -B "C:\Users\ik\Documents\ChatGPT\yertle 기반 4족보행 로봇 제작\yertle-isaaclab-host\source\isaac_lab\runs\compare_build" -A x64
cmake --build "C:\Users\ik\Documents\ChatGPT\yertle 기반 4족보행 로봇 제작\yertle-isaaclab-host\source\isaac_lab\runs\compare_build" --config Release
```

`Release/yertle_compare.exe`는 D3D12 kit를 기본으로 선택한다. `--python PATH`로
기존 Python 경로만 바꿀 수 있다. checkout 이동 후에는 다시 빌드해야 한다.
`--self-test`는 Windows Unicode/공백/따옴표 인자 전달만 검사하며 simulator를 띄우지 않는다.
