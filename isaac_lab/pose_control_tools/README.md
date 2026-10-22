# 자세 제어 도구 정리

사용자 실행 진입점은 **`../pose_control.py` 하나**다. 이 폴더에는 데이터 준비,
회귀 테스트, 예전 추가 스크립트 보관본만 둔다. 실물 연결이나 policy 실행은 하지 않는다.

```text
isaac_lab/
  pose_control.py                   자세 편집 / 물리 재생 공통 실행
  pose_control_tools/               나머지 지원 도구
    data/pose_targets.json          실제 CLI 목표를 기존 C++ API로 역변환한 데이터
    prepare_targets.py             위 데이터 재생성 (개발용; 보통 실행 불필요)
    tests/test_pose_control.py      Isaac 없이 실행하는 데이터/JSON/프레임 테스트
    archive/                       기존 추가 스크립트 5개 원본 그대로 보관
```

## 실행 (Windows PowerShell)

실제 설치된 Python과 D3D12 kit를 사용한다. `Set-Location`이나 새 설치는 필요 없다.

### 자세 편집 / 형상 확인

```powershell
D:\tools\isg\e\Scripts\python.exe "C:\Users\ik\Documents\ChatGPT\yertle 기반 4족보행 로봇 제작\yertle-isaaclab-host\source\isaac_lab\pose_control.py" --mode static --pose tripod-high --experience isaaclab.python.yertle_min_d3d12.kit --device cuda:0
```

`Standing`, `Seated`, `Body shift`, `RF25mm`, `RF half body`를 즉시 선택한다.
12개 관절의 슬라이더/입력창은 ±180도다. **이 UI 범위는 실제 모터의 안전 범위가 아니다.**
정적 모드는 몸통을 고정하고 화면에 자세를 직접 반영한다. 균형 검증으로 해석하면 안 된다.

`Load JSON`은 경로 입력창에 적힌 기존 schema-1 자세 파일을 읽는다.
`Save target JSON`은 선택한 목표 자세(실물 캘리브레이션 아님)를 저장한다.
기본 저장 폴더는 `isaac_lab/runs/pose_control/<실행시각>/`이며 창의 입력칸과 터미널에 출력된다.
저장 JSON은 기존 `play.py --pose-json`의 이름 기반 로더와 호환된다.
단, 기존 checkpoint의 action 기준/물리 설정까지 호환된다는 뜻은 아니다.

저장 자세로 바로 시작하려면 `--pose tripod-high` 대신 `--pose-json "저장한 JSON 절대경로"`를 쓴다.

### 중력·접촉을 켜고 전체 궤적 재생

```powershell
D:\tools\isg\e\Scripts\python.exe "C:\Users\ik\Documents\ChatGPT\yertle 기반 4족보행 로봇 제작\yertle-isaaclab-host\source\isaac_lab\pose_control.py" --mode physics --trajectory tripod-high --experience isaaclab.python.yertle_min_d3d12.kit --device cuda:0
```

3초 선 자세 안정화 → 뒤/왼쪽 몸체 이동 → RF 들기 → 유지 → 내리기 → 복귀.
`tripod-low`는 25mm/651프레임, `tripod-high`는 약96mm/951프레임 원본 CLI 목표다.
목표 주기20ms, PhysX dt5ms/4서브스텝, 종료 후2초 평가 구간이 있다.
GUI는 끝나도 유지된다. 다시 실행하려면 창에서 `Reset + Run ...`을 누른다.
`--exit-on-complete`를 추가하면 궤적/평가 종료 후 창도 닫힌다.

물리 모드에서는 처음 시작 및 명시적인 Reset/Run 이외에는 몸통 위치를 쓰지 않는다.
중력/지면 접촉/토크1.9Nm/속도4.6rad/s 및 기존 stiffness25/damping0.5가 작동한다.
넘어져도 자동 reset/목표 중단은 하지 않는다. 실물 CLI의8도 안전 중단을 흉내 내지 않고
한계 통과 시점을 report에 기록한다. 키네마틱 삼각형이나 정적 발 높이는 균형 증명이 아니다.
모터 펌웨어 PID·전류·Speed/ACC를 보정한 모델이 아니므로 실물 안전을 보장하지 않는다.

공통: 버튼 `R: Reset`, `Pause/resume`, `X: Exit`. 터미널 포커스일 때는 R/Space/X도 가능하다.
물리 모드의 수동 자세 변경은 매20ms 최대0.02rad로 목표를 보간한다.
정확한 CLI 궤적 재생에는 추가 보간을 넣지 않는다.
`report.json`, 물리 모드의 `physics_timeline.json`에 조건·기울기·접촉·목표/실제 각도를 남긴다.

## play.py 커밋에서 재사용한 부분

- `566af37`: 원본 관절 축/프레임 유지. 이름 기준 관절 배치, API motor ID/sign 재작성 없음.
- `d96a468`: signed 기준 자세 `0/+34/-85°`. `configure_signed_joint_reference()` 그대로 호출.
- `d23977c`: 임시 USD의12개 PhysX 위치 stop 해제 및 runtime limit buffer 반영.
  `release_joint_position_limits()`까지 실제 호출한다. 원본 USD를 덮어쓰지 않는다.
- `play_start_pose.load_pose_json()` / `reset_selected_pose()` 재사용:
  JSON 검증·이름 기반 reset. 학습 action 기준/scale를 새 자세에 맞춰 변경하지 않는다.
- 장치/experience 선택은 기존 `AppLauncher`를 사용한다.

이 도구는 PPO policy/action을 실행하지 않는 직접 목표 재생이다. 따라서 `play.py`의
속도 명령, 외란 키, gait/reward 옵션을 복제하지 않았다. RL episode가 없으므로 모든
자동 episode reset도 없다. 접촉 센서는 유지하고 몸통 접촉은 기록한다.
기존 `play.py`, 학습 설정, calibrated API, checkpoint는 수정하지 않았다.

## 데이터와 보관 정책

`data/pose_targets.json`은 실물 하드웨어가 아닌 **배포 CLI mock 실행 로그**의 전체 raw 목표를
동일 배포본 C++ `PolicyRawConverter`로 역변환했다. 모든 raw 왕복 일치 확인 후 생성했으며
원본 로그 경로/SHA-256, 변환기 SHA-256를 데이터에 저장했다. 사용자 실행에는 C++ 빌드나
Desktop API 프로젝트가 필요하지 않다. 앉은 자세는0/+75/-160도 프리셋이다.

`archive/`의5개 파일은 이동 전후 SHA-256 일치를 확인한 원본 보관본이다.
새 진입점에서는 import하지 않으며 이전 상대경로를 쓰므로 그 자리에서 실행하는 용도가 아니다.
특히 `test_tripod_dynamics.py`는 PhysX 위치 stop 해제 경로를 빠뜨렸던 이전 검증본이다.
과거 결과는 `incompatible_joint_limits`였으며 실제 궤적 성공 결과가 아니었다.

기존 커밋된 `show_default_pose.py`, 실측 로그/IMU/sweep 재생 도구는 다른 용도의 기존
호환 진입점이므로 삭제/이동하지 않았다. 새 자세 확인 작업은 `pose_control.py`를 사용한다.

CLI C++/mock 테스트는 CMake 상대경로가 있으므로 원래 프로젝트에 유지한다:

- `C:\Users\ik\Desktop\yertle-RTOS\policy_onnx\deployments\v6.20.3_model_2200\source\policy\tests\test_tripod_lift.cpp`
- 같은 `tests/test_high_tripod.cpp`, `tests/export_tripod_pose.cpp`
- 같은 `tools/verify_tripod_lift.ps1`, `tools/verify_high_tripod.ps1`

이전 workspace 측정 폴더의 `deploy.ps1`, `analyze_ranges.mjs`, `raw_inverse.cpp`,
CMake 파일과 빌드 출력은 증거/빌드 입력이므로 그대로 두었다. 런타임 의존성은 아니다.

## 개발 검증

```powershell
python -m unittest discover -s "C:\Users\ik\Documents\ChatGPT\yertle 기반 4족보행 로봇 제작\yertle-isaaclab-host\source\isaac_lab\pose_control_tools\tests" -v
```

실제 GUI 검증: 첫 실행 명령에 `--verify-view` 추가. 선 자세/발 들기 캡처 차이,
실제 PhysX 관절 위치, 발 높이 비율을 검사하고 자동 종료한다.
물리 자동 검증: 두 번째 명령에 `--headless --exit-on-complete` 추가.
저장 폴더를 명시하려면 `--output-dir "새 폴더의 절대경로"`; 기존 폴더 덮어쓰기는 거부한다.

### 2026-10-20 실제 검증 결과

- 순수 회귀 테스트7개 통과, Python 문법 검사 통과.
- D3D12 GUI 정적 검증 통과: 실제 관절 반영, 화면 캡처 차이, 슬라이더1도 변경,
  저장/재로드 왕복 확인. 발 높이/몸통 높이 비율0.500051.
- D3D12 GUI 자유 몸체 물리 검증: 원본951프레임 끝까지 재생, 평가 포함1202샘플.
  최대 기울기1.8653도, 몸통 접촉0N, 상대8도 중단 한계 통과 없음, 넘어짐 관측 없음.
  몸통 상태 쓰기는 초기 reset1회뿐이다. 보정되지 않은 모터 모델의 시뮬레이션 결과이며 실물 안전 보증은 아니다.
- 상세 증거는 workspace `measurements/pose_control_20261020/static_final/` 및
  `physics_gui_final/`의 report/PNG/JSON. 기존 실행본의 `physics_high/`는
  headless 결과(최대 기울기1.9212도)다.
- 원래 `play.py`·공용 helper·학습 config 및 원본 API/모터 매핑 변경 없음.
