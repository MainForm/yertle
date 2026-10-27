# PWM 경로 전체 소스 감사 — 2026-10-27

> Update after the audit: user explicitly removed F10330ms service and50ms
> feedback-age/PID-interval cutoffs.500ms target-command TTL is retained.
> The table below is the earlier audit snapshot, not a claim those timing
> guards remain active. Current behavior is in F103_POSITION_PID.md.

**Speed/ACC 중간 목표 생성은 사용자가 지정하지 않았는데 제가0510c6a에서 추가한 로직이었다. 이번에 제거했다.**
이 문서는 제공된 대화의 사용자 지시와 현재 코드/Git 이력을 대조한 결과다. “직접 요청 못 찾음”은
제공된 지시에 구체적 동작/수치가 없다는 뜻이며, 모든 입력 검증/구현 선택이 불필요하다는 뜻은 아니다.
나머지 후보를 사용자 동의 없이 일괄 삭제하지 않았다.

## 범위와 방법

- F103 driver/task/protocol/PID/storage, 공용API, 실제 배포본API·ONNX/CLI,
  루트의 구버전 소스 사본, Isaac Lab 학습/play/진단 소스·테스트를 목록화했다.
- 파일 551개와 해시를 기록하고 제어/제한 관련181곳을 검색해 활성 호출 경로·도입 이력을 대조했다.
- 그룹별 파일 수: `{"firmware": 72, "shared_api": 67, "active_bundle_api": 69, "root_policy": 19, "active_bundle_policy": 61, "simulation": 263}`.
- vendor/third_party·생성 build/dist·모델 바이너리·기존 로그는 코드 감사 대상에서 제외했다.
  모든 파일의 모든 설계 의도를 독립 검증했다는 보장은 아니다. `inventory.json`에 정확한 범위/후보가 있다.
- 활성 PC 소스는 `C:\Users\ik\Desktop\yertle-RTOS\policy_onnx\deployments\v6.21.6_model_2850\source`. 루트 `policy_onnx/src` 및 공용API는 별도 사본이며 서로 다르다.
  따라서 루트 코드만 검색해 “실행파일에 없다/있다”고 결론내리지 않았다.

## 이번 변경

1. `position_pid_step(goal, measured, dt)`로 단순화; Speed/ACC 인자·reference_velocity·제동거리·목표 램프 제거.
2. 일반 PWM 위치 명령과2048 복귀 모두 직접 목표를 사용. signed error, 매핑/캘리브레이션, P5/I3/D0.10과 I100/PWM800은 유지.
3. PWM만의 Speed0 거부 제거. 공통0x06 형식과 위치모드의 Speed/ACC 동작은 유지.
4. 실제 service에 Speed0/1/200/3000/32767 × ACC0/10/100/255를 넣어20조합 출력 동일성 검증.
5. 시뮬레이션 C 라이브러리를 별도 폴더에 재빌드. 오래된 profiled 라이브러리는 현재 runner에서 거부.
6. 기존 결과/로그/체크포인트는 삭제하거나 새 결과로 바꿔치기하지 않음.

## 추가된 동작 / 수치 선택의 확인 결과

| 항목 | 실제 동작 | 사용자 지시 대조·이번 처리 | 코드 |
|---|---|---|---|
| PWM Speed/ACC 중간 목표 | 목표를 속도/가속도/제동거리로 늦춤 | 직접 요청 없음.0510c6a에서 추가. **이번에 제거** | [position_pid.c:34](<C:/Users/ik/Desktop/yertle-RTOS/freeRTOS/firmware/src/position_pid.c:34>) |
| 추종 오차512raw | 목표−현재 절대차>512면 PID 거부→종료 경로. 이제 직접 목표 기준 | 구체적 수치/추종오차 중단 직접 요청 못 찾음. **유지·재검토 필요** | [position_pid.c:40](<C:/Users/ik/Desktop/yertle-RTOS/freeRTOS/firmware/src/position_pid.c:40>) |
| 센서 신선도50ms | F103은12축 유효·최신 피드백 요구, 초과 즉시 stop | 이 값의 F103 강제중단 직접 요청 못 찾음. IMU 시각 허용50ms 요청과 별개 | [pwm_contract.h:49](<C:/Users/ik/Desktop/yertle-RTOS/freeRTOS/firmware/include/pwm_contract.h:49>) |
| 서비스 간격30ms / PID dt50ms | task 서비스 지연>30ms 또는 dt>50ms면 stop | 직접 수치 요청 못 찾음. PWM 초기 구현에서 추가 | [pwm_control.c:565](<C:/Users/ik/Desktop/yertle-RTOS/freeRTOS/firmware/src/pwm_control.c:565>) / [pwm_control.c:502](<C:/Users/ik/Desktop/yertle-RTOS/freeRTOS/firmware/src/pwm_control.c:502>) |
| 목표 TTL500ms | 새 목표 없이500ms 지나면 stop | 기존 watchdog 사용 의도는 있으나 PWM용500ms 값 직접 지정은 못 찾음 | [pwm_contract.h:48](<C:/Users/ik/Desktop/yertle-RTOS/freeRTOS/firmware/include/pwm_contract.h:48>) |
| 전압5.0–7.4V | 한 샘플이라도 범위 밖이면 stop | 전압 로그 요청은 있었지만 이 전압 범위/즉시중단 요청은 못 찾음 | [pwm_control.c:557](<C:/Users/ik/Desktop/yertle-RTOS/freeRTOS/firmware/src/pwm_control.c:557>) |
| 2048 복귀 실패 판정 | 총15s /500ms간12raw 진전 없으면 OFF /±16raw 3회면 도달 | 복귀 우선·실패 시OFF는 요청됨. 이 수치들은 제가 선택한 값 | [pwm_control.c:498](<C:/Users/ik/Desktop/yertle-RTOS/freeRTOS/firmware/src/pwm_control.c:498>) / [pwm_control.c:526](<C:/Users/ik/Desktop/yertle-RTOS/freeRTOS/firmware/src/pwm_control.c:526>) |
| PC H/U 수동 자세 보간 | 주기당±8raw씩 목표 변경. 약400raw/s | 수동 자세 이동은 요청됨.8raw 제한은 별도 구현 선택. **P 정책 출력에는 미적용** | [robot_cli.cpp:762](<C:/Users/ik/Desktop/yertle-RTOS/policy_onnx/deployments/v6.21.6_model_2850/source/policy/src/robot_cli.cpp:762>) |
| 발 들기/삼각 지지 시험 제한 | 추종30raw /목표 step4·8raw /상대기울기8도면 중단 | 자세 시험은 요청됨. 수치는 명시 요청 못 찾음. **FootLift 모드에만 적용** | [robot_cli.cpp:746](<C:/Users/ik/Desktop/yertle-RTOS/policy_onnx/deployments/v6.21.6_model_2850/source/policy/src/robot_cli.cpp:746>) / [robot_cli.cpp:744](<C:/Users/ik/Desktop/yertle-RTOS/policy_onnx/deployments/v6.21.6_model_2850/source/policy/src/robot_cli.cpp:744>) |
| PID D 정의 /조건부 적분 | D=−측정속도, 포화방향으로 I가 증가할 때 적분 보류 | PID·I 제한은 요청됨. D-on-measurement와 조건부 anti-windup은 구현 선택 | [position_pid.c:43](<C:/Users/ik/Desktop/yertle-RTOS/freeRTOS/firmware/src/position_pid.c:43>) / [position_pid.c:44](<C:/Users/ik/Desktop/yertle-RTOS/freeRTOS/firmware/src/position_pid.c:44>) |
| 관절속도20rad/s 거부 | 위치 차분으로 계산한 관절속도가 넘으면 derivative 무효→55D 입력 실패 가능 | 관절 운동학은 요청됨.20rad/s 임계값은 직접 지정 못 찾음. 목표 속도 제한과는 별개 | [body_velocity.hpp:44](<C:/Users/ik/Desktop/yertle-RTOS/policy_onnx/deployments/v6.21.6_model_2850/source/api/include/yertle/body_velocity.hpp:44>) |
| 관측 시각 거부 | feedback/velocity/IMU age50ms, future skew5ms | 시간 유효성 필요. 공통50ms·미래5ms 세부값은 별도 구현 선택 | [policy_observation.hpp:36](<C:/Users/ik/Desktop/yertle-RTOS/policy_onnx/deployments/v6.21.6_model_2850/source/api/include/yertle/policy_observation.hpp:36>) / [policy_observation.hpp:37](<C:/Users/ik/Desktop/yertle-RTOS/policy_onnx/deployments/v6.21.6_model_2850/source/api/include/yertle/policy_observation.hpp:37>) |
| 지지 접촉 시간/레벨 | contact50ms /release30ms /contact level3 | 지지 접촉 분류는 요청됨. 수치는 설계 기본값. 선속도0 시험에서도 접촉 정보 계산은 남음 | [foot_support.hpp:13](<C:/Users/ik/Desktop/yertle-RTOS/policy_onnx/deployments/v6.21.6_model_2850/source/api/include/yertle/foot_support.hpp:13>) |
| 속도 추정 예측/필터 | 예측 최대100ms, 관절속도 LPF 기본0(꺼짐) | 속도 추정 개선은 요청됨. 수치는 구현 선택. **현재 zero-velocity 옵션에서는 선속도 결과를0으로 대체** | [body_velocity.hpp:49](<C:/Users/ik/Desktop/yertle-RTOS/policy_onnx/deployments/v6.21.6_model_2850/source/api/include/yertle/body_velocity.hpp:49>) / [body_velocity.hpp:43](<C:/Users/ik/Desktop/yertle-RTOS/policy_onnx/deployments/v6.21.6_model_2850/source/api/include/yertle/body_velocity.hpp:43>) |
| 로그 기록 실패 시 종료 | action trace/telemetry 파일 실패 시 제어 중단·해제 | 로그 추가는 요청됨. 로그 실패를 제어 실패로 묶는 것은 별도 구현 선택 | [controller.cpp:18](<C:/Users/ik/Desktop/yertle-RTOS/policy_onnx/deployments/v6.21.6_model_2850/source/policy/src/controller.cpp:18>) / [robot_cli.cpp:195](<C:/Users/ik/Desktop/yertle-RTOS/policy_onnx/deployments/v6.21.6_model_2850/source/policy/src/robot_cli.cpp:195>) |
| 기존 wire 유효성 | raw0..4095, shared Speed 필드 최대값 검증 | 공통0x06 규약/위치모드 호환은 유지. PWM의 Speed0 전용 거부는 제거 | [motor_device.c:145](<C:/Users/ik/Desktop/yertle-RTOS/freeRTOS/firmware/src/motor_device.c:145>) |

**특히 충돌/정체 판정을 끈 것과512raw PID guard, 복귀 중500ms no-progress는 별개의 경로다.**
일반 CLI collision/stall 분류는 꺼졌지만 이 두 조건은 남아 있다. 이제512raw는 직접 목표와 비교하므로,
큰 목표 변경이나2048 복귀가 첫 PID 단계에서 거부될 수 있다. 이를 없앴다고 설명하면 안 된다.
또한 PC의2회 센서 재시도/ACK2회 정책보다 F103의50ms feedback/dt 또는30ms service 조건이 먼저 stop시킬 수 있다.

## 명시적 요청을 확인한 항목 — 유지

- F103에서 위치PID→PWM, PC는 기존 위치 명령; 위치모드 병행, 별도 direct PWM 메시지.
- P5/I3, I 제한100, PWM 상한800: 사용자 수치 지시. D0.10은 “D값 적당하게”라는 조정 위임에 따라 제가 선택한 값.
- D 필터·PWM 상승률 제한 삭제: `70495d8`로 제거되어 현재도 없음.
- 모터별 polarity 반전, shin/thigh 연동 및 ID/부호/기준 자세/캘리브레이션: 사용자의 변경 지시가 있었음. 이번엔 변경하지 않음.
- 모터당2A/총합6A는 PC만 검사: 요청대로 F103에 동일 전류 컷오프 없음.
- 동일 모터60C 이상 fresh3회 후 종료: 사용자 요청. 일반 전압1회 컷오프와 다름.
- 모든 종료에서2048 복귀 우선·실패 시OFF: 사용자 요청. 복귀 실패 판단의 구체적 숫자는 위 표와 같이 제가 선택.
- collision/stall 일반 분류 비활성: 사용자의 오탐 제거 요청. support/contact 추정 데이터 자체는 유지.
- 선속도XYZ0 시험 + IMU yaw 각속도 유지: 사용자 선택. 목표 vx/vy/yaw 명령을0으로 바꾸는 것과 다름.
- 접촉 초기5raw/Unknown 규칙, 준비된 일부 모터에서 Contact 인정: 사용자 확인 지시.
- ACK연속2회/센서조회2회 및 레지스터 재시도5회: 각각 요청 있음. 다른 FW 조건까지2회로 바뀐 것은 아님.
- mode/calibration Flash저장, 재무장 전 rollback스냅샷, 명시적 OFF 재조회: 요청과 시험 결과에 따른 구현.

## 시뮬레이션에만 있는 추가 가정 — 이번엔 변경하지 않음

- 선형 DC 모델 `1.28*(PWM/1000 - motor_velocity/2.3)`와 clamp±1.024Nm.
  **1.28은 PWM800에서1.024Nm가 되도록 둔 모델 가정이지 실물 식별값이 아니다.**
  속도항이 추가하는 감쇠도 실물 검증 없이 가정한 부분이다. 원본 PhysX PD와 같다고 설명하면 안 된다.
- 1초간 native P25/D0.5로 서 있게 하는 warmup, 선택적 clone 동기화, 로그에서 I만 초기화:
  제가 정한 시험 초기화다. 실물의 손 지지 해제/낙하 조건과 동일하다는 증거가 없다.
- 물리0.5ms와 control20ms, randomization 비활성, first contact 이후 통계 제외:
  시험 방법의 선택이며 실물 RTOS 주기/동역학 전체를 재현하지 않는다.
- 기존 개별 관절 토크 적용은 사용자 지시 후 적용한 시험이다. 실제 링크의 물리적 토크 전달 전체를 식별한 것으로 간주하지 않는다.
- 응답 배율은 사용자 지시대로1.0 고정.0.320/0.318 과거 시험은 추가 모터 모델 변경이며 현재에 적용되지 않는다.

## 사본·문서에서 확인한 혼동 요소

- 루트 PC 소스와 활성 deployment 소스는 별개이며 일부 timer/serial 코드가 다르다.
- F103_POSITION_PID.md에 초기P2/I0.5·slew2000·D필터0.04·0x0F 제거 등의 과거 설명이 남아 있었다.
  이번에 현재 P5/I3/D0.10·direct goal·direct PWM 메시지·return-first 동작으로 문서를 수정했다.
- 활성 CLI의 `--acc` 도움말은 기본50으로 남았지만 실제 Options 기본은100이다.
  `firmware_release_may_move_raw2048` 로그 필드도 PWM에서false로 남아 현재 F103 return-first와 맞지 않는다.
  제어 동작과 무관한 오래된 표시/로그 문제로 기록했다. 이번 PWM PID 수정에 PC 실행파일 교체는 포함하지 않았다.
- `diagnose_real_tracking.py`와 예전 artifacts의 profile은 역사 재현용이다. 현재 PWM 경로와 분리해 표시했다.
  과거 결과의48raw/132raw 비교를 삭제 후 직접 목표 PID의 결과인 것처럼 재사용하지 않는다.
- 현재 ONNX 출력에는 임의 action clipping·출력 LPF가 없다. action→angle은 기존 reference+0.25*action,
  angle→raw는 기존 signed/coupled 변환, raw 범위 초과는 명시적 오류다.
- 시뮬레이션 저장소의 별도 미커밋 body-height reward 변경과 v6_22_3설정은 다른 작업 상태로 보존했다.
  요청 근거를 이 감사에서 확정하지 않았으며 이번 커밋에 포함하지 않는다.

## 검증 및 빌드

- PWM 수치/host/actuator/wire/storage 테스트6개 통과. 내부20개 Speed/ACC 조합 비교 포함.
- 기존 위치모드/통신/DMA/heartbeat/register/안전정지 회귀 테스트12개 통과.
- 현재 C와 환산 PID의 Isaac Sim 짧은 연동 시험:2env×50제어 tick(1초), vx=-0.1.
  `max|goal-reference|=0`, shadow PWM 차이0, body contact/guard 없음. **장시간 보행 검증은 아니다.**
- ARM 최종 빌드·센서 스택 정적 예산·Flash layout검증 통과.
- 바이너리: `C:\Users\ik\Desktop\yertle-RTOS\freeRTOS\build\stm32f103rb\firmware\yertle_f103rb.bin`; SHA256 `3fa6a1c6ad80820e07acce21b73bb91bac79d52240dea2455799d7037d292e44`.
- 보드 플래싱/실물 구동은 하지 않았다. 기존 CLI 바이너리의0x06 송신 형식은 호환되므로 PC 실행파일 교체는 필요하지 않다.

`direct_goal_smoke_*`, `source_manifest.json`, `inventory.json`, `smoke.log`, staged변경 및 재현 스크립트를 이 폴더에 보관했다.
