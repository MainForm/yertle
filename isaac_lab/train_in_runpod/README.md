# Yertle Runpod 학습 자동화

Python 3.11+, Git, Windows OpenSSH(`ssh`, `scp`)가 필요합니다. WSL에서도 Python으로 실행할 수 있지만 `ssh_key` 경로는 Linux 경로로 변경해야 합니다.

## 수행 범위

1. 실시간 GPU 가격·지역별 가용량과 STANDARD 볼륨 지원을 조회합니다.
2. 허용한 RTX GPU 중 저렴한 지역을 선택해 **Pod 한 개**와 **10GB 네트워크 볼륨 한 개**를 만듭니다.
3. GitHub의 `feature/isaac-lab-rl-locomotion`을 clone한 뒤, 실행 시점의 로컬 HEAD bundle을 fetch해 정확한 커밋으로 checkout합니다. GitHub에 push하지 않습니다.
4. `--include-dirty`를 명시한 경우에만 로컬 tracked diff와 허용된 미추적 소스 파일을 별도 적용합니다. 비밀 파일·기존 runs·체크포인트를 무단 포함하지 않습니다.
5. Windows CRLF/Linux LF 차이는 정규화하여 모든 tracked 소스를 SHA-256 검증하고, 추가 파일은 원본 바이트로 검증합니다.
6. 공식 Runpod Ubuntu 22.04 이미지에 **Isaac Sim 5.1.0 / Isaac Lab 2.3.2 / Torch 2.7.0 cu128 / rsl-rl-lib 3.1.2**를 설치합니다. 기존 로컬 컨테이너와 같은 핵심 버전이며 OS/드라이버까지 동일하다는 뜻은 아닙니다.
7. URDF에서 USD를 재생성하고 현재 v6.20.2 profile로 **fresh 300 iteration / 1024 env**를 실행합니다. solver position iteration 32, shoulder ±10°, 속도 초과 penalty 기준 2.30 rad/s를 포함합니다. 모터 토크는 1.9 Nm, PhysX 속도 상한은 4.6 rad/s이며 토크 증대 진단은 반영하지 않습니다.
8. 종료 후 체크포인트·환경 상태·TensorBoard·실행 로그·설정·소스 manifest를 로컬에 다운로드하고, 전송 archive와 파일별 SHA-256을 검증합니다.
9. **검증된 로컬 백업이 있으면 Pod와 이번에 만든 네트워크 볼륨을 모두 삭제하고 GET 404로 재확인합니다.** 정상 완료 후 GPU·저장소 리소스를 남기지 않습니다. 이미 발생한 요금은 없어지지 않습니다.

기본 이미지는 공식 templates 조회의 `runpod-ubuntu-2204` 이미지(`runpod/base:1.0.2-ubuntu2204`)입니다. Isaac Sim 공식 컨테이너 5.1.0은 non-root이며 sudo가 없어, 그대로 임대해 SSH/apt를 설치하는 잘못된 경로는 사용하지 않습니다.

## API 키 등록

기존 환경변수의 값으로는 HTTP 401이 발생했지만, 사용자가 지정한 `C:/Users/ik/Desktop/runpod_issac_sim/runpod_api_key2.txt`의 다른 키로는 HTTP 200 인증에 성공했습니다. 키가 만료됐다고 단정하지 않습니다. MCP 인증과 실행기 인증은 별개입니다.

Runpod Console → Settings/API Keys에서 본인 계정의 새 키를 발급하세요. 이 실행기는 catalog 조회와 Pod·network-volume 생성/조회/중지/삭제 권한이 필요합니다. 키를 채팅, Git, `config.json`, 옛 `runpod_api_key.txt`에 넣지 마세요.

PowerShell에서 현재 터미널에만 안전하게 입력합니다(Windows PowerShell 5.1 포함):

```powershell
$runpodSecret = Read-Host '새 Runpod API key' -AsSecureString
$env:RUNPOD_API_KEY = [System.Net.NetworkCredential]::new('', $runpodSecret).Password
Remove-Variable runpodSecret
```

인증/가용량만 조회하며 과금 리소스를 만들지 않는 검증:

```powershell
python .\run.py --dry-run
```

키는 로컬 프로세스의 환경변수로만 사용하며 Pod로 전달하지 않습니다. 기존 Desktop 폴더의 API 키 파일은 읽지 않습니다. 위 방식은 터미널을 닫으면 사라집니다. 영구 등록이 필요하면 Windows 환경 변수 GUI의 사용자 변수에 `RUNPOD_API_KEY`를 추가한 뒤 새 터미널을 여세요(평문 사용자 환경변수 저장임을 유의).

## 최초 시험

이 폴더에서 실행합니다. `config.json`의 `ssh_key`는 기존 공개 키와 일치하는 로컬 개인 키 경로입니다. 개인 키 내용은 bundle/로그/Pod에 복사하지 않습니다.

```powershell
python .\run.py --include-dirty --iterations 300
```

사용자가 지정한 새 키 파일로 실행하려면 아래처럼 명시합니다. 파일 내용은 로컬 제어 프로세스의 인증에만 사용하며 업로드하지 않습니다.

```powershell
python .\run.py --api-key-file 'C:\Users\ik\Desktop\runpod_issac_sim\runpod_api_key2.txt' --include-dirty --iterations 300 --backup-dir 'D:\project\yertle\runpod_temp'
```

결과는 각 job 폴더의 `checkpoints/`, `training_data/`로 분리하며, 검증 원본은 `files/`, `result.tgz`, `artifact-manifest.json`에 보존합니다.

현재 v6.20.2 shoulder ±10°·속도 초과 패널티 변경과 profile이 미커밋 상태이므로 위 실행은 `--include-dirty`가 필요합니다. 기존 작업 파일을 자동 커밋하거나 amend/push하지 않습니다.

향후 소스를 로컬에서 수정·커밋한 뒤에는 매 실행 시 그 HEAD가 자동 반영됩니다:

```powershell
python .\run.py --iterations 300
```

미커밋 변경이 하나라도 있으면 조용히 무시하지 않고 중단합니다. 명시적 허용 시에만 `--include-dirty`를 붙이세요. 학습 결과의 기본 저장 위치는 `D:/project/yertle/runpod_temp/<job-id>/files/source/isaac_lab/runs/`입니다. `--backup-dir`로 바꿀 수 있습니다. 상태·삭제 확인은 Git ignore된 `.state/<job-id>/state.json`에 기록됩니다.

로컬에 보존한 체크포인트에서 추가 학습:

```powershell
python .\run.py --include-dirty --iterations 300 --resume-checkpoint 'D:\path\model_299.pt'
```

새 Pod/새 지역에 checkpoint를 업로드하므로 과거 Pod GPU를 다시 확보할 필요가 없습니다. 물리 좌표계/action/observation/reward 설정의 호환성은 사용자 책임이며, 다른 버전 policy를 무조건 resume하지 마세요. fresh profile 자체에는 checkpoint를 쓰지 않습니다.

## 자원 부족과 지역 변경

실시간 가용 지역만 후보로 사용하며 **지역 capacity 오류가 명확한 경우에만** 다음 GPU/지역으로 넘어갑니다. 시도 실패로 만든 빈 볼륨은 바로 삭제합니다. 인증·잔액·전역 quota·설정 오류는 다음 지역으로 무작정 재시도하지 않습니다. Pod 생성이 성공한 뒤에는 image pull/설치를 기다릴 뿐 두 번째 Pod를 중복 생성하지 않습니다.

네트워크 볼륨은 지역 간 자동 이동하지 않습니다. 완료된 백업을 로컬에 둔 뒤 다음 실행 때 새 지역으로 올리는 방식입니다. 호스트 장애로 아직 백업되지 않은 결과가 이전 지역 볼륨에만 남아 있다면 별도 복구가 필요합니다. 오래된 볼륨을 지워도 된다고 추정하지 않습니다.

## 비용 보호와 한계

- `max_gpu_hourly_usd=0.90`, `max_seconds=7200`, `max_planned_usd=1.10`. GPU+80GB 임시 디스크+10GB 볼륨의 계획 단가로 더 비싼 후보의 허용 시간을 줄입니다.
- 2026-10-17 조회 기준 RTX 3090 Secure $0.50/hr, STANDARD volume $0.07/GB/month, container disk $0.10/GB/month. 디스크 요금은 730시간/월로 환산한 **계획값**이며 실제 청구액의 강제 상한이 아닙니다. 현재 가격/실제 Pod read-back이 우선합니다.
- 설치/다운로드도 GPU 대여 시간에 포함됩니다. 첫 실행은 느립니다. 이 구성은 커스텀 이미지를 registry에 push하지 않고 재현 가능한 설치를 수행하므로 새 Pod마다 설치 비용이 듭니다.
- **로컬 PC·네트워크는 학습·백업·정리가 끝날 때까지 유지하고 절전을 끄세요.** `--background`는 터미널과 분리된 로컬 제어기를 실행합니다. 로컬 PC 종료/네트워크 단절까지 견디는 서버 측 과금 차단은 아닙니다. 단순히 학습 프로세스가 끝나거나 SSH가 끊겼다고 GPU 요금이 멈추지 않습니다.
- 시간 제한/백업 실패/중단 시에는 기록된 **Pod를 삭제**하고 `/workspace` 네트워크 볼륨만 보존합니다. 임시 컨테이너 디스크와 메모리의 미저장 상태는 복구되지 않습니다. **보존 볼륨 비용은 계속 발생**합니다. API 단절로 삭제 자체가 실패하면 `cleanup_pending` 및 `URGENT`를 남기며 GPU 요금도 계속될 수 있습니다.
- 작업별 고유 ID와 출력 디렉터리를 사용하며 다른 학습의 실행을 막지 않습니다. 같은 작업의 회수·삭제만 `controller.lock`으로 보호합니다. 비정상 종료 후 잠금이 남으면 실제 제어기 프로세스가 종료됐는지 먼저 확인하세요. `.state` 기록을 지우면 추적·정리를 놓칠 수 있습니다.
- 비용 제한은 **작업별**입니다. 두 작업이면 계획 예산·시간당 사용료도 각각 합산됩니다. 계정 전체의 $10 강제 상한이나 서버 측 자동 종료 보장은 아닙니다.

## 여러 학습·백그라운드 실행·미회수 볼륨

이 폴더에서 아래 실행을 반복하면 작업마다 독립된 Pod/볼륨이 만들어집니다. 필요하면 서로 다른 `--profile`을 지정하세요. 각 실행은 유료 작업 하나를 새로 시작하므로 중복 클릭에 주의하세요.

```powershell
python .\run.py --api-key-file 'C:\Users\ik\Desktop\runpod_issac_sim\runpod_api_key2.txt' --include-dirty --iterations 300 --background
```

반환된 PID는 로컬 제어기, log는 진행 로그입니다. API 키는 명령행 값이나 Pod에 넣지 않고 해당 파일을 로컬에서 읽습니다. Codex가 응답을 마칠 때까지 기다릴 필요는 없지만 PC를 끄면 자동 정리가 중단됩니다. 기본 로컬 결과 경로는 `D:/project/yertle/runpod_temp/<job-id>/checkpoints` 및 `training_data`입니다.

Windows의 비대화형 SSH/SCP 자식 프로세스도 `CREATE_NO_WINDOW`로 실행하므로 주기적 상태 조회 때 OpenSSH 창이 뜨지 않습니다. 로컬 제어기 교체가 필요하면 **기존 제어기가 완전히 종료된 것을 확인한 뒤에만** `--monitor <job-id> --background`로 같은 작업에 붙일 수 있습니다. 이 모드는 새 Pod나 학습을 생성하지 않고, 원래 예산 종료 시각과 백업·삭제 흐름을 유지합니다. 원래 제어기에 Ctrl+C를 보내면 오류 정리로 Pod가 삭제될 수 있으므로 단순한 감시 교체 용도로 사용하지 마세요.

완료 흐름:

- 복사 및 SHA-256 검증 성공 → Pod 삭제 확인 → 볼륨 삭제 확인.
- 복사/디스크/검증 실패 → Pod 삭제 확인 → `volume_retained`로 볼륨 ID 기록.
- Pod 삭제 실패 → `cleanup_pending` 기록. 자동 종료됐다고 가정하지 말고 Console/API에서 즉시 확인.

상태·미회수 목록(로컬 기록 조회, 과금 없음):

```powershell
python .\run.py --list-jobs
```

회수(같은 지역에 **유료 임시 Pod**를 만들 수 있음; 결과만 회수하고 학습은 재개하지 않음):

```powershell
python .\run.py --api-key-file 'C:\Users\ik\Desktop\runpod_issac_sim\runpod_api_key2.txt' --recover <job-id>
```

원래 지역의 실시간 가용성·가격을 다시 확인하며, 자리가 없으면 볼륨을 그대로 두고 중단합니다. 체크포인트/학습 로그를 회수·검증한 후 임시 Pod와 볼륨을 삭제합니다. 실패하면 임시 Pod만 삭제하고 볼륨을 다시 보존합니다. 회수는 짧은 SSH/복사 제한시간을 사용하지만 별도 서버 측 과금 차단기는 아닙니다.

백업 없이 결과를 **영구 포기**할 때만 명시적으로 실행:

```powershell
python .\run.py --api-key-file 'C:\Users\ik\Desktop\runpod_issac_sim\runpod_api_key2.txt' --delete-retained <job-id> --confirm-data-loss
```

`volume_retained`이고 Pod가 없는, 이 제어기가 기록한 볼륨만 허용합니다. 사용자의 기존 다른 Pod/볼륨은 일괄 삭제하지 않습니다. 보존 10GB 볼륨은 기존 계획 단가 $0.07/GB/month 기준 약 $0.70/month이며 실제 청구 단가가 우선합니다.

## 검증

```powershell
python -m unittest discover -s . -p 'test_*.py' -v
```

오프라인 테스트는 API를 호출하지 않으며, GPU 선택·가격 cap·재시도 분류·SSH schema·Windows/Linux 소스 검증·archive 손상/경로 공격·삭제 후 재조회·페이지 처리를 검사합니다. **실제 Pod에서 Isaac Lab 300회 학습 성공을 대체하지 않습니다.**

2026-10-17 실제 Runpod 시험 결과:

- 소스: 컨테이너와 동일한 HEAD `8b768ae449fa2f7ee96ac2ceffb99356b88fb784` + 현재 미커밋 패치. 기존 source checkout·API 매핑·로컬에서 진행 중인 학습은 변경하지 않았습니다.
- RTX 3090 / EU-CZ-1, `runpod/base:1.0.2-ubuntu2204`, GPU 단가 $0.50/hr.
- 첫 시도는 SSH 세션에서 EULA 변수가 전달되지 않아 USD 변환에서 실패했습니다. 실패 로그를 백업한 뒤 Pod `ealb8f9zhda0rr`와 볼륨 `7rfz8chmnx`가 자동 삭제됐습니다.
- 후속 시도에서 EULA를 각 프로세스에 명시하고 Torch cu128을 고정했습니다. 런타임에서 누락된 `libxt6`를 보완했으며, 이 보완 사항은 `interventions.json`에 기록되고 향후 bootstrap에도 포함됐습니다.
- 성공 run: `20261017T082527Z-9b1ba1`, fresh PPO **0~299 / 300회**, scalar tag 58개. 원격 및 로컬에서 최종 checkpoint/event 경계와 총 300개 연속 이벤트를 검증했습니다.
- 체크포인트 **0, 50, 100, 150, 200, 250, 299** 총 7개를 로컬 CPU로 전부 로드했습니다. 55차원 입력·12차원 출력과 저장 iteration/다음 iteration을 확인했습니다.
- 결과: `D:/project/yertle/runpod_temp/20261017T082527Z-9b1ba1/`, 파일별 SHA-256 검증 통과.
- Pod `5qil60tma2wn7j` 및 볼륨 `lkhcicjgaw`도 자동 삭제 후 별도 GET으로 404를 재확인했습니다. 최종 계정 조회는 **Pod 0개 / network volume 0개**입니다. 이미 발생한 사용료가 취소된다는 뜻은 아닙니다.
- 오프라인 회귀 테스트는 **21개 통과**했습니다. 지역 capacity 재시도 분기는 오프라인 테스트로 확인했으며, 이번 실물 시험에서는 첫 후보 EU-CZ-1에 생성되어 다른 지역으로의 실제 전환은 발생하지 않았습니다.

이후 동시 실행·보존/회수 개선은 **30개 오프라인 테스트 통과**로 확인했습니다. 전체 제어기의 완료/복사 실패 정리, 회수 성공/재실패/지역 부족, 작업별 잠금과 영속 archive 위치를 포함합니다. 이 개선판으로 여러 유료 Pod를 동시 실행하거나 회수용 Pod를 실제 생성하는 시험은 아직 하지 않았습니다.
# v6.20.3 diagonal support experiment

The default `config.json` now selects `isaac_lab/configs/v6_20_3_training.json`.
Its fresh profile enables `--trot-support-penalties` with unchanged v6.20.2
physics, command ranges, shoulder threshold and network layout.

- `phase_stance_missing_contact_penalty`, weight **-1.0**: fraction of the two
  scheduled diagonal support feet missing current contact (>1 N defines support).
- `all_feet_airborne_penalty`, weight **-2.0**: one when every foot link has
  contact force <=0.1 N, including standing commands and pair transitions.
- Within +/-20 ms of diagonal exchange, missing-stance and existing wrong-phase
  contact costs are suppressed; incorrectly early touchdown negative scores are
  suppressed too. Positive touchdown scores remain. Brief four-foot support is
  thus allowed. Existing swing-contact penalty remains outside that window.
- Read instantaneous net forces, not history-max forces that can hide short
  airborne periods. These are shin-link contacts, not proof of rubber-only
  contact; existing nonsole penalties remain active.

These are **soft reward penalties, not a hard guarantee** of zero flight.
Policy observations/actions, joint axes, API/raw mapping and fixed 2.2 Hz / 50%
duty trot clock are unchanged. The new terms are opt-in; the v6.20.2 profile
does not enable them and remains compatible with its saved environment digest.
When intentionally switching a v6.20.2 checkpoint to the new reward config,
full environment restoration must not silently bypass the config mismatch.
Use this v6.20.3 profile fresh for the planned comparison.

The profile contains a 7000-update budget, but the Runpod controller overrides
it with `config.json`'s **iterations=300**, which is intentionally unchanged.
It also retains existing cost/time caps. This code change does not rent a pod,
upload anything or start cloud training. Do not confuse test duration with the
profile's full budget.
