# EXP-02D 보정 경로의 Jackal 물리 실행 — 2026-09-13

저장된 M3 보정 경로를 **Hospital에서 현재 controller로 실제 시뮬레이션 실행**했다.
S1은 추종 기준을 통과했다. S2는 목표에 도달하지 못했고, F1은 목표에 도달했지만
회전 추종 품질 기준을 통과하지 못했다. 과거 EXP-02D의 S/F 이름은 첫 전환 명령
점수에 관한 분류이며, 이번 물리 실행의 성공/실패를 뜻하지 않는다.

이번 결과는 기존 EXP-02D 이후의 별도 진단이다. 기존 primary의
`candidate_execution: false`와 모든 원본 결과는 그대로 보존했다. 새 결과를 과거
실험에서 이미 확인한 것으로 소급하여 해석하지 않는다.

## 실행 조건

- 입력: `data/exp02d_lookahead_direction/exp02d-lookahead-primary-20260910T171139Z/`
- 새 정량 출력: `data/exp02d_physical_execution/exp02d-physical-20260913T072000Z/`
- 설정: [exp02d_physical_execution.yaml](../configs/exp02d_physical_execution.yaml)
- S1/S2/F1 × RAW/M3 × 3회 = **18회**. 후보를 다시 최적화하지 않고 저장된 배열을 그대로 사용했다.
- RAW는 원본 EXP-02D에서 선택한 FRESH suffix다. 전체 raw chunk를 다시 선택하거나
  시작점 B를 waypoint로 삽입하지 않았다. M3와 동일한 동결 controller를 사용했다.
- 원본 DATA-02의 Hospital/Jackal asset과 authored collision을 사용했다. 별도 바닥을
  추가하지 않았다. 새 LightNav 추론, 장애물 회피 planner 또는 controller 조정은 없다.
- 각 trial에서 저장된 switch B의 world XY/yaw에 초기화하고 1초 settling 후 실행했다.
  원본의 직전 속도, PI 누적값, 접촉 이력과 OLD 주행은 복원하지 않았다.
- 물리 간격 1/60초, 제어 간격 0.1초. 같은 follower/`pi_strong` 실행 보정기를 사용했다.
- 기존 follower가 목표 판정하면 종료하고, 아니면 18초에 종료한다. 목표 허용 오차는
  위치 8 cm / yaw 0.08 rad다. 모드별 실행 시간이 같지 않을 수 있다.

## 결과와 성공/실패의 의미

판정에는 기존 Stage 0-E의 `evaluate_scenario`와 절대 기준을 그대로 사용했다.
위치 RMS < 15 cm, yaw RMS < 0.10 rad, 최종 위치 오차 < 20 cm, 최종 yaw 오차
< 0.10 rad, 포화 비율 ≤ 20%, 목표 도달 최소 2/3회 및 기존 진행/수치 안정성
기준을 함께 확인한다. 결과에 맞추어 기준을 바꾸지 않았다.

위치 RMS는 실제 위치에서 **각자 실행한 기준 경로**의 가장 가까운 선분까지의 거리다.
시간별 waypoint 오차나 원래 FRESH 의도 보존 점수가 아니다. 아래는 각 3회 평균이며,
반복 간 값은 표시한 정밀도에서 일치했다.

| 사례/경로 | 목표 도달 | 위치 RMS | yaw RMS | 실행 시간 | 물리 추종 판정 |
|---|---:|---:|---:|---:|---|
| S1 RAW | 3/3 | 0.825 cm | 0.0521 rad | 4.1초 | 통과 |
| S1 M3 | 3/3 | **0.811 cm** | **0.0565 rad** | **4.2초** | **통과** |
| S2 RAW | 3/3 | 8.541 cm | 0.4797 rad | 16.8초 | yaw RMS 실패 |
| S2 M3 | **0/3** | 0.320 cm | 0.0416 rad | **18초** | **목표/최종 위치/진행 기준 실패** |
| F1 RAW | 0/3 | 35.599 cm | 0.7097 rad | 18초 | 여러 기준 실패 |
| F1 M3 | **3/3** | 5.530 cm | **0.4691 rad** | **0.7초** | **yaw RMS/포화 기준 실패** |

**S1 — 성공 사례:** `v1:episode_000016_transition_00`. 보정 경로를 따라 목표까지
이동하며 최대 공간 이탈은 2.254 cm다. RAW도 통과했으므로 이 사례는 M3의
실행 가능 사례이며, RAW보다 크게 개선했다는 증거는 아니다.

**S2 — 목표 미도달 실패:** `v1:episode_000027_transition_01`. M3는 경로 선에
가까이 붙어 있지만 끝까지 가지 못했다. 18초 뒤 목표까지 75.03 cm가 남았다.
GUI에서는 수납 카트 앞에서 더 전진하지 못하는 모습이 보였다. 보정 경로의 장애물
간섭이 의심되지만, 별도 접촉력 계측/충돌 원인 분리 검사는 하지 않았다. 이 사례는
작은 횡방향 오차와 낮은 전환 명령 점수만으로 전체 주행 성공을 판정할 수 없음을 보여준다.

**F1 — 목표 도달 후에도 품질 기준 실패:** `v1:episode_000044_transition_04`.
M3는 0.7초 뒤 목표 허용 범위에 들어왔지만, 그 과정의 yaw RMS가 0.4691 rad였고
명령 포화 비율은 71.43%였다. 각각 0.10 rad / 20% 기준을 넘었다. M3 경로가 약
15.16 cm로 짧으므로 단순히 끝점에 가까워졌다는 사실만으로 정상 추종이라 하지 않는다.
이는 과거 원본 44번 OLD 주행의 큰 위치 이탈을 재현한 검사가 아니다. 이번에는
그 뒤에 연결될 **보정된 FRESH 경로**를 실행한 것이다.

## GUI 사용

```bash
./scripts/isaac/run_exp02d_physical_execution_gui.sh \
  --run data/exp02d_physical_execution/exp02d-physical-20260913T072000Z \
  --case success --real-time-factor 0.15
```

기본 GUI는 M3를 새 물리 실행하고, 완료 후 물리를 정지한 채 화면을 유지한다.
`Run SUCCESS case (S1)`, `Run FAILURE case (F1)`, `Run TIMEOUT case (S2)` 버튼으로
사례를 바꾼다. 버튼을 누를 때마다 reset 후 새 물리 주행을 하며, 저장된 자세를
순서대로 덮어쓰는 재생이 아니다. 같은 사례 버튼을 다시 눌러 반복할 수 있다.

- 자홍: 저장된 M3 보정 경로.
- 초록: 이번 GUI에서 새로 측정한 실제 Jackal 궤적.
- 회색: RAW 기준 경로, 비교용 배경.
- 노랑: 저장된 switch B.
- 오른쪽 평면 그림: **정량 primary의 측정 경로**. live 궤적과 구별하여 표기하며,
  특히 F1처럼 로봇 아래에 가려지는 짧은 경로를 확인하는 데 사용한다.

경로 선들은 기본적으로 동일한 world Z=0.035 m의 바닥 overlay다. 실제 XY는
정렬/확대/평행이동하지 않는다. 높이만 표시용이며 waypoint의 물리 높이가 아니다.
실내 카메라와 보조 조명은 시각화만 바꾸고, 렌더링/종료 화면에서는 물리 시간이
증가하지 않는지 확인한다. 느린 재생 배율은 wall-clock 표시 속도이며 물리/control
간격을 바꾸지 않는다.

`--case failure`는 F1, `--case S2`는 목표 미도달 사례를 바로 연다.
`--compare-raw`를 추가하면 RAW도 먼저 실제 실행해 주황 궤적을 비교한다.
`--both --no-hold`는 S1/F1 자동 실행·캡처 후 종료하는 검증용 옵션이다.
세 사례를 순서대로 보려면 `--all-cases --case-hold-seconds 8`을 사용한다.
편집기의 Stop으로 시간이 초기화되면 완료된 관측을 보존하고 재실행 안내를 표시한다.

각 GUI 실행은 새 `gui/<UTC>/` 아래 raw 관측, 지표, 원본/코드 hash, 카메라 계획,
화면 캡처 및 정량 첫 반복과의 비교를 저장한다. GUI는 정량 18회에 포함하지 않는다.
최초 RAW/M3 비교 GUI의 S1/F1 실제 pose 배열은 정량 첫 반복과 완전히 일치했다.
다른 GUI 재실행은 자체 관측과 차이를 별도로 보존하며, 일치를 가정하지 않는다.

## 그래프

- [S1 성공 GUI 화면](../data/exp02d_physical_execution/exp02d-physical-20260913T072000Z/gui/2026-09-13T075333.852691_0000/00_S1/presentation_window.png)
- [S2 목표 미도달 GUI 화면](../data/exp02d_physical_execution/exp02d-physical-20260913T072000Z/gui/2026-09-13T075333.852691_0000/01_S2/presentation_window.png)
- [F1 회전 품질 실패 GUI 화면](../data/exp02d_physical_execution/exp02d-physical-20260913T072000Z/gui/2026-09-13T075333.852691_0000/02_F1/presentation_window_reviewed.png)
- [S1 성공 비교](../data/exp02d_physical_execution/exp02d-physical-20260913T072000Z/plots/S1_physical_comparison.png)
- [S2 목표 미도달 비교](../data/exp02d_physical_execution/exp02d-physical-20260913T072000Z/plots/S2_physical_comparison.png)
- [F1 회전 추종 품질 실패 비교](../data/exp02d_physical_execution/exp02d-physical-20260913T072000Z/plots/F1_physical_comparison.png)

## 정량 재실행

```bash
./scripts/isaac/run_exp02d_physical_execution.sh --run-id exp02d-physical-new
MPLCONFIGDIR=/tmp/exp02d-physical-mpl .venv/bin/python \
  scripts/summarize_exp02d_physical_execution.py \
  data/exp02d_physical_execution/exp02d-physical-new
MPLCONFIGDIR=/tmp/exp02d-physical-mpl .venv/bin/python \
  scripts/summarize_exp02d_physical_execution.py \
  data/exp02d_physical_execution/exp02d-physical-new --gui-plots
```

새 run ID를 사용한다. 이미 존재하는 run/plot 디렉터리를 덮어쓰지 않는다.
`EXP02D_PHYSICAL_COMPLETE`, 18개 trial의 `summary.json`, summarizer 검증을 모두
확인하며 Isaac 프로세스의 종료 코드만으로 완료를 판단하지 않는다.
`--validate-only`는 이미 생성된 결과를 읽기만 하며 재검증한다.

원본 EXP-02D 및 DATA-02 입력은 hash 검증 후 읽기만 했다. raw 관측과 derived 지표,
참조 경로 복사본, GUI, 그래프를 분리했고 모든 새 출력은 Git에서 제외한다.
좌표는 Isaac world X/Y 미터, +Z 기준 반시계 yaw 라디안이다. 원본 observation,
readiness, switch 시각과 새 trial의 UTC 시작/종료 및 실행 시간이 기록되어 있다.
row 0은 settling 후 관측이고, 이후 row의 명령은 그 관측으로 끝나는 물리 구간의
명령이다. 새 추론 readiness는 해당 없으며 waypoint 시간을 만들지 않았다.

이 검사는 초기 상태를 reset한 시뮬레이션의 경로 추종 진단이다. 원래 온라인 주행
연속성, 원래 지시의 성공, 전체 주행 안전성이나 실제 로봇 성능을 검증하지 않는다.
controller, objective, LightNav 및 원본 raw VLA 출력은 변경하지 않았다.

최종 GUI `2026-09-13T075333.852691_0000`의 S1/S2/F1 측정 pose 배열은 각각 정량
첫 반복과 정확히 일치했다. 세 사례의 GUI 화면을 시각적으로 검토하고 source/candidate
hash를 다시 확인했다. 전체 테스트는 **488개 통과**했다.
