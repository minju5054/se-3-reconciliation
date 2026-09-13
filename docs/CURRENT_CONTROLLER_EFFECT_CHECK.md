# 현재 controller 효과 확인 — 2026-09-13

현재 보정기는 완만한 곡선과 S자 경로를 따라가는 데 효과가 있다. 그러나 제자리
회전에서는 목표 회전 속도를 지나치는 현상이 있고, 44번 OLD를 크게 벗어나는 문제도
남아 있다. 기존 Stage 0-E 판정은 여전히 `EXECUTION_PLATFORM_NOT_READY`다.

이번 작업은 **현재 보정기를 켠 경우와 끈 경우를 새 Isaac 물리 실행으로 비교**했다.
controller를 추가 수정하거나 gain을 재조정하지 않았다. `nominal`은 현재 코드에서
실행 보정을 끈 조건이고, `calibrated`는 동결된 `pi_strong` 보정기를 켠 조건이다.
과거 코드 전체를 checkout한 비교가 아니다. DATA-02 수집에서도 이미 이 보정기가
사용되었으므로, 아래 표는 원본 수집 결과를 새로운 수정 코드로 교체한 것이 아니다.

## 비교 조건과 자료

| 검사 | 조건 | 새 정량 실행 수 |
|---|---|---:|
| 기존 Stage 0-E 재실행 | 7개 제어 검사용 경로 + composite 1개 + 저장 OLD 3개, 두 모드, 각 3회 | 66 |
| DATA-02 OLD 검사 | 14번/44번 transition 04, 두 모드, 각 3회 | 12 |
| 제자리 회전 검사 | 목표 ω = −1.5, −0.3, +0.3, +1.5 rad/s, 두 모드, 각 3회 | 24 |

총 **102회 정량 실행**과 별도의 S자 경로 GUI 시연을 완료했다. 반복 간 결과는
표시한 정밀도에서 같았다. 동일 초기화 조건의 반복이며, 다양한 환경에 대한
통계적 일반화 검사가 아니다. 인공적으로 만든 경로와 회전 명령은 엔지니어링
테스트/시연용 fixture이며 reconciliation 연구의 실험 증거로 사용하지 않는다.

- 기존 검증 출력: `data/stage0/closed_loop_execution_validation/controller-current-20260913T062219Z/`
- 추가 검사 출력: `data/controller_effect_check/controller-current-20260913T062219Z-r2/`
- 추가 검사 설정: [controller_effect_check.yaml](../configs/controller_effect_check.yaml)
- 저장 OLD 원본: `data/data02_online_successive_v1/data02-online-successive-primary-v1/`
- 실행 시 Git HEAD: `b9efd18`; 실행 당시 작업 트리의 새 검사 코드는 `protocol.json`에
  별도 SHA-256으로 기록했다.
- 동결 모델 SHA-256:
  `40821584e14a3f444fdd19ff27acc03e752ec3d0c5f2e8635b824a1419a81464`
- controller 코드 SHA-256:
  `0bc4a97b4dac6407fe9bedd9d37451ab2d53a19c8efc396d84f496b8115c04a5`

현재 보정기는 회전 feedforward 약 7.353배, PI의 P=3/I=2, 실행 ω 한도
8 rad/s를 사용한다. 전진 명령은 직접 보정하지 않는다. 두 모드 사이에는 보정기
사용 여부만 바뀐다. 같은 follower라도 로봇 움직임에 반응하므로 경로 추종 중 생성되는
명령열까지 같다는 뜻은 아니다. 회전 검사는 두 모드에 같은 상수 목표 명령을 준다.

## 경로 추종 결과

위치 RMS는 매 물리 관측 위치에서 기준 경로의 가장 가까운 선분까지의 거리를
제곱 평균한 뒤 제곱근을 취한 값이다. 시간별 waypoint 오차가 아니다.
아래 값은 각 조건의 3회 평균이다.

| 경로 | 보정 끔 위치 RMS | 현재 보정 위치 RMS | 현재 보정의 해석 |
|---|---:|---:|---|
| 완만한 좌회전 | 11.67 cm | 0.35 cm | 기준 경로를 잘 따라감 |
| S자 | 9.65 cm | 0.98 cm | 방향이 바뀌는 경로도 개선 |
| 14번 transition 04 OLD | 6.71 cm | 1.45 cm | 3회 모두 목표 판정, 약 5.1초 |
| 44번 transition 04 OLD | 21.80 cm | 17.32 cm | 평균은 감소했으나 큰 이탈이 남음 |

**44번의 최대 공간 이탈은 오히려 27.04 cm → 37.33 cm로 커졌다.** 현재 보정은
약 7.5초 뒤 목표 판정에 들어갔지만, 이것만으로 정상 추종으로 볼 수 없다. 기존
follower의 목표 위치 허용 오차는 8 cm이고, 이 OLD 자체도 약 6.93 cm로 짧다.
14번 최대 이탈은 8.41 cm → 2.93 cm였다.

추가 OLD 검사는 원본에서 OLD가 활성화된 **직전 switch B의 world XY/yaw**에
로봇을 놓고, 평지에서 초기화 후 실행했다. 원본 Hospital 장면, 직전 속도, 접촉 상태,
PI 누적값, 온라인 교체 이력은 복원하지 않았다. 1초 settling 뒤 실제 시작 자세도
따로 기록했다. 종료는 기존 follower의 목표 판정 또는 8초 timeout이다. 두 모드의
실행 길이가 다르므로 표의 RMS는 같은 종료 규칙에서의 전체 실행 값이며 동일 시간
구간끼리의 비교는 아니다. 원본 온라인 44번 구간의 RMS와 직접 혼동하면 안 된다.

기존 Stage 0-E의 절대 기준은 바꾸지 않았다. 현재 보정은 제어 검사용 7개 경로 중
5개가 통과했고, 요구한 최소 6개에 미달했다. 강한 좌/우회전의 yaw RMS는 각각
0.1731 / 0.1703 rad로 0.10 rad 기준을 넘었다. composite와 저장 EXP-02B OLD
3개는 통과했지만, 플랫폼 전체 판정을 통과시키지는 못했다.

## 제자리 회전 결과

초기화 후 0.5초 정지, 전진 목표 `v=0`으로 2초 회전, 0.5초 정지를 실행했다.
아래 측정 ω는 회전 구간 마지막 0.5초의 평균이다. 이는 관측 창의 이름이며
완전히 안정된 상태에 도달했다는 판정은 아니다. XY 이동은 회전 직전 관측한
articulation root의 world XY에서 회전 종료 시점까지의 직선거리다.

| 목표 ω (rad/s) | 보정 끔 측정 ω | 현재 보정 측정 ω | 보정 끔 XY 이동 | 현재 보정 XY 이동 |
|---:|---:|---:|---:|---:|
| −1.5 | −0.620 | −2.536 | 12.40 cm | 29.99 cm |
| −0.3 | −0.058 | −0.435 | 3.59 cm | 11.43 cm |
| +0.3 | +0.096 | +0.456 | 2.80 cm | 10.95 cm |
| +1.5 | +0.483 | +2.411 | 11.61 cm | 10.22 cm |

쉽게 말하면, 보정 전에는 요청한 것보다 덜 돌았고 현재 보정은 이 조건에서 요청한
것보다 더 돌았다. ACTIVE 전체의 회전 속도 RMS 오차는 네 조건 중 세 조건에서
증가했다. −0.3 rad/s 조건은 0.248 → 0.236 rad/s로 조금 감소했다.

XY 이동이 모든 지표에서 커진 것은 아니다. +1.5 rad/s의 종료 이동은 감소했지만,
회전 중 최대 이동은 11.61 → 17.10 cm였다. −1.5 rad/s의 최대 이동은
12.40 → 31.43 cm였다. 위치 측정점은 로봇 articulation root이며, 질량중심이나
바퀴 접촉력의 직접 측정이 아니다. 이 값만으로 미끄러짐/접촉/모델 원인을 확정하지 않는다.

이번 검사는 **현재 보정 전체를 켰을 때 이 회전 조건에서 과도한 응답이 생긴다**는
것을 확인한다. feedforward, P, I, 명령 한도 중 어느 항목이 원인인지는 각각
분리하지 않았다. 원본 44번의 유일한 원인이나 모든 DATA-02 데이터의 무효를
판정하는 검사도 아니다. 그 판단에는 원본 초기 동역학과 온라인 전환 조건을
복원한 별도 검사가 필요하다.

## 그래프와 GUI

- [14번/44번 OLD와 실제 경로 비교](../data/controller_effect_check/controller-current-20260913T062219Z-r2/plots/saved_old_comparison.png)
- [제자리 회전 속도와 XY 이동](../data/controller_effect_check/controller-current-20260913T062219Z-r2/plots/rotation_response_and_drift.png)
- [S자 GUI 최종 화면](../data/stage0/closed_loop_execution_validation/controller-current-20260913T062219Z/gui_metadata/controlled-20260913T063329Z/final_viewport.png)

그래프에서 파랑은 저장 OLD, 주황은 보정 끔, 초록은 현재 보정이다. OLD 그래프의
삼각형은 각 실제 실행의 시작, X는 종료를 나타낸다. 각 모드의 3회 선이 겹쳐 있다.
GUI에서도 파랑은 기준, 주황/빨강은 보정 끔, 초록은 현재 보정이다. GUI 경로는
world XY를 그대로 사용하며 선을 보기 위한 높이만 올린다.

S자 GUI는 저장 자세 재생이 아니라 두 모드의 물리 실행이다. 별도 진단 시연으로
저장되며 102회 정량 실행에 포함하지 않는다. 저장 GUI 경로와 정량 실행 첫 반복의
경로가 일치하는지도 확인했다. 아래 명령은 실행 후 최종 화면을 유지한다.

```bash
./scripts/isaac/run_jackal_closed_loop_execution_validation_gui.sh \
  data/stage0/closed_loop_execution_validation/controller-current-20260913T062219Z \
  --suite controlled --scenario s_curve --real-time-factor 0.5
```

## 재실행 및 데이터 규칙

아래 ID는 예시다. 출력이 이미 있으면 새로운 ID를 사용한다. 기존 출력을 덮어쓰지 않는다.

```bash
./scripts/isaac/run_jackal_closed_loop_execution_validation.sh --run-id controller-check-new
MPLCONFIGDIR=/tmp/controller-check-mpl .venv/bin/python \
  scripts/summarize_closed_loop_execution_validation.py \
  data/stage0/closed_loop_execution_validation/controller-check-new

./scripts/isaac/run_current_controller_effect_check.sh --run-id controller-check-new
MPLCONFIGDIR=/tmp/controller-check-mpl .venv/bin/python \
  scripts/summarize_controller_effect_check.py \
  data/controller_effect_check/controller-check-new
```

추가 검사는 로그의 `CONTROLLER_EFFECT_COMPLETE`와 36개 trial을 담은
`summary.json`, summarizer의 검증 완료를 함께 확인한다. Isaac 종료 코드만으로
완료를 판단하지 않는다. summarizer는 이미 만들어진 `plots/`를 덮어쓰지 않는다.

원본 DATA-02를 읽기만 하고, 참조의 world 좌표 재구성에는 기존 검증된
`load_active_old_interval`을 썼다. source hash를 실행 전후 확인했다. 새 관측은
각 trial의 `raw/`, 계산된 지표와 provenance는 `derived/`, 그래프와 집계는
`plots/`에 분리한다. 설정 snapshot, controller 모델/코드/config hash 및 입력
source hash는 실행별 protocol에 기록한다. 전체 출력은 Git에서 제외한다.

좌표는 Isaac world X/Y 미터, +Z 기준 반시계 yaw 라디안이다. 추가 정렬/회전/평행이동은
없다. 그래프의 cm는 m × 100이다. 물리 간격은 1/60초, 제어 간격은 0.1초다.
row 0은 settling 후 관측, 이후 row의 명령은 해당 관측으로 끝나는 물리 구간에
적용된 값이다. PI는 직전 제어 구간에서 관측한 회전 속도를 사용한다. 초기/실행
타임스탬프, 원본 관측/활성화/switch 시각 및 UTC 시작/종료를 기록했다. 새 VLA
추론이 없으므로 새 inference readiness 시각은 해당 없음이며 waypoint에 시간을
새로 부여하지 않는다.

검증: 전체 pytest **468 passed**, 새 시간/회전 검사 포함 관련 27 tests 통과,
compileall와 shell syntax/whitespace 검사 통과. 기존 Stage 0-E 66개 및 추가
36개 trial의 strict validator와 raw hash 검사를 통과했다. GUI와 두 그래프를
시각적으로 확인했다. controller/follower와 원본 VLA 출력은 수정하지 않았다.
