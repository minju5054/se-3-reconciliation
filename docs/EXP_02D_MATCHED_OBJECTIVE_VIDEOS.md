# EXP-02D: 같은 에피소드의 이전 objective / Exp02D 실제 주행 영상

사용자의 후속 요청에 따라 정지 이미지 대신 실제 Isaac Hospital 물리 실행을 녹화했다.
비교 대상은 원본 FRESH만이 아니라 **이전 objective `M1_HISTORICAL_M4`와
Exp02D `M3_LOOKAHEAD`**다. 같은 세 대표 에피소드에서 RAW/M1/M3를 각각 3회씩,
총 27회 실행했다. 기존 18회 RAW/M3 실험은 보존하고 새 출력에 분리했다.

**이번 세 사례에서는 이전 objective 실패 → Exp02D 성공 사례가 없었다.**
실패/성공 영상을 원하는 결론에 맞추어 선택하거나 판정 기준을 바꾸지 않았다.

## 실제 주행 MP4

왼쪽은 이전 objective, 오른쪽은 Exp02D다. 두 독립 실행의 시작점 B와 시뮬레이션
시간 0을 맞췄고 카메라도 동일하다. **2배 느린 재생**이며 먼저 끝나는 쪽은
`RUN ENDED` 화면을 유지한다. 정지된 로봇 자세를 옮기는 애니메이션이 아니다.

- [S1: 이전 / Exp02D 비교 영상](../data/exp02d_objective_execution/exp02d-objective-matched-20260913/movies_S1/S1_BEFORE_AFTER.mp4)
- [S2: 이전 / Exp02D 비교 영상](../data/exp02d_objective_execution/exp02d-objective-matched-20260913/movies_S2/S2_BEFORE_AFTER.mp4)
- [44번 F1: 이전 / Exp02D 비교 영상](../data/exp02d_objective_execution/exp02d-objective-matched-20260913/movies_F1/F1_BEFORE_AFTER.mp4)
- [S2: 원본 FRESH / Exp02D 비교 영상](../data/exp02d_objective_execution/exp02d-objective-matched-20260913/movies_S2/S2_RAW_Exp02D.mp4)

각 폴더에는 `S1_M1_HISTORICAL_M4.mp4` 등의 개별 고해상도 GUI 영상도 있다.
자홍색은 Exp02D, 주황색은 이전 objective, 회색은 RAW, 초록색은 해당 실행에서
새로 측정한 실제 이동이다. 굵은 기준선이 현재 실행 중인 방법이다.

## 동일 조건 비교 결과

| 사례 | 이전 objective M1 | Exp02D M3 | 해석 |
|---|---|---|---|
| S1, episode 16 / transition 00 | 통과, 목표 3/3, 위치 RMS 1.075 cm | 통과, 목표 3/3, 위치 RMS 0.811 cm | 둘 다 주행 성공. 실패를 성공으로 바꾼 사례 아님 |
| S2, episode 27 / transition 01 | 실패, 목표 0/3, 위치 RMS 16.944 cm | 실패, 목표 0/3, 위치 RMS 0.320 cm | 경로 선에는 더 가깝지만 목표 미도달은 해결하지 못함 |
| F1, episode 44 / transition 04 | 실패, 목표 3/3, yaw RMS 0.250 rad | 실패, 목표 3/3, yaw RMS 0.469 rad | 둘 다 목표에는 도달하지만 회전 품질/명령 포화 기준 실패 |

S1의 실행 시간은 M1 4.4초 / M3 4.2초다. S2는 양쪽 모두 18초 timeout이며,
목표까지 M1 64.28 cm / M3 75.03 cm가 남았다. F1은 M1 0.8초 / M3 0.7초로
매우 짧고 명령 포화 비율은 각각 75.0% / 71.4%다. 각 조건의 3회 반복 결과는
표시한 정밀도에서 일치했다. 같은 초기화의 결정적 반복을 독립 에피소드로 세지 않는다.

물리 추종 판정은 기존 Stage0E의 위치/yaw/포화/목표/진행 기준을 그대로 적용했다.
위치 RMS는 자기 기준 경로까지의 최근접 거리다. 목표 도달도 **각 후보의 끝점**
기준이며, 세 방법의 끝점이 동일하다는 뜻이 아니다. 공통 원래 목적지에 도달했는지,
지시를 수행했는지, 충돌 없이 이동했는지를 이 판정만으로 알 수 없다.

## 물체에 막히면 FRESH 추론이 틀린 것인가?

충돌 장면 하나로 원인을 FRESH에 배정할 수는 없다.

| 확인할 상황 | 구분할 문제 |
|---|---|
| 원본 FRESH부터 로봇 몸체가 장애물과 겹치는 방향으로 향함 | FRESH 경로의 주행 가능성, 입력 관측/좌표/환경 일치 여부 |
| 원본은 지나가지만 보정하면서 장애물 쪽으로 이동함 | objective의 경로 변형과 장애물 제약 부재 |
| 기준 경로는 지나갈 수 있지만 실제 로봇이 선을 벗어나 충돌함 | controller 추종과 동역학 |
| 선을 잘 따라가도 문턱에서 멈추거나 바퀴가 헛돎 | 문턱 높이, 바퀴 접촉/마찰, 로봇의 지형 통과 가능성 |

S2의 RAW는 목표 3/3에 도달했으나 yaw 기준은 실패했다. M1/M3는 수납 카트 앞에서
전진하지 못하는 모습이 관찰된다. M3의 마지막 관측에서 전진 명령은 0.358 m/s,
측정 전진 속도는 약 -0.00025 m/s, 왼쪽 앞/뒤 바퀴는 7.25/7.01 rad/s였다.
즉 바퀴는 돌지만 차체는 거의 움직이지 않는 상태가 실제 관측에 있다.

또한 S2 M3는 RAW 대비 경로 XY 변형 RMS 약 24.68 cm, 끝점 이동 약 35.10 cm다.
M1도 끝점을 약 29.28 cm 옮겼다. 이 값은 같은 waypoint 행 사이의 world XY
유클리드 거리로 계산한 값이다. 기존 보고서의 SE(2) 로그 기반 변형 지표와 구분한다.
따라서 이 실패를 원본 FRESH의 잘못으로만 설명할 수 없다. 다만 접촉력/접촉 물체
로그나 장애물 제거 대조 실험은 수행하지 않아 충돌 원인을 완전히 분리한 것은 아니다.

Exp02D는 연결 순간에 controller가 바라보는 방향을 objective에 반영하는 변경이다.
장애물 회피, 로봇 몸체 여유 공간, 문턱 통과 가능성 항을 추가한 실험은 아니다.

## Exp02D의 효과를 어디까지 말할 수 있는가?

같은 에피소드, 동일 FRESH, 동일 B/초기 상태, 동일 controller/환경에서 objective만
바꾸는 비교가 필요하다는 지적은 맞다. 이번 영상이 그 비교다. 하지만 세 사례에서
주행 실패를 성공으로 바꾼 결과는 없으므로 그런 효과를 증명했다고 할 수 없다.

기존 Exp02D S1의 전환 명령 점수는 M1 0.8793 → M3 0.0793으로 개선됐다. 이것은
저장된 B에서 이전 OLD 명령과 새 명령의 차이를 계산한 결과다. 이번 정지 상태 reset
영상이 원래 온라인 전환 연속성을 재현했다는 뜻은 아니다. S1에서 불필요한 경로
변형은 줄었지만 양쪽 모두 주행 가능했고, S2에서는 낮은 명령 점수가 목표 도달로
이어지지 않았다. F1은 반례로 함께 제시한다.

과거 44번의 **OLD 활성 구간 추종 실패**와 이번 **B 이후 보정 FRESH 실행**을
비교해 개선이라고 말해서도 안 된다. 비교하는 구간이 다르다. 일반적 성능 주장은
미리 정한 공통 지표와 더 다양한 독립 에피소드의 paired 결과가 필요하다.

## 재현과 기록

새 정량 run: `data/exp02d_objective_execution/exp02d-objective-matched-20260913/`.
원본 후보, controller, follower, solver 및 LightNav 코드는 변경하지 않았다.
같은 source Hospital collision, 1/60초 physics, 0.1초 control, B에서 reset 후 1초
settling이다. 원래 온라인 속도/PI/접촉 이력은 복원하지 않았다. 좌표는 원본 Isaac
world XY 미터, +Z 반시계 yaw 라디안이며 정렬/회전/스케일 변환을 추가하지 않았다.
원본 observation/readiness/switch 시각과 새 실행 UTC, 샘플 시각을 기록했다.

```bash
./scripts/isaac/run_exp02d_physical_execution.sh \
  --config configs/exp02d_objective_execution.yaml --run-id NEW_RUN_ID

./scripts/isaac/run_exp02d_objective_video_gui.sh \
  --run data/exp02d_objective_execution/NEW_RUN_ID
# S2 RAW 진단까지 포함하려면: --cases S2 --methods M0_RAW M1_HISTORICAL_M4 M3_LOOKAHEAD

.venv/bin/python scripts/encode_exp02d_objective_videos.py \
  --recordings RECORDING_DIRECTORY --output NEW_MOVIE_DIRECTORY \
  --ffmpeg /path/to/ffmpeg
```

녹화는 physics 3 step마다 실제 Isaac 클라이언트 창을 PNG로 읽는다(시뮬레이션
20 fps). 렌더링/캡처 동안 물리를 pause하고 시간이 증가하지 않았는지 확인한다.
모든 프레임의 시각/자세를 raw telemetry와 대조하고 녹화 주행 전체를 정량 첫 반복과
비교한다. PNG와 telemetry는 별도 raw 출력으로 보존한다. MP4는 시간축을 2배 늘려
인코딩하며 30 fps 변환은 프레임 반복만 사용한다. 위치/회전 보간은 없다. 전체 창을
비율 유지 축소하고 두 실행을 좌우로 배치한 과정, 입력 hash, FFmpeg 명령은 각
`manifest.json`에 기록한다. 모든 영상/원시 데이터는 Git에서 제외한다.

완료 검증: 새 정량 27회 strict validator 통과, 녹화 7회의 전체 pose 배열은 각각
정량 첫 반복과 정확히 일치했다. 실제 GUI 프레임 1,265개의 시각/자세/hash를 확인했다.
비교 MP4는 S1 10.8초, S2 38.0초, F1 3.6초이며 S2 RAW 비교도 38.0초다.
네 비교 영상의 전체 디코딩과 시각적 샘플을 검토했고, S2의 로컬 플레이어 재생을 확인했다.
전체 테스트는 **503개 통과**했다.

후속 [곡선 사례 검색](EXP_02D_TURNING_EXCLUSIVE_SUCCESS.md)에서는 기존 세 대표 밖의
87개 곡선 전환을 비교했다. 16번 transition 02에서 M3만 추종 기준을 통과했지만
세 방법 모두 끝점에는 도달했고 RAW의 yaw 기준 초과는 근소했다. 기존 세 대표의
결과와 원본 기록은 그대로 보존한다.
