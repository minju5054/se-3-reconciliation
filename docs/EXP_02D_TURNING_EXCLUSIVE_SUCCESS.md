# EXP-02D만 추종 기준을 통과한 곡선 사례 검색

**`v1:episode_000016_transition_02`**를 찾았다. 앞서 직선 사례로 제시한 같은
에피소드의 `transition_00`과 다른 구간이다. 저장된 전환점 B에서 각각 초기화하여
RAW/M1/M3를 3회씩 실행한 결과, 기존 추종 판정에서 M3만 PASS였다.

**세 방법 모두 끝점에는 도달했다.** RAW는 yaw RMS 0.100543 rad로 0.100000 rad
기준을 근소하게 초과했다. 따라서 “RAW는 움직이지 못했는데 Exp02D가 구출했다”는
사례가 아니며, RAW 대비 큰 성공 개선의 증거로 제시해서는 안 된다.

## 실제 GUI 주행 영상

- [세 방법 동시 비교: RAW / 이전 objective / Exp02D](../data/exp02d_turning_search/exp02d-turning-search-20260913/confirmation_00/movies/R1_RAW_BEFORE_Exp02D.mp4)
- [이전 objective / Exp02D 비교](../data/exp02d_turning_search/exp02d-turning-search-20260913/confirmation_00/movies/R1_BEFORE_AFTER.mp4)
- [RAW / Exp02D 비교](../data/exp02d_turning_search/exp02d-turning-search-20260913/confirmation_00/movies/R1_RAW_Exp02D.mp4)
- [Exp02D 개별 GUI 주행](../data/exp02d_turning_search/exp02d-turning-search-20260913/confirmation_00/movies/R1_M3_LOOKAHEAD.mp4)

실제 물리 실행을 녹화했고, 영상은 2배 느리게 재생한다. 세 방법의 B/시간 0/카메라를
동일하게 맞췄다. 먼저 끝난 방법은 `RUN ENDED`로 표시하고 끝난 상태를 유지한다.
화면의 실패 판정 옆에 yaw RMS와 기준을 소수점 네 자리로 표시하여 RAW의 작은
기준 초과를 숨기지 않는다. GUI의 이번 사례 별칭은 R1이다.

## 확인된 수치

| 지표 | RAW | 이전 objective M1 | Exp02D M3 |
|---|---:|---:|---:|
| 기존 추종 판정 | FAIL | FAIL | **PASS** |
| 목표 도달 | 3/3 | 3/3 | 3/3 |
| 위치 RMS | 1.503 cm | 9.531 cm | **1.341 cm** |
| yaw RMS | 0.100543 rad | 0.852554 rad | **0.092446 rad** |
| 실패 항목 | yaw RMS | yaw RMS | 없음 |
| 목표 도달 시간 | 4.7초 | 13.2초 | **4.1초** |
| 측정 이동 거리 | 1.122 m | 2.642 m | **1.102 m** |
| 시작→종료 방향 변화 | 58.48° | 56.80° | **58.27°** |
| 실행 중 방향 변화폭 | 63.46° | 184.99° | **61.01°** |

각 조건의 반복 결과는 표시한 정밀도에서 일치했다. M3의 저장된 곡선은 길이
1.222 m, waypoint yaw 변화폭 51.73°, 선분 진행 방향 변화폭 50.88°다.
실제 M3 주행도 1.10 m를 이동하면서 약 58° 방향을 바꿨으므로 직선 사례가 아니다.
M1에서는 실행 중 방향 변화폭과 이동 거리가 더 커지고 완료도 늦어졌다.
시작→종료 방향 변화는 측정 yaw를 unwrap한 뒤 마지막 값에서 첫 값을 뺀 값이며,
방향 변화폭은 같은 배열의 최댓값에서 최솟값을 뺀 값이다.

RAW의 yaw 기준 초과는 **0.000543 rad(약 0.031°)**다. 이는 판정 경계에 가까운
사례이며, 현재 reset 조건의 결정적 3회 반복을 환경 변화에 대한 강건성 검증으로
해석하지 않는다. 위치/yaw 오차는 각각 자신의 보정 경로를 기준으로 계산한다.
원본 대비 끝점 이동도 M1 16.27 cm, M3 10.74 cm로 달라서 공통 navigation 목표나
충돌 없는 주행을 별도로 검증한 결과는 아니다.

## 검색 방법과 제외한 결과

원본 Exp02D primary의 저장된 959개 전환에서 아래 조건을 미리 적용했다.

- M3 기준 경로 길이 0.5 m 이상.
- waypoint yaw 변화폭과 XY 선분 진행 방향 변화폭이 각각 30° 이상.
- 선분 방향은 길이 0.02 m 이상인 기준 경로 선분에서 측정한다.
- 실제 M3 실행도 이동 거리 0.5 m 이상, yaw 변화폭 30° 이상이어야 한다.
- 기존 Stage0E 추종 기준을 유지하며 RAW와 M1은 실패, M3는 통과해야 한다.

좌표 회전/이동 없이 원본 world XY와 yaw에서 측정했고, yaw unwrap은 방향 변화폭
계산에만 썼다. 이 조건을 만족하는 곡선 후보 87개를 **빠짐없이 세 방법으로 1회씩**
실행하여 261개 screening trial을 저장했다. Screening은 반복 수/목표 필요 횟수만
1회로 표시하고 나머지 수치 기준은 유지했다. 유일한 후보를 원래 3회 기준으로
다시 실행하여 9개 confirmation trial을 얻었다. 목표를 놓친 baseline이 많은 사례를
우선하는 선택 규칙을 사용했지만, 최종 통과 후보는 하나뿐이었다.

| 87개 후보의 1회 screening 결과 | RAW | M1 | M3 |
|---|---:|---:|---:|
| 추종 판정 통과 | 40 | 58 | 39 |
| 각자 끝점 도달 | 70 | 66 | 67 |

이것은 M3 곡선 조건으로 고른 개발 데이터의 사후 사례 검색이다. 위 숫자는 검색한
집합의 기술 통계이며 독립 평가집합의 일반 성능이나 전체 성공률이 아니다. 긍정 사례
하나를 찾았다는 이유로 다른 실패나 baseline의 성공을 제외하지 않았다.

추가로 `v1:episode_000025_transition_02`는 screening에서 M3만 끝점에 도달했으나,
M3 yaw RMS가 0.3350 rad여서 기존 추종 기준을 실패했다. 이 사례를 엄격한 성공으로
바꾸어 부르지 않았고 3회 confirmation 대상으로도 선택하지 않았다.

## 재현과 검증

설정: [exp02d_turning_search.yaml](../configs/exp02d_turning_search.yaml).
새 출력: `data/exp02d_turning_search/exp02d-turning-search-20260913/`.

```bash
./scripts/isaac/run_exp02d_turning_search.sh --run-id NEW_SEARCH_ID
.venv/bin/python scripts/summarize_exp02d_turning_search.py \
  data/exp02d_turning_search/NEW_SEARCH_ID

./scripts/isaac/run_exp02d_objective_video_gui.sh \
  --run data/exp02d_turning_search/exp02d-turning-search-20260913/confirmation_00 \
  --cases R1 --methods M0_RAW M1_HISTORICAL_M4 M3_LOOKAHEAD

.venv/bin/python scripts/encode_exp02d_objective_videos.py \
  --recordings RECORDING_DIRECTORY --output NEW_MOVIE_DIRECTORY \
  --ffmpeg /path/to/ffmpeg --three-way
```

Controller, follower, LightNav, objective, 기존 후보와 원본 기록은 변경하지 않았다.
원본 Hospital collision과 현재 calibrated controller를 사용했다. 각 trial은 같은 B에서
reset 후 1초 settling, physics 1/60초, control 0.1초, 목표 또는 18초 종료다.
원래 온라인 속도/PI/접촉 이력은 복원하지 않는다. Scene 초기 생성 위치도 기존 S1의
B로 고정하고 protocol에 기록하여 headless/GUI 초기화를 맞췄다.

새 임의 전환 loader는 원본 S1/S2/F1 loader와 모든 후보 배열/source hash가 정확히
일치함을 확인했다. 검색 후 원본 hash를 재검증했고, 감사 도구가 261개 trial의
원시 자료·후보·지표·실제 회전·선택 결과를 재계산했다. 9개 confirmation trial도
검증했다. GUI 녹화 3회의 전체 pose 배열은 confirmation 첫 반복과 정확히 일치했다.
95/265/83개 실제 프레임을 각각 기록하고, 영상의 끝난 상태 유지와 느린 재생만 적용했다.
세 화면 비교는 3840×778, 30 fps, 28.4초이며 전체 852프레임을 디코딩 검증했다.
합성 fixture는 SE(2) 회전/이동 불변성, ±π 경계 및 직선/제자리 회전 제외 테스트에만
사용했다. 실제 주행 증거와 구분한다. 전체 테스트는 **506개 통과**했다.
