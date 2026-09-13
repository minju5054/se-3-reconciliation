# EXP-02D만 추종 기준을 통과한 곡선 사례 검색

**`v1:episode_000016_transition_02`**를 찾았다. 앞서 직선 사례로 제시한 같은
에피소드의 `transition_00`과 다른 구간이다. 저장된 전환점 B에서 각각 초기화하여
RAW/M1/M3를 3회씩 실행한 결과, 기존 추종 판정에서 M3만 PASS였다.

**세 방법 모두 끝점에는 도달했다.** RAW는 yaw RMS 0.100543 rad로 0.100000 rad
기준을 근소하게 초과했다. 따라서 “RAW는 움직이지 못했는데 Exp02D가 구출했다”는
사례가 아니며, RAW 대비 큰 성공 개선의 증거로 제시해서는 안 된다.

## 실제 GUI 주행 영상

- [OLD/FRESH 표시 추가: 이전 objective / Exp02D 비교](../data/exp02d_turning_search/exp02d-turning-search-20260913/confirmation_00/movies_old_fresh_context/R1_BEFORE_AFTER.mp4)
- [OLD/FRESH 표시 추가: RAW / 이전 objective / Exp02D 세 방법 비교](../data/exp02d_turning_search/exp02d-turning-search-20260913/confirmation_00/movies_old_fresh_context/R1_RAW_BEFORE_Exp02D.mp4)
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

## 후속 질문: 판정 기준과 목표 주변 회전

2026-09-13 저장된 9개 confirmation trial을 읽어 원래 follower를 다시 계산했다.
새 물리 실행이나 controller 수정은 없었다. 각 control 경계의 실제 pose에서 계산한
총 660개 `[v, omega]` 명령이 저장 명령과 `atol=1e-12, rtol=0`으로 일치했다.
입력 hash, 처리 코드/hash, 설정과 각 시점의 분류는
`../data/exp02d_turning_search/exp02d-turning-search-20260913/audit/terminal_tracking_followup/`
의 `diagnose.py` 및 `diagnosis.json`에 별도 보존했다. 저장된 원본은 변경하지 않았다.

추종 gate는 프로젝트 Stage 0-E 설정을 재사용한 내부 엔지니어링 기준이다. Git 이력상
Stage 0-B의 0.10-rad 기준은 `a5d88ee`, 현재 Stage 0-E 설정은 `dd9ec20`에서 추가됐다.
사용자가 그 숫자를 직접 지정했다는 대화 근거는 이 조사에서 확인하지 않았다.
커밋 작성자 이름만으로 수치 결정 주체를 추정하지 않는다. LightNav 공식 성공 판정이나
최종 navigation 목표 성공으로 취급하지 않는다.

yaw RMS는 실제 XY에서 가장 가까운 **자기 방법의 기준 경로 선분**을 찾고, 그 선분의
저장 waypoint yaw를 짧은 각도 방향으로 보간한 다음 실제 yaw와의 wrapped 차이를
제곱 평균하고 제곱근을 취한다. 모든 1/60초 물리 sample과 초기 sample을 포함한다.
경로 선분의 기울기나 회전 속도 오차가 아니며, waypoint와의 시간 정렬도 없다.
각 반복 RMS의 최댓값이 0.10 rad 미만이어야 gate를 통과한다. 목표 주변에서 오래
방향을 틀고 있으면 그 sample들도 전체 RMS에 반영된다.

종료 제어는 별개다. 현재 follower는 끝점 거리 <=8 cm와 끝점 yaw 차이 <=0.08 rad
일 때 종료한다. 거리만 만족하면 전진 0으로 마지막 yaw를 맞춘다. 회전 중 거리가
8 cm를 넘으면 다시 접근 제어로 돌아가며, 도착 상태를 유지하는 별도 latch나
진입/이탈 문턱 분리가 없다.

| 저장 실행 시점 | 이전 objective M1에서 관측된 상태 |
|---|---|
| 6.100초, control 경계 | 끝점 거리 0.0807252 m: 아직 접근 제어 |
| 6.1667초, physics sample | 거리 0.0797196 m: 잠깐 8 cm 안, control 경계 아님 |
| 6.200초, control 경계 | 거리 0.0800006 m: 다시 문턱 밖, yaw 차이 약 27.4° |
| 8.200초 | 거리 7.29 cm, yaw 차이 약 103.1°로 terminal yaw 진입 |
| 8.700초 | 측정 위치가 10.21 cm 밖으로 나가며 target 방향 회전으로 전환 |
| 9.300초 | 거리 25.58 cm에서 전진 접근 재개 |
| 12.500초 | terminal yaw 재진입; 13.2초에 종료 |

이 첫 접근에서는 10-Hz 제어가 거리 문턱 안쪽 통과를 관측하지 못했다. 당시 yaw도
종료 허용값 밖이므로, 놓친 것은 완전한 goal 종료가 아니라 위치 도달에 따른 yaw
모드 진입 기회다. 단순히 위치 허용 오차를 늘리면 해결된다고 결론 내리지 않는다.

RAW와 M3의 terminal yaw 총 시간은 각각 0.2/0.1초였고, M1은 1.2초 외에 target
방향 제자리 회전 명령 2.0초가 추가됐다. M3는 4.0초에 끝점 yaw 차이 약 6.44°로
terminal yaw에 들어가 4.1초에 종료했다. M1/M3의 요청된 끝점 yaw 차이는 약 0.28°에
불과하다. 차이는 큰 최종 yaw 명령 차이보다 **서로 다른 경로를 실행하며 만들어진
접근 위치·방향·동역학 상태**에서 나타났다.

따라서 이 결과는 같은 controller 아래에서 경로에 따라 종료부 불안정 동작이 다르게
드러남을 보여준다. follower의 감속·정지·모드 전환과 하위 실행 응답을 함께 재검증할
필요가 있다. 마찰, 접촉, 관성, PI 중 하나를 유일 원인으로 분리한 실험은 아니다.
기존 [controller 효과 검사](CURRENT_CONTROLLER_EFFECT_CHECK.md)에서도 제자리 회전의
과도 응답과 측정 위치 이동이 남아 있었다. 이번 M3 통과로 controller 전체 검증 실패를
뒤집지 않는다. 관련 기존 단위 테스트 21개가 통과했다.

## OLD/FRESH를 함께 표시한 GUI와 objective의 목적

2026-09-13 후속 GUI는 파랑 planned OLD, 청록 saved actual OLD, 회색 full raw FRESH,
주황 historical objective 후보, 자홍 Exp02D 후보, 초록 live measured motion과 노랑 B를
함께 표시한다. 현재 실행하는 기준 경로만 굵게 그린다. 각 방법의 목적과 OLD 활성화,
FRESH 관측/readiness 및 B 전환 시각을 표시하며, 모든 표시 경로를 포함해 카메라를 잡는다.

청록 선은 원래 온라인 수집에서 OLD가 활성화된 동안의 실제 주행 **정적 문맥**이다.
초록 선은 같은 B에서 reset 후 새로 실행한 물리 주행이다. 이 두 선을 동일한 연속
온라인 실행으로 제시하지 않는다. 속도·PI·접촉 이력의 reset 조건은 유지한다.
원본 world XY/yaw를 그대로 표시하고 선의 Z만 시각화 높이에 둔다. OLD 끝점을 B에
붙이거나 FRESH를 이동·회전하여 시각적으로 연결하지 않는다. 원본 경로와 후보 및
좌표/시간/hash는 각 녹화의 `R1/transition_context.json`에 기록한다.

두 objective는 OLD를 따라오던 움직임에서 선택된 FRESH suffix로 전환할 때 생기는
불연속을 줄이면서 FRESH를 보존하려는 목적을 공유한다. OLD 계획과 실제 P/B는 고정하고
FRESH[k:]의 후보 노드만 최적화한다. 이미 지나온 실제 움직임 P→B가 연결의 기준이며,
planned OLD의 마지막 점과 FRESH의 첫 점을 무조건 겹치는 문제가 아니다.

| 항목 | 이전 objective M1 | Exp02D M3 |
|---|---|---|
| 진입점 보존 E | 보정 진입 pose를 원본 FRESH 진입 pose에 가깝게 유지 | 동일 |
| 진행 방향 D | B→보정 진입점 방향을 실제 OLD 진입 방향 P→B와 맞춤 | B→보정 lookahead 지점 방향을 P→B와 맞춤 |
| yaw 변화 Y | B→보정 진입점 yaw 변화가 직전 P→B yaw 변화와 가까워지도록 함 | 동일 |
| FRESH 상대 운동 F | 연속 waypoint 사이의 상대 이동/회전을 원본과 가깝게 유지 | 동일 |

방향 residual에는 고정된 원본 B→대상점 거리도 사용하므로 순수 각도 항만은 아니다.
Exp02D의 lookahead는 동결된 follower가 raw FRESH에서 선택한 약 0.25 m 앞의 waypoint다.
진입점은 뒤나 옆에 있어도 실제 controller는 앞쪽 지점을 보고 정상적으로 전진할 수
있다. 이전 D가 그런 진입점을 문제로 간주해 과하게 보정한 EXP02C 진단 때문에, M3는
방향을 검사하는 대상을 실제 follower의 lookahead로 바꿨다. 나머지 factor/가중치/solver는
동일하다. 자세한 수식은 [Exp02D 정의](EXP_02D_LOOKAHEAD_DIRECTION.md)에 있다.

`J_cmd`는 B에서 OLD 마지막 명령과 새 첫 명령의 차이를 측정하는 평가 지표이며,
objective에 직접 추가한 명령 오차 항은 아니다. 물리 주행의 전체 추종 PASS는 별도
후속 검사다. 이 둘을 구분해야 연결부 개선이라는 연구 질문과 GUI를 일치시킬 수 있다.
현재 objective에는 장애물 거리나 충돌 제약이 없고, 원본 FRESH 보존도 soft penalty다.
따라서 연결 방향의 개선만으로 안전 주행이나 최종 목적지 성공을 보장하지 않는다.

새 녹화는 `confirmation_00/video_gui/2026-09-13T095009.381082_0000/`, 새 영상은
`confirmation_00/movies_old_fresh_context/`에 보존했다. 기존 GUI 실행 명령을 그대로
사용하고 encoder의 `--recordings`를 위 새 녹화 경로, `--output`을 위 새 영상 경로로
지정했다. RAW/M1/M3 각각 95/265/83프레임을 저장했고 세 방법의 전체 실제 pose 배열은
기존 정량 실행과 정확히 일치했다. OLD/FRESH/실제 OLD 표시 배열과 시각도 원본과 정확히
일치함을 검사했다. 관련 GUI/실행/회전/영상 시간 테스트는 57개 통과했다. 여섯 MP4 전체를
디코딩하고 원시 GUI 및 비교 영상의 시각적 표본을 검토했다.
