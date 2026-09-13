# 2쪽 교체 영상: Exp02B-R의 명령 악화와 보정 후 추종 오차

## 발표에서 사용할 영상

원본 `연구 .pdf` 2쪽에서 controller 보정 후 결과를 설명할 때는, 보정 전 OLD
진단의 평균 회전 명령 0.755 / 측정 0.139 rad/s 영상보다 **Exp02B-R 재실행**을
사용한다. 사용자가 지정한 두 수치와 정확히 같은 사례·k·후보 경로를 보여준다.

출력 폴더: `data/exp02b_presentation/slide02-20260913/movies/`

- [두 사례 연속 영상, 26초](../data/exp02b_presentation/slide02-20260913/movies/SLIDE02_B_THEN_C.mp4)
- [Case B, k=3: RAW / graph, 13초](../data/exp02b_presentation/slide02-20260913/movies/B_k3_RAW_GRAPH.mp4)
- [Case C, k=0: RAW / graph, 13초](../data/exp02b_presentation/slide02-20260913/movies/C_k0_RAW_GRAPH.mp4)
- [설명과 개별 재생 페이지](../data/exp02b_presentation/slide02-20260913/movies/index.html)
- [Case B graph 확대](../data/exp02b_presentation/slide02-20260913/movies/B_k3_graph.mp4)
- [Case C graph 확대](../data/exp02b_presentation/slide02-20260913/movies/C_k0_graph.mp4)

두 패널의 왼쪽은 원본 FRESH의 선택 suffix `raw_k`, 오른쪽은 이전 objective의
`graph`다. **두 실행 모두 전환 이후 controller 보정 ON**이다. 좌우가 controller
보정 OFF/ON 비교가 아니다. Exp02D도 이 영상에는 없다.

주 영상은 회전 사례 B로 권장한다. controller 보정 후에도 남은 회전 추종 오차가
크고, 사용자가 설명하려는 첫 명령 악화 수치도 함께 확인할 수 있다. C는 정상
전환을 불필요하게 보정한 사례이며 3쪽 Exp02C의 factor 분리 설명으로 이어진다.

## 수치의 의미

### 1. 전환 전후 첫 명령 변화

`|Δv| = |첫 FRESH desired v - 마지막 OLD desired v|`,
`|Δω| = |첫 FRESH desired ω - 마지막 OLD desired ω|`.
실제 차체의 속도 오차나 공간 오차가 아니다. 같은 B·경로·follower 상태에서
이 첫 desired command는 하위 실행 controller를 보정해도 변하지 않는다.

| 사례 | 지표 | RAW suffix | graph | graph 진입점 이동 |
|---|---|---:|---:|---:|
| B, 회전 명령 변화 큼, k=3 | 첫 `|Δω|`, rad/s | 1.4280 | 2.2092 | 8.71 cm |
| C, 원래 명령 변화 작음, k=0 | 첫 `|Δv|`, m/s | 0.0044 | 0.2410 | 38.19 cm |

두 사례 모두 graph의 인접 waypoint 상대 SE(2) 운동 차이는 수치 오차 수준이고
누적 회전 부호를 유지했다. 이는 경로의 상대 모양을 유지했다는 뜻이지, 지도상의
절대 위치나 B에서의 제어 명령이 유지되었다는 뜻은 아니다. 약 38.2 cm 이동은
**C의 결과**다. B까지 같은 양으로 움직였다고 설명하지 않는다.

### 2. 실제 실행에서 남은 오차

아래 값은 동일한 2초 창의 0.1초 제어 시각에서 계산한 비교값이다. 회전 추종
RMSE는 해당 제어 구간의 desired 회전 속도와 실제 pose 차분으로 얻은 회전
속도의 차이를 사용한다. GUI의 실행 보정 전/후 RMSE도 이 표와 같은 샘플링이다.

| 사례 / 방법 | 회전 추종 RMSE 보정 전→후, rad/s | 보정 후 공간 RMS, cm |
|---|---:|---:|
| B k=3 / RAW | 0.883069 → 0.188082 | 21.9068 |
| B k=3 / graph | **1.182479 → 0.481616** | **23.1546** |
| C k=0 / RAW | 0.028408 → 0.015519 | 0.0849 |
| C k=0 / graph | 0.005052 → 0.003696 | 5.7761 |

공간 RMS는 실제 위치와 **각 방법 자신의 후보 polyline** 사이의 최근접 거리다.
초기 B도 포함한다. 후보가 B에서 떨어져 시작하는 간격과 이후 진입 과정도 들어가므로
이를 전부 controller만의 오차로 해석하지 않는다. `tracking_metrics.json`의
1/60초 물리 샘플 RMS와 이 0.1초 비교 RMS를 혼용하지 않는다.

Case C의 graph는 실제 회전 추종 오차가 매우 작다. 이 사례를 controller 회전
추종의 큰 실패 증거로 제시하지 않는다. C가 보여주는 핵심은 **원래 작던 전진
명령 변화를 경로 보정이 크게 만들었다**는 것이다.

## 화면 읽는 방법과 재실행 범위

- 파랑: 계획 OLD. 청록: 저장된 실제 OLD 이력, 정적 배경.
- 회색: 원본 FRESH, 선택된 RAW suffix를 포함한다. 주황: graph.
- 초록: 이번 보정 controller로 새로 실행한 실제 움직임.
- 노랑: 저장된 전환점 B. 분홍: 원본 FRESH의 선택 진입점 F_k.
- 흰색: 현재 실행하는 후보의 첫 점. RAW에서는 분홍 점과 겹친다.

동일 사례의 좌우 카메라와 world XY/yaw는 같다. 주행 경로를 맞추기 위한
회전·이동·재정렬을 하지 않았다. 좌표는 Isaac world XY[m], +Z 반시계 yaw[rad]다.
선의 Z=0.55 m는 식별을 위한 표시 높이이며 경로 좌표나 물리 모델 변환이 아니다.
원본 FRESH는 관측 당시 robot frame에서 world로 변환된 저장 배열 그대로 사용한다.
관측·model ready·added-delay usable 시각은 `provenance.json`의 `source_timing`에
모두 보존하고 `context.json`에도 기록한다.

녹화는 기존 Exp02B-R 절차를 그대로 쓴다. nominal OLD 명령을 재생하고 저장 B와의
일치를 확인한 뒤 정확히 B로 reset한다. 마지막 OLD의 바퀴 목표값을 복구하고
PI 적분을 0으로 초기화한다. 첫 피드백에는 reset 직전 OLD의 마지막 측정 구간을
쓴다. 이후 같은 frozen `pi_strong`으로 2초간 물리 실행한다.

따라서 청록 OLD 이력은 **보정 전 nominal 실행**이며, 이 정적 선으로 controller
보정 후 OLD 오차를 주장하지 않는다. live 영상은 **전환 이후 보정 ON 결과**다.
연속 온라인 OLD→FRESH의 완전한 재현, 실제 하드웨어 주행, 목표 도달 성공·실패
평가도 아니다. 목표까지 실행하지 않은 2초 진단에 임의의 FAIL을 붙이지 않았다.

실시간 표의 measured 값은 현재 0.1초 제어 구간 시작부터 화면 시각까지의 pose
차분이다. t=0에서만 마지막 OLD 측정값이다. summary RMSE는 완료된 전체 0.1초
구간으로 계산한다. 첫 명령 변화량은 화면 상단에 별도로 고정 표시한다.

## 2쪽 수정 문구와 짧은 대본

권장 제목: **Controller 보정 후에도 남은 명령 악화**

본문:

> 회전 추종은 개선됐지만 오차가 남았다: Case B graph RMSE 1.182→0.482 rad/s.
> 같은 B에서 graph의 첫 회전 명령 변화는 RAW보다 컸다: 1.428→2.209 rad/s.
> 정상 사례에서도 전진 명령 변화가 증가했다: 0.0044→0.2410 m/s.

영상과 함께 읽는 대본:

> 이 영상은 controller를 보정한 뒤, 같은 위치에서 RAW 경로와 이전 objective가
> 만든 경로를 다시 실행한 결과입니다. 왼쪽이 RAW, 오른쪽이 이전 objective이고,
> 둘 다 동일한 보정 controller를 사용했습니다. 회전 사례에서는 보정 덕분에 명령과
> 실제 회전의 오차가 1.182에서 0.482로 줄었습니다. 하지만 완전히 없어진 것은
> 아닙니다. 별도로 경로가 바뀌는 순간의 명령을 보면, 이전 objective가 RAW보다
> 더 큰 회전 명령 변화를 만들었습니다. 다음 정상 사례에서는 경로 모양을 유지하면서
> 위치가 약 38센티미터 바뀌어, 원래 작던 전진 명령 변화도 커졌습니다. 그래서
> controller의 추종 문제와 경로 보정의 문제를 따로 봐야 한다고 판단했고,
> 다음 Exp02C에서 objective의 어느 항이 큰 보정을 만드는지 분리해 확인했습니다.

마지막 문장의 범위는 `objective만이 유일한 원인`이 아니다. controller 보정이
첫 desired command를 바꾸지 않는다는 통제 결과와, 아직 남은 실제 추종 오차를
함께 보고하는 것이다.

## 원자료와 재현

원래 후보 및 명령 결과:
`data/exp02b/exp02b-controller-aware-20260906T150400Z`.
보정 controller 재평가:
`data/exp02b_calibrated_reeval/exp02b-r-20260908T054233Z`.

정량 비교 원문은 [Exp02B-R 보고서](EXP_02B_CALIBRATED_REEVALUATION.md)와
해당 run의 `historical_metric_snapshot.json`,
`summary/historical_vs_calibrated.csv`, `branch_results/`다. 원자료를 수정하거나
후보를 다시 최적화하지 않았다. 녹화는 별도 파생 디렉터리에 저장한다.

```bash
./scripts/isaac/run_exp02b_calibrated_reeval_gui.sh \
  data/exp02b_calibrated_reeval/exp02b-r-20260908T054233Z \
  --case case_high_delta_omega --k 3 --method graph --no-hold \
  --record-video data/exp02b_presentation/<new_run>/recordings/B_k3_graph
# 같은 방식으로 B k3 raw_k, C k0 raw_k, C k0 graph를 각각 새 디렉터리에 녹화.

.venv/bin/python scripts/encode_exp02b_presentation_videos.py \
  --recordings data/exp02b_presentation/<new_run>/recordings \
  --output data/exp02b_presentation/<new_run>/movies \
  --ffmpeg /path/to/ffmpeg
```

파생 녹화별로 입력 파일·코드 hash, source timing, 좌표·카메라, 원본 window PNG,
프레임별 physics index/pose/time/hash, 명령과 측정값, 재계산한 비교 RMSE를 기록한다.
20 simulation frames/s로 실제 렌더를 캡처하며 4배 느린 5 fps 원본 시퀀스를
30 fps MP4로 인코딩한다. 정지 화면을 앞 2초·뒤 3초 추가한다. 보간 pose는 없다.
두 동기화된 별도 실행을 좌우에 배치한다. 원본 frame은 고치지 않는다.

최종 제공본은 `recordings_final/`이다. `recordings/`는 정보 패널이 숨겨진 초기
캡처, `recordings_v2/`는 하단 범례가 잘린 배치 점검본으로 보존하고 발표에 쓰지 않는다.

검증 결과: 요청된 네 실행 모두 OLD replay gate와 첫 desired command invariant를
통과했다. 각 121개 실제 pose가 저장 primary와 원소별 정확히 같고, 41개 캡처
frame의 시간·pose·hash를 검증했다. 각 실행에서 재계산한 제어 시각 RMSE는 기존
CSV와 1e-12 이내로 일치했다. 7개 MP4를 끝까지 디코딩하고 두 비교 영상의 시작·
중간·끝 frame을 추출했다. 최종 GUI와 비교 영상의 가독성·범례·사례 식별을 확인했다.
관련 테스트 47개와 기존 27 branch strict validation도 통과했다.

앱 내 브라우저의 로컬 file URL 자동 열기는 브라우저 보안 정책으로 거절됐다.
우회하지 않았으며, MP4와 HTML 파일은 완성된 로컬 산출물로 제공한다. 브라우저에서
직접 재생을 확인했다고 주장하지 않는다. PDF/PPTX 원본은 수정하지 않았다.
