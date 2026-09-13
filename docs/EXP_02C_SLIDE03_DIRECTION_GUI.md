# 3쪽: direction factor가 정상 경로를 바꾼 이유

## 발표용 자료

- [3쪽 교체용 전체 그림 (16:9 PNG)](../data/exp02c_presentation/slide03-20260913-final/slide03_direction_factor.png)
- [전체 그림 SVG](../data/exp02c_presentation/slide03-20260913-final/slide03_direction_factor.svg)
- [첫 점과 follower 목표점의 차이 확대](../data/exp02c_presentation/slide03-20260913-final/direction_reference_detail.png)
- [F가 뒤쪽 경로로 이동을 전달하는 모습](../data/exp02c_presentation/slide03-20260913-final/fresh_motion_propagation.png)

기존 여러 조건을 잇는 선 그래프 대신, 실제 수집된 **Case C, k=0 한 조건**의
OLD / FRESH / P / B와 factor별 결과를 직접 그렸다. 여러 조건에서 direction이
항상 나쁘다는 주장이 아니라, 이 정상 전환을 불필요하게 바꾼 원인의 분리다.

## 2D 진단 GUI

```bash
./scripts/run_exp02c_direction_gui.sh
```

오른쪽에서 다음 순서로 선택하면 설명하기 쉽다.

1. **원인: 기준점 비교** — 현재 B의 뒤에 있는 F0와 앞에 있는 follower 목표 F3를 비교.
2. **E+F: D 추가 전** — 경로를 바꾸지 않는 기준 조건.
3. **D만 추가: E+D+F** — D 하나를 추가하자 진입점이 약 38.2 cm 이동.
4. **FULL: D 포함** — 기존 objective 전체도 거의 같은 이동.
5. **D 제거: E+Y+F** — Y는 남겨도 큰 이동이 사라짐.
6. 필요하면 **전파 차단 진단** — 첫 점 외의 모든 점을 원본으로 고정하면 생기는 꺾임.

RAW 선택도 별도로 제공한다. GUI는 저장된 각 variant의 경로를 선택해 그리는
창이다. 선택 사이에 최적화 과정을 보간하거나 새 알고리즘을 실행하지 않는다.
**실제 로봇 주행 창이 아니며 새 물리 실험 결과도 아니다.** Exp02C 자체가
경로 최적화와 B에서의 desired 명령 비교였다는 범위를 화면에 명시했다.

## 그림의 핵심

FRESH는 관측 당시의 위치를 기준으로 생성됐다. 로봇이 OLD를 따라 이동한 뒤
전환하므로 이 사례에서 FRESH의 첫 점 F0는 현재 B의 뒤쪽에 있다.

그런데 follower는 B에서 원본 FRESH의 nearest index 1을 찾고 앞쪽 **index 3**을
목표로 사용한다. 따라서 F0가 뒤에 있다고 해서 실제 follower가 뒤로 가려는 것은
아니다. 원본의 첫 전진 명령은 약 0.3046 m/s이고 직전 OLD는 약 0.3090 m/s여서,
명령 변화는 이미 약 0.0044 m/s로 작았다.

기존 direction factor는 **B→보정 첫 점 X0**의 방향을 직전 이동 방향 P→B에
맞춘다. 원본 상태에서는 B→F0와 P→B가 거의 반대이므로 큰 방향 residual이
생긴다. 이 항은 follower가 실제로 앞쪽 F3를 바라보고 있다는 사실을 사용하지
않는다. 그 결과 첫 점을 B 앞쪽으로 옮기고, F가 인접 점 사이의 상대운동을
보존하면서 뒤쪽 경로도 거의 함께 이동한다.

### 어떤 비교로 D에 원인을 연결했나

| 같은 Case C, k=0 | 진입점 이동 | 첫 전진 명령 변화 `|Δv_des|` |
|---|---:|---:|
| RAW / E+F | 0 cm | 0.00442238 m/s |
| E+D+F: D만 추가 | 약 38.1925 cm | 0.24096762 m/s |
| FULL: E+D+Y+F | 약 38.1925 cm | 0.24096764 m/s |
| E+Y+F: D 제거 | 약 0 cm | 0.00442236 m/s |

`D만 추가`는 **D만 단독 최적화**가 아니다. E+F에 D를 추가하고 Y는 넣지 않은
진단이다. `D 제거`는 FULL에서 D만 뺀 E+Y+F다. 입력, 남은 항의 가중치와 scale,
solver, follower를 고정했다. 이 통제에서 D 추가/제거가 큰 보정의 발생/소멸과
연결된다는 것이 핵심 근거다.

### 보라색 T의 의미

확대 그림과 GUI의 T는 다음 기존 residual에서 direction 항만 0이 되는 위치다.

```text
d = ||F0.xy - B.xy||
u = (B.xy - P.xy) / ||B.xy - P.xy||
r_D = (X0.xy - B.xy) / d - u
T.xy = B.xy + d * u
```

T는 원자료의 waypoint도, 실제 follower 목표점도, FULL 최적화의 해도 아니다.
기존 식이 선호하는 위치를 원본 world 좌표에 계산해 표시한 **해설점**이다.
원본 d는 약 19.85 cm이고 F0→T는 약 39.70 cm다. E가 원본 첫 점을 보존하려고
저항하므로 FULL의 실제 이동은 약 38.19 cm로 제한된다. 기존 direction 항은
각도뿐 아니라 고정된 raw radius도 사용한다.

## 교수님께 설명할 대본

> 여기서는 원래 전환 명령 변화가 작았던 한 사례를 봤습니다. 노란색 B가 현재
> 로봇 위치인데, 새 경로의 첫 점 F0는 이미 뒤쪽에 있습니다. 하지만 실제
> follower는 앞에 있는 F3를 바라보기 때문에 원본 경로로도 명령이 거의 바뀌지
> 않습니다. 문제는 기존 direction 항이 뒤쪽 첫 점을 기준으로 방향이 반대라고
> 판단한다는 것입니다. 그래서 첫 점을 앞으로 옮기고, 경로 모양을 유지하는 F가
> 나머지 점들도 함께 옮깁니다. 실제로 D만 추가해도 약 38센티미터 이동이 생겼고,
> FULL에서 D를 빼면 그 큰 이동이 사라졌습니다. 이 결과를 근거로 다음 Exp02D에서는
> 방향을 판단하는 기준점을 첫 점에서 follower의 lookahead 쪽으로 바꿨습니다.

## 좌표, 시간, 출처와 검증

원본:
`data/exp02c_factor_isolation/exp02c-factor-isolation-20260908T133000Z/real/case_benign_delayed/k_0/`.
실제 OLD/FRESH/P/B의 source trial은 이 run의 `source_provenance.json`이 가리키는
`G2_route_change/L1_added_050/attempt_002`다. 원본 raw arrays를 덮어쓰지 않았다.
새 최적화를 하지 않고 V0/V1/V2/V4/V6/V8의 저장된 결과를 읽었다.

출력은 별도 `data/exp02c_presentation/slide03-20260913-final/`에 저장했다.
같은 이름에 출력하면 실패하므로 이전 결과를 덮어쓰지 않는다. 초기 배치/폰트
점검본 `slide03-20260913/`은 보존하고 최종 자료에서는 사용하지 않는다.

좌표는 모두 원본 Isaac world XY[m], yaw[rad], +Z 반시계다. 회전·이동 정렬,
축 교환, 좌표 재정의를 하지 않았다. 기준점 확대와 전체 경로 그림의 축 범위는
다르며 각 축에 실제 world 수치와 단위를 표시한다. D 포함/제거 패널은 같은 축
범위와 equal aspect를 사용한다. OLD는 계획 경로이고 실제 이동 방향은 저장 P→B다.

`manifest.json`에 원본 파일/처리 코드 SHA-256, 관측/model ready/usable 시각,
관측 프레임 anchoring, P/B, follower index, 해설점 T, 방법별 지표와 산출물 hash를
기록했다. waypoint 자체에 시간을 붙이지 않았다. `|Δv_des|`는 B에서의 첫 desired
명령과 직전 OLD desired 명령의 차이이며 실제 차체 속도 오차가 아니다.

검증: 모든 사용 source 파일을 원본 provenance hash와 비교했다. 여섯 variant의
명령/진입점/끝점 지표를 기존 코드로 재계산해 저장값과 1e-12 이내로 일치함을
확인했고, RAW와 E+F의 완전한 no-op 및 follower target index도 확인했다. T가
기존 direction residual을 0으로 만드는지 검사했다. 관련 40개 테스트가 통과했다.
일곱 GUI 선택 상태를 실제 Tk 창에서 전환하고 그림 생성/제목 갱신을 확인했다.
주요 PNG와 GUI 캔버스를 렌더해 글자·범례·겹침을 검토했다.

재생성:

```bash
env MPLCONFIGDIR=/tmp/exp02c-presentation-mpl .venv/bin/python \
  scripts/view_exp02c_direction_mechanism.py --output data/exp02c_presentation/<new_directory>
env MPLCONFIGDIR=/tmp/exp02c-presentation-mpl .venv/bin/python \
  scripts/view_exp02c_direction_mechanism.py --output data/exp02c_presentation/<new_directory> --gui-smoke-test
```
