# 연구 PDF 수정본, 2026-09-14

사용자가 제공한 10쪽 PDF를 기존 흰 배경, 파란 띠, 주황 하단선과 로고를 유지한
11쪽 발표 PDF로 수정했다. 원본 PDF와 실험 raw 기록은 변경하지 않았다.
PDF 안의 문장과 과거 발표 대본은 자료로 읽었으며, 실행할 지시로 취급하지 않았다.

## 결과물

- 로컬 PDF: `output/pdf/연구_수정본.pdf`
- PDF와 실제 영상 2개: `output/pdf/연구_발표자료.zip`
- 입력/출력 해시, 좌표·시각, 재실행 metadata, 배치 검증: `output/pdf/revision_manifest.json`
- 영상 안내: `output/pdf/영상_안내.txt`

생성물과 복사한 영상은 Git에서 제외했다. 버전 관리 대상은 생성 코드, 동결된
입력 해시 설정, 이 설명과 작업 로그다. 원본 recording을 커밋하지 않는다.

## 슬라이드 구성

1. 연구 질문과 제한적인 개선 범위
2. Exp02B의 명령 악화, 보정 전 OLD 진단, Exp02B-R의 일부 실행 개선 구분
3. Exp02C: direction 제거/유지 ablation과 38.2 cm 진입점 변위
4. 온라인 데이터 수집 과정과 959개 전환 / 고유 raw pair 191개
5. 원본 PDF의 12개 trajectory 패널, 사례 순서와 색상 의미 유지
6. 새로 추가한 44/04 확대: 관측 anchor 차이, 짧은 OLD/FRESH, 명령과 실제 이동
7. 같은 OLD의 평지 OFF/ON 재실행, 최대 이탈 악화와 검증 전 다음 실험 진행의 한계
8. Lookahead의 조건별 명령 변화 점수와 전체 우위 불확실성
9. S2의 실제 도달 실패와 장애물/접촉 원인 미분리
10. R1의 이전 objective 대비 국소 개선, RAW의 경계 판정과 사후 선정 한계
11. 제한적인 결론, 실행 검증 후 재수집 순서

38.2 cm는 보정이 이동시킨 경로 진입점의 거리이며 차체의 추가 주행 거리가 아니다.
44번의 원래 온라인 OLD와 평지 재실행 reference 배열은 동일하다. 실행 환경,
초기 동역학과 종료 규칙이 달라 초록 실제 궤적이 다른 점을 명시했다. 온라인
44번의 헛돎이나 장애물 접촉은 원인이 분리되지 않았으므로 단정하지 않는다.

`exp02b_failure.webm`은 제공된 로컬 경로와 저장소에서 찾지 못했다. 해당 파일을
만들거나 다른 영상으로 대체하지 않고 원본 PDF의 정지 화면과 요청된 파일명을
유지했다. 이 화면은 보정 전 OLD 자세 기록 재생이다. S2와 R1에는 실제 영상에서
추출한 화면과 상대 파일 링크를 넣었다. PDF 자체의 자동 영상 재생은 아니며,
지원하지 않는 뷰어에서는 묶음의 `media` 폴더에 있는 MP4를 직접 재생한다.

## 재생성

설정: [research_pdf_revision_20260914.json](../configs/research_pdf_revision_20260914.json)

생성 코드: [revise_research_pdf_20260914.py](../scripts/revise_research_pdf_20260914.py)

추가 Python 패키지는 `/tmp/research-pdf-deps`에만 설치했다. repository 가상환경이나
실험 코드의 dependency는 변경하지 않았다. 설치 버전은 reportlab 5.0.1,
pymupdf 1.28.2, Pillow 12.3.0, pypdf 6.18.1이다. 출력에 NanumGothic 폰트를 포함했다.

```bash
UV_CACHE_DIR=/tmp/research-uv-cache /home/gpuadmin/.local/bin/uv pip install \
  --python .venv/bin/python --target /tmp/research-pdf-deps \
  reportlab==5.0.1 pymupdf==1.28.2 Pillow==12.3.0 pypdf==6.18.1

PYTHONPATH=/tmp/research-pdf-deps MPLCONFIGDIR=/tmp/research-pdf-mpl \
  .venv/bin/python scripts/revise_research_pdf_20260914.py \
  --source '/home/gpuadmin/Downloads/연구 .pdf' \
  --ffmpeg /home/gpuadmin/.cache/uv/archive-v0/pZVcuoi5kwR7C-Cb/imageio_ffmpeg/binaries/ffmpeg-linux-x86_64-v7.0.2

pdftoppm -scale-to 1600 -png output/pdf/연구_수정본.pdf tmp/pdfs/revision/review
```

입력 PDF와 source 문서/영상/재실행 배열의 해시가 변경되면 생성기는 중단한다.
새 입력을 사용하려면 내용을 다시 검토하고 설정을 명시적으로 갱신한다. 파생
presentation만 재생성하며 raw VLA 및 이전 실험 출력 경로에는 쓰지 않는다.

## 좌표와 시각

추가 그래프는 기존 hash 검증 loader로 읽은 Isaac world XY[m] 그대로 표시한다.
source와 target frame이 같고 추가 이동·회전·축 교환·좌표 정렬·보간은 없다.
yaw는 +Z 반시계 rad, 표시 cm는 m×100이다. 기존 12개 패널과 ablation 그림은
원본 PDF의 raster를 재사용하고, 확대 44번과 OFF/ON 그래프는 같은 저장 배열로
다시 그렸다. 각 축에 world 좌표와 단위를 붙이고 equal aspect를 사용한다.

원본 관측, OLD 활성화, readiness, P, B는 simulation seconds로 구분해 manifest에
기록했다. 기존 loader가 표시를 위해 직전 저장 B를 처음에 추가한 경우도 표시한다.
새 측정이나 waypoint 시각으로 재해석하지 않는다. 평지 재실행의 별도 UTC 실행
시각과 초기화 metadata도 manifest에 보존한다. 어떤 새 SE(2) 연산이나 timing
convention도 도입하지 않았고, 새 로봇 실험·최적화·인과 분리 실험도 수행하지 않았다.

## 검증

- 원본 10쪽과 최종 11쪽을 렌더링하고 모든 쪽의 시각 배치를 검토했다.
- 720×405 pt 유지, 핵심 문구, 그래프/문단/표의 비중첩, 두 MP4 링크를 자동 확인했다.
- ZIP 무결성, 묶음 PDF와 개별 PDF 일치, 원본 PDF 해시 보존을 확인했다.
- 관련 기존 테스트 111개가 통과했다. ROS의 전역 pytest plugin에 누락 dependency가
  있어 repository 테스트에서는 `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`을 사용했다.

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 MPLCONFIGDIR=/tmp/research-pdf-test-mpl \
  .venv/bin/python -m pytest -q \
  tests/test_data02_high_motion_demo.py tests/test_data02_online_successive.py \
  tests/test_controller_effect_check.py tests/test_exp02c_direction_presentation.py \
  tests/test_exp02d_turning_search.py tests/test_exp02b_failure_demo.py \
  tests/test_trajectory_follower.py tests/test_se2.py tests/test_temporal.py \
  tests/test_video_timing.py
```
