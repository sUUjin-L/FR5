#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
scenario_32_35.py
=================
화면 인식으로 안전을 확인하면서 FR5 가 HMI 버튼을 순서대로 누르는 시나리오 러너.
2026-08-28 실장비 절차 기반 시나리오(25스텝 / 7블록)를 담는다.

    파일명이 32_35 인 것은 초기 실험(매뉴얼 ㉜~㉟) 흔적이다. scenario_monitor 가
    이 이름으로 import 하고 있어 그대로 둔다. 내용은 데모 시나리오 전체다.

[시나리오 — Loading On으로 시작, Loading Off로 종료]
    ① Loading On      1~3
    ② Position 설정   4~5
    ③ Gas 차단        6~9
    ④ Heater Off      10~13  (외부 표시기 전류 0 A 확인 포함)
    ⑤ Un-Loading On   14~20  (20 C 냉각·Auto Mode 포함)
    ⑥ Vent On         21~23
    ⑦ Loading Off     24~25

    각 블록은 'Main 진입 -> 서브화면 동작 -> 검증 -> Exit 복귀' 꼴이다.
    STEPS 표를 보면 스텝별 화면·타겟·게이트가 한눈에 들어온다.

[검증이 두 겹인 이유]
    화면 판별(screen_id_signature)은 "지금 무슨 화면인가"만 답한다.
    그런데 데모의 절반은 화면이 안 바뀌는 동작이다. Position 을 누르든
    Auto Mode 를 누르든 화면 이름은 그대로다. 눌렸는지 아닌지를 화면 이름으로는
    구분할 수 없다.

    그래서 검증 레이어를 하나 더 둔다. 버튼 색이다.
        1층  화면 게이트  이 동작을 해도 되는 화면인가          screen_id_signature
        2층  색 게이트    눌러서 상태가 실제로 바뀌었는가        color_gate

    색 게이트가 실패하면 **직전 스텝을 다시 누른다**(재실행). 화면 판별 실패와
    달리 '아직 안 눌렸다'는 뜻이지 위험 신호가 아니기 때문이다.
    Gate E(언로딩 완료)와 Gate H/I(Loading On/Off)는 재실행하지 않는다.
    다시 누르면 같은 장비 동작을 두 번 시작/종료할 수 있다.

[계층]
    지각   fr5_controller_260630_impl (origin/26.08.02, 메뉴 12 경로)
             카메라 -> 왜곡보정 -> AprilTag pose -> YOLO(screen) -> quad 워핑 -> OCR
             -> LayoutLM 라벨 분류
    판별   screen_id_signature.SignatureIdentifier
             앵커 42개. ANCHOR_TEXT_FIX / ANCHOR_DROP 반영분.
    검증   color_gate  (색 측정 자체는 impl._analyze_crop_color 에 위임)
    구동   converter.normalized_to_robot_apriltag -> _try_press_x
             예전 고정식(컨트롤러 11/12번과 같은 식) + 프로필 offsets

    screen_gate.py 를 쓰지 않고 게이팅을 여기에 얇게 다시 둔 이유:
    screen_gate 는 판별 로직을 자체 복사본으로 갖고 있어 앵커 보정이 반영되지 않는다.
    판별 소스를 screen_id_signature 하나로 유지하는 쪽이 안전하다.
    color_gate 에 색 측정 코드를 복사해 넣지 않은 것도 같은 이유다.

[가장 중요한 규칙]
    기권(ABSTAIN)은 통과가 아니다.
    화면을 확신하지 못하면 로봇을 움직이지 않고 시나리오를 중단한다.
    로봇이 좌표를 눌러버리는 시스템에서 "아마 맞을 것"으로 다음 단계를 여는 것이
    가장 위험하다.

[로봇은 '누르기 직전'에만 연결한다 — 지연 연결]
    FR5Controller.__init__ 은 첫 줄에서 Robot.RPC(ip) 로 로봇에 접속한다.
    예전에는 LivePerception 이 이걸 그대로 호출해서, 로봇이 없는 PC 에서는
    카메라를 켜보기도 전에 접속 단계에서 죽었다.

    지금은 scenario_monitor 와 같은 방식을 쓴다. build_vision_controller() 가
    FR5Controller.vision_only()로 원본 컨트롤러의 비전 상태만 초기화한다.
    카메라 -> YOLO -> OCR -> 화면 판별 -> 타겟 검색까지는 로봇 없이 전부 돌아가고,
    실제로 팔을 움직이는 press() 시점에 _ensure_robot() 이 처음 접속한다.

    덕분에 모델 파일이 없다거나 카메라 인덱스가 틀린 것을,
    로봇에 붙고 서보를 켜기 전에 먼저 걸러낼 수 있다.

[AprilTag 는 누르기 직전에만 필수]
    태그가 없으면 픽셀을 로봇 좌표로 바꿀 수 없으니 누를 수는 없다.
    하지만 화면 판별과 타겟 매칭에는 태그가 필요 없다.
    그래서 observe() 는 태그가 없어도 관측을 성공으로 돌려주고(pose_ok=False),
    press() 에서 태그가 없으면 그때 NO_POSE 로 중단한다.
    태그가 안 보이는 환경에서도 OCR·게이팅 디버깅은 계속할 수 있다.

[타겟 매칭은 완전일치다]
    fr5_controller 의 find_button_match_in_results 는 퍼지 매칭을 하지 않는다.
    정규화(소문자 + '_','-' -> 공백)한 뒤 문자열이 정확히 같아야 한다.
    OCR 이 오독하면 못 찾는다. 이때 우회하지 않고 [매칭 실패]를 출력하고 멈춘다.
    어느 판독이 깨졌는지 드러나야 OCR/앵커 쪽을 고칠 수 있기 때문이다.

    2026-08-03 로그 기준으로 아래 세 개는 아직 실패가 예상된다.
        Gas Main Close  OCR "Gas taain"
        Heater          OCR "eater"      (라벨도 'exit' 으로 잘못 붙어 있음)
        Un-Loading On   OCR "n-Loading Or"

    Position은 전체 detector가 "Eoaition"/조각을 내거나 polygon을 전혀 만들지
    못하는 원인을 별도 처리한다. 전체 OCR의 exact+공간 검증은 기존대로 우선하고,
    그것이 없을 때만 고정 내부 band가 실제 파란 배경인지 확인한다. 그 뒤 blue-white
    대비를 높인 context/tight ROI가 모두 detector→recognizer에서 exact Position에
    합의할 때만 전용 타겟으로 쓴다. 좌측 메뉴의 Position 오인식은 공간상 거부한다.

[구동은 사람이 승인한다 — 기본값]
    화면도 맞고 버튼도 찾았다고 바로 누르지 않는다.
    카메라 창에 상태 패널이 뜨고, 거기서 [g] 를 눌러야 그때 로봇이 움직인다.

    준비가 안 된 상태에서 [g] 를 누르면 무시하고 왜 안 되는지만 알려준다.
    화면이 다르거나, 버튼이 안 보이거나, AprilTag 가 없으면 승인되지 않는다.
    오조작으로 엉뚱한 좌표를 누르는 게 이 시스템에서 제일 위험하기 때문이다.

    승인을 기다리는 동안에도 계속 다시 관측하고 다시 매칭한다.
    [g] 를 누르기까지 10초가 걸렸다면 그 사이 카메라가 흔들렸을 수 있다.
    승인 시점이 아니라 '누르기 직전 프레임'의 좌표로 누른다.

    --auto 를 주면 예전처럼 준비되는 즉시 누른다. 창은 그대로 뜬다.

[키]
    g  구동 시작       s  이 스텝 건너뛰기
    p  일시정지(--auto 에서)   c  화면 저장      q / ESC  시나리오 중단

[사용법]
    python scenario_32_35.py --check      # 로그만으로 점검. 카메라·로봇 불필요
    python scenario_32_35.py --dry-run    # 로그 재생 모의 실행. 카메라·로봇 불필요
    python scenario_32_35.py --no-robot   # 실카메라 + 실비전, 누름만 모의. 로봇 불필요
    python scenario_32_35.py              # 실장비. [g] 승인 필요
    python scenario_32_35.py --auto       # 실장비. 승인 없이 자동
    python scenario_32_35.py --from 4 --to 6

    --check    타겟 문자열이 애초에 매칭 가능한지 (OCR 오독 확인)
    --dry-run  게이팅 흐름 자체가 성립하는지
    --no-robot 이 PC 의 실제 카메라·조명·각도에서 판별과 매칭이 되는지
               <- 다른 PC 로 옮겼을 때 제일 먼저 돌려볼 것
    --no-view  창 없이 터미널로만 (SSH 등). 승인은 g + Enter
"""

from __future__ import annotations

import argparse
import os
import sys
import time
from dataclasses import dataclass, field
from datetime import datetime
from typing import Any, Callable, Dict, List, Optional, Sequence, Tuple

# LayoutLM tokenizer를 사용한 뒤 V4L2 조절용 subprocess가 fork될 때 발생하는
# HuggingFace 경고와 잠재적 교착을 막는다. 사용자가 명시한 값은 덮어쓰지 않는다.
os.environ.setdefault("TOKENIZERS_PARALLELISM", "false")

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)

import screen_id_signature as SIG
import color_gate as CG


# ----------------------------------------------------------------------------
# 설정
# ----------------------------------------------------------------------------

DEFAULT_LOG = SIG.DEFAULT_CSV

# 타겟을 못 찾았을 때 몇 초까지 재시도할지.
# 실시간 OCR 은 프레임마다 흔들리므로 한 프레임 오독으로 시나리오를 죽이지 않는다.
# 0 으로 두면 즉시 실패한다.
MATCH_TIMEOUT_SEC = 5.0
MATCH_POLL_SEC = 0.4

# 화면 판별 관련
REQUIRE_TIMEOUT_SEC = 10.0      # 스텝 진입 시 기대 화면을 기다리는 시간
TRANSITION_TIMEOUT_SEC = 15.0   # 클릭 후 화면 전환을 기다리는 시간
HOLD_CONFIRM_SEC = 3.0          # 화면 유지형 스텝에서 유지 확인에 쓰는 시간
STABLE_FRAMES = 2               # 같은 답이 연속 몇 번 나와야 인정할지
POLL_INTERVAL_SEC = 0.5

# 실행 첫 스텝의 화면이 다르면 로봇으로 되돌리지 않는다. 사람이 HMI를 보고
# 안전한 시작 화면으로 복귀할 시간을 준다. 현재 카메라 갱신이 4초라 Main을
# 연속 2프레임 확인하는 시간까지 포함해 일반 10초보다 넉넉해야 한다.
STARTUP_REQUIRE_TIMEOUT_SEC = 60.0

# Target Offset 저장 키. 누를 좌표를 예전 고정식(컨트롤러 11/12번과 같은 식)으로
# 만들므로 그 식의 손 보정값 `offsets`(2026-08-31 X+80 Y+25 Z+3)를 같이 쓴다.
TARGET_OFFSET_PROFILE_KEY = "offsets"
# 태그 기준 좌표(tag_registration.py 등록 파일이 있을 때)는 팁이 화면 표면에 닿는 점을
# 바로 준다. 예전 식용 손 보정(`offsets`, X+40 Y+25 Z+3)을 더하면 안 되므로 키를 나눈다.
TAG_CHAIN_OFFSET_PROFILE_KEY = "tag_chain_offsets"

# 시험 간격(mm): 계산된 화면 표면에서 이만큼 앞에 팁을 세운다. Y/Z는 계산값 그대로다.
# 0 이나 None 이면 표면까지 실제로 간다(접촉).
#
# **절대 X 가 아니라 표면과의 거리다.** 카메라를 옮기거나 태그를 다시 등록해서 표면 X 가
# 달라져도 이 값을 고칠 필요가 없고, "화면에서 몇 mm 앞"이 사람이 실제로 판단하는 값이다.
# 예전에는 절대값(`PRESS_X_TEST_MAX_MM`)이었고, 그게 아래 두 사고를 한 번씩 냈다:
#   - 2026-09-18 화면 흠집 뒤 200으로 내렸는데, 2026-09-21 첫 실로봇 실행에서 스텝 1의
#     접근 이동(200 - X_APPROACH_GAP = 175)이 `MoveL error=38`로 실패했다. 알람·E-STOP·
#     안전정지는 정상이고 서보도 ON이었다. 원인은 역기구학이다 — `inverse_kin: (38, None)`,
#     `inverse_kin_has_solution(ref=actual_joint): False`. 팁을 화면 쪽(RX78/RY-89/RZ98)으로
#     세운 채로는 X 175~200mm 가 베이스에 너무 가까워 풀리는 자세가 없다.
#   - 같은 절대값이 표면 X 를 따라가지 못해, 카메라를 옮길 때마다 사람이 다시 계산해야 했다.
# 2026-09-21 실측 화면 표면 X 467.4~472.1(유리가 태그 면보다 6mm 튀어나옴)에서 20이면
# 누름 450 / 접근 425 다. 실제로 누르려면 0 으로 두거나 `--press-standoff 0`.
# impl PRESS_X_LIMIT_MM(현재 800) 상한은 이것과 무관하게 항상 따로 적용된다.
PRESS_X_TEST_STANDOFF_MM: Optional[float] = 20.0

# 접근 이동이 도달할 수 있는 최소 X(mm). 시험 간격을 크게 주면 목표가 베이스 쪽으로
# 끌려와 위 역기구학 실패가 난다. 2026-09-21 실측: 접근 175 실패, 대기 자세 250 도달.
PRESS_X_TEST_MIN_APPROACH_MM = 250.0

# LayoutLM 버튼 모델. 2026-09-18부터 기본 진입점도 320 모델을 쓴다(impl 기본값은
# 224라 컨트롤러는 그대로다). processor 크기는 impl 모델 로딩이 다시 확인하고,
# 다르면 구동 근거로 쓰지 않는다.
LAYOUTLM_MODEL_DIR = "/home/jetnano/jetson_ocr/layoutlm_320"
LAYOUTLM_MODEL_IMAGE_SIZE = 320
LAYOUTLM_MODEL_MAX_LENGTH = 320

# 창을 띄웠을 때의 폴링 간격. 0.5초면 영상이 뚝뚝 끊겨서 십자가 어디 찍혔는지
# 눈으로 따라가기 어렵다. 어차피 OCR 이 병목이라 더 줄여도 의미는 없다.
VIEW_POLL_INTERVAL_SEC = 0.10
# 선로딩은 모델 로드와 더미 첫 추론까지 포함한다. 2026-09-17 Orin(스왑 3.7GB
# 사용 중)에서 로드 36~57초, 첫 추론 58~74초였다. 첫 추론을 스텝 안에서 하면
# 스텝 1 타겟 검색 30초가 먼저 끝나 label 없이 NO_MATCH가 났다.
LAYOUT_PRELOAD_TIMEOUT_SEC = 300.0
# 첫 추론 더미 입력. 실제 Main 화면 crop(약 1732x1209)과 OCR 약 100건으로
# max_length 토큰까지 채워 실제 프레임과 같은 입력 크기로 CUDA 초기화를 끝낸다.
LAYOUT_WARMUP_CROP_SIZE = (1732, 1209)
LAYOUT_WARMUP_RECORDS = 110
LAYOUT_RESULT_WAIT_SEC = 5.0
LAYOUT_OPTIONAL_APPROVAL_WAIT_SEC = 10.0
LAYOUT_WORKER_STOP_TIMEOUT_SEC = 60.0

# 누른 뒤 HMI 가 반응할 시간
POST_PRESS_SETTLE_SEC = 1.0

# --no-robot 에서는 사람이 로봇 대신 손으로 누른다. 반응 시간을 넉넉히 준다.
HAND_REQUIRE_TIMEOUT_SEC = 60.0
HAND_TRANSITION_TIMEOUT_SEC = 60.0
HAND_HOLD_CONFIRM_SEC = 6.0

# 수동 승인 모드의 타겟 재시도 시간. 기본 5초는 [g] 를 누르려고 손이 가는 사이에
# 끝나 버린다. 어차피 승인 단계에서 매 프레임 다시 매칭하므로 길어도 손해가 없다.
HAND_MATCH_TIMEOUT_SEC = 30.0

# 색 게이트가 실패했을 때 같은 스텝을 몇 번까지 다시 누를지.
# 무한 재시도는 안 된다. 눌러도 상태가 안 바뀌는 상황(밸브 인터록 등)에서
# 로봇이 같은 자리를 계속 두드리게 된다.
MAX_STEP_RETRY = 2

# 언로딩 완료(Gate E)를 기다리는 시간. 장비 동작 시간이라 넉넉해야 한다.
UNLOADING_WAIT_SEC = 180.0


# ----------------------------------------------------------------------------
# 스텝 정의
# ----------------------------------------------------------------------------

@dataclass
class Step:
    """시나리오 한 단계.

    require      : 이 화면이 아니면 누르지 않는다.
    target       : find_button_match_in_results 에 넘길 질의.
                   라벨로 찾게 하려면 라벨명을 그대로 준다(예: menu_function).
                   LayoutLM 라벨 매칭이 텍스트 매칭보다 우선순위가 높다.
    expect_after : 누른 뒤 되어야 할 화면. require 와 같으면 '화면 유지' 스텝.
    gate         : 누르고 화면 확인까지 끝난 뒤 붙는 색 검증. 없으면 None.
    block        : 매뉴얼 p.8 기준 블록 이름. 발표용 표시에 쓴다.
    note         : 로그에 남길 설명.
    unverified   : 타겟 문자열이 학습 로그로 확인되지 않았다는 표시.
                   실장비에서 OCR 실측으로 고쳐야 하는 자리다. --check 가 따로 센다.
    observation_id: 값이 있으면 로봇 누름이 아니라 외부 표시기 카메라 관측이다.
    retry_transition_timeout: 화면 이동용 버튼을 눌렀는데 여전히 require 화면이면
                   제한 횟수 안에서 스텝을 다시 준비한다. 상태 변경 버튼에는 쓰지 않는다.
    """
    no: int
    require: str
    target: str
    expect_after: str
    note: str = ""
    gate: Optional[CG.Gate] = None
    block: str = ""
    unverified: bool = False
    observation_id: str = ""
    retry_transition_timeout: bool = False

    @property
    def holds(self) -> bool:
        """화면이 바뀌지 않는 상태 토글 스텝인가."""
        return self.require == self.expect_after

    @property
    def is_observation(self) -> bool:
        return bool(self.observation_id)


# ----------------------------------------------------------------------------
# 게이트 — 자동 A/C/D/E + 수동 B/F/G/H/I
# ----------------------------------------------------------------------------
#
# 색으로 잡을 수 있는 것은 4개(A/C/D/E)뿐이다.
# 나머지 5개(B/F/G/H/I)는 화면 신호가 없거나 완료 신호를 실측하지 못해 수동으로 본다.
# 자동 통과시키면 "안 눌렸는데 눌렸다고 보고"가 되므로, 사람이 눈으로 보고
# [g] 로 승인하게 한다. 발표 데모에서는 오히려 설명하기 좋은 지점이다.

GATE_A = CG.Gate(
    "A", CG.GATE_COLOR, screen="Main", target="Position", spec=CG.ACTIVE_BLUE,
    note="Position 이 선택되면 버튼이 파랗게 활성화된다 (로그 실측 cyan sat=134). "
         "단 타겟 문자열 자체가 OCR 로 안 잡혀서 지금은 게이트가 열리지 않는다")

GATE_B = CG.Gate(
    "B", CG.GATE_MANUAL,
    prompt="Ar / Gas Main 이 실제로 닫혔습니까?",
    note="닫아도 버튼 색이 변하지 않는다. 색·OCR 어느 쪽으로도 신호가 없어 미해결")

GATE_C = CG.Gate(
    "C", CG.GATE_VANISH, screen="Heater", target="heater_status",
    spec=CG.HEATER_ORANGE,
    note="켜는 게 아니라 끄는 절차다. 검출이 아니라 '주황색 소멸'이 성공 신호")

GATE_D = CG.Gate(
    "D", CG.GATE_COLOR, screen="Menu_mode", target="auto_mode_select",
    spec=CG.ACTIVE_BLUE,
    note="Auto Mode 가 선택되면 파랗게 활성화된다")

GATE_E = CG.Gate(
    "E", CG.GATE_WAIT, screen="Main", target="loadlock_sample",
    spec=CG.ACTIVE_BLUE, timeout_sec=UNLOADING_WAIT_SEC,
    note="언로딩이 끝나면 Loadlock 쪽 Sample 이 파래진다. 재시도가 아니라 '완료 대기'")

GATE_F = CG.Gate(
    "F", CG.GATE_MANUAL,
    prompt="Un-Loading 이 시작되었습니까?",
    note="누른 직후에는 색 변화가 없다. 완료 신호는 Gate E 가 따로 받는다")

GATE_G = CG.Gate(
    "G", CG.GATE_MANUAL,
    prompt="Vent 가 열렸습니까?",
    note="눌러도 버튼 색이 변하지 않는다. 미해결")

GATE_H = CG.Gate(
    "H", CG.GATE_MANUAL,
    prompt="Loading On 단계가 완료되었습니까?",
    note="Loading On의 자동 완료 신호를 실측하지 못했다. "
         "사람이 완료를 보고 승인하며 중복 시작 방지를 위해 재실행하지 않는다",
    retry_step=False)

GATE_I = CG.Gate(
    "I", CG.GATE_MANUAL,
    prompt="Loading Off 단계가 완료되었습니까?",
    note="Loading Off의 자동 완료 신호를 실측하지 못했다. "
         "사람이 완료를 보고 승인하며 중복 시작 방지를 위해 재실행하지 않는다",
    retry_step=False)


# ----------------------------------------------------------------------------
# 외부 히터 표시기 관측 계약
# ----------------------------------------------------------------------------

HEATER_OBSERVATIONS: Dict[str, Dict[str, Any]] = {
    "heater_process_ready": {
        "timeout_sec": 60.0,
        "stable_frames": 2,
        "requirements": {
            "current_max_a": 0.1,
            "output_active": False,
        },
    },
    "heater_current_zero": {
        "timeout_sec": 60.0,
        "stable_frames": 2,
        "requirements": {
            "current_max_a": 0.1,
            "output_active": False,
        },
    },
    "heater_cooldown_ready": {
        "timeout_sec": 180.0,
        "stable_frames": 2,
        "requirements": {
            "sp_target_c": 20.0,
            "sp_tolerance_c": 1.0,
            "pv_sp_max_delta_c": 3.0,
            "current_max_a": 0.1,
            "output_active": False,
        },
    },
}


# ----------------------------------------------------------------------------
# 25스텝: 22회 물리 누름 + 3회 카메라 관측
# ----------------------------------------------------------------------------
#
# unverified=True 인 타겟은 20260727 로그에 그 문자열이 없다.
# find_button_match_in_results 는 퍼지 매칭을 하지 않으므로(정규화 후 완전일치),
# 실장비 OCR 이 뱉는 문자열을 보고 여기를 고쳐야 한다. --check 가 목록으로 알려준다.

STEPS: List[Step] = [
    # ── ⓪ Loading On ────────────────────────────────────────
    Step(1, "Main", "menu_function", "Function",
         "좌측 메뉴바 Function을 열어 Loading On을 준비", block="0.Load On",
         retry_transition_timeout=True),
    Step(2, "Function", "Loading On", "Function",
         "로딩 시작. 로그에서 text/label 모두 Loading On으로 완전일치 "
         "@(0.309,0.233)", gate=GATE_H, block="0.Load On"),
    Step(3, "Function", "Exit", "Main",
         "Loading On 완료 확인 후 Function 닫기", block="0.Load On",
         retry_transition_timeout=True),

    Step(4, "Main", "heater:process_ready", "Main",
         "공정 종료를 시작하기 전에 꺼진 히터의 외부 Heater Current가 "
         "0.0 A인지 카메라로 확인",
         block="1.Position", observation_id="heater_process_ready"),

    # ── ① Position 설정 ────────────────────────────────────────────────────
    Step(5,  "Main",      "Position",       "Main",
         "Position 설정. Throttle Valve 패널 안의 파란 버튼 @(0.235,0.212). "
         "전체 OCR exact+공간 검증을 우선하고, 누락 시 파란 배경 gate와 blue-white "
         "강화 context+tight detector의 exact+공간 합의를 모두 요구한다",
         gate=GATE_A, block="1.Position", unverified=True),

    # ── ② Gas 차단 ────────────────────────────────────────────────────────
    Step(6,  "Main",      "Ar",             "Gas.M",
         "하단 가스 바에서 Gas.M 팝업 열기 (close 아니라 진입 버튼)", block="2.Gas",
         retry_transition_timeout=True),
    Step(7,  "Gas.M",     "Ar Close",       "Gas.M",
         "Ar 밸브 닫기", block="2.Gas"),
    Step(8,  "Gas.M",     "Gas Main Close", "Gas.M",
         "가스 메인 밸브 닫기. 로그에서는 'Gas taain'@(0.911,0.130) 으로 오독됨 "
         "— Ar Open/Close 와 같은 열 배치라 x=0.91 이 Close 쪽",
         gate=GATE_B, block="2.Gas", unverified=True),
    Step(9,  "Gas.M",     "Exit",           "Main",
         "Gas.M 팝업 닫기", block="2.Gas", retry_transition_timeout=True),

    # ── ③ Heater Off ──────────────────────────────────────────────────────
    Step(10, "Main",      "Heater",         "Heater",
         "Heater 팝업 열기. @(0.338,0.550) 의 **세로로 눕힌 탭**이라 OCR 이 못 읽는다 "
         "— 12회 실측 완전일치 0/12, 'eater' 로 잘리고 label 도 exit 로 오분류. "
         "재시도로 해결되지 않는다",
         block="3.Heater", unverified=True),
    Step(11, "Heater",    "Heater Off",     "Heater",
         "히터 끄기", gate=GATE_C, block="3.Heater"),
    Step(12, "Heater", "heater:current_zero", "Heater",
         "Heater Off 뒤 외부 Heater Current가 0.0 A인지 카메라로 확인",
         block="3.Heater", observation_id="heater_current_zero"),
    Step(13, "Heater",    "Exit",           "Main",
         "Heater 팝업 닫기", block="3.Heater", retry_transition_timeout=True),

    # ── ④ Un-Loading On (Auto Mode 포함) ──────────────────────────────────
    Step(14, "Main", "heater:cooldown_ready", "Main",
         "사람이 SP=20 C/SET을 마친 뒤 PV≈SP, 0 A가 될 때까지 카메라로 확인",
         block="4.Unload", observation_id="heater_cooldown_ready"),
    Step(15, "Main",      "Mode",           "Menu_mode",
         "Mode Select 팝업 열기", block="4.Unload",
         retry_transition_timeout=True),
    Step(16, "Menu_mode", "Auto Mode",      "Menu_mode",
         "자동 모드로 전환", gate=GATE_D, block="4.Unload"),
    Step(17, "Menu_mode", "Exit",           "Main",
         "Mode Select 팝업 닫기", block="4.Unload",
         retry_transition_timeout=True),
    Step(18, "Main",      "menu_function",  "Function",
         "좌측 메뉴바 Function (라벨로 지정)", block="4.Unload",
         retry_transition_timeout=True),
    Step(19, "Function",  "Un-Loading On",  "Function",
         "언로딩 시작 @(0.697,0.235). 글자가 버튼 테두리에 잘려 항상 'Un-Loading' / "
         "'In-Loading Or' 로 끊긴다 — 12회 실측 완전일치 0/12. "
         "한 줄은 좌->우로 [Loading On 0.309 | Loading Off 0.472 | "
         "Un-Loading On 0.697 | Un-Loading Off 0.856] 인데 앞 두 개만 라벨이 붙었다. "
         "'Loading On' 을 타겟으로 쓰면 정반대 동작(로딩)을 누른다",
         gate=GATE_F, block="4.Unload", unverified=True),
    Step(20, "Function",  "Exit",           "Main",
         "Function 닫고 Main 에서 언로딩 완료를 기다린다",
         gate=GATE_E, block="4.Unload", retry_transition_timeout=True),

    # ── ⑤ Vent On ─────────────────────────────────────────────────────────
    Step(21, "Main",      "menu_function",  "Function",
         "Function 다시 열기", block="5.Vent", retry_transition_timeout=True),
    Step(22, "Function",  "LC_Vent_On",     "Function",
         "Vent 열기. 'Vent On' 이 LC/PC 두 개라 텍스트로는 못 고른다 — 라벨로 지정. "
         "PC 쪽을 열어야 한다면 PC_Vent_On 으로 바꿀 것",
         gate=GATE_G, block="5.Vent"),
    Step(23, "Function",  "Exit",           "Main",
         "Function 닫기", block="5.Vent", retry_transition_timeout=True),

    # ── ⑥ Loading Off ───────────────────────────────────────
    Step(24, "Main", "menu_function", "Function",
         "좌측 메뉴바 Function을 열어 Loading Off를 준비", block="6.Load Off",
         retry_transition_timeout=True),
    Step(25, "Function", "Loading Off", "Function",
         "로딩 종료. 로그에서 text/label 모두 Loading Off로 완전일치 "
         "@(0.472,0.234). 이 단계를 시나리오의 마지막 동작으로 둔다",
         gate=GATE_I, block="6.Load Off"),
]


# ----------------------------------------------------------------------------
# 중단 사유
# ----------------------------------------------------------------------------

class ScenarioAbort(Exception):
    """시나리오를 더 진행하면 안 되는 상황. 로봇은 움직이지 않은 상태여야 한다."""

    def __init__(self, kind: str, message: str, step: Optional[Step] = None):
        super().__init__(message)
        # ABSTAIN   화면을 확신하지 못함
        # MISMATCH  다른 화면
        # TIMEOUT   화면이 넘어가지 않음
        # NO_MATCH  타겟 문자열 완전일치 실패 (OCR 오독)
        # NO_POSE   AprilTag 0/1 bundle 미검출 — 픽셀을 로봇 좌표로 못 바꿈
        # NO_ROBOT  --no-robot 인데 로봇이 필요한 지점에 도달
        # PRESS_X_LIMIT 누름 X가 impl PRESS_X_LIMIT_MM(화면 쪽 상한)을 넘음
        # PRESS_X_UNREACHABLE 접근 X가 베이스에 너무 가까워 역기구학 해가 없음
        # MOVE_FAIL 로봇 이동/누름 실패
        # VISION    지각 파이프라인 자체 실패
        self.kind = kind
        self.message = message
        self.step = step

    def __str__(self) -> str:
        head = f"[스텝 {self.step.no}] " if self.step else ""
        return f"{head}{self.kind} — {self.message}"


# ----------------------------------------------------------------------------
# 관측 1건
# ----------------------------------------------------------------------------

@dataclass
class Observation:
    """한 프레임에서 뽑아낸 것 전부.

    dets      : 화면 판별용. 워핑된 화면 크롭 기준 정규화 좌표.
    buttons   : 클릭 타겟 검색용. LayoutLM 라벨이 붙은 검출 리스트
                (없으면 raw OCR 결과). 픽셀 좌표는 '워핑된 크롭' 기준이다.
    ok        : AprilTag + YOLO screen + 기하 정보가 모두 갖춰졌는가.
    reason    : ok=False 일 때의 사유.
    """
    dets: List[SIG.Detection] = field(default_factory=list)
    buttons: List[Dict[str, Any]] = field(default_factory=list)
    ok: bool = True
    reason: str = ""
    context: Dict[str, Any] = field(default_factory=dict)


# ----------------------------------------------------------------------------
# 화면 판별
# ----------------------------------------------------------------------------

class ScreenJudge:
    """screen_id_signature 를 감싼 판별기. 연속 확인(stable) 로직을 얹는다."""

    def __init__(self, log_path: str = DEFAULT_LOG, stable_frames: int = STABLE_FRAMES):
        self.idf = SIG.SignatureIdentifier.from_log(log_path)
        self.stable_frames = stable_frames
        self.last: Optional[SIG.Result] = None

    def judge(self, obs: Observation) -> SIG.Result:
        if not obs.ok:
            r = SIG.Result(None, False, obs.reason, {}, [], len(obs.dets))
        else:
            r = self.idf.identify(obs.dets)
        self.last = r
        return r

    def wait_for(self, screen: str, observe: Callable[[], Observation],
                 timeout_sec: float,
                 poll_sec: float = POLL_INTERVAL_SEC,
                 tick: Optional[Callable[[Observation, SIG.Result], None]] = None
                 ) -> Tuple[bool, SIG.Result]:
        """그 화면이 stable_frames 회 연속 확인될 때까지 기다린다.

        반환 (성공여부, 마지막 판정). 실패해도 예외를 던지지 않는다 —
        '안 넘어감'과 '엉뚱한 화면'을 호출부가 구분해야 하기 때문이다.

        tick(obs, verdict)
            매 폴링마다 불린다. 화면 표시를 갱신하고 키를 확인하는 자리다.
            여기서 예외(ScenarioAbort)를 던지면 대기가 즉시 끝난다.
            이게 없으면 최대 60초 대기 동안 창이 얼어붙고 [q] 도 안 먹는다.
        """
        deadline = time.time() + timeout_sec
        streak = 0
        last_frame_token = None
        last = SIG.Result(None, False, "관측 없음", {}, [], 0)
        while True:
            obs = observe()
            last = self.judge(obs)
            if tick is not None:
                tick(obs, last)
            frame_token = obs.context.get("result_frame_id")
            new_evidence = frame_token is None or frame_token != last_frame_token
            if new_evidence:
                if last.decided and last.screen == screen:
                    streak += 1
                    if streak >= self.stable_frames:
                        return True, last
                else:
                    streak = 0
                last_frame_token = frame_token
            if time.time() >= deadline:
                return False, last
            time.sleep(poll_sec)


# ----------------------------------------------------------------------------
# 타겟 검색 (완전일치)
# ----------------------------------------------------------------------------

def normalize_query(value: Any) -> str:
    """fr5_controller._normalize_button_query 와 같은 규칙."""
    text = str(value or "").strip().lower()
    text = text.replace("_", " ").replace("-", " ")
    return " ".join(text.split())


def field_matches(value: Any, query: str) -> bool:
    """fr5_controller._button_field_matches 와 같은 규칙. 퍼지 없음."""
    v = str(value or "").strip()
    if not v:
        return False
    return v.lower() == query.strip().lower() or normalize_query(v) == normalize_query(query)


def find_target(buttons: Sequence[Dict[str, Any]], query: str) -> Optional[Dict[str, Any]]:
    """라벨 우선, 그 다음 텍스트로 완전일치 검색.

    fr5_controller.find_button_match_in_results 와 같은 우선순위를 쓴다.
    실장비에서는 그쪽 함수를 그대로 호출하고, 이 함수는 --check/--dry-run 용이다.
    """
    best = None
    best_rank = -1
    for r in buttons:
        rank = -1
        if field_matches(r.get("label"), query):
            rank = 2
        elif field_matches(r.get("text"), query):
            rank = 1
        if rank > best_rank:
            best, best_rank = r, rank
    return best if best_rank > 0 else None


def describe_near(buttons: Sequence[Dict[str, Any]], query: str, k: int = 3) -> str:
    """매칭 실패했을 때 '가까웠던 것'을 보여준다. 어느 판독이 깨졌는지 알려주는 용도."""
    import difflib
    q = normalize_query(query)
    scored = []
    for r in buttons:
        t = normalize_query(r.get("text"))
        if not t:
            continue
        ratio = difflib.SequenceMatcher(None, q, t).ratio()
        scored.append((ratio, r))
    scored.sort(key=lambda kv: -kv[0])
    out = []
    for ratio, r in scored[:k]:
        lbl = r.get("label") or "-"
        conf = r.get("ocr_confidence", r.get("confidence", 0.0)) or 0.0
        out.append(f'"{r.get("text")}" (유사도 {ratio:.2f}, ocr={float(conf):.3f}, label={lbl})')
    return " | ".join(out) if out else "검출 없음"


def _position_overlay_status(
        position_target: Any,
        position_target_meta: Any) -> Tuple[str, str]:
    """Position 전용 검출 결과를 표시용 ASCII 한 줄로만 요약한다.

    이 함수는 공통 추론 결과를 읽기만 한다. 반환 문자열과 색 이름은 화면 표시에만
    사용하며 화면 판별, 버튼 검색, 승인 또는 로봇 좌표에는 다시 넣지 않는다.
    """
    if isinstance(position_target, dict):
        source = str(position_target.get("evidence_source", ""))
        mode = ("P2" if source == "ocr_position_roi_detector_consensus"
                else "RAW")
        try:
            confidence = float(position_target.get(
                "ocr_confidence",
                position_target.get("confidence", 0.0)) or 0.0)
            confidence_text = "{:.3f}".format(confidence)
        except (TypeError, ValueError):
            confidence_text = "?"
        return "{} conf={}".format(mode, confidence_text), "ok"

    meta = (position_target_meta
            if isinstance(position_target_meta, dict) else {})
    status = str(meta.get("status") or "position_not_run")
    if status.startswith("position_"):
        status = status[len("position_"):]

    blue_fraction = (meta.get("blue_background") or {}).get(
        "blue_fraction")
    try:
        blue_text = "{:.3f}".format(float(blue_fraction))
    except (TypeError, ValueError):
        blue_text = "-"

    exact_counts: Dict[str, str] = {"context": "-", "tight": "-"}
    for item in meta.get("passes") or []:
        if not isinstance(item, dict):
            continue
        name = str(item.get("name") or "").strip().lower()
        if name not in exact_counts:
            continue
        try:
            exact_counts[name] = str(int(item.get("exact_count", 0)))
        except (TypeError, ValueError):
            exact_counts[name] = "?"

    return (
        "FAIL {} B={} C={} T={}".format(
            status.upper(),
            blue_text,
            exact_counts["context"],
            exact_counts["tight"],
        ),
        "warn",
    )


# ----------------------------------------------------------------------------
# 로봇 없이 시각 파이프라인만 세우기
# ----------------------------------------------------------------------------

def build_vision_controller(IMPL):
    """원본 FR5Controller의 vision_only()로 비전 상태만 초기화한다.

    __init__ 은 첫 줄에서 Robot.RPC(ip) 로 로봇에 접속한다. 카메라·YOLO·OCR·
    화면 판별은 로봇과 아무 상관이 없으므로, 그 접속을 여기서 미뤄둔다.
    실제로 팔을 움직일 때 attach_robot() 이 뒤늦게 채워 넣는다.

    현재 구현은 초기화 내용을 원본의 _init_vision_runtime() 한 곳에 둔다. 아래 수동
    속성 초기화는 vision_only()가 없는 이전 override 파일의 호환 폴백일 뿐이다.

    scenario_monitor.py 도 이 함수를 쓴다. 판별 소스를 하나로 유지하는 것과
    같은 이유로, 컨트롤러를 세우는 방법도 한 곳에만 둔다.
    """
    factory = getattr(IMPL.FR5Controller, "vision_only", None)
    if callable(factory):
        return factory()

    # 이전 구현 파일을 override로 쓰는 ROS 경로를 위한 호환 폴백이다. 현재
    # fr5_controller_260630_impl은 위 vision_only()를 사용하므로 이 복제 경로를
    # 타지 않는다.
    ctrl = IMPL.FR5Controller.__new__(IMPL.FR5Controller)

    ctrl.robot = None
    ctrl.force_monitor = None

    try:
        ctrl.converter = IMPL.CoordConverter(
            img_width=getattr(IMPL, "LIVE_CAMERA_WIDTH", 3840),
            img_height=getattr(IMPL, "LIVE_CAMERA_HEIGHT", 2160),
            load_calibration=False,
        )
    except TypeError:
        # img_width/img_height 인자가 없는 이전 override의 호환 폴백.
        # image1_clean.jpg는 사용하지 않는다.
        ctrl.converter = IMPL.CoordConverter(os.path.join(HERE, "image1.jpg"))
    ctrl.camera_calibration = IMPL.CameraCalibration(IMPL.CAMERA_CALIBRATION_PATH)
    ctrl.apriltag_tracker = IMPL.AprilTagPoseTracker(ctrl.camera_calibration)

    ctrl.screen_detector = None
    ctrl.screen_detector_error = False
    ctrl.ocr_engine = None
    ctrl.ocr_engine_error = False
    ctrl.layout_button_classifier = None
    ctrl.layout_button_error = False
    ctrl.layout_value_ocr_engine = None
    ctrl.layout_value_classifier = None
    ctrl.layout_value_error = False
    ctrl.layout_value_regions = {}
    ctrl.live_models_warmed_up = False
    return ctrl


def attach_robot(ctrl, IMPL, ip: Optional[str] = None) -> None:
    """비전 전용 원본 컨트롤러에 로봇 연결을 뒤늦게 채운다.

    원본 connect_robot()이 RPC 접속 / Mode(0) / ResetAllError / ForceMonitor를
    실행한다. 실패하면 예외를 그대로 올린다 —
    누르기 직전이므로 조용히 넘어가면 안 된다.
    """
    ip = ip or IMPL.ROBOT_IP
    connect = getattr(ctrl, "connect_robot", None)
    if callable(connect):
        connect(ip)
        return

    # vision_only/connect_robot API가 없는 이전 override 구현의 호환 폴백.
    print(f"\n  [FR5] 연결 중: {ip}")
    ctrl.robot = IMPL.Robot.RPC(ip)
    ctrl.robot.Mode(0)
    ctrl.robot.ResetAllError()
    time.sleep(0.5)
    ctrl.force_monitor = IMPL.ForceMonitor(ctrl.robot)
    print("  [FR5] 연결 완료 (서보는 메뉴에서 ON/OFF)\n")
    try:
        ctrl._print_robot_state_summary("연결 직후")
    except Exception:
        pass


# ----------------------------------------------------------------------------
# 지각 : 실장비
# ----------------------------------------------------------------------------

class LivePerception:
    """FR5Controller 를 감싸 '한 프레임 관측'만 뽑아낸다.

    메뉴 12(drive_loop_live_apriltag_text)의 프레임 루프에서 시각 처리 부분만
    떼어낸 것이다. 창 표시·드래그바·CSV 기록은 시나리오 실행에 필요 없으므로 뺐다.

    무거운 의존성(cv2 / fairino / TensorRT)은 여기서만 import 한다.
    --check / --dry-run 은 이 클래스를 건드리지 않으므로 개발 PC 에서도 돌아간다.

    로봇 연결은 여기서 하지 않는다. press() 가 처음 불릴 때 접속한다.
    allow_robot=False 면 아예 접속하지 않고 누름을 [모의] 로 출력한다.
    """

    def __init__(self, camera_index: int = 0, use_quad: bool = True,
                 use_undistort: bool = True, allow_robot: bool = True,
                 full_frame_ocr: Optional[bool] = None):
        import cv2  # noqa: F401  (지연 import)
        import fr5_controller_260630_impl as IMPL

        self.cv2 = cv2
        self.IMPL = IMPL
        self.allow_robot = bool(allow_robot)
        self.robot_ready = False
        self.initial_target_offsets = (0.0, 0.0, 0.0)
        self._target_offset_provider = None

        # 모델 로딩 전에 LayoutLM 모델 선택을 impl 상수에 반영한다.
        IMPL.LAYOUTLM_DIR = LAYOUTLM_MODEL_DIR
        IMPL.LAYOUTLM_IMAGE_SIZE = LAYOUTLM_MODEL_IMAGE_SIZE
        IMPL.LAYOUTLM_MAX_LENGTH = LAYOUTLM_MODEL_MAX_LENGTH
        print("  [LayoutLM] 모델: {} (image={}x{}, max_tokens={})".format(
            IMPL.LAYOUTLM_DIR, IMPL.LAYOUTLM_IMAGE_SIZE,
            IMPL.LAYOUTLM_IMAGE_SIZE, IMPL.LAYOUTLM_MAX_LENGTH))

        # 실장비는 태그 등록이 필수다. 파일이 없거나 무효하면 예전 식으로 넘어가지
        # 않고 시작을 막는다. 로봇 없는 점검에서만 예전 식 좌표를 표시한다.
        import tag_registration as TR
        self.tag_registration, tag_error = TR.load_or_error(IMPL, TR.REGISTRATION_PATH)
        if self.tag_registration is not None:
            self.target_offset_key = TAG_CHAIN_OFFSET_PROFILE_KEY
            print("  [좌표] 태그 기준 — " + self.tag_registration.summary())
        elif os.path.exists(TR.REGISTRATION_PATH):
            raise RuntimeError("태그 등록 파일을 사용할 수 없습니다: " + tag_error)
        elif self.allow_robot:
            raise RuntimeError(tag_error + " — 거리가 바뀐 뒤에는 태그 등록을 다시 완료해야 "
                               "합니다. 로봇을 연결하지 않습니다.")
        else:
            self.target_offset_key = TARGET_OFFSET_PROFILE_KEY
            print("  [좌표 경고] 태그 등록 없음 — 예전 식 좌표는 로봇 없는 점검에만 표시합니다. "
                  "실장비 실행 전 python3 tag_registration.py 로 등록하세요.")

        # 로봇에 접속하지 않고 시각 파이프라인만 세운다
        self.ctrl = build_vision_controller(IMPL)
        self.use_quad = bool(use_quad and IMPL.ENABLE_SCREEN_QUAD_WARP)
        self.use_undistort = use_undistort
        # 4K 전체 프레임 검출을 타겟 완전일치 보조 증거로 돌릴지 정한다.
        # 화면 판별·LayoutLM은 켜든 끄든 crop 경로 결과만 쓴다. 모델 로딩 전에
        # impl 상수에도 반영해야 warmup이 전체 프레임 shape를 미리 태운다.
        self.full_frame_ocr = bool(
            getattr(IMPL, "OCR_FULL_FRAME_DETECT", False)
            if full_frame_ocr is None else full_frame_ocr)
        IMPL.OCR_FULL_FRAME_DETECT = self.full_frame_ocr
        self.screen_conf_threshold = float(IMPL.YOLO_CONF_THRESHOLD)
        self.screen_quad_params: Dict[str, Any] = {}

        print("  모델 로딩 중...")
        self.screen_detector, self.ocr_engine = self.ctrl._load_live_yolo_ocr_models()
        if self.screen_detector is None or self.ocr_engine is None:
            raise RuntimeError("YOLO screen 검출기와 OCR 엔진이 모두 필요합니다.")
        self._print_ocr_input_mode()

        self.cap = self.ctrl._open_live_camera(camera_index)
        if self.cap is None:
            raise RuntimeError(f"카메라 {camera_index} 를 열 수 없습니다.")

        # 12번 튜닝 화면에서 저장한 노출/화이트밸런스/quad 값을 시나리오도
        # 똑같이 써야 한다. 이 경로가 빠져 있으면 같은 카메라가 실행 진입점에 따라
        # 서로 다른 영상을 만들고 화면 시그니처가 흔들린다.
        self._configure_runtime_profile(camera_index)
        self._last_target_offsets = tuple(self.initial_target_offsets)
        # AprilTag 는 '누르기'에만 필수다. 여기서는 실패해도 세우지 않는다.
        self.apriltag_ready = bool(self.ctrl.apriltag_tracker.load())
        if not self.apriltag_ready:
            print("  [경고] AprilTag 파라미터 로드 실패 — 화면 판별과 타겟 매칭은 계속됩니다.")
            print("         누르기 단계에서 NO_POSE 로 중단됩니다.")

        self.frame_interval = float(getattr(IMPL, "MODE12_FRAME_INTERVAL_SEC", 3.0))
        self._next_observe_at = 0.0
        print(f"  [카메라] 시각 갱신 주기: {self.frame_interval:.1f}초")

        self._last_screens: List[Dict[str, Any]] = []
        self._last_quad = None

        from live_camera_pipeline import (
            LatestFrameCapture, LatestInferenceWorker, LatestJobWorker)
        self.layout_worker = LatestJobWorker(
            self._classify_layout_job,
            name="scenario-layoutlm",
        ).start()
        try:
            self._preload_layout_model()
        except Exception:
            # __init__ 도중 실패하면 호출자가 close()를 부를 객체를 받지 못한다.
            # 카메라와 non-daemon CUDA worker를 여기서 직접 정리한다.
            self.close()
            raise
        self.capture_stream = LatestFrameCapture(
            self.cap, name="scenario-camera").start()
        self.inference_worker = LatestInferenceWorker(
            self.capture_stream,
            self._infer_frame_snapshot,
            interval_sec=self.frame_interval,
            name="scenario-vision",
        ).start()
        self._layout_submitted_frame_id = 0
        self._approval_frame_id = 0
        self._capture_fps_warned = False
        self._last_timing_logged_frame_id = 0
        self._preview_times: List[float] = []
        print("  [카메라] 캡처 스레드와 인식 worker 분리 완료 "
              "(latest frame 1장만 유지)")

    def _configure_runtime_profile(self, camera_index: int) -> None:
        """저장된 12번 비전/카메라 튜닝을 현재 시나리오 카메라에 적용한다.

        설정 적용 실패는 관측 단계가 화면을 다시 검증하도록 경고만 낸다. 프로필을
        적용했다고 화면 판별을 통과시키지는 않으며, ABSTAIN 규칙은 그대로다.
        """
        ctrl, IMPL = self.ctrl, self.IMPL
        profile = ctrl._load_apriltag_text_tuning_profile()

        saved_camera = profile.get("camera_index") if isinstance(profile, dict) else None
        same_camera = True
        if saved_camera is not None:
            try:
                same_camera = int(saved_camera) == int(camera_index)
            except (TypeError, ValueError):
                same_camera = False
        if not same_camera:
            print("  [시나리오 튜닝 경고] 저장 카메라와 현재 카메라가 달라 "
                  "저장값 대신 안전한 기본값을 씁니다.")

        offset_key = getattr(self, "target_offset_key", TARGET_OFFSET_PROFILE_KEY)
        saved_offsets = (ctrl._profile_dict(profile, offset_key)
                         if same_camera else {})
        offset_limit = float(getattr(
            IMPL, "APRILTAG_TARGET_OFFSET_LIMIT_MM", 100))
        self.initial_target_offsets = tuple(
            float(ctrl._clamp_profile_value(
                saved_offsets.get(axis, 0.0), 0.0,
                -offset_limit, offset_limit, as_int=True))
            for axis in ("x", "y", "z")
        )

        screen = ctrl._profile_dict(profile, "screen_tune") if same_camera else {}
        software = (ctrl._profile_dict(profile, "software_brightness")
                    if same_camera else {})
        ctrl.camera_brightness_alpha = float(ctrl._clamp_profile_value(
            software.get("alpha", IMPL.CAMERA_BRIGHTNESS_ALPHA_DEFAULT),
            IMPL.CAMERA_BRIGHTNESS_ALPHA_DEFAULT, 0.1, 3.0))
        ctrl.camera_brightness_beta = float(ctrl._clamp_profile_value(
            software.get("beta", IMPL.CAMERA_BRIGHTNESS_BETA_DEFAULT),
            IMPL.CAMERA_BRIGHTNESS_BETA_DEFAULT, -255.0, 255.0))
        ctrl.camera_warmth_percent = float(ctrl._clamp_profile_value(
            software.get(
                "warmth_percent", IMPL.CAMERA_WARMTH_PERCENT_DEFAULT),
            IMPL.CAMERA_WARMTH_PERCENT_DEFAULT, 0.0, 200.0))
        self.screen_conf_threshold = float(ctrl._clamp_profile_value(
            screen.get("yolo_conf", IMPL.YOLO_CONF_THRESHOLD),
            IMPL.YOLO_CONF_THRESHOLD, 0.01, 1.0))
        bbox_pad_pct = ctrl._clamp_profile_value(
            screen.get("bbox_pad_pct", int(round(IMPL.SCREEN_QUAD_BBOX_PAD * 100))),
            int(round(IMPL.SCREEN_QUAD_BBOX_PAD * 100)), 0, 30, as_int=True)
        min_area_pct = ctrl._clamp_profile_value(
            screen.get("min_area_pct", int(round(IMPL.SCREEN_QUAD_MIN_AREA_RATIO * 100))),
            int(round(IMPL.SCREEN_QUAD_MIN_AREA_RATIO * 100)), 1, 95, as_int=True)
        canny_lo = ctrl._clamp_profile_value(
            screen.get("canny_lo", IMPL.SCREEN_QUAD_CANNY_LO),
            IMPL.SCREEN_QUAD_CANNY_LO, 0, 255, as_int=True)
        canny_hi = ctrl._clamp_profile_value(
            screen.get("canny_hi", IMPL.SCREEN_QUAD_CANNY_HI),
            IMPL.SCREEN_QUAD_CANNY_HI, 0, 255, as_int=True)
        if canny_hi <= canny_lo:
            canny_hi = min(255, canny_lo + 1)
        approx_eps_x1000 = ctrl._clamp_profile_value(
            screen.get("approx_eps_x1000", int(round(IMPL.SCREEN_QUAD_APPROX_EPS * 1000))),
            int(round(IMPL.SCREEN_QUAD_APPROX_EPS * 1000)), 1, 120, as_int=True)
        dilate_kernel = ctrl._odd_kernel(ctrl._clamp_profile_value(
            screen.get("dilate_kernel", IMPL.SCREEN_QUAD_DILATE_KERNEL),
            IMPL.SCREEN_QUAD_DILATE_KERNEL, 1, 31, as_int=True))
        close_kernel = ctrl._odd_kernel(ctrl._clamp_profile_value(
            screen.get("close_kernel", IMPL.SCREEN_QUAD_CLOSE_KERNEL),
            IMPL.SCREEN_QUAD_CLOSE_KERNEL, 1, 31, as_int=True))
        self.screen_quad_params = {
            "bbox_pad": float(bbox_pad_pct) / 100.0,
            "min_area_ratio": float(min_area_pct) / 100.0,
            "canny_lo": int(canny_lo),
            "canny_hi": int(canny_hi),
            "approx_eps": float(approx_eps_x1000) / 1000.0,
            "dilate_kernel": int(dilate_kernel),
            "close_kernel": int(close_kernel),
        }
        print("  [시나리오 튜닝] YOLO conf={:.2f}, quad pad={}%, area>={}%, "
              "canny={}/{}, eps={:.3f}".format(
                  self.screen_conf_threshold, bbox_pad_pct, min_area_pct,
                  canny_lo, canny_hi, self.screen_quad_params["approx_eps"]))
        saved_v4l2 = (ctrl._profile_dict(profile, "v4l2_controls")
                      if same_camera else {})
        self.effective_camera_tuning = {
            "saved_at": profile.get("saved_at", "") if same_camera else "",
            "software_alpha": ctrl.camera_brightness_alpha,
            "software_beta": ctrl.camera_brightness_beta,
            "software_warmth_percent": ctrl.camera_warmth_percent,
            "v4l2_controls": dict(saved_v4l2),
            "screen_tune": dict(screen),
        }
        print("  [시나리오 카메라 동기화] saved_at={} alpha={:.2f} "
              "beta={:.1f} warmth={:.0f}%"
              .format(
                  self.effective_camera_tuning["saved_at"] or "(default)",
                  ctrl.camera_brightness_alpha,
                  ctrl.camera_brightness_beta,
                  ctrl.camera_warmth_percent))
        if saved_v4l2:
            print("  [시나리오 카메라 동기화] V4L2 저장값: {}".format(
                ", ".join("{}={}".format(name, value)
                          for name, value in sorted(saved_v4l2.items()))))

        # V4L2는 ctrl._open_live_camera()가 공통 함수로 이미 적용했다.
        # 여기서는 좌표/화면 인식 프로필만 읽어 두 실행 경로가 갈라지지 않게 한다.

    def _print_ocr_input_mode(self) -> None:
        """이번 실행이 어느 detector 입력을 쓰는지 시작할 때 한 번 알린다."""
        IMPL = self.IMPL
        limit = IMPL.FR5Controller._det_engine_max_input_hw(self.ocr_engine)
        limit_text = ("{}x{}".format(limit[1], limit[0]) if limit
                      else "알 수 없음")
        engine_name = os.path.basename(str(IMPL.OCR_DET_TRT_PATH))
        print("  [OCR 입력] det 엔진 {} (최대 {}); 화면 판별·LayoutLM·버튼 검색 = "
              "화면 crop 긴 변 {}px".format(
                  engine_name, limit_text, IMPL.OCR_DET_TARGET_SIZE))
        if not self.full_frame_ocr:
            print("  [OCR 입력] 전체 프레임 보조 검출 꺼짐 (--no-full-frame-ocr)")
            return
        requested = int(IMPL.OCR_DET_FULL_FRAME_TARGET_SIZE)
        print("  [OCR 입력] 전체 프레임 보조 검출 켜짐 (긴 변 {}px) — crop 경로에 "
              "타겟 완전일치가 없을 때만 씁니다".format(requested))
        if limit is not None:
            frame_shape = (IMPL.LIVE_CAMERA_HEIGHT, IMPL.LIVE_CAMERA_WIDTH)
            effective, _limit = IMPL.FR5Controller._full_frame_det_target_size(
                self.ocr_engine, frame_shape, requested)
            if int(effective) != requested:
                print("  [OCR 입력 경고] det 엔진 profile이 작아 실제 긴 변이 "
                      "{}px로 내려갑니다. 더 큰 입력으로 엔진을 다시 빌드해야 "
                      "합니다.".format(int(effective)))

    # --- 로봇 지연 연결 -------------------------------------------------------
    def _ensure_robot(self) -> None:
        """첫 [g] 승인 뒤 로봇 연결·서보 ON·이동 가능 상태를 확인한다."""
        if self.robot_ready:
            return
        if not self.allow_robot:
            raise ScenarioAbort(
                "NO_ROBOT", "--no-robot 모드에서는 로봇을 연결하지 않습니다.")
        attach_robot(self.ctrl, self.IMPL)
        try:
            print("  [시나리오 서보] 첫 구동 승인 후 RobotEnable(1)을 요청합니다.")
            self.ctrl.servo_on()
            self.ctrl._ensure_robot_can_move()
        except Exception as exc:
            try:
                self.ctrl.servo_off()
            except Exception as off_exc:
                print("  [시나리오 서보 경고] 실패 후 OFF 확인도 실패했습니다: {}"
                      .format(off_exc))
            raise ScenarioAbort(
                "SERVO_ON_FAIL",
                "서보 ON 또는 이동 가능 상태 확인에 실패했습니다. "
                "OFF를 요청했으며 로봇을 움직이지 않습니다: {}".format(exc))
        self.robot_ready = True
        print("  [시나리오 서보] rbtEnableState=1 확인 완료")

    def set_target_offset_provider(self, provider) -> None:
        """실행 중 X/Y/Z 타겟 보정값을 읽을 함수를 연결한다."""
        self._target_offset_provider = provider if callable(provider) else None

    def target_offsets(self) -> Tuple[float, float, float]:
        """현재 보정값을 컨트롤러의 허용 범위 안으로 제한한다."""
        provider = self._target_offset_provider
        if provider is None:
            return 0.0, 0.0, 0.0
        try:
            raw = tuple(provider())
            if len(raw) != 3:
                raise ValueError("X/Y/Z 3개가 필요합니다")
            limit = float(getattr(
                self.IMPL, "APRILTAG_TARGET_OFFSET_LIMIT_MM", 100))
            values = tuple(float(value) for value in raw)
        except (TypeError, ValueError, RuntimeError):
            return 0.0, 0.0, 0.0
        result = tuple(max(-limit, min(limit, value)) for value in values)
        self._last_target_offsets = result
        return result

    # --- 정리 ---------------------------------------------------------------
    def _save_target_offsets_if_changed(self) -> None:
        """Target Offset 드래그바를 바꿨으면 공통 프로필의 offsets만 갱신한다.

        카메라/화면 튜닝값은 건드리지 않는다(시나리오는 읽기만 한다).
        """
        initial = tuple(float(v) for v in getattr(
            self, "initial_target_offsets", (0.0, 0.0, 0.0)))
        latest = tuple(float(v) for v in getattr(
            self, "_last_target_offsets", initial))
        if latest == initial:
            return
        import json
        from datetime import datetime

        path = self.IMPL.APRILTAG_TEXT_TUNING_PROFILE_PATH
        try:
            with open(path, "r", encoding="utf-8") as fp:
                profile = json.load(fp)
        except (OSError, ValueError):
            profile = {}
        if not isinstance(profile, dict):
            profile = {}
        offset_key = getattr(self, "target_offset_key", TARGET_OFFSET_PROFILE_KEY)
        profile[offset_key] = {
            "x": latest[0], "y": latest[1], "z": latest[2]}
        profile[offset_key + "_saved_at"] = (
            datetime.now().isoformat(timespec="seconds"))
        with open(path, "w", encoding="utf-8") as fp:
            json.dump(profile, fp, ensure_ascii=False, indent=2, sort_keys=True)
            fp.write("\n")
        print("  [좌표 보정] Target Offset X={:+.0f} Y={:+.0f} Z={:+.0f}mm를 저장했습니다 "
              "(이전 X={:+.0f} Y={:+.0f} Z={:+.0f}).".format(*(latest + initial)))

    def close(self) -> None:
        # force_monitor 는 press_x 가 스스로 start/stop 하므로 여기서 건드리지 않는다
        try:
            self._save_target_offsets_if_changed()
        except Exception as exc:
            print("  [정리 경고] Target Offset 저장 실패: {}".format(exc))
        try:
            if getattr(self, "capture_stream", None) is not None:
                self.capture_stream.stop(release=True, timeout_sec=10.0)
            elif self.cap is not None:
                self.cap.release()
            if getattr(self, "inference_worker", None) is not None:
                self.inference_worker.stop(timeout_sec=60.0)
            if getattr(self, "layout_worker", None) is not None:
                self.layout_worker.stop(
                    timeout_sec=LAYOUT_WORKER_STOP_TIMEOUT_SEC)
                if self.layout_worker.running:
                    print("  [정리 대기] LayoutLM worker가 {:.0f}초 안에 종료되지 "
                          "않았습니다. CUDA 안전 종료까지 기다립니다.".format(
                              LAYOUT_WORKER_STOP_TIMEOUT_SEC))
                    self.layout_worker.stop(timeout_sec=None)
        except Exception as exc:
            print("  [정리 경고] 카메라/인식 worker 종료 실패: {}".format(exc))

    def _cache_observation(self, obs: Observation) -> Observation:
        # 이전 Observation에는 4K preview가 들어 있다. 별도 캐시로 한 주기 더
        # 붙잡아 두면 최신 캡처와 인식 중 프레임 외에 과거 프레임까지 상주한다.
        # 러너가 현재 Observation을 이미 소유하므로 여기서는 캐시하지 않는다.
        return obs

    def _infer_frame_snapshot(self, snapshot):
        """공통 worker에서 한 frame_id를 시나리오 관측 결과로 변환한다."""
        ctrl = self.ctrl
        config = {
            "use_undistort": bool(
                self.use_undistort and ctrl.camera_calibration.available),
            "use_quad": bool(self.use_quad),
            "quad_params": dict(self.screen_quad_params),
            "yolo_conf": float(self.screen_conf_threshold),
            "text_recognition_enabled": True,
            "full_frame_ocr": bool(self.full_frame_ocr),
        }
        cuda_context_pushed = False
        try:
            try:
                import pycuda.autoinit
                import pycuda.driver as cuda

                pycuda.autoinit.context.push()
                cuda_context_pushed = True
            except Exception:
                pass
            result = ctrl._infer_apriltag_text_snapshot(
                snapshot, self.screen_detector, self.ocr_engine, config)
        finally:
            if cuda_context_pushed:
                try:
                    cuda.Context.pop()
                except Exception:
                    pass

        result["heater_crops"] = {}
        result["heater_fallback_crop"] = None
        result["heater"] = None
        result["heater_reason"] = ""
        result["heater_source"] = ""
        source = result.get("source_frame")
        detections = result.get("yolo_detections") or []
        geometry = result.get("screen_geometry")
        if source is not None:
            try:
                import heater_vision as HV
                decoded, crops, reason = HV.decode_yolo_controller(
                    source, detections)
                result["heater_crops"] = crops
                result["heater_reason"] = reason
                if decoded is not None:
                    result["heater"] = decoded.as_dict()
                    result["heater_source"] = "yolo_regions"
                elif geometry is not None:
                    fallback_crop = HV.rectify_controller(
                        source, geometry["processed_to_source_h"])
                    result["heater_fallback_crop"] = fallback_crop
                    fallback = HV.decode_controller(fallback_crop)
                    suffix = (
                        "homography 진단 판독 성공(판정 미사용)"
                        if fallback is not None
                        else "homography 진단 판독도 실패")
                    result["heater_reason"] = "{}; {}".format(reason, suffix)
            except Exception as exc:
                result["heater_reason"] = (
                    "외부 히터 YOLO 직접 판독 오류: {}".format(exc))

        # 시나리오 창에 필요한 screen ROI를 만든 뒤 4K 중간 프레임은 결과에서
        # 제거한다. raw/undistorted/software 프레임을 모두 보존하면 3840x2160
        # BGR 기준 결과 한 개가 최대 약 72 MiB를 계속 점유한다. 승인 좌표와 판정은
        # crop/geometry/pose를 사용하고, 최신 raw preview는 캡처 객체에서 받는다.
        preview_crop, coordinates_valid = self._screen_preview_crop(result)
        result["screen_preview_crop"] = preview_crop
        result["screen_preview_coordinates_valid"] = coordinates_valid
        result.pop("raw_frame", None)
        result.pop("undistorted_frame", None)
        result.pop("source_frame", None)
        return result

    def _classify_layout_job(self, payload):
        cuda_context_pushed = False
        try:
            try:
                import pycuda.autoinit
                import pycuda.driver as cuda

                pycuda.autoinit.context.push()
                cuda_context_pushed = True
            except Exception:
                pass
            if payload is None:
                started = time.monotonic()
                loaded = bool(self.ctrl._load_layout_button_classifier())
                load_ms = (time.monotonic() - started) * 1000.0
                if not loaded:
                    return [], {"status": "LayoutLM preload error",
                                "preloaded": False, "load_ms": load_ms}
                warm_ok, warm_meta = self._warmup_layout_model()
                return [], {
                    "status": ("LayoutLM preloaded+warm" if warm_ok
                               else "LayoutLM warmup error"),
                    "preloaded": bool(warm_ok),
                    "load_ms": load_ms,
                    "warmup": warm_meta,
                }
            crop, ocr_results = payload
            return self.ctrl._classify_layout_buttons(crop, ocr_results)
        finally:
            if cuda_context_pushed:
                try:
                    cuda.Context.pop()
                except Exception:
                    pass

    def _warmup_layout_model(self):
        """실제 크기의 더미 입력으로 LayoutLM 첫 추론을 미리 끝낸다.

        결과 라벨은 버린다. 추론이 실패하면 실제 프레임도 실패할 것이므로
        선로딩 실패로 돌려 시나리오를 시작하지 않는다(fail-closed).
        """
        import numpy as np  # 지연 import (--check/--dry-run은 numpy 없이도 돈다)

        width, height = LAYOUT_WARMUP_CROP_SIZE
        image = np.full((height, width, 3), 40, dtype=np.uint8)
        words = ("Mode", "Main", "Function", "Config", "Alarm", "Stop",
                 "Open", "Close", "Vent", "Sample", "Shut#1", "RF#1")
        columns = 10
        cell_w = width // columns
        rows = max(1, (LAYOUT_WARMUP_RECORDS + columns - 1) // columns)
        cell_h = max(20, height // rows)
        records = []
        for index in range(LAYOUT_WARMUP_RECORDS):
            x1 = (index % columns) * cell_w + 8
            y1 = (index // columns) * cell_h + 6
            x2 = min(width - 1, x1 + cell_w - 16)
            y2 = min(height - 1, y1 + max(12, cell_h // 2))
            records.append({
                "text": words[index % len(words)],
                "bbox": [x1, y1, x2, y2],
                "polygon": [[x1, y1], [x2, y1], [x2, y2], [x1, y2]],
                "confidence": 0.9,
            })
        started = time.monotonic()
        try:
            _classified, meta = self.ctrl._classify_layout_buttons(image, records)
        except Exception as exc:
            return False, {"status": "warmup exception: {}: {}".format(
                type(exc).__name__, exc)}
        meta = dict(meta or {})
        meta["elapsed_ms"] = (time.monotonic() - started) * 1000.0
        status = str(meta.get("status", ""))
        ok = status.startswith("LayoutLM button:") and "error" not in status
        return ok, meta

    def _preload_layout_model(self):
        """스텝 제한시간이 시작되기 전에 전용 CUDA worker에서 모델을 로드한다."""
        print("  [LayoutLM 버튼] 시나리오 시작 전 선로딩 중...", flush=True)
        self.layout_worker.submit(0, None)
        result = self.layout_worker.wait_for_result(
            0, timeout_sec=LAYOUT_PRELOAD_TIMEOUT_SEC)
        if result is None:
            self.layout_worker.stop(timeout_sec=None)
            raise RuntimeError(
                "LayoutLM 선로딩이 {:.0f}초 안에 끝나지 않았습니다.".format(
                    LAYOUT_PRELOAD_TIMEOUT_SEC))
        if result.error:
            raise RuntimeError("LayoutLM 선로딩 worker 오류: {}".format(
                result.error))
        _classified, meta = result.value or ([], {})
        if not bool(meta.get("preloaded")):
            raise RuntimeError(
                "LayoutLM 선로딩 실패 — 라벨 완전일치 타겟을 안전하게 찾을 수 없습니다. "
                "({})".format(meta.get("status") or meta.get("warmup") or "unknown"))
        warmup = meta.get("warmup") or {}
        print("  [LayoutLM 버튼] 선로딩 확인 완료 ({:.0f}ms: 로드 {:.0f}ms + 첫 추론 "
              "{:.0f}ms)".format(result.total_ms, float(meta.get("load_ms", 0.0)),
                                  float(warmup.get("elapsed_ms", 0.0))))

    @staticmethod
    def _screen_preview_crop(result):
        """표시 창에만 쓸 화면 영역과 좌표 유효 여부를 돌려준다.

        quad가 성공했으면 실제 OCR 입력 화면을 그대로 보여준다. quad가 실패해도
        튜닝 중 화면 검출 범위를 확인할 수 있도록 가장 높은 신뢰도의 YOLO screen
        bbox만 source frame에서 잘라 보여준다. 이 bbox crop은 표시 전용이며 OCR,
        화면 판별, 버튼 좌표 또는 로봇 구동 근거로 재사용하지 않는다.
        """
        crop = result.get("screen_image")
        if crop is not None and getattr(crop, "size", 0) > 0:
            return crop, True

        source = result.get("source_frame")
        screens = result.get("screens") or []
        if source is None or getattr(source, "size", 0) <= 0 or not screens:
            return None, False
        bbox = screens[0].get("bbox") or []
        if len(bbox) != 4:
            return None, False
        height, width = source.shape[:2]
        x1 = max(0, min(width, int(round(float(bbox[0])))))
        y1 = max(0, min(height, int(round(float(bbox[1])))))
        x2 = max(0, min(width, int(round(float(bbox[2])))))
        y2 = max(0, min(height, int(round(float(bbox[3])))))
        if x2 <= x1 or y2 <= y1:
            return None, False
        return source[y1:y2, x1:x2].copy(), False

    def _observation_from_inference(self, inference, preview_snapshot=None,
                                    require_layout=False):
        if inference is None:
            return Observation(ok=False, reason="인식 결과 대기 중",
                               context={"dots": []})
        if inference.error:
            return Observation(
                ok=False,
                reason="인식 worker 오류: {}".format(inference.error),
                context={
                    "frame_id": inference.frame_id,
                    "captured_at": inference.captured_at,
                    "dots": [],
                },
            )
        result = inference.value or {}
        frame_id = int(inference.frame_id)
        crop = result.get("screen_image")
        preview_crop = result.get("screen_preview_crop")
        preview_coordinates_valid = bool(
            result.get("screen_preview_coordinates_valid"))
        # 단위 테스트나 다른 호출자가 compact 전 결과를 넘기는 경우도 지원한다.
        if "screen_preview_crop" not in result:
            preview_crop, preview_coordinates_valid = (
                self._screen_preview_crop(result))
        ocr_results = list(result.get("ocr_results") or [])

        submitted_layout = False
        if (crop is not None and ocr_results
                and frame_id > self._layout_submitted_frame_id):
            # 모델 최초 로딩/분류 중에 새 frame_id를 계속 넣으면 완료 결과가
            # 매번 폐기되어 영원히 label='-'로 남는다. 현재 작업이 끝난 뒤에만
            # 최신 프레임 하나를 제출하고, 구동에는 여전히 같은 frame_id 결과만 쓴다.
            if self.layout_worker.submit_if_idle(
                    frame_id, (crop, ocr_results)):
                self._layout_submitted_frame_id = frame_id
                submitted_layout = True
        if submitted_layout:
            layout_result = self.layout_worker.wait_for_result(
                frame_id, timeout_sec=LAYOUT_RESULT_WAIT_SEC)
        else:
            layout_result = self.layout_worker.latest()
        layout_ready = bool(
            layout_result is not None
            and layout_result.frame_id == frame_id
            and not layout_result.error)
        classified = []
        buttons = self.IMPL.merge_ocr_layout_results(ocr_results, classified)
        position_target = result.get("position_target")
        layout_status = "pending"
        layout_ms = 0.0
        if layout_ready:
            classified, meta = layout_result.value
            buttons = self.IMPL.merge_ocr_layout_results(
                ocr_results, classified)
            layout_status = str(meta.get("status", "LayoutLM button"))
            layout_ms = float(layout_result.total_ms)
        elif layout_result is not None and layout_result.frame_id == frame_id:
            layout_status = "error: {}".format(layout_result.error)

        # 캡처는 ndarray를 수정하지 않고 매번 새 객체로 교체한다. 일반 미리보기는
        # 참조만 빌려 4K 복사를 피하고, 구동 승인 경로만 pause_and_snapshot()에서
        # 의도적으로 복사한다.
        latest = preview_snapshot or self.capture_stream.latest(copy=False)
        preview_id = latest.frame_id if latest is not None else frame_id
        preview_frame = (latest.frame if latest is not None
                         else result.get("raw_frame"))
        stale = preview_id != frame_id
        timings = dict(result.get("timings") or {})
        timings["layoutlm"] = layout_ms
        if frame_id != self._last_timing_logged_frame_id:
            from live_camera_pipeline import format_timing_line
            capture_stats_for_log = self.capture_stream.stats()
            inference_stats_for_log = self.inference_worker.stats()
            print("  [PERF] " + format_timing_line(
                frame_id,
                timings,
                preview_fps=self._preview_fps(),
                capture_fps=capture_stats_for_log.get("capture_fps", 0.0),
                inference_fps=inference_stats_for_log.get(
                    "inference_fps", 0.0),
            ))
            self._last_timing_logged_frame_id = frame_id
        display_dots = self._centers(buttons)
        if isinstance(position_target, dict):
            position_center = self.IMPL._button_result_center(position_target)
            if position_center is not None:
                display_dots.append(position_center)
        import hand_eye as HE
        pose = result.get("pose")
        if pose and pose.get("ok") and not result.get("pose_ok"):
            pose_problems = [("CALIBRATION_ASPECT_MISMATCH",
                              "카메라 해상도가 왜곡 보정 파일과 맞지 않습니다.")]
        else:
            pose_problems = HE.pose_problems(pose)
        ctx = {
            "frame": preview_frame,
            "crop": crop,
            "screen_preview_crop": preview_crop,
            "screen_preview_coordinates_valid": preview_coordinates_valid,
            "geometry": result.get("screen_geometry"),
            "ocr_input": result.get("ocr_input"),
            "heater_crop": (result.get("heater_crops") or {}).get("Heater_sus"),
            "heater_crops": result.get("heater_crops") or {},
            "heater_fallback_crop": result.get("heater_fallback_crop"),
            "heater": result.get("heater"),
            "heater_reason": result.get("heater_reason", ""),
            "heater_source": result.get("heater_source", ""),
            "yolo_detections": result.get("yolo_detections") or [],
            "captured_at": float(result.get("captured_at", inference.captured_at)),
            "pose": result.get("pose"),
            # 구동에 쓸 수 있는 포즈인지는 hand_eye.pose_problems 하나로 정한다:
            # bundle(태그 0/1 모두), 태그가 영상 끝에 잘리지 않음, 재투영 오차.
            # 구동 승인([g])·진행 창 apriltag OK/NG·누름이 모두 이 값을 본다.
            "pose_ok": not pose_problems,
            "pose_reason": pose_problems[0][1] if pose_problems else "",
            "undistort_on": bool(result.get("use_undistort")),
            "frame_id": frame_id,
            "result_frame_id": frame_id,
            "preview_frame_id": preview_id,
            "stale": stale,
            "layout_status": layout_status,
            "ocr_button_count": len(ocr_results),
            "layout_button_count": len(classified),
            "button_evidence": "ocr+layoutlm" if classified else "ocr-only",
            "position_target": position_target,
            "full_frame_ocr_results": list(
                result.get("full_frame_ocr_results") or []),
            "full_frame_ocr_meta": dict(
                (result.get("ocr_input") or {}).get("full_frame") or {}),
            "position_target_meta": result.get("position_target_meta") or {},
            "timings": timings,
            "capture_stats": self.capture_stream.stats(),
            "inference_stats": self.inference_worker.stats(),
            "dots": display_dots,
        }
        ok = bool(result.get("ok"))
        reason = str(result.get("reason") or "")
        if require_layout and not layout_ready:
            layout_status = "optional; OCR-only"
            ctx["layout_status"] = layout_status
        dets = self._to_detections(buttons, crop) if buttons else []
        return Observation(
            dets=dets,
            buttons=buttons,
            ok=ok,
            reason=reason,
            context=ctx,
        )

    def observe(self) -> Observation:
        """최신 인식 결과와 최신 raw preview를 결합해 즉시 반환한다."""
        self._preview_times.append(time.monotonic())
        self._preview_times = self._preview_times[-60:]
        capture_stats = self.capture_stream.stats()
        expected_fps = float(getattr(self.IMPL, "LIVE_CAMERA_FPS", 0.0))
        if (not self._capture_fps_warned
                and int(capture_stats.get("frame_id", 0)) >= 10
                and expected_fps > 0.0
                and float(capture_stats.get("capture_fps", 0.0))
                < expected_fps * 0.7):
            print("  [카메라 성능 경고] 요청 {:.1f} FPS, 실측 {:.2f} FPS — "
                  "노출/USB/센서 모드를 확인하세요.".format(
                      expected_fps,
                      float(capture_stats.get("capture_fps", 0.0))))
            self._capture_fps_warned = True
        inference = self.inference_worker.latest()
        obs = self._observation_from_inference(inference)
        if obs.context.get("frame") is None:
            # 첫 인식 전이나 worker 오류 중에도 카메라 원본 창은 계속 갱신한다.
            # 표시 전용 키만 채우고 구동 검증에 쓰는 preview_frame_id는 건드리지 않는다.
            latest = self.capture_stream.latest(copy=False)
            if latest is not None:
                obs.context["frame"] = latest.frame
                obs.context["raw_frame_id"] = latest.frame_id
        obs.context["preview_fps"] = self._preview_fps()
        return self._cache_observation(obs)

    def _preview_fps(self):
        if (len(self._preview_times) < 2
                or self._preview_times[-1] <= self._preview_times[0]):
            return 0.0
        return float(len(self._preview_times) - 1) / float(
            self._preview_times[-1] - self._preview_times[0])

    def approval_observation(self, timeout_sec=15.0) -> Observation:
        """[g] 직전 캡처를 고정하고 같은 frame_id의 전체 결과를 다시 만든다."""
        frozen = self.capture_stream.pause_and_snapshot(timeout_sec=2.0)
        if frozen is None:
            self.capture_stream.resume()
            return Observation(ok=False, reason="승인용 최신 프레임이 없습니다")
        requested_id = self.inference_worker.request_latest()
        if requested_id != frozen.frame_id:
            self.capture_stream.resume()
            return Observation(ok=False, reason="승인 프레임 고정에 실패했습니다")
        inference = self.inference_worker.wait_for_result(
            frozen.frame_id, timeout_sec=timeout_sec)
        if inference is None or inference.frame_id != frozen.frame_id:
            self.capture_stream.resume()
            return Observation(ok=False, reason="승인용 최신 인식 시간초과")

        value = inference.value or {}
        crop = value.get("screen_image")
        ocr_results = list(value.get("ocr_results") or [])
        if crop is not None and ocr_results:
            self.layout_worker.submit(frozen.frame_id, (crop, ocr_results))
            self._layout_submitted_frame_id = frozen.frame_id
            self.layout_worker.wait_for_result(
                frozen.frame_id,
                timeout_sec=min(
                    timeout_sec, LAYOUT_OPTIONAL_APPROVAL_WAIT_SEC),
            )
        obs = self._observation_from_inference(
            inference, preview_snapshot=frozen, require_layout=False)
        if (not obs.ok
                or obs.context.get("result_frame_id") != frozen.frame_id
                or obs.context.get("preview_frame_id") != frozen.frame_id):
            self.capture_stream.resume()
            return obs
        self._approval_frame_id = frozen.frame_id
        return obs

    def release_approval_frame(self):
        self._approval_frame_id = 0
        clearer = getattr(self.ctrl.converter, "clear_live_apriltag_pose", None)
        if callable(clearer):
            clearer()
        self.capture_stream.resume()

    def _centers(self, buttons: Sequence[Dict[str, Any]]) -> List[Tuple[float, float]]:
        out: List[Tuple[float, float]] = []
        for r in buttons:
            c = self.IMPL._button_result_center(r)
            if c is not None:
                out.append((float(c[0]), float(c[1])))
        return out

    def _to_detections(self, buttons: Sequence[Dict[str, Any]],
                       crop=None) -> List[SIG.Detection]:
        """워핑된 크롭 픽셀 -> 정규화 좌표.

        학습 로그(label_snapshots.csv)의 crop_center_norm_x 는
        crop_center_x / screen_crop_width 다. 여기서도 같은 기준을 써야 한다.
        """
        if crop is not None and getattr(crop, "size", 0) > 0:
            crop_h, crop_w = crop.shape[:2]
            w = float(crop_w)
            h = float(crop_h)
        else:
            w = float(self.ctrl.converter.img_width)
            h = float(self.ctrl.converter.img_height)
        out: List[SIG.Detection] = []
        for r in buttons:
            center = self.IMPL._button_result_center(r)
            if center is None:
                continue
            cx, cy = center
            out.append(SIG.Detection(
                text=r.get("text", ""),
                label=str(r.get("label", "")),
                x=cx / w,
                y=cy / h,
                ocr_conf=float(r.get("ocr_confidence", r.get("confidence", 1.0)) or 0.0),
                conf=float(r.get("confidence", 1.0) or 0.0),
            ))
        return out

    # --- 타겟 검색 : 실장비에서는 원본 함수를 그대로 쓴다 ------------------------
    def match(self, obs: Observation, query: str) -> Optional[Dict[str, Any]]:
        position_target = obs.context.get("position_target")
        found = self.IMPL.find_button_match_in_results(
            obs.buttons, query, position_target=position_target)
        if found is not None or not getattr(self, "full_frame_ocr", False):
            return found
        # crop 경로(OCR+LayoutLM)에 없을 때만 같은 프레임의 4K 전체 프레임
        # 검출 결과에서 찾는다. 규칙은 같은 정규화 완전일치이고, 라벨이 없으므로
        # text 일치만 가능하다. Position 질의는 함수 안에서 전용 record만 허용한다.
        extra = obs.context.get("full_frame_ocr_results") or []
        if not extra:
            return None
        found = self.IMPL.find_button_match_in_results(
            extra, query, position_target=position_target)
        if found is None or found.get("match_type") != "text":
            return None
        found = dict(found)
        found["evidence_source"] = "full_frame_ocr"
        return found

    # --- 색 측정 : 색 게이트가 쓴다 --------------------------------------------
    def sample_color(self, obs: Observation,
                     match: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        """버튼 색을 잰다. 측정 자체는 impl 의 원본 함수에 위임한다.

        crop 은 워핑된 화면 크롭이고 match['record'] 의 bbox 도 크롭 픽셀 기준이라
        좌표계가 이미 맞다. 여기서 변환하면 오히려 틀어진다.
        """
        return CG.sample_color(self.ctrl, obs.context.get("crop"), match)

    # --- 구동 ---------------------------------------------------------------
    def press(self, obs: Observation, match: Dict[str, Any]) -> Tuple[bool, str]:
        """매칭된 버튼을 누른다. (성공여부, 설명)

        여기가 로봇과 AprilTag 가 처음으로 진짜 필요해지는 지점이다.
        --no-robot 이면 좌표까지만 계산해서 보여주고 팔은 움직이지 않는다.
        """
        ctrl, IMPL = self.ctrl, self.IMPL
        px = float(match["center_x"])
        py = float(match["center_y"])

        frame_id = int(obs.context.get("result_frame_id") or 0)
        preview_frame_id = int(obs.context.get("preview_frame_id") or 0)
        if (self._approval_frame_id <= 0
                or frame_id != self._approval_frame_id
                or preview_frame_id != self._approval_frame_id):
            self.release_approval_frame()
            raise ScenarioAbort(
                "STALE_VISION",
                "승인 직전 frame_id/pose/quad/target 재검증이 없습니다. "
                "로봇을 움직이지 않습니다.")

        # 좌표 변환에 pose 가 필요하다. 여기서 없으면 더 갈 수 없다.
        if not obs.context.get("pose_ok"):
            self.release_approval_frame()
            raise ScenarioAbort(
                "NO_POSE",
                "AprilTag 포즈를 구동에 쓸 수 없습니다 — {}\n"
                "             두 태그가 모두 카메라에 여유 있게 보이는지, 조명·각도를 "
                "확인하세요.".format(obs.context.get("pose_reason") or "포즈 없음"))

        pose = obs.context.get("pose") or {}
        try:
            geometry = obs.context.get("geometry") or {}
            registration = getattr(self, "tag_registration", None)
            if registration is not None:
                # 태그 기준: 픽셀 광선 ∩ 태그 평면 → T(로봇←태그 등록값). 카메라 위치와 무관.
                import hand_eye as HE
                try:
                    target = registration.press_target(
                        pose, geometry, px, py,
                        undistorted_source=bool(obs.context.get("undistort_on")))
                except HE.HandEyeError as exc:
                    raise ScenarioAbort(
                        exc.kind, exc.message + " 로봇을 움직이지 않습니다.")
                for kind, reason in target["violations"]:
                    if self.allow_robot:
                        raise ScenarioAbort(kind, reason + " 로봇을 움직이지 않습니다.")
                    print("      [좌표 경고] {}: {}".format(kind, reason))
                base_rx, base_ry, base_rz = target["robot_xyz"]
                source_desc = (f"픽셀({px:.0f},{py:.0f}) -> 태그"
                               "({:.1f},{:.1f},{:.1f})mm 법선 {:.1f}° -> 화면 표면 로봇".format(
                                   *target["tag_xyz"], target["normal_angle_deg"]))
            else:
                # 예전 고정식(컨트롤러 11/12번과 같은 식). 카메라→태그 X/Y와 화면 정규화
                # 좌표로 로봇 좌표를 만든다. 카메라를 옮기면 어긋난다.
                ctrl.converter.set_live_apriltag_pose(
                    pose["camera_to_tag_xyz"],
                    tag_to_camera_xyz=pose.get("tag_to_camera_xyz"),
                    distance_mm=pose.get("distance_mm"),
                    pitch=pose.get("pitch"),
                    yaw=pose.get("yaw"),
                    mode=pose.get("mode"),
                )
                norm_x, norm_y = IMPL.processed_to_normalized_point(
                    px, py, geometry)
                base_rx, base_ry, base_rz = (
                    ctrl.converter.normalized_to_robot_apriltag(norm_x, norm_y))
                source_desc = (f"픽셀({px:.0f},{py:.0f})/norm"
                               f"({norm_x:.4f},{norm_y:.4f}) -> 기본 로봇")
            offset_x, offset_y, offset_z = self.target_offsets()
            surface_x = base_rx + offset_x
            rx = surface_x
            ry = base_ry + offset_y
            rz = base_rz + offset_z
            # 시험 간격은 계산된 표면에서 빼기만 한다. 표면 X 자체가 바뀌어도 사람이
            # 다시 계산할 값이 없다.
            standoff = PRESS_X_TEST_STANDOFF_MM
            clamp_note = ""
            if standoff is not None and float(standoff) > 0.0:
                rx = surface_x - float(standoff)
                clamp_note = " (표면 {:.1f} - 시험 간격 {:.0f})".format(
                    surface_x, float(standoff))
            x_approach = rx - IMPL.X_APPROACH_GAP
            desc = (f"{source_desc}"
                    f"({base_rx:.1f},{base_ry:.1f},{base_rz:.1f})mm "
                    f"| 보정 X={offset_x:+.0f} Y={offset_y:+.0f} Z={offset_z:+.0f}mm "
                    f"-> 타겟({rx:.1f},{ry:.1f},{rz:.1f})mm "
                    f"| 접근 X={x_approach:.1f} 누름 X={rx:.1f}{clamp_note}")
            print(f"      {desc}")

            # 누름 X 상한(impl press_x()도 같은 값으로 막는다). 로봇 접속 전에 멈춘다.
            x_limit = float(getattr(IMPL, "PRESS_X_LIMIT_MM", 800.0))
            if rx > x_limit:
                message = ("누름 X {:.1f}mm가 상한 {:.1f}mm를 넘습니다 — Target Offset X를 "
                           "{:.0f}mm 이상 줄이세요.".format(rx, x_limit, rx - x_limit))
                if self.allow_robot:
                    raise ScenarioAbort(
                        "PRESS_X_LIMIT", message + " 로봇을 움직이지 않습니다.")
                print("      [누름 X 경고] " + message)

            # 도달 가능성. 접근 X가 베이스에 너무 가까우면 이 자세(RX/RY/RZ)의 역기구학
            # 해가 없어 MoveL이 error=38로 실패한다. 2026-09-21 실측: 접근 175 실패,
            # 대기 자세 250 도달. 팔이 움직이기 전에 여기서 멈춘다.
            min_approach = float(PRESS_X_TEST_MIN_APPROACH_MM)
            if x_approach < min_approach:
                message = ("접근 X {:.1f}mm는 이 자세로 도달할 수 없습니다(최소 {:.0f}mm) — "
                           "역기구학 해가 없어 MoveL이 실패합니다. 시험 간격"
                           "(--press-standoff)을 {:.0f}mm 이상 줄이세요.".format(
                               x_approach, min_approach, min_approach - x_approach))
                if self.allow_robot:
                    raise ScenarioAbort(
                        "PRESS_X_UNREACHABLE", message + " 로봇을 움직이지 않습니다.")
                print("      [도달 경고] " + message)

            if not self.allow_robot:
                print(f"      [모의] 로봇을 움직이지 않습니다 (--no-robot).")
                print(f"      >>> 대신 손으로 \"{match.get('text')}\" 를 눌러주세요.")
                return True, f"[모의] {desc}"

            self._ensure_robot()          # 첫 누름에서만 실제로 접속한다
            ok = ctrl._try_press_x(ry, rz, x_press=rx)
            return ok, desc
        finally:
            self.release_approval_frame()


# ----------------------------------------------------------------------------
# 지각 : 로그 재생 (로봇 없이 검증용)
# ----------------------------------------------------------------------------

# 로그 스냅샷 -> 화면. screen_id_signature.SCREEN_MAP 의 역방향.
def _snapshot_of(screen: str) -> Optional[str]:
    for sid, scr in SIG.SCREEN_MAP.items():
        if scr == screen:
            return sid
    return None


# 로그의 color_* 컬럼. 모의 실행에서 색 게이트를 진짜로 돌려보기 위한 것이다.
# SIG.Detection 은 판별에 필요한 필드만 갖고 있어 색이 없다.
COLOR_FIELDS = ("color_name", "color_state", "color_saturation_mean",
                "color_brightness_mean", "color_rgb_hex_mean", "color_hsv_mean")


def _load_log_colors(log_path: str) -> Dict[Tuple[str, str, str], Dict[str, Any]]:
    """(스냅샷ID, 소문자 텍스트, 소문자 라벨) -> color_* 딕셔너리."""
    import csv

    out: Dict[Tuple[str, str, str], Dict[str, Any]] = {}
    try:
        with open(log_path, encoding="utf-8-sig", newline="") as fp:
            for row in csv.DictReader(fp):
                key = (row.get("snapshot_id", ""),
                       str(row.get("text", "")).strip().lower(),
                       str(row.get("label", "")).strip().lower())
                # 같은 키가 여러 번 나오면 채도가 높은 쪽(활성)을 남긴다.
                prev = out.get(key)
                cur = {k: row.get(k, "") for k in COLOR_FIELDS}
                if prev is None or _sat_of(cur) > _sat_of(prev):
                    out[key] = cur
    except (OSError, csv.Error):
        return {}
    return out


def _sat_of(info: Dict[str, Any]) -> float:
    try:
        return float(info.get("color_saturation_mean") or 0.0)
    except (TypeError, ValueError):
        return 0.0


class ReplayPerception:
    """label_snapshots.csv 를 재생해 로봇 없이 시나리오를 돌린다.

    현재 화면을 내부 상태로 들고 있다가, 누름이 성공하면 그 스텝의
    expect_after 화면으로 넘어간 것처럼 굴린다. 게이팅 흐름 자체를 검증하는 용도다.
    실제 OCR 흔들림은 재현하지 않으므로 '통과했다'가 실장비 성공을 뜻하지는 않는다.
    """

    def __init__(self, log_path: str = DEFAULT_LOG, start_screen: str = "Main"):
        self.snaps = SIG.load_snapshots(log_path)
        self.screen = start_screen
        self.presses: List[str] = []
        self.colors = _load_log_colors(log_path)

    def close(self) -> None:
        pass

    def _dets(self) -> List[SIG.Detection]:
        sid = _snapshot_of(self.screen)
        if sid is None or sid not in self.snaps:
            return []
        return self.snaps[sid]

    def observe(self) -> Observation:
        dets = self._dets()
        if not dets:
            return Observation(ok=False, reason=f"'{self.screen}' 스냅샷이 로그에 없습니다")
        buttons = [{
            "text": d.text,
            "label": d.label,
            "ocr_confidence": d.ocr_conf,
            "confidence": d.conf,
            # 정규화 좌표를 그대로 중심으로 쓴다(모의 실행이므로 단위는 의미 없음)
            "bbox": [d.x, d.y, d.x, d.y],
        } for d in dets]
        return Observation(dets=list(dets), buttons=buttons, ok=True,
                           context={"captured_at": time.time()})

    def heater_reading(self, _obs: Observation, observation_id: str) -> Dict[str, Any]:
        """--dry-run 전용 계약 충족 fixture. 실기 판독 성공을 주장하지 않는다."""
        fixtures = {
            "heater_process_ready": {
                "pv_c": 25.0, "sp_c": 20.0, "current_a": 0.0,
                "aux_c": 26.0, "output_active": False,
            },
            "heater_current_zero": {
                "pv_c": 300.0, "sp_c": 300.0, "current_a": 0.0,
                "aux_c": 24.0, "output_active": False,
            },
            "heater_cooldown_ready": {
                "pv_c": 20.0, "sp_c": 20.0, "current_a": 0.0,
                "aux_c": 24.0, "output_active": False,
            },
        }
        return dict(fixtures[observation_id])

    def match(self, obs: Observation, query: str) -> Optional[Dict[str, Any]]:
        r = find_target(obs.buttons, query)
        if r is None:
            return None
        return {
            "record": r,
            "text": r.get("text", ""),
            "label": r.get("label", ""),
            "match_type": "label" if field_matches(r.get("label"), query) else "text",
            "confidence": float(r.get("ocr_confidence", 0.0) or 0.0),
            "center_x": float(r["bbox"][0]),
            "center_y": float(r["bbox"][1]),
        }

    def sample_color(self, obs: Observation,
                     match: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        """로그에 기록된 색을 그대로 돌려준다.

        재생이므로 '누른 뒤 색이 바뀌는' 것까지는 재현하지 못한다.
        로그가 찍힌 그 순간의 색이다. 그래서 모의 실행에서 색 게이트가 실패하는 건
        정상일 수 있다 — 게이트 배선이 도는지를 보는 용도다.
        """
        sid = _snapshot_of(self.screen)
        if sid is None:
            return None
        key = (sid,
               str(match.get("text", "")).strip().lower(),
               str(match.get("label", "")).strip().lower())
        return self.colors.get(key)

    def press(self, obs: Observation, match: Dict[str, Any]) -> Tuple[bool, str]:
        desc = (f"[모의] '{match['text']}' @정규화"
                f"({match['center_x']:.3f},{match['center_y']:.3f}) 누름")
        print(f"      {desc}")
        self.presses.append(match["text"])
        return True, desc

    # 러너가 누름 성공 후 호출한다 (실장비 경로에는 없는 훅)
    def advance(self, screen: str) -> None:
        self.screen = screen


# ----------------------------------------------------------------------------
# 시나리오 러너
# ----------------------------------------------------------------------------

@dataclass
class StepRecord:
    step: Step
    status: str                 # PASS / ABORT / SKIP
    detail: str = ""
    elapsed_sec: float = 0.0


class StepSkipped(Exception):
    """[s] 로 이 스텝을 건너뛴다. 시나리오는 계속 간다.

    누르지 않고 넘어가므로 화면도 안 바뀐다. 다음 스텝의 게이트가 안 열려서
    거기서 멈추는 게 정상이다. 특정 스텝만 손으로 처리하고 싶을 때 쓴다.
    """


class GateFailed(Exception):
    """색 게이트가 통과하지 못했다.

    ScenarioAbort 와 구분하는 이유가 있다. Abort 는 '위험하니 멈춘다'이고
    이쪽은 '아직 안 눌렸다'이다. 조치가 정반대다 — 전자는 정지, 후자는 재실행.
    """

    def __init__(self, gate: CG.Gate, detail: str):
        super().__init__(detail)
        self.gate = gate
        self.detail = detail


class ScenarioRunner:
    """타임아웃을 인자로 받는 이유

    --no-robot 에서는 사람이 손으로 눌러 화면을 넘긴다. 로봇이 1초 만에 하는 일을
    사람은 10초쯤 걸려서 한다. 기본값(전환 15초)으로는 손이 느리다는 이유로
    TIMEOUT 이 나서, 진짜 문제(판별 실패)와 구분이 안 된다.
    """

    def __init__(self, perception, judge: ScreenJudge,
                 match_timeout_sec: float = MATCH_TIMEOUT_SEC,
                 confirm_transition: bool = True,
                 require_timeout_sec: float = REQUIRE_TIMEOUT_SEC,
                 startup_timeout_sec: float = STARTUP_REQUIRE_TIMEOUT_SEC,
                 transition_timeout_sec: float = TRANSITION_TIMEOUT_SEC,
                 hold_confirm_sec: float = HOLD_CONFIRM_SEC,
                 view=None, auto: bool = False,
                 max_retry: int = MAX_STEP_RETRY, gates: bool = True,
                 progress_view=None):
        self.p = perception
        self.judge = judge
        self.match_timeout_sec = match_timeout_sec
        self.confirm_transition = confirm_transition
        self.require_timeout_sec = require_timeout_sec
        self.startup_timeout_sec = max(0.0, float(startup_timeout_sec))
        self.transition_timeout_sec = transition_timeout_sec
        self.hold_confirm_sec = hold_confirm_sec
        self.max_retry = int(max_retry)
        self.gates = bool(gates)
        self.records: List[StepRecord] = []

        # 패널에 찍을 마지막 색 측정값. _rows 가 읽는다.
        self._color_line: str = ""
        self._attempt: int = 1

        # view 가 None 이면 표시도 키 입력도 없다. --dry-run 이 그 경우다.
        self.view = view

        # 진행 창. 카메라 패널과 달리 '전체 중 몇 번째'를 보여준다. 모델은 창이
        # 없어도 돌린다 — 요약과 테스트가 같은 상태를 읽을 수 있어야 한다.
        import scenario_progress as PR
        self.progress = PR.ProgressModel()
        self.progress_view = progress_view
        self.auto = bool(auto)
        self.paused = False
        self.poll_sec = VIEW_POLL_INTERVAL_SEC if view is not None else POLL_INTERVAL_SEC

        self._step_i = 0
        self._total = 0
        self._banner: Optional[str] = None
        self._banner_kind = "go"
        self._banner_until = 0.0

    # ------------------------------------------------------------------
    # 표시
    # ------------------------------------------------------------------

    def _rows(self, step: Step, phase: str, phase_kind: str,
              obs: Optional[Observation], verdict: Optional[SIG.Result],
              match: Optional[Dict[str, Any]]):
        """패널에 그릴 줄들을 만든다. 판단은 안 하고 이미 나온 결과만 옮긴다."""
        import scenario_overlay as OV

        screen = verdict.screen if (verdict and verdict.decided) else None
        if verdict is None:
            gate, gate_kind = "-", "dim"
        elif not verdict.decided:
            gate, gate_kind = "ABSTAIN", "warn"
        elif screen == step.require:
            gate, gate_kind = "OK", "ok"
        else:
            gate, gate_kind = "MISMATCH", "bad"

        if step.is_observation:
            btn, btn_kind = "N/A (CAMERA)", "dim"
        elif match is None:
            btn, btn_kind = "NOT FOUND", "bad"
        else:
            btn = f"FOUND ({match.get('match_type', '?')})"
            if match.get("evidence_source") == "full_frame_ocr":
                btn = f"FOUND (4K full-frame {match.get('match_type', '?')})"
            btn_kind = "ok"

        pose_ok = bool(obs.context.get("pose_ok")) if obs else False
        arrow = "hold" if step.holds else f"-> {step.expect_after}"

        # ReplayPerception 에는 이 속성이 없다. 기본을 '로봇 없음'으로 본다.
        if not getattr(self.p, "allow_robot", False):
            robot, robot_kind = "SIMULATED (--no-robot)", "warn"
        elif getattr(self.p, "robot_ready", False):
            robot, robot_kind = "CONNECTED", "ok"
        else:
            robot, robot_kind = "not connected yet", "dim"

        head = f"STEP {step.no} / {self._total}"
        if self._attempt > 1:
            head += f"  (retry {self._attempt - 1})"

        rows = [
            OV.Row(head),
            OV.Row("BLOCK", step.block or "-", "dim", 0.5),
            OV.Row("PHASE", phase, phase_kind, 0.6),
            OV.Row("EXPECT", step.require),
            OV.Row("TARGET", f'"{step.target}"', "target"),
            OV.Row("AFTER", arrow, "dim", 0.5),
            OV.sep(),
            OV.Row("SCREEN", screen if screen else "ABSTAIN",
                   "text" if screen else "warn", 0.6),
            OV.Row("GATE", gate, gate_kind, 0.6),
            OV.Row("BUTTON", btn, btn_kind),
            OV.Row("APRILTAG", ("N/A" if step.is_observation else
                                ("OK" if pose_ok else "NG")),
                   "dim" if step.is_observation else ("ok" if pose_ok else "bad")),
            OV.Row("ROBOT", robot, robot_kind, 0.5),
        ]
        offset_getter = getattr(self.p, "target_offsets", None)
        if callable(offset_getter):
            offset_x, offset_y, offset_z = offset_getter()
            rows.append(OV.Row(
                "OFFSET",
                "X {:+.0f}  Y {:+.0f}  Z {:+.0f} mm".format(
                    offset_x, offset_y, offset_z),
                "warn" if any((offset_x, offset_y, offset_z)) else "dim", 0.48))
        if obs is not None and obs.context.get("result_frame_id") is not None:
            result_id = int(obs.context.get("result_frame_id") or 0)
            preview_id = int(obs.context.get("preview_frame_id") or 0)
            stale = bool(obs.context.get("stale"))
            rows.append(OV.Row(
                "FRAME",
                "preview {} / result {}{}".format(
                    preview_id, result_id, " STALE" if stale else ""),
                "warn" if stale else "ok", 0.48))
            capture_stats = obs.context.get("capture_stats") or {}
            inference_stats = obs.context.get("inference_stats") or {}
            rows.append(OV.Row(
                "PERF",
                "view {:.1f} / cap {:.1f} / infer {:.2f}fps {:.0f}ms".format(
                    float(obs.context.get("preview_fps", 0.0)),
                    float(capture_stats.get("capture_fps", 0.0)),
                    float(inference_stats.get("inference_fps", 0.0)),
                    float(inference_stats.get("total_ms", 0.0))),
                "dim", 0.45))
            rows.append(OV.Row(
                "TEXT",
                "{} OCR={} LM={}".format(
                    obs.context.get("button_evidence", "ocr-only"),
                    int(obs.context.get("ocr_button_count", 0)),
                    int(obs.context.get("layout_button_count", 0))),
                "text", 0.45))
        if (obs is not None
                and step.require == "Main"
                and "position_target_meta" in obs.context):
            position_value, position_kind = _position_overlay_status(
                obs.context.get("position_target"),
                obs.context.get("position_target_meta"),
            )
            # 진단 문자열은 표시 전용이다. 이 Row를 버튼 목록이나 판정 결과로
            # 되돌려 쓰지 않아 Position의 fail-closed 계약에 영향을 주지 않는다.
            rows.append(OV.Row(
                "POSITION", position_value[:48], position_kind, 0.40))
        # 색 게이트 단계에서만 의미가 있다. 그 외에는 줄을 빼서 패널을 짧게 둔다.
        if step.gate is not None:
            g = step.gate
            want = "manual" if g.is_manual else (g.spec.label if g.spec else "?")
            rows.append(OV.Row(f"GATE {g.gate_id}", want, "dim", 0.5))
            if self._color_line:
                rows.append(OV.Row("COLOR", self._color_line[:34], "text", 0.5))
        if step.is_observation:
            reading = obs.context.get("heater") if obs is not None else None
            if reading:
                value = "PV {:.0f} SP {:.0f} I {:.1f}A".format(
                    reading["pv_c"], reading["sp_c"], reading["current_a"])
                rows.append(OV.Row("HEATER", value, "text", 0.5))
            elif obs is not None:
                rows.append(OV.Row("HEATER", "ABSTAIN", "warn", 0.5))
        if obs is not None and not obs.ok and obs.reason:
            rows.append(OV.Row("REASON", obs.reason[:38], "warn", 0.46))
        if self.auto:
            rows.append(OV.Row("MODE", "AUTO (paused)" if self.paused else "AUTO",
                               "warn" if self.paused else "dim", 0.5))
        return rows

    def _render(self, step: Step, phase: str, phase_kind: str = "dim",
                obs: Optional[Observation] = None,
                verdict: Optional[SIG.Result] = None,
                match: Optional[Dict[str, Any]] = None,
                banner: Optional[str] = None, banner_kind: str = "go") -> None:
        # 진행 모델이 먼저다. 카메라 창을 끄고 돌려도(--dry-run, --no-view)
        # 어디까지 갔는지는 남아야 한다.
        self._advance_progress(step, phase, obs, verdict, match)
        if self.view is None:
            return
        import scenario_overlay as OV

        # 경고 배너는 잠깐만 띄운다. 안 그러면 언제 뜬 경고인지 알 수 없다.
        if banner is None and self._banner and time.time() < self._banner_until:
            banner, banner_kind = self._banner, self._banner_kind
        elif banner is None:
            self._banner = None

        ctx = obs.context if obs is not None else {}
        preview_coordinates_valid = bool(
            ctx.get("screen_preview_coordinates_valid"))
        target_xy = None
        if match is not None and preview_coordinates_valid:
            target_xy = (float(match["center_x"]), float(match["center_y"]))

        raw_frame_id = ctx.get("preview_frame_id") or ctx.get("raw_frame_id")
        self.view.render(
            self._rows(step, phase, phase_kind, obs, verdict, match),
            frame=None,
            raw_frame=ctx.get("frame"),
            raw_label=("CAMERA RAW  frame={}".format(raw_frame_id)
                       if raw_frame_id else "CAMERA RAW"),
            crop=ctx.get("screen_preview_crop"),
            dots=(ctx.get("dots") or ()) if preview_coordinates_valid else (),
            target_xy=target_xy,
            target_text=str(match.get("text", "")) if match else "",
            banner=banner,
            banner_kind=banner_kind,
        )

    def _advance_progress(self, step: Step, phase: str,
                          obs: Optional[Observation],
                          verdict: Optional[SIG.Result],
                          match: Optional[Dict[str, Any]]) -> None:
        """진행 모델을 갱신하고, 창이 있으면 다시 그린다.

        패널을 그리는 바로 그 자리에서 부른다. 판단 지점을 새로 만들지 않았으므로
        두 창이 서로 다른 이야기를 할 수 없다. 여기서 넘기는 `live` 는 전부 이미
        나온 결과를 옮기는 것뿐이다.
        """
        live: Dict[str, Any] = {}
        if verdict is not None:
            live["decided"] = bool(verdict.decided)
            live["screen"] = verdict.screen if verdict.decided else ""
            live["reason"] = str(getattr(verdict, "reason", "") or "")[:48]
        if match is not None:
            live["button"] = str(match.get("text", "") or match.get("label", ""))
        if obs is not None:
            live["pose_ok"] = bool(obs.context.get("pose_ok"))
        if not getattr(self.p, "allow_robot", False):
            live["robot"] = "none (--no-robot)"
        elif getattr(self.p, "robot_ready", False):
            live["robot"] = "connected"

        detail = self._color_line if (step.gate is not None or step.is_observation) else ""
        if obs is not None and not obs.ok and obs.reason:
            detail = obs.reason
        self.progress.mark(step.no, phase, attempt=self._attempt,
                           detail=detail, live=live)
        if self.progress_view is not None:
            self.progress_view.render(self.progress)

    def _repaint_progress(self) -> None:
        """스텝 결과가 바뀐 자리에서 진행 창만 다시 그린다.

        `_render` 와 달리 카메라 프레임이 필요 없다. 스텝이 끝나거나 실패한
        직후에는 다음 관측이 오기 전이라 그릴 프레임이 없기 때문이다.
        """
        if self.progress_view is not None:
            self.progress_view.render(self.progress)

    def _flash(self, text: str, kind: str = "warn", sec: float = 2.0) -> None:
        self._banner = text
        self._banner_kind = kind
        self._banner_until = time.time() + sec

    # ------------------------------------------------------------------
    # 키
    # ------------------------------------------------------------------

    def _handle_common_keys(self, key: int, step: Step) -> None:
        """어느 단계에서나 통하는 키. [g] 는 단계마다 뜻이 달라 여기서 안 다룬다."""
        import scenario_overlay as OV

        if key in OV.KEY_QUIT:
            raise ScenarioAbort("USER_STOP", "사용자가 [q] 로 중단했습니다.", step)
        if key == OV.KEY_PAUSE and self.auto:
            self.paused = not self.paused
            print(f"  자동 진행 {'일시정지' if self.paused else '재개'}")
        if key == OV.KEY_CAPTURE and self.view is not None:
            path = self.view.capture(f"step{step.no}")
            if path:
                print(f"  저장: {path}")
            if self.progress_view is not None:
                path = self.progress_view.capture(f"step{step.no}")
                if path:
                    print(f"  저장: {path}")

    def _tick(self, step: Step, phase: str, phase_kind: str = "dim",
              banner: Optional[str] = None, banner_kind: str = "warn",
              verdict_hook: Optional[Callable[[SIG.Result], None]] = None):
        """wait_for 가 매 폴링마다 부르는 콜백. 그리고, 키를 본다."""
        import scenario_overlay as OV

        def _cb(obs: Observation, verdict: SIG.Result) -> None:
            if verdict_hook is not None:
                verdict_hook(verdict)
            self._render(step, phase, phase_kind, obs, verdict,
                         banner=banner, banner_kind=banner_kind)
            if self.view is None:
                return
            while True:
                k = self.view.poll_key()
                if k == OV.NO_KEY:
                    break
                if k == OV.KEY_GO:
                    # 아직 구동할 수 있는 상태가 아니다. 무시하고 알려만 준다.
                    self._flash("NOT READY - waiting for screen", "bad")
                    print("  [무시] 아직 구동할 수 없습니다 — 게이트 통과 전입니다.")
                    continue
                self._handle_common_keys(k, step)
        return _cb

    # --- 1) 전제조건 : 지금 그 화면인가 --------------------------------------
    def _require(self, step: Step) -> None:
        startup = self._step_i == 0 and not self.records
        timeout = self.startup_timeout_sec if startup else self.require_timeout_sec
        announced: List[str] = []
        last_decided_mismatch: List[SIG.Result] = []

        def announce(verdict: SIG.Result) -> None:
            if not startup or not verdict.decided or verdict.screen == step.require:
                return
            last_decided_mismatch[:] = [verdict]
            actual = str(verdict.screen or "")
            if actual in announced:
                return
            announced.append(actual)
            print(f"    [시작 화면 준비] 기대 {step.require} / 실제 {actual}")
            print("      로봇은 연결하거나 움직이지 않습니다.")
            if (step.require == "Main"
                    and actual in {"Menu_mode", "Function", "Gas.M",
                                   "Heater", "Loadlock_ccg"}):
                print("      현재 HMI 화면의 [Exit]를 사람이 손으로 눌러 "
                      "Main으로 복귀하세요.")
            else:
                print(f"      HMI를 사람이 확인해 {step.require} 화면으로 준비하세요.")
            print(f"      최대 {timeout:.0f}초 동안 화면을 다시 확인합니다.")

        if startup:
            tick = self._tick(
                step, "STARTUP SCREEN", "warn",
                banner=f"MANUALLY SET {step.require.upper()}",
                banner_kind="warn", verdict_hook=announce)
        else:
            tick = self._tick(step, "WAIT SCREEN", "warn")

        ok, verdict = self.judge.wait_for(
            step.require, self.p.observe, timeout,
            poll_sec=self.poll_sec, tick=tick)
        if ok:
            if startup and announced:
                print(f"    시작 화면 복귀 확인 : {step.require}")
            print(f"    게이트 통과 : {step.require}  ({verdict.reason})")
            return
        evidence = ""
        if verdict.evidence:
            evidence = "\n             판정 근거: " + " | ".join(verdict.evidence[:3])
        diagnostic = ("\n             화면 판별 진단: 검출 {}건, 득표 {}".format(
            verdict.n_detections,
            dict(sorted(verdict.votes.items(), key=lambda item: -item[1])),
        ))
        if not verdict.decided and startup and last_decided_mismatch:
            confirmed = last_decided_mismatch[-1]
            raise ScenarioAbort(
                "MISMATCH",
                "시작 화면 준비 시간초과 — 기대 {} / 마지막 확정 {}. "
                "마지막 프레임은 {}로 판정할 수 없었습니다. 로봇을 움직이지 "
                "않습니다.{}{}".format(
                    step.require, confirmed.screen, verdict.reason,
                    diagnostic, evidence),
                step)
        if not verdict.decided:
            raise ScenarioAbort(
                "ABSTAIN",
                f"화면을 확신할 수 없습니다 — {verdict.reason}. "
                f"로봇을 움직이지 않습니다.{diagnostic}{evidence}",
                step)
        prefix = "시작 화면 준비 시간초과 — " if startup else ""
        raise ScenarioAbort(
            "MISMATCH",
            f"{prefix}기대 {step.require} / 실제 {verdict.screen} — "
            f"{verdict.reason}. 로봇을 움직이지 않습니다."
            f"{diagnostic}{evidence}",
            step)

    # --- 2) 타겟 검색 : 완전일치, 실패하면 우회 없이 중단 ----------------------
    def _find(self, step: Step) -> Tuple[Observation, Dict[str, Any]]:
        import scenario_overlay as OV

        deadline = time.time() + self.match_timeout_sec
        obs = self.p.observe()
        attempts = 0
        while True:
            attempts += 1
            m = self.p.match(obs, step.target) if obs.ok else None
            self._render(step, "FIND TARGET", "warn", obs, self.judge.last, m)
            if self.view is not None:
                while True:
                    k = self.view.poll_key()
                    if k == OV.NO_KEY:
                        break
                    if k == OV.KEY_GO:
                        self._flash("NOT READY - target not confirmed", "bad")
                        continue
                    self._handle_common_keys(k, step)
            if m is not None:
                kind = m.get("match_type", "?")
                lbl = f" label='{m.get('label')}'" if m.get("label") else ""
                src_note = (" (4K 전체 프레임 보조 검출 — crop 경로엔 없음, 위치를 눈으로 확인)"
                            if m.get("evidence_source") == "full_frame_ocr" else "")
                print(f"    타겟 발견 [{kind}] \"{m.get('text')}\"{lbl}{src_note}")
                return obs, m
            if time.time() >= deadline:
                break
            time.sleep(min(MATCH_POLL_SEC, self.poll_sec))
            obs = self.p.observe()

        near = describe_near(obs.buttons, step.target) if obs.ok else obs.reason
        raise ScenarioAbort(
            "NO_MATCH",
            f"'{step.target}' 를 {step.require} 화면에서 찾지 못했습니다 "
            f"({attempts}회 시도, {self.match_timeout_sec:.0f}s).\n"
            f"             완전일치 실패 — OCR 오독으로 보입니다.\n"
            f"             가까운 검출: {near}",
            step)

    def _approval_ready(self, step: Step, obs: Observation):
        """승인 시점의 한 frame_id에서 화면·pose·quad·타겟을 다시 확인한다."""
        refresher = getattr(self.p, "approval_observation", None)
        candidate = refresher() if callable(refresher) else obs
        verdict = self.judge.judge(candidate)
        screen_ok = bool(verdict.decided and verdict.screen == step.require)
        match = self.p.match(candidate, step.target) if candidate.ok else None
        pose_ok = (bool(candidate.context.get("pose_ok"))
                   or not getattr(self.p, "allow_robot", False))
        result_id = candidate.context.get("result_frame_id")
        preview_id = candidate.context.get("preview_frame_id")
        exact_frame = (result_id is None or result_id == preview_id)
        ready = bool(screen_ok and match is not None and pose_ok and exact_frame)
        if not ready:
            releaser = getattr(self.p, "release_approval_frame", None)
            if callable(releaser):
                releaser()
        if not screen_ok:
            why = "승인용 최신 화면이 맞지 않습니다"
        elif match is None:
            why = "승인용 최신 프레임에서 버튼이 안 보입니다"
        elif not pose_ok:
            why = "승인용 최신 프레임의 AprilTag 포즈를 쓸 수 없습니다 — {}".format(
                candidate.context.get("pose_reason") or "포즈 없음")
        elif not exact_frame:
            why = "승인용 preview/result frame_id가 다릅니다"
        else:
            why = ""
        return ready, candidate, match, why

    # --- 2.5) 구동 승인 : [g] 를 누를 때까지 기다린다 --------------------------
    def _await_go(self, step: Step, obs: Observation,
                  match: Dict[str, Any]) -> Tuple[Observation, Dict[str, Any]]:
        """게이트도 통과했고 버튼도 찾았다. 이제 사람이 [g] 로 승인해야 누른다.

        기다리는 동안에도 계속 다시 관측하고 다시 매칭한다. 이유가 있다.
        [g] 를 누르는 데 10초가 걸렸다면 그 사이에 카메라가 흔들렸을 수도,
        누가 HMI 를 건드렸을 수도 있다. 승인 시점의 화면이 아니라
        '누르기 직전 프레임'의 좌표로 눌러야 안전하다.

        그래서 반환값은 인자로 받은 obs/match 가 아니라 최신 것이다.
        준비가 깨지면(화면이 바뀌거나 버튼이 사라지면) NOT READY 로 돌아가고
        [g] 는 먹지 않는다.
        """
        import scenario_overlay as OV

        if self.auto and not self.paused:
            ready, fresh_obs, fresh_match, why = self._approval_ready(step, obs)
            if ready:
                return fresh_obs, fresh_match
            raise ScenarioAbort(
                "STALE_VISION",
                "자동 구동 직전 최신 프레임 재검증 실패 — {}".format(why), step)

        # 창이 없거나(--no-view) 열다 실패했으면 키를 받을 데가 없다.
        # 그대로 두면 승인할 방법 없이 영원히 대기하게 되므로 터미널로 받는다.
        if self.view is None or not getattr(self.view, "enabled", True):
            try:
                while True:
                    ans = input("    구동 g / 건너뛰기 s / 중단 q > ").strip().lower()
                    if ans == "g":
                        ready, fresh_obs, fresh_match, why = self._approval_ready(
                            step, obs)
                        if ready:
                            return fresh_obs, fresh_match
                        print("    [무시] 구동할 수 없습니다 — {}".format(why))
                        obs = self.p.observe()
                        continue
                    if ans == "s":
                        raise StepSkipped()
                    if ans == "q":
                        raise ScenarioAbort("USER_STOP", "사용자가 중단했습니다.", step)
            except (EOFError, KeyboardInterrupt):
                raise ScenarioAbort("USER_STOP", "입력이 끊겨 중단합니다.", step)

        print(f"    >>> 준비되면 창에서 [g] 를 누르세요. "
              f"(건너뛰기 [s], 중단 [q])")
        self.view.drain_keys()      # 이전 단계에서 눌린 키가 여기 먹으면 안 된다
        ready_logged = False

        while True:
            verdict = self.judge.judge(obs)
            screen_ok = verdict.decided and verdict.screen == step.require
            fresh = self.p.match(obs, step.target) if obs.ok else None
            # --no-robot 이면 좌표를 쓸 일이 없으므로 AprilTag 를 요구하지 않는다
            pose_ok = (bool(obs.context.get("pose_ok"))
                       or not getattr(self.p, "allow_robot", False))
            ready = bool(screen_ok and fresh is not None and pose_ok)

            if ready:
                phase, kind = "READY", "go"
                banner = "PRESS [g] TO DRIVE"
                if not ready_logged:
                    print(f"    준비 완료 — [g] 대기 중")
                    ready_logged = True
            else:
                phase, kind = "NOT READY", "bad"
                banner = None
                ready_logged = False

            self._render(step, phase, kind, obs, verdict, fresh,
                         banner=banner, banner_kind="go")

            while True:
                k = self.view.poll_key()
                if k == OV.NO_KEY:
                    break
                if k == OV.KEY_GO:
                    if ready:
                        approved, fresh_obs, fresh_match, why = (
                            self._approval_ready(step, obs))
                        if approved:
                            print("    [g] 구동 승인 — frame {} \"{}\" 를 누릅니다".format(
                                fresh_obs.context.get("result_frame_id", "replay"),
                                fresh_match.get("text")))
                            return fresh_obs, fresh_match
                        self._flash("NOT READY - fresh validation", "bad")
                        print("  [무시] 구동할 수 없습니다 — {}".format(why))
                        obs = self.p.observe()
                        continue
                    why = ("화면이 맞지 않습니다" if not screen_ok else
                           "버튼이 지금 안 보입니다" if fresh is None else
                           "AprilTag 포즈를 쓸 수 없습니다 — {}".format(
                               obs.context.get("pose_reason") or "포즈 없음"))
                    self._flash(f"NOT READY - {phase}", "bad")
                    print(f"  [무시] 구동할 수 없습니다 — {why}")
                    continue
                if k == OV.KEY_SKIP:
                    raise StepSkipped()
                self._handle_common_keys(k, step)

            time.sleep(self.poll_sec)
            obs = self.p.observe()

    # --- 3) 누른 뒤 확인 ------------------------------------------------------
    def _confirm(self, step: Step) -> None:
        if not self.confirm_transition:
            return
        time.sleep(POST_PRESS_SETTLE_SEC)

        if step.holds:
            # 화면이 안 바뀌는 상태 토글. '여전히 그 화면인가'만 본다.
            ok, verdict = self.judge.wait_for(
                step.expect_after, self.p.observe, self.hold_confirm_sec,
                poll_sec=self.poll_sec, tick=self._tick(step, "CONFIRM HOLD"))
            if ok:
                print(f"    유지 확인 : {step.expect_after}")
                return
            if not verdict.decided:
                raise ScenarioAbort("ABSTAIN",
                                    f"누른 뒤 화면을 확신할 수 없습니다 — {verdict.reason}", step)
            raise ScenarioAbort(
                "MISMATCH",
                f"누른 뒤 {step.expect_after} 를 유지해야 하는데 {verdict.screen} 입니다", step)

        ok, verdict = self.judge.wait_for(
            step.expect_after, self.p.observe, self.transition_timeout_sec,
            poll_sec=self.poll_sec, tick=self._tick(step, "CONFIRM MOVE"))
        if ok:
            print(f"    전환 확인 : {step.require} -> {step.expect_after}")
            return

        # 안 넘어감 / 엉뚱한 화면 / 기권을 구분한다. 조치가 다르기 때문이다.
        if not verdict.decided:
            raise ScenarioAbort(
                "ABSTAIN",
                f"전환 후 화면을 확신할 수 없습니다 — {verdict.reason}", step)
        if verdict.screen == step.require:
            raise ScenarioAbort(
                "TIMEOUT",
                f"화면이 넘어가지 않았습니다 (여전히 {step.require}) — 클릭 미반영 가능", step)
        raise ScenarioAbort(
            "MISMATCH",
            f"의도하지 않은 화면 {verdict.screen} 으로 전환되었습니다", step)

    # --- 4) 색 게이트 : 눌러서 상태가 실제로 바뀌었는가 ------------------------
    def _gate(self, step: Step) -> None:
        """통과하면 조용히 반환. 실패하면 GateFailed.

        화면 게이트(_require/_confirm)와 역할이 다르다. 저쪽은 '이 동작을 해도
        되는 화면인가'를 보고, 여기는 '동작이 먹혔는가'를 본다. 화면이 안 바뀌는
        스텝(Position/Auto Mode/Heater Off)에서는 여기가 유일한 성공 신호다.
        """
        g = step.gate
        if g is None or not self.gates:
            return

        print(f"    {g.describe()}")
        if g.note:
            print(f"      근거: {g.note}")

        if g.is_manual:
            self._gate_manual(step, g)
            return

        import scenario_overlay as OV

        phase = f"GATE {g.gate_id}"
        deadline = time.time() + g.timeout_sec
        last_why = "아직 관측 없음"
        self._color_line = ""

        while True:
            obs = self.p.observe()
            verdict = self.judge.judge(obs)
            m = None
            ok = False

            if not (verdict.decided and verdict.screen == g.screen):
                last_why = (f"게이트를 볼 화면({g.screen})이 아닙니다 — "
                            f"{verdict.screen or 'ABSTAIN'}")
            else:
                m = self.p.match(obs, g.target) if obs.ok else None
                if m is None:
                    last_why = f'"{g.target}" 가 화면에서 안 보입니다'
                else:
                    info = self.p.sample_color(obs, m)
                    self._color_line = CG.summarize(info)
                    hit, detail = g.spec.matches(info or {})
                    if g.kind == CG.GATE_VANISH:
                        # 끄는 동작이다. 색이 '남아 있으면' 아직 안 꺼진 것이다.
                        ok = not hit
                        last_why = (f"아직 {detail}" if hit
                                    else f"소멸 확인 ({CG.summarize(info)})")
                    else:
                        ok, last_why = hit, detail

            kind = "ok" if ok else ("warn" if g.kind == CG.GATE_WAIT else "bad")
            banner = None
            if not ok and g.kind == CG.GATE_WAIT:
                left = max(0.0, deadline - time.time())
                banner = f"WAITING {left:.0f}s"
            self._render(step, phase, kind, obs, verdict, m,
                         banner=banner, banner_kind="go")

            if ok:
                print(f"    게이트 {g.gate_id} 통과 — {last_why}")
                self._color_line = ""
                return

            if self.view is not None:
                while True:
                    k = self.view.poll_key()
                    if k == OV.NO_KEY:
                        break
                    if k == OV.KEY_SKIP:
                        # 게이트만 건너뛴다. 스텝은 이미 눌렀으므로 되돌릴 게 없다.
                        print(f"    게이트 {g.gate_id} 건너뜀 — [s]")
                        self._color_line = ""
                        return
                    self._handle_common_keys(k, step)

            if time.time() >= deadline:
                break
            time.sleep(self.poll_sec)

        self._color_line = ""
        raise GateFailed(
            g, f"게이트 {g.gate_id} 실패 ({g.timeout_sec:.0f}s) — {last_why}")

    def _gate_manual(self, step: Step, g: CG.Gate) -> None:
        """색으로 못 잡는 게이트. 사람이 눈으로 보고 승인한다.

        자동 통과시키지 않는 이유: B/F/G/H/I는 화면 신호가 없거나 완료 신호를
        실측하지 못했다. 통과시키면 '안 눌렸는데 눌렸다고 보고'가 되고, 시나리오는 다음 블록
        으로 넘어가 버린다. 발표 데모에서는 여기가 오히려 설명 포인트다.
        """
        prompt = g.prompt or "동작이 완료되었습니까?"
        negative = "아니오·재시도" if g.retries_step else "아니오·중단"
        print(f"    >>> {prompt}  (예 [g] / {negative} [n] / 중단 [q])")

        if self.view is None or not getattr(self.view, "enabled", True):
            # 빈 Enter 를 '예'로 받지 않는다. 장비 상태를 확인했다는 뜻이어야 하는데
            # 그냥 Enter 를 눌러 넘길 수 있으면 확인 절차의 의미가 없다.
            try:
                while True:
                    ans = input(f"    {prompt} y/n/q > ").strip().lower()
                    if ans in ("y", "g"):
                        return
                    if ans == "n":
                        raise GateFailed(g, f"게이트 {g.gate_id} — 사람이 '아니오'로 답했습니다")
                    if ans == "q":
                        raise ScenarioAbort("USER_STOP", "사용자가 중단했습니다.", step)
                    print("      y / n / q 중 하나를 입력하세요.")
            except (EOFError, KeyboardInterrupt):
                raise ScenarioAbort("USER_STOP", "입력이 끊겨 중단합니다.", step)

        import scenario_overlay as OV

        self.view.drain_keys()
        while True:
            obs = self.p.observe()
            verdict = self.judge.judge(obs)
            self._render(step, f"GATE {g.gate_id} MANUAL", "warn", obs, verdict, None,
                         banner=f"{prompt}  [g]=YES  [n]=NO", banner_kind="warn")
            while True:
                k = self.view.poll_key()
                if k == OV.NO_KEY:
                    break
                if k == OV.KEY_GO:
                    print(f"    게이트 {g.gate_id} 통과 — 사람이 확인했습니다")
                    return
                if k == ord('n'):
                    raise GateFailed(
                        g, f"게이트 {g.gate_id} — 사람이 '아니오'로 답했습니다")
                self._handle_common_keys(k, step)
            time.sleep(self.poll_sec)

    # --- 외부 히터 카메라 관측 ------------------------------------------------
    def _save_heater_failure(self, step: Step,
                             obs: Optional[Observation]) -> List[str]:
        """판독 실패 프레임/크롭을 다음 진단용으로 남긴다."""
        save_dir = getattr(self.view, "save_dir", None) if self.view else None
        if not save_dir or obs is None:
            return []
        try:
            import cv2
            os.makedirs(save_dir, exist_ok=True)
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            prefix = "heater_abstain_step{}_{}".format(step.no, stamp)
            images = {
                "frame": obs.context.get("frame"),
                "sus": (obs.context.get("heater_crops") or {}).get("Heater_sus"),
                "current": (obs.context.get("heater_crops") or {}).get(
                    "Heater_current"),
                "fallback": obs.context.get("heater_fallback_crop"),
            }
            paths = []
            for label, image in images.items():
                if image is None or not getattr(image, "size", 0):
                    continue
                path = os.path.join(
                    save_dir, "{}_{}.jpg".format(prefix, label))
                if cv2.imwrite(path, image):
                    paths.append(path)
            return paths
        except Exception as exc:
            print("    [경고] 히터 ABSTAIN 진단 이미지 저장 실패: {}".format(exc))
            return []

    def _run_observation(self, step: Step) -> None:
        """서로 다른 새 카메라 프레임에서 조건이 연속 충족될 때만 통과."""
        import heater_vision as HV

        spec = HEATER_OBSERVATIONS.get(step.observation_id)
        if spec is None:
            raise ScenarioAbort("CONFIG", "알 수 없는 히터 관측 {}".format(
                step.observation_id), step)

        t0 = time.time()
        self._require(step)
        deadline = time.time() + float(spec["timeout_sec"])
        required_streak = int(spec["stable_frames"])
        streak = 0
        last_stamp = None
        last_reason = "아직 새 카메라 프레임 없음"
        obs = None

        while time.time() <= deadline:
            obs = self.p.observe()
            verdict = self.judge.judge(obs)
            stamp = obs.context.get("captured_at") if obs is not None else None
            reading = None
            if obs is not None:
                getter = getattr(self.p, "heater_reading", None)
                reading = (getter(obs, step.observation_id) if getter is not None
                           else obs.context.get("heater"))
                if reading is not None:
                    obs.context["heater"] = reading

            fresh_frame = stamp is not None and stamp != last_stamp
            screen_ok = bool(verdict.decided and verdict.screen == step.require)
            ok = False
            if not screen_ok:
                last_reason = "화면 {} 필요, 실제 {}".format(
                    step.require, verdict.screen or "ABSTAIN")
                streak = 0
            elif reading is None:
                last_reason = str(obs.context.get("heater_reason") or
                                  "외부 히터 숫자 판독 ABSTAIN")
                streak = 0
            elif fresh_frame:
                ok, last_reason = HV.evaluate_requirements(
                    reading, spec["requirements"])
                streak = streak + 1 if ok else 0

            self._color_line = "{} ({}/{})".format(
                last_reason, streak, required_streak)
            self._render(step, "OBSERVE HEATER", "ok" if ok else "warn",
                         obs, verdict, None,
                         banner=("HEATER OK" if streak >= required_streak else
                                 "WAIT HEATER"),
                         banner_kind="go" if streak >= required_streak else "warn")

            if fresh_frame:
                last_stamp = stamp
            if streak >= required_streak:
                desc = "카메라 관측 통과: {}".format(last_reason)
                self.records.append(StepRecord(step, "PASS", desc, time.time() - t0))
                self.progress.step_done(step.no)
                self._repaint_progress()
                self._color_line = ""
                print("    {} (서로 다른 {}프레임)".format(desc, required_streak))
                return

            if self.view is not None:
                import scenario_overlay as OV
                while True:
                    key = self.view.poll_key()
                    if key == OV.NO_KEY:
                        break
                    self._handle_common_keys(key, step)
            time.sleep(self.poll_sec)

        self._color_line = ""
        saved = self._save_heater_failure(step, obs)
        if saved:
            print("    [히터 진단 저장] " + " | ".join(saved))
        raise ScenarioAbort(
            "HEATER_ABSTAIN",
            "외부 히터 관측 시간초과({:.0f}s): {}".format(
                float(spec["timeout_sec"]), last_reason), step)

    # --- 한 스텝 -------------------------------------------------------------
    def run_step(self, step: Step) -> None:
        t0 = time.time()
        arrow = "유지" if step.holds else f"-> {step.expect_after}"
        print(f"\n  ── 스텝 {step.no:2d}  [{step.require}] \"{step.target}\" {arrow}")
        if step.note:
            print(f"     {step.note}")

        if step.is_observation:
            self._run_observation(step)
            return

        self._require(step)
        obs, match = self._find(step)
        obs, match = self._await_go(step, obs, match)

        self._render(step, "DRIVING", "go", obs, self.judge.last, match,
                     banner="DRIVING", banner_kind="go")

        # press 는 로봇/AprilTag 가 없으면 ScenarioAbort 를 던진다.
        # 그쪽은 어느 스텝인지 모르므로 여기서 채워준다.
        try:
            ok, desc = self.p.press(obs, match)
        except ScenarioAbort as abort:
            if abort.step is None:
                abort.step = step
            raise

        if not ok:
            raise ScenarioAbort("MOVE_FAIL",
                                "로봇 이동/누름에 실패했습니다. "
                                "12번 화면에서 [e] 로 알람 리셋 후 재시도하세요.", step)

        if hasattr(self.p, "advance"):
            self.p.advance(step.expect_after)

        self._confirm(step)

        # 화면까지 확인했다. 이제 '진짜 눌렸는가'를 색으로 본다.
        # 여기서 GateFailed 가 나면 run() 이 이 스텝을 다시 누른다.
        self._gate(step)

        self.records.append(StepRecord(step, "PASS", desc, time.time() - t0))
        # 수동 게이트로 끝난 스텝은 '검증된 통과'가 아니다. 모델이 색을 달리
        # 칠하도록 그 사실을 그대로 넘긴다.
        self.progress.step_done(step.no)
        self._repaint_progress()
        print(f"    스텝 {step.no} 완료 ({time.time() - t0:.1f}s)")

    # --- 전체 ---------------------------------------------------------------
    def run(self, steps: Sequence[Step]) -> bool:
        self._total = len(steps)
        self.progress.begin(steps, title=f"steps {steps[0].no}-{steps[-1].no}")
        self._repaint_progress()

        print("=" * 72)
        print(f"  시나리오 시작 — 스텝 {steps[0].no}~{steps[-1].no}, "
              f"{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        print("  기권(ABSTAIN)은 통과가 아닙니다. 확신 못 하면 로봇을 세웁니다.")
        n_observation = sum(1 for s in steps if s.is_observation)
        if n_observation:
            print(f"  외부 히터 카메라 관측 {n_observation}개. "
                  "로봇을 움직이지 않고 새 프레임 2회로 확인합니다.")
        n_gate = sum(1 for s in steps if s.gate is not None)
        if not self.gates:
            print("  [주의] 색 게이트를 껐습니다(--no-gates). "
                  "눌렸는지 확인하지 않고 진행합니다.")
        elif n_gate:
            n_manual = sum(1 for s in steps if s.gate is not None and s.gate.is_manual)
            print(f"  색 게이트 {n_gate}개 (자동 {n_gate - n_manual} / "
                  f"수동 확인 {n_manual}). 실패하면 최대 {self.max_retry}회 재실행합니다.")
        if self.view is not None:
            import scenario_overlay as OV
            if self.auto:
                print("  [자동] 준비되면 바로 누릅니다. [p] 로 일시정지할 수 있습니다.")
            else:
                print("  [수동] 준비가 되어도 [g] 를 눌러야 누릅니다.")
            print(f"  키: {OV.HINT_LINE}")
        print("=" * 72)

        for i, step in enumerate(steps):
            self._step_i = i
            self._attempt = 1
            while True:
                attempt_started_at = time.time()
                try:
                    self.run_step(step)
                    break
                except StepSkipped:
                    self.records.append(StepRecord(step, "SKIP", "[s] 로 건너뜀"))
                    self.progress.step_skipped(step.no)
                    self._repaint_progress()
                    print(f"    스텝 {step.no} 건너뜀 — "
                          f"누르지 않았으므로 화면은 그대로입니다")
                    break
                except GateFailed as gf:
                    # 색 게이트 실패는 '아직 안 눌렸다'는 뜻이다. 다시 누른다.
                    # WAIT와 retry_step=False 게이트는 중복 동작 방지를 위해 재시도하지 않는다.
                    if not gf.gate.retries_step:
                        self.records.append(StepRecord(step, "ABORT", gf.detail))
                        self.progress.step_failed(step.no, gf.detail)
                        self.progress.finish(False, gf.detail)
                        self._repaint_progress()
                        print(f"\n  {'!' * 68}")
                        print(f"  [중단] {gf.detail}")
                        print(f"  이 게이트는 재실행 대상이 아닙니다"
                              f"(다시 누르면 동작이 중복됩니다).")
                        print(f"  {'!' * 68}")
                        self._summary()
                        return False
                    if self._attempt > self.max_retry:
                        self.records.append(
                            StepRecord(step, "ABORT",
                                       f"{gf.detail} — {self.max_retry}회 재실행 후 포기"))
                        self.progress.step_failed(step.no, gf.detail)
                        self.progress.finish(False, gf.detail)
                        self._repaint_progress()
                        print(f"\n  {'!' * 68}")
                        print(f"  [중단] {gf.detail}")
                        print(f"  {self.max_retry}회 다시 눌렀지만 상태가 바뀌지 "
                              f"않았습니다. 인터록이나 장비 상태를 확인하세요.")
                        print(f"  {'!' * 68}")
                        self._summary()
                        return False
                    self._attempt += 1
                    self.progress.retry(step.no)
                    self._repaint_progress()
                    print(f"    {gf.detail}")
                    print(f"    -> 스텝 {step.no} 재실행 "
                          f"({self._attempt - 1}/{self.max_retry})")
                    self._flash(f"GATE {gf.gate.gate_id} FAILED - RETRY", "bad", 2.5)
                    continue
                except ScenarioAbort as abort:
                    if (abort.kind == "TIMEOUT"
                            and step.retry_transition_timeout):
                        if self._attempt <= self.max_retry:
                            self._attempt += 1
                            self.progress.retry(step.no)
                            self._repaint_progress()
                            print("    {}".format(abort))
                            print("    -> 화면 이동 버튼 스텝 {} 재준비 "
                                  "({}/{}) — 최신 화면·타겟 확인 후 [g]를 다시 "
                                  "누르세요".format(
                                      step.no, self._attempt - 1,
                                      self.max_retry))
                            self._flash(
                                "TRANSITION TIMEOUT - RETRY", "bad", 2.5)
                            continue
                        abort.message = (
                            "{} — 화면 이동 버튼을 {}회 다시 눌렀지만 전환되지 "
                            "않았습니다".format(
                                abort.message, self.max_retry))
                    self.records.append(StepRecord(
                        step, "ABORT", str(abort),
                        time.time() - attempt_started_at))
                    self.progress.step_failed(step.no, str(abort))
                    self.progress.finish(False, str(abort))
                    self._repaint_progress()
                    print(f"\n  {'!' * 68}")
                    print(f"  [중단] {abort}")
                    print(f"  로봇은 더 움직이지 않습니다. 남은 스텝 "
                          f"{len(steps) - i - 1}개를 건너뜁니다.")
                    print(f"  {'!' * 68}")
                    self._summary()
                    return False

        counts = self.progress.counters()
        self.progress.finish(
            True,
            f"{counts['verified']} verified, {counts['manual']} by eye, "
            f"{counts['skipped']} skipped, {counts['retries']} gate retry(s)")
        self._repaint_progress()
        print("\n  모든 스텝을 완료했습니다.")
        self._summary()
        return True

    def _summary(self) -> None:
        marks = {"PASS": "O", "ABORT": "X", "SKIP": "~"}
        print("\n" + "=" * 72)
        print("  요약")
        print("=" * 72)
        block = None
        for r in self.records:
            if r.step.block and r.step.block != block:
                block = r.step.block
                print(f"  ── {block}")
            gate = f"  gate {r.step.gate.gate_id}" if r.step.gate else ""
            print(f"  {marks.get(r.status, '?')} 스텝 {r.step.no:2d} "
                  f"[{r.step.require:10s}] \"{r.step.target}\"  {r.status}  "
                  f"({r.elapsed_sec:.1f}s){gate}")
            if r.status == "ABORT":
                for line in r.detail.splitlines():
                    print(f"       {line}")


# ----------------------------------------------------------------------------
# --check : 로그로 스텝별 타겟 매칭 가능 여부만 점검
# ----------------------------------------------------------------------------

def check_targets(log_path: str = DEFAULT_LOG) -> int:
    """실장비를 켜기 전에 '이 시나리오가 애초에 성립하는가'를 본다.

    화면 판별과 타겟 완전일치를 로그 스냅샷으로 각각 확인한다.
    반환값은 실패 스텝 수.
    """
    snaps = SIG.load_snapshots(log_path)
    judge = ScreenJudge(log_path)

    print("=" * 78)
    print("  [1] 화면 판별 — 각 화면 스냅샷이 제대로 판별되는가")
    print("=" * 78)
    screens = sorted({s.require for s in STEPS} | {s.expect_after for s in STEPS})
    for scr in screens:
        sid = _snapshot_of(scr)
        if sid is None or sid not in snaps:
            print(f"  ~ {scr:12s} 로그에 스냅샷 없음")
            continue
        r = judge.idf.identify(snaps[sid])
        mark = "O" if r.screen == scr else ("-" if not r.decided else "X")
        print(f"  {mark} {scr:12s} 판정={str(r.screen):12s} {r.reason}")

    print("\n" + "=" * 78)
    print("  [2] 타겟 완전일치 — 각 스텝의 질의가 그 화면에서 찾아지는가")
    print("=" * 78)
    fails = 0
    for step in STEPS:
        if step.is_observation:
            spec = HEATER_OBSERVATIONS[step.observation_id]
            print(f"  - 스텝 {step.no:2d} [{step.require:10s}] 카메라 관측 "
                  f"{step.observation_id}: {spec['requirements']}")
            continue
        sid = _snapshot_of(step.require)
        dets = snaps.get(sid, []) if sid else []
        buttons = [{"text": d.text, "label": d.label,
                    "ocr_confidence": d.ocr_conf, "confidence": d.conf,
                    "bbox": [d.x, d.y, d.x, d.y]} for d in dets]
        m = find_target(buttons, step.target)
        if m is None:
            if (step.require == "Main"
                    and normalize_query(step.target) == "position"):
                print(f"  - 스텝 {step.no:2d} [{step.require:10s}] "
                      f"\"{step.target}\" — 과거 로그는 detector 누락; "
                      "blue-white 두 ROI exact 계약은 --no-robot에서 확인")
                continue
            fails += 1
            print(f"  X 스텝 {step.no:2d} [{step.require:10s}] \"{step.target}\"")
            print(f"       매칭 실패 — 가까운 검출: {describe_near(buttons, step.target)}")
        else:
            kind = "label" if field_matches(m.get("label"), step.target) else "text"
            print(f"  O 스텝 {step.no:2d} [{step.require:10s}] \"{step.target}\""
                  f"  <- {kind} \"{m.get('text')}\" @({m['bbox'][0]:.3f},{m['bbox'][1]:.3f})")

    press_count = sum(1 for step in STEPS if not step.is_observation)
    print(f"\n  -> 매칭 실패 {fails} / {press_count} 누름 스텝 "
          f"(카메라 관측 {len(STEPS) - press_count}개는 별도 계약 검사)")
    if fails:
        print("     실패한 스텝은 실장비에서도 [매칭 실패]로 중단됩니다.")
        print("     OCR 판독을 고치거나 타겟 문자열을 실측값에 맞춰야 합니다.")

    # --- [3] 색 게이트 -------------------------------------------------------
    print("\n" + "=" * 78)
    print("  [3] 색 게이트 — 게이트가 볼 버튼이 로그에서 어떤 색으로 찍혀 있는가")
    print("=" * 78)
    print("  로그는 '누르기 전' 상태로 찍힌 경우가 많다. 여기서 X 가 나오는 것 자체는")
    print("  문제가 아니고, 색이 읽히는지·채도 기준이 맞는지를 보는 것이다.\n")

    colors = _load_log_colors(log_path)
    for step in STEPS:
        g = step.gate
        if g is None:
            continue
        if g.is_manual:
            print(f"  - 스텝 {step.no:2d} Gate {g.gate_id} [수동] {g.prompt}")
            continue
        sid = _snapshot_of(g.screen)
        if sid is None:
            print(f"  ~ 스텝 {step.no:2d} Gate {g.gate_id} — "
                  f"'{g.screen}' 스냅샷이 로그에 없음")
            continue
        q = g.target.strip().lower()
        hit = next((v for (s, t, l), v in colors.items()
                    if s == sid and q in (t, l)), None)
        if hit is None:
            if (g.screen == "Main"
                    and normalize_query(g.target) == "position"):
                print(f"  - 스텝 {step.no:2d} Gate {g.gate_id} — 과거 로그에는 "
                      "Position exact bbox가 없어 색 검증 불가; --no-robot 필요")
                continue
            print(f"  X 스텝 {step.no:2d} Gate {g.gate_id} — "
                  f"'{g.target}' 가 {g.screen} 스냅샷에 없음 (게이트가 영영 안 열림)")
            fails += 1
            continue
        ok, why = g.spec.matches(hit)
        want = "소멸" if g.kind == CG.GATE_VANISH else "검출"
        print(f"  {'O' if ok else '.'} 스텝 {step.no:2d} Gate {g.gate_id} "
              f"[{want}] {g.screen} \"{g.target}\" -> {why}")

    # --- [4] 실측 필요 목록 ---------------------------------------------------
    todo = [s for s in STEPS if s.unverified]
    if todo:
        print("\n" + "=" * 78)
        print("  [4] 실장비에서 확인해야 하는 타겟 문자열")
        print("=" * 78)
        print("  아래 문자열은 20260727 로그에 없어서 검증이 불가능했다.")
        print("  find_button_match_in_results 는 퍼지 매칭을 하지 않으므로")
        print("  (정규화 후 완전일치), OCR 실측값으로 STEPS 를 고쳐야 한다.\n")
        for s in todo:
            print(f"  ! 스텝 {s.no:2d} [{s.require:10s}] \"{s.target}\"  — {s.note}")

    return fails


# ----------------------------------------------------------------------------
# CLI
# ----------------------------------------------------------------------------

def select_steps(first: Optional[int], last: Optional[int]) -> List[Step]:
    lo = first if first is not None else STEPS[0].no
    hi = last if last is not None else STEPS[-1].no
    out = [s for s in STEPS if lo <= s.no <= hi]
    if not out:
        raise SystemExit(f"스텝 {lo}~{hi} 범위에 해당하는 단계가 없습니다.")
    return out


def _want_progress_window(args) -> bool:
    """진행 창을 띄울 것인가. 한 군데서만 정한다.

    --no-view 는 "창을 띄우지 마라"는 뜻이다. 카메라 창만 끄고 다른 창을 대신
    띄우면 그 말을 어기는 것이고, SSH 로 붙어 --no-view 로 돌리던 사람에게는
    없던 창이 생긴다. 그래서 --no-view 면 이 창도 끈다 — 그래도 이것만 보고
    싶으면 --progress 로 명시한다.

    --dry-run 은 원래 창 없는 점검 수단이므로 같은 규칙을 따른다.
    """
    if getattr(args, "no_progress", False):
        return False
    if getattr(args, "progress", False):
        return True
    if getattr(args, "dry_run", False) or getattr(args, "no_view", False):
        return False
    return True


def _make_progress_view(args):
    """진행 창을 연다. 열 수 없는 환경이면 None 을 준다.

    디스플레이 확인은 ProgressView 가 cv2 를 건드리기 전에 한다. 없는 상태로
    창을 만들면 Qt 가 abort() 를 불러 프로세스가 통째로 죽고, 그건 여기서
    try/except 로 못 막는다.
    """
    if not _want_progress_window(args):
        return None
    try:
        import scenario_progress_view as PV
    except Exception as exc:  # cv2 가 없는 환경 등
        print(f"  [알림] 진행 창을 쓸 수 없습니다: {exc}")
        return None
    view = PV.ProgressView(save_dir=getattr(args, "save_dir", None))
    return view if view.enabled else None


def _hold_progress(progress_view, runner) -> None:
    """실행이 끝난 뒤 마지막 그림을 잠깐 남긴다.

    안 그러면 창이 결과를 띄우자마자 닫혀서, 무엇으로 끝났는지 볼 시간이 없다.
    터미널 요약은 그대로 나오므로 여기서 기다리는 건 눈으로 볼 시간뿐이다.
    """
    import cv2

    progress_view.render(runner.progress)
    print("\n  진행 창을 닫으려면 그 창에서 아무 키나 누르세요 (10초 후 자동).")
    try:
        cv2.waitKey(10000)
    except Exception:
        pass


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description="화면 인식 게이팅 기반 FR5 시나리오 러너 (스텝 32~35)")
    ap.add_argument("--log", default=DEFAULT_LOG, help="label_snapshots.csv 경로")
    ap.add_argument("--check", action="store_true",
                    help="로그로 화면 판별·타겟 매칭 가능 여부만 점검 (로봇/카메라 불필요)")
    ap.add_argument("--dry-run", action="store_true",
                    help="로그를 재생해 시나리오 흐름을 모의 실행 (로봇/카메라 불필요)")
    ap.add_argument("--no-robot", dest="no_robot", action="store_true",
                    help="실카메라·실비전으로 돌리되 로봇은 연결하지 않는다. "
                         "누를 차례가 되면 사람이 손으로 눌러 화면을 넘긴다 (로봇 불필요)")
    ap.add_argument("--camera", type=int, default=0, help="카메라 인덱스")
    ap.add_argument("--from", dest="first", type=int, default=None, help="시작 스텝 번호")
    ap.add_argument("--to", dest="last", type=int, default=None, help="끝 스텝 번호")
    ap.add_argument("--match-timeout", type=float, default=MATCH_TIMEOUT_SEC,
                    help="타겟 완전일치 재시도 시간(초). 0 이면 즉시 실패")
    ap.add_argument("--startup-timeout", type=float,
                    default=STARTUP_REQUIRE_TIMEOUT_SEC,
                    help="첫 스텝의 시작 화면을 사람이 준비할 대기 시간(초). "
                         "이 동안 로봇은 연결하지 않음")
    ap.add_argument("--no-quad", action="store_true", help="4점 워핑 끄기")
    ap.add_argument("--no-full-frame-ocr", dest="no_full_frame_ocr",
                    action="store_true",
                    help="4K 전체 프레임 보조 텍스트 검출을 끈다. 화면 판별과 "
                         "LayoutLM은 켜든 끄든 화면 crop 경로만 쓴다")
    ap.add_argument("--yes", action="store_true", help="실행 전 확인 프롬프트 생략")
    ap.add_argument("--auto", action="store_true",
                    help="[g] 승인 없이 준비되면 바로 누른다 (예전 동작). "
                         "창은 계속 뜨고 [p] 로 일시정지할 수 있다")
    ap.add_argument("--no-view", dest="no_view", action="store_true",
                    help="카메라 표시 창을 띄우지 않는다. 승인은 터미널에서 g + Enter")
    ap.add_argument("--no-progress", dest="no_progress", action="store_true",
                    help="진행 현황 창(FR5 Scenario Progress)을 띄우지 않는다. "
                         "전체 스텝 중 지금 몇 번째인지를 보여주는 창이다")
    ap.add_argument("--progress", dest="progress", action="store_true",
                    help="진행 현황 창을 명시적으로 켠다. --dry-run 이나 "
                         "--no-view 처럼 창이 안 뜨는 실행에서 이것만 띄울 때 쓴다")
    ap.add_argument("--no-gates", dest="no_gates", action="store_true",
                    help="색 검증 게이트를 끈다. 화면 판별만으로 진행한다. "
                         "타겟 문자열을 맞추는 단계에서만 쓸 것 — "
                         "'눌렸는지'를 확인하지 않으므로 데모 본 실행에는 쓰지 않는다")
    ap.add_argument("--max-retry", type=int, default=MAX_STEP_RETRY,
                    help="색 게이트 실패 시 같은 스텝을 다시 누를 최대 횟수")
    ap.add_argument("--save-dir", default=os.path.join(HERE, "scenario_captures"),
                    help="[c] 로 저장할 위치")
    ap.add_argument("--press-standoff", dest="press_standoff", type=float,
                    default=PRESS_X_TEST_STANDOFF_MM,
                    help="계산된 화면 표면에서 몇 mm 앞까지만 갈지(기본 %(default)s). "
                         "0 이면 표면까지 실제로 누른다. Y/Z는 언제나 계산값 그대로다.")
    a = ap.parse_args(argv)

    # 시험 간격은 모듈 상수로 읽는다(press() 가 같은 값을 본다).
    if a.press_standoff is not None:
        if a.press_standoff < 0.0:
            ap.error("--press-standoff 는 0 이상이어야 합니다 — 음수는 계산된 화면 "
                     "표면보다 안쪽을 누르라는 뜻이고, 그건 유리를 밀어 넣습니다.")
        globals()["PRESS_X_TEST_STANDOFF_MM"] = float(a.press_standoff)

    if a.check:
        return 1 if check_targets(a.log) else 0

    steps = select_steps(a.first, a.last)
    judge = ScreenJudge(a.log)

    if a.dry_run:
        # 로그 재생은 사람이 볼 화면도, 누를 로봇도 없다.
        # auto=True 로 두지 않으면 승인 단계에서 입력을 기다리며 멈춘다.
        p = ReplayPerception(a.log, start_screen=steps[0].require)
        # 진행 창은 카메라를 안 쓰므로 재생에서도 뜬다. 명시적으로 요청할 때만
        # 연다 — 기존 --dry-run 은 창 없는 점검 수단이고, 그 성질을 바꾸지 않는다.
        progress_view = _make_progress_view(a)
        try:
            # 재생은 '누른 뒤 색이 바뀌는' 것을 재현하지 못한다(로그의 색은 고정이다).
            # 게이트를 켜두면 전부 실패하므로 흐름 검증에서는 끈다.
            runner = ScenarioRunner(p, judge, match_timeout_sec=0.0, auto=True,
                                    gates=False, progress_view=progress_view)
            print("  [모의 실행] 로그 재생 — 로봇과 카메라를 쓰지 않습니다.\n")
            ok = runner.run(steps)
            if progress_view is not None:
                _hold_progress(progress_view, runner)
            return 0 if ok else 1
        finally:
            if progress_view is not None:
                progress_view.close()

    standoff = PRESS_X_TEST_STANDOFF_MM
    if standoff:
        print("\n  [시험 간격] 계산된 화면 표면에서 {:.0f}mm 앞까지만 갑니다 "
              "(Y/Z는 계산값 그대로). 실제로 누르려면 --press-standoff 0."
              .format(float(standoff)))
    else:
        print("\n  [시험 간격] 없음 — 계산된 화면 표면까지 실제로 누릅니다.")

    # --- 실카메라 : 로봇 없이 -------------------------------------------------
    if a.no_robot:
        print("\n  [로봇 없이 실행] 카메라·YOLO·OCR·화면 판별은 전부 실제로 돌립니다.")
        print("  로봇에는 접속하지 않습니다. 누를 차례가 되면 좌표만 계산해서 보여주고,")
        print("  화면을 넘기는 것은 사람이 손으로 해주셔야 합니다.")
        print(f"  스텝 {steps[0].no}~{steps[-1].no}, 카메라 {a.camera}")
    else:
        # --- 실장비 -----------------------------------------------------------
        print("\n  [실장비 실행] 로봇이 실제로 움직입니다.")
        print("  [좌표] 실장비는 유효한 태그 등록이 필요합니다. 없거나 무효하면 "
              "로봇 연결 전에 중단합니다.")
65        print(f"  스텝 {steps[0].no}~{steps[-1].no}, 카메라 {a.camera}")
        print("  로봇 접속은 첫 누름 직전에 이루어집니다(지연 연결).")
        print("  먼저 --check 와 --no-robot 으로 점검하는 것을 권합니다.")
        if not a.yes:
            try:
                if input("  진행하시겠습니까? [y/N] ").strip().lower() not in ("y", "yes"):
                    print("  취소했습니다.")
                    return 0
            except (EOFError, KeyboardInterrupt):
                print("\n  취소했습니다.")
                return 0

    p = None
    view = None
    progress_view = None
    try:
        p = LivePerception(camera_index=a.camera, use_quad=not a.no_quad,
                           allow_robot=not a.no_robot,
                           full_frame_ocr=not a.no_full_frame_ocr)

        if not a.no_view:
            import scenario_overlay as OV
            view = OV.LiveView(
                title="FR5 Scenario - Screen ROI", show_raw=True,
                save_dir=a.save_dir,
                target_offset_limit_mm=getattr(
                    p.IMPL, "APRILTAG_TARGET_OFFSET_LIMIT_MM", 100),
                initial_target_offsets=p.initial_target_offsets)
            if view.enabled:
                p.set_target_offset_provider(view.target_offsets)
                print("  [좌표 보정] Target Offset 창의 X/Y/Z 드래그바가 "
                      "[g] 누름 직전 최신 타겟에 적용됩니다.")
            else:
                view = None
        progress_view = _make_progress_view(a)

        # 수동 승인은 사람이 [g] 를 누를 때까지 기다린다. 타겟 재시도 시간이 짧으면
        # 승인 단계에 닿기도 전에 NO_MATCH 로 죽는다. 넉넉히 준다.
        match_timeout = a.match_timeout
        if not a.auto and match_timeout < HAND_MATCH_TIMEOUT_SEC:
            match_timeout = HAND_MATCH_TIMEOUT_SEC

        kw: Dict[str, Any] = dict(
            match_timeout_sec=match_timeout, view=view, auto=a.auto,
            gates=not a.no_gates, max_retry=a.max_retry,
            startup_timeout_sec=a.startup_timeout,
            progress_view=progress_view)
        if a.no_robot:
            # 사람이 손으로 화면을 넘겨야 하므로 대기 시간을 늘린다
            kw.update(require_timeout_sec=HAND_REQUIRE_TIMEOUT_SEC,
                      transition_timeout_sec=HAND_TRANSITION_TIMEOUT_SEC,
                      hold_confirm_sec=HAND_HOLD_CONFIRM_SEC)
        runner = ScenarioRunner(p, judge, **kw)
        ok = runner.run(steps)
        if progress_view is not None:
            _hold_progress(progress_view, runner)
        return 0 if ok else 1
    except KeyboardInterrupt:
        print("\n  [중단] 사용자가 중지했습니다.")
        return 130
    except Exception as exc:
        print(f"\n  [오류] {type(exc).__name__}: {exc}")
        return 1
    finally:
        if view is not None:
            view.close()
        if progress_view is not None:
            progress_view.close()
        if p is not None:
            p.close()


if __name__ == "__main__":
    raise SystemExit(main())
