# 빨간 목표물 검출의 순수 계산 부분입니다. ROS도 카메라도 참조하지 않습니다.
# 색 마스크 → 덩어리 선택 → 광학 프레임 단위 방향까지를 함수로 나눠
# 하드웨어 없이 정지 이미지로 검증할 수 있게 했습니다.
#
# 깊이는 쓰지 않습니다. 카메라는 "목표물이 어느 쪽에 있는가"만 답하고,
# 거리 판단은 그리퍼 PSD가 맡습니다.
#
# [공부 노트] 처리 3단계
#   컬러 사진 ─red_mask─▶ 흑백 마스크(빨강=흰색) ─largest_blob─▶ 중심 픽셀(u, v) ─bearing─▶ 방향 벡터
#   u = 가로 픽셀 위치(왼→오), v = 세로 픽셀 위치(위→아래). 사진 왼쪽 위가 (0, 0).

"""Pure image maths for the red cube detector; no ROS, no camera, no depth."""
import cv2          # OpenCV: 영상 처리 라이브러리 (apt: python3-opencv)
import numpy as np  # 배열 계산. OpenCV 이미지는 numpy 배열(높이 x 너비 x 3)입니다


# ─────────────────────────────────────────────────────────────────────────────
# red_mask: 빨간 픽셀만 흰색(255), 나머지는 검정(0)인 마스크 만들기
#   필요한 입력
#     bgr        : 컬러 이미지. OpenCV 는 RGB 가 아니라 BGR(파랑-초록-빨강) 순서
#     low1~high2 : HSV 범위 두 쌍 [H, S, V] — target.yaml 에서 조정
#   왜 HSV? RGB 는 밝기가 바뀌면 값이 다 같이 변하지만, HSV 는
#     H(색상 0~179), S(채도: 얼마나 진한 색인지), V(밝기) 로 나뉘어 "빨강인지"를 H 로만 판단하기 쉬움
#   왜 범위가 두 개? H 는 원(빙 도는 값)이라 빨강이 0 근처와 179 근처 양쪽에 걸쳐 있음
# ─────────────────────────────────────────────────────────────────────────────
def red_mask(bgr, low1, high1, low2, high2):
    """Binary mask of red pixels. Hue wraps at 0, so red needs two ranges."""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    # inRange: 범위 안 픽셀 → 255. 두 마스크를 | (OR) 로 합침. np.uint8 = 0~255 정수 자료형
    # 줄 끝의 \ 는 "다음 줄에 이어짐"
    return cv2.inRange(hsv, np.array(low1, np.uint8), np.array(high1, np.uint8)) | \
        cv2.inRange(hsv, np.array(low2, np.uint8), np.array(high2, np.uint8))


# ─────────────────────────────────────────────────────────────────────────────
# largest_blob: 마스크의 흰 덩어리 중 "정육면체처럼 생긴 가장 큰 것"의 중심 (u, v)
#   필요한 입력 (target.yaml)
#     min_area_px : 이보다 작은 덩어리는 잡음 (픽셀 수)
#     max_aspect  : 가로세로 비 한계. 2.0 이면 2배 넘게 길쭉한 건 탈락
#     min_fill    : 둘레 사각형 중 실제로 채워진 비율. 너무 비어 있으면(0.5 미만) 반사·잡동사니
#   돌려주는 값: (u, v) 또는 None(없음)
# ─────────────────────────────────────────────────────────────────────────────
def largest_blob(mask, min_area_px, max_aspect, min_fill):
    """Return (u, v) of the biggest cube-shaped red region, else None.

    A 50 mm cube is convex and roughly square in the image, so a low fill ratio
    or a long thin box is reflection or clutter, not the target.
    """
    # findContours: 흰 덩어리들의 외곽선 목록. RETR_EXTERNAL = 바깥 외곽선만
    # [-2] : OpenCV 버전마다 반환 개수가 달라서(2개/3개) 뒤에서 두 번째를 집으면 항상 외곽선 목록
    contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    best = None  # (면적, 외곽선) — 지금까지 가장 좋은 후보
    for contour in contours:
        area = cv2.contourArea(contour)
        if area < min_area_px:
            continue  # 너무 작음 → 다음 후보
        # 외곽선을 감싸는 똑바른 사각형 (왼쪽 위 x, y, 너비 w, 높이 h)
        x, y, w, h = cv2.boundingRect(contour)
        if not h or not w:
            continue  # 0 으로 나누기 방지
        aspect = max(w / h, h / w)  # 1 이면 정사각형, 클수록 길쭉
        if aspect > max_aspect or area / float(w * h) < min_fill:
            continue
        if best is None or area > best[0]:
            best = (area, contour)
    if best is None:
        return None
    # 모멘트: 덩어리의 "무게중심" 계산용 값. m00 = 면적, m10/m00 = 중심 x, m01/m00 = 중심 y
    moments = cv2.moments(best[1])
    if moments['m00'] <= 0:
        return None
    return moments['m10'] / moments['m00'], moments['m01'] / moments['m00']


# ─────────────────────────────────────────────────────────────────────────────
# bearing: 픽셀 (u, v) → 카메라에서 그 픽셀 쪽을 가리키는 길이 1 방향 벡터
#   필요한 입력 (카메라 내부 파라미터 — /camera/color/camera_info 가 알려줌)
#     fx, fy : 초점거리(픽셀 단위)   cx, cy : 사진 중심 픽셀
#   핀홀 카메라 모델: 픽셀 (u, v) 는 방향 ((u-cx)/fx, (v-cy)/fy, 1) 에서 온 빛
#   광학 프레임 규칙: z 앞, x 오른쪽, y 아래  → x > 0 이면 목표가 화면 오른쪽
# ─────────────────────────────────────────────────────────────────────────────
def bearing(u, v, fx, fy, cx, cy):
    """Unit direction to a pixel in the camera OPTICAL frame (REP-145: z forward,
    x right, y down).

    거리를 모르므로 위치가 아니라 방향만 돌려줍니다. 길이가 1이라는 것 외에
    어떤 거리도 주장하지 않습니다. 세로 중심선 정렬은 x 성분만 보면 됩니다.
    """
    if fx <= 0 or fy <= 0:
        raise ValueError('camera_info intrinsics must be positive')
    x, y, z = (u - cx) / fx, (v - cy) / fy, 1.0
    # 벡터 길이로 나눠 길이 1(단위 벡터)로 만듦. ** 0.5 = 제곱근
    norm = (x * x + y * y + z * z) ** 0.5
    return x / norm, y / norm, z / norm
