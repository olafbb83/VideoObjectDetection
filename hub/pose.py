"""
Етап 5 — поза: 17 ключових точок скелета.

YOLO11n-pose видає І рамки людей, І ключові точки за ОДИН прохід — вона сама
є детектором людей. Тому pose не додається другою моделлю до детектора,
а підмінює його: інакше ми двічі шукали б тих самих людей.

Ціна: pose-модель знає лише клас `person`. Решта 79 класів COCO у цьому
режимі недоступні.

Розкладка COCO-17 (порядок зафіксований, на нього спирається весь код нижче):

    0 ніс          5 ліве плече    9  ліве зап'ястя   13 ліве коліно
    1 ліве око     6 праве плече   10 праве зап'ястя  14 праве коліно
    2 праве око    7 лівий лікоть  11 ліве стегно     15 ліва щиколотка
    3 ліве вухо    8 правий лікоть 12 праве стегно    16 права щиколотка
    4 праве вухо

Кожна точка має свою впевненість. Це важливо: модель завжди повертає всі 17
координат, навіть для того, чого не бачить — затулені ноги отримають
випадкові координати з низькою впевненістю. Малювати й рахувати можна тільки
точки вище порогу, інакше скелет буде смикатись у випадкові боки.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import cv2
import numpy as np

NOSE = 0
L_SHOULDER, R_SHOULDER = 5, 6
L_HIP, R_HIP = 11, 12
L_KNEE, R_KNEE = 13, 14
L_ANKLE, R_ANKLE = 15, 16

KEYPOINT_NAMES = [
    "ніс", "ліве око", "праве око", "ліве вухо", "праве вухо",
    "ліве плече", "праве плече", "лівий лікоть", "правий лікоть",
    "ліве зап'ястя", "праве зап'ястя", "ліве стегно", "праве стегно",
    "ліве коліно", "праве коліно", "ліва щиколотка", "права щиколотка",
]

# Ребра скелета: пари індексів, які з'єднуємо лініями
SKELETON = [
    (15, 13), (13, 11), (16, 14), (14, 12), (11, 12),   # ноги й таз
    (5, 11), (6, 12), (5, 6),                            # тулуб
    (5, 7), (7, 9), (6, 8), (8, 10),                     # руки
    (0, 1), (0, 2), (1, 3), (2, 4),                      # голова
]

# Нижче цього порогу точку вважаємо невидимою
KP_CONF = 0.5

COLOR_LIMB = (90, 200, 255)
COLOR_TORSO = (120, 255, 160)
COLOR_POINT = (40, 40, 40)


@dataclass
class PoseFeatures:
    """
    Величини, з яких будуються правила поведінки.

    Усе в пікселях і відносних одиницях: без калібрування камери перевести
    в метри не можна, а для правил «стоїть / лежить» це й не потрібно.
    """

    visible: int                    # скільки з 17 точок видно
    torso_angle_deg: float | None   # 0 = вертикально, 90 = горизонтально
    aspect_ratio: float             # ширина рамки / висота
    center_y_rel: float | None      # висота центру мас, 0 = верх кадру, 1 = низ
    height_px: float                # висота рамки

    def as_dict(self) -> dict:
        return {
            "visible": self.visible,
            "torso_angle_deg": (round(self.torso_angle_deg, 1)
                                if self.torso_angle_deg is not None else None),
            "aspect_ratio": round(self.aspect_ratio, 2),
            "center_y_rel": (round(self.center_y_rel, 3)
                             if self.center_y_rel is not None else None),
        }


def _mid(kp, conf, a: int, b: int):
    """Середина між двома точками, якщо обидві видно."""
    if conf[a] < KP_CONF or conf[b] < KP_CONF:
        return None
    return (kp[a][0] + kp[b][0]) / 2, (kp[a][1] + kp[b][1]) / 2


def features(kp: np.ndarray, conf: np.ndarray, box_xyxy, frame_h: int) -> PoseFeatures:
    """
    kp   — (17, 2) координати в пікселях кадру
    conf — (17,) впевненість кожної точки
    """
    x1, y1, x2, y2 = (float(v) for v in box_xyxy)
    w, h = max(1.0, x2 - x1), max(1.0, y2 - y1)

    shoulders = _mid(kp, conf, L_SHOULDER, R_SHOULDER)
    hips = _mid(kp, conf, L_HIP, R_HIP)

    angle = None
    if shoulders and hips:
        dx = hips[0] - shoulders[0]
        dy = hips[1] - shoulders[1]
        # Кут вісі тулуба до ВЕРТИКАЛІ: 0 = стоїть, 90 = лежить.
        # atan2(|dx|, |dy|) саме так і рахує — від вертикальної осі,
        # а не від горизонтальної, як звичний atan2(dy, dx).
        angle = math.degrees(math.atan2(abs(dx), abs(dy)))

    center_y = None
    if hips:
        center_y = hips[1] / max(1, frame_h)
    elif shoulders:
        center_y = shoulders[1] / max(1, frame_h)

    return PoseFeatures(
        visible=int((conf >= KP_CONF).sum()),
        torso_angle_deg=angle,
        aspect_ratio=w / h,
        center_y_rel=center_y,
        height_px=h,
    )


def draw_pose(frame, keypoints, box_color=None) -> None:
    """
    Малює скелети. keypoints — results[0].keypoints від ultralytics.

    Точки з низькою впевненістю пропускаємо: модель повертає всі 17 координат
    завжди, навіть для затулених частин тіла, і без фільтра скелет смикається
    у випадкові боки.
    """
    if keypoints is None or keypoints.xy is None:
        return

    xy_all = keypoints.xy.cpu().numpy() if hasattr(keypoints.xy, "cpu") else np.asarray(keypoints.xy)
    conf_all = keypoints.conf
    if conf_all is None:
        return
    conf_all = conf_all.cpu().numpy() if hasattr(conf_all, "cpu") else np.asarray(conf_all)

    for kp, conf in zip(xy_all, conf_all):
        for a, b in SKELETON:
            if conf[a] < KP_CONF or conf[b] < KP_CONF:
                continue
            pa = (int(kp[a][0]), int(kp[a][1]))
            pb = (int(kp[b][0]), int(kp[b][1]))
            # тулуб виділяємо кольором: саме за ним рахується нахил
            torso = (a, b) in ((5, 6), (5, 11), (6, 12), (11, 12))
            cv2.line(frame, pa, pb, COLOR_TORSO if torso else COLOR_LIMB,
                     2, cv2.LINE_AA)

        for i, (x, y) in enumerate(kp):
            if conf[i] < KP_CONF:
                continue
            cv2.circle(frame, (int(x), int(y)), 3, (255, 255, 255), -1, cv2.LINE_AA)
            cv2.circle(frame, (int(x), int(y)), 3, COLOR_POINT, 1, cv2.LINE_AA)


def pose_features_for(results, frame_h: int) -> dict[int, PoseFeatures]:
    """
    Рахує ознаки для кожного треку в кадрі. Ключ — track_id.
    Треки без ID пропускаються: правила поведінки без них безглузді.
    """
    out: dict[int, PoseFeatures] = {}
    kps = results.keypoints
    boxes = results.boxes
    if kps is None or boxes is None or kps.xy is None or kps.conf is None:
        return out

    xy_all = kps.xy.cpu().numpy() if hasattr(kps.xy, "cpu") else np.asarray(kps.xy)
    conf_all = kps.conf.cpu().numpy() if hasattr(kps.conf, "cpu") else np.asarray(kps.conf)

    for i, box in enumerate(boxes):
        if box.id is None or i >= len(xy_all):
            continue
        out[int(box.id[0])] = features(xy_all[i], conf_all[i],
                                       [float(v) for v in box.xyxy[0]], frame_h)
    return out
