import os

import numpy as np


def load_legacy_mediapipe():
    os.environ.setdefault('MPLCONFIGDIR', '/tmp/matplotlib')
    os.makedirs(os.environ['MPLCONFIGDIR'], exist_ok=True)

    try:
        import mediapipe as mp
    except ImportError:
        return None

    if not hasattr(mp, 'solutions') or not hasattr(mp.solutions, 'pose'):
        return None
    return mp


LEGACY_MEDIAPIPE = load_legacy_mediapipe()

import cv2


class PoseDetector:
    def __init__(self):
        mp = LEGACY_MEDIAPIPE
        if mp is None:
            print(
                "MediaPipe legacy pose API is unavailable. "
                "Install a mediapipe version that provides mp.solutions "
                "to enable arm pose tracking.",
                flush=True
            )
            self.enabled = False
            self.results = None
            self.lm_list = []
            return

        self.enabled = True
        self.mpPose = mp.solutions.pose
        self.pose = self.mpPose.Pose(
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5
        )
        self.mpDraw = mp.solutions.drawing_utils
        self.results = None
        self.lm_list = []

    def findPose(self, frame, draw=True):
        if not self.enabled:
            return frame

        imgRGB = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        self.results = self.pose.process(imgRGB)

        if self.results.pose_landmarks and draw:
            self.mpDraw.draw_landmarks(
                frame,
                self.results.pose_landmarks,
                self.mpPose.POSE_CONNECTIONS
            )
        return frame

    def findPosition(self, frame):
        self.lm_list = []

        if self.enabled and self.results and self.results.pose_landmarks:
            h, w, _ = frame.shape
            for idx, lm in enumerate(self.results.pose_landmarks.landmark):
                cx, cy = int(lm.x * w), int(lm.y * h)
                self.lm_list.append([idx, cx, cy, lm.visibility])

        return self.lm_list

    def getLandmark(self, idx, min_visibility=0.45):
        if len(self.lm_list) <= idx:
            return None

        _, x, y, visibility = self.lm_list[idx]
        if visibility < min_visibility:
            return None
        return np.array([x, y], dtype=float)

    def getArmPoints(self, side):
        if side == 'left':
            indexes = (11, 13, 15)
        else:
            indexes = (12, 14, 16)

        points = [self.getLandmark(idx) for idx in indexes]
        if any(point is None for point in points):
            return None
        return points

    def findAngle(self, p1, p2, p3):
        p1 = self.getLandmark(p1)
        p2 = self.getLandmark(p2)
        p3 = self.getLandmark(p3)
        if p1 is None or p2 is None or p3 is None:
            return 0.0

        v1 = p1 - p2
        v2 = p3 - p2
        denom = np.linalg.norm(v1) * np.linalg.norm(v2)
        if denom < 1e-6:
            return 0.0

        cos_angle = np.clip(np.dot(v1, v2) / denom, -1.0, 1.0)
        return np.degrees(np.arccos(cos_angle))
