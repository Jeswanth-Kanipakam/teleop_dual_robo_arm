import mediapipe as mp
import numpy as np
import cv2


class poseDetector():
    def __init__(self):

        self.mpPose = mp.solutions.pose
        self.pose = self.mpPose.Pose()
        self.mpDraw = mp.solutions.drawing_utils

    def findPose(self, frame, draw=True):
        imgRGB = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        self.results = self.pose.process(imgRGB)

        if self.results.pose_landmarks and draw:
            self.mpDraw.draw_landmarks(
                frame, self.results.pose_landmarks, self.mpPose.POSE_CONNECTIONS)
        return frame

    def findPosePosition(self, frame):
        lm_list = []
        if self.results.pose_landmarks:
            h, w, _ = frame.shape

            for id, lm in enumerate(self.results.pose_landmarks.landmark):
                cx, cy = int(lm.x * w), int(lm.y * h)
                lm_list.append([id, cx, cy])

        self.lm_list = lm_list
        return lm_list

    def findAngle(self, frame, p1, p2, p3):
        x1, y1 = self.lm_list[p1][1:]
        x2, y2 = self.lm_list[p2][1:]
        x3, y3 = self.lm_list[p3][1:]

        v1 = np.array([x1 - x2, y1 - y2])
        v2 = np.array([x3 - x2, y3 - y2])

        angle = np.degrees(
            np.arccos(np.dot(v1, v2) /
                      (np.linalg.norm(v1) * np.linalg.norm(v2)))
        )
        return angle