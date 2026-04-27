import mediapipe as mp
import matplotlib.pyplot as plt
import numpy as np
import cv2

class HandDetector():
    def __init__(self, mode=False, maxHands=2, modCompl=1, detCon=0.5, trackCon=0.5):

        self.mpHands = mp.solutions.hands
        self.hands = self.mpHands.Hands(
            static_image_mode=mode,
            max_num_hands=maxHands,
            model_complexity=modCompl,
            min_detection_confidence=detCon,
            min_tracking_confidence=trackCon
        )
        self.mpDraw = mp.solutions.drawing_utils

    def findHands(self, frame, draw=True, return_handedness=False):
        imgRGB = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        self.results = self.hands.process(imgRGB)

        if self.results.multi_hand_landmarks:
            for handLMs in self.results.multi_hand_landmarks:
                if draw:
                    self.mpDraw.draw_landmarks(
                        frame, handLMs, self.mpHands.HAND_CONNECTIONS)

        if return_handedness:
            return frame, self.results.multi_handedness
        return frame

    def findHandPosition(self, frame, hand_num=0):
        lm_list = []
        if self.results.multi_hand_landmarks:
            hand = self.results.multi_hand_landmarks[hand_num]
            h, w, _ = frame.shape

            for id, lm in enumerate(hand.landmark):
                cx, cy = int(lm.x * w), int(lm.y * h)
                lm_list.append([id, cx, cy])

        return lm_list, frame

    def findHandAperture(self, frame):
        try:
            thumb = np.array(self.lm_list[4][1:])
            index = np.array(self.lm_list[8][1:])
            dist = np.linalg.norm(index - thumb)

            aperture = np.interp(dist, [20, 200], [0, 100])
            return frame, aperture
        except:
            return frame, 0
    