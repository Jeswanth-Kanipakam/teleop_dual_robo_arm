import mediapipe as mp
import cv2
import numpy as np


class HandDetector:
    def __init__(self, maxHands=2):
        self.mpHands = mp.solutions.hands
        self.hands = self.mpHands.Hands(
            static_image_mode=False,
            max_num_hands=maxHands,
            model_complexity=1,
            min_detection_confidence=0.7,
            min_tracking_confidence=0.7
        )

        self.mpDraw = mp.solutions.drawing_utils
        self.results = None
        self.lm_list = []

    def findHands(self, frame, draw=True):
        imgRGB = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        self.results = self.hands.process(imgRGB)

        if self.results.multi_hand_landmarks:
            for handLMs in self.results.multi_hand_landmarks:
                if draw:
                    self.mpDraw.draw_landmarks(
                        frame,
                        handLMs,
                        self.mpHands.HAND_CONNECTIONS
                    )
        return frame

    def findPosition(self, frame, hand_num=0):
        self.lm_list = []

        if self.results.multi_hand_landmarks:
            hand = self.results.multi_hand_landmarks[hand_num]
            h, w, _ = frame.shape

            for idx, lm in enumerate(hand.landmark):
                cx, cy = int(lm.x * w), int(lm.y * h)
                self.lm_list.append([idx, cx, cy])

        return self.lm_list

    def getHandCenter(self):
        if len(self.lm_list) >= 21:
            wrist = np.array(self.lm_list[0][1:])
            palm = np.array(self.lm_list[9][1:])
            return (wrist + palm) / 2
        return None

    def findAperture(self):
        if len(self.lm_list) >= 9:
            thumb = np.array(self.lm_list[4][1:])
            index = np.array(self.lm_list[8][1:])
            dist = np.linalg.norm(index - thumb)
            return np.interp(dist, [20, 150], [0.0, 1.0])
        return 0.0
    
