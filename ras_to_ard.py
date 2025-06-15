#!/usr/bin/env python3
# coding: utf-8

import cv2
import numpy as np
import serial
import time

# シリアル初期化
SERIAL_PORT = '/dev/ttyACM0'   # 必要に応じて変更
BAUD_RATE   = 9600
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
time.sleep(2)  # Arduino リセット待ち

# カメラ初期化
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("ERROR: カメラが開けませんでした")
    exit(1)

# カメラの水平視野角（度）
HFOV = 60.0

# 色ごとのHSV範囲（辞書で定義）
color_ranges = {
    "Red": [
        (np.array([0,   100, 100]), np.array([10,  255, 255])),
        (np.array([160, 100, 100]), np.array([180, 255, 255]))
    ],
    "Blue": [
        (np.array([100, 150, 100]), np.array([130, 255, 255]))
    ],
    "Yellow": [
        (np.array([20,  100, 100]), np.array([30,  255, 255]))
    ]
}

# 描画用の BGR カラー
draw_colors = {
    "Red":    (0,   0,   255),
    "Blue":   (255, 0,   0),
    "Yellow": (0,   255, 255)
}

while True:
    ret, frame = cap.read()
    if not ret:
        print("ERROR: フレーム取得失敗")
        break

    height, width = frame.shape[:2]
    center_x = width / 2

    # 前処理
    blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    hsv     = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    # 最大検出情報
    max_area_color = None
    max_area_value = 0
    max_angle      = 0.0

    # 各色ごとにマスク→輪郭検出
    for color_name, ranges in color_ranges.items():
        # 空のマスクを作成
        mask = np.zeros_like(cv2.inRange(hsv, (0,0,0), (0,0,0)))
        for lower, upper in ranges:
            mask |= cv2.inRange(hsv, lower, upper)

        # 輪郭取得
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            continue

        # 面積最大の輪郭
        cnt  = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(cnt)
        if area < 500:
            continue

        # 最小外接円で中心位置・角度計算
        (x, y), radius = cv2.minEnclosingCircle(cnt)
        dx    = x - center_x
        angle = (dx / center_x) * (HFOV / 2)

        # 最大面積更新
        if area > max_area_value:
            max_area_color = color_name
            max_area_value = area
            max_angle      = angle

        # 画面にも描画（任意）
        cv2.circle(frame, (int(x), int(y)), int(radius), draw_colors[color_name], 2)
        cv2.putText(frame,
                    f"{color_name}: {area:.0f}px², {angle:.1f}°",
                    (int(x-radius), int(y-radius)-10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, draw_colors[color_name], 2)

    # シリアル送信
    if max_area_color:
        # フォーマット: 色名,面積,角度\n
        msg = f"{max_area_color},{int(max_area_value)},{max_angle:.2f}\n"
        ser.write(msg.encode('utf-8'))

    # ウィンドウ表示
    cv2.imshow("Multi Color Ball Tracking", frame)
    key = cv2.waitKey(1)
    if key == 27:  # ESC キーで終了
        break

# 後処理
cap.release()
cv2.destroyAllWindows()
ser.close()

