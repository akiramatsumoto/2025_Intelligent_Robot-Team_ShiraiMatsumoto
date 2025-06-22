#!/usr/bin/env python3
# coding: utf-8

import cv2
import numpy as np
import serial
import time
import sys

# === 設定 ===
SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE   = 9600
MAX_RETRIES = 5
RETRY_DELAY = 2  # 秒
HFOV = 60.0  # カメラの水平視野角

# === 赤色のHSV範囲のみ ===
color_ranges = {
    "Red": [
        (np.array([0,   100, 100]), np.array([10,  255, 255])),
        (np.array([160, 100, 100]), np.array([180, 255, 255]))
    ]
}

def init_serial():
    for i in range(MAX_RETRIES):
        try:
            ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
            time.sleep(2)
            print(f"[INFO] Serial connected to {SERIAL_PORT}")
            return ser
        except serial.SerialException as e:
            print(f"[WARN] Serial connect failed ({i+1}/{MAX_RETRIES}): {e}")
            time.sleep(RETRY_DELAY)
    print("[ERROR] Could not open serial port.")
    return None

def safe_serial_write(ser, msg):
    try:
        if ser and ser.is_open:
            ser.write(msg.encode('utf-8'))
        else:
            raise serial.SerialException("Port not open")
    except serial.SerialException as e:
        print("[WARN] Serial write failed:", e)
        print("[INFO] Attempting to reconnect...")
        return init_serial()
    return ser

def main():
    ser = init_serial()

    cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
    if not cap.isOpened():
        print("[ERROR] カメラが開けませんでした")
        sys.exit(1)

    while True:
        ret, frame = cap.read()
        if not ret:
            print("[WARN] フレーム取得失敗")
            time.sleep(0.5)
            continue

        height, width = frame.shape[:2]
        center_x = width / 2

        blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        hsv     = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        mask = np.zeros_like(cv2.inRange(hsv, (0,0,0), (0,0,0)))
        for lower, upper in color_ranges["Red"]:
            mask |= cv2.inRange(hsv, lower, upper)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        try:
            if not contours:
                msg = "None,-1,0.00\n"
                print(f"[SEND] {msg.strip()}")
                ser = safe_serial_write(ser, msg)
                time.sleep(0.05)
                continue

            cnt  = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(cnt)
            if area < 500:
                msg = "None,-1,0.00\n"
                print(f"[SEND] {msg.strip()}")
                ser = safe_serial_write(ser, msg)
                time.sleep(0.05)
                continue

            (x, y), radius = cv2.minEnclosingCircle(cnt)
            dx    = x - center_x
            angle = (dx / center_x) * (HFOV / 2)

            msg = f"Red,{int(area)},{angle:.2f}\n"
            print(f"[SEND] {msg.strip()}")
            ser = safe_serial_write(ser, msg)

        except Exception as e:
            print(f"[ERROR] 処理中にエラーが発生: {e}")

        time.sleep(0.05)

    cap.release()
    if ser:
        ser.close()

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n[INFO] 終了します")
