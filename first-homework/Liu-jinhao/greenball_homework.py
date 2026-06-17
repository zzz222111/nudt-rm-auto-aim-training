import cv2
import numpy as np

# 视频路径
VIDEO_PATH = "first-homework/first-homework.mp4"

# HSV绿色范围
LOWER_GREEN = np.array([35, 40, 40], dtype=np.uint8)
UPPER_GREEN = np.array([85, 255, 255], dtype=np.uint8)

# 形态学核
KERNEL = np.ones((5,5), np.uint8)

# 筛选参数
MIN_AREA = 100
ASPECT_RATIO_RANGE = (0.7, 1.3)
MIN_CIRCULARITY = 0.7
ARROW_SCALE = 2.5

cap = cv2.VideoCapture(VIDEO_PATH)
if not cap.isOpened():
    print("无法打开视频！")
    exit()

prev_center = None  # 上一帧球心位置

while True:
    ret, frame = cap.read()
    if not ret:
        break
    output = frame.copy()

    # 1. HSV颜色提取
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, LOWER_GREEN, UPPER_GREEN)

    # 2. 降噪处理
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, KERNEL)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, KERNEL)

    # 3. 轮廓检测+形状筛选
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best_cnt = None
    best_center = None
    best_radius = 0
    max_area = 0

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < MIN_AREA:
            continue
        x, y, w, h = cv2.boundingRect(cnt)
        ratio = w / h
        if not (ASPECT_RATIO_RANGE[0] <= ratio <= ASPECT_RATIO_RANGE[1]):
            continue
        perimeter = cv2.arcLength(cnt, True)
        circularity = (4 * np.pi * area) / (perimeter ** 2) if perimeter > 0 else 0
        if circularity < MIN_CIRCULARITY:
            continue
        if area > max_area:
            max_area = area
            best_cnt = cnt
            (cx, cy), r = cv2.minEnclosingCircle(cnt)
            best_center = (int(cx), int(cy))
            best_radius = int(r)

    # 4. 绘制轮廓和箭头
    if best_center is not None:
        # 绘制轮廓和圆
        cv2.drawContours(output, [best_cnt], -1, (0,255,0), 2)
        cv2.circle(output, best_center, best_radius, (0,0,255), 2)
        cv2.circle(output, best_center, 3, (255,0,0), -1)

        # 绘制运动箭头
        if prev_center is not None:
            dx = best_center[0] - prev_center[0]
            dy = best_center[1] - prev_center[1]
            speed = np.sqrt(dx**2 + dy**2)
            arrow_end = (
                int(best_center[0] + dx * ARROW_SCALE),
                int(best_center[1] + dy * ARROW_SCALE)
            )
            cv2.arrowedLine(output, best_center, arrow_end, (0,255,255), 2, tipLength=0.3)
            cv2.putText(output, f"Speed: {speed:.1f} px/frame", 
                        (best_center[0]+10, best_center[1]-10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,255), 2)
        prev_center = best_center
    else:
        prev_center = None
        cv2.putText(output, "Target Lost", (50,50), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,0,255), 2)

    cv2.imshow("Green Ball Tracking", output)
    # cv2.imshow("Mask", mask)  # 显示掩膜窗口

    if cv2.waitKey(30) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
