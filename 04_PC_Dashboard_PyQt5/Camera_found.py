import cv2

def find_camera_indices():
    available_indices = []
    # 通常设备不会超过 10 个，遍历 0-9 即可
    for i in range(10):
        # 推荐加上 cv2.CAP_DSHOW 强制使用 DirectShow 框架，在 Windows 下枚举更快更稳定
        cap = cv2.VideoCapture(i, cv2.CAP_DSHOW)
        if cap.isOpened():
            print(f"找到可用视频设备，索引号: {i}")
            available_indices.append(i)
            cap.release()
    return available_indices

print("当前系统可用的摄像头索引:", find_camera_indices())
