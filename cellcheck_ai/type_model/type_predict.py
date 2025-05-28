import numpy as np
import cv2
import socket
import json
from ultralytics import YOLO

# 모델 로딩 (배터리 종류 분류 모델)
model = YOLO("type_model/type_classification.pt")

# 마스크에서 중심, 박스, 영역 추출
def extract_confident_mask_region(mask_tensor, conf, min_thresh=0.05, max_thresh=0.4):
    threshold = min_thresh + (max_thresh - min_thresh) * conf
    binary_mask = (mask_tensor.cpu().numpy() > threshold).astype(np.uint8)
    contours, _ = cv2.findContours(binary_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    cnt = max(contours, key=cv2.contourArea)
    x, y, w, h = cv2.boundingRect(cnt)
    cx = x + w // 2
    cy = y + h // 2
    area = cv2.contourArea(cnt)
    return (x, y, w, h), (cx, cy), area

# 후처리 함수 (비율 기반 분류)
def postprocess(results):
    battery_info = []
    stick_dims = []

    for mask_tensor, cls, conf in zip(results.masks.data, results.boxes.cls, results.boxes.conf):
        label = model.names[int(cls)]
        result = extract_confident_mask_region(mask_tensor, conf.item())
        if result is None:
            continue
        bbox, center, area = result

        if label == "stick":
            stick_dims.append((bbox[2], bbox[3], area))
        elif label in ["AA", "AAA", "C", "D"]:
            battery_info.append({
                "label": label,
                "conf": conf.item(),
                "center": center,
                "bbox": bbox,
                "area": area
            })

    if not battery_info or not stick_dims:
        return None

    b = max(battery_info, key=lambda x: x["area"])
    s_area = np.mean([x[2] for x in stick_dims])
    s_len = np.mean([max(x[0], x[1]) for x in stick_dims])
    b_len = max(b["bbox"][2], b["bbox"][3])
    area_ratio = b["area"] / s_area if s_area > 0 else 0
    length_ratio = b_len / s_len if s_len > 0 else 0

    def classify(area_ratio, length_ratio):
        if area_ratio < 1.0:
            return 1  # AAA
        elif area_ratio < 2.0:
            return 0  # AA
        elif area_ratio < 3.5:
            return 2  # C
        else:
            return 3  # D

    class_id = classify(area_ratio, length_ratio)

    return {
        "class": class_id,
        "bbox": b["bbox"]
    }

# 예측 및 후처리 실행
def predict_from_bytes(image_bytes):
    img_np = np.frombuffer(image_bytes, np.uint8)
    img = cv2.imdecode(img_np, cv2.IMREAD_COLOR)
    results = model.predict(img, save=False, imgsz=640, conf=0.15)
    return postprocess(results[0])

# 소켓 전송 함수
def send_result_to_client(result, host='127.0.0.1', port=8888):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((host, port))
        message = json.dumps(result).encode('utf-8')
        s.sendall(message)
