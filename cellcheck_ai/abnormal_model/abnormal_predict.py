import numpy as np
import cv2
import socket
import json
from ultralytics import YOLO

# 모델 로딩
model = YOLO("abnormal_model/abnormal_classification.pt")  # 실제 모델 경로로 수정

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

# 후처리 함수 (비정상/정상 판단만 수행하고 class 번호 반환)
def postprocess(results):
    for mask_tensor, cls, conf in zip(results.masks.data, results.boxes.cls, results.boxes.conf):
        label = model.names[int(cls)]
        result = extract_confident_mask_region(mask_tensor, conf.item())
        if result is None:
            continue
        bbox, center, area = result

        # 비정상은 class 1, 정상은 class 0으로 가정
        if label == "abnormal":
            return {
                "abnormal": True,
                "bbox": list(bbox)
            }
        elif label == "normal":
            return {
                "abnormal": False,
                "bbox": list(bbox)
            }

    return None

# 예측 및 후처리
def predict_from_bytes(image_bytes):
    np_img = np.frombuffer(image_bytes, np.uint8)
    img = cv2.imdecode(np_img, cv2.IMREAD_COLOR)
    results = model.predict(img, save=False, imgsz=640, conf=0.15)
    return postprocess(results[0])

# 클라이언트로 전송
def send_result_to_client(result, host='127.0.0.1', port=8888):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((host, port))
        message = json.dumps(result).encode('utf-8')
        s.sendall(message)
