
---
##변수

### YOLO
- `/yolo/detections`
- `/yolo/detections_raw` ✅ (main input)
- `/yolo/annotated/image_raw`

### Perception file
- `/perception/state`
- `/perception/human_detected`
- `/perception/zone`
- `/perception/best_conf`
- `/perception/summary`

### final Control
- `/cmd_vel`

---
## file 설명

### 🔹 yolo_ros2_node.py
- **Input:** `/camera/camera/color/image_raw`
- **Output:**  
  - `/yolo/detections_raw`  
  - `/yolo/annotated/image_raw`  
- **Role:** TensorRT FP16 기반 YOLO 추론 수행

---

### 🔹 person_state_zone_node.py
- **Input:**  
  - `/yolo/detections_raw`  
  - `/camera/camera/color/camera_info`  
- **Output:**  
  - `/perception/state`  
  - `/perception/human_detected`  
  - `/perception/zone`  
  - `/perception/best_conf`  
- **Role:**  
  - 사람 객체 필터링  
  - bbox 중심 기반 zone 계산  (bounding box 사람타겟 박스 생성, 신뢰도 person92%)
  - 히스테리시스 기반 state 판단 (노이즈 완화)

---

### 🔹 perception_summary_node.py
- **Input:**  
  - `/perception/state`  
  - `/perception/zone`  
  - `/perception/best_conf`  
  - `/cmd_vel`  
- **Output:** `/perception/summary`  
- **Role:** 디버깅 및 터미널  관제용 상태 문자열 생성

---

### 🔹 avoid_by_zone_node.py
- **Input:**  
  - `/perception/state`  
  - `/perception/zone`  
- **Output:** `/cmd_vel`  
- **Role:** zone 기반 회피 주행 명령 생성 (사람이 Left, Center, Right에 있다는 표시zone)

---

## ⚙️ Control Logic
- **NO_HUMAN → 전진**
- **HUMAN_DETECTED + LEFT → 우회전**
- **HUMAN_DETECTED + RIGHT → 좌회전**
- **HUMAN_DETECTED + CENTER → 정지**

---

## 🛡️ Safety
- perception 입력이 stale(오래될) 될 경우 → `cmd_vel = 0` (정지)  
- 데이터 유효성 실패 시 안전 정지  

---

## 🧩 Models
- `yolov8n.onnx` → TensorRT / ONNX inference model  
- `yolov8n.pt` → PyTorch training model  

---

## 실행
- 터미널1 
source /opt/ros/humble/setup.bash
ros2 launch realsense2_camera rs_launch.py
- 터미널2
source /opt/ros/humble/setup.bash
cd ~/root_readable
python3 yolo_ros2_node.py & bash perception_stack.sh (위 여러게 py파일들 묶어놓은 것들)
