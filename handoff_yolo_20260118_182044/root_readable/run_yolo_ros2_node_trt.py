# (1) 전역 변수 초기화
import sys
import types
import runpy
import torch
import importlib.metadata as importlib_metadata

# (2) 원래 version 함수 백업
_real_version = importlib_metadata.version

# (3) 함수 정의: torchvision 버전 조회 우회
def _safe_version(name: str) -> str:
    # (3-1) torchvision 조회면 가짜 버전 반환
    if name == "torchvision":
        return "0.0.0"
    # (3-2) 그 외는 원래대로
    return _real_version(name)

# (4) 함수 정의: pure torch NMS
def _pure_torch_nms(boxes: torch.Tensor, scores: torch.Tensor, iou_threshold: float) -> torch.Tensor:
    # (4-1) 빈 입력 처리
    if boxes.numel() == 0:
        return torch.empty((0,), dtype=torch.long, device=boxes.device)

    # (4-2) 좌표 분리
    x1 = boxes[:, 0]
    y1 = boxes[:, 1]
    x2 = boxes[:, 2]
    y2 = boxes[:, 3]

    # (4-3) 면적 계산
    areas = (x2 - x1).clamp(min=0) * (y2 - y1).clamp(min=0)

    # (4-4) score 내림차순 정렬
    order = scores.argsort(descending=True)

    # (4-5) keep 저장
    keep = []

    # (4-6) NMS 반복
    while order.numel() > 0:
        i = order[0]
        keep.append(i)

        if order.numel() == 1:
            break

        rest = order[1:]

        xx1 = torch.maximum(x1[i], x1[rest])
        yy1 = torch.maximum(y1[i], y1[rest])
        xx2 = torch.minimum(x2[i], x2[rest])
        yy2 = torch.minimum(y2[i], y2[rest])

        w = (xx2 - xx1).clamp(min=0)
        h = (yy2 - yy1).clamp(min=0)
        inter = w * h

        union = areas[i] + areas[rest] - inter
        iou = torch.where(union > 0, inter / union, torch.zeros_like(inter))

        order = rest[iou <= iou_threshold]

    # (4-7) 결과 반환
    return torch.stack(keep) if keep else torch.empty((0,), dtype=torch.long, device=boxes.device)

# (5) monkey patch 적용
importlib_metadata.version = _safe_version

# (6) torchvision 더미 모듈 주입
if "torchvision" not in sys.modules:
    # (6-1) 최상위 모듈
    tv = types.ModuleType("torchvision")
    tv.__version__ = "0.0.0"

    # (6-2) ops 모듈
    ops = types.ModuleType("torchvision.ops")
    ops.nms = _pure_torch_nms

    # (6-3) 연결
    tv.ops = ops

    # (6-4) 등록
    sys.modules["torchvision"] = tv
    sys.modules["torchvision.ops"] = ops

# (7) 원래 YOLO ROS2 노드 실행
runpy.run_path("/root/yolo_ros2_node.py", run_name="__main__")
