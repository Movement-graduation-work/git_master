<?xml version="1.0" encoding="UTF-8"?>
<project>

    <name>YOLO-Based Autonomous Perception & Control Stack</name>

    <summary>
        YOLO 기반 사람 인식 → 상태 판단 → 회피 주행(cmd_vel 생성)까지 수행하는
        ROS2 기반 자율주행 perception 시스템
    </summary>

    <description>
        본 프로젝트는 Realsense 카메라 입력을 기반으로 YOLO 객체 인식을 수행하고,
        사람의 위치 및 상태를 판단한 후, zone 기반 회피 제어를 통해 cmd_vel을 생성한다.
        모든 노드는 ROS2 토픽 기반으로 loosely coupled 구조로 구성되어 있다.
    </description>

    <!-- ====================== PIPELINE ====================== -->
    <pipeline>

        <step order="1">/camera/camera/color/image_raw</step>
        <step order="2">yolo_ros2_node.py</step>

        <step order="3">/yolo/detections_raw (main input)</step>
        <step order="4">person_state_zone_node.py</step>

        <step order="5">/perception/state</step>
        <step order="6">/perception/zone</step>
        <step order="7">/perception/best_conf</step>

        <step order="8">perception_summary_node.py</step>
        <step order="9">/perception/summary</step>

        <step order="10">avoid_by_zone_node.py</step>
        <step order="11">/cmd_vel</step>

    </pipeline>

    <!-- ====================== TOPICS ====================== -->
    <topics>

        <yolo>
            <topic>/yolo/detections</topic>
            <topic>/yolo/detections_raw</topic>
            <topic>/yolo/annotated/image_raw</topic>
            <note>현재 메인 입력은 /yolo/detections_raw</note>
        </yolo>

        <perception>
            <topic>/perception/state</topic>
            <topic>/perception/human_detected</topic>
            <topic>/perception/zone</topic>
            <topic>/perception/best_conf</topic>
            <topic>/perception/summary</topic>
        </perception>

        <control>
            <topic>/cmd_vel</topic>
        </control>

    </topics>

    <!-- ====================== NODE ROLES ====================== -->
    <nodes>

        <node name="yolo_ros2_node.py">
            <input>/camera/camera/color/image_raw</input>
            <output>/yolo/detections_raw</output>
            <output>/yolo/annotated/image_raw</output>
            <role>TensorRT FP16 기반 YOLO 추론 수행</role>
        </node>

        <node name="person_state_zone_node.py">
            <input>/yolo/detections_raw</input>
            <input>/camera/camera/color/camera_info</input>
            <output>/perception/state</output>
            <output>/perception/human_detected</output>
            <output>/perception/zone</output>
            <output>/perception/best_conf</output>
            <role>
                사람 객체 필터링, bbox 중심 기반 zone 계산,
                히스테리시스 기반 state 판단
            </role>
        </node>

        <node name="perception_summary_node.py">
            <input>/perception/state</input>
            <input>/perception/zone</input>
            <input>/perception/best_conf</input>
            <input>/cmd_vel</input>
            <output>/perception/summary</output>
            <role>디버깅 및 관제용 상태 문자열 생성</role>
        </node>

        <node name="avoid_by_zone_node.py">
            <input>/perception/state</input>
            <input>/perception/zone</input>
            <output>/cmd_vel</output>
            <role>zone 기반 회피 주행 명령 생성</role>
        </node>

    </nodes>

    <!-- ====================== CONTROL LOGIC ====================== -->
    <control_logic>

        <rule>NO_HUMAN → 전진</rule>
        <rule>HUMAN_DETECTED + LEFT → 우회전</rule>
        <rule>HUMAN_DETECTED + RIGHT → 좌회전</rule>
        <rule>HUMAN_DETECTED + CENTER → 정지</rule>

    </control_logic>

    <!-- ====================== SAFETY ====================== -->
    <safety>

        <rule>perception 입력이 stale 될 경우 → cmd_vel = 0 (정지)</rule>
        <rule>데이터 유효성 실패 시 안전 정지</rule>

    </safety>

    <!-- ====================== MODELS ====================== -->
    <models>
        <file name="yolov8n.onnx">TensorRT/ONNX inference model</file>
        <file name="yolov8n.pt">PyTorch training model</file>
    </models>

    <!-- ====================== NOTES ====================== -->
    <notes>

        <item>*.bak 파일은 실험용 백업이며 실행에 사용되지 않음</item>
        <item>ROS2 Humble 환경 기준</item>
        <item>/cmd_vel은 현재 실제 하드웨어에 연결되지 않은 상태일 수 있음</item>

    </notes>

    <!-- ====================== USAGE ====================== -->
    <usage>

        <step>ROS2 환경 설정</step>
        <step>colcon build 수행</step>
        <step>perception_stack.sh 실행</step>

    </usage>

</project>
