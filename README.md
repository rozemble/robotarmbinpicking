# Robot manipulator를 활용한 AI Bin Picking Solution

Doosan Robotics의 A0509 협동로봇 기반 Vision AI를 활용한 Bin Picking 솔루션 코드 샘플입니다.
주요 시스템 구성은 다음과 같습니다.

1. Hardware
	- Robot : Doosan Robotics A0509 Collaborative Robot
	- Vision : Intel Realsense D435 RGBD Camera(attached to end-tool)
	- Suction : Piab Suction cup & SMC Vacuum Ejector
2. Software
	- Language : Python, DRL(두산 협동로봇 프로그래밍 언어)
	- Library : Detectron, Open3D, Opencv, Numpy, Pandas
3. 그외 활용 툴
	- Blender : AI 학습용 합성 데이터셋 생성

## 주요 모듈(개발 기여도 80% 이상)
- 메인 프로세스 
	- a0509.py
	- module/robot/robot.py
-  비전 모듈
	- module/vision/visionstream.py
-  객체 인식 모듈 
	- module/detectron/detectron.py
-  로봇 석션 포인트 탐지 모듈 
	- module/vision/Estimator_Giftbox.py
 	- module/vision/Estimator_Pipe.py

### 그 외 모듈
-  통신 모듈 
	- module/comm/tcp_socket.py
- 좌표 변환 등 유틸 
	- module/util/convertUtils.py
	- module/util/util.py
-  Robot-side 모듈
	- main.drl

### 시연영상
[![영상1](https://img.youtube.com/vi/LY-KS3O2tJ4/0.jpg)](https://www.youtube.com/watch?v=LY-KS3O2tJ4)

[![영상2](https://img.youtube.com/vi/3L-fJwPd2m0/0.jpg)](https://youtu.be/3L-fJwPd2m0)

