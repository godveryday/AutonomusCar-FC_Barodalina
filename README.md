# FC_바로달리나 

### 2023 DIFA 자율주행 경진대회 정리
---
#### 차량 스펙

<p align="left">
  <img width="30%" img src="https://github.com/user-attachments/assets/f89a4e7b-1374-4c86-ac04-dbbb3407c030"> <img width="30%" img src="https://github.com/user-attachments/assets/2bcfdc48-e6eb-4a15-a853-89120595d067">  <img width="30%" img src="https://github.com/user-attachments/assets/7e5287b6-afb9-4256-b59e-cad4f16c9832">
</p>

차량 SPEC에 맞는 외관을 Shapr3D, Autodesk Inventor를 활용해 제작
<br/><br/><br/><br/>

<p align="left">
  <img width = "55%" img src ="https://github.com/user-attachments/assets/74b7bc6f-d4bf-4f88-a5d7-1cff88c4754d1"> <img width = "35%" img src ="https://github.com/user-attachments/assets/f68eafbf-4dd3-47a1-a1d3-f3741d07743b">
</p>

OpenCV를 활용, HSV는 위 값을 기반으로 수정함, OpenCV와 ROS사이 연결은 위와 같음 

ROS와 OpenCV와의 Interface package인 cv_bridge를 

이를 통해 ROS에서 영상 데이터를 OpenCV format으로 변환해 처리가능

이후 처리 된 데이터를 ROS 영상으로 변환하고 토픽을 게시하여 노드 간 영상 전송을 실현

#### 당시 주행과 동시에 실시간 rqt를 확인할 수 없어 개발에 차질 있었음
