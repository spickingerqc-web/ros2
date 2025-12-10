# Line Tracer

라인 트레이서(Line Tracer) 실험 기록용 리포지토리입니다.

---

## Author (작성자)

- 정진용

---

## Demo Videos (시연 영상)

### 1. 5 lt cw 100rpm – Out Line Tracer  
[▶ Watch on YouTube](https://youtu.be/BwHTx4GQWgU)

### 2. 7 lt ccw 100rpm – In Line Tracer  
[▶ Watch on YouTube](https://www.youtube.com/watch?v=McKT8FgOp5I)

---

# 코드 간략 설명

## pub (퍼블리셔)

라인 트레이서 실험에서 **영상 소스를 ROS2 이미지 토픽으로 퍼블리시하는 노드**입니다.  
아래는 pub 노드의 헤더 파일(`pub.hpp`)에 대한 간단한 설명입니다.  
(`pub.cpp`는 나중에 따로 섹션을 추가하면 됨)

---

### pub.hpp

**파일 역할 (Role)**  
- `CamPubNode` 클래스를 선언하는 헤더 파일  
- 비디오 소스를 읽어서 `sensor_msgs::msg::CompressedImage` 타입으로 퍼블리시하는 ROS2 노드의 인터페이스 정의

---

#### 1. 생성자 기본 인자

```cpp
CamPubNode(
    const std::string & node_name   = "campub_7",
    const std::string & topic_name  = "image/compressed_7",
    const std::string & video_source = "/home/linux/simulation/7_lt_ccw_100rpm_in.mp4");
