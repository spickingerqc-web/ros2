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
```
- node_name (기본값: `campub_7`)  
  ROS2에서 사용할 node 이름(Node name)입니다.

- topic_name (기본값: `image/compressed_7`)  
  압축 이미지를 publish 할 topic 이름(Topic name)입니다.

- video_source (기본값: `/home/linux/simulation/7_lt_ccw_100rpm_in.mp4`)  
  OpenCV VideoCapture 가 여는 video source path 입니다.

- CamPubNode() 처럼 인자 없이 호출하면  
  위 기본값(Node name, Topic name, Video source)이 그대로 자동 적용됩니다.

#### 2. private 멤버 및 함수 역할

```cpp
private:
    void publish_frame();  // 타이머 콜백에서 한 프레임씩 publish

    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::QoS qos_profile_;

    cv::VideoCapture cap_;
    cv::Mat frame_;
    std::string video_source_;
};
```

- publish_frame()
  타이머 주기로 호출되어 한 프레임을 읽어와 토픽으로 이미지를 publish 하는 함수이다.

- publisher_
  sensor_msgs::msg::CompressedImage 메시지를 지정된 토픽으로 publish 하는 ROS2 publisher 이다.

- timer_
  일정 주기로 publish_frame() 함수를 호출하도록 스케줄링 하는 타이머이다.

- qos_profile_
  publisher 에 사용될 QoS 설정 정보를 저장하는 멤버이다.

- cap_
  영상 파일 또는 카메라로부터 프레임을 읽어오는 OpenCV VideoCapture 객체이다.

- frame_
  현재 읽어온 한 프레임 이미지를 담는 cv::Mat 버퍼이다.

- video_source_
  사용할 영상 소스의 경로 또는 파이프라인 문자열을 저장하는 멤버이다.

