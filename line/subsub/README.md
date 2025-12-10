# 코드 간략 설명

## sub (subscribe)

라인 트레이서 실험에서 **pub 노드가 퍼블리시한 압축 영상 토픽**을 구독해서,  
영상 처리(ROI 추출, 이진화, 라인 중심 검출)를 수행하고,  
결과를 OpenCV 창에 시각화하면서 **라인 중심 위치 에러(error)를 계산**하는 노드입니다.  

---

### vision.hpp

### 파일 역할 (Role)
- `CamSubNode` 클래스를 선언하는 **헤더 파일**  
- `sensor_msgs::msg::CompressedImage` 타입 토픽을 **구독(subscribe)** 하는 ROS2 노드 인터페이스 정의
- 콜백 함수 `mysub_callback()` 에서:
  - 압축 이미지를 OpenCV `Mat`으로 디코딩
  - ROI(하단 부)만 사용해서 이진화
  - 라인 중심을 찾고, 이전 프레임과 연결해 추적
  - 화면에 디버깅용 윈도우 3개를 띄우고, 에러 및 처리 시간을 로그로 출력

---

```cpp
class CamSubNode : public rclcpp::Node
{
public:
    CamSubNode();
    ~CamSubNode();

private:
    void mysub_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg);
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr subscription_;
};
```
- CamSubNode()  
  압축 이미지 토픽 구독 설정과 OpenCV 디버그 창 생성을 포함해 subscriber node를 초기화하는 기본 생성자이다.

- mysub_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg)  
  수신된 압축 이미지 메시지를 디코딩해 라인 검출과 에러 계산 및 화면 표시를 수행하는 콜백 함수이다.

- subscription_
  지정된 토픽에서 sensor_msgs::msg::CompressedImage 메시지를 구독하여 mysub_callback()으로 전달하는 ROS2 구독 객체이다.


