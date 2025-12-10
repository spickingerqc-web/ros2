# 코드 간략 설명

## sub (subscribe / vision)

라인 트레이서 실험에서 **pub 노드가 퍼블리시한 압축 영상 토픽**을 구독해서,  
영상 처리(ROI 추출, 이진화, 라인 중심 검출)를 수행하고,  
결과를 OpenCV 창에 시각화하면서 **라인 중심 위치 에러(error)를 계산**하는 노드입니다. :contentReference[oaicite:0]{index=0}  

아래는 `vision.hpp` / `vision.cpp` 에 대한 간단한 설명입니다.  

---

## 1. vision.hpp

> 코드 전문은 생략하고, 역할 중심으로만 정리한 섹션입니다.

### 파일 역할 (Role)

- `CamSubNode` 클래스를 선언하는 **헤더 파일**  
- `sensor_msgs::msg::CompressedImage` 타입 토픽을 **구독(subscribe)** 하는 ROS2 노드 인터페이스 정의
- 콜백 함수 `mysub_callback()` 에서:
  - 압축 이미지를 OpenCV `Mat`으로 디코딩
  - ROI(하단 부)만 사용해서 이진화
  - 라인 중심을 찾고, 이전 프레임과 연결해 추적
  - 화면에 디버깅용 윈도우 3개를 띄우고, 에러 및 처리 시간을 로그로 출력

---

## 2. vision.cpp

```cpp
#include "subsub/vision.hpp"
#include <sys/time.h>
#include <unistd.h>

bool ctrl_c_pressed = false;
void ctrlc(int)
{
    ctrl_c_pressed = true;
}
