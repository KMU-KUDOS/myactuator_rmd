# MyActuator RMD 모터 제어 라이브러리 (V1.61)

이 라이브러리는 CAN 통신을 통해 MyActuator RMD 시리즈 모터(V1.61 프로토콜)를 제어하기 위한 C++ 라이브러리입니다.

## 특징

- CAN 통신을 통한 MyActuator RMD 모터 제어
- 다중 모터 동시 제어 지원
- 단순하고 직관적인 C++ API
- 강력한 오류 처리 (Abseil Status/StatusOr 사용)
- 모듈형 설계로 확장성 제공
- 철저한 문서화 및 예제 코드 제공

## 주요 컴포넌트

- **CanInterface**: CAN 통신을 담당하는 인터페이스
- **MotorRegistry**: 여러 모터 ID를 관리하는 레지스트리
- **MotorV161**: 모터 제어를 위한 주요 클래스
- **MotorActuator**: 모터 제어 명령 (토크, 속도, 위치 등)
- **MotorStatusQuerier**: 모터 상태 조회 기능
- **MotorConfigurator**: 모터 구성 및 설정 관리

## 지원하는 기능

- 모터 상태 조회 (온도, 전압, 전류, 위치, 속도 등)
- 모터 제어 모드 (토크, 속도, 위치)
- 다양한 위치 제어 모드 (단일턴, 멀티턴, 속도 제한, 방향 지정 등)
- PID 및 가속도 설정
- 엔코더 설정
- 오류 상태 확인 및 초기화

## 빌드 방법

이 라이브러리는 CMake를 사용하여 빌드됩니다. 다음 명령을 사용하여 빌드할 수 있습니다:

```bash
mkdir -p build && cd build
cmake ..
make
```

## 사용 방법

다음은 기본적인 사용 예시입니다:

```cpp
#include "myactuator_rmd/can_interface.h"
#include "myactuator_rmd/motor_registry.h"
#include "myactuator_rmd/protocol/motor_v161.h"

int main() {
  // CAN 인터페이스 생성
  auto can_interface = std::make_shared<v161_motor_control::CanInterface>("can0");
  
  // 모터 레지스트리 생성
  auto motor_registry = std::make_shared<v161_motor_control::MotorRegistry>();
  
  // 모터 ID 등록
  auto status = motor_registry->addMotorId(1);
  if (!status.ok()) {
    std::cerr << "모터 ID 등록 실패: " << status.message() << std::endl;
    return 1;
  }
  
  // 모터 객체 생성
  v161_motor_control::MotorV161 motor(can_interface, motor_registry, 1);
  
  // 모터 상태 읽기
  auto status_or = motor.readStatus1();
  if (!status_or.ok()) {
    std::cerr << "모터 상태 읽기 실패: " << status_or.status().message() << std::endl;
    return 1;
  }
  
  // 모터 상태 사용
  auto status_data = status_or.value();
  std::cout << "모터 온도: " << static_cast<int>(status_data.temperature) << "°C" << std::endl;
  std::cout << "모터 전압: " << static_cast<float>(status_data.voltage) * 0.1f << "V" << std::endl;
  
  // 모터 제어 (예: 속도 제어)
  auto feedback_or = motor.setSpeedControl(100);  // 1 dps
  if (!feedback_or.ok()) {
    std::cerr << "속도 제어 실패: " << feedback_or.status().message() << std::endl;
    return 1;
  }
  
  return 0;
}
```

더 많은 예제는 `examples/v161_motor` 디렉토리를 참조하세요.

## 오류 처리

이 라이브러리는 Abseil의 Status와 StatusOr 클래스를 사용하여 오류를 처리합니다. 일반적인 오류 상태 코드는 다음과 같습니다:

- `absl::InvalidArgumentError`: 메서드에 유효하지 않은 인자가 전달됨
- `absl::UnavailableError`: CAN 통신 오류 발생 (연결 문제 등)
- `absl::InternalError`: 내부 처리 오류 (응답 파싱 실패 등)
- `absl::DeadlineExceededError`: 명령 시간 초과 (응답 받지 못함)
- `absl::AlreadyExistsError`: 이미 존재하는 리소스 (모터 ID 등) 추가 시도

상세한 오류 처리 방법 및 권장 패턴은 [오류 처리 가이드](docs/error_handling.md)를 참조하세요.

## 의존성

- C++17 이상
- [Abseil 라이브러리](https://abseil.io/)
- Linux SocketCAN (CAN 통신용)

## 라이선스

이 라이브러리는 MIT 라이선스로 제공됩니다.

## 주의사항

- 실제 모터를 제어할 때는 모터가 안전하게 고정되어 있고 주변에 장애물이 없는지 확인하세요.
- 테스트 전에 모터 전원을 끄고, 테스트 중에는 비상 정지 장치를 사용할 수 있도록 하세요.
- 모터 파라미터 설정 (특히 ROM에 쓰기)은 모터의 동작에 영향을 미칠 수 있으므로 주의해서 사용하세요. 