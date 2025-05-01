# 오류 처리 가이드

이 문서는 MyActuator RMD 모터 제어 라이브러리의 오류 처리 방식과 일반적인 오류 상황에 대한 대처 방법을 설명합니다.

## 오류 처리 원칙

이 라이브러리는 Abseil의 `Status` 및 `StatusOr<T>` 클래스를 사용하여 오류를 처리합니다. 이 접근 방식은 다음과 같은 이점을 제공합니다:

1. 오류 상태 코드와 메시지를 통한 상세한 오류 정보
2. 함수 반환 값과 오류 상태를 함께 처리할 수 있는 통합된 방식
3. 일관된 오류 처리 패턴
4. 예외 기반 오류 처리의 성능 문제 회피

## 상태 코드

라이브러리에서 사용하는 주요 상태 코드는 다음과 같습니다:

| 상태 코드 | 설명 | 일반적인 원인 | 대처 방법 |
|---|---|---|---|
| `absl::OkStatus()` | 작업이 성공적으로 완료됨 | 정상 동작 | - |
| `absl::InvalidArgumentError` | 메서드에 유효하지 않은 인자가 전달됨 | 범위를 벗어난 모터 ID, 토크 값, 속도 값 등 | 인자 값을 확인하고 유효한 범위 내의 값을 사용 |
| `absl::UnavailableError` | CAN 통신 오류 발생 | CAN 버스 연결 문제, 모터 전원 문제 | CAN 인터페이스 연결 및 모터 전원 확인 |
| `absl::InternalError` | 내부 처리 오류 | 응답 파싱 실패, 예상치 못한 응답 등 | 로그 확인 및 모터 상태 점검 |
| `absl::DeadlineExceededError` | 명령 시간 초과 | 모터가 응답하지 않음 | CAN 버스 부하 확인, 모터 연결 및 전원 확인 |
| `absl::AlreadyExistsError` | 이미 존재하는 리소스 추가 시도 | 이미 등록된 모터 ID 재등록 | 기존 ID 사용 또는 필요한 경우에만 ID 변경 |

## Status 사용 예시

`Status`를 반환하는 함수를 호출할 때:

```cpp
// 모터 정지 명령 실행
auto status = motor.motorStop();
if (!status.ok()) {
  // 오류 처리
  std::cerr << "모터 정지 실패: " << status.message() << std::endl;
  
  if (absl::IsUnavailable(status)) {
    std::cerr << "CAN 통신 오류가 발생했습니다. 연결을 확인하세요." << std::endl;
    // 통신 연결 재시도 또는 다른 조치
  } else if (absl::IsInternal(status)) {
    std::cerr << "내부 처리 오류가 발생했습니다." << std::endl;
    // 로그 기록, 재시도 등
  }
  return false;
}

// 성공적으로 명령이 실행됨
std::cout << "모터가 성공적으로 정지되었습니다." << std::endl;
```

## StatusOr 사용 예시

`StatusOr<T>`를 반환하는 함수를 호출할 때:

```cpp
// 모터 상태 읽기
auto status_or = motor.readStatus1();
if (!status_or.ok()) {
  // 오류 처리
  std::cerr << "모터 상태 읽기 실패: " << status_or.status().message() << std::endl;
  
  if (absl::IsUnavailable(status_or.status())) {
    std::cerr << "CAN 통신 오류가 발생했습니다. 연결을 확인하세요." << std::endl;
    // 통신 연결 재시도 또는 다른 조치
  }
  return false;
}

// 성공적으로 상태를 읽었으므로 값 사용
auto status_data = status_or.value();  // .value() 또는 * 연산자로 값에 접근
std::cout << "모터 온도: " << static_cast<int>(status_data.temperature) << "°C" << std::endl;
std::cout << "모터 전압: " << static_cast<float>(status_data.voltage) * 0.1f << "V" << std::endl;
```

## 일반적인 오류 상황 및 대처 방법

### 1. CAN 통신 오류 (`absl::UnavailableError`)

가장 흔한 오류 유형으로, 다음과 같은 원인으로 발생할 수 있습니다:

- CAN 인터페이스가 존재하지 않거나 잘못 구성됨
- CAN 버스 연결 문제 (케이블, 커넥터 등)
- 모터 전원이 꺼져 있음
- CAN 버스 속도 불일치

**대처 방법:**
- `ip link show` 명령으로 CAN 인터페이스 상태 확인
- `ip link set <인터페이스> up type can bitrate 1000000` 명령으로 CAN 인터페이스 설정
- CAN 케이블 및 모터 전원 연결 확인
- 다른 CAN 장치와의 충돌 확인

### 2. 유효하지 않은 인자 오류 (`absl::InvalidArgumentError`)

다음과 같은 경우에 발생할 수 있습니다:

- 모터 ID가 유효 범위(1-32)를 벗어남
- 토크, 속도, 위치 등의 값이 허용 범위를 벗어남
- 잘못된 명령 코드 사용

**대처 방법:**
- 모터 ID는 1-32 범위 내에서 사용
- 토크 제어 값은 -2000 ~ 2000 범위 내에서 사용
- 각도 값은 해당 명령의 유효 범위 내에서 사용
- 라이브러리 문서에서 각 메서드의 매개변수 요구사항 확인

### 3. 내부 처리 오류 (`absl::InternalError`)

다음과 같은 경우에 발생할 수 있습니다:

- 모터로부터 예상치 못한 응답 수신
- 응답 데이터 파싱 오류
- 일치하지 않는 응답 명령 코드

**대처 방법:**
- 모터 펌웨어 버전 확인 (V1.61 프로토콜 지원 여부)
- 모터 상태 확인 및 재시작
- 오류가 지속되면 모터 오류 플래그 초기화 시도 (`clearErrorFlag()` 메서드)

### 4. 시간 초과 오류 (`absl::DeadlineExceededError`)

다음과 같은 경우에 발생할 수 있습니다:

- 모터가 응답하지 않음
- CAN 버스 과부하
- 모터 처리 지연

**대처 방법:**
- CAN 버스의 다른 트래픽 확인 및 감소
- 명령 간 지연 추가 (과부하 방지)
- 모터 전원 및 상태 확인

### 5. 이미 존재하는 리소스 오류 (`absl::AlreadyExistsError`)

다음과 같은 경우에 발생할 수 있습니다:

- 이미 등록된 모터 ID를 다시 등록 시도

**대처 방법:**
- 동일한 모터 ID를 여러 번 등록하지 않음
- 이미 등록된 ID를 사용하거나 다른 ID 할당

## 오류 로깅 및 분석

효과적인 오류 분석을 위해 다음과 같은 접근 방식을 권장합니다:

1. 상태 코드뿐 아니라 상태 메시지도 로깅
2. CAN 트래픽 모니터링 도구 사용 (예: candump)
3. 반복되는 오류 패턴 분석
4. 모터 상태 데이터 주기적 확인

## 일반적인 오류 처리 패턴

다음은 이 라이브러리 사용 시 권장되는 일반적인 오류 처리 패턴입니다:

```cpp
// 헬퍼 함수 - absl::Status 처리
bool checkStatus(const absl::Status& status, const std::string& operation) {
  if (!status.ok()) {
    std::cerr << "오류 발생 (" << operation << "): " << status.message() << std::endl;
    // 특정 오류 유형에 따른 추가 처리
    return false;
  }
  return true;
}

// 헬퍼 함수 - absl::StatusOr<T> 처리
template <typename T>
bool checkStatusOr(const absl::StatusOr<T>& status_or, T& out_value, 
                  const std::string& operation) {
  if (!status_or.ok()) {
    std::cerr << "오류 발생 (" << operation << "): " << status_or.status().message() << std::endl;
    // 특정 오류 유형에 따른 추가 처리
    return false;
  }
  out_value = status_or.value();
  return true;
}

// 사용 예시
v161_motor_control::types::Status1DataV161 status1_data;
if (!checkStatusOr(motor.readStatus1(), status1_data, "모터 상태 읽기")) {
  // 오류 발생 시 처리
  return;
}

// 성공 시 값 사용
std::cout << "모터 온도: " << static_cast<int>(status1_data.temperature) << "°C" << std::endl;
```

이러한 패턴을 일관되게 사용하면 코드 가독성이 향상되고 오류 처리가 더 효과적으로 이루어집니다. 