# MyActuator RMD 모터 V161 제어 예제

이 디렉토리에는 MyActuator RMD 모터 시리즈(V1.61 프로토콜)를 제어하기 위한 예제 코드가 포함되어 있습니다.

## 기능 개요

이 예제는 다음과 같은 기능을 보여줍니다:

* CAN 인터페이스를 통한 모터 연결
* 단일 및 다중 모터 제어
* 모터 상태 정보 읽기 (온도, 전압, 엔코더 값 등)
* 위치 제어, 속도 제어, 토크 제어 모드 사용법
* PID 및 가속도 설정 값 읽기/쓰기
* absl::Status 및 absl::StatusOr를 사용한 오류 처리

## 빌드 방법

프로젝트 루트 디렉토리에서 다음 명령을 실행하여 전체 프로젝트와 예제를 빌드합니다:

```bash
mkdir -p build && cd build
cmake ..
make
```

## 실행 방법

빌드 완료 후 다음과 같이 예제를 실행할 수 있습니다:

```bash
# 기본 설정으로 실행 (CAN 인터페이스: can0, 모터 ID: 1)
./examples/v161_motor/v161_example

# 특정 CAN 인터페이스와 모터 ID로 실행
./examples/v161_motor/v161_example --can can1 --motor-id 2

# 다중 모터 제어 데모 실행 (여러 모터 필요)
./examples/v161_motor/v161_example --motor-id 1 --add-motor-id 2 --demo-multi-motor

# 오류 처리 데모 실행 (실제 모터 연결 불필요)
./examples/v161_motor/v161_example --demo-error-handling
```

## 명령줄 옵션

이 예제는 다음과 같은 명령줄 옵션을 지원합니다:

* `--can <인터페이스>`: 사용할 CAN 인터페이스 (기본값: can0)
* `--motor-id <ID>`: 제어할 모터 ID (기본값: 1)
* `--add-motor-id <ID>`: 추가 모터 ID (다중 모터 제어용)
* `--demo-multi-motor`: 다중 모터 제어 데모 실행
* `--demo-error-handling`: 오류 처리 데모 실행
* `--help`: 도움말 메시지 표시

## 오류 처리

이 예제는 absl::Status 및 absl::StatusOr 패턴을 사용하여 오류를 처리합니다. 다음과 같은 주요 오류 상태 코드가 사용됩니다:

* `absl::InvalidArgumentError`: 메서드에 유효하지 않은 인자가 전달됨
* `absl::UnavailableError`: CAN 통신 오류 발생 (연결 문제 등)
* `absl::InternalError`: 내부 처리 오류 (응답 파싱 실패 등)
* `absl::DeadlineExceededError`: 명령 시간 초과 (응답 받지 못함)
* `absl::AlreadyExistsError`: 이미 존재하는 리소스 (모터 ID 등) 추가 시도

## 주의사항

* 이 예제를 실행할 때는 모터가 안전하게 고정되어 있고 주변에 장애물이 없는지 확인하세요.
* 테스트 전에 모터 전원을 끄고, 테스트 중에는 비상 정지 장치를 사용할 수 있도록 하세요.
* 모터 파라미터 설정 (특히 ROM에 쓰기)은 모터의 동작에 영향을 미칠 수 있으므로 주의해서 사용하세요. 