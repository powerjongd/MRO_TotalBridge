# Mro Unified Bridge

Unreal 기반 Morai Sim: Air(MRO) 와 외부 Software 사이에서 이미지를 중계하고, 센서/짐벌 제어, Gazebo UDP 릴레이, MAVLink 시리얼 변환 등을 수행하는 데스크톱 앱입니다. 영문명은 **Unified Bridge** (통합브릿지) 입니다.
GUI / Headless(무화면) 모두 지원하며, 설정과 데이터는 실행 파일(또는 main.py) 바로 아래에 저장합니다.

# 주요 기능
- **Image Stream Module (ImageStreamBridge)**
  - UDP JPEG 수신 → 실시간 프리뷰 및 저장
  - TCP MroCameraControl: Req_Capture(최신 프레임을 SaveFile/000.jpg 순환 저장), Set_Count, Get_ImgNum, Req_SendImg 지원
  - SaveFile(`./SaveFile/000.jpg`) 실시간 캡처 또는 PreDefinedImageSet(`./PreDefinedImageSet/000.jpg`) 사전 이미지 중 UI에서 선택 가능
  - GUI 프리뷰 스냅샷 저장: `./SaveFile/preview_<timestamp>.jpg`
- **Gimbal Control Module**
  - SensorGimbalCtrl(10706): position(double[3]) + orientation(float[4, quaternion])
  - SensorPowerCtrl(10707): sensor_type / id / power(0/1)
  - UI에서 Sensor type/ID/Power/MaxRate/TargetPose(xyzrpy) 편집 가능
- **Sensor Relay Module (Gazebo/가상 센서 릴레이)**
  - Gazebo UDP(자세/속도) → ExternalCtrl UDP로 원본 릴레이
  - Gazebo UDP → MAVLink OPTICAL_FLOW(100) 시리얼 송신
  - Generator Distance RAW UDP 수신 → 시리얼 변환 송신 + UI에 Current Distance 표시
  - Optical Flow / Distance 각각 Heartbeat(1Hz) 송신
- **MAVLink 시리얼 상호작용**
  - Inbound: HEARTBEAT, GIMBAL_DEVICE_SET_ATTITUDE, PARAM_REQUEST_LIST/READ
  - Outbound: HEARTBEAT(1Hz), GIMBAL_DEVICE_ATTITUDE_STATUS(10Hz), PARAM_VALUE
- **설정 저장/복구**
  - 모든 설정은 ./savedata/config.json에 원자적 저장 (임시파일→교체)

참고: 현재 형상은 “Power 체크 해제 시 즉시 TCP 중단(로컬 게이트)” 기능은 미적용입니다. (Apply Power 버튼을 눌러야 Generator에 반영됩니다.)

# 폴더 구조
unified-bridge/
├─ main.py
├─ requirements.txt
├─ core/
├─ ui/
├─ utils/
├─ savedata/        # 실행 후 자동 생성(설정/상태 저장)
├─ SaveFile/        # 실시간 캡처 이미지/프리뷰 저장 (기본 이미지 라이브러리)
└─ PreDefinedImageSet/ # 사전 정의된 JPEG 이미지 세트 (선택 시 TCP 응답 사용)

# 실행 방법
## 1) 의존성 설치 (권장: venv)
'''
 python -m venv .venv
 .\.venv\Scripts\activate
 python -m pip install --upgrade pip setuptools wheel
 pip install -r requirements.txt
'''

## 2) GUI 실행
 python main.py

## 3) Headless(무화면) 실행
 python main.py --nogui --ip 0.0.0.0 --tcp 9999 --udp 9998

인자를 주지 않거나 유효하지 않으면 콘솔에서 입력을 요청합니다.

# PyInstaller 빌드 (파이썬/라이브러리 포함 EXE)

## one-folder (콘솔 숨김):
pyinstaller --noconfirm --clean --name MroUnifiedBridge --hidden-import "pymavlink.dialects.v20.common" --hidden-import "pymavlink.dialects.v20.ardupilotmega" --hidden-import "serial.tools.list_ports" --collect-submodules "pymavlink" --collect-submodules "PIL" --collect-data "pymavlink" --collect-data "PIL" --windowed main.py
## one-file (단일 exe, 최초 실행 다소 느림):

pyinstaller --noconfirm --clean --name MroUnifiedBridge --hidden-import "pymavlink.dialects.v20.common" --hidden-import "pymavlink.dialects.v20.ardupilotmega" --hidden-import "serial.tools.list_ports" --collect-submodules "pymavlink" --collect-submodules "PIL" --collect-data "pymavlink" --collect-data "PIL" --windowed --onefile main.py

# 설정(Editable)
 Image Stream Module Settings: Bind IP, TCP Port(9999), UDP Port(9998), Image Source Mode(Realtime SaveFile / PreDefined ImageSet) 및 각 디렉터리 지정
 Gimbal Controls: Sensor Type(ID), Power(Apply Power), Max Rate, Target Pose(x, y, z, r, p, y)
 Gazebo Relay:
Gazebo Listen IP/Port (기본 0.0.0.0:17000)
ExternalCtrl UDP Out IP/Port (기본 127.0.0.1:9091)
Distance Input UDP IP/Port (기본 0.0.0.0:14650, 모드 raw)
Serial(COM) / Baud (공용, 기본 115200)
OpticalFlow 품질/스케일/패널티, Heartbeat 파라미터(OF/Distance), Auto-Start on Launch

# 기타 기능
 포터블 저장: 설정/이미지가 exe 옆 폴더에 저장
 원자적 저장: 설정 파일을 임시파일→교체 방식으로 저장
 GUI/Headless 겸용 + 프리뷰: 무화면 장비에서도 동작, GUI에선 실시간 미리보기/일시정지/스냅샷

# 빠른 점검
 프리뷰 안 뜸 → UDP 포트/방화벽/Generator 발신 IP·Port 일치 확인
 TCP 명령 안 됨 → Start Server 상태, 포트 충돌/방화벽 확인
 Distance 미표시 → Distance UDP 설정/모드(raw) 확인
 시리얼 송신 안 됨 → COM 포트 점유/속도/드라이버 확인
