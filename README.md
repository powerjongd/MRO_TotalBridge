Mro Total Bridge

Unreal 기반 Morai Sim: Air(MRO) 와 외부 Software 사이에서 이미지를 중계하고, 센서/짐벌 제어, Gazebo UDP 릴레이, MAVLink 시리얼 변환 등을 수행하는 데스크톱 앱입니다.
GUI / Headless(무화면) 모두 지원하며, 설정과 데이터는 실행 파일(또는 main.py) 바로 아래에 저장합니다.

✨ 주요 기능

UDP JPEG 수신 → 미리보기/저장

최신 프레임을 실시간 프리뷰, 수동 저장(./images/preview_<timestamp>.jpg)

MroCameraControl (TCP) 서비스

0x01 Req_Capture (uint8=1) → 최신 프레임 저장(000~999 순환)

0x03 Set_Count (uint32) → 다음 저장 번호 설정

0x04 Get_ImgNum (uint8=1) → 0x11 Img_Num_Response 회신

0x05 Req_SendImg (uint32) → 0x12 File_ImgTransfer 회신(파일/미존재 0)

센서/짐벌 제어 (Bridge → Generator, UDP)

SensorGimbalCtrl(10706): position(double[3]) + orientation(float[4, quaternion])

SensorPowerCtrl(10707): sensor_type / id / power(0/1)

UI에서 Sensor type/ID/Power/MaxRate/TargetPose(xyzrpy) 편집 가능

Gazebo UDP 릴레이 + 시리얼 변환

Gazebo UDP(자세/속도) → ExternalCtrl UDP로 원본 릴레이

Gazebo UDP → MAVLink OPTICAL_FLOW(100) 시리얼 송신(옵션)

Generator Distance RAW UDP 수신 → 시리얼 변환 송신 + UI에 Current Distance 표시

Optical Flow / Distance 각각 Heartbeat(1Hz) 송신

MAVLink 시리얼 상호작용(옵션)

Inbound: HEARTBEAT, GIMBAL_DEVICE_SET_ATTITUDE, PARAM_REQUEST_LIST/READ

Outbound: HEARTBEAT(1Hz), GIMBAL_DEVICE_ATTITUDE_STATUS(10Hz), PARAM_VALUE

설정 저장/복구

모든 설정은 ./savedata/config.json에 원자적 저장 (임시파일→교체)

참고: 현재 형상은 “Power 체크 해제 시 즉시 TCP 중단(로컬 게이트)” 기능은 미적용입니다. (Apply Power 버튼을 눌러야 Generator에 반영됩니다.)

📁 폴더 구조
image-bridge/
├─ main.py
├─ requirements.txt
├─ core/
├─ ui/
├─ utils/
├─ savedata/        # 실행 후 자동 생성(설정/상태 저장)
└─ images/          # 미리보기/캡처 이미지 저장

🚀 실행 방법
1) 의존성 설치 (권장: venv)
python -m venv .venv
.\.venv\Scripts\activate
python -m pip install --upgrade pip setuptools wheel
pip install -r requirements.txt

2) GUI 실행
python main.py

3) Headless(무화면) 실행
python main.py --nogui --ip 0.0.0.0 --tcp 9999 --udp 9998


인자를 주지 않거나 유효하지 않으면 콘솔에서 입력을 요청합니다.

🔧 PyInstaller 빌드 (파이썬/라이브러리 포함 EXE)

one-folder (권장, 빠르고 안정적 / 아이콘 없음 / 콘솔 숨김):

pyinstaller --noconfirm --clean --name ImageBridge --hidden-import "pymavlink.dialects.v20.common" --hidden-import "pymavlink.dialects.v20.ardupilotmega" --hidden-import "serial.tools.list_ports" --collect-submodules "pymavlink" --collect-submodules "PIL" --collect-data "pymavlink" --collect-data "PIL" --windowed main.py


one-file (단일 exe, 최초 실행 다소 느림):

pyinstaller --noconfirm --clean --name ImageBridge --hidden-import "pymavlink.dialects.v20.common" --hidden-import "pymavlink.dialects.v20.ardupilotmega" --hidden-import "serial.tools.list_ports" --collect-submodules "pymavlink" --collect-submodules "PIL" --collect-data "pymavlink" --collect-data "PIL" --windowed --onefile main.py


빌드 결과: dist/ImageBridge/ImageBridge.exe

⚙️ 설정(Editable)

Bridge Settings: Bind IP, TCP Port(9999), UDP Port(9998)

Gimbal Controls: Sensor Type(ID), Power(Apply Power), Max Rate, Target Pose(x, y, z, r, p, y)

Gazebo Relay:

Gazebo Listen IP/Port (기본 0.0.0.0:17000)

ExternalCtrl UDP Out IP/Port (기본 127.0.0.1:9091)

Distance Input UDP IP/Port (기본 0.0.0.0:14650, 모드 raw)

Serial(COM) / Baud (공용, 기본 115200)

OpticalFlow 품질/스케일/패널티, Heartbeat 파라미터(OF/Distance), Auto-Start on Launch

🧰 유틸리티 장점

포터블 저장: 설정/이미지가 exe 옆 폴더에 저장 → 배포/백업 쉬움

원자적 저장: 설정 파일을 임시파일→교체 방식으로 저장 → 손상 방지

GUI/Headless 겸용 + 프리뷰: 무화면 장비에서도 동작, GUI에선 실시간 미리보기/일시정지/스냅샷

🧪 빠른 점검

프리뷰 안 뜸 → UDP 포트/방화벽/Generator 발신 IP·Port 일치 확인

TCP 명령 안 됨 → Start Server 상태, 포트 충돌/방화벽 확인

Distance 미표시 → Distance UDP 설정/모드(raw) 확인

시리얼 송신 안 됨 → COM 포트 점유/속도/드라이버 확인