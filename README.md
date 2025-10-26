# Mro Unified Bridge

Unreal 기반 MORAI Sim Air(MRO)와 외부 소프트웨어 사이에서 이미지를 중계하고, 센서·짐벌 제어, Gazebo UDP 릴레이, MAVLink 시리얼 변환 등을 수행하는 데스크톱 애플리케이션입니다. 영문명은 **Unified Bridge**이며, 현재는 PySide6 기반 GUI로 통합 실행됩니다. 실행 파일(또는 `main.py`)과 동일한 경로에 설정과 데이터가 저장됩니다.

## 주요 기능

- **Image Stream Module (ImageStreamBridge)**
  - UDP JPEG 수신 → 실시간 프리뷰 및 저장
  - TCP `MroCameraControl`: `Req_Capture`(최신 프레임을 `SaveFile/000.jpg` 순환 저장), `Set_Count`, `Get_ImgNum`, `Req_SendImg`, `Set_Zoomratio`/`Get_Zoomratio`(응답 `Ack_Zoomratio`) 지원

  - `./SaveFile/000.jpg`(실시간 캡처) 또는 `./PreDefinedImageSet/000.jpg`(사전 이미지) 중 UI에서 선택 가능
  - GUI 프리뷰 스냅샷 저장: `./SaveFile/preview_<timestamp>.jpg`
  - 짐벌 TCP 제어에서 전달된 디지털 줌 배율을 반영해 `Req_SendImg` JPEG를 중앙 크롭 후 리사이즈(광학 줌 대신 화상 확대)
- **Gimbal Control Module**
  - `SensorGimbalCtrl`(10706): position(double[3]) + orientation(float[4, quaternion]) 송신
  - `SensorPowerCtrl`(10707): sensor_type / id / power(0/1) 송신
  - UI에서 Sensor type/ID/Power/MaxRate/TargetPose(x, y, z, r, p, y) 편집 및 UDP Apply
  - 저장된 센서 프리셋을 개별 적용하거나 **Apply All** 버튼으로 한 번에 순차 전송(100 ms 간격)
  - TCP `GimbalControl`: 길이(4B) + `<ts_sec, ts_nsec, cmd>` 헤더로 Pose/Zoom 설정 및 상태 조회 지원 (아래 Sensor Control ICD 참고)

- **Sensor Relay Module (Gazebo/가상 센서 릴레이)**
  - Gazebo UDP(자세/속도) → ExternalCtrl UDP 원본 릴레이
  - Gazebo UDP → MAVLink `OPTICAL_FLOW`(100) 시리얼 송신
  - Generator Distance RAW UDP 수신 → 시리얼 변환 송신 + UI에 현재 거리(Current Distance) 표시
  - Optical Flow / Distance 각각 Heartbeat(1 Hz) 송신
- **MAVLink 시리얼 상호작용**
  - Inbound: `HEARTBEAT`, `GIMBAL_DEVICE_SET_ATTITUDE`, `PARAM_REQUEST_LIST`, `PARAM_REQUEST_READ`
  - Outbound: `HEARTBEAT`(1 Hz), `GIMBAL_DEVICE_ATTITUDE_STATUS`(10 Hz), `PARAM_VALUE`
- **설정 저장/복구**
  - 모든 설정은 `./savedata/config.json`에 원자적 저장(임시 파일 → 교체)

> 참고: “Power 체크 해제 시 즉시 TCP 중단(로컬 게이트)” 기능은 미적용 상태입니다. **Apply Power** 버튼을 눌러야 Generator에 반영됩니다.

## 폴더 구조

```
unified-bridge/
├─ main.py
├─ requirements.txt
├─ core/
├─ ui/
├─ utils/
├─ savedata/          # 실행 후 자동 생성(설정/상태 저장)
├─ SaveFile/          # 실시간 캡처 이미지/프리뷰 저장 (기본 이미지 라이브러리)
└─ PreDefinedImageSet/ # 사전 정의된 JPEG 이미지 세트 (선택 시 TCP 응답 사용)
```

## 실행 방법

### 1) 의존성 설치 (권장: venv)

```bash
python -m venv .venv
.\.venv\Scripts\activate  # Windows
# source .venv/bin/activate  # macOS / Linux
python -m pip install --upgrade pip setuptools wheel
pip install -r requirements.txt
```

### 2) GUI 실행

```bash
python main.py
```

### 3) GUI 모드에서 로그 확인

PyInstaller `--windowed` 빌드처럼 콘솔 창이 열리지 않는 환경에서도, 메인 GUI 우측의 **"실시간 로그 (콘솔 대체)"** 영역에서 동일한 로그를 확인할 수 있습니다. 앱 시작 직후 찍히는 초기화 로그도 자동으로 적재되며, 필요하면 `로그 복사` 버튼으로 전체 내용을 클립보드에 복사해 팀에 전달할 수 있습니다.

## PyInstaller 빌드 (파이썬/라이브러리 포함 EXE)

### One-folder (콘솔 숨김)

```bash
pyinstaller --noconfirm --clean --name MroUnifiedBridge \
  --hidden-import "pymavlink.dialects.v20.common" \
  --hidden-import "pymavlink.dialects.v20.ardupilotmega" \
  --hidden-import "serial.tools.list_ports" \
  --hidden-import "PySide6.QtCore" \
  --hidden-import "PySide6.QtGui" \
  --hidden-import "PySide6.QtWidgets" \
  --collect-submodules "pymavlink" \
  --collect-submodules "PIL" \
  --collect-submodules "PySide6" \
  --collect-data "pymavlink" \
  --collect-data "PIL" \
  --collect-data "PySide6" \
  --windowed main.py
```

### One-file (단일 exe, 최초 실행 다소 느림)

```bash
pyinstaller --noconfirm --clean --name MroUnifiedBridge \
  --hidden-import "pymavlink.dialects.v20.common" \
  --hidden-import "pymavlink.dialects.v20.ardupilotmega" \
  --hidden-import "serial.tools.list_ports" \
  --hidden-import "PySide6.QtCore" \
  --hidden-import "PySide6.QtGui" \
  --hidden-import "PySide6.QtWidgets" \
  --collect-submodules "pymavlink" \
  --collect-submodules "PIL" \
  --collect-submodules "PySide6" \
  --collect-data "pymavlink" \
  --collect-data "PIL" \
  --collect-data "PySide6" \
  --windowed --onefile main.py
```

## 설정 항목(Editable)

- **Image Stream Module**: Bind IP, TCP Port(9999), UDP Port(9998), Image Source Mode(Realtime SaveFile / PreDefined ImageSet) 및 각 디렉터리 지정
- **Gimbal Controls**: Sensor Type(ID), Power(Apply Power), Max Rate, Target Pose(x, y, z, r, p, y), Preset 저장/적용(Apply/Apply All), TCP Bind IP/Port(짐벌 수신) 및 Zoom Scale
- **Gazebo Relay**
  - Gazebo Listen IP/Port (기본 `0.0.0.0:17000`)
  - ExternalCtrl UDP Out IP/Port (기본 `127.0.0.1:9091`)
  - Distance Input UDP IP/Port (기본 `0.0.0.0:14650`, 모드 raw)
- **Serial(COM) / Baud**: 공용, 기본 115200
- **OpticalFlow**: 품질/스케일/패널티, Heartbeat 파라미터(OF/Distance), Auto-Start on Launch

## 기타 기능

- 포터블 저장: 설정/이미지가 실행 파일(.exe) 옆 폴더에 저장
- 원자적 저장: 설정 파일을 임시 파일로 작성 후 교체하여 무결성 확보
- PySide6 GUI 프리뷰: 실시간 미리보기/일시정지/스냅샷 기능 제공

## Sensor Control ICD

### UDP (Generator Forward)
- **10706 SensorGimbalCtrl**: `<uint8 sensor_type><uint8 sensor_id><float64 pos_x><float64 pos_y><float64 pos_z><float32 quat_x><float32 quat_y><float32 quat_z><float32 quat_w>` (little-endian)
- **10707 SensorPowerCtrl**: `<uint16 sensor_type><uint16 sensor_id><uint8 power_on>`

### TCP `MroCameraControl` Command Set
- Frame: `<uint32 payload_len>` prefix + payload (`payload_len` bytes)
- Payload header: `<uint32 ts_sec><uint32 ts_nsec><uint8 cmd_id>` (little-endian)

| Cmd ID | 이름 | Payload 구조 | 비고 |
| ------ | ---- | ------------ | ---- |
| `0x01` | Req_Capture | `<uint8 capture=1>` | 최신 이미지(UDP 수신본)를 SaveFile 디렉터리에 저장하고 파일 번호를 +1. |
| `0x02` | Set_Gimbal | `<float x, y, z, roll, pitch, yaw>` | 이미지 스트림 설정에 지정된 센서 유형/ID로 짐벌 제어 모듈에 전달되어 UDP SensorGimbalCtrl 패킷을 즉시 발사합니다. |
| `0x03` | Set_Count | `<uint32 count_num>` | 다음 저장될 이미지 번호를 설정(000~999 순환). |
| `0x04` | Get_ImgNum | `<uint8 get_flag=1>` | 마지막으로 저장된 이미지 번호 질의. `Img_Num_Response`(0x11) 반환. |
| `0x05` | Req_SendImg | `<uint32 img_num>` | 지정 번호 이미지를 TCP 전송. `File_ImgTransfer`(0x12) 응답. |
| `0x06` | Set_Zoomratio | `<float zoom_ratio>` | 디지털 줌 배율 설정. 적용된 배율은 `Ack_Zoomratio`(0x13)로 회신. |
| `0x07` | Get_Zoomratio | `<uint8 get_flag=1>` | 현재 줌 배율 질의. `Ack_Zoomratio` 응답. |
| `0x11` | Img_Num_Response | `<uint32 ack_uuid><uint32 img_num>` | 마지막 저장 번호 응답(기존 동작 유지). |
| `0x12` | File_ImgTransfer | `<uint32 ack_uuid><uint32 img_num><uint32 data_size><byte[] data>` | JPEG 바이너리 응답(줌 1.0 초과 시 중앙 크롭 후 리사이즈). |
| `0x13` | Ack_Zoomratio | `<float zoom_ratio>` | Set/Get 요청에 대한 현재 줌 배율 회신. |

> `zoom_ratio` 는 디지털 확대 배율(1.0 = 원본)이며, Pillow 가 사용 가능하면 JPEG 중앙부를 crop 후 원 해상도로 리사이즈하여 적용합니다.

### TCP `GimbalControl` Command Set
- Framing: `<uint32 payload_len>` prefix + payload (`payload_len` bytes)
- Payload header: `<uint32 ts_sec><uint32 ts_nsec><uint8 cmd_id>`
- All values are little-endian. Responses use the same header.

| Cmd ID | 이름 | Payload 구조 | 비고 |
| ------ | ---- | ------------ | ---- |
| `0x01` | Set_TargetPose | `<int16 sensor_type><int16 sensor_id><float64 pos_x><float64 pos_y><float64 pos_z><float32 roll_deg><float32 pitch_deg><float32 yaw_deg>` | UDP 루프의 목표 포즈를 갱신. 성공 시 `Status`(0x81) 응답. |
| `0x02` | Set_Zoom | `<float32 zoom_scale>` (1.0 이상) | 디지털 줌 배율을 설정. ImageStreamBridge가 TCP 이미지 송신 시 동일 배율로 중앙 크롭/리사이즈. 성공 시 `Status` 응답. |
| `0x80` | Get_Status | (없음) | 현재 상태 보고를 요청. 즉시 `Status` 응답. |
| `0x81` | Status | `<int16 sensor_type><int16 sensor_id><float64 pos_x><float64 pos_y><float64 pos_z><float32 cur_roll><float32 cur_pitch><float32 cur_yaw><float32 tgt_roll><float32 tgt_pitch><float32 tgt_yaw><float32 zoom_scale><float32 max_rate_dps>` | 서버→클라이언트 전용. 현재/목표 RPY, 위치, 줌 배율, 최대 속도 포함. |

> `zoom_scale` 은 디지털 확대 배율(1.0 = 원본)로, TCP 카메라 응답에도 즉시 반영되며 `Status` 응답으로 조회할 수 있습니다.

## 빠른 점검

- 프리뷰가 표시되지 않음 → UDP 포트/방화벽/Generator 발신 IP·Port 일치 확인
- TCP 명령 미동작 → Start Server 상태, 포트 충돌/방화벽 확인
- Distance 미표시 → Distance UDP 설정/모드(raw) 확인
- 시리얼 송신 불가 → COM 포트 점유/속도/드라이버 확인
