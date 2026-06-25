

  '''
  sudo apt install ros-humble-usb-cam
  '''
  
usb-cam 패키지 없으면 이거부터 깔아야함



# idle_vision

일반 USB RGB 카메라로 작업대 위 물체를 인식하는 ROS2 패키지입니다.

현재 운영 기준은 USB 카메라 한 대로 RGB 블럭과 바구니를 검출하고,
픽셀-작업대 homography를 이용해 base 기준 `x`, `y`, `yaw`를 publish하는 구조입니다.
카메라가 고정되어 있고 물체도 정지한 상태에서 좌표를 안정적으로 읽는 것을 우선합니다.

## 기능

- USB RGB 카메라 실행
- 빨강/초록/파랑 블럭 검출
- 바구니 검출
- 회전 bounding box, 중심 픽셀, yaw 추정
- homography 기반 base 좌표 `x_m`, `y_m`, `z_m`, `yaw` publish
- HSV 튜닝용 ROI 통계 publish
- 선택 사항: Whisper STT + Ollama/Qwen 명령 파싱

## 빌드

```bash
cd ~/idle_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select idle_vision
source install/setup.bash
```

```bash
mkdir -p ~/idle_ws/src
cd ~/idle_ws/src
# 여기에 idle_vision 패키지 clone/copy

cd ~/idle_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src/idle_vision --ignore-src -r -y
colcon build --packages-select idle_vision
source install/setup.bash
```

## 최종 실행

카메라, 박스/바구니 검출, rqt 확인 창을 한 번에 실행합니다.

```bash
ros2 launch idle_vision usb_rgb_box_pose_rqt.launch.py
```

이 명령의 기본값은 현장 운영 기준으로 맞춰져 있습니다.

- 카메라 장치: `IDLE_VISION_VIDEO_DEVICE` 환경변수가 있으면 그 값, 없으면 `/dev/video2`
- 해상도: `1280x720`
- pixel format: `mjpeg2rgb`
- 자동 화이트밸런스: 꺼짐
- 자동 노출: 꺼짐
- 검출 색상: `scene`, 즉 `red`, `green`, `blue`, `basket`
- 좌표계: 저장된 homography로 base 기준 좌표 publish
- smoothing: 고정 물체 좌표 안정화를 위해 느리지만 차분한 값 사용

출력 확인:

```bash
ros2 topic echo --full-length /idle_vision/box_poses
```

자주 보는 토픽:

- `/image_raw`: USB 카메라 원본 이미지
- `/camera_info`: USB 카메라 정보
- `/idle_vision/box_poses`: 감지된 물체 목록 JSON
- `/idle_vision/box_pose_array`: 감지된 물체 `PoseArray`
- `/idle_vision/box_pose/debug_image`: rqt에서 보는 overlay 이미지
- `/idle_vision/box_pose/mask`: 색 mask 이미지
- `/idle_vision/box_pose/markers`: 시각화용 marker

## 최종 launch 구성

`usb_rgb_box_pose_rqt.launch.py`는 아래 node를 실행합니다.

1. `usb_cam/usb_cam_node_exe`
   - node name: `usb_rgb_camera`
   - USB 카메라를 열고 `/image_raw`, `/camera_info`를 publish합니다.

2. `idle_vision/box_pose_node`
   - node name: `usb_rgb_box_pose_node`
   - `/image_raw`를 받아 HSV 기반으로 RGB 블럭과 바구니를 검출합니다.
   - homography가 있으면 픽셀 중심을 base 기준 `x_m`, `y_m`으로 변환합니다.
   - `/idle_vision/box_poses`, `/idle_vision/box_pose_array`,
     `/idle_vision/box_pose/debug_image`, `/idle_vision/box_pose/mask`,
     `/idle_vision/box_pose/markers`를 publish합니다.

3. `rqt_image_view`
   - `/idle_vision/box_pose/debug_image`를 띄웁니다.

4. `rqt_image_view`
   - `/image_raw`를 띄웁니다.

즉 평소에는 아래 하나만 실행하면 됩니다.

```bash
ros2 launch idle_vision usb_rgb_box_pose_rqt.launch.py
```

## 카메라 장치 설정

USB 카메라 번호는 부팅이나 포트에 따라 바뀔 수 있습니다.
팀원 PC에서는 먼저 장치를 확인합니다.

```bash
ls -l /dev/v4l/by-id/
```

가능하면 `/dev/video2` 같은 번호보다 `/dev/v4l/by-id/...` 경로를 사용합니다.
한 터미널에서만 임시로 쓸 때:

```bash
export IDLE_VISION_VIDEO_DEVICE=/dev/v4l/by-id/usb-046d_0825_D087B1E0-video-index0
```

매번 설정하기 싫으면 `~/.bashrc`에 저장합니다.

```bash
echo 'export IDLE_VISION_VIDEO_DEVICE=/dev/v4l/by-id/usb-046d_0825_D087B1E0-video-index0' >> ~/.bashrc
```

그 다음부터는 실행 명령에 `video_device:=...`를 붙이지 않아도 됩니다.

```bash
ros2 launch idle_vision usb_rgb_box_pose_rqt.launch.py
```

## 좌표 변환

일반 RGB 카메라는 픽셀 좌표만 직접 얻을 수 있으므로,
작업대 평면 위의 점들을 이용해 `pixel(u, v) -> base(x, y)` homography를 사용합니다.

현재 launch에는 최근 카메라 위치에서 계산한 homography가 기본 저장되어 있습니다.
카메라 위치와 해상도가 그대로라면 실행 명령에 행렬을 붙일 필요가 없습니다.

```bash
ros2 launch idle_vision usb_rgb_box_pose_rqt.launch.py
```

저장된 변환 행렬:

```text
[[0.0009504612,-2.1327e-06,-0.5866006127],
 [1.9451e-06,-0.0009616124,0.928124009],
 [-6.2509e-06,-2.12835e-05,1.0]]
```

최근 8점 calibration 결과:

```text
(0.305, 0.355) -> center_px [933.542, 604.765]
(0.305, 0.755) -> center_px [935.000, 193.000]
(0.105, 0.605) -> center_px [726.500, 342.500]
(0.105, 0.755) -> center_px [730.000, 186.000]
(-0.105, 0.605) -> center_px [505.889, 343.483]
(-0.105, 0.755) -> center_px [506.500, 186.500]
(-0.305, 0.355) -> center_px [303.500, 603.000]
(-0.305, 0.755) -> center_px [299.500, 186.000]
```

계산 오차:

```text
rms_error_m = 0.00225
max_error_m = 0.00349
```

카메라 위치가 바뀌면 homography를 다시 잡아야 합니다.
평소 실행 명령에 긴 JSON을 붙이지 말고, 새 행렬이 안정적으로 맞으면
`launch/usb_rgb_box_pose_rqt.launch.py`의 `DEFAULT_PLANE_HOMOGRAPHY_JSON` 기본값을 갱신합니다.

### Homography 다시 잡는 순서

먼저 최종 launch를 켭니다.

```bash
ros2 launch idle_vision usb_rgb_box_pose_rqt.launch.py
```

실제 base 좌표를 알고 있는 위치에 블럭을 놓고 중심 픽셀을 읽습니다.

```bash
ros2 topic echo --once --full-length /idle_vision/box_poses
```

출력 JSON에서 `center_px`를 기록합니다. 최소 4점, 가능하면 6~9점을 화면 전체에 퍼지게 찍습니다.
예시 파일:

```json
{
  "points": [
    {"pixel": [613.0, 343.0], "base": [0.20, 0.10]},
    {"pixel": [780.0, 340.0], "base": [0.20, -0.10]},
    {"pixel": [620.0, 520.0], "base": [0.40, 0.10]},
    {"pixel": [790.0, 515.0], "base": [0.40, -0.10]}
  ]
}
```

계산:

```bash
ros2 run idle_vision compute_homography \
  --points-file /tmp/usb_rgb_homography_points.json \
  --output /tmp/usb_rgb_homography.json
```

출력된 `rms_error_m`, `max_error_m`가 작으면 계산된 3x3 행렬을 launch 기본값에 저장합니다.

## 색상 검출

기본 실행은 `scene` 모드입니다.

- `red`: 빨간 블럭
- `green`: 초록 블럭
- `blue`: 파란 블럭
- `basket`: 바구니

특정 색만 확인하고 싶을 때만 launch argument를 붙입니다.

```bash
ros2 launch idle_vision usb_rgb_box_pose_rqt.launch.py target_color:=basket
ros2 launch idle_vision usb_rgb_box_pose_rqt.launch.py target_color:=auto
```

`auto`는 `red`, `green`, `blue`만 봅니다.

현재 HSV preset은 자동 화이트밸런스와 자동 노출을 끈 상태에서 튜닝했습니다.

- 빈바닥 샘플: median `H 95-96, S 56-59, V 159-168`
- `red`: median `H 177, S 210-211, V 248-249`
- `green`: median `H 38-39, S 160-162, V 156-158`
- `blue`: median `H 96-97, S 154-155, V 255`
- `basket`: median `H 19, S 79, V 197`

바구니는 무늬가 끊겨 잡히는 경우가 있어서 블럭과 다르게 close morphology 위주로 mask를 이어붙입니다.
또한 작은 조각이나 프로파일 모서리를 줄이기 위해 면적과 비율 조건을 같이 봅니다.

주요 기본값:

- `basket_morph_close_kernel_size:=51`
- `basket_min_area_px:=6000`
- `basket_max_area_px:=150000`
- `basket_min_bbox_width_px:=50`
- `basket_min_bbox_height_px:=70`
- `basket_min_aspect_ratio:=1.05`
- `basket_max_aspect_ratio:=5.0`
- `pose_smoothing_alpha:=0.08`
- `yaw_smoothing_alpha:=0.05`

고정 물체 좌표를 안정적으로 읽기 위해 smoothing은 느리지만 차분하게 설정되어 있습니다.

## HSV 튜닝

HSV 값을 다시 찍을 때는 튜너 launch를 사용합니다.

```bash
ros2 launch idle_vision usb_rgb_hsv_tuner.launch.py
```

이미 최종 launch에서 카메라가 켜져 있으면 카메라를 중복으로 열지 않고 튜너만 켭니다.

```bash
ros2 launch idle_vision usb_rgb_hsv_tuner.launch.py start_camera:=false
```

상태 JSON:

```bash
ros2 topic echo /idle_vision/hsv_tuner/status
```

rqt로 ROI 확인:

```bash
  ros2 run rqt_image_view rqt_image_view /idle_vision/hsv_tuner/debug_image
```

`status` JSON에서 주로 보는 값:

- `sample_px`: 샘플링 픽셀
- `roi_xyxy`: ROI 범위
- `hsv.min`
- `hsv.median`
- `hsv.p05`
- `hsv.p95`
- `hsv.max`
- `suggested_hsv_ranges_json`

## 음성/Qwen 선택

카메라와 좌표 추출은 최종 launch로 켜고, 음성/Qwen은 별도 launch로 분리해서 켭니다.

터미널 1:

```bash
ros2 launch idle_vision usb_rgb_box_pose_rqt.launch.py
```

터미널 2:

```bash
ollama serve
```

터미널 3:

```bash
ros2 launch idle_vision qwen_voice_selector.launch.py
```

Qwen launch 구성:

1. `idle_vision/qwen_box_selector_node`
   - `/idle_vision/box_poses`에서 현재 물체 목록을 읽습니다.
   - 자연어 명령을 색상/위치 조건으로 파싱합니다.
   - 선택된 물체를 `/idle_vision/qwen/selected_box`,
     `/idle_vision/qwen/target_pose`로 publish합니다.

2. `idle_vision/voice_command_node`
   - 마이크 입력을 Whisper로 텍스트화합니다.
   - 결과를 `/idle_vision/voice/transcript`,
     `/idle_vision/qwen/command`로 publish합니다.

출력 확인:

```bash
ros2 topic echo --full-length /idle_vision/qwen/parsed_command
ros2 topic echo --full-length /idle_vision/qwen/selected_box
ros2 topic echo /idle_vision/qwen/target_pose
ros2 topic echo /idle_vision/voice/transcript
```

필요 패키지:

```bash
pip install faster-whisper sounddevice
ollama pull qwen2.5:7b
```

## 코드 구성

- `idle_vision/box_pose_node.py`: HSV 검출, yaw 안정화, homography 좌표 publish
- `idle_vision/color_segmentation.py`: HSV preset, 색상 quality check, target color parser
- `idle_vision/hsv_tuner_node.py`: ROI HSV 통계 publish
- `idle_vision/compute_homography.py`: pixel-base homography 계산 CLI
- `idle_vision/qwen_box_selector_node.py`: 자연어 명령 파싱 및 목표 물체 선택
- `idle_vision/voice_command_node.py`: Whisper STT 입력 node
- `idle_vision/vision_utils.py`: timestamp 등 공통 helper

## 문제 확인

카메라가 열리지 않을 때:

```bash
ls -l /dev/v4l/by-id/
```

다른 프로그램이 카메라를 잡고 있으면 먼저 종료합니다.

박스가 안 잡힐 때:

```bash
ros2 launch idle_vision usb_rgb_hsv_tuner.launch.py start_camera:=false
ros2 topic echo /idle_vision/hsv_tuner/status
```

좌표가 이상할 때:

- 카메라 위치가 바뀌었는지 확인
- 해상도가 `1280x720`인지 확인
- homography를 다시 계산

bounding box가 떨릴 때:

- 현재 기본 smoothing은 안정형입니다.
- 더 안정적으로 보고 싶으면 물체를 놓고 1~2초 기다린 뒤 `/idle_vision/box_poses`를 읽습니다.
- 빠르게 움직이는 물체를 따라가야 할 때만 smoothing alpha를 키웁니다.
