int32 CHARGE_MODE=0 # 스테이션에서 충전 및 데이터 전송모드, low_mode_etc 패키지에 구현
int32 STANDBY_MODE=1 # 스테이션에서 대기하고 있는 모드, 별도 세부구현 없음
int32 DOCKING_MODE=2 # 스테이션에 DOCKING 하는 모드, docking 패키지에 구현
int32 DOCK_IN_MODE=3 # 스테이션 유도로를 따라 진입하는 모드, docking 패키지에 구현
int32 DOCK_OUT_MODE=4 # 스테이션 유도로를 따라 나가는 모드, docking 패키지에 구현
int32 DOCK_OUT_PRE_MODE=5 # STANDBY에서 DOCK_OUT 변경 시, 스테이션 문 열기 등 사전 작업하는 모드, docking 패키지에 구현
int32 DOCK_IN_PRE_MODE=6 # AUTO_MODE에서 DOCK_IN 변경 시, 초기에 위치를 잡기 위한 모드, docking 패키지에 구현
int32 AUTO_PRE_IMAGE_MODE=7 # AUTO_MODE로 변경하기 전, 초기 길 방향을 찾는 모드(영상), self_driving 패키지에 구현
int32 AUTO_PRE_LIDAR_MODE=8 # AUTO_MODE로 변경하기 전, 초기 길 방향을 찾는 모드(라이다)
int32 AUTO_IMAGE_MODE=9 # 자율주행 모드(영상), self_driving 패키지에 구현
int32 AUTO_LIDAR_MODE=10 # 자율주행 모드(라이다)
int32 MANUAL_MODE=11 # 수동조작 모드, 별도 세부구현 없음
int32 STOP_MODE=12 # 중지모드로, 기본모드를 MANUAL MODE로 하면 의도치 않게 조종될수 있어, 이를 방지하기 위함
int32 TURN_MODE=13 # 제자리 회전 모드, 방향 전환용 180도 회전, low_mode_etc 패키지에 구현