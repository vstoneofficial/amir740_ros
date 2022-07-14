# AMIR 740＋メカナムローバーVer 3.0　MoveIt構成のパッケージ

以下のコマンドで立ち上げてください。
```
roslaunch mecanum3_arm_moveit_config demo.launch
```

ロボットの実機を動作する場合
```
roslaunch mecanum3_arm_moveit_config demo.launch moveit_controller_manager:=ros_control
```