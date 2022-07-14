# AMIR 740＋メガローバーF120A　MoveIt構成のパッケージ

以下のコマンドで立ち上げてください。
```
roslaunch f120a_arm_moveit_config demo.launch
```

ロボットの実機を動作する場合
```
roslaunch f120a_arm_moveit_config demo.launch moveit_controller_manager:=ros_control
```