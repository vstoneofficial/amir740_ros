# AMIR 740 台車用ロボットアーム ROS対応SDK

ヴイストン株式会社より発売されている台車用ロボットアーム「AMIR 740」をROSで制御するためのSDKです。 別途Linux搭載のPC及びロボット実機が必要になります。

## 各パッケージの説明
- amir_control
  - AMIR 740をROS Controlで制御するパッケージ
- amir_description
  - AMIR740のモデルを表示するためのパッケージ
- amir_motion
  - AMIR 740のサンプルモーション（MoveItへ指令を出すノード）のパッケージ
- amir_moveit_config
  - AMIR 740　MoveItの設定パッケージ
- f120a_arm_moveit_config
  - メガローバー F120Aに搭載したAMIR 740 MoveItの設定パッケージ
- mecanum3_arm_moveit_config
  - メカナムローバー Ver3.0に搭載したAMIR 740 MoveItの設定パッケージ
- mega3_arm_moveit_config
  - メガローバー Ver3.0に搭載したAMIR 740 MoveItの設定パッケージ
- ros_control_boilerplate
  - ROS_controlのハードウェアインターフェイスをセットアップするためのテンプレート