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

## 変更履歴
### 1.1.0 (2022-08-17)
- Gazeboシミュレータ機能を実装(詳細は[amir_moveit_configのREADME.md](amir_moveit_config/README.md)参照)
- AMIR 740の物理的パラメータの調整
- グリッパのジョイント名の変更

### 1.0.0 (2022-07-14)
- 初版