## AMIR 740 台車用ロボットアーム ROSパッケージ

<p align="center">
  <img src="./images/amir-1.png" width="600" />
</p>

ヴイストン株式会社より発売されている台車用ロボットアーム「AMIR 740」をROSで制御するためのSDKです。別途Linux搭載のPC及びロボット実機が必要になります。

AMIR 740については、[製品ページ](https://www.vstone.co.jp/products/amir740/index.html)をご覧ください。

## 目次
- [インストール方法](#インストール方法)
- [各パッケージの説明と利用方法](#各パッケージの説明と利用方法)
    - [amir\_control](#amir_control)
    - [amir\_description](#amir_description)
    - [amir\_motion](#amir_motion)
    - [amir\_moveit\_config](#amir_moveit_config)
      - [MoveIt](#moveit)
      - [Gazeboシミュレータ](#gazeboシミュレータ)
    - [f120a\_arm\_moveit\_config](#f120a_arm_moveit_config)
    - [mecanum3\_arm\_moveit\_config](#mecanum3_arm_moveit_config)
    - [mega3\_arm\_moveit\_config](#mega3_arm_moveit_config)

# インストール方法

- ROSのインストール方法は[こちら](http://wiki.ros.org/noetic/Installation/Ubuntu)を確認してください。
- catkin ワークスペースを作成
  ```bash
  mkdir -p ~/amir_ws/src
  ```
- ワークスペースの`src`フォルダ内に移動し、本パッケージをクローンします。
  ```bash
  cd ~/amir_ws/src
  git clone https://github.com/vstoneofficial/amir740_ros.git
  ```
- Gazebo上でAMIR 740を動作したい場合、グリッパを操作するための[mimic_joint_plugin](https://github.com/roboticsgroup/roboticsgroup_upatras_gazebo_plugins)が必要です。
  ```bash
  git clone https://github.com/roboticsgroup/roboticsgroup_upatras_gazebo_plugins.git
  ```
- 台車ロボットのモデルを表示したい場合、台車ロボット関連のパッケージをクローンします。
  ```bash
  git clone https://github.com/vstoneofficial/megarover_description.git # メガローバー
  git clone https://github.com/vstoneofficial/mecanumrover_description.git　# メカナムローバー
  git clone https://github.com/vstoneofficial/vs_rover_options_description.git　# 台車オプション
  ```
- 依存関係にあるパッケージをインストールします。
  ```bash
  rosdep install -r -y --from-paths . --ignore-src
  ```
- ソースコードをビルドします。
  ```bash
  cd ~/amir_ws
  catkin_make
  ```
- ワークスペースのオーバレイ作業
  ```bash
  source devel/setup.bash
  ```

以上でamir740_rosパッケージのセットアップは完了です。

# 各パッケージの説明と利用方法
### amir_control
AMIR 740をROS Controlで制御するパッケージ。\
以下のコマンドでAMIR 740をROSと接続。
```
roslaunch amir_control rosserial.launch
```

以下のコマンドでAMIR 740をros_controlで制御する。
```
roslaunch amir_control amir_control_HW.launch
```

実機を操作する前に、Rviz上で動作確認のためにこちらのノードを起動してください。

> **Warning**\
> 実機のAMIR 740を操作する時に起動しないでください。

```
rosrun amir_control amir_sim_echo
```

### amir_description
AMIR740のモデルを表示するためのパッケージ。\
以下のコマンドで立ち上げます。
```
roslaunch amir_description display.launch
```
以下の画面が表示します。
![](images/amir-description-rviz.png)

- メガローバーF120Aに取りつけた場合は
```bash
roslaunch amir_description display.launch rover_type:=f120a
```
![](images/amir-f120a-description-rviz.png)

- メガローバーVer3.0に取りつけた場合は
```bash
roslaunch amir_description display.launch rover_type:=mega3
```
![](images/amir-mega3-description-rviz.png)

- メカナムローバーVer3.0に取りつけた場合は
```bash
roslaunch amir_description display.launch rover_type:=mecanum3
```
![](images/amir-mecanum3-description-rviz.png)


実機をROSデバイスと接続した上で、下記のノードを立ち上げると、AMIR 740をGUI上のスライダで動作可能

```
rosrun amir_control joint_state_relay
```

> **Warning**\
> 実機のAMIR 740を接続した後、RViz画面上のポーズに動き出すので、十分注意してください。 

### amir_motion
AMIR 740のサンプルモーション（MoveItへ指令を出すノード）のパッケージ。\
各ノードの詳細は`src`フォルダーの各ファイルに記載されています。

### amir_moveit_config
AMIR 740　MoveItの設定パッケージ
#### MoveIt
以下のコマンドでRViz上でMoveItを操作するデモを立ち上げてください。
```
roslaunch amir_moveit_config demo.launch
```

#### Gazeboシミュレータ
以下のコマンドでGazeboでAMIR 740のシミュレーションを起動します。
```
roslaunch amir_moveit_config gazebo.launch
```
MoveItで操作するGazeboのシミュレーションは次のコマンドで立ち上げてください。
```
roslaunch amir_moveit_config demo_gazebo.launch
```

### f120a_arm_moveit_config
メガローバー F120Aに搭載したAMIR 740 MoveItの設定パッケージ。\
以下のコマンドで立ち上げてください。
```
roslaunch f120a_arm_moveit_config demo.launch
```

ロボットの実機を動作する場合
```
roslaunch f120a_arm_moveit_config demo.launch moveit_controller_manager:=ros_control
```
### mecanum3_arm_moveit_config
メカナムローバー Ver3.0に搭載したAMIR 740 MoveItの設定パッケージ。\
以下のコマンドで立ち上げてください。
```
roslaunch mecanum3_arm_moveit_config demo.launch
```

ロボットの実機を動作する場合
```
roslaunch mecanum3_arm_moveit_config demo.launch moveit_controller_manager:=ros_control
```

### mega3_arm_moveit_config
メガローバー Ver3.0に搭載したAMIR 740 MoveItの設定パッケージ。\
以下のコマンドで立ち上げてください。
```
roslaunch mega3_arm_moveit_config demo.launch
```

ロボットの実機を動作する場合
```
roslaunch mega3_arm_moveit_config demo.launch moveit_controller_manager:=ros_control
```
