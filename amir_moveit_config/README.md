# AMIR 740　MoveIt構成のパッケージ

## MoveIt
以下のコマンドでRViz上でMoveItを操作するデモを立ち上げてください。
```
roslaunch amir_moveit_config demo.launch
```

## Gazeboシミュレータ
まずは、（グリッパの平行リンクを動作するため）以下のリポジトリをクローンしてください
```
git clone https://github.com/roboticsgroup/roboticsgroup_upatras_gazebo_plugins.git
```
次のコマンドでGazeboでAMIR 740のシミュレーションを起動します。
```
roslaunch amir_moveit_config　gazebo.launch
```
MoveItで操作するGazeboのシミュレーションは次のコマンドで立ち上げてください。
```
roslaunch amir_moveit_config　demo_gazebo.launch
```