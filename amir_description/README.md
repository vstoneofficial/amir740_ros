# AMIR 740のモデルを表示するためのパッケージ

以下のコマンドで立ち上げます。
```
roslaunch amir_description display.launch
```
以下の画面が表示します。
![](../images/amir-description-rviz.png)

メガローバーF120Aに取りつけた場合は
```
roslaunch amir_description display.launch rover_type:=f120a
```
![](../images/amir-f120a-description-rviz.png)

メガローバーVer3.0に取りつけた場合は
```
roslaunch amir_description display.launch rover_type:=mega3
```
![](../images/amir-mega3-description-rviz.png)

メカナムローバーVer3.0に取りつけた場合は
```
roslaunch amir_description display.launch rover_type:=mecanum3
```
![](../images/amir-mecanum3-description-rviz.png)


実機をROSデバイスと接続した上で、下記のノードを立ち上げると、AMIR 740をGUI上のスライダで動作可能

```
rosrun amir_control joint_state_relay
```


> **Warning**
> 実機のAMIR 740を接続した後、RViz画面上のポーズに動き出すので、十分注意してください。 