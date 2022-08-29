# amcl_subscriber
## 概要
４章のサンプルプログラム  
/amcl_poseと/odomのトピックをサブスクライブして，それから姿勢を取り出して端末に表示するプログラム


## インストール
Chapter4のパッケージは全部まとめてインストール・ビルドをします．
- [第4章 インストール](https://github.com/AI-Robot-Book/chapter4)を参照してください．


## 実行
端末2を3分割して，上の端末で以下のコマンドでシミュレータを起動
```
cd ~/airobot_ws
source install/setup.bash
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py  
```

中の端末で以下のコマンドを実行
```
cd ~/airobot_ws
source install/setup.bash
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=$HOME/map/
house.yaml
```

下の端末で以下のコマンドを実行してロボットを移動させ，トピックの値を確認する．
```
cd ~/airobot_ws
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```


## ヘルプ
トラブルシュートや気を付けたいことなどを書く．
　
 
## 著者
出村 公成


## 履歴
バグフィクスやチェンジログ
- 2022-08-23: READMEの修正
- 2022-03-15: 初期版


## ライセンス
Apach 2.0 


## 参考文献
- 今のところなし．
