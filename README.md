# 第４章　ナビゲーション
## 概要
ROS2とPythonで作って学ぶAIロボット入門（出村・萩原・升谷・タン著，講談社）第４章のサンプルプログラムと補足情報などを掲載しています．

## ディレクトリ構成
- **amcl_subscriber**: /amclをサブスクライブするパッケージ
- **happy_lidar**: シンプルなLidarパッケージとドアオープンのサンプル
- **happy_move**: シンプルな移動パッケージ
- **happy_teleop**: 自作遠隔操作パッケー
- **map**: 地図ファイルを格納するディレクトリ
- **path_planning**: 経路計画のサンプルプログラム
- **waypoint_navi**: ウェイポイントナビゲーションのサンプルパッケージ

## インストール
Chapter4の全パッケージを以下のコマンドでインストールします．
- ROSのワークスペースを`~/airobot_ws`とする．
  ```
  cd ~/airobot_ws/src
  ```

- Chapter4のリポジトリを入手
  ```
  git clone https://github.com/AI-Robot-Book/chapter4
  ```

## 補足情報
- ナビゲーション関連のサンプルプログラムを実行するときは，以下の説明に従って事前に地図ファイルをインストールしてください．  
-- [https://github.com/AI-Robot-Book/chapter4/tree/main/map](https://github.com/AI-Robot-Book/chapter4/tree/main/map)
