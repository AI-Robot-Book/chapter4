# path_planning
経路計画のサンプルコード．現時点ではROS2のパッケージではない．

## 実行方法
コードのある場所に移動する．
~~~
cd ~/airobot_ws/src/chapter4/search
~~~
端末を３分割して次のコマンドを実行する．
~~~
python3 path_planning.py 1      # 幅優先探索
pyhton3 path_planning.py 2      # ダイクストラ法
python3 path_planning.py 3      # A*アルゴリズム
~~~
