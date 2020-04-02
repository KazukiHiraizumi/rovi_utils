# RoVI Utilities

## Contents

|ユーテリティ名|アウトライン|ドキュメント|
|:----|:----|:----|
|config_tf|カメラの構成をTFに設定する|[Link](config_tf/README.md)|
|rcalib|カメラとロボットのキャリブレーション|[Link](r-calib/README.md)|
|cropper|点群合成とクロッピング処理|[Link](cropper/README.md)|
|searcher|マスタ／シーンのマッチング|[Link](searcher/README.md)|
|picker|マッチング評価|[Link](picker/README.md)|
|tf_lookup|任意のフレーム間のTransformを表示するツール|[Link](#tf_lookup)|
|tf_euler|任意の回転順序のEuler角⇔Quaternion変換ツール|[Link](#tf_euler)|

## 追加インストール
### Open3D  
Open3Dは0.7以上が必要です。これより古い場合は以下にてアップデートします。
~~~
pip install open3d-python==0.7.0 --user
~~~
インストールにはPipのバージョン9.0.1以上が必要です、これより古い場合は以下にてアップデートします。
~~~
pip install pip==9.0.3
~~~

## Build  
標準手順どおり(catkin_make)

## ツール
<a name="tf_lookup"></a>
## tf_lookup  
1. 使用法
~~~
tf_lookup.py
~~~
にて起動し、標準入力から入力された座標フレームのTransformを標準出力に出力します。
入力の座標フレームは１〜３個を与えることができます。  
2. 入出力例  
フレーム名が１つのときは、world基準のTransformを出力します。以下はworld基準のcameraフレームを出力します。  
~~~
camera
~~~
フレーム名が２つのときは、１つ目のフレーム基準の２つ目のフレームのTransformを出力します。以下はbase基準のcameraフレームを出力します。  
~~~
base camera
~~~
フレーム名が３つのときは、１つ目のフレーム基準の２つ目のフレームのTransformと同じになる、３つ目のフレームに対する基準フレームを出力します。以下は、base基準のcameraフレームのTransformと、出力されるTransform基準のsolveフレームのそれとが同じとなります。  
~~~
base camera solve
~~~

<a name="tf_euler"></a>
## tf_euler
1. 使用法
~~~
tf_euler.py <軸回転順> <角度単位>
~~~
にて起動し、標準入力から入力された、Euler角またはQuaternionに相当する、それぞれのQuaternion、Euler角を標準出力に出力します。  
第1パラメータの軸回転順は4文字の文字で表します。最初の1文字は、回転基準を操縦者またはワールドを指定するもので、"r"が操縦者、"s"がワールドです。 これに続き3つの軸記号"x","y","z"の3文字のシーケンスにて順序を表します。  
第2パラメータは、角度単位を度(deg)またはラジアン(rad)にて与えます。  
下例はいわゆるロールピッチヨーの回転を指定する場合です。
~~~
tf_euler.py rxyz deg
~~~
2. 入出力例  
入力が、Euler角かQuaternionかの判別は、単にパラメータ数によって行います。値が３つのときはEuler角、４つはQuarternionです。入力値は","区切りで与えます。以下は、15,15,15のEuler角度を入力され、これに相当するQuarternionが出力されます。
~~~
15,15,15
~~~
Quaternionのときは、4つの値が与えられたときはQuaternionと判断します。上記の出力のQuaternionを入力すると元のEuler角が出力されます。
~~~
0.14519373836191624,0.11141107393065441,0.14519373836191624,0.9723297430844131
~~~
