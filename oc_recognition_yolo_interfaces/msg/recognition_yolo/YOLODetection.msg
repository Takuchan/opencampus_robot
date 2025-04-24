# 検出時刻（ROSのTime型）
std_msgs/Header header

# 画像サイズ（幅、高さ）
uint32 width
uint32 height

# 検出信頼度の閾値
float32 threshold

# 検出領域の座標（矩形の左上・右下）
# ここでは2次元座標として geometry_msgs/Point を利用（x: 横, y: 縦）
geometry_msgs/Point startpoint
geometry_msgs/Point endpoint

# オブジェクトのクラス名
string object_class

# 検出領域の中心付近の深度値（メートルなど、センサに合わせた単位）
float32 depth
