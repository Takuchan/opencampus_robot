# oc_photo_apiserver

`oc_photo_apiserver` は、ROS 2 の画像トピックを受信し、受信した画像をディレクトリに保存する機能を提供します。また、保存された画像を FastAPI を通じてアクセス可能にします。

## 機能
- テスト用に黒画像をトピックに配信する機能。(launchに含まれない)
- 画像トピックを購読し、受信した画像をディレクトリに保存。
- FastAPI サーバーを通じて保存された画像を一覧表示および取得可能。


## 使用方法
### launchファイルで一括で起動する方法
推奨。
```bash
ros2 launch oc_photo_apiserver photo_apiserver_launch.py
```

### 個別で起動する方法
1. 画像配信ノードを起動します:
   ```bash
   ros2 run oc_photo_apiserver test_photo_publish
   ```

2. 画像保存ノードを起動します:
   ```bash
   ros2 run oc_photo_apiserver photoclient
   ```

3. FastAPI サーバーを起動します:
   ```bash
   python3 oc_photo_apiserver/fastapi_image_server.py
   ```

4. FastAPI サーバーにアクセスします:
   - 画像一覧: [http://localhost:8000/images](http://localhost:8000/images)
   - 画像取得: [http://localhost:8000/images/{filename}](http://localhost:8000/images/{filename})



## トピック

| トピック名               | メッセージ型         | 説明                                       |
|--------------------------|----------------------|--------------------------------------------|
| `/oc/image/image_raw`    | `sensor_msgs/Image`  | 生の画像データを配信または購読します。      |

## ノード

### `test_photo_publish.py`
- **目的**: テスト用に `/oc/image/image_raw` トピックに黒画像を配信します。
- **実行方法**:
  ```bash
  ros2 run oc_photo_apiserver test_photo_publish
  ```

### `photoclient.py`
- **目的**: `/oc/image/image_raw` トピックを購読し、受信した画像を `oc_saved_images` ディレクトリに保存します。
- **実行方法**:
  ```bash
  ros2 run oc_photo_apiserver photoclient
  ```

### `fastapi_image_server.py`
- **目的**: FastAPI サーバーを起動し、保存された画像を一覧表示および取得可能にします。
- **実行方法**:
  ```bash
  python3 oc_photo_apiserver/fastapi_image_server.py
  ```

## FastAPI エンドポイント

| エンドポイント           | メソッド | 説明                                       |
|--------------------------|----------|--------------------------------------------|
| `/images`               | GET      | 保存された画像の一覧を取得します。          |
| `/images/{filename}`    | GET      | 指定された画像を取得します。                |

## インストール手順


1. 依存関係をインストールします:
   ```bash
   rosdep install --from-paths src --ignore-src -r -y
   ```

2. ワークスペースをビルドします:
   ```bash
   cd ~/ros2_ws
   colcon build
   ```

3. ワークスペースをソースします:
   ```bash
   source ~/ros2_ws/install/setup.bash
   ```
