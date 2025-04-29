# oc_photo_apiserver

## 概要

このパッケージは、ROS 2トピックから画像をキャプチャし、ディスクに保存し、FastAPIを使用してWeb API経由で提供する機能を提供します。テスト画像を発行するためのユーティリティノードも含まれています。

## 機能

-   画像トピックを購読し、受信した画像をディレクトリに保存します。
-   FastAPIサーバーを通じて、保存された画像の一覧表示および取得を可能にします。
-   テスト用に黒画像をトピックに配信する機能を提供します（メインのlaunchには含まれません）。

## トピックとサービス

| 種別         | 名前           | 型                      | 説明                                                                 |
| :----------- | :------------- | :---------------------- | :------------------------------------------------------------------- |
| Subscription | (設定可能)     | `sensor_msgs/msg/Image` | 画像をキャプチャするために購読する画像トピック（パラメータまたはlaunchで指定） |

**APIエンドポイント (FastAPIサーバー提供)**

| メソッド | パス                 | 説明                               |
| :------- | :------------------- | :--------------------------------- |
| `GET`    | `/images/`           | 保存されている画像ファイル名の一覧を返す |
| `GET`    | `/images/{filename}` | `{filename}`で指定された画像ファイルを返す |

## 事前準備

-   `image_saver_node` が購読するトピックに画像をパブリッシュするROS 2ノードが必要です。
-   Python依存関係: `fastapi`, `uvicorn`。pipでインストールします:
    ```bash
    pip install fastapi "uvicorn[standard]"
    ```

## 使用方法

### launchファイルで一括で起動する方法 (推奨)
```bash
ros2 launch oc_photo_apiserver photo_apiserver_launch.py
```
通常、これにより画像保存ノードとFastAPIサーバーの両方が起動します。

### 個別で起動する方法
1.  画像保存ノードを起動します:
    ```bash
    # 例: 'image_saver_node' を実際の実行ファイル名に置き換えてください
    ros2 run oc_photo_apiserver image_saver_node --ros-args -p image_topic:=/camera/image_raw
    ```
2.  FastAPIサーバーを起動します:
    ```bash
    # 例: 'api_server_node' を実際の実行ファイル名に置き換えてください
    ros2 run oc_photo_apiserver api_server_node
    # または、標準的なFastAPIアプリであればuvicornで直接実行
    # uvicorn oc_photo_apiserver.main:app --host 0.0.0.0 --port 8000
    ```

## テスト

標準的なROS 2リンター（copyright, pep257）が設定されています。テストは以下のコマンドで実行します:

```bash
colcon test --packages-select oc_photo_apiserver
colcon test-result --all
```
