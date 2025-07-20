# oc_tts

## 概要

このパッケージは、OpenCampus Robotプロジェクトのためのテキスト読み上げ（TTS）サービスノードを提供します。バックエンドのTTSエンジン（例：Aivis）と連携し、テキスト文字列を音声に変換することを目的としています。

## 機能

-   テキスト文字列から音声合成を要求するためのROS 2サービスを提供します。
-   合成された音声を再生します。

## トピックとサービス

| 種別   | 名前           | 型                               | 説明                                   |
| :----- | :------------- | :------------------------------- | :------------------------------------- |
| Service | `/tts_service` | `oc_tts_interfaces/srv/TTS` | テキストを受け取り、音声合成を実行するサービス |

**サービス詳細 (`oc_tts_interfaces/srv/TTS`)**

-   **リクエスト:**
    -   `string text`: 合成して読み上げるテキスト
-   **レスポンス:**
    -   `bool success`: TTSリクエストが成功したかどうか
    -   `string message`: 処理結果に関するメッセージ

## 事前準備

-   バックエンドのTTSエンジンが実行中であり、アクセス可能である必要があります。例（Aivisを使用する場合）:
    ```bash
    docker run --rm -p '10101:10101' -v ~/.local/share/AivisSpeech-Engine:/home/user/.local/share/AivisSpeech-Engine-Dev ghcr.io/aivis-project/aivisspeech-engine:cpu-latest
    ```
-   `oc_tts_interfaces` パッケージがビルドされている必要があります。
-   具体的なTTSサーバー実装（例：`oc_aivis_tts` パッケージのノード）が実行されている必要があります。起動例:
    ```bash
    # oc_aivis_tts がサーバーノードを提供すると仮定
    ros2 run oc_aivis_tts tts_server_node
    # または、利用可能な場合は launch ファイルを使用
    ros2 launch oc_aivis_tts tts.launch.py
    ```

## テスト

標準的なROS 2リンター（copyright, pep257）が設定されています。テストは以下のコマンドで実行します:

```bash
colcon test --packages-select oc_tts
colcon test-result --all
```

## その他

-   このパッケージはサービスインターフェースと、クライアントまたは基本的なサーバー構造を定義する可能性があります。TTSエンジン（Aivisなど）への実際の接続は、別のパッケージ（例：`oc_aivis_tts`）で実装される場合があります。
