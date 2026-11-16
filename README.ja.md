# NS1 Wakeup Demo

日本語 | [English](README.md)

ESP32をNintendo Switch 1のPro Controllerとして動作させ、スリープ解除の通信フローをエミュレートします。

## 必要なもの

- ESP32 Rev3が搭載されたボード
- Windows PC
- PlatformIO Core
- SDL3

動作テスト用の`NS Input Passthrough`は実行ファイルと同じディレクトリの`SDL3.dll`を使用し、存在しない場合はSteamのインストール先から読み込みます。

## 対応環境と制約

ESP32 Rev3とESP-IDF 5.5.0の組み合わせで動作を確認しています。<br>
ファームウェアはPlatformIO上のESP-IDF 5.5.0でビルド可能であることを確認しています。

ESP-IDFのAPIで管理できないベースバンド制御のため、ファームウェアはROMと共有メモリを直接操作します。<br>
そのため、異なるリビジョンのESP32では正常に動作しない可能性があります。

## セットアップ

### ファームウェアをESP32へ書き込む

プロジェクトのルートディレクトリで次のコマンドを実行します。<br>
コマンドの`COMxxx`には、ESP32のCOMポート名を指定してください。

```powershell
pio run --target upload --upload-port COMxxx
```

### Pro ControllerとしてSwitchへ登録する

スリープ解除を行うためには、Switchとコントローラーのペアリングが行われている必要があります。

`NS Input Passthrough`を起動し、ESP32のCOMポートを選択して`Open Port`をクリックします。<br>
登録をやり直す場合は`Pair`をクリックします。

```powershell
& '.\input_passthrough\bin\NS Input Passthrough.exe'
```

### パススルーのテスト

接続先のSwitch本体に合わせて`NS1`または`NS2`の項目を選択して切り替えます。<br>
Switchがスリープ状態に入っている際にPC接続したコントローラーのHOMEボタンまたはPSボタンを押すと、スリープ解除処理と再接続処理が開始されます。<br>
Switchと未接続の状態でHOME以外のボタンを押すと、通常の再接続処理が開始されます。<br>
GUIの`Reconnect`、`Wake Up`、`Disconnect`から、各接続操作を手動で実行できます。

Switch 2のスリープ解除機能を使用する場合は`NS2`を選択して`BLE Scan`をクリックし、Switch 2とペアリング済みのSwitch 2用コントローラーでスリープ解除操作を行ってパケットキャプチャしておく必要があります。<br>
ウェイクアップパケットの保存後はESP32からSwitch 2のスリープ解除が行えるようになります。

## ディレクトリ構成

| パス | 内容 |
|---|---|
| `src/main.c` | Bluetoothの初期化とタスクの生成 |
| `src/bt/` | ペアリング、接続状態、HID接続の管理 |
| `src/hid/` | Pro ControllerのHIDレポート処理 |
| `src/bridge/` | UARTフレームの受信とコマンド処理 |
| `src/wake/` | ウェイクアップ通信と接続サイクルの管理 |
| `src/baseband/` | BluetoothコントローラーのROMと共有メモリを操作する処理 |
| `input_passthrough/` | Windows PCのゲームコントローラー入力をESP32へ送信するテストツール |
| `docs/` | [Switch 1ウェイクアッププロトコル](docs/wakeup-protocol.md)の説明 |

## ライセンス

本プロジェクトに含まれるソースコード、ドキュメントは[CC BY-NC 4.0](https://creativecommons.org/licenses/by-nc/4.0/)ライセンスの下で提供されます。

適切なクレジット表記と変更点の明示を条件に、非営利目的であればこのプロジェクトを制作物に使用することが可能です。
