### フォルダの説明
  - mqtt_bridge: ROSとAWS IoT Core(MQTT)とのブリッジ(*1)
  - map_launcher: 地図データおよび3Dモデルを保持するパッケージ(*2)
  - mr_converter: クラウドとオンプレサーバの間のフォーマット変更を行うパッケージ
  - mr_script: オンプレサーバ全体制御パッケージ
  - multi_turtlebots_nav: 複数ロボットナビゲーションのパッケージ(*2)
  - multi_turtlebots_sim: 複数ロボットシミュレーションのパッケージ(*2)  

  (*1) https://github.com/groove-x/mqtt_bridge + MSLでのカスタマイズ
  (*2) https://github.com/Pallav1299/Navigate_Multiple_Robots

### 主要な設定ファイル
- mr_script/launch/
  - multi_sim.launch: シミュレーション環境でのメインスクリプト
  - single_nav.launch: 実機環境でのメインスクリプト

- map_launcher/map/
  - foo/bar.world: 3Dモデル
  - foo/bar.png: 地図画像データ
  
  - foo/bar.yaml: 地図画像メタデータ
  - foo/bar_to_local.csv: グローバル座標をローカル座標に変換するための変換行列
  - foo/bar_to_global.csv: ローカル座標をグローバル座標に変換するための変換行列  
  
- mr_converter/launch/
  - converter.launch: シミュレーション環境でのフォーマット変換の設定(複数ロボット間のトピックのマッピングなど)
  - single_converter.launch: 実機環境での〃
  
- mqtt_bridge/launch/
  - multi_demo.launch: シミュレーション環境用
  - actual_demo.launch: 実機環境用

- mqtt_bridge/config/ (記載方法はMQTT_Bridge_User Manual.xlsxを参照)
  - demo_params.yaml: mqttメッセージからROS topicへのマッピング定義
  - tls_params.yaml: AWSへの接続設定
  
### 重要な設定パラメータ
- シミュレーション:
  - ナビゲーション用ローカル地図のパス:mr_script/launch/multi_sim.launchのlocal_map_path
  - ローカル地図上での初期座標:multi_turtlebots_nav/launch/amcl_robot1.launchのinitial_pose
  - gazebo用3Dモデルのバス:mr_script/launch/のmulti_sim.launchのglobal_map_path
  - gazebo用3Dモデル内での初期座標: multi_turtlebots_sim/launch/MR-robots.launchのinit_pose_x/y/a
- 実機:
  - ナビゲーション用ローカル地図のパス:mr_script/launch/のsingle_nav.launchのlocal_map_path
  - ローカル地図上での初期座標:multi_turtlebots_nav/launch/turtlebot3_navigation.launchのinitial_pose_x/y/a
