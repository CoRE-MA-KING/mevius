# Mevius プロジェクト概要 (ROS 2 Jazzy)

このドキュメントは、四足歩行ロボットMEVIUSプロジェクトの概要を提供するもので、ハードウェア、ソフトウェアコンポーネント、依存関係、および操作手順が含まれています。このプロジェクトは、Eコマースで入手可能なコンポーネント、板金溶接、機械加工を用いて容易に製作できる四足歩行ロボットを創出することを目的としています。

このドキュメントは特に **ROS 2 Jazzy** 環境に合わせて調整されています。

## 1. プロジェクト関連リンク

*   [プロジェクトページ](https://haraduka.github.io/mevius-hardware)
*   [arXiv](https://arxiv.org/abs/2409.14721)
*   [YouTube](https://www.youtube.com/watch?v=XXJ4EK3Y4zQ)

## 2. ハードウェア情報

すべてのハードウェアコンポーネントの詳細は、以下のGoogle Driveリンクで確認できます。
*   [Google Drive - ハードウェアコンポーネント](https://drive.google.com/drive/folders/18i6CWtUG8fkY0rCcbjN5WiUYItjIX0WM?usp=sharing)

**モーター設定 (T-Motor AK70-10):**
*   [RUBIK LINK V2.0](https://store.tmotor.com/product/rubik-link-v2.html) から上位コンピュータのインストールプログラムをダウンロードし、すべてのモーターをMITモードに設定してください。
*   [AK70-10 製品ページ](https://store.tmotor.com/product/ak70-10-dynamical-modular.html)
*   [AKシリーズマニュアル](https://store.tmotor.com/images/file/202208/251661393360838805.pdf)

**モーターの順序とリンク/関節名:**
*   モーター順序: FR(1-3) --> FL(4-6) --> BR(7-9) --> BL(10-12) / 根元側 --> 先端側
*   リンク名: `*_scapula_link` --> `*_thigh_link` --> `*_calf_link`
*   関節名: `*_collar` --> `*_hip` --> `*_knee`

## 3. ソフトウェア依存関係

### 3.1 ROS 2 Jazzy 依存関係

このプロジェクトはROS 2 Jazzy上で構築されています。主要なROS 2パッケージとその役割は以下の通りです。

*   **ament_cmake**: ROS 2パッケージ用のコアCMakeビルドシステム。
*   **rosidl_default_generators**: `.msg`, `.srv`, `.action`ファイルからC、C++、Pythonコードを生成。
*   **rosidl_default_runtime**: 生成されたROS IDLインターフェースのランタイムサポートを提供。
*   **std_msgs**: 標準的なメッセージ定義 (例: `String`, `Int32`)。
*   **robot_state_publisher**: URDFに基づいてロボットの状態を`tf2`に発行。
*   **rviz**: ROS 2用の3D可視化ツール。
*   **joint_state_publisher_gui**: URDFモデルの関節状態を手動で発行するためのGUIツール。
*   **gazebo_ros_pkgs**: Gazeboシミュレータとの連携 (ROS 2では通常Gazebo Garden/Ignition)。
*   **mevius_msgs**: カスタムメッセージパッケージ (`package.xml`の`mevius_massage2`から推測)。

追加のROS関連依存関係:

*   [urdfdom-py](https://github.com/ros/urdf_parser_py): URDF解析用のPythonバインディング。
*   **realsense-ros**: Intel RealSenseカメラ用。**注:** `README.md`では`ros1-legacy`ブランチが参照されています。これをROS 2互換バージョンに更新するか、ROS 2ネイティブドライバーに置き換える必要があります。
*   **spacenav_node**: SpaceNavigatorデバイス用。**注:** `README.md`ではROS 1パッケージが参照されています。ROS 2での同等品または代替品が必要です。
*   **rqt_virtual_joystick**: 仮想ジョイスティックGUI。**注:** `README.md`ではROS 1パッケージが参照されています。ROS 2での同等品または代替品が必要です。

### 3.2 Pythonパッケージ依存関係

*   `bitstring`
*   `pytorch`
*   `scipy`
*   `mujoco`
*   `mujoco-python-viewer`

### 3.3 外部リポジトリ/ライブラリ

*   [legged_gym](https://github.com/leggedrobotics/legged_gym)
    *   強化学習のために、特定のフォーク/ブランチ `haraduka/mevius` が使用されます。
*   `mevius/tmotor_lib/tmotor_lib.py`: 元は [mini-cheetah-tmotor-python-can](https://github.com/dfki-ric-underactuated-lab/mini-cheetah-tmotor-python-can) から。
*   `mevius/mevius_utils/legged_gym_math/isaacgym_torch_utils/isaacgym_torch_utils.py`: 元は [IsaacGym](https://developer.nvidia.com/isaac-gym) から。
*   `mevius/mevius_utils/legged_gym_math/legged_gym_math.py`: 元は [LeggedGym](https://github.com/leggedrobotics/legged_gym) から。

## 4. ビルド手順

このプロジェクトは、Pythonパッケージに`ament_python`を、C++コンポーネント（もしあれば）に`ament_cmake`を使用します。

```bash
mkdir -p ~/mevius_ws/src
cd ~/mevius_ws/src
git clone git@github.com:haraduka/mevius.git
cd ..
colcon build --symlink-install
source install/setup.bash
```

## 5. 強化学習

このプロジェクトには強化学習ポリシーが含まれています。現在のポリシーは単純でそれほど安定していないと説明されており、多くの改善が施された安定版が近々アップロードされる予定です。

### トレーニングプロセス (`legged_gym`環境内):

1.  `haraduka/legged_gym`をリモートとして追加し、`haraduka/mevius`ブランチをチェックアウトします。
2.  **第一段階トレーニング:**
    ```bash
    python3 legged_gym/scripts/train.py --task mevius
    ```
3.  **第二段階トレーニング:**
    *   `legged_gym/envs/mevius/mevius_config.py`内のパラメータを "first" から "second" に変更します（具体的には`commands.ranges`と`domain_rand.friction_range`）。
    ```bash
    python3 legged_gym/scripts/train.py --task mevius
    ```
4.  **ポリシーの実行:**
    ```bash
    python3 legged_gym/scripts/play.py --task mevius --load_run ( log_dir )
    ```
5.  **ポリシーのエクスポート:**
    ```bash
    cp logs/flat_mevius/( log_dir )/exported/policies/policy_1.pt ../mevius/models/policy.pt
    ```

## 6. 使用方法

### 6.1 基本的なユーティリティテスト

```bash
python3 scripts/mevius_utils.py
```

### 6.2 モーターテスト

1.  CANインターフェースのセットアップ:
    ```bash
    ./bin/can_setup.sh
    ```
2.  モーター制御テスト:
    ```bash
    python scripts/tmotor_test.py --ids 1 --task sense
    ```

### 6.3 実機操作 (ROS 2 Jazzy)

*   **MEVIUS PC上で:**
    ```bash
    # ROS 2デーモンが実行されていない場合は起動
    ros2 daemon start 
    # RealSenseカメラを起動 (ROS 2互換のrealsense-rosパッケージが必要)
    ros2 launch realsense2_camera rs_t265.launch 
    ./bin/can_setup.sh
    python3 scripts/mevius_main.py
    ```
*   **ローカルPC上で:**
    ```bash
    # SpaceNavigatorノードを起動 (ROS 2互換のspacenav_nodeパッケージが必要)
    ros2 launch spacenav_node classic.launch
    ```
    *   **操作:** 左クリック: SITDOWN/STANDUP, 右クリック: STANDUP/WALK

### 6.4 シミュレーション

*   **MuJoCo シミュレーション:**
    ```bash
    python3 scripts/mevius_main.py --sim
    ```
    *   MuJoCoビューアで 'D'キーを押すと、毎ステップのレンダリングをオフにできます。

## 7. 謝辞

*   `mevius/tmotor_lib/tmotor_lib.py`: [mini-cheetah-tmotor-python-can](https://github.com/dfki-ric-underactuated-lab/mini-cheetah-tmotor-python-can) を改変。
*   `mevius/mevius_utils/legged_gym_math/isaacgym_torch_utils/isaacgym_torch_utils.py`: [IsaacGym](https://developer.nvidia.com/isaac-gym) を改変。
*   `mevius/mevius_utils/legged_gym_math/legged_gym_math.py`: [LeggedGym](https://github.com/leggedrobotics/legged_gym) を改変。

## 8. 引用

```
@inproceedings{kawaharazuka2024mevius,
  author={K. Kawaharazuka and S. Inoue and T. Suzuki and S. Yuzai and S. Sawaguchi and K. Okada and M. Inaba},
  title={{MEVIUS: A Quadruped Robot Easily Constructed through E-Commerce with Sheet Metal Welding and Machining}},
  booktitle={Proceedings of the 2024 IEEE-RAS International Conference on Humanoid Robots},
  year=2024,
}
```