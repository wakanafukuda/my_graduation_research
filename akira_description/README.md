パッケージ名
==========
akira_description

概要
==========
あきらのロボットモデルです。
あきらのロボットモデルはまだ開発途中であり、特に腕・ハンドのxacroはサーボ間の距離やリンク間のツリー構造など、根本的な修正が必要な場合があります。
適宜、修正・追記していってください。

現在のロボットモデルツリー
==========
* check_urdfの結果

robot name is: akira
---------- Successfully Parsed XML ---------------
root Link: base_link has 3 child(ren)
　　child(1):  base_footprint
　　child(2):  base_laser_bottom
　　　　child(1):  base_laser_middle
　　　　　　child(1):  base_laser
　　child(3):  torso_link
　　　　child(1):  right_arm_base_link
　　　　　　child(1):  right_arm_shoulder_lift_L_link
　　　　　　　　child(1):  right_arm_shoulder_roll_link
　　　　　　　　　　child(1):  right_arm_shoulder_pan_link
　　　　　　　　　　　　child(1):  right_arm_elbow_flex_link
　　　　　　　　　　　　　　　child(1):  right_arm_forearm_flex_link
　　　　　　　　　　　　　　　　　　child(1):  right_arm_wrist_flex_link
　　　　　　　　　　　　　　　　　　　　child(1):  right_gripper_link
　　　　　　child(2):  right_arm_shoulder_lift_R_link
　　　　child(2):  camera_link
　　　　　　child(1):  camera_depth_frame
　　　　　　　　child(1):  camera_depth_optical_frame
　　　　　　child(2):  camera_rgb_frame
　　　　　　　　child(1):  camera_rgb_optical_frame
　　　　child(3):  face_link

* right_arm_shoulder_lift_L_linkとright_arm_shoulder_lift_R_linkが分かれていることについて
    - 肩の二つのサーボは二つで一つの役割を果たしますが、現在、right_arm_shoulder_lift_R_linkのほうは右腕に加えないことにしています。
    - もし実機で右腕が安全に動くことが確認されたら、右腕のツリーとして追加してもいいかもしれません。

* right_gripper_linkの位置
    - right_gripper_linkの位置は、手を開いたときの真ん中の位置です。これは、把持作業のときに、この位置をオブジェクトに近づけることで把持できるようにするためです。

よく使うコマンド
==========

```bash
# xacroからurdf変換 
rosrun xacro xacro.py `rospack find akira_description`/urdf/akira.xacro -o `rospack find akira_description`/urdf/akira.urdf
```

```bash
# 記述したurdfが正しいかチェック
rosrun urdfdom check_urdf `rospack find akira_description`/urdf/akira.urdf
# または、
check_urdf `rospack find akira_description`/urdf/akira.urdf
```

