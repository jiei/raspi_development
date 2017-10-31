# raspi_development
Package for development of Raspberry Pi 3 with ROS.

## node
* joy_to_velocity：geometry_msgs::Joyのトピックをgeometry_msgs::Vectir3の目標速度ベクトルに変換する
* mimic_localization：geometry_msgs::Vectir3の目標速度ベクトルを受け取り、それを積算してgeometry_msgs::Pose2Dの自己位置座標として流す
* make_path_CSV：経路を自動生成する。make_path.launchで動かすと経路の描画もやってくれる。

## launch
* manual_example.launch：joy/joy_nodeでコントローラで入力→目標速度ベクトル化→積算して自己位置座標化→RVizに描画
* make_path.launch：経路の自動生成およびRVizで可視化をしてくれる
