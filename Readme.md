# path_optimizer
##I/O topics
Input:
nav_msgs::msg::Odometry "input/kinematics" 
autoware_auto_planning_msgs::msg::Trajectory "input/trajectory"
 "input/objects"

Output:
autoware_auto_planning_msgs::msg::Trajectory "output/trajectory"

##Usage
add to launch.xml
```bash
  <node pkg="path_optimizer" exec="path_optimizer" name="path_optimizer" output="screen">
    <param name="w_origin" value="0.6"/>
    <param name="w_smooth" value="0.4"/>
    <param name="optimize_torelance" value="0.5"/>
    <remap from="input/kinematics" to="/localization/kinematic_state"/>
    <remap from="input/trajectory" to="/planning/scenario_planning/trajectory"/>
    <remap from="input/objects" to="perception/object_recognition/objects"/>
    <remap from="output/trajectory" to="/planning/scenario_planning/opt_trajectory"/>
  </node>
```

##Contents
###path_smoothing(int max_times) 
受け取った経路を平滑化、平滑化した際の経路点の移動量が閾値以内or max_times回平滑化したら終了。後段のpure_pursuitのlookaheadの距離が長過ぎるかつこの平滑化を行いすぎると看過できないレベルのショートカットが行われるのではという懸念も。

###avoid_objects() 
周辺オブジェクトの予測軌道から回避、追い抜きのための経路を算出する関数。
オブジェクトの予測軌道と最適経路を比較し、目的経路とオブジェクト軌道がcollision_dist_以下のときにoffset_y_だけ進行方向に対し垂直方向に経路点をオフセットする。
追い抜き後オブジェクトが急加速して後ろから激突されるため最後に衝突判定、オフセット追加したindexの先もいくらかオフセット続行し衝突を免れようとした。
しかし、回避するときにレーンから離れすぎてしまっているので(特にコーナー)、avoid_objects()は使わず追い抜かない速度で走っている。左右どちらから追い越すかを目的経路点に対してオブジェクトが左右どちらにいるかで決めているため、経路点上を反復横飛びされたときに左右迷って暴れる減少も観測されており、回避方向を保持する処理を考えるか、現在位置も含めて回避経路を作ったほうが良い。

# chimple_pure_pursuit
##I/O topics
Input:
nav_msgs::msg::Odometry "input/kinematics"
autoware_auto_planning_msgs::msg::Trajectory "input/trajectory"

Output:
autoware_auto_control_msgs::msg::AckermannControlCommand "output/control_cmd"

##Contents
calc_longi_cmd()で目的経路の曲率から最適速度を計算、
calc_lat_cmd()で現在速度、目的経路上の追従点から舵角量を計算し合わせてコマンドを出力。

###calc_longi_cmd()
目的経路Pathの指定範囲size_t future_curvature_idx_(2とか)からsize_t future_curvature_predict_idx_(100とか、多分速度で変化した方が良い)までの各点で曲率を計算し、最大曲率となる部分を走れるであろう速度で速度制限を行う。S字カーブも検出し専用の速度制限もしている。明らかに曲率による速度制限が機能していない部分があるため曲率計算部は要確認。加速度制限も入れているが値は適当なので実質役に立っていない。

###calc_lat_cmd()
現在速度からlookahead計算して追従点に向かって舵角切るだけ。普通のpure_pursuit
舵角が高速時に遅れるので速度に応じて舵角にゲインを入れている。経路への距離や方位角差で修正のほうが良いかも。


##Usage
add to launch.xml
```bash
  <node pkg="chimple_pure_pursuit" exec="chimple_pure_pursuit" name="chimple_pure_pursuit_node" output="screen">
    <param name="use_external_target_vel" value="false"/>
    <param name="external_target_vel" value="100.0"/>
    <param name="lookahead_gain" value="0.5"/>
    <param name="lookahead_min_distance" value="10.0"/>
    <param name="speed_proportional_gain" value="1.0"/>
    
    <remap from="input/kinematics" to="/localization/kinematic_state"/>
    <remap from="input/trajectory" to="/planning/scenario_planning/opt_trajectory"/>
    <remap from="output/control_cmd" to="/control/command/control_cmd"/>
  </node>
```


