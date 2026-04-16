```
lg@nlg-Nitro-AN515-57:~$ rostopic list|grep stereo
/iris/stereo_camera/left/camera_info *
/iris/stereo_camera/left/image_raw *
/iris/stereo_camera/left/image_raw/compressed
/iris/stereo_camera/left/image_raw/compressed/parameter_descriptions
/iris/stereo_camera/left/image_raw/compressed/parameter_updates
/iris/stereo_camera/left/image_raw/compressedDepth
/iris/stereo_camera/left/image_raw/compressedDepth/parameter_descriptions
/iris/stereo_camera/left/image_raw/compressedDepth/parameter_updates
/iris/stereo_camera/left/image_raw/theora
/iris/stereo_camera/left/image_raw/theora/parameter_descriptions
/iris/stereo_camera/left/image_raw/theora/parameter_updates
/iris/stereo_camera/left/parameter_descriptions
/iris/stereo_camera/left/parameter_updates
/iris/stereo_camera/right/camera_info *
/iris/stereo_camera/right/image_raw *
/iris/stereo_camera/right/image_raw/compressed
/iris/stereo_camera/right/image_raw/compressed/parameter_descriptions
/iris/stereo_camera/right/image_raw/compressed/parameter_updates
/iris/stereo_camera/right/image_raw/compressedDepth
/iris/stereo_camera/right/image_raw/compressedDepth/parameter_descriptions
/iris/stereo_camera/right/image_raw/compressedDepth/parameter_updates
/iris/stereo_camera/right/image_raw/theora
/iris/stereo_camera/right/image_raw/theora/parameter_descriptions
/iris/stereo_camera/right/image_raw/theora/parameter_updates
/iris/stereo_camera/right/parameter_descriptions
/iris/stereo_camera/right/parameter_updates
nlg@nlg-Nitro-AN515-57:~$ 


```
-Topic,Lý do cần thiết cho dự án MOCHA,TID gợi ý,Ghi chú
- /mavros/local_position/pose,Pose 6D của UAV (dùng để bay qua vị trí UGV khi COMM),uav_pose,Quan trọng nhất cho SEARCH & COMM *
- /mavros/local_position/odom,Odometry filter (EKF) – sync với odom của Jackal,uav_odom,Xây shared map *
- /mavros/local_position/velocity_local,Vận tốc UAV (dùng cho planner),uav_vel,Optional nhưng rất hữu ích
- /mavros/state,"Trạng thái UAV (armed, mode, offboard…)",uav_state,Kiểm soát mission
- /mavros/battery,Pin UAV → quyết định kết thúc mission,uav_battery,Theo paper: terminate khi pin thấp
- /mavros/mission/waypoints,Waypoints exploration (INIT & EXPL state),uav_waypoints,Planner dùng để bay predefined waypoints
- /mavros/mission/reached,Đã đến waypoint chưa,uav_wp_reached,Trigger transition state
- /mavros/setpoint_velocity/cmd_vel,Command velocity (offboard control),uav_cmd_vel,Planner publish setpoint
- /mavros/setpoint_raw/local,Setpoint raw (position + velocity + yaw),uav_setpoint_raw,Dùng cho communication-aware planner
- /tf và /tf_static,Transform tree (base_link → lidar → map),tf,Sync map giữa UAV & UGV *
- /odometry/filtered,Odometry tổng hợp (nếu dùng robot_localization),filtered_odom,Shared odometry *


# Jackal
```
lg@nlg-Nitro-AN515-57:~$ rostopic list
/bluetooth_teleop/cmd_vel
/bluetooth_teleop/joy
/bluetooth_teleop/joy/set_feedback
/clock
/cmd_vel
/diagnostics
/e_stop
/gazebo/link_states
/gazebo/model_states
/gazebo/parameter_descriptions
/gazebo/parameter_updates
/gazebo/performance_metrics
/gazebo/set_link_state
/gazebo/set_model_state
/imu/data
/imu/data/accel/parameter_descriptions
/imu/data/accel/parameter_updates
/imu/data/bias
/imu/data/rate/parameter_descriptions
/imu/data/rate/parameter_updates
/imu/data/yaw/parameter_descriptions
/imu/data/yaw/parameter_updates
/jackal_velocity_controller/cmd_vel
/jackal_velocity_controller/cmd_vel_out
/jackal_velocity_controller/odom
/jackal_velocity_controller/parameter_descriptions
/jackal_velocity_controller/parameter_updates
/joint_states
/joy_teleop/cmd_vel
/mid/points
/navsat/fix
/navsat/fix/position/parameter_descriptions
/navsat/fix/position/parameter_updates
/navsat/fix/status/parameter_descriptions
/navsat/fix/status/parameter_updates
/navsat/fix/velocity/parameter_descriptions
/navsat/fix/velocity/parameter_updates
/navsat/vel
/odometry/filtered
/rosout
/rosout_agg
/set_pose
/tf
/tf_static
/twist_marker_server/cmd_vel
/twist_marker_server/feedback
/twist_marker_server/update
/twist_marker_server/update_full

```
- Topic,Lý do quan trọng trong MOCHA,TID gợi ý (shared_config.yaml),Ghi chú
- /mid/points,LiDAR point cloud thô của Jackal (ground-level mapping),jackal_lidar_points,Core nhất – UGV truyền chi tiết vật cản lên UAV *
- /jackal_velocity_controller/odom,Odometry chính thức của Jackal,jackal_odom,Sync odometry để build shared map *
- /odometry/filtered,Odometry đã filter (EKF) – đáng tin cậy nhất,jackal_filtered_odom,Dùng để merge map từ UAV *
- /tf và /tf_static,Transform tree (base_link → lidar → map),jackal_tf,Bắt buộc để reconstruct map giữa 2 robot *
- /cmd_vel,Velocity command (navigation),jackal_cmd_vel,Jackal nhận command từ planner (sau khi merge map từ UAV)
- /navsat/fix và /navsat/vel,GPS (nếu dùng cho last-known-position),jackal_gps,Hữu ích khi UAV SEARCH Jackal

**✅ Phân tích CHI TIẾT topics cần giữ cho Iris (EDEN planner) + Jackal (LRAE planner) trong dự án MOCHA**

Chào bạn!  

Rất tốt khi bạn chọn **EDEN** cho Iris (UAV) và **LRAE** cho Jackal (UGV). Đây là 2 planner cùng nhóm NKU-MobFly-Robotics, rất phù hợp với heterogeneous collaboration theo MOCHA:

- **EDEN** (UAV): Dual-layer (long-term region routing + short-term curvature-penalized viewpoint) + ASEO trajectory → cần **depth point cloud mạnh** (Realsense) + odom/pose/velocity để xây Dynamic Voxel Map + DTG.
- **LRAE** (UGV): Large-Region-Aware + traversability analysis trên uneven terrain → cần **LiDAR point cloud mạnh** để build traversability map + large unknown region detection.

Dưới đây là **topics nên giữ** (Essential) cho mỗi robot. Tôi chỉ giữ những gì **planner thực sự subscribe/publish** + những gì **MOCHA cần sync** (map, pose, odom để opportunistic communication + data mule).

### 1. Iris (UAV) + EDEN Planner + Realsense
EDEN cần: depth image/point cloud → voxel map, current pose/velocity/acceleration, tf.

**Topics Essential (phải giữ & sync qua MOCHA):**

| Topic | Lý do (theo EDEN paper) | TID gợi ý trong shared_config.yaml | Ghi chú |
|-------|--------------------------|------------------------------------|---------|
| `/camera/aligned_depth_to_color/points` (hoặc `/camera/depth/points`) | Input chính xây Dynamic Voxel Map + EROI | `iris_depth_points` | **Quan trọng nhất** |
| `/camera/depth/image_rect_raw` + `/camera/color/image_raw` | Depth image + RGB (nếu dùng semantic) | `iris_depth_image` | Dùng cho voxel map |
| `/camera/imu` | IMU Realsense hỗ trợ odometry | `iris_realsense_imu` | Cải thiện EKF |
| `/mavros/local_position/pose` | Current pose 6D (dùng cho DTG + viewpoint planning) | `iris_pose` | Bắt buộc |
| `/mavros/local_position/odom` | Odometry (input cho voxel map) | `iris_odom` | Bắt buộc |
| `/mavros/local_position/velocity_local` | Velocity + acceleration (ASEO trajectory) | `iris_velocity` | Bắt buộc |
| `/mavros/state` | Armed / Offboard mode | `iris_state` | Mission control |
| `/mavros/battery` | Pin để terminate mission | `iris_battery` | Safety |
| `/mavros/setpoint_raw/local` | Output setpoint từ EDEN (position + yaw) | `iris_setpoint` | Planner publish |
| `/tf` + `/tf_static` | Transform tree (camera → base_link → map) | `iris_tf` | Bắt buộc |

**Bỏ qua**: Tất cả mavros/hil/debug/gps_rtk/adsb, compressedDepth/theora.

### 2. Jackal (UGV) + LRAE Planner + LiDAR
LRAE cần: LiDAR point cloud → traversability map + large-region detection, odom, tf. Output: `/cmd_vel`.

**Topics Essential (phải giữ & sync qua MOCHA):**

| Topic | Lý do (theo LRAE paper + GitHub) | TID gợi ý trong shared_config.yaml | Ghi chú |
|-------|----------------------------------|------------------------------------|---------|
| `/mid/points` | LiDAR point cloud thô → traversability + exploration map | `jackal_lidar_points` | **Quan trọng nhất** |
| `/jackal_velocity_controller/odom` hoặc `/odometry/filtered` | Odometry chính thức | `jackal_odom` | Bắt buộc |
| `/tf` + `/tf_static` | Transform tree (base_link → lidar → map) | `jackal_tf` | Bắt buộc |
| `/cmd_vel` | Velocity command output từ LRAE | `jackal_cmd_vel` | Planner publish |
| `/imu/data` | IMU hỗ trợ traversability | `jackal_imu` | Hữu ích |

**Bỏ qua**: Tất cả bluetooth/joy/teleop, gazebo/*, parameter_*, rosout.

### 3. Topics chung quan trọng nhất cần sync qua MOCHA (gossip)
Những topic này **cả 2 robot đều cần** để opportunistic communication + shared map:

- **Map / Point Cloud**: `iris_depth_points` + `jackal_lidar_points` (hoặc OctoMap sau khi convert).
- **Pose/Odom**: `iris_pose`, `iris_odom`, `jackal_odom`.
- **TF tree**: `iris_tf` + `jackal_tf`.
- **Last-known-position**: Dùng để UAV (EDEN) chuyển sang **SEARCH** state tìm UGV.

**Gợi ý tối ưu cho MOCHA**:
- Chỉ sync **latest version** của map (timestamp-based) → tiết kiệm bandwidth.
- UAV (Iris) làm data mule: nhận LiDAR map từ Jackal → mang đi truyền cho Jackal khác (nếu có nhiều UGV).
- Jackal nhận aerial depth map từ Iris → cải thiện traversability + goal planning trong LRAE.

### Bước tiếp theo tôi khuyên bạn (theo thứ tự triển khai)
1. Cập nhật **shared_config.yaml** với **chỉ 10-12 topic Essential** ở trên (tôi có thể viết mẫu ngay).
2. Viết **Software Interface** cho Iris (subscribe Realsense + mavros → insert DB) và cho Jackal (subscribe LiDAR + odom → insert DB).
3. Convert depth/LiDAR → **OctoMap** hoặc **voxel grid** trước khi insert DB (để EDEN và LRAE dễ merge map).
4. Test sync ZeroMQ giữa Iris (EDEN) ↔ Jackal (LRAE).


**✅ Phân loại CHI TIẾT topics cho Iris (UAV) + EDEN Planner + Realsense Depth Camera**

Chào bạn!  

Tôi đã phân tích **toàn bộ** danh sách `rostopic list` bạn đưa (Iris với Realsense depth camera + stereo camera + PX4 mavros).  

**Tóm tắt nhanh theo EDEN + MOCHA**:
- **EDEN** cần: depth point cloud/image (để xây Dynamic Voxel Map + EROI + DTG), current pose/velocity/acceleration, tf, state, battery, setpoint output.
- **MOCHA** cần: chỉ sync **latest version** của map + pose + odom để opportunistic comm (UAV làm data mule truyền aerial depth map xuống Jackal).
- **Realsense** là sensor chính (depth points/image), stereo camera chỉ là phụ (có thể bỏ nếu không dùng).

Dưới đây là bảng **chính xác** những topic **nên giữ** và **bỏ qua**.

### 1. **CẦN THIẾT (Essential – Phải giữ & sync qua MOCHA)**
Những topic này **bắt buộc** cho EDEN planner + MOCHA.

| Topic | Lý do theo EDEN + MOCHA | TID gợi ý trong shared_config.yaml | Ghi chú |
|-------|--------------------------|------------------------------------|---------|
| `/iris/realsense/depth_camera/depth/points` | Depth point cloud chính → Dynamic Voxel Map + EROI | `iris_depth_points` | **Quan trọng nhất** |
| `/iris/realsense/depth_camera/depth/image_raw` | Depth image (dùng cho voxel/octomap) | `iris_depth_image` | Bắt buộc |
| `/iris/realsense/depth_camera/color/image_raw` | RGB image (nếu EDEN dùng semantic) | `iris_rgb_image` | Hữu ích |
| `/iris/realsense/depth_camera/depth/camera_info` | Calibration depth camera | `iris_depth_info` | Bắt buộc |
| `/iris/realsense/depth_camera/color/camera_info` | Calibration color camera | `iris_color_info` | Bắt buộc |
| `/mavros/local_position/pose` | Pose 6D (DTG + viewpoint planning) | `iris_pose` | Bắt buộc |
| `/mavros/local_position/odom` | Odometry input cho voxel map | `iris_odom` | Bắt buộc |
| `/mavros/local_position/velocity_local` | Velocity + accel (ASEO trajectory) | `iris_velocity` | Bắt buộc |
| `/mavros/local_position/accel` | Acceleration (ASEO) | `iris_accel` | Hữu ích |
| `/mavros/state` | Offboard / armed status | `iris_state` | Mission control |
| `/mavros/battery` | Pin → terminate mission | `iris_battery` | Safety |
| `/mavros/setpoint_raw/local` | Output setpoint từ EDEN | `iris_setpoint_raw` | Planner publish |
| `/mavros/setpoint_velocity/cmd_vel` | Velocity command | `iris_cmd_vel` | Backup cho EDEN |
| `/tf` + `/tf_static` | Transform tree (camera → base_link → map) | `iris_tf` | Bắt buộc |
| `/mavros/mission/waypoints` | Waypoints cho long-term routing | `iris_waypoints` | EDEN long-term region |

**→ Tổng 14 topic essential**. Đây là những gì bạn **phải** đưa vào MOCHA DB Server.

### 2. **HỮU ÍCH (Useful – Giữ nếu có bandwidth dư)**
| Topic | Lý do |
|-------|------|
| `/mavros/imu/data`, `/mavros/imu/data_raw` | IMU hỗ trợ odometry |
| `/mavros/global_position/local` | Global position (SEARCH UGV) |
| `/mavros/extended_state`, `/mavros/sys_status` | Monitoring & safety |
| `/mavros/nav_controller_output` | Debug EDEN trajectory |

### 3. **KHÔNG CẦN (Bỏ qua hoàn toàn)**
- Tất cả `/iris/stereo_camera/*` → Không cần vì Realsense depth đã thay thế.
- Tất cả `/gazebo/*` (link_states, model_states, parameter_*, set_*, performance_metrics) → Chỉ simulation.
- Tất cả `/mavros/hil/*`, `/mavros/debug_value/*`, `/mavros/adsb/*`, `/mavros/gps_rtk/*`, `/mavros/gpsstatus/*`, `/mavros/rc/*`, `/mavros/manual_control/*` → HIL sim, debug, RC.
- Tất cả compressed/compressedDepth/theora/parameter_* → Chỉ visualization.
- `/mavlink/*`, `/mavros/camera/image_captured`, `/mavros/landing_target/*`, `/mavros/mount_control/*`, `/mavros/px4flow/*`, `/mavros/tunnel/*`, `/mavros/statustext/*`, `/mavros/play_tune`, `/mavros/obstacle/send`, `/mavros/terrain/report`, v.v. → Không liên quan đến EDEN.
- `/move_base_simple/goal`, `/rosout`, `/rosout_agg`, `/clock`, `/diagnostics` → Không sync.

**Tóm tắt cho Iris**:
- Giữ **14 topic essential** (Realsense depth + mavros pose/odom/velocity/state/battery/setpoint + tf).
- Bỏ **hơn 80%** danh sách → giảm bandwidth rất mạnh khi sync opportunistic.

### Gợi ý triển khai ngay
- Trong **shared_config.yaml**: chỉ khai báo 14 topic trên (RID = Iris).
- EDEN sẽ subscribe Realsense depth points/image → build voxel map → publish setpoint_raw/local.
- MOCHA Software Interface: subscribe các topic essential → insert DB (chỉ latest timestamp).

Bạn muốn tôi **viết ngay**:
- `shared_config.yaml` mẫu đầy đủ cho Iris (EDEN + Realsense)?
- Hay code Python **Software Interface** cho Iris (subscribe Realsense + mavros)?
- Hoặc cách tích hợp EDEN node vào launch file Iris?

Reply **“viết config yaml”** hoặc **“viết software interface”** để tôi đưa code luôn.  

Dự án của bạn đang rất rõ ràng: Iris dùng EDEN + Realsense aerial depth map, Jackal giữ LRAE + LiDAR ground map → MOCHA sẽ kết nối chúng opportunistic một cách hoàn hảo! 🚀  

Sẵn sàng bước nào tiếp theo?