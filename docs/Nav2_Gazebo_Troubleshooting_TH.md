# บันทึกการแก้ปัญหา Nav2 และ Gazebo บน ROS 2 Jazzy

## 1. ข้อมูลระบบ

- ระบบปฏิบัติการ: Ubuntu 24.04 LTS
- ROS: ROS 2 Jazzy
- Simulator: Gazebo Harmonic
- เครื่องมือแสดงผลและควบคุม: RViz2
- ระบบนำทาง: Navigation2 (Nav2)
- หุ่นยนต์จำลอง: TurtleBot3 Waffle
- คำสั่งที่ใช้เริ่มระบบ:

```bash
source /opt/ros/jazzy/setup.bash
ros2 launch nav2_bringup tb3_simulation_launch.py headless:=False autostart:=True
```

## 2. ภาพรวมปัญหา

Gazebo และ RViz สามารถเปิดได้ตามปกติ และ RViz สามารถแสดงแผนที่ได้ แต่เมื่อกำหนดเป้าหมายด้วย `Nav2 Goal` หุ่นยนต์ใน Gazebo ไม่เคลื่อนที่

ปัญหาไม่ได้เกิดจากตัวหุ่นยนต์หรือ Gazebo โดยตรง แต่เกิดจาก Nav2 เปิดทำงานไม่ครบทุกส่วน โดยพบปัญหาต่อเนื่อง 3 ขั้น ได้แก่:

1. `nav2_container` แครชระหว่างเริ่มระบบ
2. ระบบยังไม่ทราบตำแหน่งเริ่มต้นของหุ่นยนต์บนแผนที่
3. Node ที่ส่งคำสั่งความเร็วไปยัง Gazebo ยังอยู่ในสถานะ `inactive`

## 3. ลำดับการทำงานที่ถูกต้อง

เส้นทางของคำสั่งนำทางมีลักษณะดังนี้:

```text
RViz ส่งตำแหน่งเป้าหมาย
        ↓
bt_navigator จัดการภารกิจนำทาง
        ↓
planner_server วางเส้นทาง
        ↓
controller_server คำนวณคำสั่งความเร็ว
        ↓
velocity_smoother ปรับความเร็วให้ต่อเนื่อง
        ↓
collision_monitor ตรวจสอบความปลอดภัย
        ↓
Gazebo รับคำสั่งและทำให้หุ่นยนต์เคลื่อนที่
```

หาก Node ใด Node หนึ่งในเส้นทางนี้ไม่ทำงาน หุ่นยนต์อาจรับเป้าหมายหรือสร้างเส้นทางได้ แต่ไม่สามารถเคลื่อนที่จริงใน Gazebo

## 4. ปัญหาที่ 1: nav2_container แครช

### อาการ

Terminal แสดงข้อความลักษณะดังนี้:

```text
Magic: abort due to signal 11 (SIGSEGV)
process has died ... component_container_isolated
```

และ RViz แสดงข้อความซ้ำ ๆ เช่น:

```text
Message Filter dropping message: frame 'base_scan'
discarding message because the queue is full
```

### สาเหตุ

`nav2_container` แครชระหว่างกำหนดค่า `route_server` ทำให้ Node หลายตัวที่ทำงานอยู่ภายใน container เดียวกันหยุดตามไปด้วย

ข้อความ `queue is full` ใน RViz ไม่ใช่ต้นเหตุหลัก แต่เป็นผลตามมาจากการที่ TF และ Node ของ Nav2 หยุดทำงาน

### วิธีแก้

อัปเดตและติดตั้งแพ็กเกจ Nav2 ใหม่:

```bash
sudo apt update
sudo apt upgrade -y
sudo apt install --reinstall \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup \
  ros-jazzy-nav2-costmap-2d \
  ros-jazzy-nav2-route \
  ros-jazzy-nav2-minimal-tb3-sim
sudo reboot
```

หลังจากอัปเดตแล้ว `nav2_container` ไม่แครชอีก

## 5. ปัญหาที่ 2: Nav2 ไม่ทราบตำแหน่งเริ่มต้น

### อาการ

Terminal แสดงข้อความ:

```text
AMCL cannot publish a pose or update the transform. Please set the initial pose...
Invalid frame ID "map"
Timed out waiting for transform from base_link to map
```

เมื่อพยายามเปิด `planner_server` จะได้:

```text
Transitioning failed
```

### สาเหตุ

AMCL ยังไม่ทราบว่าหุ่นยนต์ใน Gazebo อยู่ตำแหน่งใดบนแผนที่ จึงยังไม่สามารถสร้าง TF ต่อไปนี้ได้:

```text
map → odom → base_link
```

เมื่อไม่มี TF `map → base_link` ระบบวางแผนเส้นทางไม่สามารถระบุตำแหน่งเริ่มต้นของหุ่นยนต์ได้

### วิธีแก้

1. กด `2D Pose Estimate` ใน RViz
2. คลิกตรงตำแหน่งของหุ่นยนต์บนแผนที่
3. ลากลูกศรให้ตรงกับทิศที่หุ่นยนต์หันอยู่
4. ปรับจน LaserScan ซ้อนตรงกับกำแพงหรือสิ่งกีดขวางบนแผนที่

ตำแหน่งเริ่มต้นโดยประมาณของการทดลองครั้งนี้คือ:

```text
x ≈ -1.9 เมตร
y ≈ -0.35 เมตร
yaw ≈ 0 องศา
```

ตรวจสอบ TF ด้วยคำสั่ง:

```bash
ros2 run tf2_ros tf2_echo map base_link
```

หากมีค่า Translation และ Rotation แสดงต่อเนื่อง แสดงว่า localization และ TF ทำงานแล้ว

จากนั้นเปิด `planner_server`:

```bash
ros2 lifecycle set /planner_server activate
```

## 6. ปัญหาที่ 3: รับ Goal แล้ว แต่หุ่นยนต์ไม่เคลื่อนที่

### อาการ

- RViz รับเป้าหมายจาก `Nav2 Goal`
- แสดง `Feedback: active`
- แสดงระยะทางที่เหลือ
- อาจเริ่ม Recovery
- แต่หุ่นยนต์ใน Gazebo ไม่ขยับ

ตรวจสอบ Node หลักแล้วพบว่า:

```text
planner_server     active [3]
bt_navigator       active [3]
controller_server  active [3]
```

### สาเหตุ

แม้ Node หลักของ Nav2 ทำงานแล้ว แต่ Node ที่อยู่ในสายส่งคำสั่งความเร็วยังเป็น `inactive` ได้แก่:

```text
velocity_smoother
collision_monitor
```

จึงเกิดสถานการณ์ดังนี้:

```text
Controller สร้างคำสั่งความเร็วได้
        ↓
คำสั่งหยุดอยู่ที่ Node ซึ่งยัง inactive
        ↓
Gazebo ไม่ได้รับคำสั่งความเร็ว
        ↓
หุ่นยนต์ไม่เคลื่อนที่
```

### วิธีแก้

ตรวจสอบสถานะ:

```bash
ros2 lifecycle get /velocity_smoother
ros2 lifecycle get /collision_monitor
ros2 lifecycle get /smoother_server
ros2 lifecycle get /behavior_server
```

เปิดใช้งาน Node ที่ยังเป็น `inactive [2]`:

```bash
ros2 lifecycle set /velocity_smoother activate
ros2 lifecycle set /collision_monitor activate
ros2 lifecycle set /smoother_server activate
ros2 lifecycle set /behavior_server activate
```

หลังจาก `velocity_smoother` และ `collision_monitor` เป็น `active [3]` คำสั่งความเร็วสามารถส่งไปถึง Gazebo และหุ่นยนต์เริ่มเคลื่อนที่ได้

## 7. คำสั่งตรวจสอบระบบ

### ตรวจสอบ Node หลัก

```bash
ros2 lifecycle get /planner_server
ros2 lifecycle get /bt_navigator
ros2 lifecycle get /controller_server
ros2 lifecycle get /velocity_smoother
ros2 lifecycle get /collision_monitor
```

Node ที่จำเป็นควรแสดง:

```text
active [3]
```

### ตรวจสอบ NavigateToPose Action

```bash
ros2 action info /navigate_to_pose
```

ควรพบอย่างน้อย:

```text
Action servers: 1
```

การมี Action Server เพียงอย่างเดียวไม่ได้หมายความว่า Nav2 พร้อมทำงานทั้งหมด ต้องตรวจสอบ lifecycle ของ Node อื่นร่วมด้วย

### ตรวจสอบสถานะ Goal

```bash
ros2 topic echo /navigate_to_pose/_action/status
```

### ตรวจสอบ Topic ความเร็ว

```bash
ros2 topic list | grep cmd_vel
```

## 8. สรุปสาเหตุแบบสั้น

ปัญหาเกิดจาก Nav2 เปิดทำงานเพียงบางส่วน:

1. ตอนแรก `nav2_container` แครช ทำให้ Node ภายในหยุดทำงาน
2. หลังแก้การแครชแล้ว AMCL ยังไม่มี initial pose จึงไม่มี TF `map → base_link`
3. หลังตั้งตำแหน่งและเปิด Node หลักแล้ว ระบบรับ Goal ได้ แต่ `velocity_smoother` และ `collision_monitor` ยัง inactive ทำให้คำสั่งความเร็วไม่ถึง Gazebo

เมื่ออัปเดตแพ็กเกจ ตั้ง `2D Pose Estimate` และ activate Node ที่เกี่ยวข้องครบ หุ่นยนต์จึงสามารถรับเป้าหมาย วางเส้นทาง และเคลื่อนที่ใน Gazebo ได้สำเร็จ

## 9. หมายเหตุสำหรับการพัฒนาต่อ

การ activate Node ด้วยคำสั่ง `ros2 lifecycle set` มีผลเฉพาะการเปิดระบบรอบปัจจุบัน เมื่อปิดแล้วเปิด simulation ใหม่อาจต้องทำซ้ำ

ขั้นต่อไปควรสร้าง launch file และ parameter file ของโปรเจกต์เอง เพื่อให้:

- Node ที่จำเป็นเริ่มทำงานโดยอัตโนมัติ
- กำหนดลำดับ lifecycle ได้ถูกต้อง
- ตัด Node ที่ยังไม่จำเป็น เช่น `route_server` ออกจากระบบทดลองเบื้องต้น
- ลดการตั้งค่าด้วยมือทุกครั้งที่เริ่ม simulation

