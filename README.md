# Autonomous Differential Drive Robot Simulation & Vision System (ROS 2 Jazzy)

Proyek simulasi robot *differential drive* otonom berbasis **ROS 2 Jazzy** dan **Gazebo Sim (Harmonic)**. Sistem ini mencakup pemodelan robot, dunia simulasi arena berlintasan gerbang buoy, perekaman dan navigasi *waypoint* berpresisi tinggi, serta modul *computer vision* (C++ / OpenCV) untuk pemusatan jalur gerbang (*gate centering*) dan penangkapan citra rintangan (*obstacle capture*).

---

## Fitur Utama

1. **Gazebo Sim & Robot Spawning ([spawn_robot.launch.py](src/robot_bringup/launch/spawn_robot.launch.py))**
  - Meluncurkan dunia arena [world/field.sdf](world/field.sdf) dengan 10 gerbang buoy (merah & hijau) serta balok rintangan.
  - Spawning robot [diff_drive_robot](models/diff_drive_robot/model.sdf) pada dock awal `(x: 1.80, y: -2.24, yaw: 1.5708)`.
   - Mengaktifkan jembatan topik `ros_gz_bridge` (`/cmd_vel`, `/camera/image_raw`, `/imu/data`, `/joint_states`, dan `/ground_truth/odom`).

2. **Ground-Truth Odometry (Anti-Slip Roda)**
   - Menggunakan plugin `gz::sim::systems::OdometryPublisher` pada robot untuk menghasilkan posisi absolut tanpa terpengaruh oleh *wheel slip* atau akumulasi drift fisika.

3. **Manual Waypoint Mapping & Navigation**
  - **[waypoint_logger.py](src/robot_bringup/scripts/waypoint_logger.py)**: Mencatat koordinat `(x, y, yaw)` secara real-time saat robot dikendalikan secara manual via teleop dan menyimpannya ke format file YAML (`waypoints.yaml`).
  - **[waypoint_follower.py](src/robot_bringup/scripts/waypoint_follower.py)**: Mengarahkan robot menyusuri urutan waypoint dengan kontrol **Rotate First, Then Drive** (putar robot di tempat sampai lurus sebelum maju), mencegah efek *corner cutting* pada celah sempit.

4. **Modul Computer Vision (C++ / OpenCV)**
  - **[buoy_centering.cpp](src/robot_bringup/src/buoy_centering.cpp)**:
     - Mendeteksi bentuk bulat buoy merah dan hijau melalui filter *circularity* dan segmentasi warna HSV.
     - Menghitung titik tengah (*midpoint*) kedua buoy dan menghasilkan kontrol kemudi `/cmd_vel` agar robot selalu melintas tepat di tengah gerbang.
     - Melacak total 10 gerbang yang dilalui dan mempublikasikan status `/vision/gate_10_passed`.
   - **[obstacle_detector_capture.cpp](src/robot_bringup/src/obstacle_detector_capture.cpp)**:
     - Mendeteksi balok rintangan persegi panjang warna **kuning** dan **biru** menggunakan *aspect ratio*, *circularity*, dan *extent*.
     - **Gate Lock**: Pengambilan foto terkunci dan hanya aktif setelah robot melewati gerbang ke-10.
    - Menyimpan foto hasil deteksi secara otomatis ke dalam folder [saved_capcture/](saved_capcture) dengan *timestamp* dan *cooldown* 2 detik.

---

## Struktur Direktori

```text
UTS_fix/
├── models/
│   └── diff_drive_robot/       # Model SDF robot, mesh STL (chassis, roda, imu, kamera)
│       ├── meshes/
│       ├── model.config
│       └── model.sdf
├── world/
│   ├── field.sdf               # Arena simulasi Gazebo (10 pasang gerbang buoy & balok rintangan)
│   └── lintasan_a.sdf          # Varian world dengan modular inclusion
├── saved_capcture/             # Direktori penyimpanan otomatis hasil capture foto objek
├── src/
│   └── robot_bringup/          # Paket ROS 2
│       ├── CMakeLists.txt
│       ├── package.xml
│       ├── launch/
│       │   └── spawn_robot.launch.py   # Launch simulasi Gazebo, robot, bridge & kedua node vision C++
│       ├── scripts/
│       │   ├── waypoint_logger.py      # Python node logger waypoint interaktif
│       │   └── waypoint_follower.py    # Python node navigasi waypoint dengan koreksi vision buoy
│       └── src/
│           ├── buoy_centering.cpp            # C++ node visual tracking buoy, centering assist & gate counter
│           └── obstacle_detector_capture.cpp # C++ node deteksi & capture kotak kuning/biru setelah gerbang 10
├── waypoints.yaml              # File daftar waypoint hasil mapping manual
└── README.md                   # Dokumentasi proyek
```

---

## Persyaratan Sistem

- **Sistem Operasi:** Ubuntu 24.04 LTS
- **ROS 2 Distro:** ROS 2 Jazzy Jalisco
- **Gazebo:** Gazebo Sim Harmonic (`ros_gz_sim`, `ros_gz_bridge`)
- **Pustaka C++ & Python:** OpenCV (`libopencv-dev`), `cv_bridge`, `PyYAML`

---

## Panduan Instalasi & Build

Buka terminal di root direktori proyek, lalu jalankan:

```bash
# 1. Source ROS 2 environment
source /opt/ros/jazzy/setup.bash

# 2. Build paket robot_bringup
colcon build --packages-select robot_bringup

# 3. Source hasil instalasi paket lokal
source install/setup.bash
```

---

## Langkah-Langkah Menjalankan

### Langkah 1: Jalankan Simulasi & Visualisasi (Terminal 1)
Pilih salah satu mode tampilan sesuai kebutuhan:

- **Mode Default (Hanya Gazebo):**
  ```bash
  source /opt/ros/jazzy/setup.bash
  source install/setup.bash
  ros2 launch robot_bringup spawn_robot.launch.py
  ```
  *(Atau secara eksplisit: `ros2 launch robot_bringup spawn_robot.launch.py mode:=gazebo`)*

- **Mode Full Visualize (Gazebo + RViz2 bersamaan):**
  ```bash
  ros2 launch robot_bringup spawn_robot.launch.py mode:=full_visualize
  ```
  *Membuka jendela Gazebo Sim dan RViz2 secara bersamaan dengan line tracking (jejak lintasan) dan panah arah gerak robot.*

- **Mode RViz (Hanya RViz2, Gazebo headless di background):**
  ```bash
  ros2 launch robot_bringup spawn_robot.launch.py mode:=rviz
  ```
  *Hanya menampilkan jendela RViz2 (simulasi Gazebo tetap berjalan di background secara headless untuk menghemat beban GPU/CPU).*

---

### Langkah 2: Perekaman Waypoint Manual (Opsional / Jika ingin mapping baru)
Jika ingin melakukan pemetaan ulang titik-titik lintasan:

1. **Jalankan Keyboard Teleop (Terminal 2):**
   ```bash
   source /opt/ros/jazzy/setup.bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard
   ```
2. **Jalankan Waypoint Logger (Terminal 3):**
   ```bash
   python3 src/robot_bringup/scripts/waypoint_logger.py
   ```
   - Kemudikan robot menyusuri lintasan dengan tombol `i` (maju), `j` (belok kiri), `l` (belok kanan), `k` (berhenti).
   - Tekan **`ENTER`** / **`SPACE`** di terminal logger untuk menyimpan titik waypoint saat ini.
   - Tekan **`d`** untuk menghapus waypoint terakhir bila meleset.
   - Tekan **`s`** untuk menyimpan semua waypoint ke `waypoints.yaml` dan keluar.

---

### Langkah 3: Menjalankan Navigasi Otomatis Terintegrasi Vision (Terminal 2)
Jalankan node follower untuk menggerakkan robot mengikuti lintasan waypoint sekaligus menerima bantuan koreksi visual untuk tetap berada di tengah-tengah gerbang buoy:
```bash
python3 src/robot_bringup/scripts/waypoint_follower.py
```
*Opsi parameter:*
- Mode looping tanpa henti:
  ```bash
  python3 src/robot_bringup/scripts/waypoint_follower.py --ros-args -p loop:=true
  ```

---

### *(Opsional) Langkah 4: Memantau Tampilan Kamera Real-Time (Terminal 3)*
Node vision C++ (`buoy_centering` dan `obstacle_detector_capture`) **sudah otomatis berjalan bersama simulasi di Terminal 1**. Untuk melihat visualisasi feed kamera real-time:
```bash
ros2 run rqt_image_view rqt_image_view
```
Pilih topik visualisasi pada dropdown menu di kiri atas:
- `/vision/buoy_centering/image` (Pelacakan buoy & garis tengah gerbang)
- `/vision/obstacle_detection/image` (Deteksi & status capture kotak kuning/biru)

---

## Daftar Topik ROS 2

| Nama Topik | Tipe Pesan | Deskripsi |
| :--- | :--- | :--- |
| `/camera/image_raw` | `sensor_msgs/msg/Image` | Citra mentah dari kamera depan robot |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Perintah kecepatan linier dan angular robot |
| `/ground_truth/odom` | `nav_msgs/msg/Odometry` | Posisi absolut robot dari simulasi Gazebo (anti slip) |
| `/imu/data` | `sensor_msgs/msg/Imu` | Data sensor IMU onboard |
| `/vision/gate_detected` | `std_msgs/msg/Bool` | Status visual deteksi gerbang buoy |
| `/vision/centering_error` | `std_msgs/msg/Float32` | Deviasi posisi robot terhadap titik tengah gerbang |
| `/vision/gate_count` | `std_msgs/msg/Int32` | Jumlah gerbang buoy yang berhasil dilewati |
| `/vision/gate_10_passed` | `std_msgs/msg/Bool` | Status apakah gerbang ke-10 telah dilewati (pemicu capture) |
| `/vision/buoy_centering/image` | `sensor_msgs/msg/Image` | Visualisasi debug pemusatan buoy |
| `/vision/obstacle_detection/image` | `sensor_msgs/msg/Image` | Visualisasi debug deteksi kotak rintangan |

---

## Publikasi ke GitHub

File hasil build ROS 2 (`build/`, `install/`, dan `log/`), cache Python, serta gambar hasil capture tidak perlu dipublikasikan. Semua folder tersebut sudah tercantum di [.gitignore](.gitignore). File sumber, model, world, launch, script, `waypoints.yaml`, dan README tetap disimpan di repository.

Untuk membuat repository dan mengirim perubahan ke GitHub:

```bash
git init
git remote add origin https://github.com/youngIcom/diff_drive_robot_simulation.git
git add .
git commit -m "Initial commit"
git branch -M main
git push -u origin main
```

Sebelum menjalankan `git push`, pastikan repository GitHub sudah dibuat dan akun GitHub telah terautentikasi. Jangan memasukkan password, token, atau file rahasia ke dalam repository.

