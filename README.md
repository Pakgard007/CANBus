

---

# CAN Bus Motor Control System

## 📌 Overview

ระบบนี้เป็นโปรแกรมควบคุมมอเตอร์ผ่าน **CAN Bus** โดยรองรับการรับค่ามุมจาก ROS 2 topic (`/cmd_angle`) แล้วแปลงเป็นค่าตำแหน่งเป้าหมาย พร้อมส่งคำสั่งแบบ **SDO (Service Data Object)** ไปยังมอเตอร์
นอกจากนี้ยังมีระบบ **smooth transition** เพื่อให้การหมุนเป็นไปอย่างนุ่มนวล และระบบ **feedback monitoring** เพื่อตรวจสอบการตอบสนองของมอเตอร์

---

## ⚙️ System Structure

### 1. การเริ่มต้นใช้งาน CAN Bus

* ใช้คำสั่ง `os.system()` เพื่อกำหนดค่าพารามิเตอร์ของ CAN Interface (`can0`)
* ตั้งค่า **bitrate** และ **queue length**
* ใช้ไลบรารี `python-can` สร้างอินเทอร์เฟซสำหรับส่งและรับข้อมูล

```python
os.system("sudo ip link set can0 down")
os.system("sudo ip link set can0 type can bitrate 500000 restart-ms 100")
os.system("sudo ip link set can0 txqueuelen 1000")
os.system("sudo ip link set can0 up")
bus = can.interface.Bus(channel='can0', bustype='socketcan')
```

---

### 2. การ Enable มอเตอร์

* ส่งคำสั่ง **SDO** ไปยัง Object Dictionary `0x200D` เพื่อเปิดใช้งานมอเตอร์

```python
enable_msg = can.Message(
    arbitration_id=0x601,
    data=[0x2B, 0x0D, 0x20, 0x00, 0x01, 0x00, 0x00, 0x00],
    is_extended_id=False
)
bus.send(enable_msg)
```

---

### 3. การ Subscribe ข้อมูลจาก Topic `/cmd_angle`

* รับมุมเป็นองศาจาก ROS 2 (`std_msgs/Float32`)

```python
def angle_callback(msg):
    angle_deg = msg.data
    # ... process angle here
```

---

### 4. การ Mapping มุมไปเป็นค่าตำแหน่ง

* ใช้ฟังก์ชัน `map_range()` แปลงค่ามุมจากช่วง `-5° ถึง +5°`
  → เป็นค่าตำแหน่ง `-50 ถึง +50`

```python
def map_range(value, in_min, in_max, out_min, out_max):
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

target_position = map_range(angle_deg, -5, 5, -50, 50)
```

---

### 5. ฟังก์ชันควบคุมการหมุนแบบ Smooth

* ลดความกระชากด้วยการส่งตำแหน่งทีละขั้น (`step size = 10`)

```python
def smooth_position_transition(current, target, step=10):
    while current != target:
        if abs(target - current) < step:
            current = target
        elif target > current:
            current += step
        else:
            current -= step
        send_position_command(current)
```

---

### 6. การส่งคำสั่งตำแหน่ง

* ส่งคำสั่งไปยัง Object Dictionary `0x2002` (Target Position)
* แปลงข้อมูลเป็น **Little Endian**

```python
pos = int(target_position)
data_le = pos.to_bytes(4, byteorder='little', signed=True)

msg_position = can.Message(
    arbitration_id=0x601,
    data=[0x23, 0x02, 0x20, 0x01] + list(data_le),
    is_extended_id=False
)
bus.send(msg_position)
```

---

### 7. การ Monitor Feedback

* ใช้ **thread** แยกเพื่อฟังข้อมูลจาก Feedback ID
* แสดงผลใน console เพื่อตรวจสอบการตอบสนอง

```python
def feedback_listener():
    while True:
        msg = bus.recv()
        if msg and msg.arbitration_id == feedback_id:
            print("Feedback:", msg.data)
```

---

### 8. การปิดระบบ

* ส่งคำสั่ง **disable motor**
* ปิด CAN interface

```python
disable_msg = can.Message(
    arbitration_id=0x601,
    data=[0x2B, 0x0D, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00],
    is_extended_id=False
)
bus.send(disable_msg)

os.system("sudo ip link set can0 down")
```

---

## 🛠 Example Usage

```python
target_position = map_range(angle_deg, -5, 5, -50, 50)
send_position_command(target_position)
```

