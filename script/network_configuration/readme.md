# 🌐 Switch AiNex to Client Mode via Configuration File

---

## ✅ Step-by-Step Instructions

### 🖥️ 1. Connect to AiNex via VNC
- Use **VNC Viewer** to connect (default IP: `192.168.149.2`)
- **Username:** `pi`  
- **Password:** `raspberry` *(or `raspberrypi` on Pi 5)*
- (optional) can use monitor to set-up wifi

---

### 💻 2. Open Terminal on AiNex
- Double-click the **Terminal** icon on the desktop

---

### 📁 3. Open Wi-Fi Config File
```bash
cd wifi_manager/
gedit wifi_conf.py
```

- Change mode from AP to **Client (LAN)**:
```python
HW_WIFI_MODE = 2  # 1 = AP, 2 = Client
HW_WIFI_STA_SSID = "YourWiFi"
HW_WIFI_STA_PASSWORD = "YourPassword"
```

- Save and exit:

---

### 🧭 4. Set a Static IP
```bash
gedit wifi.py
```
- Change this line:
```python
HW_WIFI_AP_GATEWAY = "192.168.1.105"
```

- Save and exit 

---

### 🔄 5. Restart Wi-Fi Service
```bash
sudo systemctl restart wifi.service
```

---

### ✅ 6. Reconnect via New IP
- Wait ~15 seconds for AiNex to join your Wi-Fi
- Use VNC Viewer with the **new IP** (e.g., `192.168.1.105`)
- Login using:
  - **Username:** `pi`
  - **Password:** `raspberrypi`
