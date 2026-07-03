## CAに関する注意
	独自CAを使います。現在使用しているCAのcertは[`Robot_Control/MQTT/cert/rootCA.pem`](Robot_Control/MQTT/cert/rootCA.pem)です。サーバー認証用の鍵の実物については問い合わせてください。`localhost`及び`uclab-deskmeet-1`用のサーバー鍵は作成済です。サーバー鍵は`~/.local/share/ssl/`の下にあることにしています。[`./certificates/`](./certificates/)にcdして、[`./mklink.sh`](./certificates/mklink.sh)を実行しすると、そこにsymlinkを作成します。Next.jsのdevサーバーとMosquittoはそれを見に行きます。

[`./Robot_Control/`](./Robot_Control/)の下のクライアントのPythonスクリプトは[`Robot_Control/MQTT/cert/rootCA.pem`](Robot_Control/MQTT/cert/rootCA.pem)を使います。[`MetaworkMQTT.py`managerクライアント](./Mosquitto/MetaworkMQTT/MetaworkMQTT.py)はこの版ではサーバー認証しませんがコマンドラインオプションでrootCAのcertを参照するようにします。

pemファイルの調べ方等は[こちらを見てください](./Robot_Control/MQTT/README.md)。

# Web-Based VR Teleoperation System

<div align="center">
  <img src="./Readme_imgs/system.svg" alt="System Architecture" width="800"/>
  <p><em>Figure 1: System Overview.</em></p>
</div>

**Check our [demo video](https://www.youtube.com/watch?v=mr9DcDyEC9o) here!**

## Quick Start
- [Step 1: Run HTTPS Server](#step-1-run-https-server)
- [Step 2: Build MQTT Broker](#step-2-build-mqtt-broker)
- [Step 3: Debug with WebXR and Simulator](#step-3-debug-with-webxr-and-simulator)
- [Step 4: Run Your Robot](#step-4-run-your-robot)
- [Controller Operations](#controller-operations)
- [Citations](#citations)
- [Python Packages](#python-packages)

---
## Step 1: Run HTTPS Server

💡 **If this is your first time running the project, install the required Node.js modules:**
```bash
npm install
```

**The project is designed to run in VS Code. Download it here:**  
[https://code.visualstudio.com/](https://code.visualstudio.com/Download)

**If you do not have Node.js installed, download it here:**  
[https://nodejs.org/en/download](https://nodejs.org/en/download)

**On Windows, you may need to allow script execution before running the server:**
```powershell
Set-ExecutionPolicy -Scope CurrentUser -ExecutionPolicy RemoteSigned
```

🚀 **To start the Next.js HTTPS server, run:**
```bash
npm run dev-https
```

After starting, you will see two URLs:
- Local:   [https://localhost:3000](https://localhost:3000)
- Network: https://192.168.197.**:****

⚠️ **The Network IP address may vary** depending on your network environment.

<div align="center">
  <img src="./Readme_imgs/npmrun.png" alt="Start Https Server." width="600"/>
  <p><em>Figure 2: Start Https Server.</em></p>
</div>

⚠️ **In VR, only HTTPS can enter VR/AR mode.**

**Open the browser in your VR device and enter `https://192.168.197.**:****` to access the web interface.**

---
## Step 2: Build MQTT Broker

### 1. Install Mosquitto

Download Mosquitto from the official website: [https://mosquitto.org/download/](https://mosquitto.org/download/)

On Windows, Mosquitto is usually installed in:
```
C:\Program Files\mosquitto
```

### 2. Configure Mosquitto

Open `mosquitto.conf` in the installation folder and add the following settings:

**WebSocket (non-secure, ws):**
```
listener 9001
protocol websockets
```
This enables WebSocket connections on port 9001.

**Secure WebSocket (wss):**

First, generate `.pem` certificates.

To generate self-certification files, in the folder `MQTT_Client` run
```bash
node .\generate-ssl-cert.js 
```

Then you can get `cert.pem` and `key.pem` self-certification files.

Copy `cert.pem` and `key.pem` to the installation folder, add the following lines to `mosquitto.conf`:
```
listener 8333
protocol websockets
certfile C:\Program Files\mosquitto\cert.pem
keyfile C:\Program Files\mosquitto\key.pem
allow_anonymous true
```
Port numbers (e.g., 9001, 8333) can be customized.

### 3. Verify the MQTT Broker

**Find your server address:**  
On Windows, run `ipconfig` in the terminal, and look for:

```
IPv4 Address. . . . . . . . . . . : 192.168.197.39
```

Use this IP together with your MQTT port. For example:
```
192.168.197.39:9001   (for ws)
192.168.197.39:8333   (for wss)
```

> ⚠️ **Important: Verify the MQTT port before MQTT communication.**  
> To verify, open your browser and go to your MQTT port. For example:
> ```
> https://192.168.197.39:8333
> ```

### 4. Start the MQTT Broker

After verifying, you can use your local MQTT server for communication in your local network.

**Run Mosquitto as Administrator:**
```
cd "C:\Program Files\mosquitto"
mosquitto -v
```

### 5. Test Your MQTT Broker

To test publishing a topic, navigate to the `MQTT_Client` folder and run:

**Publish a test message:**
```bash
python local_mqtt_test_pub.py
```

**Subscribe to the test message:**
```bash
python local_mqtt_test_sub.py
```

To check all topics in MQTT, run:
```bash
python MQTT_Topic_list.py
```

### 6. Configure Your MQTT Broker URL

The MQTT broker URL depends on your network environment and may differ from the sample code.  
**To update the MQTT broker URL:**

**(1) Web Interface Configuration**  
Open `/src/lib/MetaworkMQTT.js` and set the `MQTT_BROKER_URL` variable to your broker's URL.

> ⚠️ **Remember to verify the new MQTT broker URL after making changes.**

**(2) Robot Controller Configuration**  
Ensure the robot controller uses the same broker URL as the server.  
Open `/Robot_Control/MQTT/MQTT_Client.py` and update the `MQTT_LOCAL_SERVER` variable with your broker's URL.

> ⚠️ **After updating, verify the MQTT connection on your robot controller's PC.**

---
## Step 3: Debug with WebXR and Simulator

### 🥽 Install WebXR Plugin

If you don't have a VR device, you can use WebXR tools for debugging.

On Chrome, you can install the **Immersive Web Emulator** WebXR plugin:

[Immersive Web Emulator (Chrome Web Store)](https://chromewebstore.google.com/detail/immersive-web-emulator/cgffilbpcibhmcfbgggfhfolhkfbhmik)

After installation, press **F12** to open Developer Tools.  
You will find the **WebXR** tab in the developer tools bar, which allows you to emulate VR devices and test WebXR features directly in your browser.

<div align="center">
  <img src="./Readme_imgs/webxr.png" alt="webxr" width="1000"/>
  <p><em>Figure 3: Chorme with WebXR.</em></p>
</div>

### 🧪 Run in Simulator

You can also simulate teleoperation using [CoppeliaSim](https://www.coppeliarobotics.com/).

1. **Download CoppeliaSim**

   Visit the official website to download the latest version: [https://www.coppeliarobotics.com/](https://www.coppeliarobotics.com/)

2. **Launch CoppeliaSim**

   - **On Ubuntu:** Navigate to your CoppeliaSim installation directory and run:
     ```bash
     ./coppeliaSim
     ```
   - **On Windows:** Run the application directly by double-clicking the executable.

3. **Load the simulation scene**

   In CoppeliaSim, open the scene file `"piper_robot_sample.ttt"` located in the `Simulation` folder.

4. **Start the simulation**

   Click the "Play" button in CoppeliaSim to start the simulation.

5. **Open the VR Teleoperation Web Interface in Chrome**

   Visit:
   ```
   https://192.168.197.**:****
   ```
   > Check your terminal to confirm the IP address.

   Click the `AR` button to enter AR mode.

6. **Connect to the MQTT Broker**

   The `MQTT Control ID` is displayed at the top of the web page. Each device has a unique ID.  
   Copy this ID and replace `UserUUID` in `./Robot_Control/MQTT/MQTT_Client.py`.

   In VR, copying the ID directly may be difficult. Alternatively, you can check your control ID using:
   ```bash
   python MQTT_Topic_list.py
   ```
   Each topic published by the VR device will be shown here.

7. **Run the Simulator Control Script**
   ```bash
   python MQTT_Simulation.py
   ```

Now you can control the robot via WebXR.

<div align="center">
  <img src="./Readme_imgs/sim.png" alt="sim" width="1000"/>
  <p><em>Figure 4: Simulation in Coppeliasim.</em></p>
</div>

## Step 4: Run Your Robot

After enter the VR mode, follow the steps below to control the **AgileX-PiPER** robot via MQTT. Currently, the PiPEER robot can only run on **Ubuntu**.

1. **Start the PiPER SDK UI**
      
   🌐 Download the PiPER SDK UI
   
   ```arduion
   https://github.com/agilexrobotics/Piper_sdk_ui.git
   ```

   Open PiPER SDK UI
   ```bash
   cd Piper_sdk_ui
   python piper_ui.py 
   ```

   To reset the robot, open the PiPER SDK Tools and perform the following operations:

      (0) Click **Find CAN Port**
      
      (1) Click **Reset**
      
      (2) Click **Enable**
      
      (3) Click **Go Zero**
      
   🔁 If the robot fails to go to the zero position, repeat steps (1)~(3) a few times until successful.

   To reset the tool, Click **Gripper Zero**

   In this project, right arm use `can0` port and left arm use `can1` port.

2. **Set the Robot to the Initial Working Position**

   In the `Robot_Control` folder, run the following scripts to initialize each arm:

   - **Right Arm:**
     ```bash
     python Initalize_arm_right.py
     ```

   - **Left Arm:**
     ```bash
     python Initalize_arm_left.py
     ```

   - **Cam Arm:**
     ```bash
     python Initalize_arm_cam.py
     ```

3. **Retrieve your USER_UUID**

   > Note: Each device has a unique USER_UUID.  
   > Follow Step 3 in this guide to obtain your USER_UUID.

4. **Run the Robot Controller Script**

   - **Control the Right Arm via MQTT:**
     ```bash
     python MQTT_teleoperation_right.py
     ```

   - **Control the Left Arm via MQTT:**
     ```bash
     python MQTT_teleoperation_left.py
     ```

   - **Control the Cam Arm via MQTT:**
     ```bash
     python MQTT_teleoperation_cam.py
     ```

> (Optional) If you have a [SORA WebRTC SFU](https://sora.shiguredo.jp/) server, you can also try teleoperation using WebRTC's data channel:
> 
> - **Right Arm via WebRTC:**
>   ```bash
>   python WebRTC_teleoperation_right.py
>   ```
> 
> **Other WebRTC applications and their features:**
>
> - [LiveKit](https://github.com/livekit/client-sdk-js):  
>   Open-source, scalable WebRTC platform. Provides SDKs for multiple languages and supports advanced features like recording and media routing. Suitable for real-time communication and robotics teleoperation.
>
> - [Amazon KVS](https://github.com/awslabs/amazon-kinesis-video-streams-webrtc-sdk-js/tree/master):  
>   Node.js-based, cloud-hosted by AWS. Offers secure, scalable signaling and media relay. Good for integration with AWS infrastructure and IoT devices.
>
> - [aiortc](https://github.com/aiortc/aiortc):  
>   Python-based, free and open-source. Lightweight and easy to customize, but requires users to implement signaling and media handling logic. Suitable for research and rapid prototyping.
>
> - Other Open Source WebRTCs (keep updating):
>   https://github.com/rtc-io/rtc-quickconnect 

5. **Visual Assistance**

   Run the following script to enable YOLO-based visual assistance:
   ```bash
   python MQTT_visual_assistance.py
   ```

   There are two modes available: **teleoperation** and **visual assistance**.  
   You can switch modes by pressing the right thumbstick down.
   

## Controller Operations

🎮 The following input mappings are used to operate the PiPER robot via the VR controller:

| Input Combination                              | Action                                 |
|------------------------------------------------|----------------------------------------|
| Trigger Right + Motion                         | Right Arm Move                         |
| Button A                                       | Right Gripper Grasp                    |
| Button B                                       | Right Gripper Release                  |
| Trigger Left + Motion                          | Left Arm Move                          |
| Button X                                       | Left Gripper Grasp                     |
| Button Y                                       | Left Gripper Release                   |
| Grip Left + Left thumbstick moved (↕️ ↔️)      | Cam Arm Translation Up/Down/Left/Right |
| Grip Left + Right thumbstick moved (↕️)        | Cam Arm Translation Forward/Back       |
| Grip Right + Right thumbstick moved (↕️ ↔️)    | Cam Arm Rotation Y/X                   |
| Grip Right + Left thumbstick moved (↕️)        | Cam Arm Rotation Z                     |
| Left thumbstick down                           | Call Menu                              |
| Right thumbstick down                          | Change mode (Teleoperation/Visual Assistance) |
| Right thumbstick moved (⬆️)                    | Visual Assistance Move Flag Signal     |
| Right thumbstick moved (⬇️)                    | Robot Return                           |

> Make sure the controller is tracked and visible to the VR camera to ensure accurate input.


### ⚠️ Notifications

1. **Keep the VR controller within camera view**  
   The pose of the VR controller is estimated using both the onboard **accelerometer** and the **tracking camera** located on the side of the VR headset.  
   > ⚠️ If the controller goes out of view, pose estimation may become inaccurate, resulting in input drift.

2. **Wait for system initialization**  
   After putting on the VR headset or restarting the system, **always wait until initialization is complete**.  
   Skipping this step may result in control drift or unstable input.
   
   > ⚠️ If the controller appears frozen or unresponsive, it may indicate a tracking issue.
   
   > ✅ If you can see the controller moving in sync with your hand, it is functioning correctly.  

## Citations

The inverse kinematics (IK) implementation in this project is based on the **Modern Robotics** library by Kevin Lynch et al.

- 📘 Book: *Modern Robotics: Mechanics, Planning, and Control*  
- 💻 Source Code: [NxRLab/ModernRobotics GitHub Repository](https://github.com/NxRLab/ModernRobotics)

⚠️ In the original library, the function `AxisAng3` has a bug when the input vector is a zero vector.

**Please change the function to:**
```python
def AxisAng3(expc3):
    norm = np.linalg.norm(expc3)
    if NearZero(norm):
        return (Normalize(expc3), 0)
    else:
        return (Normalize(expc3), np.linalg.norm(expc3))
```

Or use the modified library in `Robot_Control/Modern_Robotics/core.py`.

## Python Packages

It is recommended to use a `conda` environment to manage dependencies, for example:

```bash
conda create -n Modern_Robotics_Control_IK python=3.12
conda activate Modern_Robotics_Control_IK
```

If you forgot your conda environment name:
```bash
conda info --envs
```
to check your conda environemnt list.

### Dependencies
📐 Math & Utilities
```bash
pip install numpy
```

📷 Visual (OpenCV, YOLO)
```bash
pip install opencv-python
pip install ultralytics
```

📡 Communication (MQTT, WebRTC)
```bash
pip install paho-mqtt
pip install sora-sdk
```

🤖 Robotics
```bash
pip install piper-sdk
pip install modern-robotics
```

🧪 Simulator (CoppeliaSim Remote API)
```bash
pip install coppeliasim-zmqremoteapi-client
```


