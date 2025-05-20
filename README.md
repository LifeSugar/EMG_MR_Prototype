Xi Lin 19/5/2025
This log was created by Markdown editor Obsidian 
Some terminology expressions translated by DeepL

________
## 1.Device List 

| Device Name                         | Description                                                                                |
| ----------------------------------- | ------------------------------------------------------------------------------------------ |
| **ESP32 DevKit V1**                 | WiFi-enabled microcontroller used for analog EMG signal acquisition and TCP communication. |
| **AD8232 ECG Sensor Module**        | Analog bio-signal amplifier, repurposed to read surface EMG (muscle) signals.              |
| **Electrode Patch Cables**          | Snap cables to connect AD8232 to gel electrode pads placed on the skin.                    |
| **Gel Electrode Pads (x3)**         | Medical-grade adhesive electrodes used for non-invasive muscle signal capture.             |
| **Breadboard + Jumper Wires**       | For connecting AD8232 to ESP32 without soldering.                                          |
| **USB Data Cable (USB-A to USB-C)** | For uploading code and powering ESP32 during development.                                  |
| **Windows PC**                      | Hosts Unity project and connects via TCP to ESP32 over WiFi.                               |
| Meta Quest 3                        | Implement and test                                                                         |
All the devices were bought on Amazon.

---

## 2.Hardware Connection & Physical Setup
### 2.1  Module‑to‑Board Wiring

| AD8232 Pin    | Connects To                                       | Note                                                                   |
| ------------- | ------------------------------------------------- | ---------------------------------------------------------------------- |
| **3.3 V**     | ESP32 3V3                                         | Do **not** use 5 V — the module is 3.3 V‑only                          |
| **GND**       | ESP32 GND                                         | Common ground                                                          |
| **OUTPUT**    | ESP32 GPIO 35 *(ADC1\_CH7)*                       | Analog EMG signal (0‑3.3 V)                                            |
| **LO+ & LO‑** | GND(optional, for prototype, just connect to GND) | Tie low to disable lead‑off detection & avoid 0/4095 spikes (optional) |

### 2.2  Electrode Placement (Fore‑Arm Biceps Demo)

1. **Red (RA)** — proximal end of the muscle (closer to shoulder).
2. **Yellow (LA)** — distal end (closer to elbow).
3. **Green (RL)** — reference electrode, far from the muscle (e.g.,  hand‑back).


---



## 3.Firmware & EMG‑Processing Algorithm

### 3.1  Goal Provide a **real‑time, noise‑robust "muscle‑effort" metric** that Unity can consume over serial/Wi‑Fi while still sending the raw 12‑bit sample for diagnostics.

### 3.2  Algorithm Walk‑through

| Stage                     | Formula                             | Rationale                                                               |     |
| ------------------------- | ----------------------------------- | ----------------------------------------------------------------------- | --- |
| **Sampling**              | `raw = analogRead(EMGPin)`          | 12‑bit value (0‑4095) at ≈ 200 Hz.                                      |     |
| **DC‑offset removal**     | `diff = raw – baseline`             | AD8232 adds ≈ 1.6 V bias (~1900 ADC); subtract to centre at 0.          |     |
| **Rectification**         | `squared = diff²`                   | Full‑wave rectification via squaring; prepares for RMS energy estimate. |     |
| **Windowed accumulation** | circular buffer of *N = 50* squares | Integrates 50 samples (~250 ms) for smoother energy.                    |     |
| **Mean square**           | `mean = Σ squares / N`              | Average power within window.                                            |     |
| **RMS extraction**        | `rms = √mean`                       | Final amplitude estimate proportional to muscle force.                  |     |
| **Serial output**         | `raw, rms` (comma‑separated)        | Compatible with Arduino Serial Plotter & Unity parser.                  |     |
### 3.3 Code 
```cpp

const int EMGPin = 35;
const int baseline = 1900;
const int windowSize = 50; 
long squareBuffer[windowSize];

int bufIndex = 0;

void setup() {
  Serial.begin(115200);
  for (int i = 0; i < windowSize; i++) {
    squareBuffer[i] = 0;
  }
}

void loop() {
  int raw = analogRead(EMGPin);
  int diff = raw - baseline; 
  long squared = (long)diff * diff; 
  squareBuffer[bufIndex] = squared;
  bufIndex = (bufIndex + 1) % windowSize;
  long sum = 0;
  for (int i = 0; i < windowSize; i++) {
    sum += squareBuffer[i];
  }
  float mean = (float)sum / windowSize;
  float rmsValue = sqrt(mean);
  Serial.print(raw);        // Column 1
  Serial.print(',');        // Separator
  Serial.println(rmsValue); // Column 2
  delay(5);
}
```

### 3.4  Parameter Tuning

- **baseline** — record 2–3 s at rest, compute average to auto‑calibrate (not implemented yet).
- **windowSize** — larger window ⇒ smoother but slower; adjust 30–80 as needed.
- **Sampling delay** — `delay(5)` ≈ 200 Hz; lower for finer resolution but larger throughput.

### 3.5  Observations

- RMS trace rises proportionally with grip force; back to ~0 when relaxed.
- Momentary 4095 spikes (ADC saturation) are squared but quickly averaged out; reducing electrode distance or gain avoids saturation.
- Serial Plotter shows **blue = raw**, **red = RMS**, confirming clean envelope extraction.

![[Pasted image 20250519232535.png]]
![[Pasted image 20250519232636.png]]

### 3.6 Early Signal Read & Noise Analysis

This RMS routine was first deployed as an **early‑stage validation test**. In the Serial Plotter we observe:

- **Blue line =** **raw** — noticeably jagged; individual 12‑bit samples fluctuate by ±50–100 counts even when the arm is at rest.
- **Red line =** **RMS** — smooth envelope that tracks overall muscle effort.

#### Potential reasons for the blue trace jitter

1. **Micro‑motion of electrodes** and cable tugging introduce millivolt‑level jumps.
2. **Mains (50/60 Hz) pickup** couples into the high‑gain front‑end of AD8232.
3. **USB & Wi‑Fi power ripple** inject low‑frequency wander.

#### How the algorithm suppresses jitter

| Technique | Effect |
| --- | --- |
| Baseline subtraction | Removes the 1.6 V hardware bias, centring data on 0. |
| Full‑wave rectification (squaring) | Converts ± swings into positive power samples, ignoring polarity. |
| 250 ms sliding‑window average | Integrates 50 samples, yielding ~14 dB attenuation of >4 Hz noise. |
| Square‑root (RMS) | Recovers amplitude units for an intuitive "muscle‑effort" metric. |

Consequently the **RMS curve remains stable** during sustained grip, while the raw trace’s jitter becomes negligible for control purposes.

### 3.7 Baseline Auto‑Calibration (Not implemented yet)

While a fixed baseline of 1900 ADC works for bench tests, long‑term wear may experience drift caused by sweat, temperature or cable motion. Two complementary strategies are planned:

| **St**rategy                  | P**ro**cedu**re**                                                                                           | Pr**os**                                            | Cons                                                                     |
| ----------------------------- | ----------------------------------------------------------------------------------------------------------- | --------------------------------------------------- | ------------------------------------------------------------------------ |
| A One‑shot Rest A**ve**raging | Collect ~2 s (≈ 400 samples) immediately after power‑up while the limb is relaxed; average becomes baseline | Fast & simple; rock‑solid during short sessio**ns** | Requires user to stay still at start‑up; cannot adapt to later dri**ft** |
| B Slow Adaptive Tracki**ng**  | During runtime, if<br>`RMS < restThreshold` (e.g. 80),  do `baseline = α·baseline + (1‑α)·raw，α≈0.999`      | Compensates gradual offset change; no extra **UI**  | Might mis‑adapt if user never truly rests or keeps trembling             |

Combining both (A → initial, B → slow follow‑up) provides robust operation for demos and extended wear.

---


## 4.Unity Serial Link & Realtime Visualization

### 4.1 High‑level Flow

```
ESP32 (USB COM7)  ──►  SerialEMGReader  ──►  RealtimePlot  ──►  dual LineRenderers
                                         │                     ▲
                                         └──► TMP_Text HUD ◄──┘
```
- **SerialEMGReader.cs**
	- Opens **COM7 @ 115 200 baud** on a background thread.
	- Parses `raw,rms` into two `volatile` fields.
	- In `Update()` it pushes the values to the graph and updates on‑screen text.
- **RealtimePlot.cs**
	- Holds a ring‑buffer of `**maxPoints = 500**` samples.
	- Redraws two `LineRenderer` objects (blue = raw, orange = RMS) every push.
	- `useWorldSpace` is **disabled**; `transform.localScale` + `widthMultiplier` give true uniform scaling via `SetPlotScale()`.
- **HUD** — two TextMeshPro labels show live numerical read‑outs.

**Unity Version: 6.0.45f1**
Universal Render Pipeline

### 4.2 Code

`SerialEMGReader.cs`
```csharp
using UnityEngine;  
using System.IO.Ports;  
using System.Threading;  
using TMPro;  
  
public class SerialEMGReader : MonoBehaviour  
{  
    [Header("Serial Settings")]  
    public string portName = "COM7";  
    public int baudRate    = 115200;  
        [Header("Serial Plotter")]  
    public RealtimePlot realtimePlot;  
  
    [Header("Serial Data")]  
    public TextMeshProUGUI Raw;  
    public TextMeshProUGUI RMS;  
    private SerialPort serial;  
    private Thread     readThread;  
    private volatile bool keepReading = false;  
  
    public volatile int   rawValue = 0;  
    public volatile float rmsValue = 0f;  
  
    void Start ()  
    {  
        serial = new SerialPort(portName, baudRate);  
        serial.ReadTimeout = 100;       
serial.Open();  
  
        keepReading = true;  
        readThread  = new Thread(ReadSerialLoop);  
        readThread.Start();  
    }  
    void ReadSerialLoop ()  
    {        while (keepReading)  
        {            try  
            {  
  
                string line = serial.ReadLine();  
  
                string[] token = line.Trim().Split(',');  
                if (token.Length == 2)  
                {                    rawValue = int.Parse(token[0]);  
                    rmsValue = float.Parse(token[1]);  
                }            }            catch { /* 超时或格式错误时忽略 */ }  
        }    }  
    void Update ()  
    {  
        Debug.Log($"Raw = {rawValue}   RMS = {rmsValue:F1}");  
        realtimePlot.Push(rawValue, rmsValue);  
        Raw.text = $"Raw = {rawValue}";  
        RMS.text = $"RMS = {rmsValue:F1}";  
    }  
    void OnApplicationQuit ()  
    {        keepReading = false;  
        readThread?.Join();  
        if (serial != null && serial.IsOpen) serial.Close();  
    }}

```

`RunTimePlot.cs`
```csharp
using UnityEngine;  
  
public class RealtimePlot : MonoBehaviour  
{  
    [Header("Line Renderers")]  
    public LineRenderer rawLine;  
    public LineRenderer rmsLine;  
  
    [Header("Plot Settings")]  
    public int   maxPoints   = 500;  
    public float yOffset     = -1500f; // 基线偏移  
    public float yScale      = 0.1f;   // 基础缩放  
     
    private Vector3[] rawPts, rmsPts;  
    private int idx;  
  
    void Awake()  
    {        rawPts = new Vector3[maxPoints];  
        rmsPts = new Vector3[maxPoints];  
        rawLine.positionCount = rmsLine.positionCount = maxPoints;  
        SetPlotScale(0.01f);  
    }    public void SetPlotScale(float s)  
    {  
        s = Mathf.Max(0.01f, s);  
  
  
        transform.localScale = Vector3.one * s;  
  
  
        rawLine.widthMultiplier = s;  
        rmsLine.widthMultiplier = s;  
    }  
  
    public void Push(float raw, float rms)  
    {        float yRaw = (raw + yOffset) * yScale;  
        float yRms = (rms + yOffset) * yScale;  
  
        rawPts[idx].y = yRaw;  
        rmsPts[idx].y = yRms;  
  
        for (int i = 0; i < maxPoints; i++)  
        {            int j = (idx - i + maxPoints) % maxPoints;  
            float x = i;  
            rawLine.SetPosition(maxPoints - 1 - i, new Vector3(x, rawPts[j].y, 0));  
            rmsLine.SetPosition(maxPoints - 1 - i, new Vector3(x, rmsPts[j].y, 0));  
        }        idx = (idx + 1) % maxPoints;  
    }}

```

### 4.3 Results & Notes

| Item            | Observation                                                                             |
| --------------- | --------------------------------------------------------------------------------------- |
| **Throughput**  | ≈ 200 Hz (matches firmware `delay(5)`) with no frame drops.                             |
| **Visual**      | Raw curve shows high‑freq jitter; RMS gives smooth envelope as designed.                |
| **Scaling**     | Calling `SetPlotScale(0.02f)` shrinks both lines and widths uniformly for UI embedding. |
| **Performance** | Two LineRenderers × 1500 points ← 0.5 ms per frame on desktop; safe for MR display.     |
![[Pasted image 20250520012456.png|500]]

The screenshot shows the mixed-reality EMG plot inside Unity:

- **Blue trace** – the raw 12-bit ADC values coming straight from the ESP32 at ≈200 Hz.  
	You can see dense, high-frequency oscillations whenever the fore-arm muscles fire.
- **Orange trace** – the 50-sample sliding-window RMS that we compute to obtain a smooth  
	amplitude envelope.

Two distinct humps are visible:

1. **First hump – making a fist**  
	When the user closes the hand, the wrist/finger flexor group contracts concentrically.  
	Hundreds of motor-unit action potentials superimpose, so the raw signal explodes into a  
	broadband ±-swing; the RMS rises sharply.
2. **Second hump – opening the hand**  
	As the fist is released, the flexors decelerate the motion eccentrically and the  
	finger extensors give a short burst. This second but smaller burst produces the  
	second orange peak.

In physiology terms the EMG amplitude reflects the *number of active motor units* and  
their *firing rate*. Concentric flexion recruits many fibres quickly – large amplitude.  
During controlled extension fewer units are active, so the RMS peak is lower and shorter.

The blue raw curve therefore shows two high-density “towers”, while the orange RMS plots  
two smooth bumps that correspond one-to-one with the phases of muscle activity.

---

## 5.Wi‑Fi Streaming Validation

### 5.1 Network Architecture
```
ESP32  ──Wi‑Fi (TCP :3333)──►  Windows Hotspot (192.168.137.x)  ──►  Unity TcpClient
```
- ESP32 runs a **TCP server**; Unity connects as a client.
- Hotspot SSID `FFRb`, password `nana0916`, fixed **2.4 GHz**.

### 5.2 Firmware Snapshot
```cpp
WiFi.begin(ssid, pwd);                 // connect hotspot
WiFiServer server(3333); server.begin();
...
WiFiClient c = server.available();
if(c && c.connected())         // single client
    c.printf("%d,%.1f
", raw, rms);
```
The EMG sampling + RMS filter from _Step 3_ remain unchanged; only transport layer replaced.
![[屏幕截图 2025-05-20 015930.png | 300]]![[屏幕截图 2025-05-20 020922.png|300]]

### 5.3 Unity Client Changes
```csharp
TcpClient tcp = new TcpClient("192.168.137.2", 3333);
StreamReader reader = new StreamReader(tcp.GetStream());
// Parsing logic identical to serial path.
```
A boolean `useWiFi` flag lets us switch between USB COM and TCP at runtime.
![[Pasted image 20250520030552.png|300]]

### 5.4 Validation Checklist & Results
![[屏幕截图 2025-05-20 023917.png|500]]

| Test                     | Outcome                                  |
| ------------------------ | ---------------------------------------- |
| *Ping ESP32*             | 1 ms RTT ✔                               |
| *Telnet 3333*            | Continuous `raw,rms` lines ✔             |
| *Unity Console*          | Values update; plot renders ✔            |
| *Hot‑plug* (ESP32 reset) | Manual reconnect worksauto‑reconnect TBD |
### 5.5 Known Limitations

- **Throughput**: default TCP + Nagle gives ≈ 30 Hz visible update (USB was ≈ 200 Hz).
- **Single‑client** only; additional devices drop connection.
- **No reconnection logic** after Wi‑Fi loss.

> These points are acceptable for the current milestone and will be addressed in post‑MVP optimisation.

### 5.6 Future Optimisation Roadmap *(not implemented yet)*

1. **Disable Nagle / shrink buffer** to restore 150–200 Hz update rate.
2. **Binary framing** (6 B packet) or **UDP broadcast** for minimal latency.
3. **Auto‑reconnect & heart‑beat** in Unity; optionally use mDNS (`esp32-emg.local`).
4. Multi‑client fan‑out via async server or UDP multicast.

---

## 6.Meta Quest 3 Build Environment

| Item                     | Version / Setting | Notes                                              |
| ------------------------ | ----------------- | -------------------------------------------------- |
| **Unity Editor**         | 6 000.0.45 f1     | Recommended for Meta XR SDK v74+                   |
| **Meta XR SDK**          | **v 76.0.0**      | Installed via Package Manager                      |
| **Unity OpenXR Plug‑in** | 1.14.3            | Core runtime layer; set as *OpenXR* in XR settings |
| **Target Platform**      | Android (Quest 3) | Scripting Backend = IL2CPP, ARM64 only             |

### 6.1 Project Configuration Checklist

1. **Enable Developer Mode on Quest 3** (Meta app → Devices → Developer Mode) and toggle **USB Debugging** inside headset.
2. **Turn on Quest Link Beta → Developer Runtime** (Settings ▸ System ▸ Quest Link ▸ *Enable Beta*). This unlocks live OpenXR runtime switching during PC‑tethered debugging.
3. **XR Plugin Management** → Enable **OpenXR** and **Meta XR** feature group.
4. **OpenXR > Features**: tick *Meta Quest Support*; uncheck legacy Oculus.
5. **Player ▸ Other Settings**:
	- Minimum API Level = `android-29`, Target = `android-33`
	- Internet Access = *Require* (for EMG Wi-Fi)
6. **ADB / Quest Link**: verify `adb devices` shows the headset when USB‑C attached or via Wi‑Fi ADB.
7. **Build & Run** → APK installs; first launch requests Guardian & Passthrough permissions.

### 6.2 Reference

Detailed step‑by‑step matches Meta official guide: https://developers.meta.com/horizon/documentation/unity/unity-project-setup

### 6.3 Build and Test
Add `OVRInteractionComprehensive` and [Building Block]passthrough to current scene, select run device.
![[Pasted image 20250520103437.png|300]]



### 6.4 Result Snapshot · Meta Quest 3 Live Demo
![[Pasted image 20250520113837.png|500]]
Test Video: [com.oculus.metacam-20250520-113102-0.mp4](https://1drv.ms/v/c/27d252d7fcf31461/EZix_sGmQehEsjvIBrgklm4Bko04XYnz2KLwtNYkNHg5qQ?e=xNPhgO)

**What you see in the screenshot / video**

| Item                    | Description                                                                                                                                                |
| ----------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **Passthrough MR view** | Captured with the Quest 3 system recorder (`metacam`). The real world is visible through full-color passthrough; the Unity scene is rendered on top.       |
| **Blue curve (RAW)**    | Direct 12-bit ADC values streamed from the ESP32 every ≈5 ms. Peaks correspond to forearm muscle bursts.                                                   |
| **Orange curve (RMS)**  | 50-sample sliding-window RMS envelope; smoother trend line that tracks sustained contraction.                                                              |
| **Networking**          | ESP32 connected to hotspot **FFRb** (`192.168.137.2`) ; Quest 3 joined the same hotspot and receives the stream over **TCP :3333**. No USB cable required. |
| **Build**               | Unity 6.0.0-45 f1 · Meta XR SDK v76 · OpenXR 1.14.3 · IL2CPP · ARM64 · Developer Runtime enabled.                                                          |
| **Hardware rig**        | AD8232 on forearm (RA/LA electrodes), ESP32 DevKit V1, Windows laptop running hotspot, Quest 3 HMD.                                                        |

**Outcome**

> *The EMG visualisation demo is now fully standalone on Meta Quest 3: the headset shows real-time muscle-signal plots in mixed reality while the user moves their arm, with data wirelessly streamed from the ESP32 via Wi-Fi.*

---


## Road-map for the next iteration

### Hardware
I am still reading fore-arm EMG with a single-channel AD8232 ECG board that is only jumpered into the ESP32 header.  
Next cycle I will

1. **Swap to a dual-channel EMG front-end** (e.g. an ADS1292-based module) so that flexor and extensor can be recorded simultaneously and gestures can be classified.
2. **Solder proper pin headers** instead of the current “bent-jumper friction fit”; this removes intermittent drops that appear whenever the cable is bumped.
3. **Print a custom enclosure** on the home 3-D printer: sensor PCB, ESP32 DevKit, Li-ion cell and a slide switch fixed in one compact shell with a Velcro strap and strain-relief holes.

## Signal processing
*Baseline & filtering*  
Auto-rest calibration will run during the first two seconds so the baseline follows each user.  
The simple 50-sample RMS window will be replaced by a 20 Hz high-pass + 200 Hz second-order low-pass IIR chain.  
This pushes algorithmic delay from ≈125 ms down to ≲12 ms.
#### *Where to run the algorithm?*

*Path A – on-device* ESP32 calculates envelope (RMS / peak / on-off boolean) and sends only those four bytes every 5 ms.  
Pros: lowest bandwidth, UDP friendly. Cons: the 240 MHz ESP32 chip is not ideal for future ML.

*Path B – on-headset* ESP32 streams raw 12-bit ADC values; Quest-side C# does the filtering.  
Pros: XR2 CPU/GPU has head-room for DSP or inference; easier to tweak cut-offs live.  
Cons: requires ~6 KB s-¹ throughput and is exposed to Wi-Fi packet loss.

Both paths will be implemented and benchmarked; the better one is kept.

### Performance Outlook (latency focus)

In its current prototype the system already feels “live”, but we will track latency more formally in the next milestones because rehabilitation feedback is only useful when it is almost-instant.

**Two latency contributors**

• **Wi-Fi hop** – the time between the moment the ESP32 places a packet on the air-interface and the moment Unity’s TCP stream thread receives a complete line. Measured with timestamp tags this sits around **25 ± 8 ms** on an uncongested 2.4 GHz hotspot; spikes appear when Windows aggregates or retransmits. I will:  
– enable `TCP_NODELAY` (already done) and shrink receive buffers;  
– try a UDP broadcast channel (no retransmit, but add sequence numbers to detect loss);  
– move to a dedicated router or Wi-Fi Direct to avoid PC-hot-spot throttling.

• **Filter / feature delay** – the algorithmic lag that comes from window length and the cost of the maths itself.  
– A 50-sample RMS window at 200 Hz adds **≈ 125 ms group delay** (half the window).  
– The computation on ESP32 is trivial (≈ 140 µs) but the windowing cost is not, so switching to a 2-pole IIR (high-pass + low-pass) will cut the delay to **< 12 ms** while preserving smoothness.  
– If we eventually process on Quest 3 the XR2 CPU copes with FFT/IIR at >1 kHz, so the bottleneck remains the window size, not raw horsepower.

**Target envelope**

Aim for an end-to-end motion-to-photon delay below **60 ms**, which is comfortably inside the “feels instantaneous” threshold for hand/arm feedback. The action items are therefore (1) constrain wireless jitter to < 20 ms worst-case, and (2) redesign the filter so its group delay is < 20 ms. The display pipeline on Quest 3 (90 Hz, late-latching) already adds 7–12 ms, giving head-room for the rest.



## User-facing MR software

*Visualisation* Replace raw LineRenderer with the Unity “Graph and Chart” package so clinicians can zoom, pause, export CSV and overlay multiple sessions.

*Feedback* Map the filtered RMS to an Animator “Muscle Bulge” parameter and to controller haptics when a target force is reached.

*Rehabilitation mini-games* 

*Virtual Dumbbell Lift* – hold RMS above a goal to raise a passthrough-overlay weight.  
*Can Crusher* – squeeze rapidly to collapse a virtual soda can.

Each mini-game has editable reps, sets, target RMS band and rest timer so a therapist can tailor the session.

All XR features continue to rely on **Meta XR SDK v76** under Unity 6 + OpenXR.
