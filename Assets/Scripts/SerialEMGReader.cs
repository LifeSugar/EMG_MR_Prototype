using UnityEngine;
using System.IO.Ports;
using System.Net.Sockets;
using System.IO;
using System.Threading;
using TMPro;

public class EMGReader : MonoBehaviour
{
    public bool useWiFi = true;
    public bool useLineRenderer;

    [Header("Serial")]
    public string com = "COM7";
    public int    baud = 115200;

    [Header("Wi-Fi")]
    public string ip = "192.168.137.2";
    public int    port = 3333;
    public TextMeshProUGUI wifiInfo;

    [Header("UI/Plot")]
    public RealtimePlot plot;
    public TextMeshProUGUI rawTMP;
    public TextMeshProUGUI rmsTMP;
    // public UGUIWavePlot rawWavePlot;
    // public TextMeshProUGUI rmsWavePlot;
    
    public LineDrawer rawLine;
    public LineDrawer rmsLine;
    
    SerialPort  serial;
    TcpClient   tcp;
    StreamReader reader;
    Thread      t;
    volatile bool run;
    public volatile int   rawVal = 0;
    public volatile float rmsVal = 0f;

    void Start()
    {
        if (useWiFi)
            StartWiFi();
        else
            StartSerial();

        run = true;
        t = new Thread(ReadLoop);
        t.Start();
    }

    void StartSerial()
    {
        serial = new SerialPort(com, baud){ ReadTimeout = 200 };
        serial.Open();
        Debug.Log("<color=cyan>[EMG] Serial opened</color>");
    }

    void StartWiFi()
    {
        try{
            tcp = new TcpClient();
            tcp.Connect(ip, port);
            tcp.NoDelay = true;
            reader = new StreamReader(tcp.GetStream());
            Debug.Log($"<color=cyan>[EMG] Wi-Fi connected {ip}:{port}</color>");
            wifiInfo.text = $"[EMG] Wi-Fi connected {ip}:{port}\n";
            
        }catch(System.Exception e){
            Debug.LogError($"TCP connect fail: {e.Message}");
            wifiInfo.text = $"[EMG] Wi-Fi connect fail: {e.Message}";
        }
    }

    void ReadLoop()
    {
        while (run)
        {
            try{
                string line = useWiFi ? reader?.ReadLine() : serial.ReadLine();
                if (string.IsNullOrEmpty(line)) continue;

                string[] tok = line.Trim().Split(',');
                if (tok.Length == 2){
                    rawVal = int.Parse(tok[0]);
                    rmsVal = float.Parse(tok[1]);
                }
            }
            catch(System.Exception ex)
            {
                Debug.LogWarning($"Read err: {ex.Message}");
                if (useWiFi){          // 尝试重连
                    Thread.Sleep(1000);
                    StartWiFi();
                }
            }
        }
    }

    void Update()
    {
        if (useLineRenderer)
            plot.Push(rawVal, rmsVal);
        if (showRaw)
            rawLine.PushSample(rawVal);
        rmsLine.PushSample(rmsVal);
        rawTMP.text = $"Raw = {rawVal}";
        rmsTMP.text = $"RMS = {rmsVal:F1}";
        // Debug.Log($"{rawVal}, {rmsVal:F1}");
    }

    void OnApplicationQuit()
    {
        run = false;
        t?.Join();
        serial?.Close();
        tcp?.Close();
    }
    
    private bool showRaw = false;
    public TextMeshPro showRawTMP;
    public void SwitchRaw()
    {
        if (showRaw)
        {
            rawLine.gameObject.SetActive(false);
            showRaw = false;
            showRawTMP.text = "Show\nRaw";
        }
        else
        {
            rawLine.gameObject.SetActive(true);
            showRaw = true;
            showRawTMP.text = "Hide\nRaw";
        }
    }
}
