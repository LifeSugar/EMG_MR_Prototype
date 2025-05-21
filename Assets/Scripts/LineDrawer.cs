using UnityEngine;
using UnityEngine.UI.Extensions;

public class LineDrawer : MonoBehaviour
{
    [Header("UI Line")]
    public UILineRenderer line;          // 拖入 UILineRenderer

    [Header("Plot Settings")]
    public int   maxPoints = 500;
    public float yOffset   = -1500f;
    public float yScale    = 0.1f;

    // 环形缓冲仅存 y 值（x 固定）
    private float[] ringBuf;
    private Vector2[] drawPts;
    private int cursor;                  // 指向“下一次写入”的索引

    void Start()
    {
        ringBuf  = new float[maxPoints];
        drawPts  = new Vector2[maxPoints];

        // 预填 x 坐标
        for (int i = 0; i < maxPoints; i++)
            drawPts[i] = new Vector2(i, 0);

        line.Points = drawPts;
        line.SetVerticesDirty();
    }

    /// <summary>推入一个新采样（最新点将显示在最左端）</summary>
    public void PushSample(float adcValue)
    {
        // 1. 写入环缓存
        ringBuf[cursor] = (adcValue + yOffset) * yScale;
        cursor = (cursor + 1) % maxPoints;

        // 2. 按“时间先进先右”的顺序映射到 drawPts
        //    最新点 = 左端 0，下一个 = 1 ...
        for (int i = 0; i < maxPoints; i++)
        {
            // ringIndex 从 cursor-1 开始逆序取
            int ringIndex = (cursor - 1 - i + maxPoints) % maxPoints;
            drawPts[i].y = ringBuf[ringIndex];
        }

        // 3. 通知 UILineRenderer 重建网格
        line.Points = drawPts;
        line.SetVerticesDirty();
    }
}