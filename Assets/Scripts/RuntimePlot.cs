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
    [Range(0,1)]
    public float scaleFactor   = 0.1f;   

    private Vector3[] rawPts, rmsPts;
    private int idx;

    void Awake()
    {
        rawPts = new Vector3[maxPoints];
        rmsPts = new Vector3[maxPoints];
        rawLine.positionCount = rmsLine.positionCount = maxPoints;
        SetPlotScale(0.01f * scaleFactor);
    }
    
    public void SetPlotScale(float s)
    {

        s = Mathf.Max(0.001f, s);


        transform.localScale = Vector3.one * s;


        rawLine.widthMultiplier = s;
        rmsLine.widthMultiplier = s;
    }


    public void Push(float raw, float rms)
    {
        float yRaw = (raw + yOffset) * yScale;
        float yRms = (rms + yOffset) * yScale;

        rawPts[idx].y = yRaw;
        rmsPts[idx].y = yRms;

        for (int i = 0; i < maxPoints; i++)
        {
            int j = (idx - i + maxPoints) % maxPoints;
            float x = i;
            rawLine.SetPosition(maxPoints - 1 - i, new Vector3(x, rawPts[j].y, 0));
            rmsLine.SetPosition(maxPoints - 1 - i, new Vector3(x, rmsPts[j].y, 0));
        }
        idx = (idx + 1) % maxPoints;
    }
}