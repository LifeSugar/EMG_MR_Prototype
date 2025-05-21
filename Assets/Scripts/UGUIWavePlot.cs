using UnityEngine;
using UnityEngine.UI;

public class UGUIWavePlot : MonoBehaviour
{
    [Header("UI")]
    public RawImage targetImage;      

    [Header("Plot Settings")]
    public int width      = 512;      
    public int height     = 256;      
    public Color bgColor  = Color.black;
    public Color lineColor= Color.green;
    public float yScale   = 0.1f;     // ADC to pixel

    private Texture2D tex;
    private Color[]   bgLine;         
    private int       xPos = 0;       

    void Start()
    {

        tex = new Texture2D(width, height, TextureFormat.RGB24, false);
        tex.filterMode = FilterMode.Point;
        tex.wrapMode   = TextureWrapMode.Clamp;

        bgLine = new Color[height];
        for (int i = 0; i < height; i++) bgLine[i] = bgColor;
        for (int x = 0; x < width; x++) tex.SetPixels(x, 0, 1, height, bgLine);

        tex.Apply();
        targetImage.texture = tex;
    }


    public void PushSample(float value)
    {
        // 1. 清当前列
        tex.SetPixels(xPos, 0, 1, height, bgLine);

        // 2. 写一个像素
        int y = Mathf.Clamp(Mathf.RoundToInt((value * yScale) + height / 2), 0, height - 1);
        tex.SetPixel(xPos, y, lineColor);
        tex.Apply(false);

        // 3. 移动写入指针
        xPos = (xPos + 1) % width;

        // 4. 更新 RawImage 的 uvRect.x，使纹理滚动
        //  (0..1 对应 0..width)
        var r = targetImage.uvRect;
        r.x = (float)xPos / width;
        targetImage.uvRect = r;
    }
}
