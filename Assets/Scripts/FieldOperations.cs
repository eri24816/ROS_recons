using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public static class FieldOperations
{
    public static float GaussianKernel(float r2)
    {
        return Mathf.Exp(-r2 * 4);
    }
    public static float[,] Blur(float[,] m, int r)
    {
        float[,] b = new float[m.GetLength(0), m.GetLength(1)];
        int w = m.GetLength(0);
        int h = m.GetLength(1);
        for (int x = 0; x < w; x++)
        {
            for (int y = 0; y < h; y++)
            {
                float kernelSum = 0;
                float sum = 0;
                for (int dx = -r; dx <= r; dx++)
                {
                    for (int dy = -r; dy <= r; dy++)
                    {
                        if (x + dx < 0 || y + dy < 0 || x + dx >= w || y + dy >= h) continue;
                        float k = GaussianKernel(((float)dx * dx + dy * dy) / (r * r));
                        kernelSum += k;
                        sum += m[x + dx, y + dy] * k;
                    }
                }
                b[x, y] = sum / kernelSum;
            }
        }
        return b;
    }
    public static Vector2[,] Gradient(float[,] m)
    {
        Vector2[,] g = new Vector2[m.GetLength(0), m.GetLength(1)];
        int w = m.GetLength(0);
        int h = m.GetLength(1);
        for (int x = 0; x < w; x++)
        {
            for (int y = 0; y < h; y++)
            {

                g[x, y] = new Vector2(
                    -(x - 1 >= 0 ? m[x - 1, y] : 0) + (x + 1 < w ? m[x + 1, y] : 0),
                    -(y - 1 >= 0 ? m[x, y - 1] : 0) + (y + 1 < h ? m[x, y + 1] : 0)
                    );
            }
        }
        return g;
    }
    public static float[,] AddPadding(float[,] m, int pad)
    {
        float[,] r = new float[m.GetLength(0) + 2 * pad, m.GetLength(1) + 2 * pad];
        int w = m.GetLength(0);
        int h = m.GetLength(1);
        for (int x = 0; x < r.GetLength(0); x++)
        {
            for (int y = 0; y < r.GetLength(1); y++)
            {
                r[x, y] = 0;
            }
        }
        for (int x = 0; x < w; x++)
        {
            for (int y = 0; y < h; y++)
            {
                r[x + pad, y + pad] = m[x, y];
            }
        }
        return r;
    }
    public static float[,] DownScale(float[,] m, float scale)
    {
        int w = (int)(m.GetLength(0) / scale);
        int h = (int)(m.GetLength(1) / scale);
        float[,] r = new float[w, h];
        float[,] k = new float[w, h];
        for (int x = 0; x < m.GetLength(0); x++)
        {
            int x_ = (int)(x / scale);
            if (x_ >= w) continue;
            for (int y = 0; y < m.GetLength(0); y++)
            {
                int y_ = (int)(y / scale);
                if (y_ >= h) continue;

                r[x_, y_] += m[x, y];
                k[x_, y_] += 1;
            }
        }
        for (int x = 0; x < w; x++)
        {
            for (int y = 0; y < h; y++)
            {
                r[x, y] /= k[x, y];
            }
        }
        return r;
    }
}
