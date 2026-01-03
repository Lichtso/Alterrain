using System;
using Vintagestory.API.MathTools;

namespace Alterrain;

public struct QuadraticBezierCurve
{
    public FastVec2i a;
    public FastVec2i b;
    public FastVec2i c;
    public int height;
}

public class HeightMapRenderer
{
    public Rectanglei frame;
    public float[] squaredDistanceMap;

    public HeightMapRenderer(Rectanglei rect)
    {
        frame = rect;
        squaredDistanceMap = new float[(frame.X2 - frame.X1) * (frame.Y2 - frame.Y1)];
        for (int i = 0; i < squaredDistanceMap.Length; ++i)
        {
            squaredDistanceMap[i] = squaredDistanceMap.Length;
        }
    }

    public void Point(int x, int y, int z)
    {
        if (x < frame.X1 || x >= frame.X2 || z < frame.Y1 || z >= frame.Y2)
        {
            return;
        }
        squaredDistanceMap[(z - frame.X1) * (frame.X2 - frame.X1) + (x - frame.X1)] = (float) y * (float) y;
    }

    // https://www.geeksforgeeks.org/python/bresenhams-algorithm-for-3-d-line-drawing/
    public void Bresenham3D(int x1, int y1, int z1, int x2, int y2, int z2)
    {
        if (x1 < frame.X1 || x1 >= frame.X2 || z1 < frame.Y1 || z1 >= frame.Y2 || x2 < frame.X1 || x2 >= frame.X2 || z2 < frame.Y1 || z2 >= frame.Y2)
        {
            return;
        }
        x1 -= frame.X1;
        z1 -= frame.Y1;
        x2 -= frame.X1;
        z2 -= frame.Y1;
        int stride = frame.X2 - frame.X1;
        squaredDistanceMap[z1 * stride + x1] = (float) y1 * (float) y1;
        int dx = Math.Abs(x2 - x1);
        int dy = Math.Abs(y2 - y1);
        int dz = Math.Abs(z2 - z1);
        int xs;
        int ys;
        int zs;
        if (x2 > x1)
            xs = 1;
        else
            xs = -1;
        if (y2 > y1)
            ys = 1;
        else
            ys = -1;
        if (z2 > z1)
            zs = 1;
        else
            zs = -1;

        // Driving axis is X-axis
        if (dx >= dy && dx >= dz)
        {
            int p1 = 2 * dy - dx;
            int p2 = 2 * dz - dx;
            while (x1 != x2)
            {
                x1 += xs;
                if (p1 >= 0)
                {
                    y1 += ys;
                    p1 -= 2 * dx;
                }
                if (p2 >= 0)
                {
                    z1 += zs;
                    p2 -= 2 * dx;
                }
                p1 += 2 * dy;
                p2 += 2 * dz;
                squaredDistanceMap[z1 * stride + x1] = (float) y1 * (float) y1;
            }

            // Driving axis is Y-axis
        }
        else if (dy >= dx && dy >= dz)
        {
            int p1 = 2 * dx - dy;
            int p2 = 2 * dz - dy;
            while (y1 != y2)
            {
                y1 += ys;
                if (p1 >= 0)
                {
                    x1 += xs;
                    p1 -= 2 * dy;
                }
                if (p2 >= 0)
                {
                    z1 += zs;
                    p2 -= 2 * dy;
                }
                p1 += 2 * dx;
                p2 += 2 * dz;
                squaredDistanceMap[z1 * stride + x1] = (float) y1 * (float) y1;
            }

            // Driving axis is Z-axis
        }
        else
        {
            int p1 = 2 * dy - dz;
            int p2 = 2 * dx - dz;
            while (z1 != z2)
            {
                z1 += zs;
                if (p1 >= 0)
                {
                    y1 += ys;
                    p1 -= 2 * dz;
                }
                if (p2 >= 0)
                {
                    x1 += xs;
                    p2 -= 2 * dz;
                }
                p1 += 2 * dy;
                p2 += 2 * dx;
                squaredDistanceMap[z1 * stride + x1] = (float) y1 * (float) y1;
            }
        }
    }

    // https://cs.brown.edu/people/pfelzens/papers/dt-final.pdf (Distance Transforms of Sampled Functions, Pedro F. Felzenszwalb, Daniel P. Huttenlocher)
    private void DistanceTransformRowOrColumn(uint offset, uint stride, uint n)
    {
        uint[] v = new uint[n];
        float[] z = new float[n + 1];
        float[] aux = new float[n];
        for (uint q = 0; q < n; ++q)
        {
            aux[q] = squaredDistanceMap[offset + stride * q];
        }
        uint max = n * n * n;
        v[0] = 0;
        z[0] = -max;
        z[1] = max;
        float s;
        uint k = 0;
        for (uint q = 1; q < n; ++q)
        {
            do
            {
                uint vk = v[k];
                s = ((aux[q] + q * q) - (aux[vk] + vk * vk)) / (2 * (q - vk));
            } while(s <= z[k--]);
            k += 2;
            v[k] = q;
            z[k] = s;
            z[k + 1] = max;
        }
        k = 0;
        for (uint q = 0; q < n; ++q)
        {
            while (z[k + 1] < q)
            {
                ++k;
            }
            uint vk = v[k];
            squaredDistanceMap[offset + stride * q] = (q - vk) * (q - vk) + aux[vk];
        }
    }

    public void DistanceTransform()
    {
        uint width = (uint) (frame.X2 - frame.X1);
        uint height = (uint) (frame.Y2 - frame.Y1);
        for (uint y = 0; y < height; ++y)
        {
            DistanceTransformRowOrColumn(y * width, 1, width);
        }
        for (uint x = 0; x < width; ++x)
        {
            DistanceTransformRowOrColumn(x, width, height);
        }
    }
}
