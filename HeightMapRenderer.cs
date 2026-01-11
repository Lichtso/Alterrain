using System;
using System.Collections.Generic;
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
    public float[] input;
    public (int, int, float)[] output;

    public HeightMapRenderer(Rectanglei rect)
    {
        frame = rect;
        input = new float[(frame.X2 - frame.X1) * (frame.Y2 - frame.Y1)];
        output = new (int, int, float)[(frame.X2 - frame.X1) * (frame.Y2 - frame.Y1)];
        for (int i = 0; i < input.Length; ++i)
        {
            input[i] = input.Length;
        }
    }

    public void Point(int x, int y, int z)
    {
        if (x < frame.X1 || x >= frame.X2 || z < frame.Y1 || z >= frame.Y2)
            return;
        input[(z - frame.Y1) * (frame.X2 - frame.X1) + (x - frame.X1)] = (float) y;
    }

    // http://members.chello.at/easyfilter/bresenham.c
    public void LineSegment3D(int x0, int y0, int z0, int x1, int y1, int z1)
    {
        if (x0 < frame.X1 || x0 >= frame.X2 || z0 < frame.Y1 || z0 >= frame.Y2 ||
            x1 < frame.X1 || x1 >= frame.X2 || z1 < frame.Y1 || z1 >= frame.Y2)
            return;
        x0 -= frame.X1;
        z0 -= frame.Y1;
        x1 -= frame.X1;
        z1 -= frame.Y1;
        int stride = frame.X2 - frame.X1;
        int dx = Math.Abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
        int dy = Math.Abs(y1 - y0), sy = y0 < y1 ? 1 : -1; 
        int dz = Math.Abs(z1 - z0), sz = z0 < z1 ? 1 : -1; 
        int dm = Math.Max(dx, Math.Max(dy, dz)), i = dm;
        x1 = y1 = z1 = dm / 2;
        for (;;)
        {
            input[z0 * stride + x0] = Math.Min(input[z0 * stride + x0], (float) y0);
            if (i-- == 0) break;
            x1 -= dx; if (x1 < 0) { x1 += dm; x0 += sx; } 
            y1 -= dy; if (y1 < 0) { y1 += dm; y0 += sy; } 
            z1 -= dz; if (z1 < 0) { z1 += dm; z0 += sz; } 
        }
    }

    // http://members.chello.at/easyfilter/bresenham.c
    private void QuadraticBezierSegment(int x0, int z0, int x1, int z1, int x2, int z2, int y)
    {
        int stride = frame.X2 - frame.X1;
        int sx = x2-x1, sz = z2-z1;
        long xx = x0-x1, zz = z0-z1, xz;
        double dx, dz, err, cur = xx*sz-zz*sx;
        if (xx*sx > 0 || zz*sz > 0) return;
        if (sx*(long)sx+sz*(long)sz > xx*xx+zz*zz)
        {
            x2 = x0; x0 = sx+x1; z2 = z0; z0 = sz+z1; cur = -cur;
        }  
        if (cur != 0)
        {
            xx += sx; xx *= sx = x0 < x2 ? 1 : -1;
            zz += sz; zz *= sz = z0 < z2 ? 1 : -1;
            xz = 2*xx*zz; xx *= xx; zz *= zz;
            if (cur*sx*sz < 0)
            {
                xx = -xx; zz = -zz; xz = -xz; cur = -cur;
            }
            dx = 4.0*sz*cur*(x1-x0)+xx-xz;
            dz = 4.0*sx*cur*(z0-z1)+zz-xz;
            xx += xx; zz += zz; err = dx+dz+xz;
            do
            {                              
                input[z0 * stride + x0] = Math.Min(input[z0 * stride + x0], (float) y);
                if (x0 == x2 && z0 == z2) return;
                bool cond = 2*err < dx;
                if (2*err > dz) { x0 += sx; dx -= xz; err += dz += zz; }
                if (cond) { z0 += sz; dz -= xz; err += dx += xx; }
            }
            while (dz < dx);
        }
        LineSegment3D(x0 + frame.X1, y, z0 + frame.Y1, x2 + frame.X1, y, z2 + frame.Y1);
    }

    // http://members.chello.at/easyfilter/bresenham.c
    public void QuadraticBezierCurve(int x0, int z0, int x1, int z1, int x2, int z2, int y)
    {
        if (x0 < frame.X1 || x0 >= frame.X2 || z0 < frame.Y1 || z0 >= frame.Y2 ||
            x1 < frame.X1 || x1 >= frame.X2 || z1 < frame.Y1 || z1 >= frame.Y2 ||
            x2 < frame.X1 || x2 >= frame.X2 || z2 < frame.Y1 || z2 >= frame.Y2)
            return;
        x0 -= frame.X1;
        z0 -= frame.Y1;
        x1 -= frame.X1;
        z1 -= frame.Y1;
        x2 -= frame.X1;
        z2 -= frame.Y1;
        int x = x0-x1, z = z0-z1;
        double t = x0-2*x1+x2, r;
        if ((long)x*(x2-x1) > 0)
        {
            if ((long)z*(z2-z1) > 0 && Math.Abs((z0-2*z1+z2)/t*x) > Math.Abs(z))
            {
                x0 = x2; x2 = x+x1; z0 = z2; z2 = z+z1;
            }
            t = (x0-x1)/t;
            r = (1-t)*((1-t)*z0+2.0*t*z1)+t*t*z2;
            t = (x0*x2-x1*x1)*t/(x0-x1);
            x = (int) (t+0.5); z = (int) (r+0.5);
            r = (z1-z0)*(t-x0)/(x1-x0)+z0;
            QuadraticBezierSegment(x0, z0, x, (int) (r+0.5), x, z, y);
            r = (z1-z2)*(t-x2)/(x1-x2)+z2;
            x0 = x1 = x; z0 = z; z1 = (int) (r+0.5);
        }
        if ((long)(z0-z1)*(z2-z1) > 0)
        {
            t = z0-2*z1+z2; t = (z0-z1)/t;
            r = (1-t)*((1-t)*x0+2.0*t*x1)+t*t*x2;
            t = (z0*z2-z1*z1)*t/(z0-z1);
            x = (int) (r+0.5); z = (int) (t+0.5);
            r = (x1-x0)*(t-z0)/(z1-z0)+x0;
            QuadraticBezierSegment(x0, z0, (int) (r+0.5), z, x, z, y);
            r = (x1-x2)*(t-z2)/(z1-z2)+x2;
            x0 = x; x1 = (int) (r+0.5); z0 = z1 = z;
        }
        QuadraticBezierSegment(x0, z0, x1, z1, x2, z2, y);
    }

    private void DistanceTransformPixel(int stride, int x, int y, int nx, int ny)
    {
        int pixelIndex = y * stride + x;
        (int diffX, int diffY, float currentValue) = output[pixelIndex];
        (diffX, diffY, _) = output[ny * stride + nx];
        diffX += nx - x;
        diffY += ny - y;
        float newValue = (float) Math.Sqrt(diffX * diffX + diffY * diffY);
        if (currentValue > newValue)
            output[pixelIndex] = (diffX, diffY, newValue);
    }

    // Cuisenaire, Olivier, and Benoit Macq. "Fast and exact signed Euclidean distance transformation with linear complexity."
    // 1999 IEEE International Conference on Acoustics, Speech, and Signal Processing.
    // https://infoscience.epfl.ch/server/api/core/bitstreams/3334e273-b447-47a2-981b-cb884ae7407d/content
    public void DistanceTransform()
    {
        int width = frame.X2 - frame.X1;
        int height = frame.Y2 - frame.Y1;
        for (int i = 0; i < width * height; ++i)
        {
            output[i] = (0, 0, input[i]);
        }
        for (int y = 0; y < height; ++y)
        {
            for (int x = 0; x < width; ++x)
            {
                if (x > 0)
                    DistanceTransformPixel(width, x, y, x - 1, y);
                if (y > 0)
                    DistanceTransformPixel(width, x, y, x, y - 1);
            }
            for (int x = width - 1; x >= 0; --x)
            {
                if (x < width - 1)
                    DistanceTransformPixel(width, x, y, x + 1, y);
            }
        }
        for (int y = height - 1; y >= 0; --y)
        {
            for (int x = width - 1; x >= 0; --x)
            {
                if (x < width - 1)
                    DistanceTransformPixel(width, x, y, x + 1, y);
                if (y < height - 1)
                    DistanceTransformPixel(width, x, y, x, y + 1);
            }
            for (int x = 0; x < width; ++x)
            {
                if (x > 0)
                    DistanceTransformPixel(width, x, y, x - 1, y);
            }
        }
    }
}

public class SlopeInterval
{
    public double length;
    public double slope;
}

public class SlopeProfile
{
    (double, double, double)[] intervals;

    public SlopeProfile(List<SlopeInterval> intervals)
    {
        double prevEnd = 0.0;
        double startHeight = -7.0;
        this.intervals = new (double, double, double)[intervals.Count];
        for (int i = 0; i < intervals.Count; ++i)
        {
            this.intervals[i] = (prevEnd + intervals[i].length, startHeight - prevEnd * intervals[i].slope, intervals[i].slope);
            prevEnd += intervals[i].length;
            startHeight += intervals[i].length * intervals[i].slope;
        }
    }

    public double distanceToHeight(double distance)
    {
        int low = 0;
        int size = intervals.Length;
        double boundary;
        while (size > 1) {
            int half = size / 2;
            int mid = low + half;
            (boundary, _, _) = intervals[mid];
            low = (boundary > distance) ? low : mid;
            size -= half;
        }
        (boundary, _, _) = intervals[low];
        if (boundary < distance)
            ++low;
        (_, double offset, double factor) = intervals[low];
        return distance * factor + offset;
    }
}
