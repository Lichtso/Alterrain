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

    // http://members.chello.at/easyfilter/bresenham.c
    private static void PlotSegment(int x0, int z0, int x1, int z1, int x2, int z2, int y, PlotDelegate3D PlotPoint)
    {
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
                PlotPoint(x0, y, z0);
                if (x0 == x2 && z0 == z2) return;
                bool cond = 2*err < dx;
                if (2*err > dz) { x0 += sx; dx -= xz; err += dz += zz; }
                if (cond) { z0 += sz; dz -= xz; err += dx += xx; }
            }
            while (dz < dx);
        }
        GameMath.BresenHamPlotLine3d(x0, y, z0, x2, y, z2, PlotPoint);
    }

    // http://members.chello.at/easyfilter/bresenham.c
    public void Plot(Rectanglei frame, PlotDelegate3D PlotPoint)
    {
        if (a.X < frame.X1 || a.X >= frame.X2 || a.Y < frame.Y1 || a.Y >= frame.Y2 ||
            b.X < frame.X1 || b.X >= frame.X2 || b.Y < frame.Y1 || b.Y >= frame.Y2 ||
            c.X < frame.X1 || c.X >= frame.X2 || c.Y < frame.Y1 || c.Y >= frame.Y2)
            return;
        int x0 = a.X - frame.X1;
        int z0 = a.Y - frame.Y1;
        int x1 = b.X - frame.X1;
        int z1 = b.Y - frame.Y1;
        int x2 = c.X - frame.X1;
        int z2 = c.Y - frame.Y1;
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
            PlotSegment(x0, z0, x, (int) (r+0.5), x, z, height, PlotPoint);
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
            PlotSegment(x0, z0, (int) (r+0.5), z, x, z, height, PlotPoint);
            r = (x1-x2)*(t-z2)/(z1-z2)+x2;
            x0 = x; x1 = (int) (r+0.5); z0 = z1 = z;
        }
        PlotSegment(x0, z0, x1, z1, x2, z2, height, PlotPoint);
    }
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

    public void PlotPoint(int x, int y, int z)
    {
        int index = z * (frame.X2 - frame.X1) + x;
        input[index] = Math.Min(input[index], (float) y);
    }

    private void DistanceTransformPixel(int stride, int x, int y, int nx, int ny)
    {
        int pixelIndex = y * stride + x;
        (int diffX, int diffY, float currentValue) = output[pixelIndex];
        (diffX, diffY, _) = output[ny * stride + nx];
        float height = input[(ny + diffY) * stride + (nx + diffX)];
        diffX += nx - x;
        diffY += ny - y;
        float newValue = (float) Math.Sqrt(diffX * diffX + diffY * diffY) + height;
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
