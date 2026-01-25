using System;
using Vintagestory.API.MathTools;
using System.Runtime.CompilerServices;

public struct BarycentricTriangle
{
    public FastVec2i vertexA, vertexB, vertexC;
    public double a, b, c, max;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public FastVec2i ClosestVertex()
    {
        if(a == max)
            return vertexA;
        else if(b == max)
            return vertexB;
        else
            return vertexC;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public double Interpolate(double valA, double valB, double valC)
    {
        return a * valA + b * valB + c * valC;
    }
}

public class HexGrid
{
    public int cellHeight;
    public int cellWidth;
    public FastVec2i[] neighborHexOffsets;
    public FastVec2i[] neighborTriOffsets;

    public HexGrid(int cellHeight)
    {
        this.cellHeight = cellHeight;
        cellWidth = (int) (Math.Sqrt(3.0) * 0.5 * cellHeight);
        //        -2,-2 -1,-2 +0,-2
        //     -2,-1 -1,-1 +0,-1 +1,-1
        //  -2,+0 -1,+0 +0,+0 +1,+0 +2,+0
        //     -1,+1 +0,+1 +1,+1 +2,+1
        //        +0,+2 +1,+2 +2,+2
        neighborHexOffsets = new FastVec2i[19]
        {
            new FastVec2i(0, 0),
            new FastVec2i(0, -1),
            new FastVec2i(1, 0),
            new FastVec2i(1, 1),
            new FastVec2i(0, 1),
            new FastVec2i(-1, 0),
            new FastVec2i(-1, -1),
            new FastVec2i(0, -2),
            new FastVec2i(1, -1),
            new FastVec2i(2, 0),
            new FastVec2i(2, 1),
            new FastVec2i(2, 2),
            new FastVec2i(1, 2),
            new FastVec2i(0, 2),
            new FastVec2i(-1, 1),
            new FastVec2i(-2, 0),
            new FastVec2i(-2, -1),
            new FastVec2i(-2, -2),
            new FastVec2i(-1, -2),
        };
        neighborTriOffsets = new FastVec2i[7];
        for (int i = 0; i < 6; ++i)
        {
            FastVec2i a = HexToCartesian(neighborHexOffsets[1 + i]);
            FastVec2i b = HexToCartesian(neighborHexOffsets[1 + (i + 1) % 6]);
            neighborTriOffsets[i] = a + b;
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public FastVec2i HexToCartesian(FastVec2i hex)
    {
        return new FastVec2i(
            hex.X * cellWidth,
            hex.Y * cellHeight - cellHeight * hex.X / 2
        );
    }

    // https://www.redblobgames.com/grids/hexagons/more-pixel-to-hex.html#chris-cox
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public FastVec2i CartesianToHex(FastVec2i cartesian)
    {
        double y = (double) cartesian.Y / (double) cellHeight;
        double x = (double) cartesian.X / (double) cellWidth * 1.5 + 1.0;
        double aux = Math.Floor(x + y);
        int q = (int) Math.Floor((aux + x - y) / 3.0);
        int r = (int) Math.Floor((aux + 2.0 * y + 1.0) / 3.0);
        return new FastVec2i(q, r);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public FastVec2i HexToCartesianWithJitter(LCGRandom rng, FastVec2i hex)
    {
        rng.InitPositionSeed(hex.X, hex.Y);
        int triangle = rng.NextInt(6);
        FastVec2i a = neighborTriOffsets[triangle];
        FastVec2i b = neighborTriOffsets[(triangle + 1) % 6];
        const float jitter = 0.7F;
        float u = rng.NextFloat() * jitter;
        float v = rng.NextFloat() * jitter;
        float sum = u + v - jitter;
        if (sum > 0.0F)
        {
            u -= sum;
            v -= sum;
        }
        FastVec2i cartesian = HexToCartesian(hex);
        cartesian.X += (int) ((u * a.X + v * b.X) / 3.0F);
        cartesian.Y += (int) ((u * a.Y + v * b.Y) / 3.0F);
        return cartesian;
    }

    public (double, FastVec2i) VoronoiClosest(LCGRandom rng, FastVec2i centralCartesian)
    {
        double squaredDistance = double.PositiveInfinity;
        FastVec2i closestHex = new FastVec2i(0, 0);
        FastVec2i centralHex = CartesianToHex(centralCartesian);
        for (int i = 0; i < 7; ++i)
        {
            FastVec2i hex = centralHex + neighborHexOffsets[i];
            FastVec2i cartesian = HexToCartesianWithJitter(rng, hex);
            double diffX = cartesian.X - centralCartesian.X;
            double diffZ = cartesian.Y - centralCartesian.Y;
            double dist = diffX * diffX + diffZ * diffZ;
            if (squaredDistance > dist)
            {
                squaredDistance = dist;
                closestHex = hex;
            }
        }
        return (squaredDistance, closestHex);
    }

    public BarycentricTriangle BarycentricTriangle(LCGRandom rng, FastVec2i point)
    {
        BarycentricTriangle result;
        result.vertexA = CartesianToHex(point);
        FastVec2i cartesianA = HexToCartesianWithJitter(rng, result.vertexA);
        point -= cartesianA;
        result.vertexB = result.vertexA + neighborHexOffsets[6];
        result.vertexC = new FastVec2i(0, 0);
        FastVec2i cartesianB = HexToCartesianWithJitter(rng, result.vertexB) - cartesianA, cartesianC = new FastVec2i(0, 0);
        int areaB = cartesianB.Y * point.X - cartesianB.X * point.Y, areaC = 0;
        for (int i = 1; i < 7; ++i)
        {
            result.vertexC = result.vertexA + neighborHexOffsets[i];
            cartesianC = HexToCartesianWithJitter(rng, result.vertexC) - cartesianA;
            areaC = cartesianC.Y * point.X - cartesianC.X * point.Y;
            if (areaB <= 0 && areaC >= 0)
            {
                break;
            }
            result.vertexB = result.vertexC;
            cartesianB = cartesianC;
            areaB = areaC;
        }
        double factor = 1.0 / (double) (cartesianC.Y * cartesianB.X - cartesianC.X * cartesianB.Y);
        result.b = factor * areaC;
        result.c = factor * -areaB;
        result.a = 1.0 - result.b - result.c;
        result.max = Math.Max(result.a, Math.Max(result.b, result.c));
        return result;
    }
}
