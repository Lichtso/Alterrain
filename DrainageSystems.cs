using System;
using System.Collections.Generic;
using Vintagestory.API.Datastructures;
using Vintagestory.API.MathTools;
using Vintagestory.API.Server;
using Vintagestory.API.Util;

namespace Alterrain;

public struct RiverNode
{
    public FastVec2i cartesian;
    public float proximity;
    public float flow;
    public FastVec2i downstreamCoord;

    public RiverNode(HexGrid riverGrid, LCGRandom rng, Basin basin, FastVec2i nodeCoord)
    {
        cartesian = riverGrid.HexToCartesianWithJitter(rng, nodeCoord);
        BarycentricTriangle triangle = basin.basinGrid.BarycentricTriangle(rng, cartesian);
        proximity = (triangle.ClosestVertex() == basin.coord) ? (float) triangle.max : float.PositiveInfinity;
        flow = 1.0F;
        downstreamCoord.X = -1;
        downstreamCoord.Y = -1;
    }
}

public class Basin
{
    public LCGRandom rng;
    public HexGrid basinGrid;
    public FastVec2i coord;

    public Basin(LCGRandom rng, HexGrid basinGrid, FastVec2i basinCoord)
    {
        this.rng = rng;
        this.basinGrid = basinGrid;
        this.coord = basinCoord;
    }

    public Dictionary<FastVec2i, int> GenerateClimate(ICoreServerAPI api)
    {
        ITreeAttribute worldConfig = api.WorldManager.SaveGame.WorldConfiguration;
        float geologicActivityInv = 1.0F / worldConfig.GetString("geologicActivity").ToFloat(0.05F);
        float temperatureModifier = worldConfig.GetString("globalTemperature", "1").ToFloat(1);
        float rainModifier = worldConfig.GetString("globalPrecipitation", "1").ToFloat(1);
        int halfRange = worldConfig.GetString("polarEquatorDistance", "50000").ToInt(50000);
        string climate = worldConfig.GetString("worldClimate", "realistic");
        string startingClimate = worldConfig.GetString("startingClimate");
        int spawnTemperature = 127; // Climate.DescaleTemperature((6 + 14) / 2);
        switch (startingClimate)
        {
            case "hot":
                spawnTemperature = 212; // Climate.DescaleTemperature((28 + 32) / 2);
                break;
            case "warm":
                spawnTemperature = 174; // Climate.DescaleTemperature((19 + 23) / 2);
                break;
            case "cool":
                spawnTemperature = 76; // Climate.DescaleTemperature((-5 + 1) / 2);
                break;
            case "icy":
                spawnTemperature = 34; // Climate.DescaleTemperature((-15 -10) / 2);
                break;
        }
        int temperatureOffsetZ = (int) (spawnTemperature / 255.0 * halfRange) + halfRange - api.WorldManager.MapSizeZ / 2;
        int geologicActivity = 0, temperature = 0, rain = 0;
        Dictionary<FastVec2i, int> climateAtBasinCoord = new Dictionary<FastVec2i, int>();
        for (int i = 0; i < 19; ++i)
        {
            FastVec2i basinCoord = coord + basinGrid.neighborHexOffsets[i];
            FastVec2i basinCenter = basinGrid.HexToCartesianWithJitter(rng, basinCoord);
            switch (climate)
            {
                case "realistic":
                    geologicActivity = (int) Math.Max(0, Math.Pow(rng.NextInt(256) / 255.0F, geologicActivityInv) * 255);
                    temperature = (int) ((double) Math.Abs((basinCenter.Y + temperatureOffsetZ) % (2 * halfRange) - halfRange) / (double) halfRange * 255.0);
                    temperature += rng.NextInt(40) - 20;
                    rain = rng.NextInt(256);
                    break;
                default:
                    geologicActivity = Math.Max(0, rng.NextInt(256) - 128) * 2;
                    temperature = 165;
                    temperature = 100 + (rng.NextInt(temperature) + rng.NextInt(temperature)) / 2;
                    rain = 60 + temperature;
                    rain = (rng.NextInt(rain) + rng.NextInt(rain) + rng.NextInt(rain)) / 3;
                    break;
            }
            temperature = (int) Math.Min(255, temperature * temperatureModifier);
            rain = (int) Math.Min(255, rain * rainModifier);
            climateAtBasinCoord.Add(basinCoord, (temperature << 16) + (rain << 8) + (geologicActivity));
        }
        return climateAtBasinCoord;
    }

    public List<QuadraticBezierCurve> GenerateDrainageSystem(HexGrid riverGrid)
    {
        FastVec2i basinCenter = basinGrid.HexToCartesianWithJitter(rng, coord);
        FastVec2i rootNodeCoord = riverGrid.CartesianToHex(new FastVec2i(basinCenter.X, basinCenter.Y));
        RiverNode rootNode = new RiverNode(riverGrid, rng, this, rootNodeCoord);
        IDictionary<FastVec2i, RiverNode> nodes = new Dictionary<FastVec2i, RiverNode>();
        nodes.Add(rootNodeCoord, rootNode);
        List<FastVec2i> topologicallySorted = new List<FastVec2i>();
        List<FastVec2i> nodeCoordPool = new List<FastVec2i>();
        rng.InitPositionSeed(coord.X, coord.Y);
        for (int j = 1; j < 7; ++j)
        {
            FastVec2i upstreamNodeCoord = rootNodeCoord + riverGrid.neighborHexOffsets[j];
            RiverNode upstreamNode = new RiverNode(riverGrid, rng, this, upstreamNodeCoord);
            upstreamNode.downstreamCoord = rootNodeCoord;
            nodes[upstreamNodeCoord] = upstreamNode;
            nodeCoordPool.Add(upstreamNodeCoord);
        }
        while (nodeCoordPool.Count > 0)
        {
            int i = rng.NextInt(nodeCoordPool.Count);
            FastVec2i nodeCoord = nodeCoordPool[i];
            nodeCoordPool[i] = nodeCoordPool[nodeCoordPool.Count - 1];
            nodeCoordPool.RemoveAt(nodeCoordPool.Count - 1);
            RiverNode node = nodes[nodeCoord];
            int offset = rng.NextInt(6);
            int nodeCoordPoolCount = nodeCoordPool.Count;
            for (int j = 0; j < 6; ++j)
            {
                FastVec2i upstreamNodeCoord = nodeCoord + riverGrid.neighborHexOffsets[(j + offset) % 6 + 1];
                RiverNode upstreamNode;
                if (!nodes.TryGetValue(upstreamNodeCoord, out upstreamNode))
                {
                    upstreamNode = new RiverNode(riverGrid, rng, this, upstreamNodeCoord);
                    nodes[upstreamNodeCoord] = upstreamNode;
                }
                if (upstreamNode.proximity <= node.proximity &&
                    upstreamNode.downstreamCoord.X == -1 &&
                    upstreamNode.downstreamCoord.Y == -1)
                {
                    upstreamNode.downstreamCoord = nodeCoord;
                    nodes[upstreamNodeCoord] = upstreamNode;
                    nodeCoordPool.Add(upstreamNodeCoord);
                }
            }
            if (nodeCoordPool.Count > nodeCoordPoolCount)
                topologicallySorted.Add(nodeCoord);
        }
        List<QuadraticBezierCurve> drainageSystem = new List<QuadraticBezierCurve>();
        for (int i = topologicallySorted.Count - 1; i >= 0; --i)
        {
            FastVec2i nodeCoord = topologicallySorted[i];
            RiverNode node = nodes[nodeCoord];
            RiverNode downstreamNode = nodes[node.downstreamCoord];
            QuadraticBezierCurve segment = new QuadraticBezierCurve();
            segment.b = node.cartesian;
            FastVec2i endPoint = (downstreamNode.cartesian + node.cartesian) / 2;
            FastVec2i normal = new FastVec2i(downstreamNode.cartesian.Y - node.cartesian.Y, node.cartesian.X - downstreamNode.cartesian.X);
            int offset = 0;
            for (int j = 0; j < 6; ++j)
            {
                FastVec2i upstreamNodeCoord = nodeCoord + riverGrid.neighborHexOffsets[j + 1];
                if (upstreamNodeCoord == node.downstreamCoord)
                    offset = j;
                RiverNode upstreamNode = nodes[upstreamNodeCoord];
                if (upstreamNode.downstreamCoord == nodeCoord)
                    node.flow += upstreamNode.flow;
            }
            nodes[nodeCoord] = node;
            double riverDepth = Math.Ceiling(Math.Sqrt(node.flow));
            double normalFactor = riverDepth / (1.4 * node.flow * Math.Sqrt(normal.X * normal.X + normal.Y * normal.Y));
            double riverOffset = -node.flow;
            for (int j = 0; j < 6; ++j)
            {
                FastVec2i upstreamNodeCoord = nodeCoord + riverGrid.neighborHexOffsets[(j + offset) % 6 + 1];
                RiverNode upstreamNode = nodes[upstreamNodeCoord];
                if (upstreamNode.downstreamCoord != nodeCoord)
                    continue;
                segment.a = upstreamNode.cartesian;
                if (upstreamNode.flow > 1.0F)
                    segment.a = (segment.a + segment.b) / 2;
                segment.c = endPoint + (riverOffset + upstreamNode.flow) * normalFactor * normal;
                segment.height = (int) Math.Ceiling(Math.Sqrt(upstreamNode.flow));
                segment.UpdateBounds();
                drainageSystem.Add(segment);
                riverOffset += 2.0 * upstreamNode.flow;
            }
        }
        return drainageSystem;
    }
}
