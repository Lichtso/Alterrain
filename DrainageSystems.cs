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
    public float squaredDistance;
    public float flow;
    public FastVec2i downstreamCoord;

    public RiverNode(HexGrid riverGrid, LCGRandom rng, Basin basin, FastVec2i nodeCoord)
    {
        cartesian = riverGrid.HexToCartesianWithJitter(rng, nodeCoord);
        (double squaredDistance, FastVec2i closestBasinCoord) = basin.basinGrid.VoronoiClosest(rng, cartesian);
        flow = (closestBasinCoord == basin.coord) ? 1.0F : 0.0F;
        this.squaredDistance = (float) squaredDistance;
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

    public List<QuadraticBezierCurve> GenerateDrainageSystem(HexGrid riverGrid, float mountainStreamStartHeight)
    {
        FastVec2i basinCenter = basinGrid.HexToCartesianWithJitter(rng, coord);
        List<FastVec2i> topologicallySorted = new List<FastVec2i>();
        FastVec2i rootNodeCoord = riverGrid.CartesianToHex(new FastVec2i(basinCenter.X, basinCenter.Y));
        RiverNode rootNode = new RiverNode(riverGrid, rng, this, rootNodeCoord);
        IDictionary<FastVec2i, RiverNode> nodes = new Dictionary<FastVec2i, RiverNode>();
        nodes.Add(rootNodeCoord, rootNode);
        topologicallySorted.Add(rootNodeCoord);
        for (int i = 0; i < topologicallySorted.Count; ++i)
        {
            FastVec2i nodeCoord = topologicallySorted[i];
            RiverNode node = nodes[nodeCoord];
            for (int j = 1; j < 7; ++j)
            {
                FastVec2i neighborNodeCoord = nodeCoord + riverGrid.neighborHexOffsets[j];
                RiverNode neighborNode;
                if (!nodes.TryGetValue(neighborNodeCoord, out neighborNode))
                {
                    neighborNode = new RiverNode(riverGrid, rng, this, neighborNodeCoord);
                    if (neighborNode.flow > 0.0)
                    {
                        nodes.Add(neighborNodeCoord, neighborNode);
                        topologicallySorted.Add(neighborNodeCoord);
                    }
                }
            }
        }
        rng.InitPositionSeed(coord.X, coord.Y);
        for (int i = topologicallySorted.Count - 1; i >= 0; --i)
        {
            FastVec2i nodeCoord = topologicallySorted[i];
            RiverNode node = nodes[nodeCoord];
            List<FastVec2i> candidateNodeCoords = new List<FastVec2i>();
            for (int j = 1; j < 7; ++j)
            {
                FastVec2i neighborNodeCoord = nodeCoord + riverGrid.neighborHexOffsets[j];
                RiverNode neighborNode;
                if (nodes.TryGetValue(neighborNodeCoord, out neighborNode) &&
                    neighborNode.downstreamCoord.X == -1 &&
                    neighborNode.downstreamCoord.Y == -1 &&
                    neighborNode.squaredDistance < node.squaredDistance)
                {
                    candidateNodeCoords.Add(neighborNodeCoord);
                }
            }
            if (candidateNodeCoords.Count == 0)
            {
                continue;
            }
            node.downstreamCoord = candidateNodeCoords[rng.NextInt(candidateNodeCoords.Count)];
            RiverNode downstreamNode = nodes[node.downstreamCoord];
            downstreamNode.flow += node.flow;
            nodes[node.downstreamCoord] = downstreamNode;
            nodes[nodeCoord] = node;
        }
        List<QuadraticBezierCurve> drainageSystem = new List<QuadraticBezierCurve>();
        for (int i = 0; i < topologicallySorted.Count; ++i)
        {
            FastVec2i nodeCoord = topologicallySorted[i];
            RiverNode upstreamNode = nodes[nodeCoord];
            if (upstreamNode.downstreamCoord.X == -1 && upstreamNode.downstreamCoord.Y == -1)
            {
                continue;
            }
            RiverNode node = nodes[upstreamNode.downstreamCoord];
            if (node.downstreamCoord.X == -1 && node.downstreamCoord.Y == -1)
            {
                continue;
            }
            RiverNode downstreamNode = nodes[node.downstreamCoord];
            QuadraticBezierCurve segment = new QuadraticBezierCurve();
            segment.a = new FastVec2i(upstreamNode.cartesian.X, upstreamNode.cartesian.Y);
            segment.b = new FastVec2i(node.cartesian.X, node.cartesian.Y);
            segment.c = new FastVec2i(downstreamNode.cartesian.X, downstreamNode.cartesian.Y);
            if (upstreamNode.flow > 1.0F)
                segment.a = (segment.a + segment.b) / 2;
            segment.c = (segment.c + segment.b) / 2;
            float x = 1.0F - Math.Min(1.0F, upstreamNode.flow / 50.0F);
            segment.height = (int) (x * x * x * mountainStreamStartHeight);
            segment.UpdateBounds();
            drainageSystem.Add(segment);
        }
        return drainageSystem;
    }
}
