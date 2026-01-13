using System;
using System.Collections.Generic;
using Vintagestory.API.Datastructures;
using Vintagestory.API.MathTools;
using Vintagestory.API.Server;
using Vintagestory.API.Util;

namespace Alterrain;

public struct RiverNode
{
    public FastVec2i position;
    public float squaredDistance;
    public float flow;
    public FastVec2i downstreamCoord;

    static readonly int cellHeight = 64;
    static readonly int cellWidth = (int) (Math.Sqrt(3.0) * 0.5 * cellHeight);

    public static FastVec2i CoordsAt(int positionX, int positionZ)
    {
        int x = positionX / cellWidth;
        return new FastVec2i(x, (positionZ + cellHeight * x / 2) / cellHeight);
    }

    public RiverNode(LCGRandom rng, FastVec2i basinCoord, Basin basin, FastVec2i nodeCoord)
    {
        rng.InitPositionSeed(nodeCoord.X, nodeCoord.Y);
        position = new FastVec2i(
            nodeCoord.X * cellWidth,
            nodeCoord.Y * cellHeight - cellHeight * nodeCoord.X / 2
        );
        position.X += rng.NextInt(cellWidth / 2) - cellWidth / 4;
        position.Y += rng.NextInt(cellHeight / 2) - cellHeight / 4;
        int basinCellX = GameMath.Clamp(position.X / Basin.cellSpacing - basinCoord.X + 2, 1, 3);
        int basinCellZ = GameMath.Clamp(position.Y / Basin.cellSpacing - basinCoord.Y + 2, 1, 3);
        (double squaredDistance, int closestBasinIndex) = basin.FindClosestBasin(basinCellX, basinCellZ, position);
        flow = (closestBasinIndex == 12) ? 1.0F : 0.0F;
        this.squaredDistance = (float) squaredDistance;
        downstreamCoord.X = -1;
        downstreamCoord.Y = -1;
    }
}

public class Basin
{
    public FastVec2i[] neighborCenter;
    public int[] neighborClimate;

    public const int cellSpacing = 2048;

    public Basin(LCGRandom rng, FastVec2i basinCoord)
    {
        neighborCenter = new FastVec2i[25];
        for (int z = 0; z < 5; ++z)
        {
            for (int x = 0; x < 5; ++x)
            {
                rng.InitPositionSeed(basinCoord.X + x - 2, basinCoord.Y + z - 2);
                neighborCenter[z * 5 + x] = new FastVec2i(
                    (basinCoord.X + x - 2) * cellSpacing + rng.NextInt(cellSpacing),
                    (basinCoord.Y + z - 2) * cellSpacing + rng.NextInt(cellSpacing)
                );
            }
        }
    }

    public void InitClimate(ICoreServerAPI api, LCGRandom rng, FastVec2i basinCoord)
    {
        neighborClimate = new int[25];
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
        for (int z = 0; z < 5; ++z)
        {
            for (int x = 0; x < 5; ++x)
            {
                rng.InitPositionSeed(basinCoord.X + x - 2, basinCoord.Y + z - 2);
                FastVec2i center = new FastVec2i(
                    (basinCoord.X + x - 2) * cellSpacing + rng.NextInt(cellSpacing),
                    (basinCoord.Y + z - 2) * cellSpacing + rng.NextInt(cellSpacing)
                );
                switch (climate)
                {
                    case "realistic":
                        geologicActivity = (int) Math.Max(0, Math.Pow(rng.NextInt(256) / 255.0F, geologicActivityInv) * 255);
                        temperature = (int) ((double) Math.Abs((center.Y + temperatureOffsetZ) % (2 * halfRange) - halfRange) / (double) halfRange * 255.0);
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
                neighborClimate[z * 5 + x] = (temperature << 16) + (rain << 8) + (geologicActivity);
            }
        }
    }

    public (double, int) FindClosestBasin(int basinCellX, int basinCellZ, FastVec2i position)
    {
        double squaredDistance = double.PositiveInfinity;
        int closestBasinIndex = 25;
        for (int z = basinCellZ - 1; z <= basinCellZ + 1; ++z)
        {
            for (int x = basinCellX - 1; x <= basinCellX + 1; ++x)
            {
                int i = z * 5 + x;
                double diffX = neighborCenter[i].X - position.X;
                double diffZ = neighborCenter[i].Y - position.Y;
                double dist = diffX * diffX + diffZ * diffZ;
                if (squaredDistance > dist)
                {
                    squaredDistance = dist;
                    closestBasinIndex = i;
                }
            }
        }
        return (squaredDistance, closestBasinIndex);
    }

    public List<QuadraticBezierCurve> GenerateDrainageSystem(LCGRandom rng, FastVec2i basinCoord, float mountainStreamStartHeight)
    {
        List<FastVec2i> topologicallySorted = new List<FastVec2i>();
        FastVec2i rootNodeCoord = RiverNode.CoordsAt(neighborCenter[12].X, neighborCenter[12].Y);
        RiverNode rootNode = new RiverNode(rng, basinCoord, this, rootNodeCoord);
        IDictionary<FastVec2i, RiverNode> nodes = new Dictionary<FastVec2i, RiverNode>();
        nodes.Add(rootNodeCoord, rootNode);
        topologicallySorted.Add(rootNodeCoord);
        FastVec2i[] relativeCoords = new FastVec2i[6]
        {
            new FastVec2i(0, -1),
            new FastVec2i(1, 0),
            new FastVec2i(1, 1),
            new FastVec2i(0, 1),
            new FastVec2i(-1, 0),
            new FastVec2i(-1, -1),
        };
        for (int i = 0; i < topologicallySorted.Count; ++i)
        {
            FastVec2i nodeCoord = topologicallySorted[i];
            RiverNode node = nodes[nodeCoord];
            foreach (FastVec2i relativeCoord in relativeCoords)
            {
                FastVec2i neighborNodeCoord = nodeCoord + relativeCoord;
                RiverNode neighborNode;
                if (!nodes.TryGetValue(neighborNodeCoord, out neighborNode))
                {
                    neighborNode = new RiverNode(rng, basinCoord, this, neighborNodeCoord);
                    if (neighborNode.flow > 0.0)
                    {
                        nodes.Add(neighborNodeCoord, neighborNode);
                        topologicallySorted.Add(neighborNodeCoord);
                    }
                }
            }
        }
        rng.InitPositionSeed(basinCoord.X, basinCoord.Y);
        for (int i = topologicallySorted.Count - 1; i >= 0; --i)
        {
            FastVec2i nodeCoord = topologicallySorted[i];
            RiverNode node = nodes[nodeCoord];
            List<FastVec2i> candidateNodeCoords = new List<FastVec2i>();
            foreach (FastVec2i relativeCoord in relativeCoords)
            {
                FastVec2i neighborNodeCoord = nodeCoord + relativeCoord;
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
            segment.a = new FastVec2i(upstreamNode.position.X, upstreamNode.position.Y);
            segment.b = new FastVec2i(node.position.X, node.position.Y);
            segment.c = new FastVec2i(downstreamNode.position.X, downstreamNode.position.Y);
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
