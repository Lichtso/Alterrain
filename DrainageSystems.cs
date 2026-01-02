using System;
using System.Collections.Generic;
using Vintagestory.API.MathTools;

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
        return new FastVec2i(x, (int) ((positionZ + cellHeight * x / 2) / cellHeight));
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
    public FastVec2i[] neighborBasins;

    public const int cellSpacing = 2048;

    public Basin(LCGRandom rng, FastVec2i basinCoord)
    {
        neighborBasins = new FastVec2i[25];
        for (int z = 0; z < 5; ++z)
        {
            for (int x = 0; x < 5; ++x)
            {
                rng.InitPositionSeed(basinCoord.X + x - 2, basinCoord.Y + z - 2);
                neighborBasins[z * 5 + x] = new FastVec2i(
                    (basinCoord.X + x - 2) * cellSpacing + rng.NextInt(cellSpacing),
                    (basinCoord.Y + z - 2) * cellSpacing + rng.NextInt(cellSpacing)
                );
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
                double diffX = neighborBasins[i].X - position.X;
                double diffZ = neighborBasins[i].Y - position.Y;
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

    public List<QuadraticBezierCurve> GenerateDrainageSystem(LCGRandom rng, FastVec2i basinCoord, float maxMountainHeight)
    {
        List<FastVec2i> topologicallySorted = new List<FastVec2i>();
        FastVec2i rootNodeCoord = RiverNode.CoordsAt((int) neighborBasins[12].X, (int) neighborBasins[12].Y);
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
            segment.a = (segment.a + segment.b) / 2;
            segment.c = (segment.c + segment.b) / 2;
            segment.height = (int) Math.Max(0.0F, maxMountainHeight - upstreamNode.flow * 8.0F);
            drainageSystem.Add(segment);
        }
        return drainageSystem;
    }
}
