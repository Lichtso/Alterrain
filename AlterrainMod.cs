using System;
using System.Collections.Generic;
using Vintagestory.API.Common;
using Vintagestory.API.Server;
using Vintagestory.API.Datastructures;
using Vintagestory.API.Config;
using Vintagestory.API.MathTools;
using Vintagestory.ServerMods;

namespace Alterrain
{
    public static class HeightMapRenderer
    {
        // https://www.geeksforgeeks.org/python/bresenhams-algorithm-for-3-d-line-drawing/
        public static void Bresenham3D(float[] squaredDistanceMap, int stride, int x1, int y1, int z1, int x2, int y2, int z2)
        {
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
        public static void DistanceTransformRowOrColumn(float[] squaredDistanceMap, uint offset, uint stride, uint n)
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

        public static void DistanceTransform(float[] squaredDistanceMap, uint width, uint height)
        {
            for (uint y = 0; y < height; ++y)
            {
                DistanceTransformRowOrColumn(squaredDistanceMap, y * width, 1, width);
            }
            for (uint x = 0; x < width; ++x)
            {
                DistanceTransformRowOrColumn(squaredDistanceMap, x, width, height);
            }
        }
    }

    public class AlterrainMod : ModStdWorldGen
    {
        ICoreServerAPI api;
        int mantleBlockId;
        int defaultRockId;
        int waterBlockId;
        IDictionary<FastVec2i, List<(FastVec3i, FastVec3i)>> drainageSystems;
        LCGRandom rng;

        private void OnInitWorldGen()
        {
            drainageSystems = new Dictionary<FastVec2i, List<(FastVec3i, FastVec3i)>>();
            rng = new LCGRandom(api.WorldManager.Seed);
        }

        private void OnMapRegionGen(IMapRegion mapRegion, int regionX, int regionZ, ITreeAttribute chunkGenParams = null)
        {
            int regionChunkSize = api.WorldManager.RegionSize / GlobalConstants.ChunkSize;
            const float riverDepth = 7.0F;
            float maxMountainHeight = (float) (api.WorldManager.MapSizeY - TerraGenConfig.seaLevel) - riverDepth;
            float[] squaredDistanceMap = new float[api.WorldManager.RegionSize * api.WorldManager.RegionSize * 9];
            for (int i = 0; i < squaredDistanceMap.Length; ++i)
            {
                squaredDistanceMap[i] = squaredDistanceMap.Length;
            }
            Rectanglei region = new Rectanglei((regionX - 1) * api.WorldManager.RegionSize, (regionZ - 1) * api.WorldManager.RegionSize, api.WorldManager.RegionSize * 3, api.WorldManager.RegionSize * 3);
            FastVec2i centralBasinCoord = new FastVec2i(
                (region.X2 + region.X1) / (2 * Basin.cellSpacing),
                (region.Y2 + region.Y1) / (2 * Basin.cellSpacing)
            );
            for (int z = 0; z < 3; ++z)
            {
                for (int x = 0; x < 3; ++x)
                {
                    FastVec2i basinCoord = new FastVec2i(
                        centralBasinCoord.X + x - 1,
                        centralBasinCoord.Y + z - 1
                    );
                    List<(FastVec3i, FastVec3i)> drainageSystem;
                    if (!drainageSystems.TryGetValue(basinCoord, out drainageSystem))
                    {
                        Basin basin = new Basin(rng, basinCoord);
                        drainageSystem = basin.GenerateDrainageSystem(rng, basinCoord, maxMountainHeight);
                        drainageSystems.Add(basinCoord, drainageSystem);
                    }
                    foreach ((FastVec3i upstream, FastVec3i downstream) in drainageSystem)
                    {
                        HeightMapRenderer.Bresenham3D(squaredDistanceMap, api.WorldManager.RegionSize * 3, upstream.X, upstream.Y, upstream.Z, downstream.X, downstream.Y, downstream.Z);
                    }
                }
            }
            HeightMapRenderer.DistanceTransform(squaredDistanceMap, (uint) api.WorldManager.RegionSize * 3, (uint) api.WorldManager.RegionSize * 3);
            ushort[] heightMap = new ushort[GlobalConstants.ChunkSize * GlobalConstants.ChunkSize];
            for (uint rlZ = 0; rlZ < regionChunkSize; ++rlZ)
            {
                for (uint rlX = 0; rlX < regionChunkSize; ++rlX)
                {
                    for (uint lZ = 0; lZ < GlobalConstants.ChunkSize; lZ++)
                    {
                        for (uint lX = 0; lX < GlobalConstants.ChunkSize; lX++)
                        {
                            double height = Math.Sqrt(squaredDistanceMap[(api.WorldManager.RegionSize + rlZ * GlobalConstants.ChunkSize + lZ) * api.WorldManager.RegionSize * 3 + (api.WorldManager.RegionSize + rlX * GlobalConstants.ChunkSize + lX)]);
                            height = (height < riverDepth) ? (height - riverDepth) :
                                    (height < 18.0) ? height * 0.07 :
                                    (height < 33.0) ? height * 0.5 - 8.0 :
                                    (height < 50.0) ? height - 24.0 :
                                    height * 1.5 - 49.0;
                            height = Math.Min(height + TerraGenConfig.seaLevel, api.WorldManager.MapSizeY - 1);
                            heightMap[lZ * GlobalConstants.ChunkSize + lX] = (ushort) height;
                        }
                    }
                    mapRegion.SetModdata<ushort[]>(String.Format("heightMap:{0},{1}", rlX, rlZ), heightMap);
                }
            }
            for (uint i = 0; i < mapRegion.BeachMap.Data.Length; ++i)
            {
                mapRegion.BeachMap.Data[i] = 0;
            }
        }

        private void OnChunkColumnGen(IChunkColumnGenerateRequest request)
        {
            int regionChunkSize = api.WorldManager.RegionSize / GlobalConstants.ChunkSize;
            int rlX = request.ChunkX % regionChunkSize;
            int rlZ = request.ChunkZ % regionChunkSize;
            IMapChunk mapChunk = request.Chunks[0].MapChunk;
            string chunkDataName = String.Format("heightMap:{0},{1}", rlX, rlZ);
            ushort[] heightMap = mapChunk.MapRegion.GetModdata<ushort[]>(chunkDataName);
            mapChunk.MapRegion.RemoveModdata(chunkDataName);
            ushort[] rainheightmap = mapChunk.RainHeightMap;
            ushort[] terrainheightmap = mapChunk.WorldGenTerrainHeightMap;
            IChunkBlocks chunkBlockData = request.Chunks[0].Data;
            mapChunk.YMax = 0;
            ushort YMin = (ushort) api.WorldManager.MapSizeY;
            for (uint lZ = 0; lZ < GlobalConstants.ChunkSize; lZ++)
            {
                for (uint lX = 0; lX < GlobalConstants.ChunkSize; lX++)
                {
                    ushort height = heightMap[lZ * GlobalConstants.ChunkSize + lX];
                    YMin = Math.Min(YMin, height);
                    mapChunk.YMax = Math.Max(mapChunk.YMax, height);
                }
            }
            ++YMin;
            uint stride = GlobalConstants.ChunkSize * GlobalConstants.ChunkSize;
            for (uint chunkY = 0; chunkY < (YMin + GlobalConstants.ChunkSize - 1) / GlobalConstants.ChunkSize; chunkY++)
            {
                chunkBlockData = request.Chunks[chunkY].Data;
                uint endY = (uint) Math.Min(YMin - chunkY * GlobalConstants.ChunkSize, GlobalConstants.ChunkSize);
                for (uint lY = 0; lY < endY; lY++) {
                    chunkBlockData.SetBlockBulk((int) (lY * stride), GlobalConstants.ChunkSize, GlobalConstants.ChunkSize, defaultRockId);
                }
            }
            chunkBlockData = request.Chunks[0].Data;
            chunkBlockData.SetBlockBulk(0, GlobalConstants.ChunkSize, GlobalConstants.ChunkSize, mantleBlockId);
            for (uint lZ = 0; lZ < GlobalConstants.ChunkSize; lZ++)
            {
                for (uint lX = 0; lX < GlobalConstants.ChunkSize; lX++)
                {
                    uint offset = lZ * GlobalConstants.ChunkSize + lX;
                    uint YMax = heightMap[offset];
                    rainheightmap[offset] = (ushort) Math.Max(YMax, TerraGenConfig.seaLevel - 1);
                    terrainheightmap[offset] = (ushort) YMax;
                    ++YMax;
                    for (int lY = YMin; lY < YMax; lY++)
                    {
                        chunkBlockData = request.Chunks[lY / GlobalConstants.ChunkSize].Data;
                        chunkBlockData[(int) ((lY % GlobalConstants.ChunkSize) * stride + offset)] = defaultRockId;
                    }
                    for (uint lY = YMax; lY < TerraGenConfig.seaLevel; lY++)
                    {
                        chunkBlockData = request.Chunks[lY / GlobalConstants.ChunkSize].Data;
                        chunkBlockData.SetFluid((int) ((lY % GlobalConstants.ChunkSize) * stride + offset), waterBlockId);
                    }
                }
            }
        }

        public override bool ShouldLoad(EnumAppSide side)
        {
            return side == EnumAppSide.Server;
        }

        public override double ExecuteOrder()
        {
            return 0.5;
        }

        public override void AssetsFinalize(ICoreAPI coreApi)
        {
            api = (ICoreServerAPI)coreApi;
            mantleBlockId = api.World.GetBlock("mantle")?.BlockId ?? 0;
            defaultRockId = api.World.GetBlock("rock-granite")?.BlockId ?? 0;
            waterBlockId = api.World.GetBlock("water-still-7")?.BlockId ?? 0;
        }

        public override void StartServerSide(ICoreServerAPI coreApi)
        {
            api = coreApi;
            IWorldGenHandler handles = api.Event.GetRegisteredWorldGenHandlers("standard");
            var GenTerra = handles.OnChunkColumnGen[(int)EnumWorldGenPass.Terrain].FindIndex(a => a.Method.DeclaringType == typeof(GenTerra));
            handles.OnChunkColumnGen[(int)EnumWorldGenPass.Terrain].RemoveAt(GenTerra);
            api.Event.InitWorldGenerator(OnInitWorldGen, "standard");
            handles.OnMapRegionGen.Add(OnMapRegionGen);
            handles.OnChunkColumnGen[(int)EnumWorldGenPass.Terrain].Insert(0, OnChunkColumnGen);
        }
    }
}
