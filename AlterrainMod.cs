using System;
using System.Collections.Generic;
using Vintagestory.API.Common;
using Vintagestory.API.Server;
using Vintagestory.API.Datastructures;
using Vintagestory.API.Config;
using Vintagestory.API.MathTools;
using Vintagestory.ServerMods;
using static Vintagestory.ServerMods.NoObf.GlobalConfig;

namespace Alterrain
{
    public static class HeightMapRenderer
    {
        // https://www.geeksforgeeks.org/python/bresenhams-algorithm-for-3-d-line-drawing/
        public static void Bresenham3D(float[] quadratureMap, int stride, int x1, int y1, int z1, int x2, int y2, int z2)
        {
            quadratureMap[z1 * stride + x1] = (float) y1;
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
                    quadratureMap[z1 * stride + x1] = (float) y1 * (float) y1;
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
                    quadratureMap[z1 * stride + x1] = (float) y1 * (float) y1;
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
                    quadratureMap[z1 * stride + x1] = (float) y1 * (float) y1;
                }
            }
        }

        // https://cs.brown.edu/people/pfelzens/papers/dt-final.pdf (Distance Transforms of Sampled Functions, Pedro F. Felzenszwalb, Daniel P. Huttenlocher)
        public static void DistanceTransformRowOrColumn(float[] quadratureMap, uint offset, uint stride, uint n)
        {
            uint[] v = new uint[n];
            float[] z = new float[n + 1];
            float[] aux = new float[n];
            for (uint q = 0; q < n; ++q)
            {
                aux[q] = quadratureMap[offset + stride * q];
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
                quadratureMap[offset + stride * q] = (q - vk) * (q - vk) + aux[vk];
            }
        }

        public static void DistanceTransform(float[] quadratureMap, uint width, uint height)
        {
            for (uint y = 0; y < height; ++y)
            {
                DistanceTransformRowOrColumn(quadratureMap, y * width, 1, width);
            }
            for (uint x = 0; x < width; ++x)
            {
                DistanceTransformRowOrColumn(quadratureMap, x, width, height);
            }
        }

        public static void RenderStream(ICoreServerAPI api, LCGRandom rng, float[] quadratureMap, uint regionSize, int offsetX, int offsetZ)
        {
            (double positionX, double positionZ) = (offsetX * regionSize + rng.NextInt((int) regionSize), offsetZ * regionSize + rng.NextInt((int) regionSize));
            (double directionX, double directionZ) = (offsetX * regionSize + regionSize / 2 - positionX, offsetZ * regionSize + regionSize / 2 - positionZ);
            double factor = 15.0 / Math.Sqrt(directionX * directionX + directionZ * directionZ);
            directionX *= factor;
            directionZ *= factor;
            double angularVelocity = 0, angularVelocityDelayed = 0;
            for (uint i = 0; i < 100; ++i)
            {
                (double prevX, double prevZ) = (positionX, positionZ);
                positionX += directionX;
                positionZ += directionZ;
                if (positionX < offsetX * regionSize || positionX >= (offsetX + 1) * regionSize || positionZ < offsetZ * regionSize || positionZ >= (offsetZ + 1) * regionSize)
                {
                    break;
                }
                HeightMapRenderer.Bresenham3D(quadratureMap, (int) (regionSize * 3), (int) prevX, 0, (int) prevZ, (int) positionX, 0, (int) positionZ);
                (double s, double c) = Math.SinCos(angularVelocity);
                (directionX, directionZ) = (directionX * c - directionZ * s, directionX * s + directionZ * c);
                angularVelocity = GameMath.Clamp(angularVelocity * 0.95 - 0.05 * angularVelocityDelayed + (rng.NextFloat() - 0.5) * 0.2 * Math.PI, -0.3, 0.3);
                angularVelocityDelayed += angularVelocity;
            }
        }
    }

    public class AlterrainMod : ModStdWorldGen
    {
        ICoreServerAPI api;
        int mantleBlockId;
        int defaultRockId;
        int waterBlockId;
        LCGRandom rng;

        private void OnInitWorldGen()
        {
            rng = new LCGRandom(api.WorldManager.Seed);
        }

        private void OnMapRegionGen(IMapRegion mapRegion, int regionX, int regionZ, ITreeAttribute chunkGenParams = null)
        {
            api.Server.LogEvent(String.Format("OnMapRegionGen {0} {1}", regionX, regionZ));
            float[] quadratureMap = new float[api.WorldManager.RegionSize * api.WorldManager.RegionSize * 9];
            for (uint i = 0; i < quadratureMap.Length; i++)
            {
                quadratureMap[i] = 150F * 150F;
            }
            IntDataMap2D upheavalMap = mapRegion.UpheavelMap;
            int regionChunkSize = api.WorldManager.RegionSize / GlobalConstants.ChunkSize;
            float ufac = (float)upheavalMap.InnerSize / regionChunkSize;
            const float chunkBlockDelta = 1.0f / GlobalConstants.ChunkSize;
            for (int offsetZ = -1; offsetZ <= 1; ++offsetZ)
            {
                for (int offsetX = -1; offsetX <= 1; ++offsetX)
                {
                    rng.InitPositionSeed(regionX + offsetX, regionZ + offsetZ);
                    HeightMapRenderer.RenderStream(api, rng, quadratureMap, (uint) api.WorldManager.RegionSize, offsetX + 1, offsetZ + 1);
                }
            }
            HeightMapRenderer.DistanceTransform(quadratureMap, (uint) api.WorldManager.RegionSize * 3, (uint) api.WorldManager.RegionSize * 3);
            for (uint rlZ = 0; rlZ < regionChunkSize; ++rlZ)
            {
                for (uint rlX = 0; rlX < regionChunkSize; ++rlX)
                {
                    double upheavalMapUpLeft = 0;
                    double upheavalMapUpRight = 0;
                    double upheavalMapBotLeft = 0;
                    double upheavalMapBotRight = 0;
                    if (upheavalMap != null)
                    {
                        upheavalMapUpLeft = upheavalMap.GetUnpaddedInt((int)(rlX * ufac), (int)(rlZ * ufac));
                        upheavalMapUpRight = upheavalMap.GetUnpaddedInt((int)(rlX * ufac + ufac), (int)(rlZ * ufac));
                        upheavalMapBotLeft = upheavalMap.GetUnpaddedInt((int)(rlX * ufac), (int)(rlZ * ufac + ufac));
                        upheavalMapBotRight = upheavalMap.GetUnpaddedInt((int)(rlX * ufac + ufac), (int)(rlZ * ufac + ufac));
                    }
                    ushort[] heightMap = new ushort[GlobalConstants.ChunkSize * GlobalConstants.ChunkSize];
                    for (uint lZ = 0; lZ < GlobalConstants.ChunkSize; lZ++)
                    {
                        for (uint lX = 0; lX < GlobalConstants.ChunkSize; lX++)
                        {
                            double upheaval = GameMath.BiLerp(upheavalMapUpLeft, upheavalMapUpRight, upheavalMapBotLeft, upheavalMapBotRight, lX * chunkBlockDelta, lZ * chunkBlockDelta) / api.WorldManager.MapSizeY;
                            double height = upheaval * Math.Sqrt(quadratureMap[(api.WorldManager.RegionSize + rlZ * GlobalConstants.ChunkSize + lZ) * api.WorldManager.RegionSize * 3 + (api.WorldManager.RegionSize + rlX * GlobalConstants.ChunkSize + lX)]);
                            height = (height <= 7.0) ? (height - 7.0) :
                                    (height <= 18.0) ? height * 0.07 :
                                    (height <= 33.0) ? height * 0.5 - 8.0 :
                                    height - 24.0;
                            height = Math.Min(height + TerraGenConfig.seaLevel, api.WorldManager.MapSizeY - 1);
                            heightMap[lZ * GlobalConstants.ChunkSize + lX] = (ushort) height;
                        }
                    }
                    mapRegion.SetModdata<ushort[]>(String.Format("heightMap:{0},{1}", rlX, rlZ), heightMap);
                }
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
                    rainheightmap[offset] = (ushort) Math.Max(YMax, TerraGenConfig.seaLevel);
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
                        chunkBlockData[(int) ((lY % GlobalConstants.ChunkSize) * stride + offset)] = waterBlockId;
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
            var GenCaves1 = handles.OnChunkColumnGen[(int)EnumWorldGenPass.Terrain].FindIndex(a => a.Method.DeclaringType == typeof(GenPartial));
            handles.OnChunkColumnGen[(int)EnumWorldGenPass.Terrain].RemoveAt(GenCaves1);
            var GenCaves2 = handles.OnMapChunkGen.FindIndex(a => a.Method.DeclaringType == typeof(GenCaves));
            handles.OnMapChunkGen.RemoveAt(GenCaves2);
            api.Event.InitWorldGenerator(OnInitWorldGen, "standard");
            handles.OnMapRegionGen.Add(OnMapRegionGen);
            handles.OnChunkColumnGen[(int)EnumWorldGenPass.Terrain].Insert(0, OnChunkColumnGen);
        }
    }
}
