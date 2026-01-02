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
    public class AlterrainMod : ModStdWorldGen
    {
        ICoreServerAPI api;
        int mantleBlockId;
        int defaultRockId;
        int waterBlockId;
        IDictionary<FastVec2i, List<QuadraticBezierCurve>> drainageSystems;
        LCGRandom rng;

        private void OnInitWorldGen()
        {
            drainageSystems = new Dictionary<FastVec2i, List<QuadraticBezierCurve>>();
            rng = new LCGRandom(api.WorldManager.Seed);
        }

        private void OnMapRegionGen(IMapRegion mapRegion, int regionX, int regionZ, ITreeAttribute chunkGenParams = null)
        {
            const float riverDepth = 7.0F;
            float maxMountainHeight = (float) (api.WorldManager.MapSizeY - TerraGenConfig.seaLevel) - riverDepth;
            HeightMapRenderer renderer = new HeightMapRenderer(new Rectanglei(
                (regionX - 1) * api.WorldManager.RegionSize,
                (regionZ - 1) * api.WorldManager.RegionSize,
                api.WorldManager.RegionSize * 3,
                api.WorldManager.RegionSize * 3
            ));
            FastVec2i centralBasinCoord = new FastVec2i(
                (renderer.frame.X2 + renderer.frame.X1) / (2 * Basin.cellSpacing),
                (renderer.frame.Y2 + renderer.frame.Y1) / (2 * Basin.cellSpacing)
            );
            for (int z = 0; z < 3; ++z)
            {
                for (int x = 0; x < 3; ++x)
                {
                    FastVec2i basinCoord = new FastVec2i(
                        centralBasinCoord.X + x - 1,
                        centralBasinCoord.Y + z - 1
                    );
                    List<QuadraticBezierCurve> drainageSystem;
                    if (!drainageSystems.TryGetValue(basinCoord, out drainageSystem))
                    {
                        Basin basin = new Basin(rng, basinCoord);
                        drainageSystem = basin.GenerateDrainageSystem(rng, basinCoord, maxMountainHeight);
                        drainageSystems.Add(basinCoord, drainageSystem);
                    }
                    foreach (QuadraticBezierCurve segment in drainageSystem)
                    {
                        renderer.QuadraticBezierCurve(segment.a.X, segment.a.Y, segment.b.X, segment.b.Y, segment.c.X, segment.c.Y, segment.height);
                    }
                }
            }
            renderer.DistanceTransform();
            uint stride = (uint) (renderer.frame.X2 - renderer.frame.X1);
            int regionChunkSize = api.WorldManager.RegionSize / GlobalConstants.ChunkSize;
            ushort[] heightMap = new ushort[GlobalConstants.ChunkSize * GlobalConstants.ChunkSize];
            for (uint rlZ = 0; rlZ < regionChunkSize; ++rlZ)
            {
                for (uint rlX = 0; rlX < regionChunkSize; ++rlX)
                {
                    for (uint lZ = 0; lZ < GlobalConstants.ChunkSize; lZ++)
                    {
                        for (uint lX = 0; lX < GlobalConstants.ChunkSize; lX++)
                        {
                            double height = Math.Sqrt(renderer.squaredDistanceMap[(api.WorldManager.RegionSize + rlZ * GlobalConstants.ChunkSize + lZ) * stride + (api.WorldManager.RegionSize + rlX * GlobalConstants.ChunkSize + lX)]);
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
