using System;
using System.Collections.Generic;
using Vintagestory.API.Config;
using Vintagestory.API.Common;
using Vintagestory.API.Datastructures;
using Vintagestory.API.MathTools;
using Vintagestory.API.Server;
using Vintagestory.ServerMods;

namespace Alterrain
{
    public class AlterrainMod : ModStdWorldGen
    {
        ICoreServerAPI api;
        int mantleBlockId;
        int defaultRockId;
        int waterBlockId;
        SlopeProfile slopeProfile;
        IDictionary<FastVec2i, List<QuadraticBezierCurve>> drainageSystems;
        LCGRandom rng;

        private void OnInitWorldGen()
        {
            drainageSystems = new Dictionary<FastVec2i, List<QuadraticBezierCurve>>();
            rng = new LCGRandom(api.WorldManager.Seed);
        }

        private void OnMapRegionGen(IMapRegion mapRegion, int regionX, int regionZ, ITreeAttribute chunkGenParams = null)
        {
            float maxMountainHeight = (float) (api.WorldManager.MapSizeY - TerraGenConfig.seaLevel) - 7.0F;
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
            Basin centalBasin = new Basin(rng, centralBasinCoord);
            centalBasin.InitClimate(api, rng, centralBasinCoord);
            int stride = renderer.frame.X2 - renderer.frame.X1;
            const float chunkBlockDelta = 1.0f / GlobalConstants.ChunkSize;
            int regionChunkSize = api.WorldManager.RegionSize / GlobalConstants.ChunkSize;
            ushort[] heightMap = new ushort[GlobalConstants.ChunkSize * GlobalConstants.ChunkSize];
            for (int rlZ = 0; rlZ < regionChunkSize; ++rlZ)
            {
                for (int rlX = 0; rlX < regionChunkSize; ++rlX)
                {
                    int chunkGlobalX = renderer.frame.X1 + api.WorldManager.RegionSize + rlX * GlobalConstants.ChunkSize;
                    int chunkGlobalZ = renderer.frame.Y1 + api.WorldManager.RegionSize + rlZ * GlobalConstants.ChunkSize;
                    double depressionStrength = 2.0 / Basin.cellSpacing;
                    (double depressionUpLeft, _) = centalBasin.FindClosestBasin(2, 2, new FastVec2i(chunkGlobalX, chunkGlobalZ));
                    (double depressionUpRight, _) = centalBasin.FindClosestBasin(2, 2, new FastVec2i(chunkGlobalX + GlobalConstants.ChunkSize, chunkGlobalZ));
                    (double depressionBotLeft, _) = centalBasin.FindClosestBasin(2, 2, new FastVec2i(chunkGlobalX, chunkGlobalZ + GlobalConstants.ChunkSize));
                    (double depressionBotRight, _) = centalBasin.FindClosestBasin(2, 2, new FastVec2i(chunkGlobalX + GlobalConstants.ChunkSize, chunkGlobalZ + GlobalConstants.ChunkSize));
                    depressionUpLeft = Math.Min(1.0, Math.Sqrt(depressionUpLeft) * depressionStrength);
                    depressionUpRight = Math.Min(1.0, Math.Sqrt(depressionUpRight) * depressionStrength);
                    depressionBotLeft = Math.Min(1.0, Math.Sqrt(depressionBotLeft) * depressionStrength);
                    depressionBotRight = Math.Min(1.0, Math.Sqrt(depressionBotRight) * depressionStrength);
                    for (int lZ = 0; lZ < GlobalConstants.ChunkSize; lZ++)
                    {
                        for (int lX = 0; lX < GlobalConstants.ChunkSize; lX++)
                        {
                            double depression = GameMath.BiLerp(depressionUpLeft, depressionUpRight, depressionBotLeft, depressionBotRight, lX * chunkBlockDelta, lZ * chunkBlockDelta);
                            double distance = Math.Sqrt(renderer.squaredDistanceMap[(chunkGlobalZ - renderer.frame.Y1 + lZ) * stride + (chunkGlobalX - renderer.frame.X1 + lX)]);
                            double height = Math.Min(TerraGenConfig.seaLevel + slopeProfile.distanceToHeight(distance * depression), api.WorldManager.MapSizeY - 3);
                            heightMap[lZ * GlobalConstants.ChunkSize + lX] = (ushort) height;
                        }
                    }
                    mapRegion.SetModdata<ushort[]>(String.Format("heightMap:{0},{1}", rlX, rlZ), heightMap);
                }
            }
            for (int i = 0; i < mapRegion.UpheavelMap.Data.Length; ++i)
            {
                mapRegion.UpheavelMap.Data[i] = 0;
            }
            for (int i = 0; i < mapRegion.OceanMap.Data.Length; ++i)
            {
                mapRegion.OceanMap.Data[i] = 0;
            }
            int climateMapOrigX = renderer.frame.X1 + api.WorldManager.RegionSize - mapRegion.ClimateMap.TopLeftPadding * TerraGenConfig.climateMapScale;
            int climateMapOrigZ = renderer.frame.Y1 + api.WorldManager.RegionSize - mapRegion.ClimateMap.TopLeftPadding * TerraGenConfig.climateMapScale;
            for (int pixelZ = 0; pixelZ < mapRegion.ClimateMap.Size; ++pixelZ)
            {
                for (int pixelX = 0; pixelX < mapRegion.ClimateMap.Size; ++pixelX)
                {
                    (_, int closestBasinIndex) = centalBasin.FindClosestBasin(2, 2, new FastVec2i(
                        climateMapOrigX + pixelX * TerraGenConfig.climateMapScale, climateMapOrigZ + pixelZ * TerraGenConfig.climateMapScale
                    ));
                    mapRegion.ClimateMap.Data[pixelZ * mapRegion.ClimateMap.Size + pixelX] = centalBasin.neighborClimate[closestBasinIndex];
                }
            }
            int forestMapOrigX = api.WorldManager.RegionSize - mapRegion.ForestMap.TopLeftPadding * TerraGenConfig.forestMapScale;
            int forestMapOrigZ = api.WorldManager.RegionSize - mapRegion.ForestMap.TopLeftPadding * TerraGenConfig.forestMapScale;
            for (int pixelZ = 0; pixelZ < mapRegion.ForestMap.Size; ++pixelZ)
            {
                for (int pixelX = 0; pixelX < mapRegion.ForestMap.Size; ++pixelX)
                {
                    double distToRiver = Math.Sqrt(renderer.squaredDistanceMap[
                        (forestMapOrigZ + pixelZ * TerraGenConfig.forestMapScale) * stride + (forestMapOrigX + pixelX * TerraGenConfig.forestMapScale)
                    ]);
                    mapRegion.ForestMap.Data[pixelZ * mapRegion.ForestMap.Size + pixelX] = (int) (511.0 * Math.Max(0.0, 1.0 - Math.Abs(distToRiver - 40.0) / 30.0));
                }
            }
            for (int i = 0; i < mapRegion.BeachMap.Data.Length; ++i)
            {
                mapRegion.BeachMap.Data[i] = 0;
            }
            int geoProvMapOrigX = renderer.frame.X1 + api.WorldManager.RegionSize - mapRegion.GeologicProvinceMap.TopLeftPadding * TerraGenConfig.geoProvMapScale;
            int geoProvMapOrigZ = renderer.frame.Y1 + api.WorldManager.RegionSize - mapRegion.GeologicProvinceMap.TopLeftPadding * TerraGenConfig.geoProvMapScale;
            for (int pixelZ = 0; pixelZ < mapRegion.GeologicProvinceMap.Size; ++pixelZ)
            {
                for (int pixelX = 0; pixelX < mapRegion.GeologicProvinceMap.Size; ++pixelX)
                {
                    (double depression, _) = centalBasin.FindClosestBasin(2, 2, new FastVec2i(
                        geoProvMapOrigZ + pixelZ * TerraGenConfig.geoProvMapScale, geoProvMapOrigX + pixelX * TerraGenConfig.geoProvMapScale
                    ));
                    depression = Math.Sqrt(depression) / Basin.cellSpacing;
                    mapRegion.GeologicProvinceMap.Data[pixelZ * mapRegion.GeologicProvinceMap.Size + pixelX] = (depression < 0.2) ? 3 : (depression < 0.4) ? 2 : 0;
                }
            }
            for (int i = 0; i < mapRegion.LandformMap.Data.Length; ++i)
            {
                mapRegion.LandformMap.Data[i] = 0;
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
            for (int lZ = 0; lZ < GlobalConstants.ChunkSize; lZ++)
            {
                for (int lX = 0; lX < GlobalConstants.ChunkSize; lX++)
                {
                    ushort height = heightMap[lZ * GlobalConstants.ChunkSize + lX];
                    YMin = Math.Min(YMin, height);
                    mapChunk.YMax = Math.Max(mapChunk.YMax, height);
                }
            }
            ++YMin;
            int stride = GlobalConstants.ChunkSize * GlobalConstants.ChunkSize;
            for (int chunkY = 0; chunkY < (YMin + GlobalConstants.ChunkSize - 1) / GlobalConstants.ChunkSize; chunkY++)
            {
                chunkBlockData = request.Chunks[chunkY].Data;
                int endY = Math.Min(YMin - chunkY * GlobalConstants.ChunkSize, GlobalConstants.ChunkSize);
                for (int lY = 0; lY < endY; lY++) {
                    chunkBlockData.SetBlockBulk(lY * stride, GlobalConstants.ChunkSize, GlobalConstants.ChunkSize, defaultRockId);
                }
            }
            chunkBlockData = request.Chunks[0].Data;
            chunkBlockData.SetBlockBulk(0, GlobalConstants.ChunkSize, GlobalConstants.ChunkSize, mantleBlockId);
            for (int lZ = 0; lZ < GlobalConstants.ChunkSize; lZ++)
            {
                for (int lX = 0; lX < GlobalConstants.ChunkSize; lX++)
                {
                    int offset = lZ * GlobalConstants.ChunkSize + lX;
                    int YMax = heightMap[offset];
                    rainheightmap[offset] = (ushort) Math.Max(YMax, TerraGenConfig.seaLevel - 1);
                    terrainheightmap[offset] = (ushort) YMax;
                    ++YMax;
                    for (int lY = YMin; lY < YMax; lY++)
                    {
                        chunkBlockData = request.Chunks[lY / GlobalConstants.ChunkSize].Data;
                        chunkBlockData[(lY % GlobalConstants.ChunkSize) * stride + offset] = defaultRockId;
                    }
                    for (int lY = YMax; lY < TerraGenConfig.seaLevel; lY++)
                    {
                        chunkBlockData = request.Chunks[lY / GlobalConstants.ChunkSize].Data;
                        chunkBlockData.SetFluid((lY % GlobalConstants.ChunkSize) * stride + offset, waterBlockId);
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
            slopeProfile = new SlopeProfile(new (double, double, double)[] {
                (7.0, -7.0, 1.0),
                (11.0, 0.0, 0.07),
                (15.0, 0.0, 0.5),
                (17.0, 0.0, 1.0),
                (10000.0, 0.0, 1.5)
            });
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
