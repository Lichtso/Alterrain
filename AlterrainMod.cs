using System;
using System.Collections.Generic;
using Vintagestory.API.Config;
using Vintagestory.API.Common;
using Vintagestory.API.Datastructures;
using Vintagestory.API.MathTools;
using Vintagestory.API.Server;
using Vintagestory.API.Util;
using Vintagestory.ServerMods;

namespace Alterrain
{
    public class AlterrainMod : ModStdWorldGen
    {
        ICoreServerAPI api;
        IWorldGenBlockAccessor blockAccessor;
        int mantleBlockId;
        int defaultRockId;
        int waterBlockId;
        SlopeProfile slopeProfile;
        IDictionary<FastVec2i, List<QuadraticBezierCurve>> drainageSystems;
        LCGRandom rng;
        HexGrid riverGrid;
        HexGrid basinGrid;

        private void OnWorldGenBlockAccessor(IChunkProviderThread chunkProvider)
        {
            blockAccessor = chunkProvider.GetBlockAccessor(true);
        }

        private void OnInitWorldGen()
        {
            drainageSystems = new Dictionary<FastVec2i, List<QuadraticBezierCurve>>();
            rng = new LCGRandom(api.WorldManager.Seed);
            ITreeAttribute worldConfig = api.WorldManager.SaveGame.WorldConfiguration;
            float landformScale = worldConfig.GetString("landformScale", "1").ToFloat(1);
            riverGrid = new HexGrid((int) (landformScale * 64.0F));
            basinGrid = new HexGrid((int) (landformScale * 3500.0F));
        }

        private void OnMapRegionGen(IMapRegion mapRegion, int regionX, int regionZ, ITreeAttribute chunkGenParams = null)
        {
            float mountainStreamStartHeight = (float) (api.WorldManager.MapSizeY - TerraGenConfig.seaLevel) - 55.0F;
            HeightMapRenderer renderer = new HeightMapRenderer(new Rectanglei(
                (regionX - 1) * api.WorldManager.RegionSize,
                (regionZ - 1) * api.WorldManager.RegionSize,
                api.WorldManager.RegionSize * 3,
                api.WorldManager.RegionSize * 3
            ));
            FastVec2i regionCenter = new FastVec2i(
                (renderer.frame.X2 + renderer.frame.X1) / 2,
                (renderer.frame.Y2 + renderer.frame.Y1) / 2
            );
            FastVec2i basinCoord = basinGrid.CartesianToHex(regionCenter);
            (_, FastVec2i closestBasinCoord) = basinGrid.VoronoiClosest(rng, regionCenter);
            for (int i = 0; i < 7; ++i)
            {
                basinCoord = closestBasinCoord + basinGrid.neighborHexOffsets[i];
                List<QuadraticBezierCurve> drainageSystem;
                if (!drainageSystems.TryGetValue(basinCoord, out drainageSystem))
                {
                    Basin basin = new Basin(rng, basinGrid, basinCoord);
                    drainageSystem = basin.GenerateDrainageSystem(riverGrid, mountainStreamStartHeight);
                    drainageSystems.Add(basinCoord, drainageSystem);
                }
                foreach (QuadraticBezierCurve segment in drainageSystem)
                {
                    if (segment.bounds.X1 >= renderer.frame.X1 && segment.bounds.X2 < renderer.frame.X2 && segment.bounds.Y1 >= renderer.frame.Y1 && segment.bounds.Y2 < renderer.frame.Y2)
                        segment.Plot(renderer.frame, renderer.PlotPoint);
                }
            }
            renderer.DistanceTransform();
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
                    double depressionStrength = 4.0 / basinGrid.cellHeight;
                    (double depressionUpLeft, _) = basinGrid.VoronoiClosest(rng, new FastVec2i(chunkGlobalX, chunkGlobalZ));
                    (double depressionUpRight, _) = basinGrid.VoronoiClosest(rng, new FastVec2i(chunkGlobalX + GlobalConstants.ChunkSize, chunkGlobalZ));
                    (double depressionBotLeft, _) = basinGrid.VoronoiClosest(rng, new FastVec2i(chunkGlobalX, chunkGlobalZ + GlobalConstants.ChunkSize));
                    (double depressionBotRight, _) = basinGrid.VoronoiClosest(rng, new FastVec2i(chunkGlobalX + GlobalConstants.ChunkSize, chunkGlobalZ + GlobalConstants.ChunkSize));
                    depressionUpLeft = Math.Sqrt(depressionUpLeft) * depressionStrength;
                    depressionUpRight = Math.Sqrt(depressionUpRight) * depressionStrength;
                    depressionBotLeft = Math.Sqrt(depressionBotLeft) * depressionStrength;
                    depressionBotRight = Math.Sqrt(depressionBotRight) * depressionStrength;
                    for (int lZ = 0; lZ < GlobalConstants.ChunkSize; lZ++)
                    {
                        for (int lX = 0; lX < GlobalConstants.ChunkSize; lX++)
                        {
                            double depression = GameMath.BiLerp(depressionUpLeft, depressionUpRight, depressionBotLeft, depressionBotRight, lX * chunkBlockDelta, lZ * chunkBlockDelta);
                            (_, _, float distance) = renderer.output[(chunkGlobalZ - renderer.frame.Y1 + lZ) * stride + (chunkGlobalX - renderer.frame.X1 + lX)];
                            double height = Math.Min(TerraGenConfig.seaLevel + slopeProfile.distanceToHeight(distance * GameMath.Clamp(depression, 0.2, 1.0)) + 50.0 * Math.Min(0.0, depression - 0.2), api.WorldManager.MapSizeY - 3);
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
            Basin centralBasin = new Basin(rng, basinGrid, basinCoord);
            Dictionary<FastVec2i, int> climateAtBasinCoord = centralBasin.GenerateClimate(api);
            int climateMapOrigX = renderer.frame.X1 + api.WorldManager.RegionSize - mapRegion.ClimateMap.TopLeftPadding * TerraGenConfig.climateMapScale;
            int climateMapOrigZ = renderer.frame.Y1 + api.WorldManager.RegionSize - mapRegion.ClimateMap.TopLeftPadding * TerraGenConfig.climateMapScale;
            for (int pixelZ = 0; pixelZ < mapRegion.ClimateMap.Size; ++pixelZ)
            {
                for (int pixelX = 0; pixelX < mapRegion.ClimateMap.Size; ++pixelX)
                {
                    (_, closestBasinCoord) = basinGrid.VoronoiClosest(rng, new FastVec2i(
                        climateMapOrigX + pixelX * TerraGenConfig.climateMapScale, climateMapOrigZ + pixelZ * TerraGenConfig.climateMapScale
                    ));
                    mapRegion.ClimateMap.Data[pixelZ * mapRegion.ClimateMap.Size + pixelX] = climateAtBasinCoord[closestBasinCoord];
                }
            }
            int forestMapOrigX = api.WorldManager.RegionSize - mapRegion.ForestMap.TopLeftPadding * TerraGenConfig.forestMapScale;
            int forestMapOrigZ = api.WorldManager.RegionSize - mapRegion.ForestMap.TopLeftPadding * TerraGenConfig.forestMapScale;
            for (int pixelZ = 0; pixelZ < mapRegion.ForestMap.Size; ++pixelZ)
            {
                for (int pixelX = 0; pixelX < mapRegion.ForestMap.Size; ++pixelX)
                {
                    (_, _, float distToRiver) = renderer.output[
                        (forestMapOrigZ + pixelZ * TerraGenConfig.forestMapScale) * stride + (forestMapOrigX + pixelX * TerraGenConfig.forestMapScale)
                    ];
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
                    (double depression, _) = basinGrid.VoronoiClosest(rng, new FastVec2i(
                        geoProvMapOrigZ + pixelZ * TerraGenConfig.geoProvMapScale, geoProvMapOrigX + pixelX * TerraGenConfig.geoProvMapScale
                    ));
                    depression = Math.Sqrt(depression) / basinGrid.cellHeight * 0.5;
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
            blockAccessor.BeginColumn();
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
            ushort YMax = 0;
            ushort YMin = (ushort) api.WorldManager.MapSizeY;
            for (int lZ = 0; lZ < GlobalConstants.ChunkSize; lZ++)
            {
                for (int lX = 0; lX < GlobalConstants.ChunkSize; lX++)
                {
                    ushort height = heightMap[lZ * GlobalConstants.ChunkSize + lX];
                    YMin = Math.Min(YMin, height);
                    YMax = Math.Max(YMax, height);
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
                    int height = heightMap[offset];
                    rainheightmap[offset] = (ushort) Math.Max(height, TerraGenConfig.seaLevel - 1);
                    terrainheightmap[offset] = (ushort) height;
                    ++height;
                    for (int lY = YMin; lY < height; lY++)
                    {
                        chunkBlockData = request.Chunks[lY / GlobalConstants.ChunkSize].Data;
                        chunkBlockData[(lY % GlobalConstants.ChunkSize) * stride + offset] = defaultRockId;
                    }
                    for (int lY = height; lY < TerraGenConfig.seaLevel; lY++)
                    {
                        chunkBlockData = request.Chunks[lY / GlobalConstants.ChunkSize].Data;
                        chunkBlockData.SetFluid((lY % GlobalConstants.ChunkSize) * stride + offset, waterBlockId);
                    }
                }
            }
            mapChunk.YMax = YMax;
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
            IAsset asset = api.Assets.Get("survival:worldgen/slopeprofile.json");
            slopeProfile = new SlopeProfile(asset.ToObject<List<SlopeInterval>>());
        }

        public override void StartServerSide(ICoreServerAPI coreApi)
        {
            api = coreApi;
            IWorldGenHandler handles = api.Event.GetRegisteredWorldGenHandlers("standard");
            var GenTerra = handles.OnChunkColumnGen[(int)EnumWorldGenPass.Terrain].FindIndex(a => a.Method.DeclaringType == typeof(GenTerra));
            handles.OnChunkColumnGen[(int)EnumWorldGenPass.Terrain].RemoveAt(GenTerra);
            api.Event.GetWorldgenBlockAccessor(OnWorldGenBlockAccessor);
            api.Event.InitWorldGenerator(OnInitWorldGen, "standard");
            handles.OnMapRegionGen.Add(OnMapRegionGen);
            handles.OnChunkColumnGen[(int)EnumWorldGenPass.Terrain].Insert(0, OnChunkColumnGen);
        }
    }
}
