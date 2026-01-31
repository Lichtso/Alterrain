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
        IDictionary<FastVec2i, List<QuadraticBezierCurve>> drainageSystems;
        LCGRandom rng;
        HexGrid riverGrid;
        HexGrid basinGrid;
        HeightMapRenderer renderer;

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
            renderer = new HeightMapRenderer(api.WorldManager.RegionSize * 2 * api.WorldManager.RegionSize * 2);
            renderer.depthSlopeStart = 2.0F;
            renderer.heightSlopeScale = 0.01F;
            renderer.slopeBase = 0.3F;
            renderer.slopeRange = 1.0F;
        }

        private void GenerateFloraMap(HeightMapRenderer renderer, int regionX, int regionZ, IntDataMap2D map, int mapScale, double bandDistance)
        {
            map.TopLeftPadding = 0;
            map.BottomRightPadding = 1;
            map.Size = api.WorldManager.RegionSize / mapScale + map.TopLeftPadding + map.BottomRightPadding;
            map.Data = new int[map.Size * map.Size];
            int flowerMapOrigX = regionX * api.WorldManager.RegionSize - renderer.frame.X1 - map.TopLeftPadding * mapScale;
            int flowerMapOrigZ = regionZ * api.WorldManager.RegionSize - renderer.frame.Y1 - map.TopLeftPadding * mapScale;
            int stride = GlobalConstants.ChunkSize * GlobalConstants.ChunkSize;
            for (int pixelZ = 0; pixelZ < map.Size; ++pixelZ)
            {
                for (int pixelX = 0; pixelX < map.Size; ++pixelX)
                {
                    int rendererX = flowerMapOrigX + pixelX * mapScale;
                    int rendererZ = flowerMapOrigZ + pixelZ * mapScale;
                    (int diffX, int diffZ, _) = renderer.output[rendererZ * stride + rendererX];
                    int riverDepth = renderer.input[(rendererZ + diffZ) * stride + (rendererX + diffX)] - 1;
                    float distToRiver = (float) Math.Sqrt(diffX * diffX + diffZ * diffZ);
                    map.Data[pixelZ * map.Size + pixelX] = (riverDepth == 0) ? 0 : (int) (511.0 * Math.Max(0.0, 1.0 - Math.Abs(distToRiver - bandDistance) / 30.0));
                }
            }
        }

        private void OnMapRegionGen(IMapRegion mapRegion, int regionX, int regionZ, ITreeAttribute chunkGenParams = null)
        {
            const float chunkBlockDelta = 1.0f / GlobalConstants.ChunkSize;
            int regionChunkSize = api.WorldManager.RegionSize / GlobalConstants.ChunkSize;
            renderer.frame = new Rectanglei(
                regionX * api.WorldManager.RegionSize - api.WorldManager.RegionSize / 2,
                regionZ * api.WorldManager.RegionSize - api.WorldManager.RegionSize / 2,
                api.WorldManager.RegionSize * 2,
                api.WorldManager.RegionSize * 2
            );
            int stride = renderer.frame.X2 - renderer.frame.X1;
            FastVec2i regionCenter = new FastVec2i(
                (renderer.frame.X2 + renderer.frame.X1) / 2,
                (renderer.frame.Y2 + renderer.frame.Y1) / 2
            );
            for (int i = 0; i < renderer.output.Length; ++i)
            {
                renderer.input[i] = 0;
                renderer.output[i] = (0, 0, api.WorldManager.MapSizeY);
            }
            FastVec2i basinCoord = basinGrid.CartesianToHex(regionCenter);
            BarycentricTriangle triangle = basinGrid.BarycentricTriangle(rng, regionCenter);
            for (int i = 0; i < 7; ++i)
            {
                basinCoord = triangle.ClosestVertex() + basinGrid.neighborHexOffsets[i];
                List<QuadraticBezierCurve> drainageSystem;
                if (!drainageSystems.TryGetValue(basinCoord, out drainageSystem))
                {
                    Basin basin = new Basin(rng, basinGrid, basinCoord);
                    drainageSystem = basin.GenerateDrainageSystem(riverGrid, 0.88F);
                    drainageSystems.Add(basinCoord, drainageSystem);
                }
                foreach (QuadraticBezierCurve segment in drainageSystem)
                {
                    if (segment.bounds.X1 >= renderer.frame.X1 && segment.bounds.X2 < renderer.frame.X2 && segment.bounds.Y1 >= renderer.frame.Y1 && segment.bounds.Y2 < renderer.frame.Y2)
                        segment.Plot(renderer.frame, renderer.PlotPoint);
                }
            }
            for (int rlZ = 0; rlZ < regionChunkSize * 2; ++rlZ)
            {
                for (int rlX = 0; rlX < regionChunkSize * 2; ++rlX)
                {
                    int chunkGlobalX = renderer.frame.X1 + rlX * GlobalConstants.ChunkSize;
                    int chunkGlobalZ = renderer.frame.Y1 + rlZ * GlobalConstants.ChunkSize;
                    rng.InitPositionSeed(chunkGlobalX, chunkGlobalZ);
                    double elevationUpLeft = rng.NextInt(256) / 255.0;
                    rng.InitPositionSeed(chunkGlobalX + GlobalConstants.ChunkSize, chunkGlobalZ);
                    double elevationUpRight = rng.NextInt(256) / 255.0;
                    rng.InitPositionSeed(chunkGlobalX, chunkGlobalZ + GlobalConstants.ChunkSize);
                    double elevationBotLeft = rng.NextInt(256) / 255.0;
                    rng.InitPositionSeed(chunkGlobalX + GlobalConstants.ChunkSize, chunkGlobalZ + GlobalConstants.ChunkSize);
                    double elevationBotRight = rng.NextInt(256) / 255.0;
                    BarycentricTriangle triangleUpLeft = basinGrid.BarycentricTriangle(rng, new FastVec2i(chunkGlobalX, chunkGlobalZ));
                    BarycentricTriangle triangleUpRight = basinGrid.BarycentricTriangle(rng, new FastVec2i(chunkGlobalX + GlobalConstants.ChunkSize, chunkGlobalZ));
                    BarycentricTriangle triangleBotLeft = basinGrid.BarycentricTriangle(rng, new FastVec2i(chunkGlobalX, chunkGlobalZ + GlobalConstants.ChunkSize));
                    BarycentricTriangle triangleBotRight = basinGrid.BarycentricTriangle(rng, new FastVec2i(chunkGlobalX + GlobalConstants.ChunkSize, chunkGlobalZ + GlobalConstants.ChunkSize));
                    for (int lZ = 0; lZ < GlobalConstants.ChunkSize; lZ++)
                    {
                        for (int lX = 0; lX < GlobalConstants.ChunkSize; lX++)
                        {
                            double elevation = GameMath.BiLerp(elevationUpLeft, elevationUpRight, elevationBotLeft, elevationBotRight, lX * chunkBlockDelta, lZ * chunkBlockDelta);
                            double proximity = GameMath.BiLerp(triangleUpLeft.max, triangleUpRight.max, triangleBotLeft.max, triangleBotRight.max, lX * chunkBlockDelta, lZ * chunkBlockDelta);
                            if (proximity - (float) elevation * 0.02F > 0.89F)
                            {
                                int index = (chunkGlobalZ - renderer.frame.Y1 + lZ) * stride + (chunkGlobalX - renderer.frame.X1 + lX);
                                renderer.input[index] = (byte) (elevation * 5 + 5);
                                renderer.output[index] = (0, 0, (float) TerraGenConfig.seaLevel - renderer.input[index]);
                            }
                        }
                    }
                }
            }
            renderer.DistanceTransform();
            ushort[] heightMap = new ushort[GlobalConstants.ChunkSize * GlobalConstants.ChunkSize];
            for (int rlZ = 0; rlZ < regionChunkSize; ++rlZ)
            {
                for (int rlX = 0; rlX < regionChunkSize; ++rlX)
                {
                    int chunkGlobalX = regionX * api.WorldManager.RegionSize + rlX * GlobalConstants.ChunkSize;
                    int chunkGlobalZ = regionZ * api.WorldManager.RegionSize + rlZ * GlobalConstants.ChunkSize;
                    for (int lZ = 0; lZ < GlobalConstants.ChunkSize; lZ++)
                    {
                        for (int lX = 0; lX < GlobalConstants.ChunkSize; lX++)
                        {
                            (int diffX, int diffZ, float heightFloat) = renderer.output[(chunkGlobalZ - renderer.frame.Y1 + lZ) * stride + (chunkGlobalX - renderer.frame.X1 + lX)];
                            int height = (int) Math.Floor(heightFloat);
                            int index = (chunkGlobalZ - renderer.frame.Y1 + lZ + diffZ) * stride + (chunkGlobalX - renderer.frame.X1 + lX + diffX);
                            int riverDepth = renderer.input[index] - 1;
                            (_, _, float riverHeight) = renderer.output[index];
                            int waterheight = Math.Sqrt(diffX * diffX + diffZ * diffZ) <= riverDepth ? (int) Math.Floor(riverHeight + riverDepth) - height : 0;
                            int terrainheight = GameMath.Clamp(height, 3, api.WorldManager.MapSizeY - 3);
                            heightMap[lZ * GlobalConstants.ChunkSize + lX] = (ushort) ((waterheight << 9) | terrainheight);
                        }
                    }
                    mapRegion.SetModdata<ushort[]>(String.Format("heightMap:{0},{1}", rlX, rlZ), heightMap);
                }
            }
            mapRegion.UpheavelMap.TopLeftPadding = mapRegion.UpheavelMap.BottomRightPadding = 3; // TerraGenConfig.upheavelMapPadding;
            mapRegion.UpheavelMap.Size = api.WorldManager.RegionSize / TerraGenConfig.climateMapScale + mapRegion.UpheavelMap.TopLeftPadding + mapRegion.UpheavelMap.BottomRightPadding;
            mapRegion.UpheavelMap.Data = new int[mapRegion.UpheavelMap.Size * mapRegion.UpheavelMap.Size];
            mapRegion.OceanMap.TopLeftPadding = mapRegion.OceanMap.BottomRightPadding = 5; // TerraGenConfig.oceanMapPadding;
            mapRegion.OceanMap.Size = api.WorldManager.RegionSize / TerraGenConfig.oceanMapScale + mapRegion.OceanMap.TopLeftPadding + mapRegion.OceanMap.BottomRightPadding;
            mapRegion.OceanMap.Data = new int[mapRegion.OceanMap.Size * mapRegion.OceanMap.Size];
            Basin centralBasin = new Basin(rng, basinGrid, basinCoord);
            Dictionary<FastVec2i, int> climateAtBasinCoord = centralBasin.GenerateClimate(api);
            mapRegion.ClimateMap.TopLeftPadding = mapRegion.ClimateMap.BottomRightPadding = 2; // TerraGenConfig.climateMapPadding;
            mapRegion.ClimateMap.Size = api.WorldManager.RegionSize / TerraGenConfig.climateMapScale + mapRegion.ClimateMap.TopLeftPadding + mapRegion.ClimateMap.BottomRightPadding;
            mapRegion.ClimateMap.Data = new int[mapRegion.ClimateMap.Size * mapRegion.ClimateMap.Size];
            int climateMapOrigX = regionX * api.WorldManager.RegionSize - mapRegion.ClimateMap.TopLeftPadding * TerraGenConfig.climateMapScale;
            int climateMapOrigZ = regionZ * api.WorldManager.RegionSize - mapRegion.ClimateMap.TopLeftPadding * TerraGenConfig.climateMapScale;
            for (int pixelZ = 0; pixelZ < mapRegion.ClimateMap.Size; ++pixelZ)
            {
                for (int pixelX = 0; pixelX < mapRegion.ClimateMap.Size; ++pixelX)
                {
                    triangle = basinGrid.BarycentricTriangle(rng, new FastVec2i(
                        climateMapOrigX + pixelX * TerraGenConfig.climateMapScale, climateMapOrigZ + pixelZ * TerraGenConfig.climateMapScale
                    ));
                    mapRegion.ClimateMap.Data[pixelZ * mapRegion.ClimateMap.Size + pixelX] = climateAtBasinCoord[triangle.ClosestVertex()];
                }
            }
            GenerateFloraMap(renderer, regionX, regionZ, mapRegion.FlowerMap, TerraGenConfig.forestMapScale, 30.0);
            GenerateFloraMap(renderer, regionX, regionZ, mapRegion.ShrubMap, TerraGenConfig.shrubMapScale, 50.0);
            GenerateFloraMap(renderer, regionX, regionZ, mapRegion.ForestMap, TerraGenConfig.forestMapScale, 40.0);
            mapRegion.BeachMap.TopLeftPadding = 0; // TerraGenConfig.beachMapPadding;
            mapRegion.BeachMap.BottomRightPadding = 1;
            mapRegion.BeachMap.Size = api.WorldManager.RegionSize / TerraGenConfig.beachMapScale + mapRegion.BeachMap.TopLeftPadding + mapRegion.BeachMap.BottomRightPadding;
            mapRegion.BeachMap.Data = new int[mapRegion.BeachMap.Size * mapRegion.BeachMap.Size];
            mapRegion.GeologicProvinceMap.TopLeftPadding = mapRegion.GeologicProvinceMap.BottomRightPadding = TerraGenConfig.geoProvMapPadding;
            mapRegion.GeologicProvinceMap.Size = api.WorldManager.RegionSize / TerraGenConfig.geoProvMapScale + mapRegion.GeologicProvinceMap.TopLeftPadding + mapRegion.GeologicProvinceMap.BottomRightPadding;
            mapRegion.GeologicProvinceMap.Data = new int[mapRegion.GeologicProvinceMap.Size * mapRegion.GeologicProvinceMap.Size];
            int geoProvMapOrigX = regionX * api.WorldManager.RegionSize - mapRegion.GeologicProvinceMap.TopLeftPadding * TerraGenConfig.geoProvMapScale;
            int geoProvMapOrigZ = regionZ * api.WorldManager.RegionSize - mapRegion.GeologicProvinceMap.TopLeftPadding * TerraGenConfig.geoProvMapScale;
            for (int pixelZ = 0; pixelZ < mapRegion.GeologicProvinceMap.Size; ++pixelZ)
            {
                for (int pixelX = 0; pixelX < mapRegion.GeologicProvinceMap.Size; ++pixelX)
                {
                    triangle = basinGrid.BarycentricTriangle(rng, new FastVec2i(
                        geoProvMapOrigZ + pixelZ * TerraGenConfig.geoProvMapScale, geoProvMapOrigX + pixelX * TerraGenConfig.geoProvMapScale
                    ));
                    mapRegion.GeologicProvinceMap.Data[pixelZ * mapRegion.GeologicProvinceMap.Size + pixelX] = (triangle.max > 0.8) ? 3 : (triangle.max > 0.6) ? 2 : 0;
                }
            }
            mapRegion.LandformMap.TopLeftPadding = mapRegion.LandformMap.BottomRightPadding = TerraGenConfig.landformMapPadding;
            mapRegion.LandformMap.Size = api.WorldManager.RegionSize / TerraGenConfig.landformMapScale + mapRegion.LandformMap.TopLeftPadding + mapRegion.LandformMap.BottomRightPadding;
            mapRegion.LandformMap.Data = new int[mapRegion.LandformMap.Size * mapRegion.LandformMap.Size];
        }

        private void OnChunkColumnGen(IChunkColumnGenerateRequest request)
        {
            blockAccessor.BeginColumn();
            Rectanglei chunkBounds = new Rectanglei(
                request.ChunkX * GlobalConstants.ChunkSize,
                request.ChunkZ * GlobalConstants.ChunkSize,
                GlobalConstants.ChunkSize,
                GlobalConstants.ChunkSize
            );
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
                    ushort height = (ushort) (heightMap[lZ * GlobalConstants.ChunkSize + lX] & 0x1FF);
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
                    int terrainheight = height & 0x1FF;
                    int rainheight = Math.Max(terrainheight + (height >> 9), TerraGenConfig.seaLevel - 1);
                    terrainheightmap[offset] = (ushort) terrainheight;
                    rainheightmap[offset] = (ushort) rainheight;
                    for (int lY = YMin; lY <= terrainheight; lY++)
                    {
                        chunkBlockData = request.Chunks[lY / GlobalConstants.ChunkSize].Data;
                        chunkBlockData[(lY % GlobalConstants.ChunkSize) * stride + offset] = defaultRockId;
                    }
                    for (int lY = terrainheight + 1; lY <= rainheight; lY++)
                    {
                        chunkBlockData = request.Chunks[lY / GlobalConstants.ChunkSize].Data;
                        chunkBlockData.SetFluid((lY % GlobalConstants.ChunkSize) * stride + offset, waterBlockId);
                    }
                    if (rainheight >= TerraGenConfig.seaLevel)
                        blockAccessor.ScheduleBlockUpdate(new BlockPos(chunkBounds.X1 + lX, rainheight, chunkBounds.Y1 + lZ));
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
            return 1.0;
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
            var GenMaps = handles.OnMapRegionGen.FindIndex(a => a.Method.DeclaringType == typeof(GenMaps));
            handles.OnMapRegionGen.RemoveAt(GenMaps);
            var GenTerra = handles.OnChunkColumnGen[(int)EnumWorldGenPass.Terrain].FindIndex(a => a.Method.DeclaringType == typeof(GenTerra));
            handles.OnChunkColumnGen[(int)EnumWorldGenPass.Terrain].RemoveAt(GenTerra);
            var GenTerraPostProcess = handles.OnChunkColumnGen[(int)EnumWorldGenPass.TerrainFeatures].FindIndex(a => a.Method.DeclaringType == typeof(GenTerraPostProcess));
            handles.OnChunkColumnGen[(int)EnumWorldGenPass.TerrainFeatures].RemoveAt(GenTerraPostProcess);
            var GenPonds = handles.OnChunkColumnGen[(int)EnumWorldGenPass.TerrainFeatures].FindIndex(a => a.Method.DeclaringType == typeof(GenPonds));
            handles.OnChunkColumnGen[(int)EnumWorldGenPass.TerrainFeatures].RemoveAt(GenPonds);
            var GenRivulets = handles.OnChunkColumnGen[(int)EnumWorldGenPass.Vegetation].FindIndex(a => a.Method.DeclaringType == typeof(GenRivulets));
            handles.OnChunkColumnGen[(int)EnumWorldGenPass.Vegetation].RemoveAt(GenRivulets);
            api.Event.GetWorldgenBlockAccessor(OnWorldGenBlockAccessor);
            api.Event.InitWorldGenerator(OnInitWorldGen, "standard");
            handles.OnMapRegionGen.Add(OnMapRegionGen);
            handles.OnChunkColumnGen[(int)EnumWorldGenPass.Terrain].Insert(0, OnChunkColumnGen);
        }
    }
}
