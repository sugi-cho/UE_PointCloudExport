#include "LidarPointCloudResampler.h"
#include "Algo/RandomShuffle.h"
#include "Containers/Set.h"

/**************************** Helpers ******************************/
static ULidarPointCloud* WriteBackOrDuplicate(ULidarPointCloud* Src,
    TArray<FLidarPointCloudPoint>& InOut,
    bool bInPlace)
{
    const int64 Num = InOut.Num();
    if (bInPlace)
    {
        Src->SetData(InOut.GetData(), Num, nullptr);
        return Src;
    }
    else
    {
        ULidarPointCloud* NewCloud = DuplicateObject<ULidarPointCloud>(Src, GetTransientPackage());
        NewCloud->SetData(InOut.GetData(), Num, nullptr);
        return NewCloud;
    }
}

/**************** Random density‑proportional sampler **************/
ULidarPointCloud* ULidarPointCloudResampler::ResampleRandom(ULidarPointCloud* InCloud,
    int32 TargetPointCount,
    bool  bInPlace)
{
    if (!InCloud || TargetPointCount <= 0) return nullptr;
    const int64 Original = InCloud->GetNumPoints();
    if (Original <= TargetPointCount) return InCloud;

    TArray<FLidarPointCloudPoint> Points;
    Points.Reserve(Original);
    InCloud->GetPointsAsCopies(Points, /*bVisibleOnly=*/false, 0, Original);

    Algo::RandomShuffle(Points);
    Points.SetNum(TargetPointCount, EAllowShrinking::No);

    return WriteBackOrDuplicate(InCloud, Points, bInPlace);
}

/*************** Spatially uniform voxel‑grid sampler **************/
ULidarPointCloud* ULidarPointCloudResampler::ResampleUniformGrid(ULidarPointCloud* InCloud,
    int32 TargetPointCount,
    bool  bInPlace)
{
    if (!InCloud || TargetPointCount <= 0) return nullptr;
    const int64 Original = InCloud->GetNumPoints();
    if (Original <= TargetPointCount) return InCloud;

    /* 1. Stream‑copy */
    TArray<FLidarPointCloudPoint> Pool;
    Pool.Reserve(Original);
    InCloud->GetPointsAsCopies(Pool, false, 0, Original);

    /* 2. Voxel size */
    const FBox3d Bounds = InCloud->GetBounds();
    const FVector3d Ext = Bounds.Max - Bounds.Min;
    const double CellVol = (Ext.X * Ext.Y * Ext.Z) / static_cast<double>(TargetPointCount);
    const double CellSize = FMath::Pow(CellVol, 1.0 / 3.0);
    if (CellSize <= 0.0) return nullptr;

    /* 3. Uniform selection */
    Algo::RandomShuffle(Pool);
    TArray<FLidarPointCloudPoint> Sampled;
    Sampled.Reserve(TargetPointCount);
    TSet<FIntVector> Occupied;
    Occupied.Reserve(TargetPointCount * 2);

    for (const FLidarPointCloudPoint& Pt : Pool)
    {
        if (Sampled.Num() >= TargetPointCount) break;
        const FVector3d Rel = FVector3d(Pt.Location) - Bounds.Min; // ensure same type
        const FIntVector Key(
            FMath::FloorToInt(Rel.X / CellSize),
            FMath::FloorToInt(Rel.Y / CellSize),
            FMath::FloorToInt(Rel.Z / CellSize));
        if (!Occupied.Contains(Key))
        {
            Occupied.Add(Key);
            Sampled.Add(Pt);
        }
    }

    /* 4. Top‑up if needed */
    if (Sampled.Num() < TargetPointCount)
    {
        for (const FLidarPointCloudPoint& Pt : Pool)
        {
            if (Sampled.Num() >= TargetPointCount) break;
            Sampled.Add(Pt);
        }
    }

    return WriteBackOrDuplicate(InCloud, Sampled, bInPlace);
}
