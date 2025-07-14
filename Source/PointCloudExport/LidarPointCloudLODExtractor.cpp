#include "LidarPointCloudLODExtractor.h"
#include "UObject/UObjectGlobals.h"
#include "AssetRegistry/AssetRegistryModule.h"

/* --------------- Helper: Recursively subdivide AABB ------------- */
static void SubdivideAndCollect(
    ULidarPointCloud* Cloud,
    const FBox& Box,
    const FVector& CamLoc,
    float                             ThrSq,
    int32                             Depth,
    int32                             MaxDepth,
    TArray<FLidarPointCloudPoint>& Out)
{
    if (Depth > MaxDepth)
    {
        Cloud->GetPointsInBoxAsCopies(Out, Box, /*bVisibleOnly=*/false, false);
        return;
    }
    const FVector Center = Box.GetCenter();
    const float   Radius = Box.GetExtent().Size();
    const float   DistSq = FVector::DistSquared(Center, CamLoc);

    if (DistSq < SMALL_NUMBER || (Radius * Radius / DistSq) > ThrSq)
    {
        // さらに細分化
        const FVector Ext = Box.GetExtent() * 0.5f;
        for (int32 X = -1; X <= 1; X += 2)
        {
            for (int32 Y = -1; Y <= 1; Y += 2)
            {
                for (int32 Z = -1; Z <= 1; Z += 2)
                {
                    const FVector Offset = FVector(X * Ext.X, Y * Ext.Y, Z * Ext.Z);
                    const FVector NewCenter = Center + Offset;
                    FBox Child(NewCenter - Ext, NewCenter + Ext);
                    SubdivideAndCollect(Cloud, Child, CamLoc, ThrSq, Depth + 1, MaxDepth, Out);
                }
            }
        }
    }
    else
    {
        // この Box で十分小さい => 点を取得
        Cloud->GetPointsInBoxAsCopies(Out, Box, /*bVisibleOnly=*/false, false);
    }
}

/* ----------------------- Save Utility --------------------------- */
static ULidarPointCloud* SaveAsAsset(
    ULidarPointCloud* Src,
    const TArray<FLidarPointCloudPoint>& Pts,
    bool                                    bDuplicate)
{
    if (!Src) return nullptr;

    if (!bDuplicate)
    {
        Src->SetData(Pts.GetData(), Pts.Num(), nullptr);
        return Src;
    }

    ULidarPointCloud* NewCloud = DuplicateObject<ULidarPointCloud>(Src, GetTransientPackage());
    NewCloud->SetData(Pts.GetData(), Pts.Num(), nullptr);
    return NewCloud;
}

/* ------------------- Public Blueprint Call ---------------------- */
ULidarPointCloud* ULidarPointCloudLODExtractor::ExtractLODSubset(
    ULidarPointCloud* InCloud,
    const FVector& CameraLoc,
    float             Threshold,
    int32             MaxDepth,
    bool              bDuplicate)
{
    if (!InCloud || Threshold <= 0.f || MaxDepth < 0)
        return nullptr;

    TArray<FLidarPointCloudPoint> Points;
    Points.Reserve(FMath::Clamp<int64>(InCloud->GetNumPoints() / 8, 1000, 5'000'000));

    const float ThrSq = Threshold; // (radius/dist)^2

    const FBox Bounds = InCloud->GetBounds();
    SubdivideAndCollect(InCloud, Bounds, CameraLoc, ThrSq, 0, MaxDepth, Points);

    return SaveAsAsset(InCloud, Points, bDuplicate);
}
