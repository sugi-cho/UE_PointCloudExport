#include "ExportVisibleLidarPointsLOD.h"

#include "LidarPointCloudComponent.h"
#include "LidarPointCloud.h"
#include "LidarPointCloudOctree.h"
#include "SceneManagement.h"
#include "Camera/CameraComponent.h"
#include "Math/Vector.h"
#include "Math/Box.h"
#include "Math/Plane.h"
#include "HAL/FileManager.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "EngineUtils.h"
#include "Async/Async.h"
#include "Async/ParallelFor.h"
#include "HAL/PlatformTime.h"
#include <cfloat>
#if WITH_EDITOR
#include "AssetRegistry/AssetRegistryModule.h"
#include "UObject/Package.h"
#include "Misc/PackageName.h"
#include "Engine/Texture2D.h"
#endif

#if WITH_EDITOR
// Return a package name that does not conflict with existing assets
static FString MakeUniquePackageName(const FString& FolderPath, const FString& BaseName)
{
    FString PackageName = FolderPath / BaseName;
    FString FileName = FPackageName::LongPackageNameToFilename(PackageName, FPackageName::GetAssetPackageExtension());
    int32 Suffix = 1;
    while (IFileManager::Get().FileExists(*FileName))
    {
        PackageName = FolderPath / FString::Printf(TEXT("%s_%d"), *BaseName, Suffix++);
        FileName = FPackageName::LongPackageNameToFilename(PackageName, FPackageName::GetAssetPackageExtension());
    }
    return PackageName;
}
#endif

// ------------------------------------------------------------
//  ヘルパ: カメラの視錐台を作る
// ------------------------------------------------------------
static void BuildFrustumFromCamera(const UCameraComponent* Camera, FConvexVolume& OutFrustum, float Far)
{
    OutFrustum.Planes.Empty();

    const FVector CamLoc = Camera->GetComponentLocation();
    const FRotator CamRot = Camera->GetComponentRotation();
    const float Near = GNearClippingPlane;
    const float Aspect = Camera->AspectRatio;
    const float FOV = FMath::DegreesToRadians(Camera->FieldOfView);

    const FVector Forward = CamRot.Vector();
    const FVector Right = FRotationMatrix(CamRot).GetScaledAxis(EAxis::Y);
    const FVector Up = FRotationMatrix(CamRot).GetScaledAxis(EAxis::Z);

    const FVector NearCenter = CamLoc + Forward * Near;
    const FVector FarCenter = CamLoc + Forward * Far;

    const float NearHeight = 2.f * FMath::Tan(FOV / 2.f) * Near;
    const float NearWidth = NearHeight * Aspect;
    const float FarHeight = 2.f * FMath::Tan(FOV / 2.f) * Far;
    const float FarWidth = FarHeight * Aspect;

    // Near plane corners
    FVector NTl = NearCenter + (Up * (NearHeight / 2)) - (Right * (NearWidth / 2));
    FVector NTr = NearCenter + (Up * (NearHeight / 2)) + (Right * (NearWidth / 2));
    FVector NBl = NearCenter - (Up * (NearHeight / 2)) - (Right * (NearWidth / 2));
    FVector NBr = NearCenter - (Up * (NearHeight / 2)) + (Right * (NearWidth / 2));
    // Far plane corners
    FVector FTl = FarCenter + (Up * (FarHeight / 2)) - (Right * (FarWidth / 2));
    FVector FTr = FarCenter + (Up * (FarHeight / 2)) + (Right * (FarWidth / 2));
    FVector FBl = FarCenter - (Up * (FarHeight / 2)) - (Right * (FarWidth / 2));
    FVector FBr = FarCenter - (Up * (FarHeight / 2)) + (Right * (FarWidth / 2));

    // 6 planes
    OutFrustum.Planes.Add(FPlane(NTl, NTr, NBr)); // Near
    OutFrustum.Planes.Add(FPlane(FTr, FTl, FBl)); // Far
    OutFrustum.Planes.Add(FPlane(FTl, NTl, NBl)); // Left
    OutFrustum.Planes.Add(FPlane(NTr, FTr, FBr)); // Right
    OutFrustum.Planes.Add(FPlane(NTl, FTl, FTr)); // Top
    OutFrustum.Planes.Add(FPlane(NBl, NBr, FBr)); // Bottom

    OutFrustum.Init();
}

struct FPointRec
{
    FVector WorldPos;
    FVector LocalPos;
    float   Distance = 0.f;
    FColor  Color;
};

// -----------------------------------------------------------------------------
//  Octree LOD support types
// -----------------------------------------------------------------------------
struct FPointRec_Octree
{
    FVector WorldPos;
    FVector LocalPos;
    float   DistanceSq = 0.f;
    FColor  Color;
};

static FORCEINLINE uint32 ComputeAllowedDepth(float Distance, float NearR, float FarR, int32 NearD, int32 FarD)
{
    if (Distance <= NearR)         { return (uint32)NearD; }
    if (Distance >= FarR)          { return (uint32)FarD;  }
    const float T = (Distance - NearR) / (FarR - NearR);
    return (uint32)FMath::RoundToInt(FMath::Lerp((float)NearD, (float)FarD, T));
}

template <typename Predicate>
static void ParallelBitonicSort(TArray<FPointRec>& Array, Predicate Pred)
{
    const int32 N = Array.Num();
    int32 Pow2 = 1;
    while (Pow2 < N)
    {
        Pow2 <<= 1;
    }

    if (Pow2 > N)
    {
        FPointRec Sentinel;
        Sentinel.Distance = FLT_MAX;
        Array.AddDefaulted(Pow2 - N);
        for (int32 i = N; i < Pow2; ++i)
        {
            Array[i] = Sentinel;
        }
    }

    for (int32 k = 2; k <= Pow2; k <<= 1)
    {
        for (int32 j = k >> 1; j > 0; j >>= 1)
        {
            ParallelFor(Pow2, [&](int32 i)
            {
                int32 ixj = i ^ j;
                if (ixj > i)
                {
                    const bool Asc = (i & k) == 0;
                    const bool SwapNeeded = Asc ? Pred(Array[ixj], Array[i]) : Pred(Array[i], Array[ixj]);
                    if (SwapNeeded)
                    {
                        Swap(Array[i], Array[ixj]);
                    }
                }
            });
        }
    }

    if (Pow2 > N)
    {
        Array.SetNum(N);
    }
}

template <typename Predicate>
static void ParallelBitonicSort(TArray<FPointRec_Octree>& Array, Predicate Pred)
{
    const int32 N = Array.Num();
    int32 Pow2 = 1;
    while (Pow2 < N)
    {
        Pow2 <<= 1;
    }

    if (Pow2 > N)
    {
        FPointRec_Octree Sentinel;
        Sentinel.DistanceSq = FLT_MAX;
        Array.AddDefaulted(Pow2 - N);
        for (int32 i = N; i < Pow2; ++i)
        {
            Array[i] = Sentinel;
        }
    }

    for (int32 k = 2; k <= Pow2; k <<= 1)
    {
        for (int32 j = k >> 1; j > 0; j >>= 1)
        {
            ParallelFor(Pow2, [&](int32 i)
            {
                int32 ixj = i ^ j;
                if (ixj > i)
                {
                    const bool Asc = (i & k) == 0;
                    const bool SwapNeeded = Asc ? Pred(Array[ixj], Array[i]) : Pred(Array[i], Array[ixj]);
                    if (SwapNeeded)
                    {
                        Swap(Array[i], Array[ixj]);
                    }
                }
            });
        }
    }

    if (Pow2 > N)
    {
        Array.SetNum(N);
    }
}


// ------------------------------------------------------------
//  メイン関数: 点群エクスポート
// ------------------------------------------------------------
bool UExportVisibleLidarPointsLOD::ExportVisiblePointsLOD(
    const TArray<ALidarPointCloudActor*>& PointCloudActors,
    UCameraComponent* Camera,
    const FString& AbsoluteFilePath,
    float FrustumFar,
    float NearFullResRadius,
    float MidSkipRadius,
    float FarSkipRadius,
    int32 SkipFactorMid,
    int32 SkipFactorFar,
    bool bWorldSpace,
    bool bExportTexture,
    int32 MaxPointCount)
{
    if (PointCloudActors.Num() == 0 || !Camera)
    {
        UE_LOG(LogTemp, Warning, TEXT("ExportVisiblePointsLOD: Invalid input."));
        return false;
    }

    if (AbsoluteFilePath.IsEmpty())
    {
        UE_LOG(LogTemp, Warning, TEXT("ExportVisiblePointsLOD: AbsoluteFilePath is empty."));
        return false;
    }
    if (NearFullResRadius <= 0.f || MidSkipRadius <= 0.f || FarSkipRadius <= 0.f)
    {
        UE_LOG(LogTemp, Warning, TEXT("ExportVisiblePointsLOD: Radius values must be > 0."));
        return false;
    }
    if (!(NearFullResRadius < MidSkipRadius && MidSkipRadius < FarSkipRadius))
    {
        UE_LOG(LogTemp, Warning, TEXT("ExportVisiblePointsLOD: Radius values are inconsistent."));
        return false;
    }
    if (SkipFactorMid < 1 || SkipFactorFar < 1)
    {
        UE_LOG(LogTemp, Warning, TEXT("ExportVisiblePointsLOD: Skip factors must be >= 1."));
        return false;
    }
    if (SkipFactorFar < SkipFactorMid)
    {
        UE_LOG(LogTemp, Warning, TEXT("ExportVisiblePointsLOD: SkipFactorFar should be >= SkipFactorMid."));
        return false;
    }

    const double StartTime = FPlatformTime::Seconds();

    // 1) 視錐台フィルタリング
    FConvexVolume WorldFrustum;
    BuildFrustumFromCamera(Camera, WorldFrustum, FrustumFar);

    TArray<FPointRec> AllPoints;
    ULidarPointCloud* FirstCloud = nullptr;

    const bool bUseLimit = MaxPointCount > 0;

    const FVector CamLoc = Camera->GetComponentLocation();

    TArray<TFuture<TArray<FPointRec>>> Futures;
    const double GatherStart = FPlatformTime::Seconds();

    for (ALidarPointCloudActor* Actor : PointCloudActors)
    {
        if (!Actor) continue;
        Futures.Add(Async(EAsyncExecution::ThreadPool,
            [Actor, &WorldFrustum, CamLoc,
             NearFullResRadius, MidSkipRadius, FarSkipRadius, SkipFactorMid, SkipFactorFar]()
        {
            TArray<FPointRec> LocalPoints;
            ULidarPointCloudComponent* Comp = Actor->GetPointCloudComponent();
            ULidarPointCloud* Cloud = Comp ? Comp->GetPointCloud() : nullptr;
            if (!Cloud) return LocalPoints;

            FConvexVolume LocalFrustum = WorldFrustum;
            const FMatrix WorldToCloud = Comp->GetComponentTransform().ToMatrixWithScale().Inverse();
            const FVector LocationOffset = Cloud->LocationOffset;
            for (FPlane& Plane : LocalFrustum.Planes)
            {
                Plane = Plane.TransformBy(WorldToCloud);
                Plane = Plane.TransformBy(FTranslationMatrix(-LocationOffset));
                Plane.Normalize();
            }
            LocalFrustum.Init();

            TArray64<FLidarPointCloudPoint*> VisiblePts;
            Cloud->GetPointsInConvexVolume(VisiblePts, LocalFrustum, /*bVisibleOnly=*/true);

            const FTransform& CloudToWorld = Comp->GetComponentTransform();
            for (int32 Index = 0; Index < VisiblePts.Num(); ++Index)
            {
                const auto* P = VisiblePts[Index];
                const FVector WorldPos = CloudToWorld.TransformPosition(FVector(P->Location) + LocationOffset);
                const float Dist = FVector::Dist(WorldPos, CamLoc);

                float Skip = 1.f;
                if (Dist > FarSkipRadius)
                {
                    Skip = (float)SkipFactorFar;
                }
                else if (Dist > MidSkipRadius)
                {
                    const float t = (Dist - MidSkipRadius) / (FarSkipRadius - MidSkipRadius);
                    Skip = FMath::Lerp((float)SkipFactorMid, (float)SkipFactorFar, t);
                }
                else if (Dist > NearFullResRadius)
                {
                    const float t = (Dist - NearFullResRadius) / (MidSkipRadius - NearFullResRadius);
                    Skip = FMath::Lerp(1.f, (float)SkipFactorMid, t);
                }

                if (FMath::Fmod((float)(Index + 1), Skip) >= 1.f)
                {
                    continue;
                }

                FPointRec Rec;
                Rec.WorldPos = WorldPos;
                Rec.LocalPos = FVector(P->Location) + LocationOffset;
                Rec.Distance = Dist;
                Rec.Color = P->Color;
                LocalPoints.Add(Rec);
            }
            return LocalPoints;
        }));

        ULidarPointCloudComponent* CompCheck = Actor->GetPointCloudComponent();
        ULidarPointCloud* CloudCheck = CompCheck ? CompCheck->GetPointCloud() : nullptr;
        if (!FirstCloud && CloudCheck)
        {
            FirstCloud = CloudCheck;
        }
    }

    for (TFuture<TArray<FPointRec>>& Future : Futures)
    {
        TArray<FPointRec> Points = Future.Get();
        AllPoints.Append(MoveTemp(Points));
    }

    const double GatherTime = FPlatformTime::Seconds() - GatherStart;
    UE_LOG(LogTemp, Log, TEXT("ExportVisiblePointsLOD: Gathered %d points in %.2f sec"), AllPoints.Num(), GatherTime);

    if (AllPoints.Num() == 0)
    {
        UE_LOG(LogTemp, Warning, TEXT("ExportVisiblePointsLOD: No points in frustum."));
        return false;
    }

    const double SortStart = FPlatformTime::Seconds();
    if (bUseLimit && AllPoints.Num() > MaxPointCount)
    {
        ParallelBitonicSort(AllPoints, [](const FPointRec& A, const FPointRec& B)
        {
            return A.Distance < B.Distance;
        });
        AllPoints.SetNum(MaxPointCount);
    }
    const double SortTime = FPlatformTime::Seconds() - SortStart;
    UE_LOG(LogTemp, Log, TEXT("ExportVisiblePointsLOD: Sort/Limit took %.2f sec"), SortTime);

    const int32 ReserveCount = AllPoints.Num();
    TArray<FString> Lines;
    Lines.Reserve(ReserveCount);
#if WITH_EDITOR
    TArray<FLinearColor> PosBuffer;
    TArray<FColor> ColorBuffer;
    if (bExportTexture)
    {
        PosBuffer.Reserve(ReserveCount);
        ColorBuffer.Reserve(ReserveCount);
    }
#endif

    const double FormatStart = FPlatformTime::Seconds();

    for (int32 Index = 0; Index < AllPoints.Num(); ++Index)
    {
        const FPointRec& Rec = AllPoints[Index];

        const FVector UsePos = (bWorldSpace ? Rec.WorldPos : Rec.LocalPos);
        Lines.Add(FString::Printf(TEXT("%.8f %.8f %.8f %d %d %d %d"),
            UsePos.X * 0.01f, -UsePos.Y * 0.01f, UsePos.Z * 0.01f,
            Rec.Color.A, Rec.Color.R, Rec.Color.G, Rec.Color.B));
#if WITH_EDITOR
        if (bExportTexture)
        {
            PosBuffer.Add(FLinearColor(UsePos.X, UsePos.Y, UsePos.Z, 1.f));
            // Preserve the original alpha channel which stores point intensity
            ColorBuffer.Add(FColor(Rec.Color.R, Rec.Color.G, Rec.Color.B, Rec.Color.A));
        }
#endif

    }

    const double FormatTime = FPlatformTime::Seconds() - FormatStart;
    UE_LOG(LogTemp, Log, TEXT("ExportVisiblePointsLOD: Format output took %.2f sec"), FormatTime);

    if (Lines.Num() == 0)
    {
        UE_LOG(LogTemp, Warning, TEXT("ExportVisiblePointsLOD: All points skipped by LOD."));
        return false;
    }

    const double WriteStart = FPlatformTime::Seconds();

    const FString DirectoryPath = FPaths::GetPath(AbsoluteFilePath);
    if (!DirectoryPath.IsEmpty() && !IFileManager::Get().DirectoryExists(*DirectoryPath))
    {
        if (!IFileManager::Get().MakeDirectory(*DirectoryPath, true))
        {
            UE_LOG(LogTemp, Error,
                TEXT("ExportVisiblePointsLOD: Failed to create directory %s"), *DirectoryPath);
            return false;
        }
    }

    const FString Joined = FString::Join(Lines, TEXT("\n")) + TEXT("\n");
    if (!FFileHelper::SaveStringToFile(
        Joined, *AbsoluteFilePath,
        FFileHelper::EEncodingOptions::AutoDetect,
        &IFileManager::Get(), FILEWRITE_AllowRead))
    {
        UE_LOG(LogTemp, Error,
            TEXT("ExportVisiblePointsLOD: Failed to save file %s"), *AbsoluteFilePath);
        return false;
    }

#if WITH_EDITOR
    const int32 PointCount = Lines.Num();
    if (bExportTexture && PosBuffer.Num() == PointCount && ColorBuffer.Num() == PointCount && FirstCloud)
    {
        const int32 TexDim = FMath::CeilToInt(FMath::Sqrt((float)PointCount));
        TArray<FFloat16Color> PosPixels;
        TArray<FColor> ColorPixels;
        PosPixels.Init(FFloat16Color(FLinearColor::Transparent), TexDim * TexDim);
        ColorPixels.Init(FColor(0, 0, 0, 0), TexDim * TexDim);
        for (int32 i = 0; i < PointCount; ++i)
        {
            const int32 X = i % TexDim;
            const int32 Y = i / TexDim;
            const int32 Idx = Y * TexDim + X;
            PosPixels[Idx] = FFloat16Color(PosBuffer[i]);
            ColorPixels[Idx] = ColorBuffer[i];
        }
        const FString CloudPackage = FirstCloud->GetOutermost()->GetName();
        const FString FolderPath = FPackageName::GetLongPackagePath(CloudPackage);
        const FString BaseName = FirstCloud->GetName();

        const FString PosTexPackageName = MakeUniquePackageName(FolderPath, BaseName + TEXT("_PosTex"));
        UPackage* PosPackage = CreatePackage(*PosTexPackageName);
        UTexture2D* PosTex = NewObject<UTexture2D>(PosPackage, *FPackageName::GetShortName(PosTexPackageName), RF_Public | RF_Standalone);
        PosTex->Source.Init(TexDim, TexDim, 1, 1, TSF_RGBA16F, (const uint8*)PosPixels.GetData());
        PosTex->CompressionSettings = TC_HDR;
        PosTex->SRGB = false;
        PosTex->UpdateResource();
        FAssetRegistryModule::AssetCreated(PosTex);
        PosPackage->MarkPackageDirty();
        const FString PosFileName = FPackageName::LongPackageNameToFilename(PosTexPackageName, FPackageName::GetAssetPackageExtension());
        UPackage::SavePackage(PosPackage, PosTex, EObjectFlags::RF_Public | RF_Standalone, *PosFileName);

        const FString ColorTexPackageName = MakeUniquePackageName(FolderPath, BaseName + TEXT("_ColorTex"));
        UPackage* ColorPackage = CreatePackage(*ColorTexPackageName);
        UTexture2D* ColorTex = NewObject<UTexture2D>(ColorPackage, *FPackageName::GetShortName(ColorTexPackageName), RF_Public | RF_Standalone);
        ColorTex->Source.Init(TexDim, TexDim, 1, 1, TSF_BGRA8, (const uint8*)ColorPixels.GetData());
        ColorTex->CompressionSettings = TC_Default;
        ColorTex->SRGB = true;
        ColorTex->UpdateResource();
        FAssetRegistryModule::AssetCreated(ColorTex);
        ColorPackage->MarkPackageDirty();
        const FString ColorFileName = FPackageName::LongPackageNameToFilename(ColorTexPackageName, FPackageName::GetAssetPackageExtension());
        UPackage::SavePackage(ColorPackage, ColorTex, EObjectFlags::RF_Public | RF_Standalone, *ColorFileName);
    }
#endif

    const double WriteTime = FPlatformTime::Seconds() - WriteStart;
    const double TotalTime = FPlatformTime::Seconds() - StartTime;
    UE_LOG(LogTemp, Log, TEXT("ExportVisiblePointsLOD: Write file/textures took %.2f sec"), WriteTime);
    UE_LOG(LogTemp, Log,
        TEXT("ExportVisiblePointsLOD: Wrote %d points → %s (%.2f sec total)"),
        Lines.Num(), *AbsoluteFilePath, TotalTime);
    return true;
}

// ------------------------------------------------------------
//  指定カメラから見える LidarPointCloudActor を取得
// ------------------------------------------------------------
TArray<ALidarPointCloudActor*> UExportVisibleLidarPointsLOD::GetVisibleLidarActors(
    UCameraComponent* Camera,
    float FrustumFar,
    float NearFullResRadius,
    float MidSkipRadius,
    float FarSkipRadius,
    int32 SkipFactorMid,
    int32 SkipFactorFar)
{
    TArray<ALidarPointCloudActor*> Result;
    if (!Camera)
    {
        return Result;
    }

    UWorld* World = Camera->GetWorld();
    if (!World)
    {
        return Result;
    }

    const double StartTime = FPlatformTime::Seconds();

    FConvexVolume WorldFrustum;
    BuildFrustumFromCamera(Camera, WorldFrustum, FrustumFar);

    // Statistics: total points across visible actors and the estimated count
    int64 TotalPointCount = 0;
    int64 PredictedPointCount = 0;
    const FVector CamLoc = Camera->GetComponentLocation();

    TArray<ALidarPointCloudActor*> AllActors;
    for (TActorIterator<ALidarPointCloudActor> It(World); It; ++It)
    {
        if (*It)
        {
            AllActors.Add(*It);
        }
    }

    struct FThreadResult
    {
        ALidarPointCloudActor* Actor = nullptr;
        int64 TotalCount = 0;
        int64 LODCount = 0;
    };

    TArray<TFuture<FThreadResult>> Futures;
    Futures.Reserve(AllActors.Num());

    for (ALidarPointCloudActor* Actor : AllActors)
    {
        Futures.Add(Async(EAsyncExecution::ThreadPool, [Actor, &WorldFrustum, CamLoc, NearFullResRadius, MidSkipRadius, FarSkipRadius, SkipFactorMid, SkipFactorFar]()
        {
            FThreadResult Res;
            if (!Actor) return Res;

            ULidarPointCloudComponent* Comp = Actor->GetPointCloudComponent();
            if (!Comp) return Res;

            // Use the actor's component bounds instead of the private CalcBounds API
            FBox BoundsBox = Actor->GetComponentsBoundingBox(true);
            FBoxSphereBounds Bounds(BoundsBox);
            if (!WorldFrustum.IntersectBox(Bounds.Origin, Bounds.BoxExtent))
            {
                return Res;
            }

            Res.Actor = Actor;

            ULidarPointCloud* Cloud = Comp->GetPointCloud();
            if (Cloud)
            {
                FConvexVolume LocalFrustum = WorldFrustum;
                const FMatrix WorldToCloud = Comp->GetComponentTransform().ToMatrixWithScale().Inverse();
                const FVector LocationOffset = Cloud->LocationOffset;
                for (FPlane& Plane : LocalFrustum.Planes)
                {
                    Plane = Plane.TransformBy(WorldToCloud);
                    Plane = Plane.TransformBy(FTranslationMatrix(-LocationOffset));
                    Plane.Normalize();
                }
                LocalFrustum.Init();

                TArray64<FLidarPointCloudPoint*> Points;
                Cloud->GetPointsInConvexVolume(Points, LocalFrustum, /*bVisibleOnly=*/true);
                Res.TotalCount = Points.Num();

                const FTransform& CloudToWorld = Comp->GetComponentTransform();
                for (int64 Index = 0; Index < Res.TotalCount; ++Index)
                {
                    const FLidarPointCloudPoint* P = Points[Index];
                    const FVector WorldPos = CloudToWorld.TransformPosition(FVector(P->Location) + LocationOffset);
                    const float Dist = FVector::Dist(WorldPos, CamLoc);

                    float Skip = 1.f;
                    if (Dist > FarSkipRadius)
                    {
                        Skip = (float)SkipFactorFar;
                    }
                    else if (Dist > MidSkipRadius)
                    {
                        const float t = (Dist - MidSkipRadius) / (FarSkipRadius - MidSkipRadius);
                        Skip = FMath::Lerp((float)SkipFactorMid, (float)SkipFactorFar, t);
                    }
                    else if (Dist > NearFullResRadius)
                    {
                        const float t = (Dist - NearFullResRadius) / (MidSkipRadius - NearFullResRadius);
                        Skip = FMath::Lerp(1.f, (float)SkipFactorMid, t);
                    }

                    if (FMath::Fmod((float)(Index + 1), Skip) < 1.f)
                    {
                        ++Res.LODCount;
                    }
                }
            }

            return Res;
        }));
    }

    for (TFuture<FThreadResult>& Future : Futures)
    {
        FThreadResult Res = Future.Get();
        if (Res.Actor)
        {
            Result.Add(Res.Actor);
            TotalPointCount += Res.TotalCount;
            PredictedPointCount += Res.LODCount;
        }
    }

    const double TotalTime = FPlatformTime::Seconds() - StartTime;
    UE_LOG(LogTemp, Log, TEXT("GetVisibleLidarActors: Total Points = %lld, Estimated LOD Points = %lld (%.2f sec)"),
        TotalPointCount, PredictedPointCount, TotalTime);

    return Result;
}

// ------------------------------------------------------------
//  LidarPointCloud からテクスチャを生成して保存
// ------------------------------------------------------------
bool UExportVisibleLidarPointsLOD::SavePointCloudTextures(ULidarPointCloud* PointCloud)
{
#if WITH_EDITOR
    if (!PointCloud)
    {
        UE_LOG(LogTemp, Warning, TEXT("SavePointCloudTextures: Invalid PointCloud"));
        return false;
    }

    TArray64<FLidarPointCloudPoint*> Points;
    PointCloud->GetPoints(Points);

    if (Points.Num() == 0)
    {
        UE_LOG(LogTemp, Warning, TEXT("SavePointCloudTextures: No points in asset"));
        return false;
    }

    const double StartTime = FPlatformTime::Seconds();

    const int32 PointCount = Points.Num();
    const int32 TexDim = FMath::CeilToInt(FMath::Sqrt((float)PointCount));

    TArray<FFloat16Color> PosPixels;
    TArray<FColor> ColorPixels;
    PosPixels.Init(FFloat16Color(FLinearColor::Transparent), TexDim * TexDim);
    ColorPixels.Init(FColor(0, 0, 0, 0), TexDim * TexDim);

    for (int32 i = 0; i < PointCount; ++i)
    {
        const int32 X = i % TexDim;
        const int32 Y = i / TexDim;
        const int32 Idx = Y * TexDim + X;

        const FLidarPointCloudPoint* P = Points[i];
        FVector Pos = FVector(P->Location);
        PosPixels[Idx] = FFloat16Color(FLinearColor(Pos.X, Pos.Y, Pos.Z, 1.f));
        ColorPixels[Idx] = FColor(P->Color.R, P->Color.G, P->Color.B, P->Color.A);
    }

    const FString CloudPackage = PointCloud->GetOutermost()->GetName();
    const FString FolderPath = FPackageName::GetLongPackagePath(CloudPackage);
    const FString BaseName = PointCloud->GetName();

    const FString PosTexPackageName = MakeUniquePackageName(FolderPath, BaseName + TEXT("_PosTex"));
    UPackage* PosPackage = CreatePackage(*PosTexPackageName);
    UTexture2D* PosTex = NewObject<UTexture2D>(PosPackage, *FPackageName::GetShortName(PosTexPackageName), RF_Public | RF_Standalone);
    PosTex->Source.Init(TexDim, TexDim, 1, 1, TSF_RGBA16F, (const uint8*)PosPixels.GetData());
    PosTex->CompressionSettings = TC_HDR;
    PosTex->SRGB = false;
    PosTex->UpdateResource();
    FAssetRegistryModule::AssetCreated(PosTex);
    PosPackage->MarkPackageDirty();
    const FString PosFileName = FPackageName::LongPackageNameToFilename(PosTexPackageName, FPackageName::GetAssetPackageExtension());
    UPackage::SavePackage(PosPackage, PosTex, EObjectFlags::RF_Public | RF_Standalone, *PosFileName);

    const FString ColorTexPackageName = MakeUniquePackageName(FolderPath, BaseName + TEXT("_ColorTex"));
    UPackage* ColorPackage = CreatePackage(*ColorTexPackageName);
    UTexture2D* ColorTex = NewObject<UTexture2D>(ColorPackage, *FPackageName::GetShortName(ColorTexPackageName), RF_Public | RF_Standalone);
    ColorTex->Source.Init(TexDim, TexDim, 1, 1, TSF_BGRA8, (const uint8*)ColorPixels.GetData());
    ColorTex->CompressionSettings = TC_Default;
    ColorTex->SRGB = true;
    ColorTex->UpdateResource();
    FAssetRegistryModule::AssetCreated(ColorTex);
    ColorPackage->MarkPackageDirty();
    const FString ColorFileName = FPackageName::LongPackageNameToFilename(ColorTexPackageName, FPackageName::GetAssetPackageExtension());
    UPackage::SavePackage(ColorPackage, ColorTex, EObjectFlags::RF_Public | RF_Standalone, *ColorFileName);

    const double TotalTime = FPlatformTime::Seconds() - StartTime;
    UE_LOG(LogTemp, Log, TEXT("SavePointCloudTextures: Saved %d points to %s (%.2f sec)"), PointCount, *FolderPath, TotalTime);
    return true;
#else
    return false;
#endif
}

// -----------------------------------------------------------------------------
//  Octree-based LOD sampling and export
// -----------------------------------------------------------------------------
bool UExportVisibleLidarPointsLOD::ExportVisiblePointsOctreeLOD(
    const TArray<ALidarPointCloudActor*>& PointCloudActors,
    UCameraComponent*                     Camera,
    const FString&                        AbsoluteFilePath,
    float                                 FrustumFar,
    float                                 NearDepthRadius,
    float                                 FarDepthRadius,
    int32                                 NearDepth,
    int32                                 FarDepth,
    bool                                  bWorldSpace,
    int32                                 MaxPointCount)
{
    if (PointCloudActors.Num() == 0 || !Camera)
    {
        UE_LOG(LogTemp, Warning, TEXT("ExportVisiblePointsOctreeLOD: Invalid input."));
        return false;
    }

    if (AbsoluteFilePath.IsEmpty())
    {
        UE_LOG(LogTemp, Warning, TEXT("ExportVisiblePointsOctreeLOD: AbsoluteFilePath is empty."));
        return false;
    }

    if (!(NearDepthRadius < FarDepthRadius) || NearDepthRadius <= 0.f)
    {
        UE_LOG(LogTemp, Warning, TEXT("ExportVisiblePointsOctreeLOD: Radius values invalid."));
        return false;
    }

    if (!(NearDepth >= 0 && FarDepth >= 0 && NearDepth <= FarDepth))
    {
        UE_LOG(LogTemp, Warning, TEXT("ExportVisiblePointsOctreeLOD: Depth parameters invalid."));
        return false;
    }

    const double StartTime = FPlatformTime::Seconds();

    FConvexVolume WorldFrustum;
    BuildFrustumFromCamera(Camera, WorldFrustum, FrustumFar);

    const FVector CamLoc = Camera->GetComponentLocation();
    const bool bUseLimit = MaxPointCount > 0;

    TArray<FPointRec_Octree> AllPoints;

    TArray<TFuture<TArray<FPointRec_Octree>>> Futures;
    Futures.Reserve(PointCloudActors.Num());

    for (ALidarPointCloudActor* Actor : PointCloudActors)
    {
        Futures.Add(Async(EAsyncExecution::ThreadPool, [Actor, WorldFrustum, CamLoc,
                                                        NearDepthRadius, FarDepthRadius,
                                                        NearDepth,       FarDepth]() -> TArray<FPointRec_Octree>
        {
            TArray<FPointRec_Octree> LocalOutput;
            if (!Actor) return LocalOutput;

            ULidarPointCloudComponent* Comp = Actor->GetPointCloudComponent();
            ULidarPointCloud*          Cloud = Comp ? Comp->GetPointCloud() : nullptr;
            if (!Cloud) return LocalOutput;

            FConvexVolume LocalFrustum = WorldFrustum;
            const FMatrix WorldToLocal = Comp->GetComponentTransform().ToMatrixWithScale().Inverse();
            const FVector Offset       = Cloud->LocationOffset;
            for (FPlane& Plane : LocalFrustum.Planes)
            {
                Plane = Plane.TransformBy(WorldToLocal);
                Plane = Plane.TransformBy(FTranslationMatrix(-Offset));
                Plane.Normalize();
            }
            LocalFrustum.Init();

            FLidarPointCloudOctree& Octree = Cloud->GetOctree();
            const FTransform& LocalToWorld = Comp->GetComponentTransform();
            LocalOutput.Reserve(1024);

            FLidarPointCloudTraversalOctree Traversal(&Octree, LocalToWorld);
            Traversal.Traverse(true,
                [&](FLidarPointCloudTraversalOctree::FNode& Node, bool bNodeCompletelyInside)
            {
                const FVector NodeCenterWS = LocalToWorld.TransformPosition(Node.Bounds.GetCenter() + Offset);
                const float   Dist        = FVector::Dist(NodeCenterWS, CamLoc);
                const uint32  DepthLimit  = ComputeAllowedDepth(Dist, NearDepthRadius, FarDepthRadius, NearDepth, FarDepth);

                if (Node.Node->GetDepth() >= DepthLimit)
                {
                    const FLidarPointCloudPoint* Pts = Node.Node->GetData();
                    const uint32 Num                 = Node.Node->GetNumPoints();
                    for (uint32 idx = 0; idx < Num; ++idx)
                    {
                        const FLidarPointCloudPoint& Pt = Pts[idx];
                        FPointRec_Octree Rec;
                        Rec.LocalPos   = FVector(Pt.Location) + Offset;
                        Rec.WorldPos   = LocalToWorld.TransformPosition(Rec.LocalPos);
                        Rec.DistanceSq = FVector::DistSquared(Rec.WorldPos, CamLoc);
                        Rec.Color      = Pt.Color;
                        LocalOutput.Add(Rec);
                    }
                    return false;
                }
                return true;
            },
            &LocalFrustum);

            return LocalOutput;
        }));
    }

    for (TFuture<TArray<FPointRec_Octree>>& Future : Futures)
    {
        AllPoints.Append(Future.Get());
    }

    if (AllPoints.Num() == 0)
    {
        UE_LOG(LogTemp, Warning, TEXT("ExportVisiblePointsOctreeLOD: No points collected."));
        return false;
    }

    if (bUseLimit && AllPoints.Num() > MaxPointCount)
    {
        ParallelBitonicSort(AllPoints, [](const FPointRec_Octree& A, const FPointRec_Octree& B)
        {
            return A.DistanceSq < B.DistanceSq;
        });
        AllPoints.SetNum(MaxPointCount);
    }

    TArray<FString> Lines;
    Lines.Reserve(AllPoints.Num());
    for (const FPointRec_Octree& Rec : AllPoints)
    {
        const FVector& Pos = bWorldSpace ? Rec.WorldPos : Rec.LocalPos;
        Lines.Add(FString::Printf(TEXT("%.8f %.8f %.8f %d %d %d %d"),
            Pos.X * 0.01f, -Pos.Y * 0.01f, Pos.Z * 0.01f,
            Rec.Color.A, Rec.Color.R, Rec.Color.G, Rec.Color.B));
    }

    const FString DirPath = FPaths::GetPath(AbsoluteFilePath);
    if (!DirPath.IsEmpty() && !IFileManager::Get().DirectoryExists(*DirPath))
    {
        if (!IFileManager::Get().MakeDirectory(*DirPath, true))
        {
            UE_LOG(LogTemp, Error, TEXT("ExportVisiblePointsOctreeLOD: Failed to create dir %s"), *DirPath);
            return false;
        }
    }

    const FString Joined = FString::Join(Lines, TEXT("\n")) + TEXT("\n");
    if (!FFileHelper::SaveStringToFile(Joined, *AbsoluteFilePath, FFileHelper::EEncodingOptions::AutoDetect, &IFileManager::Get(), FILEWRITE_AllowRead))
    {
        UE_LOG(LogTemp, Error, TEXT("ExportVisiblePointsOctreeLOD: Failed to save %s"), *AbsoluteFilePath);
        return false;
    }

    const double TotalTime = FPlatformTime::Seconds() - StartTime;
    UE_LOG(LogTemp, Log, TEXT("ExportVisiblePointsOctreeLOD: Wrote %d points to %s (%.2f sec)"), Lines.Num(), *AbsoluteFilePath, TotalTime);
    return true;
}

