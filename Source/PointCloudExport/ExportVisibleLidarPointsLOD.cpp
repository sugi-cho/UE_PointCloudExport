#include "ExportVisibleLidarPointsLOD.h"

#include "LidarPointCloudComponent.h"
#include "LidarPointCloud.h"
#include "SceneManagement.h"
#include "Camera/CameraComponent.h"
#include "Runtime/Engine/Classes/Engine/EngineTypes.h"
#include "Math/Vector.h"
#include "Math/Box.h"
#include "Async/ParallelFor.h"
#include "Math/Plane.h"
#include "HAL/FileManager.h"
#include "Misc/FileHelper.h"
#include "EngineUtils.h"
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

#if !(UE_BUILD_SHIPPING)
    UE_LOG(LogTemp, Log, TEXT("Manual Frustum Planes (No Flip):"));
    for (int32 i = 0; i < OutFrustum.Planes.Num(); ++i)
    {
        const FPlane& Plane = OutFrustum.Planes[i];
        UE_LOG(LogTemp, Log, TEXT("  Plane[%d]: Normal=(%.3f, %.3f, %.3f) W=%.3f"),
            i, Plane.X, Plane.Y, Plane.Z, Plane.W);
    }
#endif
}

namespace
{
    struct FSimpleOctreeNode
    {
        FBox Bounds;
        TArray<int32> Indices;
        FSimpleOctreeNode* Children[8]{ nullptr };

        explicit FSimpleOctreeNode(const FBox& InBounds) : Bounds(InBounds)
        {
            for (int32 i = 0; i < 8; ++i) { Children[i] = nullptr; }
        }
        ~FSimpleOctreeNode()
        {
            for (int32 i = 0; i < 8; ++i) { if (Children[i]) delete Children[i]; }
        }
        bool IsLeaf() const { return Children[0] == nullptr; }
    };

    class FSimpleOctree
    {
        FSimpleOctreeNode* Root;
        float MinSize;

        static bool BoxSphereOverlap(const FBox& B, const FVector& C, float RadiusSq)
        {
            float DistSq = 0.f;
            if (C.X < B.Min.X) DistSq += FMath::Square(C.X - B.Min.X); else if (C.X > B.Max.X) DistSq += FMath::Square(C.X - B.Max.X);
            if (C.Y < B.Min.Y) DistSq += FMath::Square(C.Y - B.Min.Y); else if (C.Y > B.Max.Y) DistSq += FMath::Square(C.Y - B.Max.Y);
            if (C.Z < B.Min.Z) DistSq += FMath::Square(C.Z - B.Min.Z); else if (C.Z > B.Max.Z) DistSq += FMath::Square(C.Z - B.Max.Z);
            return DistSq <= RadiusSq;
        }

        static void Subdivide(FSimpleOctreeNode* Node)
        {
            const FVector C = Node->Bounds.GetCenter();
            const FVector Ext = Node->Bounds.GetExtent() * 0.5f;
            for (int32 i = 0; i < 8; ++i)
            {
                const FVector ChildCenter = C + FVector((i & 1 ? 1.f : -1.f) * Ext.X,
                    (i & 2 ? 1.f : -1.f) * Ext.Y,
                    (i & 4 ? 1.f : -1.f) * Ext.Z);
                const FVector Min = ChildCenter - Ext;
                const FVector Max = ChildCenter + Ext;
                Node->Children[i] = new FSimpleOctreeNode(FBox(Min, Max));
            }
        }

        void InsertInternal(FSimpleOctreeNode* Node, const FVector& P, int32 Index)
        {
            if (Node->Bounds.GetExtent().GetMax() * 2.f <= MinSize)
            {
                Node->Indices.Add(Index);
                return;
            }
            if (Node->IsLeaf())
            {
                Subdivide(Node);
            }
            for (int32 i = 0; i < 8; ++i)
            {
                if (Node->Children[i]->Bounds.IsInside(P))
                {
                    InsertInternal(Node->Children[i], P, Index);
                    return;
                }
            }
            Node->Indices.Add(Index);
        }

        void QueryInternal(FSimpleOctreeNode* Node, const FVector& C, float RadiusSq, TArray<int32>& Out) const
        {
            if (!BoxSphereOverlap(Node->Bounds, C, RadiusSq))
            {
                return;
            }
            Out.Append(Node->Indices);
            if (Node->IsLeaf()) return;
            for (int32 i = 0; i < 8; ++i)
            {
                if (Node->Children[i])
                {
                    QueryInternal(Node->Children[i], C, RadiusSq, Out);
                }
            }
        }

    public:
        FSimpleOctree(const FBox& InBounds, float InMinSize)
            : Root(new FSimpleOctreeNode(InBounds)), MinSize(InMinSize) {
        }
        ~FSimpleOctree() { delete Root; }

        void Insert(const FVector& P, int32 Index) { InsertInternal(Root, P, Index); }
        void Query(const FVector& Center, float Radius, TArray<int32>& Out) const
        {
            QueryInternal(Root, Center, Radius * Radius, Out);
        }
    };
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
    float MergeDistance)
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

    // 1) 視錐台フィルタリング
    FConvexVolume WorldFrustum;
    BuildFrustumFromCamera(Camera, WorldFrustum, FrustumFar);

    struct FPointRec
    {
        FVector WorldPos;
        FVector LocalPos;
        // Color.A stores the intensity value from the source point cloud
        FColor   Color;
        ULidarPointCloud* SourceCloud;
    };

    TArray<FPointRec> AllPoints;
    ULidarPointCloud* FirstCloud = nullptr;

    for (ALidarPointCloudActor* Actor : PointCloudActors)
    {
        if (!Actor) continue;
        ULidarPointCloudComponent* Comp = Actor->GetPointCloudComponent();
        ULidarPointCloud* Cloud = Comp ? Comp->GetPointCloud() : nullptr;
        if (!Cloud) continue;

        if (!FirstCloud) FirstCloud = Cloud;

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
        for (const auto* P : VisiblePts)
        {
            FPointRec Rec;
            Rec.WorldPos = CloudToWorld.TransformPosition(FVector(P->Location) + LocationOffset);
            Rec.LocalPos = FVector(P->Location) + LocationOffset;
            Rec.Color = P->Color;
            Rec.SourceCloud = Cloud;
            AllPoints.Add(Rec);
        }
    }

    if (AllPoints.Num() == 0)
    {
        UE_LOG(LogTemp, Warning, TEXT("ExportVisiblePointsLOD: No points in frustum."));
        return false;
    }

    TArray<FPointRec> PointsToProcess;
    if (MergeDistance > 0.f)
    {
        const float MergeDistSq = MergeDistance * MergeDistance;
        FBox Bounds(EForceInit::ForceInit);
        for (const FPointRec& P : AllPoints)
        {
            Bounds += P.WorldPos;
        }
        Bounds = Bounds.ExpandBy(MergeDistance);

        FSimpleOctree Octree(Bounds, MergeDistance);
        for (const FPointRec& P : AllPoints)
        {
            TArray<int32> Nearby;
            Octree.Query(P.WorldPos, MergeDistance, Nearby);
            bool bMerged = false;
            for (int32 Idx : Nearby)
            {
                if (FVector::DistSquared(PointsToProcess[Idx].WorldPos, P.WorldPos) <= MergeDistSq)
                {
                    FPointRec& Existing = PointsToProcess[Idx];
                    Existing.WorldPos = (Existing.WorldPos + P.WorldPos) * 0.5f;
                    Existing.LocalPos = (Existing.LocalPos + P.LocalPos) * 0.5f;
                    Existing.Color.R = (Existing.Color.R + P.Color.R) / 2;
                    Existing.Color.G = (Existing.Color.G + P.Color.G) / 2;
                    Existing.Color.B = (Existing.Color.B + P.Color.B) / 2;
                    Existing.Color.A = (Existing.Color.A + P.Color.A) / 2;
                    bMerged = true;
                    break;
                }
            }
            if (!bMerged)
            {
                int32 NewIdx = PointsToProcess.Add(P);
                Octree.Insert(P.WorldPos, NewIdx);
            }
        }
    }
    else
    {
        PointsToProcess = AllPoints;
    }

    const FVector CamLoc = Camera->GetComponentLocation();

    TArray<FString> Lines;
    Lines.SetNum(PointsToProcess.Num());
#if WITH_EDITOR
    TArray<FLinearColor> PosBuffer;
    TArray<FColor> ColorBuffer;
    if (bExportTexture)
    {
        PosBuffer.SetNum(PointsToProcess.Num());
        ColorBuffer.SetNum(PointsToProcess.Num());
    }
#endif

    ParallelFor(PointsToProcess.Num(), [&](int32 Index)
        {
            const FPointRec& Rec = PointsToProcess[Index];
            float Dist = FVector::Dist(Rec.WorldPos, CamLoc);
            float Skip = 1.0f;

            if (Dist > FarSkipRadius) {
                Skip = (float)SkipFactorFar;
            }
            else if (Dist > MidSkipRadius) {
                float t = (Dist - MidSkipRadius) / (FarSkipRadius - MidSkipRadius);
                Skip = FMath::Lerp((float)SkipFactorMid, (float)SkipFactorFar, t);
            }
            else if (Dist > NearFullResRadius) {
                float t = (Dist - NearFullResRadius) / (MidSkipRadius - NearFullResRadius);
                Skip = FMath::Lerp(1.0f, (float)SkipFactorMid, t);
            }

            if (FMath::Fmod((float)(Index + 1), Skip) >= 1.0f)
            {
                return;
            }

            const FVector UsePos = (bWorldSpace ? Rec.WorldPos : Rec.LocalPos);
            Lines[Index] = FString::Printf(TEXT("%.8f %.8f %.8f %d %d %d %d"),
                UsePos.X * 0.01f, -UsePos.Y * 0.01f, UsePos.Z * 0.01f,
                Rec.Color.A, Rec.Color.R, Rec.Color.G, Rec.Color.B);
#if WITH_EDITOR
            if (bExportTexture)
            {
                PosBuffer[Index] = FLinearColor(UsePos.X, UsePos.Y, UsePos.Z, 1.f);
                // Preserve the original alpha channel which stores point intensity
                ColorBuffer[Index] = FColor(Rec.Color.R, Rec.Color.G, Rec.Color.B, Rec.Color.A);
            }
#endif
        });

    TArray<FString> FinalLines;
    FinalLines.Reserve(PointsToProcess.Num());
#if WITH_EDITOR
    TArray<FLinearColor> FinalPos;
    TArray<FColor> FinalColor;
    if (bExportTexture)
    {
        FinalPos.Reserve(PointsToProcess.Num());
        FinalColor.Reserve(PointsToProcess.Num());
    }
#endif

    for (int32 i = 0; i < Lines.Num(); ++i)
    {
        if (!Lines[i].IsEmpty())
        {
            FinalLines.Add(Lines[i]);
#if WITH_EDITOR
            if (bExportTexture)
            {
                FinalPos.Add(PosBuffer[i]);
                FinalColor.Add(ColorBuffer[i]);
            }
#endif
        }
    }

    Lines = MoveTemp(FinalLines);
#if WITH_EDITOR
    if (bExportTexture)
    {
        PosBuffer = MoveTemp(FinalPos);
        ColorBuffer = MoveTemp(FinalColor);
    }
#endif

    if (Lines.Num() == 0)
    {
        UE_LOG(LogTemp, Warning, TEXT("ExportVisiblePointsLOD: All points skipped by LOD."));
        return false;
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

    UE_LOG(LogTemp, Log,
        TEXT("ExportVisiblePointsLOD: Wrote %d points → %s"),
        Lines.Num(), *AbsoluteFilePath);
    return true;
}

// ------------------------------------------------------------
//  指定カメラから見える LidarPointCloudActor を取得
// ------------------------------------------------------------
TArray<ALidarPointCloudActor*> UExportVisibleLidarPointsLOD::GetVisibleLidarActors(
    UCameraComponent* Camera,
    float FrustumFar)
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

    FConvexVolume WorldFrustum;
    BuildFrustumFromCamera(Camera, WorldFrustum, FrustumFar);

    // Statistics: total points across visible actors and the estimated count
    int64 TotalPointCount = 0;
    int64 PredictedPointCount = 0;
    const FVector CamLoc = Camera->GetComponentLocation();

    // LOD parameters mirroring ExportVisiblePointsLOD defaults (units are cm)
    const float NearFullResRadius = 5000.f;
    const float MidSkipRadius    = 20000.f;
    const float FarSkipRadius    = 100000.f;
    const int32 SkipFactorMid    = 2;
    const int32 SkipFactorFar    = 10;

    for (TActorIterator<ALidarPointCloudActor> It(World); It; ++It)
    {
        ALidarPointCloudActor* Actor = *It;
        if (!Actor)
        {
            continue;
        }
        ULidarPointCloudComponent* Comp = Actor->GetPointCloudComponent();
        if (!Comp)
        {
            continue;
        }

        // Use the actor's component bounds instead of the private CalcBounds API
        FBox BoundsBox = Actor->GetComponentsBoundingBox(true);
        FBoxSphereBounds Bounds(BoundsBox);
        if (WorldFrustum.IntersectBox(Bounds.Origin, Bounds.BoxExtent))
        {
            Result.Add(Actor);

            ULidarPointCloud* Cloud = Comp->GetPointCloud();
            if (Cloud)
            {
                TArray64<FLidarPointCloudPoint*> Points;
                Cloud->GetPoints(Points);
                const int64 NumPoints = Points.Num();
                TotalPointCount += NumPoints;

                const FTransform& CloudToWorld = Comp->GetComponentTransform();
                const FVector LocationOffset = Cloud->LocationOffset;

                int64 LODCount = 0;
                for (int64 Index = 0; Index < NumPoints; ++Index)
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
                        ++LODCount;
                    }
                }

                PredictedPointCount += LODCount;
            }
        }
    }

    UE_LOG(LogTemp, Log, TEXT("GetVisibleLidarActors: Total Points = %lld, Estimated LOD Points = %lld"),
        TotalPointCount, PredictedPointCount);

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

    UE_LOG(LogTemp, Log, TEXT("SavePointCloudTextures: Saved %d points to %s"), PointCount, *FolderPath);
    return true;
#else
    return false;
#endif
}
