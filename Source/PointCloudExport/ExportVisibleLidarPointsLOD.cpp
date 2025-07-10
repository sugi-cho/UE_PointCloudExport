#include "ExportVisibleLidarPointsLOD.h"

#include "LidarPointCloudComponent.h"
#include "SceneManagement.h"
#include "Camera/CameraComponent.h"
#include "Runtime/Engine/Classes/Engine/EngineTypes.h"
#include "Math/Vector.h"
#include "Math/Plane.h"
#include "HAL/FileManager.h"
#include "Misc/FileHelper.h"
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
        for (FPlane& Plane : LocalFrustum.Planes)
        {
            Plane = Plane.TransformBy(WorldToCloud);
            Plane.Normalize();
        }

        TArray64<FLidarPointCloudPoint*> VisiblePts;
        Cloud->GetPointsInConvexVolume(VisiblePts, LocalFrustum, /*bVisibleOnly=*/true);

        const FTransform& CloudToWorld = Comp->GetComponentTransform();
        for (const auto* P : VisiblePts)
        {
            FPointRec Rec;
            Rec.WorldPos = CloudToWorld.TransformPosition(FVector(P->Location));
            Rec.LocalPos = FVector(P->Location);
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
        for (const FPointRec& P : AllPoints)
        {
            bool bMerged = false;
            for (FPointRec& Existing : PointsToProcess)
            {
                if (FVector::DistSquared(Existing.WorldPos, P.WorldPos) <= MergeDistSq)
                {
                    Existing.WorldPos = (Existing.WorldPos + P.WorldPos) * 0.5f;
                    Existing.LocalPos = (Existing.LocalPos + P.LocalPos) * 0.5f;
                    Existing.Color.R = (Existing.Color.R + P.Color.R) / 2;
                    Existing.Color.G = (Existing.Color.G + P.Color.G) / 2;
                    Existing.Color.B = (Existing.Color.B + P.Color.B) / 2;
                    bMerged = true;
                    break;
                }
            }
            if (!bMerged)
            {
                PointsToProcess.Add(P);
            }
        }
    }
    else
    {
        PointsToProcess = AllPoints;
    }

    const FVector CamLoc = Camera->GetComponentLocation();

    TArray<FString> Lines;
    Lines.Reserve(PointsToProcess.Num());
#if WITH_EDITOR
    TArray<FLinearColor> PosBuffer;
    TArray<FColor> ColorBuffer;
    if (bExportTexture)
    {
        PosBuffer.Reserve(PointsToProcess.Num());
        ColorBuffer.Reserve(PointsToProcess.Num());
    }
#endif

    int32 SampleCounter = 0;
    for (const FPointRec& Rec : PointsToProcess)
    {
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

        ++SampleCounter;
        if (FMath::Fmod((float)SampleCounter, Skip) >= 1.0f) continue;

        const FVector UsePos = (bWorldSpace ? Rec.WorldPos : Rec.LocalPos);
        Lines.Emplace(
            FString::Printf(TEXT("%.8f %.8f %.8f %d %d %d"),
                UsePos.X * 0.01f, -UsePos.Y * 0.01f, UsePos.Z * 0.01f,
                Rec.Color.R, Rec.Color.G, Rec.Color.B));
#if WITH_EDITOR
        if (bExportTexture)
        {
            PosBuffer.Add(FLinearColor(UsePos.X, UsePos.Y, UsePos.Z, 1.f));
            ColorBuffer.Add(FColor(Rec.Color.R, Rec.Color.G, Rec.Color.B, 255));
        }
#endif
    }

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
