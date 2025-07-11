#include "ExportVisibleLidarPointsLOD.h"

#include "LidarPointCloudComponent.h"
#include "LidarPointCloud.h"
#include "SceneManagement.h"
#include "Camera/CameraComponent.h"
#include "PointCloudExportUtils.h"
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

    const FString PosTexPackageName = PointCloudExportUtils::MakeUniquePackageName(FolderPath, BaseName + TEXT("_PosTex"));
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

    const FString ColorTexPackageName = PointCloudExportUtils::MakeUniquePackageName(FolderPath, BaseName + TEXT("_ColorTex"));
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
