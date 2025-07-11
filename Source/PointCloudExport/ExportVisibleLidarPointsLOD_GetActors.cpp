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
        }
    }

    return Result;
}
