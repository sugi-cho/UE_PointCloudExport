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
    PointCloudExportUtils::BuildFrustumFromCamera(Camera, WorldFrustum, FrustumFar);

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
