// ExportVisibleLidarPointsLOD.cpp  – UE 5.5‑compatible, cm‑based, frustum‑culling fixed (Far plane −CamDir)
// NOTE: replace the existing file with this entire content.

#include "ExportVisibleLidarPointsLOD.h"

#include "LidarPointCloudComponent.h"
#include "SceneManagement.h"          // GetViewFrustumBounds, DrawDebug helpers
#include "Math/Plane.h"               // FPlane utilities
#include "Kismet/GameplayStatics.h"   // POV helpers
#include "HAL/FileManager.h"
#include "Misc/FileHelper.h"

////////////////////////////////////////////////////////////////
// 1) Build view/proj & frustum  (world space)
////////////////////////////////////////////////////////////////
void UExportVisibleLidarPointsLOD::BuildFrustumFromCamera(
    const APlayerCameraManager* Camera,
    FMatrix& OutView,
    FMatrix& OutProj,
    FConvexVolume& OutFrustumWS)
{
    const FMinimalViewInfo ViewInfo = Camera->GetCameraCacheView();   // UE5.5 preferred

    OutView = FInverseRotationMatrix(ViewInfo.Rotation) *
        FTranslationMatrix(-ViewInfo.Location);

    OutProj = ViewInfo.CalculateProjectionMatrix();                   // Reverse‑Z persp

    const FMatrix ViewProj = OutView * OutProj;
    GetViewFrustumBounds(OutFrustumWS, ViewProj, /*bUseNearPlane*/true);  // 5 planes

    //----------------- add Far plane -----------------
    constexpr float FarDist = 20000.f;                     // 200 m  (scene‑scale → cm)
    const FVector CamPos = ViewInfo.Location;
    const FVector CamDir = ViewInfo.Rotation.Vector();     // forward +X world

    // UE rule: inside = (PlaneDot >= 0) → use −CamDir so inside faces camera.
    FPlane FarWS(CamPos + CamDir * FarDist, -CamDir);
    FarWS.Normalize();
    OutFrustumWS.Planes.Add(FarWS);

    //----------------- verbose log -----------------
    UE_LOG(LogTemp, Log, TEXT("\n-- BuildFrustumFromCamera --"));
    UE_LOG(LogTemp, Log, TEXT("Location: %s"), *CamPos.ToString());
    UE_LOG(LogTemp, Log, TEXT("Rotation: %s"), *ViewInfo.Rotation.ToString());
    UE_LOG(LogTemp, Log, TEXT("HalfFOV(rad): %.6f  Aspect: %.6f"),
        FMath::DegreesToRadians(ViewInfo.FOV * 0.5f), ViewInfo.AspectRatio);
    UE_LOG(LogTemp, Log, TEXT("View  : %s"), *OutView.ToString());
    UE_LOG(LogTemp, Log, TEXT("Proj  : %s"), *OutProj.ToString());
    UE_LOG(LogTemp, Log, TEXT("ViewP : %s"), *ViewProj.ToString());

    for (int32 i = 0; i < OutFrustumWS.Planes.Num(); ++i)
        UE_LOG(LogTemp, Log, TEXT("Plane %d: %s  dotCam=%.1f"),
            i, *OutFrustumWS.Planes[i].ToString(), OutFrustumWS.Planes[i].PlaneDot(CamPos));
}

////////////////////////////////////////////////////////////////
// 2) Export entry
////////////////////////////////////////////////////////////////
bool UExportVisibleLidarPointsLOD::ExportVisiblePointsLOD(
    ALidarPointCloudActor* PointCloudActor,
    APlayerCameraManager* Camera,
    const FString& FilePath,
    float NearFullResRadius,
    float MidSkipRadius,
    float FarSkipRadius,
    int32 SkipFactorMid,
    int32 SkipFactorFar,
    bool  bWorldSpace)
{
    if (!PointCloudActor || !Camera) { UE_LOG(LogTemp, Warning, TEXT("Invalid input")); return false; }

    ULidarPointCloudComponent* Comp = PointCloudActor->GetPointCloudComponent();
    ULidarPointCloud* Cloud = Comp ? Comp->GetPointCloud() : nullptr;
    if (!Cloud) { UE_LOG(LogTemp, Warning, TEXT("No point cloud set")); return false; }

    //---------------- 1) frustum WS
    FMatrix ViewM, ProjM; FConvexVolume FrustumWS;
    BuildFrustumFromCamera(Camera, ViewM, ProjM, FrustumWS);

    //---------------- 2) frustum → local
    const FMatrix WorldToLocal = Comp->GetComponentTransform().ToMatrixWithScale().InverseFast();
    FConvexVolume FrustumLS;  FrustumLS.Planes.Reserve(FrustumWS.Planes.Num());

    for (const FPlane& P : FrustumWS.Planes)
    {
        FPlane Local = P.TransformBy(WorldToLocal); // internal handles inverse transpose
        Local.Normalize();
        FrustumLS.Planes.Add(Local);
    }

    //---------------- 3) collect points
    TArray64<FLidarPointCloudPoint*> VisiblePts;
    Cloud->GetPointsInConvexVolume(VisiblePts, FrustumLS, true);
    UE_LOG(LogTemp, Log, TEXT("== Points in FrustumLS: %lld =="), VisiblePts.Num());

    if (VisiblePts.IsEmpty()) { UE_LOG(LogTemp, Warning, TEXT("No points in frustum")); return false; }

    //---------------- 4) distance LOD
    const FVector CamLoc = Camera->GetCameraLocation();
    const float NearSq = FMath::Square(NearFullResRadius);
    const float MidSq = FMath::Square(MidSkipRadius);
    const float FarSq = FMath::Square(FarSkipRadius);

    const FTransform& CloudToWorld = Comp->GetComponentTransform();
    TArray<FString> Lines; Lines.Reserve(VisiblePts.Num());

    int32 NearCnt = 0, MidCnt = 0, FarCnt = 0; int32 MidCtr = 0, FarCtr = 0;

    for (const FLidarPointCloudPoint* P : VisiblePts)
    {
        FVector WP = CloudToWorld.TransformPosition(FVector(P->Location));
        float DistSq = FVector::DistSquared(WP, CamLoc);

        bool Keep = true;
        if (DistSq > FarSq) { Keep = (++FarCtr % SkipFactorFar) == 0; ++FarCnt; }
        else if (DistSq > MidSq) { Keep = (++MidCtr % SkipFactorMid) == 0; ++MidCnt; }
        else { ++NearCnt; }

        if (!Keep) continue;
        const FVector OutPos = bWorldSpace ? WP : FVector(P->Location);
        const FColor& C = P->Color;
        Lines.Emplace(FString::Printf(TEXT("%.3f %.3f %.3f %d %d %d"), OutPos.X, OutPos.Y, OutPos.Z, C.R, C.G, C.B));
    }

    UE_LOG(LogTemp, Log, TEXT("After LOD   Near:%d Mid:%d Far:%d  -> Written:%d"), NearCnt, MidCnt, FarCnt, Lines.Num());
    if (Lines.IsEmpty()) { UE_LOG(LogTemp, Warning, TEXT("All points skipped")); return false; }

    //---------------- 5) write TXT (meters)
    FString Content = FString::Join(Lines, TEXT("\n")) + TEXT("\n");
    if (!FFileHelper::SaveStringToFile(Content, *FilePath, FFileHelper::EEncodingOptions::AutoDetect, &IFileManager::Get(), FILEWRITE_AllowRead))
    {
        UE_LOG(LogTemp, Error, TEXT("Failed to save %s"), *FilePath); return false;
    }

    UE_LOG(LogTemp, Log, TEXT("ExportVisiblePointsLOD: wrote %d lines => %s"), Lines.Num(), *FilePath);
    return true;
}
