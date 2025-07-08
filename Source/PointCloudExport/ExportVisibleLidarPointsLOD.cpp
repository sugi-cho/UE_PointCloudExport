#include "ExportVisibleLidarPointsLOD.h"

#include "LidarPointCloudComponent.h"
#include "SceneManagement.h"
#include "Camera/PlayerCameraManager.h"
#include "Runtime/Engine/Classes/Engine/EngineTypes.h"
#include "Math/Vector.h"
#include "Math/Plane.h"
#include "HAL/FileManager.h"
#include "Misc/FileHelper.h"

// ------------------------------------------------------------
//  ヘルパ: カメラの視錐台を作る
// ------------------------------------------------------------
static void BuildFrustumFromCamera(const APlayerCameraManager* Camera, FConvexVolume& OutFrustum, float Far)
{
    OutFrustum.Planes.Empty();

    const FMinimalViewInfo ViewInfo = Camera->GetCameraCacheView();
    const FVector CamLoc = ViewInfo.Location;
    const FRotator CamRot = ViewInfo.Rotation;
    const float Near = GNearClippingPlane;
    const float Aspect = ViewInfo.AspectRatio;
    const float FOV = FMath::DegreesToRadians(ViewInfo.FOV);

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


bool IsPointInFrustum(const FVector& Pt, const FConvexVolume& Frustum)
{
    for (const FPlane& Plane : Frustum.Planes)
    {
        if (Plane.PlaneDot(Pt) > 0.f) // 点が平面の外側にある
        {
            return false;
        }
    }
    return true; // 全ての平面の内側にある
}

// ------------------------------------------------------------
//  メイン関数: 点群エクスポート
// ------------------------------------------------------------
bool UExportVisibleLidarPointsLOD::ExportVisiblePointsLOD(
    ALidarPointCloudActor* PointCloudActor,
    APlayerCameraManager* Camera,
    const FString& FilePath,
    float FrustumFar,
    float NearFullResRadius,
    float MidSkipRadius,
    float FarSkipRadius,
    int32 SkipFactorMid,
    int32 SkipFactorFar,
    bool bWorldSpace)
{
    if (!PointCloudActor || !Camera)
    {
        UE_LOG(LogTemp, Warning, TEXT("ExportVisiblePointsLOD: Invalid input."));
        return false;
    }

    ULidarPointCloudComponent* Comp = PointCloudActor->GetPointCloudComponent();
    ULidarPointCloud* Cloud = Comp ? Comp->GetPointCloud() : nullptr;
    if (!Cloud)
    {
        UE_LOG(LogTemp, Warning, TEXT("ExportVisiblePointsLOD: No point cloud set."));
        return false;
    }

    // 1) 視錐台フィルタリング
    FConvexVolume Frustum;
    BuildFrustumFromCamera(Camera, Frustum, FrustumFar);

#if !(UE_BUILD_SHIPPING)
    const FTransform& CloudTransform = Comp->GetComponentTransform();
    UE_LOG(LogTemp, Log,
        TEXT("Cloud Transform: Loc=%s Rot=%s Scale=%s"),
        *CloudTransform.GetLocation().ToString(),
        *CloudTransform.GetRotation().Rotator().ToString(),
        *CloudTransform.GetScale3D().ToString());

#endif

    // ワールド→点群ローカル変換行列
    const FMatrix WorldToCloud = Comp->GetComponentTransform().ToMatrixWithScale().Inverse();
#if !(UE_BUILD_SHIPPING)
    UE_LOG(LogTemp, Log, TEXT("WorldToCloud matrix:\n%s"), *WorldToCloud.ToString());
#endif

    // 平面をローカル空間に変換
    for (int32 i = 0; i < Frustum.Planes.Num(); ++i)
    {
        FPlane& Plane = Frustum.Planes[i];
        Plane = Plane.TransformBy(WorldToCloud);
        Plane.Normalize();
#if !(UE_BUILD_SHIPPING)
        UE_LOG(LogTemp, Log,
            TEXT("Frustum Plane[%d] (post-transform): Normal=(%.3f, %.3f, %.3f) W=%.3f"),
            i, Plane.X, Plane.Y, Plane.Z, Plane.W);
#endif
    }

    // 視錐台内の点群取得
    TArray64<FLidarPointCloudPoint*> VisiblePts;
    Cloud->GetPointsInConvexVolume(VisiblePts, Frustum, /*bVisibleOnly=*/true);
#if !(UE_BUILD_SHIPPING)
    UE_LOG(LogTemp, Log,
        TEXT("ExportVisiblePointsLOD: %lld points in frustum"), VisiblePts.Num());
#endif
    if (VisiblePts.Num() == 0)
    {
        UE_LOG(LogTemp, Warning, TEXT("ExportVisiblePointsLOD: No points in frustum."));
        return false;
    }

    // 2) 距離ベースの簡易LOD
    const FVector CamLoc = Camera->GetCameraLocation();

    const FTransform& CloudToWorld = Comp->GetComponentTransform();
    TArray<FString> Lines;
    Lines.Reserve(VisiblePts.Num());

#if !(UE_BUILD_SHIPPING)
    const int32 CheckCount = FMath::Min<int32>(10, VisiblePts.Num());
    for (int32 idx = 0; idx < CheckCount; ++idx)
    {
        const auto* P = VisiblePts[idx];
        UE_LOG(LogTemp, Verbose,
            TEXT("Pt[%d]: LocalPos=%s"), idx, *P->Location.ToString());
    }
#endif

    int32 SampleCounter = 0;
    for (const auto* P : VisiblePts)
    {
        const FVector PosWS = CloudToWorld.TransformPosition(FVector(P->Location));
        float Dist = FVector::Dist(PosWS, CamLoc);
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
        // else: Skip = 1.0f

        ++SampleCounter;
        if (FMath::Fmod((float)SampleCounter, Skip) >= 1.0f) continue;
        if (IsPointInFrustum(PosWS, Frustum) == false)
        {
            continue; // 視錐台外の点はスキップ
        }
        const FVector LocalPos = FVector(P->Location);
        const FVector UsePos = (bWorldSpace ? PosWS : LocalPos) * 0.01f;
        Lines.Emplace(
            FString::Printf(TEXT("%.8f %.8f %.8f %d %d %d"),
                UsePos.X, -UsePos.Y, UsePos.Z,
                P->Color.R, P->Color.G, P->Color.B));
    }

    if (Lines.Num() == 0)
    {
        UE_LOG(LogTemp, Warning, TEXT("ExportVisiblePointsLOD: All points skipped by LOD."));
        return false;
    }

#if !(UE_BUILD_SHIPPING)
    UE_LOG(LogTemp, Log,
        TEXT("ExportVisiblePointsLOD: %d of %lld points after LOD"),
        Lines.Num(), VisiblePts.Num());
#endif

    // 3) ファイル書き出し
    const FString Joined = FString::Join(Lines, TEXT("\n")) + TEXT("\n");
    if (!FFileHelper::SaveStringToFile(
        Joined, *FilePath,
        FFileHelper::EEncodingOptions::AutoDetect,
        &IFileManager::Get(), FILEWRITE_AllowRead))
    {
        UE_LOG(LogTemp, Error,
            TEXT("ExportVisiblePointsLOD: Failed to save file %s"), *FilePath);
        return false;
    }

    UE_LOG(LogTemp, Log,
        TEXT("ExportVisiblePointsLOD: Wrote %d of %lld points → %s"),
        Lines.Num(), VisiblePts.Num(), *FilePath);
    return true;
}
