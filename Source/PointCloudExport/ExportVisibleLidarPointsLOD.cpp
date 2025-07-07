#include "ExportVisibleLidarPointsLOD.h"

#include "LidarPointCloudComponent.h"
#include "SceneManagement.h"
#include "Camera/PlayerCameraManager.h"
#include "Runtime/Engine/Classes/Engine/EngineTypes.h"
#include "Math/Vector.h"
#include "Math/Plane.h"
#include "HAL/FileManager.h"
#include "Misc/Paths.h"
#include "Misc/FileHelper.h"

// ------------------------------------------------------------
//  ヘルパ: カメラの視錐台を作る
// ------------------------------------------------------------
static void BuildFrustumFromCamera(
    const APlayerCameraManager* Camera,
    FConvexVolume& OutFrustum)
{
    // カメラ情報取得
    const FMinimalViewInfo ViewInfo = Camera->GetCameraCacheView();

    // ビュー行列生成
    const FMatrix ViewMat = FInverseRotationMatrix(ViewInfo.Rotation) *
        FTranslationMatrix(-ViewInfo.Location);

    // クリップ面設定
    const float NearPlane = GNearClippingPlane;    // 例: 0.01
    const float FarPlane = 100000.f;               // 有限の最大距離を設定
    const float HalfFOV = FMath::DegreesToRadians(ViewInfo.FOV) * 0.5f;

    // Reversed-Z 射影行列 (Far/Near の順に引数を渡す)
    const FMatrix ProjMat = FReversedZPerspectiveMatrix(
        HalfFOV * 2.f,
        ViewInfo.AspectRatio,
        FarPlane,
        NearPlane);

#if !(UE_BUILD_SHIPPING)
    UE_LOG(LogTemp, Log, TEXT("ViewMat:\n%s"), *ViewMat.ToString());
    UE_LOG(LogTemp, Log, TEXT("ProjMat:\n%s"), *ProjMat.ToString());
#endif

    // ビュー・プロジェクション行列合成
    const FMatrix ViewProj = ViewMat * ProjMat;

    // 視錐台構築 (ニアクリップ面も含める)
    GetViewFrustumBounds(OutFrustum, ViewProj, /*bUseNearPlane=*/true);

#if !(UE_BUILD_SHIPPING)
    UE_LOG(LogTemp, Log, TEXT("Num Frustum Planes (pre-transform): %d"), OutFrustum.Planes.Num());
    for (int32 i = 0; i < OutFrustum.Planes.Num(); ++i)
    {
        const FPlane& Plane = OutFrustum.Planes[i];
        UE_LOG(LogTemp, Log,
            TEXT("  Plane[%d]: Normal=(%.3f, %.3f, %.3f) W=%.3f"),
            i, Plane.X, Plane.Y, Plane.Z, Plane.W);
    }
    UE_LOG(LogTemp, Log,
        TEXT("BuildFrustumFromCamera: Loc=%s Rot=%s FOV=%.3f Aspect=%.3f"),
        *ViewInfo.Location.ToString(), *ViewInfo.Rotation.ToString(),
        ViewInfo.FOV, ViewInfo.AspectRatio);
#endif
}

// ------------------------------------------------------------
//  メイン関数: 点群エクスポート
// ------------------------------------------------------------
bool UExportVisibleLidarPointsLOD::ExportVisiblePointsLOD(
    ALidarPointCloudActor* PointCloudActor,
    APlayerCameraManager* Camera,
    const FString& FilePath,
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
    BuildFrustumFromCamera(Camera, Frustum);

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
    Cloud->GetPointsInFrustum(VisiblePts, Frustum, /*bVisibleOnly=*/true);
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
    const float NearSq = FMath::Square(NearFullResRadius);
    const float MidSq = FMath::Square(MidSkipRadius);
    const float FarSq = FMath::Square(FarSkipRadius);

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

    int32 MidCounter = 0, FarCounter = 0;
    for (const auto* P : VisiblePts)
    {
        const FVector PosWS = CloudToWorld.TransformPosition(FVector(P->Location));
        const float DistSq = FVector::DistSquared(PosWS, CamLoc);
        if (DistSq > FarSq)
        {
            if ((++FarCounter % SkipFactorFar) != 0) continue;
        }
        else if (DistSq > MidSq)
        {
            if ((++MidCounter % SkipFactorMid) != 0) continue;
        }
        const FVector LocalPos = FVector(P->Location);
        const FVector UsePos = (bWorldSpace ? PosWS : LocalPos) * 0.01f;
        Lines.Emplace(
            FString::Printf(TEXT("%.8f %.8f %.8f %d %d %d"),
                UsePos.X, UsePos.Y, UsePos.Z,
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
