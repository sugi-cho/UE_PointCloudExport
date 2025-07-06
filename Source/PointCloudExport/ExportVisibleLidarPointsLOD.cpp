#include "ExportVisibleLidarPointsLOD.h"

#include "LidarPointCloudComponent.h"
#include "SceneManagement.h"            // GetViewFrustumBounds
#include "Camera/PlayerCameraManager.h"
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
    FMinimalViewInfo ViewInfo;
    Camera->GetCameraView(0.f, ViewInfo);

    const FMatrix ViewMat = FInverseRotationMatrix(ViewInfo.Rotation) *
        FTranslationMatrix(-ViewInfo.Location);

    const float   NearPlane = GNearClippingPlane;
    const float   FarPlane = WORLD_MAX;                     // 十分遠い
    const float   HalfFOV = FMath::DegreesToRadians(ViewInfo.FOV) * 0.5f;

    // UE は Reversed‑Z がデフォルト
    const FMatrix ProjMat = FReversedZPerspectiveMatrix(
        HalfFOV * 2.f,
        ViewInfo.AspectRatio,
        NearPlane,
        FarPlane);

    const FMatrix ViewProj = ViewMat * ProjMat;
    GetViewFrustumBounds(OutFrustum, ViewProj, /*bUseNearPlane=*/false);
}

// ------------------------------------------------------------
//  メイン関数
// ------------------------------------------------------------
bool UExportVisibleLidarPointsLOD::ExportVisiblePointsLOD(
    ALidarPointCloudActor* PointCloudActor,
    APlayerCameraManager* Camera,
    const FString& FilePath,
    float                  NearFullResRadius,
    float                  MidSkipRadius,
    float                  FarSkipRadius,
    int32                  SkipFactorMid,
    int32                  SkipFactorFar,
    bool                   bWorldSpace)
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

    /*------------------------------------------------------------*/
    /* 1) 視錐台でフィルタ                                       */
    /*------------------------------------------------------------*/
    FConvexVolume Frustum;
    BuildFrustumFromCamera(Camera, Frustum);

    TArray64<FLidarPointCloudPoint*> VisiblePts;
    Cloud->GetPointsInFrustum(VisiblePts, Frustum, /*bVisibleOnly=*/true);

    if (VisiblePts.Num() == 0)
    {
        UE_LOG(LogTemp, Warning, TEXT("ExportVisiblePointsLOD: No points in frustum."));
        return false;
    }

    /*------------------------------------------------------------*/
    /* 2) 距離ベース簡易 LOD                                      */
    /*------------------------------------------------------------*/
    const FVector CamLoc = Camera->GetCameraLocation();
    const float NearSq = NearFullResRadius * NearFullResRadius;
    const float MidSq = MidSkipRadius * MidSkipRadius;
    const float FarSq = FarSkipRadius * FarSkipRadius;

    const FTransform& CloudToWorld = Comp->GetComponentTransform();

    //  出力先文字列バッファ
    TArray<FString> Lines;
    Lines.Reserve(VisiblePts.Num());        // 上限見積もり

    int32 MidCounter = 0;
    int32 FarCounter = 0;

    for (const FLidarPointCloudPoint* P : VisiblePts)
    {
        // ローカル→ワールドで距離判定
        const FVector PosWS = CloudToWorld.TransformPosition(
            FVector(P->Location));

        const float DistSq = FVector::DistSquared(PosWS, CamLoc);

        // --- LOD スキップ判定 ---
        if (DistSq > FarSq)
        {
            if (++FarCounter % SkipFactorFar) continue;     // Skip
        }
        else if (DistSq > MidSq)
        {
            if (++MidCounter % SkipFactorMid) continue;     // Skip
        }
        // else Near 範囲: 常に保持

        // --- 行を追加 ---
        const FVector UsePos = bWorldSpace ? PosWS
            : FVector(P->Location);

        const FColor& Col = P->Color;
        Lines.Emplace(
            FString::Printf(TEXT("%.3f %.3f %.3f %d %d %d"),
                UsePos.X, UsePos.Y, UsePos.Z,
                Col.R, Col.G, Col.B));
    }

    if (Lines.Num() == 0)
    {
        UE_LOG(LogTemp, Warning, TEXT("ExportVisiblePointsLOD: All points skipped by LOD."));
        return false;
    }

    /*------------------------------------------------------------*/
    /* 3) ファイル書き出し                                         */
    /*------------------------------------------------------------*/
    const FString Joined = FString::Join(Lines, TEXT("\n")) + TEXT("\n");
    if (!FFileHelper::SaveStringToFile(
        Joined,
        *FilePath,
        FFileHelper::EEncodingOptions::AutoDetect,
        &IFileManager::Get(),
        FILEWRITE_AllowRead))
    {
        UE_LOG(LogTemp, Error, TEXT("ExportVisiblePointsLOD: Failed to save file %s"),
            *FilePath);
        return false;
    }

    UE_LOG(LogTemp, Log, TEXT("ExportVisiblePointsLOD: Wrote %d points → %s"),
        Lines.Num(), *FilePath);
    return true;
}
