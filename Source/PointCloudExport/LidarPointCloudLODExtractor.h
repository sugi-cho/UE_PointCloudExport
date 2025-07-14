#pragma once

#include "CoreMinimal.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include "LidarPointCloud.h"

#include "LidarPointCloudLODExtractor.generated.h"  // ← 最後

UCLASS()
class POINTCLOUDEXPORT_API ULidarPointCloudLODExtractor : public UBlueprintFunctionLibrary
{
    GENERATED_BODY()
public:
    /**
     * カメラ距離ベースの LOD 抽出 (公開 API のみ使用)。
     * @param InCloud    元クラウド
     * @param CameraLoc  カメラワールド座標
     * @param Threshold  (半径/距離)^2 の閾値 (例: 0.0001f)
     * @param MaxDepth   ソフト Octree の最大深さ (例: 8)
     * @param bDuplicate true=新規アセット, false=上書き
     */
    UFUNCTION(BlueprintCallable, CallInEditor, Category = "LiDAR|PointCloud")
    static ULidarPointCloud* ExtractLODSubset(
        ULidarPointCloud* InCloud,
        const FVector& CameraLoc,
        float             Threshold = 1e-4f,
        int32             MaxDepth = 8,
        bool              bDuplicate = true);
};