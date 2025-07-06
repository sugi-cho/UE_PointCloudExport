#pragma once

#include "CoreMinimal.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include "LidarPointCloudActor.h"
#include "ExportVisibleLidarPointsLOD.generated.h"

/**
 * 可視点群を書き出すユーティリティ（距離ベース簡易 LOD 付）
 *
 * 生成されるファイル: 1 行 1 点の ASCII
 *   X Y Z R G B
 * 単位: Unreal ワールド座標系 (cm)
 */
UCLASS()
class YOURPROJECT_API UExportVisibleLidarPointsLOD final
    : public UBlueprintFunctionLibrary
{
    GENERATED_BODY()

public:

    /**
     * @param PointCloudActor     対象となる LidarPointCloudActor
     * @param Camera              参照するカメラ (GetPlayerCameraManager(0) 推奨)
     * @param AbsoluteFilePath    例: "C:/Temp/VisiblePoints.txt"
     * @param NearFullResRadius   この距離以内は全点保持         [cm]
     * @param MidSkipRadius       この距離を超えると SkipFactorMid で間引く [cm]
     * @param FarSkipRadius       この距離を超えると SkipFactorFar で間引く [cm]
     * @param SkipFactorMid       上記距離帯でのサンプリング間隔 (2=1/2 点)
     * @param SkipFactorFar       最遠距離帯でのサンプリング間隔
     * @param bWorldSpace         true: ワールド座標 / false: 点群ローカル
     * @return                    成功可否
     */
    UFUNCTION(BlueprintCallable, Category = "Lidar|Export")
    static bool ExportVisiblePointsLOD(
        ALidarPointCloudActor* PointCloudActor,
        APlayerCameraManager* Camera,
        const FString& AbsoluteFilePath,
        float                  NearFullResRadius = 5000.f,
        float                  MidSkipRadius = 20000.f,
        float                  FarSkipRadius = 100000.f,
        int32                  SkipFactorMid = 2,
        int32                  SkipFactorFar = 10,
        bool                   bWorldSpace = true
    );
};
