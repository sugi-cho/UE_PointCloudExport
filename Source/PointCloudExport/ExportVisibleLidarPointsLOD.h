#pragma once

#include "CoreMinimal.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include "LidarPointCloudActor.h"
#include "ExportVisibleLidarPointsLOD.generated.h"

class UCameraComponent;

/**
 *
 * 生成されるファイル: 1 行 1 点の ASCII
 *   X Y Z Intensity R G B
 * 単位: メートル (Unreal ワールド座標を m へ変換)
 */
UCLASS()
class POINTCLOUDEXPORT_API UExportVisibleLidarPointsLOD final
    : public UBlueprintFunctionLibrary
{
    GENERATED_BODY()

public:

    /**
     * @param PointCloudActors    対象となる LidarPointCloudActor 配列
     * @param Camera              参照するカメラコンポーネント
     * @param AbsoluteFilePath    例: "C:/Temp/VisiblePoints.txt"
     * @param FrustumFar          視錐台の Far 値                  [cm]
     * @param NearFullResRadius   この距離以内は全点保持         [cm]
     * @param MidSkipRadius       この距離を超えると SkipFactorMid で間引く [cm]
     * @param FarSkipRadius       この距離を超えると SkipFactorFar で間引く [cm]
     * @param SkipFactorMid       上記距離帯でのサンプリング間隔 (2=1/2 点)
     * @param SkipFactorFar       最遠距離帯でのサンプリング間隔
     * @param bWorldSpace         true: ワールド座標 / false: 点群ローカル
     * @param bExportTexture      位置/色テクスチャを UAsset として保存
     * @param MaxPointCount       LOD 適用後に出力するポイント数の上限。上限に達すると以降のポイントは処理をスキップする (0 以下で無制限)
     * @return                    成功可否
     */
    UFUNCTION(BlueprintCallable, Category = "Lidar|Export")
      static bool ExportVisiblePointsLOD(
          const TArray<ALidarPointCloudActor*>& PointCloudActors,
          UCameraComponent*      Camera,
          const FString& AbsoluteFilePath,
        float                  FrustumFar = 10000.f,
        float                  NearFullResRadius = 5000.f,
        float                  MidSkipRadius = 20000.f,
        float                  FarSkipRadius = 100000.f,
        int32                  SkipFactorMid = 2,
        int32                  SkipFactorFar = 10,
        bool                   bWorldSpace = true,
          bool                   bExportTexture = false,
        int32                  MaxPointCount = 20000000
      );

    /**
     * カメラの視錐台に入っている LidarPointCloudActor を取得
     *
     * @param Camera           チェックするカメラコンポーネント
     * @param FrustumFar       視錐台の Far 値 [cm]
     * @param NearFullResRadius   この距離以内は全点保持 [cm]
     * @param MidSkipRadius       この距離を超えると SkipFactorMid で間引く [cm]
     * @param FarSkipRadius       この距離を超えると SkipFactorFar で間引く [cm]
     * @param SkipFactorMid       近距離～中距離でのサンプリング間隔
     * @param SkipFactorFar       最遠距離帯でのサンプリング間隔
     * @return                 視錐台に入るアクター配列
     */
    UFUNCTION(BlueprintCallable, Category = "Lidar|Export")
    static TArray<ALidarPointCloudActor*> GetVisibleLidarActors(
        UCameraComponent* Camera,
        float FrustumFar = 10000.f,
        float NearFullResRadius = 5000.f,
        float MidSkipRadius = 20000.f,
        float FarSkipRadius = 100000.f,
        int32 SkipFactorMid = 2,
        int32 SkipFactorFar = 10
    );

    /**
     * 指定した LidarPointCloud アセットから位置/色テクスチャを生成して保存
     * 元のアセットと同じフォルダに PosTex と ColorTex を作成する
     *
     * @param PointCloud   対象となる LidarPointCloud アセット
     * @return             成功可否
     */
    UFUNCTION(BlueprintCallable, Category = "Lidar|Export")
    static bool SavePointCloudTextures(ULidarPointCloud* PointCloud);
};
