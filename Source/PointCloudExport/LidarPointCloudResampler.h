#pragma once

#include "CoreMinimal.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include "LidarPointCloud.h"
#include "UObject/UObjectGlobals.h"            // DuplicateObject
#include "LidarPointCloudResampler.generated.h"

UCLASS()
class ULidarPointCloudResampler : public UBlueprintFunctionLibrary
{
    GENERATED_BODY()
public:
    UFUNCTION(BlueprintCallable, Category = "LiDAR|PointCloud",
        meta = (DisplayName = "Resample LiDAR Point Cloud (Random)", AdvancedDisplay = "bInPlace"))
    static ULidarPointCloud* ResampleRandom(ULidarPointCloud* InCloud, int32 TargetPointCount, bool bInPlace = false);

    UFUNCTION(BlueprintCallable, Category = "LiDAR|PointCloud",
        meta = (DisplayName = "Resample LiDAR Point Cloud (Uniform Grid)", AdvancedDisplay = "bInPlace"))
    static ULidarPointCloud* ResampleUniformGrid(ULidarPointCloud* InCloud, int32 TargetPointCount, bool bInPlace = false);
};