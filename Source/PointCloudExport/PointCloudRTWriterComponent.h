#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "PointCloudRTWriterComponent.generated.h"

class UTextureRenderTarget2D;

/**
 * Component that writes point cloud positions and colors to render targets each tick.
 * Attach to a LidarPointCloudActor and assign render targets for positions and colors.
 */
UCLASS(ClassGroup=(Custom), meta=(BlueprintSpawnableComponent))
class POINTCLOUDEXPORT_API UPointCloudRTWriterComponent : public UActorComponent
{
    GENERATED_BODY()

public:
    UPointCloudRTWriterComponent();

    virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

    /** Render target receiving XYZ positions encoded as float16 RGBA */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="PointCloud")
    UTextureRenderTarget2D* PositionRenderTarget;

    /** Render target receiving point colors */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="PointCloud")
    UTextureRenderTarget2D* ColorRenderTarget;

protected:
    virtual void BeginPlay() override;
};

