#include "PointCloudRTWriterComponent.h"

#include "LidarPointCloudActor.h"
#include "LidarPointCloudComponent.h"
#include "LidarPointCloud.h"
#include "Engine/TextureRenderTarget2D.h"
#include "RenderUtils.h"
#include "RHICommandList.h"

UPointCloudRTWriterComponent::UPointCloudRTWriterComponent()
{
    PrimaryComponentTick.bCanEverTick = true;
}

void UPointCloudRTWriterComponent::BeginPlay()
{
    Super::BeginPlay();
}

void UPointCloudRTWriterComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
    Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

    ALidarPointCloudActor* OwnerActor = Cast<ALidarPointCloudActor>(GetOwner());
    if (!OwnerActor || !PositionRenderTarget || !ColorRenderTarget)
    {
        return;
    }

    ULidarPointCloudComponent* Comp = OwnerActor->GetPointCloudComponent();
    ULidarPointCloud* Cloud = Comp ? Comp->GetPointCloud() : nullptr;
    if (!Cloud)
    {
        return;
    }

    TArray64<FLidarPointCloudPoint*> Points;
    Cloud->GetPoints(Points);
    const int32 PointCount = Points.Num();
    if (PointCount == 0)
    {
        return;
    }

    const int32 TexDim = FMath::CeilToInt(FMath::Sqrt(static_cast<float>(PointCount)));
    if (PositionRenderTarget->SizeX != TexDim || PositionRenderTarget->SizeY != TexDim)
    {
        PositionRenderTarget->ResizeTarget(TexDim, TexDim);
    }
    if (ColorRenderTarget->SizeX != TexDim || ColorRenderTarget->SizeY != TexDim)
    {
        ColorRenderTarget->ResizeTarget(TexDim, TexDim);
    }

    TArray<FFloat16Color> PosPixels;
    TArray<FColor> ColorPixels;
    PosPixels.Init(FFloat16Color(FLinearColor::Transparent), TexDim * TexDim);
    ColorPixels.Init(FColor(0, 0, 0, 0), TexDim * TexDim);

    for (int32 i = 0; i < PointCount; ++i)
    {
        const FLidarPointCloudPoint* P = Points[i];
        const FVector Pos = FVector(P->Location);
        PosPixels[i] = FFloat16Color(FLinearColor(Pos.X, Pos.Y, Pos.Z, 1.f));
        ColorPixels[i] = FColor(P->Color.R, P->Color.G, P->Color.B, P->Color.A);
    }

    FTextureRenderTargetResource* PosResource = PositionRenderTarget->GameThread_GetRenderTargetResource();
    FTextureRenderTargetResource* ColorResource = ColorRenderTarget->GameThread_GetRenderTargetResource();

    TArray<FFloat16Color> PosData = PosPixels;
    TArray<FColor> ColorData = ColorPixels;

    ENQUEUE_RENDER_COMMAND(UpdatePointCloudRT)(
        [PosResource, ColorResource, PosData, ColorData, TexDim](FRHICommandListImmediate& RHICmdList)
        {
            FUpdateTextureRegion2D Region(0, 0, 0, 0, TexDim, TexDim);
            const uint32 PosPitch = TexDim * sizeof(FFloat16Color);
            RHICmdList.UpdateTexture2D(PosResource->GetTexture2DRHI(), 0, Region, PosPitch, (uint8*)PosData.GetData());
            const uint32 ColorPitch = TexDim * sizeof(FColor);
            RHICmdList.UpdateTexture2D(ColorResource->GetTexture2DRHI(), 0, Region, ColorPitch, (uint8*)ColorData.GetData());
        });
}

