#pragma once

#include "CoreMinimal.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include "LidarPointCloudActor.h"
#include "ExportVisibleLidarPointsLOD.generated.h"

/**
 * ���_�Q�������o�����[�e�B���e�B�i�����x�[�X�Ȉ� LOD �t�j
 *
 * ���������t�@�C��: 1 �s 1 �_�� ASCII
 *   X Y Z R G B
 * �P��: Unreal ���[���h���W�n (cm)
 */
UCLASS()
class YOURPROJECT_API UExportVisibleLidarPointsLOD final
    : public UBlueprintFunctionLibrary
{
    GENERATED_BODY()

public:

    /**
     * @param PointCloudActor     �ΏۂƂȂ� LidarPointCloudActor
     * @param Camera              �Q�Ƃ���J���� (GetPlayerCameraManager(0) ����)
     * @param AbsoluteFilePath    ��: "C:/Temp/VisiblePoints.txt"
     * @param NearFullResRadius   ���̋����ȓ��͑S�_�ێ�         [cm]
     * @param MidSkipRadius       ���̋����𒴂���� SkipFactorMid �ŊԈ��� [cm]
     * @param FarSkipRadius       ���̋����𒴂���� SkipFactorFar �ŊԈ��� [cm]
     * @param SkipFactorMid       ��L�����тł̃T���v�����O�Ԋu (2=1/2 �_)
     * @param SkipFactorFar       �ŉ������тł̃T���v�����O�Ԋu
     * @param bWorldSpace         true: ���[���h���W / false: �_�Q���[�J��
     * @return                    ������
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
