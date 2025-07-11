#pragma once

#include "CoreMinimal.h"
#include "SceneManagement.h"
#include "Camera/CameraComponent.h"

namespace PointCloudExportUtils
{
#if WITH_EDITOR
    /**
     * Return a package name that does not conflict with existing assets.
     * Appends a numbered suffix if a package with the same name already exists.
     */
    FString MakeUniquePackageName(const FString& FolderPath, const FString& BaseName);
#endif

    /**
     * Build a view frustum for the given camera with the specified far plane.
     */
    void BuildFrustumFromCamera(const UCameraComponent* Camera, FConvexVolume& OutFrustum, float Far);
}

