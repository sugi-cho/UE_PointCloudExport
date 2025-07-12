# PointCloudExport

Developed with Unreal Engine 5

PointCloudExport is a plugin and sample project for Unreal Engine 5. It exports the visible points from one or more `LidarPointCloudActor`s to a text file.

| ![FullPointCloud](https://github.com/user-attachments/assets/88b9548c-3679-49fa-8877-03193212590a) | ![ExportedPointCloudLOD](https://github.com/user-attachments/assets/7cbf0fee-59da-4f15-b43b-93b4876c56cf) |
| -- | -- |
| 1,000,000 points | 99,497 points |

## Features
- Export visible points from multiple `LidarPointCloudActor` instances
- Output LOD-processed point clouds from a camera frustum
- Export position and color textures directly from a `LidarPointCloud` asset

Outputting LOD-Processed Point Clouds from a Camera Frustum

## Installing Dependencies
1. Install the **Lidar Point Cloud** plugin via the Epic Games Launcher.
2. Launch Unreal Editor and enable **Lidar Point Cloud** from `Edit > Plugins`.

## Building the Plugin
1. Clone this repository and generate project files for `PointCloudExport.uproject` via Unreal Editor or the `GenerateProjectFiles` script.
2. Build the project using Visual Studio (or your IDE of choice) or compile automatically when launching `UE5Editor`.

## Sample Scene
1. Open the project and load `Content/LiDAR-Test/L_Test.umap`.
2. The `BP_Test` blueprint calls `ExportVisiblePointsLOD` with an array of `LidarPointCloudActor` references and its `CameraComponent`. The visible portions of all clouds are merged and exported to `output.txt`.
3. You can limit the number of exported points with the optional `MaxPointCount` parameter. The limit is applied after LOD processing and points beyond the limit are skipped to avoid long export times. The default is `20,000,000`.

## Example Output
`docs/example_output.txt` shows a sample of the exported data. Each line follows the format `X Y Z Intensity R G B` where `Intensity` is measured in meters.

| X | Y | Z | Intensity | R | G | B |
| --- | --- | --- | --- | --- | --- | --- |
| 0.123456 | -1.234567 | 1.234567 | 64 | 255 | 127 | 0 |
| -1.23456 | -0.345678 | 2.345678 | 128 | 0 | 255 | 127 |
| 0.456789 | 0.3456789 | 0.123456 | 200 | 127 | 0 | 255 |

![image](https://github.com/user-attachments/assets/20b55dfb-8459-4b8d-96ff-9db1ad6f79fd)

## HDR Texture Export
`ExportVisiblePointsLOD` can optionally save two UAssets: an HDR texture encoding point positions in RGB, and a color texture storing the point colors. The alpha channel of the color texture now contains the intensity value from the point cloud. Set `bExportTexture` to `true` to generate these textures in the same folder as the original LidarPointCloudAsset. The textures are stored in an NxN square layout. Unused pixels are written as RGBA=0. If a UAsset with the same name already exists, a numbered suffix like `_1` is appended.

`SavePointCloudTextures` generates the same textures directly from a `LidarPointCloud` asset without any filtering.

| Position Texture | Color Texture |
| --- | --- |
| ![LPC_exported_PosTex](https://github.com/user-attachments/assets/3bf75909-378d-479a-b0ef-ca38bf5e1d9a) | ![LPC_exported_ColorTex](https://github.com/user-attachments/assets/cb0c40bb-8a15-46de-866e-d3d576bebde2) |

https://github.com/user-attachments/assets/e04cfc73-da85-4b91-8200-584a6a38ebab

## License

This project is licensed under the [MIT License](LICENSE).

