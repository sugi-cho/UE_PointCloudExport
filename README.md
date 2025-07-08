# PointCloudExport

Developed with Unreal Engine 5

PointCloudExport is a plugin and sample project for Unreal Engine 5. It allows you to export only the visible points of a Lidar PointCloud to a text file while applying LOD reduction.

## Installing Dependencies
1. Install the **Lidar Point Cloud** plugin via the Epic Games Launcher.
2. Launch Unreal Editor and enable **Lidar Point Cloud** from `Edit > Plugins`.

## Building the Plugin
1. Clone this repository and generate project files for `PointCloudExport.uproject` via Unreal Editor or the `GenerateProjectFiles` script.
2. Build the project using Visual Studio (or your IDE of choice) or compile automatically when launching `UE5Editor`.

## Sample Scene
1. Open the project and load `Content/LiDAR-Test/L_Test.umap`.
2. The `BP_Test` blueprint in the scene calls `ExportVisiblePointsLOD`, which exports the visible point cloud within the view frustum as `output.txt`.

## Example Output
`docs/example_output.txt` shows a sample of the exported data. Each line follows the format `X Y Z R G B` in meters.

| X | Y | Z | R | G | B |
| --- | --- | --- | --- | --- | --- |
| 0.123456 | -1.234567 | 1.234567 | 255 | 127 | 0 |
| -1.23456 | -0.345678 | 2.345678 | 0 | 255 | 127 |
| 0.456789 | 0.3456789 | 0.123456 | 127 | 0 | 255 |

![image](https://github.com/user-attachments/assets/20b55dfb-8459-4b8d-96ff-9db1ad6f79fd)

## HDR Texture Export
`ExportVisiblePointsLOD` can optionally save two UAssets: an HDR texture encoding point positions in RGB, and a color texture storing only the point colors. Set `bExportTexture` to `true` to generate these textures in the same folder as the original LidarPointCloudAsset. The textures are stored in an NxN square layout. Unused pixels are written as RGBA=0. If a UAsset with the same name already exists, a numbered suffix like `_1` is appended.

| Position Texture | Color Texture |
| --- | --- |
| ![LPC_exported_PosTex](https://github.com/user-attachments/assets/3bf75909-378d-479a-b0ef-ca38bf5e1d9a) | ![LPC_exported_ColorTex](https://github.com/user-attachments/assets/cb0c40bb-8a15-46de-866e-d3d576bebde2) |

https://github.com/user-attachments/assets/e04cfc73-da85-4b91-8200-584a6a38ebab
