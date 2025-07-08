# PointCloudExport

Developed with Unreal Engine 5

PointCloudExport は Unreal Engine 5 用のプラグイン/サンプルプロジェクトです。Lidar PointCloud の可視ポイントを LOD 処理しながらテキストファイルへエクスポートする機能を提供します。

## 依存プラグインのインストール
1. Epic Games Launcher から **Lidar Point Cloud** プラグインをインストールします。
2. Unreal Editor を起動し、`Edit > Plugins` で **Lidar Point Cloud** を有効化してください。

## プラグインのビルド
1. 本リポジトリをクローンし、`PointCloudExport.uproject` を Unreal Editor または `GenerateProjectFiles` スクリプトでプロジェクトファイル化します。
2. Visual Studio 等の IDE からビルドするか、`UE5Editor` 起動時に自動コンパイルさせます。

## サンプルシーンでの使用例
1. プロジェクトを開き、`Content/LiDAR-Test/L_Test.umap` をロードします。
2. シーン内の `BP_Test` ブループリントには `ExportVisiblePointsLOD` 関数呼び出しが組み込まれており、実行すると視錐台内の点群を `output.txt` として書き出します。

## 出力ファイル例
`docs/example_output.txt` にはサンプルの書き出し結果を示します。各行は `X Y Z R G B` 形式で、単位はメートル基準です。

| X | Y | Z | R | G | B |
| --- | --- | --- | --- | --- | --- |
| 0.123456 | -1.234567 | 1.234567 | 255 | 127 | 0 |
| -1.23456 | -0.345678 | 2.345678 | 0 | 255 | 127 |
| 0.456789 | 0.3456789 | 0.123456 | 127 | 0 | 255 |

![image](https://github.com/user-attachments/assets/20b55dfb-8459-4b8d-96ff-9db1ad6f79fd)

## HDR テクスチャ出力オプション
`ExportVisiblePointsLOD` には、出力するテキストファイルに加えて
各点の位置を RGB に書き込んだ HDR テクスチャと、点色のみを格納した
カラーテクスチャを UAsset として保存するオプションがあります。
関数の引数 `bExportTexture` を `true` にすると、これら 2 種類のテクスチャが
元の LidarPointCloudAsset と同じフォルダに生成されます。
テクスチャは NxN の正方形に点データを格納し、余ったピクセルは
RGBA=0 となります。
