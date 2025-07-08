# Niagara モジュール: ポイントクラウドテクスチャ読み込み

このドキュメントでは `ExportVisiblePointsLOD` が出力する `PosTex` と `ColorTex` テクスチャを読み込み、
各ピクセルの情報を `Particle.Position` と `Particle.Color` に書き込む簡単な Niagara モジュールの例を示します。

## 概要

`ExportVisiblePointsLOD` はテキストデータに加えて 2 種類のテクスチャをオプションで出力できます。
テクスチャは `TexDim` x `TexDim` の正方形で、各ピクセルに 1 点分のデータが格納されます。
位置テクスチャは 16 ビット HDR で RGB にワールド位置 (単位: メートル) を記録し、
カラーテクスチャには元の点の色が保存されます。

モジュールでは各パーティクルの `ParticleID` をインデックスとして両テクスチャをサンプリングし、
取得した値をパーティクルの属性に設定します。

## カスタム HLSL

Niagara Module Script を作成し、`CustomHlsl` ノードに下記コードを記述してください。
補助関数は `Shaders/NiagaraPointTexSampler.ush` に含まれています。

```hlsl
#include "NiagaraPointTexSampler.ush"

void Main(
    uint    ParticleID,
    Texture2D PosTex,
    Texture2D ColorTex,
    int TexDim,
    out float3 Position,
    out float4 Color)
{
    SamplePointTex(ParticleID, Position, Color, PosTex, ColorTex, TexDim);
}
```

### パラメータ
* **PosTex** : `_PosTex` というサフィックスで出力される HDR テクスチャ (SRGB 無効)
* **ColorTex** : `_ColorTex` というサフィックスで出力されるカラーテクスチャ (SRGB 有効)
* **TexDim** : テクスチャのピクセル数 (幅/高さ)

サンプルは `Texture.Load` を使って各ピクセルを読み込むためサンプラーステートは
不要です。位置データはメートルからセンチメートルへ変換するため `100` 倍して返
し、`Particle.Position` が Unreal の座標系と一致するようにします。

## 使い方
1. Niagara System を作成し、初期化ステージにこのモジュールを追加します。
2. `PosTex`、`ColorTex`、`TexDim` をユーザーパラメータとして公開し、エクスポートされたテクスチャを割り当てます。
3. テクスチャに含まれる有効ピクセル数と同数のパーティクルをスポーンさせます。
4. スポーン時にモジュールが `Particle.Position` と `Particle.Color` をテクスチャから設定します。
