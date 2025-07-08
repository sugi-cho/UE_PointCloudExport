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
    SamplerState Sampler,
    int TexDim,
    out float3 Position,
    out float4 Color)
{
    SamplePointTex(ParticleID, Position, Color, PosTex, ColorTex, Sampler, TexDim);
}
```

### パラメータ
* **PosTex** : `_PosTex` というサフィックスで出力される HDR テクスチャ (SRGB 無効)
* **ColorTex** : `_ColorTex` というサフィックスで出力されるカラーテクスチャ (SRGB 有効)
* **TexDim** : テクスチャのピクセル数 (幅/高さ)
* **Sampler** : テクスチャサンプラー (Clamp を推奨)

補助関数では位置をメートルからセンチメートルに変換するため `100` 倍して返します。
これにより `Particle.Position` は Unreal の座標系と一致します。

## 使い方
1. Niagara System を作成し、初期化ステージにこのモジュールを追加します。
2. `PosTex`、`ColorTex`、`TexDim` をユーザーパラメータとして公開し、エクスポートされたテクスチャを割り当てます。
3. テクスチャに含まれる有効ピクセル数と同数のパーティクルをスポーンさせます。
4. スポーン時にモジュールが `Particle.Position` と `Particle.Color` をテクスチャから設定します。
