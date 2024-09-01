# geimetry-engine

## 開発

### 依存

- CMake 3.16以上
- VisualStudio 2022 C++以上

### ビルド方法

1. CMakeの実行
    - `cmake -G "Visual Studio 17 2022" -B build`
    - 上記コマンドでエラーが発生せず、`build`フォルダに`geometry-engine.sln`が生成されていることを確認してください。
2. VisualStudio2022で`geometry-engine.sln`を開く
3. スタートアッププロジェクトを`geometry-engine`に設定し、DebugもしくはReleaseモードでビルド
