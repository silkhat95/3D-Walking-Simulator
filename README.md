# SilkHat - 3D Walking Simulator

**[日本語]**
このゲームは、街の中を歩き回り、自動運転の電車に乗って移動する、DirectX 12 で作られた3D世界を楽しむゲームです。
外部のテクスチャや3Dモデル素材を一切使わず、すべてプログラム上でリアルタイムに合成された世界を自由に散策できます。

**[English]**
This is a 3D exploration game built with DirectX 12, where you can walk around a city and ride automated trains to enjoy the virtual world at your own pace.
Everything in the world is synthesized in real time through code without using any external textures or 3D model assets.

---

## ✨ Features / 特徴

* **100% Procedural World & Vehicles / 外部アセット不使用の完全リアルタイム合成**
  Terrain, city buildings (including furnished interiors), stations, LED departure boards, and detailed trains (interior seats, driver's cab, pantographs) are all procedurally generated in code without external textures or model files.
  地形や街並み（家具付きのビル内装まで含む）、駅のLED発車標、車両の内外装に至るまで、外部素材を一切使わずプログラムだけで合成・描画しています。
* **Hybrid Ray Tracing (DXR 1.1 & Software BVH) / ハイブリッド・レイトレーシング対応**
  Supports 7 graphics levels ranging from lightweight rasterization to hardware ray tracing (DXR 1.1) and custom software BVH ray tracing, complete with dynamic time-of-day lighting, reflections, and shadows.
  軽量な通常描画から、ハードウェアレイトレーシング（DXR 1.1）および独自のソフトウェアBVHレイトレーシングまで7段階のグラフィック設定に対応し、時間帯による空や照明の変化を楽しめます。
* **Autonomous Railway & Interactive Life / 自律運行ダイヤとインタラクティブ要素**
  Trains operate on a scheduled timetable with block signaling and railway crossings. You can ride trains, sit on seats, check departure boards, or interact with objects inside your house.
  閉塞信号や踏切と連動した自動ダイヤで電車が運行しています。電車への乗降や着席、発車標の確認、さらには自宅内の照明や家電の操作も可能です。

---

## Download & Play / ダウンロードと遊び方

1. Open the **Releases** page of this repository.
   本リポジトリの **Releases** ページを開きます。
2. Download the latest `SilkHat-3D_WalkingSimulator.zip` and extract it anywhere.
   最新の `SilkHat-3D_WalkingSimulator.zip` をダウンロードし、任意のフォルダに解凍します。
3. Run the `.exe` file to start.
   解凍したフォルダ内の `.exe` ファイルをダブルクリックして起動します。

### System Requirements / 動作環境

#### Confirmed Minimum Specs / 確認済みの最低動作環境
* **OS:** Windows 10 / Windows 11 (64-bit only)
* **CPU:** Intel Core i5-1135G7
* **GPU:** Intel Iris Graphics (DirectX 12 compatible)
* **RAM:** 8GB
* *Note: Some features are unavailable on GPUs without RT cores.*
* *※RTコアが存在しないGPUでは、一部の機能が使えません。*

#### Recommended Specs / 推奨環境
* **OS:** Windows 10 / Windows 11 (64-bit only)
* **CPU:** Intel Core 7 240H
* **GPU:** NVIDIA GeForce RTX 4050 Laptop GPU (DXR 1.1 capable)
* **RAM:** 16GB

### Controls / 基本操作
* **Move / 移動:** `W` `A` `S` `D` (`Shift` to Dash / ダッシュ, `Space` to Jump / ジャンプ)
* **Look / 視点移動:** Mouse (or `I` `J` `K` `L` when Mouse Mode is OFF)
* **Interact / 座る・立つ・操作する:** `E` (Board/sit on trains, check station markers, operate doors/lights/appliances)
* **Camera / カメラ切替（運転席着席時）:** `1` (Follow / 追従) / `2` (First-Person / 一人称視点)
* **Menu / メニュー（時刻・グラフィック設定）:** `Esc`
* **Fullscreen / フルスクリーン切替:** `F11`

---

## How to Build / ビルド方法（開発者向け）

1. Open `SilkHat-3D_WalkingSimulator.slnx` (or `TrainSim.vcxproj`) in **Visual Studio 2022** or later.
   Visual Studio 2022 以降で `SilkHat-3D_WalkingSimulator.slnx`（または `TrainSim.vcxproj`）を開きます。
2. Set the solution configuration to **Release** and platform to **x64**.
   構成を **Release**、プラットフォームを **x64** に設定します。
3. Build the solution (`Ctrl + Shift + B`). Only the standard Windows SDK is required.
   ソリューションをビルドします。標準の Windows SDK 以外の外部ライブラリは不要です。

---

## Notes / 開発に関する補足

* **Code Style & AI Assistance / コード構成とAI利用について:**
  This is a personal hobby project. The source code prioritizes working functionality over clean formatting, contains minimal comments, and utilizes AI assistance in parts of the codebase and initial icon design.
  本作は個人趣味のプロジェクトです。動作することを優先しているためコード内のコメントは少なく、コードの一部および初期アイコンの作成にAIを活用しています。
* **Code Signing / 電子署名:** Free code signing provided by [SignPath.io](https://signpath.io), certificate by [SignPath Foundation](https://signpath.org).

---

## License / ライセンス

This project is licensed under the **MIT License** - see the [LICENSE](LICENSE) file for details.
