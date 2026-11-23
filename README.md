# SilkHat - 3D Walking Simulator

**[日本語]**
このゲームは、街の中を歩き回り、自動運転の電車に乗って移動する、DirectX 12 で作られた3D世界を楽しむゲームです。
外部のテクスチャや3Dモデル素材を一切使わず、すべてプログラム上でリアルタイムに合成された世界を自由に散策できます。

**[English]**
This is a 3D exploration game built with DirectX 12, where you can walk around a city and ride automated trains to enjoy the virtual world at your own pace.
Everything in the world is synthesized in real time through code without using any external textures or 3D model assets.

---

## Screen Shots / スクリーンショット
<img width="480" height="270" alt="HXPXCxFaQAAnzm3" src="https://github.com/user-attachments/assets/52484aef-0f8b-4c3a-b966-75a0206907c2" />
<img width="480" height="270" alt="HXPXHCAbwAAIgua" src="https://github.com/user-attachments/assets/ed17d6a5-e1c7-4c39-913f-ad7f61620c0f" />
<img width="480" height="270" alt="HXPXMATaQAAoR5R" src="https://github.com/user-attachments/assets/9ff07e7d-761f-4aa2-aa84-e51104a43c4d" />
<img width="480" height="270" alt="HXPXN-8a4AA-tfl" src="https://github.com/user-attachments/assets/18a81a4a-6a88-498b-a472-2806c66d9f1d" />
<img width="480" height="270" alt="HXPXXYCaQAEgM4q" src="https://github.com/user-attachments/assets/992adbf2-7a8c-435b-a556-6f8b4004b3de" />
<img width="480" height="270" alt="HXPXanVakAAKtdS" src="https://github.com/user-attachments/assets/be20f5bc-3590-4379-88e7-5385a6639cf0" />


*Note: These screenshots were captured with ray tracing enabled.*  
*※これらの画像は、レイトレーシングをオンにした状態のものです。*

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

> ** Notice / お知らせ:**  
> Because a valid code signature could not be prepared, the release (`v26.11.18-release`) is currently distributed as an **unsigned build**. Please note that Windows SmartScreen or Smart App Control may display a warning for this unsigned build.  
> 現在、有効な電子署名が準備できなかったため、**リリース（`v26.11.18-release`）**を未署名の状態で配布しています。未署名ビルドのため、起動時に Windows のセキュリティ警告が表示される場合がありますのであらかじめご了承ください。

1. Open the **Releases** page of this repository.
   
   本リポジトリの **Releases** ページを開きます。
2. Download `SilkHat-3D_WalkingSimulator.zip` (currently `v26.11.18-release`) and extract it anywhere.
   
   `SilkHat-3D_WalkingSimulator.zip`（現在は `v26.11.18-release`）をダウンロードし、任意のフォルダに解凍します。
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

   
3. Set the solution configuration to **Release** and platform to **x64**.
   
   構成を **Release**、プラットフォームを **x64** に設定します。

   
5. Build the solution (`Ctrl + Shift + B`). Only the standard Windows SDK is required.
   
   ソリューションをビルドします。標準の Windows SDK 以外の外部ライブラリは不要です。

---

## Notes / 開発に関する補足

* **Code Style & AI Assistance / コード構成とAI利用について:**
  
  This is a personal hobby project. The source code prioritizes working functionality over clean formatting, contains minimal comments, and utilizes AI assistance in parts of the codebase and initial icon design.
  
  本作は個人趣味のプロジェクトです。動作することを優先しているためコード内のコメントは少なく、コードの一部およびアイコンにAIを活用しています。

  
- **Code Signing / 電子署名:** We are currently unable to provide code signing for this release. We will continue working to improve the project.
   
  現在、コード署名を準備できませんでした。引き続き、プロジェクトの改善を進めてまいります。


---

## License / ライセンス

This project is licensed under the **MIT License** - see the [LICENSE](LICENSE) file for details.
