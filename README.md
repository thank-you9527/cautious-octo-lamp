# MONKEY CAROUSEL

六門 1/6 BJD 旋轉收納展示亭，目前版本 **V0.4：Design Freedom Map／裝飾安全設計區**。
先驗證，再設計製造零件。此儲存庫沒有正式列印 STL。

V0.4沿用V0.3優先候選：`R180、板寬167、高295、厚6 mm`，六根Ø10/內孔Ø8固定角柱、
Ø6轉軸；m1.5中央200T／門40T加15:1可逆減速，旋鈕720°。
新增固定A／隨門B／條件C／禁入D／結構E區、六門局部厚度圖及STEP自動檢查。
指定包絡與誤差條件下驗證，不是製造放行。詳見 [V0.4設計自由度報告](docs/design_freedom_map.md)
及[V0.3報告](docs/v03_report.md)。V0.3的1.0499 mm不是全域可加裝厚度。
原本R140/L90模型保留供回歸測試，**不符合V0.3接近閉合外牆需求**。

## 重現

Python **3.12**，Linux x86-64；在儲存庫根目錄執行：

```bash
python -m venv --system-site-packages .venv
.venv/bin/python -m pip install -r requirements.txt
.venv/bin/python -m monkey_carousel.study
.venv/bin/python -m monkey_carousel.v03
.venv/bin/python -m monkey_carousel.freedom_study
.venv/bin/python -m monkey_carousel.freedom_covers
.venv/bin/python -m pytest -q
```

依賴需從 PyPI 下載；不需 API 金鑰、GPU、資料庫或外部服務。
CadQuery 使用 OpenCascade 匯出 STEP 及獨立布林交集測試。
**先等 study 完成，再執行測試**，避免讀到尚在寫入的預覽。
舊版研究寫入`outputs/`；V0.3寫入`outputs/v03/`，完整重現通常需數分鐘。
V0.3記錄來源SHA-256，測試會拒絕與目前來源不同的過期報告。
V0.4同樣記錄來源及參數；兩個命令必須依序完成後才跑測試。

自動檢查一個門上參數化盒體（局部mm、向外為−Y）：

```bash
.venv/bin/python -m monkey_carousel.decoration_check --box 10 2 10 --frame door0 --at 90 -4 50
.venv/bin/python -m monkey_carousel.decoration_check --step candidate.step --frame door0 --at 90 -3 50 --output result.json
```

STEP依每個實體的OCCT幾何外接盒保守檢查，凹槽及孔可能造成保守拒絕；不能由拒絕
直接宣稱實際STEP必定碰撞。VERIFIED只證明已宣告場景中的幾何，不證明固定方式或強度。

只檢查自訂尺寸：

```python
from monkey_carousel.model import Parameters, certify
print(certify(Parameters(radius=145, length=130)))
```

## 交付

- [V0.4報告](docs/design_freedom_map.md)、[參數](freedom_settings.json)、[完整證據](outputs/v04/report.json)。
- [V0.4驗證紀錄](docs/v04_validation.md)：51項測試通過及未驗證範圍。
- [彩色STEP](outputs/v04/zones.step)、[彩色GLB](outputs/v04/zones.glb)、[分層檔案](outputs/v04/layers)。
- [隨動安全體積](outputs/v04/moving_safe_installed.glb)、[局部厚度圖](outputs/v04/door0_thickness.png)
  與[CSV](outputs/v04/door0_thickness_mm.csv)；door0..door5各自輸出。
- [條件罩體與卡榫資訊](outputs/v04/cover_and_clip_manifest.json)、[驗證等級](outputs/v04/verification_levels.json)。

40 mm是本次厚度搜尋上限，標示CAP的格子尚未找到物理最大值。固定／隨門區不可混用座標。
柱面只剩約0.074 mm理想徑向空間，未證明有可列印的PETG包覆。罩體及卡榫仍為CONDITIONAL。

- [V0.3完整報告](docs/v03_report.md)、[V0.3碰撞與傳動JSON](outputs/v03/report.json)。
- [V0.3動作GIF](outputs/v03/motion.gif)、[收納PNG](outputs/v03/storage.png)、[展示PNG](outputs/v03/display.png)。
- [收納GLB](outputs/v03/storage_research.glb)、[展示GLB](outputs/v03/display_research.glb)：含研究漸開線齒形。
- [收納包絡STEP](outputs/v03/storage_clearance.step)、[展示包絡STEP](outputs/v03/display_clearance.step)。
- [門板拆件概念STEP](outputs/v03/panel_split_concept.step)、[拆件尺寸計畫](outputs/v03/split_plan.json)、[定位曲線](outputs/v03/detent_curve.csv)。

以下為保留的第一階段交付：

- [完整報告](docs/design_report.md)：幾何、搜尋限制、傳動比、扭矩、材料與公差。
- [機器可讀碰撞報告](outputs/collision_report.json)：原候選、搜尋、選定尺寸、敏感度、估算。
- [GIF](outputs/motion.gif)、[收納](outputs/storage.png)、[展示](outputs/display.png)。
- [收納 GLB](outputs/storage_envelopes.glb)、[展示 GLB](outputs/display_envelopes.glb)。
- [收納 STEP](outputs/storage_envelopes.step)、[展示 STEP](outputs/display_envelopes.step)。

所有模型均為**研究用途**，不是可直接製造的裝配件；舊版與STEP主體是保守包絡，
V0.3 GLB另外含名義折線漸開線齒形。
GIF 為每 4° 預覽；驗證則為每 1°，近間隙加密到 0.1°，並採連續行程位移界限。
GLB 原始座標數值單位為 mm；glTF 標準檢視器若按公尺解讀，請整體縮放 0.001。
STEP 使用 mm。屋頂與底座只在 GIF/PNG 隱藏；碰撞檢查及 3D 匯出均保留。
