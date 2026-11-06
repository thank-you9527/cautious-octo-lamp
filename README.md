# MONKEY CAROUSEL

六門 1/6 BJD 旋轉收納展示亭，第一階段：**虛擬包絡與傳動可行性研究**。
先驗證，再設計製造零件。此儲存庫沒有正式列印 STL。

目前建議 `R=140 mm、L=90 mm`；粗搜尋後的局部細搜尋也找到較緊湊的
`R=138 mm、L=88 mm`，目前保留較多名義餘量。這些均不是最終尺寸，
也不是整套機構已驗證。詳見 [設計報告](docs/design_report.md)
與 [風險清單](docs/risks.md)。

## 重現

Python **3.12**，Linux x86-64；在儲存庫根目錄執行：

```bash
python -m venv --system-site-packages .venv
.venv/bin/python -m pip install -r requirements.txt
.venv/bin/python -m monkey_carousel.study
.venv/bin/python -m pytest -q
```

依賴需從 PyPI 下載；不需 API 金鑰、GPU、資料庫或外部服務。
CadQuery 使用 OpenCascade 匯出 STEP 及獨立布林交集測試。
**先等 study 完成，再執行測試**，避免讀到尚在寫入的預覽。
完整研究會重新搜尋並覆寫 `outputs/` 中的研究產物，通常需數分鐘。

只檢查自訂尺寸：

```python
from monkey_carousel.model import Parameters, certify
print(certify(Parameters(radius=145, length=130)))
```

## 交付

- [完整報告](docs/design_report.md)：幾何、搜尋限制、傳動比、扭矩、材料與公差。
- [機器可讀碰撞報告](outputs/collision_report.json)：原候選、搜尋、選定尺寸、敏感度、估算。
- [GIF](outputs/motion.gif)、[收納](outputs/storage.png)、[展示](outputs/display.png)。
- [收納 GLB](outputs/storage_envelopes.glb)、[展示 GLB](outputs/display_envelopes.glb)。
- [收納 STEP](outputs/storage_envelopes.step)、[展示 STEP](outputs/display_envelopes.step)。

所有模型均為**研究用保守包絡實體**，不是可直接製造的裝配件。
GIF 為每 4° 預覽；驗證則為每 1°，近間隙加密到 0.1°，並採連續行程位移界限。
GLB 原始座標數值單位為 mm；glTF 標準檢視器若按公尺解讀，請整體縮放 0.001。
STEP 使用 mm。屋頂與底座只在 GIF/PNG 隱藏；碰撞檢查及 3D 匯出均保留。
