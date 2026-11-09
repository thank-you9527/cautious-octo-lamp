# V0.4 本次驗證紀錄

日期：2026-11-09。Linux x86-64、Python3.12.14，沿用現有虛擬環境。
固定Matplotlib3.10.8（已安裝）作最終毫米座標、色條、PNG及SVG圖；其餘沿用既有依賴。

已實際執行並完成：

1. `.venv/bin/python -m monkey_carousel.freedom_study`：完整世界外包、A/C/D/E完整格子、
   六門相對掃掠與厚度、檢查器例子，以及STEP／GLB輸出。
2. `.venv/bin/python -m monkey_carousel.freedom_covers`：罩體、底座全拆卸路線、
   柱面理想間隙、卡榫路徑、等級／運動清單、透明分層與最終圖表。
3. `.venv/bin/python -m monkey_carousel.decoration_check --box 10 2 10 --frame door0 --at 90 -4 50 --settings freedom_settings.json --output /tmp/v04-cli-check.json`。
4. `.venv/bin/python -m pytest -q`：**51 passed**，原27項保留、新增24個測試案例；
   沒有跳過或預期失敗。環境預裝anyio有一個pytest未來版本相容性警告，非專案失敗。
5. 來源雜湊與參數一致性檢查、本地文件連結、STEP／GLB與六門CSV／PNG／SVG讀取；
   實際檢視門0熱度圖。沒有製造STL，沒有單檔超過90 MiB。

數值結果：每門340格、101格有正厚度保證；大部分正格達40 mm搜尋上限，
少數為18.25 mm，不能把CAP當成無上限的最大值。
柱面理想圓形厚度約0.074065 mm，沒有可列印PETG包覆的驗證結論。

驗證特別包含：

- 取樣之間姿態與獨立相位，不以離散聯集冒充連續外包。
- 緩衝圓弧向外修正、Z高度排除使用實際設定的間隙，非寫死1.4 mm。
- A完整格子避開D/E/C，D覆蓋非樣本姿態；壓縮長方體可完全重建原旗標。
- 世界A也避開未來B裝飾預算；B每格位於預算內，並檢查其他五門。
- 所有Workplane／STEP實體都保留；有孔STEP採外接盒可保守拒絕，不能誤說精確原件必碰。
- 不允許把埋入宿主的零件當作「安裝面接觸」略過。
- 罩體全220 mm升降、底座全40 mm平移及卡榫全插入／變形盒體路線，
  並保留在C而非A，防止未來固定裝飾堵住拆裝。
- 未完成角柱固定端歸E，未把結構預留只當一般維修區。
- 實際PBR透明材質可分層查看；顏色及圖表不當作碰撞通過證據。

未驗證事項維持不變：裝飾重量／強度、真實列印誤差能力、卡榫接收槽與疲勞、
未知螺絲、工具操作順序、人因可達性，以及V0.3剩餘製造關卡。
本機通過不代表GitHub Actions已執行；CI已加入兩個V0.4重現命令。
