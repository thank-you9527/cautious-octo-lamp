# 本次驗證記錄

執行環境：Linux x86-64、Python 3.12.14。頂層套件版本固定於requirements.txt。
在專案根目錄建立 `--system-site-packages` 虛擬環境，執行依賴安裝成功，
重用現有符合版本的套件；未另行宣稱完全空白機器安裝已測試。

本次已執行：

1. `.venv/bin/python -m monkey_carousel.study`：73個粗搜尋參數組合，各至少241角度；
   包含原候選、名義尺寸、敏感度與必要加密；產生兩個STEP、兩個GLB、GIF及PNG。
2. 完整study也包含9個局部細搜尋方案並保存JSON，已在上述虛擬環境完整重跑成功。
3. `python -m monkey_carousel.audit`：OpenCascade精確距離見證：
   原候選1.355703 mm；暫定方案所選見證4.000000 mm。
4. `.venv/bin/python -m pytest -q`：**14 passed**，沒有跳過或預期失敗。
   環境預裝anyio的pytest外掛發出一個pytest未來版本相容性警告，非測試失敗。
5. 實際檢視PNG預覽；測試讀入STEP、GLB及完整61幀GIF，確認沒有STL輸出。

研究與測試依序執行，避免讀到尚在寫入的產物；README及CI均指定先完成study再測試。

上述測試驗證的是指定研究模型及程式，不代表承載、耐久、列印或完整傳動合格。
GitHub Actions工作流程已提供；本機通過不代表遠端Actions已成功執行。
