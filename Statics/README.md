# 桿件靜力分析

從 `new/Inverse_dynamics_optimization` 抽出的獨立工具，沿用原本的最小平方法求解桿件軸力，並計算節點力、應力、應變和伸長量。整個資料夾可單獨複製使用。

## 執行

在本資料夾用 Python 執行以下指令。

```powershell
python -m pip install -r requirements.txt
python Static_sim.py
```

在這台 Windows 電腦上，如果 `python` 指令無法啟動，可改用目前已安裝且含 NumPy 的 Python。

```powershell
& "$env:USERPROFILE\AppData\Local\Python\bin\python.exe" .\Static_sim.py
```

預設使用 `data/test_set.json`，在終端機顯示報告，並把文字報告及桿力 CSV 存到本資料夾的 `output/`。

## 換用自己的資料

執行 `python Static_sim.py 路徑/設定.json` 指定載荷案例。沿用範例的 `static` 結構，修改 `files` 裡的節點及桿件 CSV 路徑、`forces` 裡的節點外力，以及 `output` 的輸出開關。CSV 路徑以設定 JSON 所在資料夾為基準，也可以使用絕對路徑。

節點的 `fixed=1` 表示固定，桿件的 `Ft=1` 表示參與求解。軸力正值為拉力，負值為壓力。`Node Equilibrium` 中的 `Residual` 是自由節點的剩餘力，可用來檢查該節點是否達成平衡。

匯出 CSV 沿用原版欄名。範例採 N、mm、mm² 和 MPa，所以幾何量、應力、剛度及儲能的數值應分別按 mm 或 mm²、MPa、N/mm、N·mm 解讀。
