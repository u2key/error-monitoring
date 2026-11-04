# error-monitoring

三重化(TMR)したデザインを FPGA に実装して放射線試験などを行うときに、三重化フリップフロップの
不一致を別の FPGA から読み出して数えるための回路とホストソフトです。

```
 被測定 FPGA                         検出 FPGA                        ホスト PC
┌──────────────────────┐  lane_a/b/c  ┌─────────────────────┐  UART  ┌────────────┐
│ 三重化デザイン        │ ───────────▶ │ em_detector_top      │ ◀────▶ │ em_host.py │
│  (--tmr --probe)      │   clk_out    │  走査・判定・カウンタ │        │  時間管理    │
│   TMRDFF_A/_B/_C ──▶ tmrdff_readout │ ───────────▶ │                     │        │  再構成      │
│                      │ ◀─────────── │                     │        │  CSV 保存    │
└──────────────────────┘   rd_addr    └─────────────────────┘        └─────┬──────┘
          ▲                                                                │
          └──────────────────── quartus_pgm (USB-Blaster) で再構成 ◀─────────┘
```

## 判定のしかた

- フリップフロップの出力は、generate-tmr-vhdl の `--probe` で `TMRDFF_A/_B/_C` としてトップに出ます。
- 全ビットを一度に送れないので、検出側がアドレスを出して `LANES` ビットずつ(既定16並列)読み出します。
  ワード k は TMRDFF のビット `k*LANES` 〜 `k*LANES+LANES-1` です。
- 各アドレスで、安定待ち(`SETTLE` サイクル)のあと、**設計の連続した2サイクル**を読みます。
  ビットごとに A/B/C がそろっていなければ、そのサンプルはエラーです。
  - 片方のサンプルだけエラー → **フリップフロップのエラー**。数えてホストへ報告し、走査を続けます。
  - 同じビットが2サイクル続けてエラー → **構成回路のエラー**。数えて報告し、走査を止めます。
    ホストが被測定 FPGA を再構成してから再開を指示すると、走査を再開します。
- 場所ごとに数えます。
  - 検出 FPGA 内にアドレスごとのカウンタ(フリップフロップエラー16ビット、構成エラー8ビット、飽和)があります。
  - ホストはイベントからビットごと・レジスタごとに集計します。
- 被測定 FPGA のクロック(転送クロック)が止まったことも検出して報告します。
  ホストの設定で、これを構成エラーとして扱えます。

### 注意: フリップフロップエラーの検出率

tmrff(や `--light`)の三重化は、毎クロック多数決した値を書き戻します。そのため、1つのレプリカが反転しても次のクロックで直ります。
走査は1アドレスにつき2サイクルしか見ないので、1回の反転が検出される確率はおよそ

    2 / (アドレス数 × (SETTLE + 4))

です。送信回路(687,208 ビット、16並列で 42,951 アドレス、SETTLE=8)では約 4×10⁻⁶ です。
発生したフリップフロップエラーの大半は見えません(構成エラーは持続するので、1回の走査以内に必ず見えます)。
フリップフロップエラーの数を絶対値として使う場合は、この点の扱いを決める必要があります。

## ファイル

| パス | 内容 |
|---|---|
| `hdl/tmrdff_readout.vhd` | 被測定 FPGA 側。TMRDFF をアドレスで選んで出力し、クロックを転送する |
| `hdl/em_detector_top.vhd` | 検出 FPGA のトップ |
| `hdl/em_scanner.vhd` | 走査・判定・カウンタ更新(転送クロックで動作) |
| `hdl/em_host_if.vhd` | UART のコマンドとフレーム(検出ボード自身のクロックで動作) |
| `hdl/em_async_fifo.vhd`, `em_dpram.vhd`, `em_uart.vhd`, `em_pkg.vhd` | 部品 |
| `host/em_host.py` | 試験の実行(時間管理、再構成、CSV 出力) |
| `host/config.example.ini` | 設定の例 |
| `sim/run_sim.sh` | システム・テストベンチ(Questa) |
| `host/test_em_host.py` | ホスト側のテスト |

HDL はすべて VHDL-93 です。テストベンチだけが VHDL-2008 を使います。ボードには依存しません。

## 被測定 FPGA 側

1. デザインを generate-tmr-vhdl で `--tmr --probe` 付きで変換します(`--light` はシミュレーション用なので実機には使いません)。
   トップの `<top>.tmrdff.map` の1行目に、1系統あたりのビット数(WIDTH)が出ます。
2. ボードのトップで、三重化デザインと `tmrdff_readout` をつなぎます。

```vhdl
u_ro : entity work.tmrdff_readout
  generic map (WIDTH => 687208, LANES => 16, ADDR_WIDTH => 16)
  port map (clk => clk, tmrdff_a => tmrdff_a, tmrdff_b => tmrdff_b, tmrdff_c => tmrdff_c,
            rd_addr => gpio_addr, lane_a => gpio_lane_a, lane_b => gpio_lane_b,
            lane_c => gpio_lane_c, clk_out => gpio_clk);
```

`ADDR_WIDTH` は `ceil(WIDTH / LANES)` 個のアドレスが表せる幅にします。

## 検出 FPGA 側

`em_detector_top` をボードのトップにし、ピンを割り当てます。主なジェネリックは次のとおりです。

| ジェネリック | 意味 | 既定 |
|---|---|---|
| `CLK_HZ` | 検出ボードのクロック(`sys_clk`)の周波数 | 50 MHz |
| `BAUD` | UART の速度 | 115200 |
| `LANES` | 並列数(被測定側と同じ値) | 16 |
| `ADDR_WIDTH` | アドレス幅(被測定側と同じ値) | 16 |
| `NUM_ADDR` | アドレス数 = `ceil(WIDTH / LANES)` | 42951 |
| `SETTLE` | アドレス変更からサンプルまでのサイクル数 | 8 |
| `SWEEP_LOG` | 走査 `2**SWEEP_LOG` 回ごとに走査数を報告 | 8 |
| `FIFO_LOG` | イベント FIFO の深さ `2**FIFO_LOG` | 6 |
| `ALIVE_US` | 転送クロックがこの時間来なければ「停止」 | 1000 |
| `CAPTURE_FALLING` | レーンを転送クロックの立ち下がりで取り込む | true |

### ピン数(LANES=16、ADDR_WIDTH=16 のとき)

| 信号 | 本数 | 向き(検出側から見て) |
|---|---|---|
| `dut_lane_a/b/c` | 48 | 入力 |
| `dut_addr` | 16 | 出力 |
| `dut_clk` | 1 | 入力(転送クロック。クロック入力ピンを推奨) |
| 合計 | 65 | GPIO 2本(約72本)に収まります |

### タイミング

- **レーンの取り込み:** レーンは被測定側のクロックで出て、同じクロック(転送クロック)と一緒に届きます。
  検出側は既定で、立ち下がりで取り込みます(半周期の余裕)。ボード間の配線遅延が大きい場合は、
  被測定側のクロック周波数を下げるか、PLL で位相を調整してください。
- **`SETTLE` の決め方:** 往復の遅延をカバーする値にします。
  往復は、検出側のアドレス出力 → 配線 → 被測定側の2段同期 → レーンのレジスタ → 配線 → 検出側の取り込み2段、で約6サイクルです。
- **`dut_addr` の扱い:** 被測定側では非同期として扱われます(2段で同期化)。アドレスが変わってから
  `SETTLE` サイクルたつまではサンプルしないので、多ビットの同期化で一時的に値が乱れても判定には入りません。

## ホスト PC

```
cd host
cp config.example.ini config.ini     # ポート、map、quartus_pgm のコマンドを設定
python3 em_host.py config.ini --duration 3600
```

- **前提:** pyserial が必要です。
- **試験時間:** ホストが管理します(`[test] duration_s`)。`count_downtime = no` なら、再構成で止まっていた時間を除いて走査時間が `duration_s` に達するまで続けます。
- **構成エラーのとき:** `[reconfigure] command` を実行し、`settle_s` 待ってから再開を送ります。

### 出力(`[output] dir/<日時>/`)

| ファイル | 内容 |
|---|---|
| `events.csv` | エラー1ビットごとに1行。`time_s, wall_time, kind (soft/config/clock_loss), sweep, address, lane, bit, register, sample (1/2/both), replicas (ずれた系統)` |
| `reconfig.csv` | 再構成ごとに1行。`n, trigger, start_s, end_s, duration_s, return_code, output` |
| `counters.csv` | 検出 FPGA のアドレスごとのカウンタ(終了時のダンプ)。`address, bit_lo, bit_hi, registers, soft, config` |
| `counters_history.csv` | `dump_interval_s` を設定したときの途中のダンプ |
| `per_bit.csv` | ビットごとの集計。`bit, address, lane, register, soft, config` |
| `summary.csv` | 開始・終了、総時間、停止時間、走査時間、合計、FIFO あふれの有無など |

## プロトコル

- **ホスト → 検出(1バイトのコマンド):**
  `S` 開始、`P` 停止、`C` カウンタのクリア、`R` 構成エラーの停止からの再開、`D` カウンタのダンプ、`I` 情報、`?` 状態。
- **検出 → ホスト(フレーム):** `0xA5, type, length, payload, checksum`。checksum は type・length・payload の XOR です。
  各フレームの内容は `hdl/em_host_if.vhd` の冒頭を参照してください。
- **SOFT/CONFIG のペイロード:** アドレス、走査番号、両サンプルの A/B/C それぞれについて「多数決と食い違ったビットのマスク」です。

## 検証

```
sim/run_sim.sh                         # Questa。GEN_TMR に generate-tmr-vhdl の場所
cd host && python3 -m unittest test_em_host
```

**システム・テストベンチ**

小さな三重化デザイン(`sim/dut/areset.vhd` を `--tmr --light --probe` で変換、TMRDFF 37ビット)と、
読み出し回路、検出回路、UART のホストモデルをつないで、次を確かめます。

1. INFO・クリア・開始に応答する。
2. **1サイクルだけの反転:** 注入タイミングを変えて15回入れる。検出されないか、正しい場所の FF エラーとして検出されるかのどちらかで、構成エラーとは判定されない。少なくとも1回は検出される。
3. **固定した反転:** 構成エラーとして正しい場所で報告され、走査が止まる。解除と再開のあとはエラーが出ない。
4. カウンタのダンプが、受信したイベントと一致する。
5. 転送クロックの停止・復帰が報告される。

**ホスト側のテスト**

検出器のモデルを相手に、FF エラーと構成エラーを受けたときの流れ(再構成コマンドの実行と再開)と、各 CSV の内容を確かめます。
