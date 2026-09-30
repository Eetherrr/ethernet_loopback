# ethernet_loopback — 轻量 UDP 协议栈 + 串口回环

在 Xilinx 7 系列 FPGA 上用纯 RTL（SystemVerilog）从零实现一套精简以太网协议栈：
RGMII → MAC → ARP/IP/UDP → 应用层 → UART。跑通之后，PC 可以用 UDP 把数据打到板子、
从串口读出来；也可以往串口写数据，板子把它打包成一个 UDP 包发回 PC。

整个设计**只用一个时钟**（PHY 提供的 RGMII RX 时钟），没有任何跨时钟域逻辑，
也不需要任何 Xilinx IP 核（RGMII 用 DDR 原语，缓冲用推断出来的 BRAM）。

---

## 1. 架构

```
                 ┌──────────────────────────── 接收方向 ────────────────────────────┐
   PHY ──RGMII──►MACRX──►Arbiter──┬──►IPRX──►UDPRX──►APP──►UARTTX──► PC(串口)
    ▲              (剥帧头/剥FCS)  │  (EtherType=0x0800)
    │                             └──►ARPRX──►IP/MAC 表
    │                                  (EtherType=0x0806)   ▲
    │                                                        │ 查表
    └──RGMII──MACTX◄──TxArbiter◄──┬──IPTX◄──UDPTX◄──APP◄────┘
                (加帧头/FCS/IFG)  │                   ▲
                                  └──ARPTX◄───────────┘
                                     (ARP 请求 / 应答)      UARTRX ◄── PC(串口)
```

**接收**：MAC 剥掉前导码/以太网帧头/FCS 并做 CRC32 与帧长校验，Arbiter 按 EtherType
把载荷分给 IP 或 ARP。IP 层校验首部并按 `total_length` 裁掉以太网 padding；UDP 层
再按 UDP `length` 裁出数据；APP 把数据缓冲后按串口速率吐出。

**发送**：APP 从串口攒字节，空闲成包后查 IP/MAC 表；命中就直接发 UDP，
未命中就先发 ARP 请求、等应答把表填上再补发 UDP。UDP 层加 8 字节首部，
IP 层加 20 字节首部并算首部校验和，TxArbiter 让 ARP 插队，MAC 加前导码/FCS/帧间隔。

### 目录

```
rtl/
  System_top.sv          顶层：把下面所有模块接起来
  sys_define.svh         全局常量（帧结构 / CRC / EtherType / ARP / IP / UDP）
  PHY/RGMII.sv           RGMII 收发，调用 ODDR/IDDR 原语
  MAC/MAC.sv             MAC 顶层（MACRX + MACTX）
  MAC/MACRX.sv           接收：前导码同步、目的 MAC 过滤、剥 FCS、CRC32、帧长校验
  MAC/MACTX.sv           发送：前导码/SFD、自动 padding、FCS、IFG、underflow 保护
  ARB/Arbiter.sv         接收分流：按 EtherType 分给 IP / ARP，未支持类型丢弃
  ARB/TxArbiter.sv       发送仲裁：ARPTX / IPTX 帧级互斥，ARP 优先
  ARP/ARPRX.sv           ARP 解析、学习、判断是否需要应答
  ARP/ARPTX.sv           ARP 请求 / 应答组装
  ARP/ip_mac_map.sv      IP→MAC 缓存表（默认 3 项，循环替换）
  IP/IPRX.sv             IPv4 首部解析 + 校验和验证 + 按 total_length 截断
  IP/IPTX.sv             IPv4 首部组装 + 首部校验和
  UDP/UDPRX.sv           UDP 首部解析 + 按 length 截断
  UDP/UDPTX.sv           UDP 首部组装
  UART/UARTRX.sv         串口接收（8N1，起始位中点复采样 + 停止位校验）
  UART/UARTTX.sv         串口发送（8N1）
  APP/APP.sv             应用层：收包缓冲喂串口 / 串口攒包发 UDP / 查表 / ARP 补表
tb/
  tb_common.svh          各层 TB 共用工具（CRC、组帧、RGMII 激励、检查）
  tb_mac.sv              MAC 层单元验证
  tb_arbiter.sv          MAC + Arbiter 联调
  tb_udp_loopback.sv     系统级 UDP 回环验证（PHY 模型 + UART 模型）
scripts/                 Vivado TCL（建工程 / 综合 / 仿真 / 引脚生成）
constrs/pins.csv         引脚约束（需要自己填）
sim/                     仿真产物（VCD 波形）
```

---

## 2. 关键约定

### 2.1 字节流接口

各层之间统一用一条**直通字节流**：

| 信号 | 含义 |
|---|---|
| `xxx_data[7:0]` | 数据 |
| `xxx_valid` | 该拍数据有效 |
| `xxx_sop` | 与载荷第 1 个字节同拍 |
| `xxx_eop` | 与载荷最后 1 个字节同拍 |
| `xxx_err` | 与 `eop` 同拍，本帧有错 |

约定：**只要出现过 `sop`，之后必定出现 `eop`**。接收方向没有 ready/反压，
下游必须跟得上线速（本设计里下游都是寄存器级状态机，天然满足）。

发送方向对 MAC 的接口遵循 MACTX 的握手：`SOP` 那一拍要同时给出目的 MAC、
EtherType 和第 1 个字节；之后进入载荷阶段，`tx_ready` 恒为 1，
**中间不能断流**（断了判 underflow，本帧会用已发内容收尾成 runt）。

### 2.2 时钟方案

全设计只用 **PHY 提供的 RGMII RX 时钟 `phy_rx_clk`**，它同时作为 RGMII 发送侧
ODDR 的时钟。好处是零跨时钟域、零 FIFO；代价是**链路 down 时 PHY 不输出 RXCLK，
整个设计会失去时钟**。

实机需要注意：
- `phy_rx_clk` 进 FPGA 后要过 BUFG 再当全局时钟；
- 复位要同步到该时钟域后再送进 `rst_n`；
- 千兆 RGMII 的 RX 时钟/数据偏斜需要用 PHY 内部延时或 IDELAY 处理
  （当前 RTL 没有加 IDELAY，仿真里由 TB 保证时序）。

### 2.3 IP/MAC 缓存表

`ip_mac_map` 默认 3 项循环表：写入时若该 IP 已在表中就**就地刷新 MAC**（不占新表项），
否则写到写指针处并循环前进——等价于"只保留最近 3 次接收到的 ip/mac"。
查询是纯组合的，APP 在同一拍就能决定"直接发 UDP"还是"先发 ARP"。

### 2.4 串口成包规则

串口是字节流、没有天然边界。APP 用**空闲间隔**成包：每收到 1 个字节就把计时器清零，
空闲达到 `GAP_BITS` 个 bit 时间且缓冲非空，就把缓冲内容作为**一个** UDP 包发出去；
缓冲达到 `MAX_TX_PAYLOAD`（1472）也立即发包。

默认 115200 波特、`GAP_BITS=40` → 两包之间静默约 **0.35 ms** 即成一包。

---

## 3. 默认参数

| 参数 | 默认值 | 说明 |
|---|---|---|
| `LOCAL_MAC` | `00:11:22:33:44:55` | 板子 MAC |
| `LOCAL_IP` | `192.168.1.100` | 板子 IP |
| `LOCAL_PORT` | `1234` | 板子 UDP 端口 |
| `CLK_FREQ` | 125 MHz | 时钟频率（算波特率分频用） |
| `BAUD` | 115200 | 串口波特率（8N1） |
| `BUF_AW` | 11 | 收发缓冲各 2 KB |
| `GAP_BITS` | 40 | 空闲成包门限（bit 时间） |
| `ARP_TIMEOUT` | 125_000_000 | ARP 等待超时（1 s @125 MHz） |

对端地址不是配的，而是**从最近一次成功收到的 UDP 包学到**（源 IP + 源端口）。
所以必须先由 PC 主动发一个 UDP 包，板子才知道该往哪回。

---

## 4. 仿真

仿真走 Vivado 工程（不依赖第三方仿真器），VCD 自动导出到 `sim/`：

```bash
make sim                          # 默认跑系统级 tb_udp_loopback
make sim TB_TOP=tb_mac            # MAC 层
make sim TB_TOP=tb_arbiter        # MAC + Arbiter
make sim TB_TOP=tb_mac WAVE=1     # 打开 Vivado 波形查看器
```

VCD 文件名带 TB 名：`sim/waveform_tb_mac.vcd` 等。

> xsim 没有 `-vcd` 命令行开关，VCD 是用 xsim 的 Tcl 命令
> `open_vcd`/`log_vcd`/`close_vcd` 导出的（见 `scripts/sim.tcl`）。
>
> 另外 xsim 会弄乱 `string` **实参**里的非 ASCII 字符，所以 TB 里所有
> 传给检查任务的字符串都是 ASCII（代码注释保持中文不受影响）。

### 验证覆盖

`tb_mac.sv`（11 个用例）：单播/广播接收、元数据、CRC 错误、目的 MAC 过滤、
短帧、发送帧逐字节比对、自动 padding、最小帧间隔、payload 断流、
收发环回、RGMII RXERR 标志。

`tb_arbiter.sv`（9 个用例）：IP/ARP 分流与元数据、IPv6 丢弃、IP/ARP 交替路由、
坏 CRC 帧照常路由但带错误标记、runt 帧、目的 MAC 不匹配静默丢弃、
收发环回、统计计数器比对。

`tb_udp_loopback.sv`（6 个用例）：UDP 收到→串口吐出、串口输入→UDP 发出
（线上帧逐字节比对）、**查表未命中→先发 ARP 请求→PC 应答→自动补发 UDP**、
ARP 请求→应答、坏 CRC 不吐串口、IP/MAC 表只保留最近 3 条。

三个 TB 目前全部通过、无编译/elaborate 警告。

---

## 5. 综合 / 上板

```bash
make pins        # 从顶层端口生成 constrs/pins.csv 模板
                 #   然后手工填引脚号和电平标准
make synth       # 综合
make impl        # 实现
make bitstream   # 综合 + 实现 + bitstream
make gui         # 打开 Vivado GUI
```

顶层模块是 `System_top`。引脚需要按自己板子填 `constrs/pins.csv`
（`make refresh` 会自动把它转成 `constrs/pins.xdc`）。

综合资源占用（xc7z010clg400-1，`make synth` 实测）：

| 资源 | 用量 | 占比 |
|---|---|---|
| Slice LUTs | 1671 | 9.49 % |
| Slice Registers | 2020 | 5.74 % |
| Block RAM Tile | 1（APP 的收发缓冲各推断出半个） | 1.67 % |
| Bonded IOB | 15 | 15 % |

没有实例化任何 Xilinx IP：RGMII 用 ODDR/IDDR 原语，2 KB 收发缓冲由行为级
RAM 推断成 BRAM。

---

## 6. PC 侧怎么测

假设板子 IP `192.168.1.100`、UDP 端口 `1234`，PC 侧串口 `/dev/ttyUSB0`。

**PC → 板子 → 串口**：先让板子学到你的地址，再把载荷读出来

```bash
# 1) 发一个 UDP 包（板子据此学到对端 IP/端口）
echo -n "hello-fpga" | nc -u -w1 -p 5000 192.168.1.100 1234

# 2) 从串口读（115200 8N1）
python3 - <<'EOF'
import serial
s = serial.Serial('/dev/ttyUSB0', 115200, timeout=3)
print(s.read(10))          # b'hello-fpga'
EOF
```

**串口 → 板子 → PC**：写一串字节，静默 0.35 ms 以上即成包

```bash
python3 - <<'EOF'
import serial, socket, time
s = serial.Serial('/dev/ttyUSB0', 115200)
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(('192.168.1.101', 5000))
sock.settimeout(3)

s.write(b'ping-from-uart')
time.sleep(0.05)                       # > GAP_BITS 个 bit 时间
data, addr = sock.recvfrom(2048)
print(data)                            # b'ping-from-uart'
EOF
```

如果表里还没有 PC 的 MAC，抓包会先看到一个 ARP request（问 `192.168.1.101`），
板子收到应答后会自动把 UDP 补发出去——不需要 PC 侧重发。

---

## 7. 已知限制

- **单时钟方案**：链路 down 时没有 RXCLK，设计停止工作。
- **接收无深缓冲**：串口比以太网慢几个数量级，上一包还在往串口喂的时候
  再来的 UDP 帧会被**整帧丢弃**（计入 `stat_drop`）。所以 PC 侧不要
  连续高速灌包，要等前一包吐完。
- **UDP 校验和固定填 0**（RFC 768 允许 IPv4 下置 0 表示"未计算"），
  IP 首部校验和是正常计算的。
- 只支持 IPv4 无选项（IHL=5）、单播与广播、无 VLAN、无分片、无 IP options。
  未支持的 EtherType 由 Arbiter 丢弃并计数。
- ARP 表 3 项循环替换，不是 LRU。
- 发送方向要求 payload 不断流；上层需要有缓冲（本设计的 APP 自带 2 KB 缓冲）。

---

## 8. 实现过程中修掉的关键问题

留个记录，这几个都属于"看起来能跑、实际必错"的坑：

1. **RGMII 原语复位极性**：ODDR/IDDR 的 `R` 是**高有效**，代码原来把低有效的
   `rst_sys` 直接接上去，结果复位释放后原语仍被钉在复位态，收发全废。
2. **IDDR 的 `D` 没做位选**：`.D(phy_rxd)` 接的是整条 4bit 总线而不是 `phy_rxd[j]`，
   导致输出每一位都跟着 `phy_rxd[0]` 走（解出来只有 `0x00`/`0xff`）。
3. **RX IDDR 的工作模式**：RGMII 一个字节的低半字节在上升沿、高半字节在下降沿，
   属于**同一个时钟周期**，必须用 `SAME_EDGE_PIPELINED`；用 `SAME_EDGE` 时 Q2 落后
   半个周期，配出来的字节整体错位（`0xd5` 被读成 `0x5d`）。
4. **CRC32 残差常量**：反射实现（poly `0xEDB88320`）的残差是 `0xDEBB20E3`；
   `0xC704DD7B` 是它的 32 位比特翻转，只对非反射实现成立。同时 FCS 在线上的
   字节序是**小端**（先发 `fcs[7:0]`），这两条不对外就和 PC 网卡不通。
5. **UARTRX 的 `bit_idx` 没在进入数据段时清零**：只有第 1 个字节是对的，
   之后每个字节只采到 bit7、其余位保留上一个字节的值。
6. **APP 缓冲的两处问题**：收包缓冲在 `sop` 那一拍照着上一包的残留写指针写，
   会丢掉第 1 个字节；以及没有"串口忙"保护，新帧会把正在发送的包冲掉。
