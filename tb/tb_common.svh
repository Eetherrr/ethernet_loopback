// ============================================================
//  tb_common.svh — 各层 TB 共用的小工具（CRC / 组帧 / RGMII 激励 / 检查）
// ------------------------------------------------------------
//  用法：在 TB 的 module 体内部 `include "tb_common.svh"
//
//  前提：宿主 TB 里必须存在下列名字（任务靠名字引用它们）
//      logic       clk;          // 8ns 周期，125MHz
//      logic [7:0] phy_rxd;      // RGMII 8bit SDR 侧激励
//      logic       phy_rxdv;
//      logic       phy_err;      // RGMII RX_CTL 下降沿采样值（规范编码）
//      int         errors;       // 失败计数
//      // MACTX 用户接口（user_send 用）
//      logic [7:0] tx_data;  logic tx_valid, tx_sop, tx_eop, tx_ready;
//      logic [47:0] tx_dst_mac;  logic [15:0] tx_ethertype;
//
//  注意：xsim 会弄乱 string **实参**里的非 ASCII 字符，所以所有
//        传给 chk*/任务名字的字符串必须是 ASCII。
// ============================================================
`ifndef TB_COMMON_SVH
`define TB_COMMON_SVH

// ----------------------------------------
//  CRC32（反射实现，与 RTL 中 crc32_byte 一致）
// ----------------------------------------
function automatic [31: 0] crc32_body(input byte unsigned body[$]);
    logic [31: 0] c;
    c = 32'hFFFF_FFFF;
    foreach (body[i]) begin
        c = c ^ {24'h0, body[i]};
        for (int k = 0; k < 8; k++)
            c = c[0] ? ((c >> 1) ^ `CRC_POLY) : (c >> 1);
    end
    return c;
endfunction

// 组帧：DST + SRC + TYPE + payload(+padding)
// pad_to > 0 时补 0 到该长度；pad_to = 0 表示不补
task automatic make_body(input logic [47: 0] dst,
                         input logic [47: 0] src,
                         input logic [15: 0] etype,
                         input byte unsigned pay[$],
                         input int pad_to,
                         output byte unsigned body[$]);
    body.delete();
    for (int i = 0; i < 6; i++) body.push_back(dst[8*(5-i) +: 8]);
    for (int i = 0; i < 6; i++) body.push_back(src[8*(5-i) +: 8]);
    body.push_back(etype[15: 8]);
    body.push_back(etype[7: 0]);
    foreach (pay[i]) body.push_back(pay[i]);
    while ((pad_to > 0) && (body.size() < pad_to)) body.push_back(8'h00);
endtask

// 追加 FCS；注意线序是小端：先发 fcs[7:0]，最后发 fcs[31:24]
task automatic push_fcs(inout byte unsigned q[$], input byte unsigned body[$]);
    logic [31: 0] fcs;
    fcs = ~crc32_body(body);
    q.push_back(fcs[7: 0]);
    q.push_back(fcs[15: 8]);
    q.push_back(fcs[23: 16]);
    q.push_back(fcs[31: 24]);
endtask

// body 中真正的 payload（去掉 14 字节帧头；padding 属于数据字段，保留）
task automatic body_payload(input byte unsigned body[$], output byte unsigned pay[$]);
    pay.delete();
    for (int i = `MAC_HEAD_LEN; i < body.size(); i++) pay.push_back(body[i]);
endtask

// 队列相等比较（不用 == 运算符，避免工具差异）
function automatic bit q_equal(input byte unsigned a[$], input byte unsigned b[$]);
    if (a.size() != b.size()) return 1'b0;
    foreach (a[i]) if (a[i] != b[i]) return 1'b0;
    return 1'b1;
endfunction

// 只依赖 clk，系统级 TB 也能用
task automatic wait_cycles(input int n);
    repeat (n) @(posedge clk);
endtask

`ifndef TB_COMMON_NO_BFM
// ----------------------------------------
//  时间 / RGMII 激励
// ----------------------------------------
// 发一个 RGMII 字节。flag_err=1 时按 RGMII 规范把 RX_CTL 在下降沿拉反，
// 表示 RXERR（正常无错时下降沿与 RXDV 相同，即 phy_err = phy_rxdv）。
task automatic rgmii_byte(input logic [7: 0] b, input bit flag_err = 1'b0);
    @(negedge clk);
    phy_rxd  = b;
    phy_rxdv = 1'b1;
    phy_err  = flag_err ? 1'b0 : 1'b1;
endtask

task automatic rgmii_idle(input int n);
    @(negedge clk);
    phy_rxdv = 1'b0;
    phy_rxd  = 8'h00;
    phy_err  = 1'b0;
    repeat (n) @(negedge clk);
endtask

// 把一整帧（含 FCS）按以太网时序打到 RGMII 上
//   bad_fcs : 1 时破坏最后一个 FCS 字节
//   err_at  : 在该索引的字节上拉 RXERR；-1 表示不注入
task automatic rgmii_send(input byte unsigned frame[$],
                          input bit bad_fcs,
                          input int err_at);
    repeat (`PRE_LEN) rgmii_byte(`PRE_CODE);
    rgmii_byte(`SFD_CODE);
    foreach (frame[i]) begin
        if (bad_fcs && (i == frame.size()-1)) rgmii_byte(frame[i] ^ 8'hFF, i == err_at);
        else                                  rgmii_byte(frame[i],          i == err_at);
    end
    rgmii_idle(6);
endtask

// ----------------------------------------
//  用户侧发帧（驱动 MACTX 的 valid/ready 接口）
// ----------------------------------------
//  注意：payload 不足 46 字节时 MACTX 会自动补 0，
//        接收侧看到的 payload 会包含这些 padding 字节。
task automatic user_send(input logic [47: 0] dst, input logic [15: 0] etype,
                         input byte unsigned pay[$]);
    // 1) SOP 拍：同拍给出 dst MAC / EtherType / 第 1 个 payload 字节
    @(negedge clk);
    tx_data      = pay[0];
    tx_sop       = 1'b1;
    tx_eop       = (pay.size() == 1);
    tx_valid     = 1'b1;
    tx_dst_mac   = dst;
    tx_ethertype = etype;
    @(posedge clk);                       // SOP 在此拍被接受
    // 2) 撤 SOP，等 MAC 发完 22 拍帧头
    @(negedge clk);
    tx_sop   = 1'b0;
    tx_valid = 1'b0;
    while (!tx_ready) @(negedge clk);
    // 3) 连续送 payload 字节（中间不能断流）
    for (int i = 1; i < pay.size(); i++) begin
        tx_data  = pay[i];
        tx_eop   = (i == pay.size()-1);
        tx_valid = 1'b1;
        @(negedge clk);
    end
    tx_valid = 1'b0;
    tx_eop   = 1'b0;
endtask

`endif // TB_COMMON_NO_BFM

// ----------------------------------------
//  检查
// ----------------------------------------
task automatic chk(input string name, input bit cond);
    if (cond) $display("  [PASS] %s", name);
    else begin
        $display("  [FAIL] %s", name);
        errors = errors + 1;
    end
endtask

task automatic chk_eq_int(input string name, input int got, input int want);
    if (got == want) $display("  [PASS] %s = %0d", name, got);
    else begin
        $display("  [FAIL] %s = %0d, expect %0d", name, got, want);
        errors = errors + 1;
    end
endtask

task automatic chk_queue(input string name,
                         input byte unsigned got[$],
                         input byte unsigned want[$]);
    if (q_equal(got, want)) begin
        $display("  [PASS] %s: %0d bytes match", name, want.size());
    end
    else begin
        $display("  [FAIL] %s: mismatch (got %0d bytes, want %0d bytes)",
                 name, got.size(), want.size());
        $write("         got : ");
        foreach (got[i]) $write("%02X ", got[i]);
        $write("\n         want: ");
        foreach (want[i]) $write("%02X ", want[i]);
        $write("\n");
        errors = errors + 1;
    end
endtask

`endif // TB_COMMON_SVH
