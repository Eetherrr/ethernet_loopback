`timescale 1ns / 1ps
`include "../sys_define.svh"

// ============================================================
//  MACRX — 以太网 MAC 接收层
// ------------------------------------------------------------
//  时钟域
//      clk_rx 必须是 RGMII 的 RX 时钟域（PHY 提供的 phy_rx_clk），
//      因为 rgmii_rxd / rgmii_rxdv / rgmii_err 直接来自 RGMII 模块里
//      由 phy_rx_clk 驱动的 IDDR。rst_rx 需已同步到 clk_rx。
//
//  数据流
//      RGMII(8bit SDR)
//        → 前导码/SFD 同步
//        → 目的 MAC 过滤（本机单播 or 广播）
//        → 收源 MAC / EtherType
//        → 剥掉 FCS（5 拍流水延迟 + 帧尾抑制）
//        → CRC32 校验 / 帧长校验
//        → payload 字节流交给上层（Arbiter）
//
//  输出时序（直通 cut-through，无缓存，下游必须跟得上线速）
//      rx_valid  : 该拍 rx_data 有效
//      rx_sop    : 与 payload 第 1 个字节同拍
//      rx_eop    : 与 payload 最后 1 个字节同拍
//      rx_err    : 与 rx_eop 同拍，本帧有任何错误
//      rx_crc_err: 与 rx_eop 同拍，CRC 校验失败
//      约定：只要出现过 rx_sop，之后必定出现 rx_eop
//            （超长帧保护会提前停发数据，但 rx_eop 仍会在帧真正结束时给出，
//              此时 rx_valid 可能为 0，下游应检查 rx_err 并丢弃该帧）
//
//  帧头元数据
//      rx_dst_mac / rx_src_mac / rx_ethertype 在 rx_sop 之前就已稳定，
//      直到 rx_eop 之后才可能被下一帧刷新；
//      rx_len（payload 字节数，不含 FCS）在 rx_eop 时有效。
// ============================================================
module MACRX #(
        parameter [47:0] LOCAL_MAC = 48'h00_11_22_33_44_55,
        // 1: rgmii_err 直接作为错误标志（高有效，少数 PHY 的做法）
        // 0: 按 RGMII 规范解码——RX_CTL 上升沿 = RXDV、下降沿 = RXDV ^ RXERR，
        //    即 IDDR 输出 Q1=rxdv、Q2=err 时，错误 = (err ^ rxdv)
        parameter bit    ERR_ACTIVE_HIGH = 1'b0
    )(
        // System Interface（RGMII RX 时钟域）
        input  logic         clk_rx,
        input  logic         rst_rx,

        // User Interface（→ Arbiter）
        output logic [7: 0]  rx_data,
        output logic         rx_valid,
        output logic         rx_sop,
        output logic         rx_eop,
        output logic         rx_err,
        output logic         rx_crc_err,
        output logic [15: 0] rx_len,
        output logic [47: 0] rx_dst_mac,
        output logic [47: 0] rx_src_mac,
        output logic [15: 0] rx_ethertype,

        // RGMII Interface
        input  logic [7: 0]  rgmii_rxd,
        input  logic         rgmii_rxdv,
        input  logic         rgmii_err
    );

    // ----------------------------------------
    //  状态编码（独热）
    // ----------------------------------------
    localparam [7:0] S_IDLE    = 8'b0000_0001,
                     S_PRE     = 8'b0000_0010,   // 前导码 + SFD 同步
                     S_DST     = 8'b0000_0100,
                     S_SRC     = 8'b0000_1000,
                     S_TYPE    = 8'b0001_0000,
                     S_PAYLOAD = 8'b0010_0000,
                     S_DROP    = 8'b0100_0000;   // 不是发给本机的帧，等 rxdv 撤销

    // 异常长帧保护：帧体超过该长度就停止转发，避免打爆下游
    localparam [15:0] MAX_FRAME_GUARD = 16'd2048;

    // ----------------------------------------
    //  内部寄存器 / 线网声明（必须在组合逻辑引用之前）
    // ----------------------------------------
    logic [7: 0]  state;

    logic [2: 0]  pre_cnt;          // 前导码 0x55 计数
    logic [2: 0]  cnt_byte;         // DST/SRC/TYPE 段内字节计数
    logic [15: 0] cnt_frame;        // 帧体长度（DST..FCS）
    logic [15: 0] cnt_payload;      // payload 段字节数（含 FCS）

    logic [47: 0] dst_mac_r;
    logic [47: 0] src_mac_r;
    logic [15: 0] type_r;

    logic [31: 0] crc_r;
    logic         err_seen;         // 帧内出现过 rgmii_err
    logic         out_en;           // 处于可转发窗口
    logic         discard;          // 超长帧保护：停止转发
    logic         frame_fwd;        // 本帧已转发过至少一个字节

    logic [39: 0] pipe_sr;          // 5 级 × 8bit 输出流水线
    logic [4: 0]  pipe_pay;

    // ----------------------------------------
    //  输入错误标志
    // ----------------------------------------
    wire rx_error = ERR_ACTIVE_HIGH ? rgmii_err : (rgmii_err ^ rgmii_rxdv);

    // ----------------------------------------
    //  状态转移条件
    // ----------------------------------------
    wire idle2pre = (state == S_IDLE) && rgmii_rxdv && (rgmii_rxd == `PRE_CODE);

    // 前导码至少 5 个 0x55 后收到 0xD5 才认为同步（兼容缩短的前导码）
    wire pre2dst  = (state == S_PRE) && rgmii_rxdv && (rgmii_rxd == `SFD_CODE)
                    && (pre_cnt >= 3'd4);
    wire pre2idle = (state == S_PRE) && (!rgmii_rxdv
                    || ((rgmii_rxd != `PRE_CODE) && !pre2dst));

    wire [47: 0] dst_mac_full = {dst_mac_r[39: 0], rgmii_rxd};
    wire dst_matched = (dst_mac_full == LOCAL_MAC) || (dst_mac_full == `BROADCAST_MAC);

    wire dst_last = (state == S_DST) && rgmii_rxdv && (cnt_byte == 3'd5);
    wire dst2src  = dst_last && dst_matched;
    wire dst2drop = dst_last && !dst_matched;
    wire dst2idle = (state == S_DST) && !rgmii_rxdv;

    wire src2type = (state == S_SRC) && rgmii_rxdv && (cnt_byte == 3'd5);
    wire src2idle = (state == S_SRC) && !rgmii_rxdv;

    wire type2payload = (state == S_TYPE) && rgmii_rxdv && (cnt_byte == 3'd1);
    wire type2idle    = (state == S_TYPE) && !rgmii_rxdv;

    wire payload2idle = (state == S_PAYLOAD) && !rgmii_rxdv;   // 正常帧结束
    wire drop2idle    = (state == S_DROP) && !rgmii_rxdv;

    wire frame_end = payload2idle;                             // rxdv 拉低那一拍
    wire in_body   = (state == S_DST) || (state == S_SRC)
                     || (state == S_TYPE) || (state == S_PAYLOAD);

    wire pipe_we = rgmii_rxdv && (state == S_PAYLOAD);
    wire crc_en  = rgmii_rxdv && in_body;

    // ----------------------------------------
    //  FSM
    // ----------------------------------------
    always_ff @(posedge clk_rx or negedge rst_rx) begin
        if (!rst_rx) begin
            state <= S_IDLE;
        end
        else begin
            case (state)
                S_IDLE: begin
                    if (idle2pre) state <= S_PRE;
                end
                S_PRE: begin
                    if (pre2dst)       state <= S_DST;
                    else if (pre2idle) state <= S_IDLE;
                end
                S_DST: begin
                    if (dst2src)       state <= S_SRC;
                    else if (dst2drop) state <= S_DROP;
                    else if (dst2idle) state <= S_IDLE;
                end
                S_SRC: begin
                    if (src2type)      state <= S_TYPE;
                    else if (src2idle) state <= S_IDLE;
                end
                S_TYPE: begin
                    if (type2payload)   state <= S_PAYLOAD;
                    else if (type2idle) state <= S_IDLE;
                end
                S_PAYLOAD: begin
                    if (payload2idle) state <= S_IDLE;
                end
                S_DROP: begin
                    if (drop2idle) state <= S_IDLE;
                end
                default: state <= S_IDLE;
            endcase
        end
    end

    // ----------------------------------------
    //  计数器
    // ----------------------------------------
    always_ff @(posedge clk_rx or negedge rst_rx) begin
        if (!rst_rx) begin
            pre_cnt <= 3'd0;
        end
        else if (idle2pre) begin
            pre_cnt <= 3'd1;                       // S_IDLE 里那个 0x55 是第 1 个
        end
        else if ((state == S_PRE) && rgmii_rxdv && (rgmii_rxd == `PRE_CODE)
                 && (pre_cnt != 3'd7)) begin
            pre_cnt <= pre_cnt + 3'd1;
        end
    end

    always_ff @(posedge clk_rx or negedge rst_rx) begin
        if (!rst_rx) begin
            cnt_byte <= 3'd0;
        end
        else if (rgmii_rxdv) begin
            case (state)
                S_DST:   cnt_byte <= (cnt_byte == 3'd5) ? 3'd0 : cnt_byte + 3'd1;
                S_SRC:   cnt_byte <= (cnt_byte == 3'd5) ? 3'd0 : cnt_byte + 3'd1;
                S_TYPE:  cnt_byte <= (cnt_byte == 3'd1) ? 3'd0 : cnt_byte + 3'd1;
                default: cnt_byte <= 3'd0;
            endcase
        end
    end

    always_ff @(posedge clk_rx or negedge rst_rx) begin
        if (!rst_rx) begin
            cnt_frame <= 16'd0;
        end
        else if (pre2dst) begin
            cnt_frame <= 16'd0;
        end
        else if (rgmii_rxdv && in_body) begin
            cnt_frame <= cnt_frame + 16'd1;
        end
    end

    always_ff @(posedge clk_rx or negedge rst_rx) begin
        if (!rst_rx) begin
            cnt_payload <= 16'd0;
        end
        else if (type2payload) begin
            cnt_payload <= 16'd0;
        end
        else if (rgmii_rxdv && (state == S_PAYLOAD)) begin
            cnt_payload <= cnt_payload + 16'd1;
        end
    end

    // ----------------------------------------
    //  帧头字段接收 / 目的 MAC 过滤
    // ----------------------------------------
    always_ff @(posedge clk_rx or negedge rst_rx) begin
        if (!rst_rx) dst_mac_r <= 48'd0;
        else if ((state == S_DST) && rgmii_rxdv) dst_mac_r <= {dst_mac_r[39: 0], rgmii_rxd};
    end

    always_ff @(posedge clk_rx or negedge rst_rx) begin
        if (!rst_rx) src_mac_r <= 48'd0;
        else if ((state == S_SRC) && rgmii_rxdv) src_mac_r <= {src_mac_r[39: 0], rgmii_rxd};
    end

    always_ff @(posedge clk_rx or negedge rst_rx) begin
        if (!rst_rx) type_r <= 16'd0;
        else if ((state == S_TYPE) && rgmii_rxdv) type_r <= {type_r[7: 0], rgmii_rxd};
    end

    // 帧头元数据锁存（在 rx_sop 之前稳定，直到下一帧帧头才刷新）
    always_ff @(posedge clk_rx or negedge rst_rx) begin
        if (!rst_rx) begin
            rx_dst_mac   <= 48'd0;
            rx_src_mac   <= 48'd0;
            rx_ethertype <= 16'd0;
        end
        else begin
            if (dst2src)      rx_dst_mac   <= dst_mac_full;
            if (src2type)     rx_src_mac   <= {src_mac_r[39: 0], rgmii_rxd};
            if (type2payload) rx_ethertype <= {type_r[7: 0], rgmii_rxd};
        end
    end

    // payload 长度（不含 FCS），rx_eop 时有效
    assign rx_len = (cnt_payload >= `FCS_LEN) ? (cnt_payload - `FCS_LEN) : 16'd0;

    // ----------------------------------------
    //  CRC32 累加（DST..PAYLOAD..FCS 全部参与）
    // ----------------------------------------
    function [31: 0] crc32_byte;
        input [31: 0] crc_in;
        input [7: 0]  data;
        reg [31: 0] c;
        integer i;
        begin
            c = crc_in ^ {24'h0, data};
            for (i = 0; i < 8; i = i + 1)
                c = c[0] ? ((c >> 1) ^ `CRC_POLY) : (c >> 1);
            crc32_byte = c;
        end
    endfunction

    always_ff @(posedge clk_rx or negedge rst_rx) begin
        if (!rst_rx) begin
            crc_r <= 32'hFFFF_FFFF;
        end
        else if (pre2dst) begin
            crc_r <= 32'hFFFF_FFFF;
        end
        else if (crc_en) begin
            crc_r <= crc32_byte(crc_r, rgmii_rxd);
        end
    end

    // 帧结束时（rxdv 拉低那一拍）crc_r 已经是最终值
    wire crc_bad = (crc_r != `CRC_RESIDUE);
    wire len_ok  = (cnt_frame >= `MIN_FRAME_LEN) && (cnt_frame <= `MAX_FRAME_LEN);

    // ----------------------------------------
    //  帧内错误记录
    // ----------------------------------------
    always_ff @(posedge clk_rx or negedge rst_rx) begin
        if (!rst_rx) begin
            err_seen <= 1'b0;
        end
        else if (idle2pre || pre2dst) begin
            err_seen <= 1'b0;
        end
        else if (rx_error && in_body) begin
            err_seen <= 1'b1;
        end
    end

    // ----------------------------------------
    //  输出流水线：延迟 5 拍，丢掉末尾 4 字节 FCS
    // ----------------------------------------
    //  pipe_sr[39:32] / pipe_pay[4] 对应 5 拍前写入的那个字节。
    //  这样在 rxdv 拉低那一拍，流水线吐出的正好是 payload 最后一个字节，
    //  于是可以在同拍给出 rx_eop 并附上 CRC/帧长校验结果。
    always_ff @(posedge clk_rx or negedge rst_rx) begin
        if (!rst_rx) begin
            pipe_sr  <= 40'd0;
            pipe_pay <= 5'd0;
        end
        else begin
            pipe_sr  <= {pipe_sr[31: 0], pipe_we ? rgmii_rxd : 8'h00};
            pipe_pay <= {pipe_pay[3: 0], pipe_we};
        end
    end

    // out_en：进入 payload 后拉高，帧结束那一拍之后拉低
    always_ff @(posedge clk_rx or negedge rst_rx) begin
        if (!rst_rx) begin
            out_en <= 1'b0;
        end
        else if (type2payload) begin
            out_en <= 1'b1;
        end
        else if (frame_end || (state == S_DROP)) begin
            out_en <= 1'b0;
        end
    end

    // 超长帧保护：停止转发，但 rx_eop 仍会在帧真正结束时给出
    always_ff @(posedge clk_rx or negedge rst_rx) begin
        if (!rst_rx) begin
            discard <= 1'b0;
        end
        else if (idle2pre || pre2dst) begin
            discard <= 1'b0;
        end
        else if ((state == S_PAYLOAD) && rgmii_rxdv && (cnt_frame >= MAX_FRAME_GUARD)) begin
            discard <= 1'b1;
        end
    end

    assign rx_data  = pipe_sr[39: 32];
    assign rx_valid = pipe_pay[4] && out_en && !discard;

    // frame_fwd：本帧已经转发过至少一个字节（用于产生 rx_sop）
    always_ff @(posedge clk_rx or negedge rst_rx) begin
        if (!rst_rx) begin
            frame_fwd <= 1'b0;
        end
        else if (frame_end) begin
            frame_fwd <= 1'b0;
        end
        else if (rx_valid) begin
            frame_fwd <= 1'b1;
        end
    end

    assign rx_sop = rx_valid && !frame_fwd;
    assign rx_eop = frame_end && frame_fwd;

    assign rx_err     = rx_eop && (crc_bad || !len_ok || err_seen);
    assign rx_crc_err = rx_eop && crc_bad;

endmodule
