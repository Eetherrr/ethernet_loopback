`timescale 1ns / 1ps
module RGMII(
        // System Interface
        input wire clk_sys,
        input wire rst_sys,

        // MAC Interface
        input wire [7: 0] rgmii_txd,
        input wire rgmii_txdv,

        output wire [7: 0] rgmii_rxd,
        output wire rgmii_rxdv,
        output wire rgmii_err,

        // PHY Interface
        // TX
        output wire phy_tx_clk,
        output wire [3: 0] phy_txd,
        output wire phy_tx_ctl,
        // RX
        input wire phy_rx_clk,
        input wire [3: 0] phy_rxd,
        input wire phy_rx_ctl
    );

    // RX 侧 IDDR 必须用 SAME_EDGE_PIPELINED：
    //   RGMII 一个字节的低半字节在上升沿、高半字节在下降沿，属于**同一个
    //   时钟周期**。SAME_EDGE 模式下 Q2 取的是上一个下降沿，配出来的字节
    //   会整体错半个周期（形如 0xd5 被读成 0x5d）。
    //
    // ODDR/IDDR 原语的 R/S 都是**高有效**异步复位，而本设计统一用低有效
    // rst_sys，这里必须取反，否则复位释放后原语仍被钉在复位态
    // （表现为 phy_txd 恒 0、rgmii_rxd/rxdv 恒 0，收发全废）。
    wire rst_hi = ~rst_sys;

    // CLK 与数据同步输出
    ODDR #(
             .DDR_CLK_EDGE("SAME_EDGE"),
             .INIT(1'b0),
             .SRTYPE("ASYNC")
         ) ODDR_inst_clk (
             .Q(phy_tx_clk),
             .C(clk_sys),
             .CE(1'b1),
             .D1(1'b1),
             .D2(1'b0),
             .R(1'b0),
             .S(1'b0)
         );

    genvar i;
    generate
        for (i = 0 ; i < 4; i = i + 1) begin : tx_oddr_gen
            ODDR #(
                     .DDR_CLK_EDGE("SAME_EDGE"),
                     .INIT(1'b0),
                     .SRTYPE("ASYNC")
                 ) ODDR_inst_txd (
                     .Q(phy_txd[i]),
                     .C(clk_sys),
                     .CE(1'b1),
                     .D1(rgmii_txd[i]),
                     .D2(rgmii_txd[i + 4]),
                     .R(rst_hi),
                     .S(1'b0)
                 );
        end
    endgenerate

    ODDR #(
             .DDR_CLK_EDGE("SAME_EDGE"),
             .INIT(1'b0),
             .SRTYPE("ASYNC")
         ) ODDR_inst_ctrl (
             .Q(phy_tx_ctl),
             .C(clk_sys),
             .CE(1'b1),
             .D1(rgmii_txdv),
             .D2(1'b0),
             .R(rst_hi),
             .S(1'b0)
         );

    genvar j;
    generate
        for (j = 0 ; j < 4; j = j + 1) begin : rx_iddr_gen
            IDDR #(
                     .DDR_CLK_EDGE("SAME_EDGE_PIPELINED"),
                     .INIT_Q1(1'b0),
                     .INIT_Q2(1'b0),
                     .SRTYPE("ASYNC")
                 ) IDDR_inst_rxd (
                     .Q1(rgmii_rxd[j]),
                     .Q2(rgmii_rxd[j + 4]),
                     .C(phy_rx_clk),
                     .CE(1'b1),
                     .D(phy_rxd[j]),      // 位选！原来接的是整条 4bit 总线
                     .R(rst_hi),
                     .S(1'b0)
                 );
        end
    endgenerate

    IDDR #(
             .DDR_CLK_EDGE("SAME_EDGE_PIPELINED"),
             .INIT_Q1(1'b0),
             .INIT_Q2(1'b0),
             .SRTYPE("ASYNC")
         ) IDDR_inst_ctrl (
             .Q1(rgmii_rxdv),
             .Q2(rgmii_err),
             .C(phy_rx_clk),
             .CE(1'b1),
             .D(phy_rx_ctl),
             .R(rst_hi),
             .S(1'b0)
         );

endmodule
