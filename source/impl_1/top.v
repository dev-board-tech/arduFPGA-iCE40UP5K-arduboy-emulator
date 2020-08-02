/*
 * This IP is the MEGA/XMEGA TOP implementation.
 * 
 * Copyright (C) 2020  Iulian Gheorghiu (morgoth@devboard.tech)
 * 
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */
 
`timescale 1ns / 1ps

`include "mega-def.v"

`define REV							"1.1"
`define USE_PLL						"TRUE"

`define PLATFORM					"iCE40UP"
`define FLASH_ROM_FILE_NAME			"l1_boot_ld"

module top(
	input clk,
	output RGB0, 
	output RGB1, 
	output RGB2, 
	output OLED_DC,
	output OLED_SS,
	output OLED_RST,
	output SCK,
	inout MOSI,
	inout MISO,
	input BTN_UP,
	input BTN_DN,
	input BTN_BACK,
	input BTN_OK,
	input BTN_INTERRUPT,
	output DES_SS,
	output uSD_SS,
	input uSD_CD,
	output APP_SS,
	output ADC_SS,
	output UART_TX,
	input UART_RX,
	inout D0_P,
	inout D0_N,
	inout D1_P,
	inout D1_N,
	inout D2_P,
	inout D2_N,
	inout D3_P,
	inout D3_N,
	inout D4_P,
	inout D4_N,
	inout D5_P,
	inout D5_N,
	inout D6_P,
	inout D6_N,
	inout D7_P,
	inout D7_N
	);

wire pll_locked;
reg [3:0]pll_locked_buf;
wire from_pll_clk;
wire sys_clk;
wire sys_rst = ~pll_locked_buf[3];
wire pll_clk = sys_clk;
wire sys_clk_int;
reg [1:0]sys_clk_t;


always @ (posedge sys_clk)
begin
	pll_locked_buf <= {pll_locked_buf[2:0], pll_locked}; 
end

PLL_DEV_16M PLL_inst(
	.ref_clk_i(clk), 
	.bypass_i(1'b0),
	.rst_n_i(1'b1), 
	.lock_o(pll_locked), 
	.outcore_o(sys_clk_int), 
	.outglobal_o(from_pll_clk) 
);

assign sys_clk = `USE_PLL == "TRUE" ? from_pll_clk : clk;

//always @ (posedge sys_clk_int) sys_clk_t <= sys_clk_t + 2'h1;
 
wire [7:0]io_addr;
wire [7:0]io_out;
wire io_write;
wire [7:0]io_in;
wire io_read;
wire io_rst;

wire ssd1306_scl;
wire ssd1306_dc;
wire ssd1306_ss;
wire ssd1306_rst;
wire [2:0]ld;

wire nmi_sig;
wire nmi_rst;

wire uc_miso;
wire uc_mosi;

wire sec_reg_rst;
wire sec_en;

reg BTN_INTERRUPT_reg, BTN_BACK_reg, BTN_OK_reg, BTN_UP_reg, BTN_DN_reg, D2_P_reg, D2_N_reg, D1_P_reg, D1_N_reg, D0_P_reg, D0_N_reg, uSD_CD_reg;
always @ (*)
begin
	//BTN_BACK_reg = BTN_BACK & D2_P;
	//BTN_OK_reg = BTN_OK & D2_N;
	//BTN_UP_reg = BTN_UP & D1_P;
	//BTN_DN_reg = BTN_DN & D1_N;
	BTN_INTERRUPT_reg = BTN_INTERRUPT;
	
	D2_P_reg = D2_P;
	D2_N_reg = D2_N;
	D1_P_reg = D1_P;
	D1_N_reg = D1_N;
	D0_P_reg = D0_P;
	D0_N_reg = D0_N;
	uSD_CD_reg = uSD_CD;
end

always @ (posedge sys_clk)
begin
	BTN_BACK_reg <= BTN_BACK & D2_P;
	BTN_OK_reg <= BTN_OK & D2_N;
	BTN_UP_reg <= BTN_UP & D1_P;
	BTN_DN_reg <= BTN_DN & D1_N;
	//BTN_INTERRUPT_reg <= BTN_INTERRUPT;
	
	//D2_P_reg <= D2_P;
	//D2_N_reg <= D2_N;
	//D1_P_reg <= D1_P;
	//D1_N_reg <= D1_N;
	//D0_P_reg <= D0_P;
	//D0_N_reg <= D0_N;
	//uSD_CD_reg <= uSD_CD;
end
 
atmega32u4_arduboy # (
	.PLATFORM(`PLATFORM),
	.BOOT_ADDR(16'hFC00),
	.ARDU_FPGA_ICE40UP5K_GAME("FALSE"),
	
	.CORE_TYPE(`MEGA_ENHANCED_128K),
	.ROM_ADDR_WIDTH(15), // 14 = 16K Words / 32K Bytes; 15 = 32K Words / 64K Bytes; 16 = 64K Words / 128K Bytes Not supported yet.
	.BOOT_ADDR_WIDTH(10), // 1024 Words / 2048 Bytes, how big the first stage boot-loader ROM to be.
	.BUS_ADDR_DATA_LEN(16), // Max 64K Bytes.
	.RAM_TYPE("SRAM"),  // "BLOCK","SRAM"// If "SRAM" is choosen, will be a 32KB block of RAM.
	.RAM_ADDR_WIDTH(15), // 32KB, if you use "SRAM" this value need to be 15.
	.EEP_ADDR_WIDTH(10), // 1K Bytes.
	.RESERVED_RAM_FOR_IO(12'h100), // Lowest 256 Bytes of RAM addresses are reserved for IO's.
	.VECTOR_INT_TABLE_SIZE(43),// 42 of original ATmega32U4 + NMI
	.WATCHDOG_CNT_WIDTH(0),//27 // We do not use watchdog, is not a critical design and most of arduboy games does not use him.

	.REGS_REGISTERED("FALSE"),
	.ROM_PATH(`FLASH_ROM_FILE_NAME),
	.USE_PIO_B("TRUE"),
	.USE_PIO_C("TRUE"),
	.USE_PIO_D("TRUE"),
	.USE_PIO_E("TRUE"),
	.USE_PIO_F("TRUE"),
	.USE_PLL("TRUE"),
	.USE_PLL_HI_FREQ("FALSE"),
	.USE_TIMER_0("TRUE"),
	.USE_TIMER_1("FALSE"),
	.USE_TIMER_3("TRUE"),
	.USE_TIMER_4("TRUE"),
	.USE_SPI_1("TRUE"),
	.USE_UART_1("TRUE"),
	.USE_EEPROM("TRUE"),
	.USE_RNG_AS_ADC("TRUE")
) atmega32u4_arduboy_inst (
	.core_rst(sys_rst),
	.dev_rst(sys_rst),
	.clk(sys_clk),
	.clk_pll(pll_clk),
	.nmi_sig(nmi_sig),
	.nmi_rst(nmi_rst),
	.sec_reg_rst(sec_reg_rst),
	.sec_en(sec_en),
    .buttons({D2_P_reg, D2_N_reg, D1_P_reg, D1_N_reg, D0_P_reg, D0_N_reg}),
    .RGB(ld),
    .Buzzer1(D3_P),
    .Buzzer2(D3_N),
    .OledDC(ssd1306_dc),
    .OledCS(ssd1306_ss),
    .OledRST(ssd1306_rst),
    .spi_scl(ssd1306_scl),
    .spi_mosi(uc_mosi),
	.spi_miso(uc_miso),
	.uSD_CS(uSD_SS),
	.uSD_CD(uSD_CD_reg),
	.ADC_CS(ADC_SS),
	.VS_RST(),
	.VS_xCS(),
	.VS_xDCS(),
	.VS_DREQ(),
	.uart_tx(UART_TX),
	.uart_rx(UART_RX),
	
	.io_addr(io_addr),
	.io_out(io_out),
	.io_write(io_write),
	.io_in(io_in),
	.io_read(io_read),
	.io_sel(io_sel),
	.io_rst(io_rst)
);


rtc #(
	.PERIOD_STATIC(16000),
	.CNT_SIZE(14)
	)rtc_inst(
	.rst_i(io_rst),
	.clk_i(sys_clk),
	.intr_o(nmi_sig),
	.int_ack_i(nmi_rst)
	);
 
wire [5:0]dummy_out_port_a;
wire [7:0]dat_pa_d_out;
atmega_pio # (
	.PLATFORM(`PLATFORM),
	.BUS_ADDR_DATA_LEN(8),
	.PORT_OUT_ADDR('h22),
	.DDR_ADDR('h21),
	.PIN_ADDR('h20),
	.PINMASK(8'b11111011),
	.PULLUP_MASK(8'b00000000),
	.PULLDN_MASK(8'b00000000),
	.INVERSE_MASK(8'b00000000),
	.OUT_ENABLED_MASK(8'b00000011),
	.INITIAL_OUTPUT_VALUE(8'b00000011),
	.INITIAL_DIR_VALUE(8'b00000011)
)pio_a(
	.rst_i(io_rst),
	.clk_i(sys_clk),
	.addr_i(io_addr[7:0]),
	.wr_i(io_write/* & sec_en*/),
	.rd_i(io_read),
	.bus_i(io_out),
	.bus_o(dat_pa_d_out),

	.io_i({BTN_UP_reg, BTN_DN_reg, BTN_BACK_reg, BTN_OK_reg, BTN_INTERRUPT_reg, 3'b000}),
	.io_o({dummy_out_port_a, APP_SS, DES_SS}),
	.pio_out_io_connect_o()
	);

generate

if(`REV == "1.0")
begin
	assign MOSI = ~DES_SS ? 1'bz : uc_mosi;
	assign MISO = ~DES_SS ? uc_mosi : 1'bz;
	assign uc_miso = ~DES_SS ? MOSI : MISO;
end
else
begin
	assign MOSI = uc_mosi;
	assign uc_miso = MISO;
end
endgenerate

assign io_in = dat_pa_d_out;

assign {OLED_DC, OLED_SS, OLED_RST, SCK} = {ssd1306_dc, ssd1306_ss, ssd1306_rst, ssd1306_scl};

BB_OD LED_B_Inst (
  .T_N (1'b1),  // I
  .I   (~ld[2]),  // I
  .O   (),  // O
  .B   (RGB2)   // IO
);
BB_OD LED_G_Inst (
  .T_N (1'b1),  // I
  .I   (~ld[1]),  // I
  .O   (),  // O
  .B   (RGB1)   // IO
);
BB_OD LED_R_Inst (
  .T_N (1'b1),  // I
  .I   (~ld[0]),  // I
  .O   (),  // O
  .B   (RGB0)   // IO
);
endmodule