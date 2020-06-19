/*
 * This IP is the arduboy TOP simulation.
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
 
module top_sim(

    );

reg clk = 1;
always	#(5)	clk	<=	~clk;

initial begin
	wait(clk);
	#50000;
	$finish;
end

top top_inst(
`ifndef ARDU_FPGA_ICE40UP5K_GAME
	.clk(clk),
`endif
	.RGB0(), 
	.RGB1(), 
	.RGB2(), 
	.OLED_DC(),
	.OLED_SS(),
	.OLED_RST(),
	.SCK(),
	.MOSI(),
	.MISO(),
	.BTN_UP(),
	.BTN_DN(),
	.BTN_BACK(),
	.BTN_OK(),
	.BTN_INTERRUPT(),
	.DES_SS(),
	.uSD_SS(),
	.uSD_CD(),
	.APP_SS(),
	.ADC_SS(),
	.UART_TX(),
	.UART_RX(),
	.D0_P(),
	.D0_N(),
	.D1_P(),
	.D1_N(),
	.D2_P(),
	.D2_N(),
	.D3_P(),
	.D3_N(),
	.D4_P(),
	.D4_N(),
	.D5_P(),
	.D5_N(),
	.D6_P(),
	.D6_N(),
	.D7_P(),
	.D7_N()
);
endmodule
