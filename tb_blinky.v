`timescale 1ns / 1ps

module tb_blinky();
   reg clk;
   wire reset = 0; 
   wire [4:0] leds;
   reg  rx = 1'b0;
   wire tx;

   blinky uut(
     .clk(clk),
     .reset(reset),
     .leds(leds),
     .rx(rx),
     .tx(tx)
   );

   
   
   always begin
   clk = 1'b0;
   #10;
   clk = 1'b1;
   #10;
   end
   
   initial begin
	$monitor("LEDS = %b",leds);
   end


endmodule   
