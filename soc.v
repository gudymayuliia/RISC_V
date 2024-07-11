module soc (
    input  CLK,        // system clock 
    input  RESET,      // reset button
    input  RXD,        // UART receive
    output TXD         // UART transmit
);

   wire    clk;
   wire    resetn;

   memory RAM(
      .clk(clk),
      .mem_addr(mem_addr),
      .mem_rdata(mem_rdata),
      .mem_rstrb(mem_rstrb),
      .mem_wdata(mem_wdata),
      .mem_wmask(mem_wmask)
   );

   wire [31:0] mem_addr;
   wire [31:0] mem_rdata;
   wire mem_rstrb;
   wire [31:0] x1;

   risc_v CPU(
      .clk(clk),
      .resetn(resetn),		 
      .mem_addr(mem_addr),
      .mem_rdata(mem_rdata),
      .mem_rstrb(mem_rstrb),
      .mem_wdata(mem_wdata),
      .mem_wmask(mem_wmask),
      .x1(x1)		 
   );
  

   assign TXD  = 1'b0; // not used for now   

endmodule