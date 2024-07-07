`timescale 1ns / 1ps

module blinky(
    input clk, reset,
    output [4:0] leds,
    input rx,
    output tx
    );

reg [4:0] pc = 0;
reg [4:0] count =0;
reg [4:0] memory [0:20];

initial begin
       memory[0]  = 5'b00000;
       memory[1]  = 5'b00001;
       memory[2]  = 5'b00010;
       memory[3]  = 5'b00100;
       memory[4]  = 5'b01000;
       memory[5]  = 5'b10000;
       memory[6]  = 5'b10001;
       memory[7]  = 5'b10010;
       memory[8]  = 5'b10100;
       memory[9]  = 5'b11000;
       memory[10] = 5'b11001;
       memory[11] = 5'b11010;
       memory[12] = 5'b11100;
       memory[13] = 5'b11101;
       memory[14] = 5'b11110;
       memory[15] = 5'b11111;
       memory[16] = 5'b11110;
       memory[17] = 5'b11100;
       memory[18] = 5'b11000;
       memory[19] = 5'b10000;
       memory[20] = 5'b00000;       
end


always @(posedge clk) begin
    count <= memory[pc];
    pc <= (reset || pc == 20) ? 0 : pc +1;
end   

assign leds = count;
assign tx = 1'b0;    

endmodule
