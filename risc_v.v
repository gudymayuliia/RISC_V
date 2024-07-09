module risc_v(
    input  CLK,        // system clock 
    input  RESET,      // reset button
    output [4:0] LEDS, // system LEDs
    input  RXD,        // UART receive
    output TXD         // UART transmit
);

   
   reg [31:0] MEM [0:255]; //memory
   reg [31:0] PC;       // program counter
   reg [31:0] instr;    // current instruction
   
   reg [4:0] leds;
   assign LEDS = leds;
   `include "risc_assembly.v"
   //instuction decoder
   initial begin
      PC = 0;
      ADD(x0,x0,x0);
      ADD(x1,x0,x0);
      ADDI(x1,x1,1);
      ADDI(x1,x1,1);
      ADDI(x1,x1,1);
      ADDI(x1,x1,1);
      ADD(x2,x1,x0);
      ADD(x3,x1,x2);
      SRLI(x3,x3,3);
      SLLI(x3,x3,31);
      SRAI(x3,x3,5);
      SRLI(x1,x3,26);
      EBREAK();
   end

   // The 10 RISC-V instructions
   wire isALUreg  =  (instr[6:0] == 7'b0110011); // rd <- rs1 OP rs2   
   wire isALUimm  =  (instr[6:0] == 7'b0010011); // rd <- rs1 OP Iimm
   wire isBranch  =  (instr[6:0] == 7'b1100011); // if(rs1 OP rs2) PC<-PC+Bimm
   wire isJALR    =  (instr[6:0] == 7'b1100111); // rd <- PC+4; PC<-rs1+Iimm
   wire isJAL     =  (instr[6:0] == 7'b1101111); // rd <- PC+4; PC<-PC+Jimm
   wire isAUIPC   =  (instr[6:0] == 7'b0010111); // rd <- PC + Uimm
   wire isLUI     =  (instr[6:0] == 7'b0110111); // rd <- Uimm   
   wire isLoad    =  (instr[6:0] == 7'b0000011); // rd <- mem[rs1+Iimm]
   wire isStore   =  (instr[6:0] == 7'b0100011); // mem[rs1+Simm] <- rs2
   wire isSYSTEM  =  (instr[6:0] == 7'b1110011); // special

   // The 5 immediate formats
   wire [31:0] Uimm={    instr[31],   instr[30:12], {12{1'b0}}};
   wire [31:0] Iimm={{21{instr[31]}}, instr[30:20]};
   wire [31:0] Simm={{21{instr[31]}}, instr[30:25],instr[11:7]};
   wire [31:0] Bimm={{20{instr[31]}}, instr[7],instr[30:25],instr[11:8],1'b0};
   wire [31:0] Jimm={{12{instr[31]}}, instr[19:12],instr[20],instr[30:21],1'b0};

   // Source and destination registers
   wire [4:0] rs1Id = instr[19:15];
   wire [4:0] rs2Id = instr[24:20];
   wire [4:0] rdId  = instr[11:7];

   // function codes
   wire [2:0] funct3 = instr[14:12];
   wire [6:0] funct7 = instr[31:25];
   
   // register bank
   reg [31:0] RegisterBank [0:31];
   reg [31:0] rs1;
   reg [31:0] rs2;
   wire [31:0] writeBackData;
   wire        writeBackEn;
   assign writeBackData = 0; // for now
   assign writeBackEn = 0;   // for now
   
   // The ALU
   wire [31:0] aluIn1 = rs1; //first input
   wire [31:0] aluIn2 = isALUreg ? rs2 : Iimm; //second input, register or immidiate value
   reg [31:0] aluOut; //store the computed value
   wire [4:0] shamt = isALUreg ? rs2[4:0] : instr[24:20]; // shift amount

   always @(*) begin
      case(funct3)
	3'b000: aluOut = (funct7[5] & instr[5]) ? (aluIn1 - aluIn2) : (aluIn1 + aluIn2); //sub or add, depends on func7
	3'b001: aluOut = aluIn1 << shamt; //left shift
	3'b010: aluOut = ($signed(aluIn1) < $signed(aluIn2)); //signed comparison
	3'b011: aluOut = (aluIn1 < aluIn2);//unsigned comparison
	3'b100: aluOut = (aluIn1 ^ aluIn2); //xor
	3'b101: aluOut = funct7[5]? ($signed(aluIn1) >>> shamt) : ($signed(aluIn1) >> shamt); //arithmetic/logical right shift
	3'b110: aluOut = (aluIn1 | aluIn2); //or
	3'b111: aluOut = (aluIn1 & aluIn2);	//and
      endcase
   end
   
   //state machine for implementing 3 steps:
   //fetching the instruction from memory
   //fetching the values of source registers 
   //computing source registers based on operation
   
   localparam FETCH_INSTR = 0;
   localparam FETCH_REGS  = 1;
   localparam EXECUTE     = 2;
   reg [1:0] state = FETCH_INSTR;
   
   
   assign writeBackData = aluOut; 
   assign writeBackEn = (state == EXECUTE && (isALUreg || isALUimm));
   
   //implementing 4th step - storing the reult in destination register
   always @(posedge clk) begin
      if(RESET) begin
	   PC    <= 0;
	   state <= FETCH_INSTR;
	   instr <= 32'b0000000_00000_00000_000_00000_0110011; // NOP
      end 
      else begin
	   if(writeBackEn && rdId != 0) begin
	   RegisterBank[rdId] <= writeBackData;
	 end
   
    //state machine
	 case(state)
	   FETCH_INSTR: begin
	      instr <= MEM[PC[31:2]];
	      state <= FETCH_REGS;
	   end
	   FETCH_REGS: begin
	      rs1 <= RegisterBank[rs1Id];
	      rs2 <= RegisterBank[rs2Id];
	      state <= EXECUTE;
	   end
	   EXECUTE: begin
	      PC <= PC + 4;
	      state <= FETCH_INSTR;	      
	   end
	 endcase
      end 
   end 
   

   assign TXD  = 1'b0; // not used for now   
endmodule
