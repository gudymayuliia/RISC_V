module risc_v(
    input  CLK,        // system clock 
    input  RESET,      // reset button
    output     [31:0] mem_addr, 
    input      [31:0] mem_rdata, 
    output 	      mem_rstrb,
    output [31:0] mem_wdata,
    output [3:0] mem_wmask,
    output reg [31:0] x1       // UART transmit
);

   reg [31:0] PC;       // program counter
   reg [31:0] instr;    // current instruction
   
   

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
   
   //load instructions
   wire [31:0] loadstore_addr = rs1 + (isStore? Simm :Iimm);
   wire [15:0] LOAD_halfword =
	       loadstore_addr[1] ? mem_rdata[31:16] : mem_rdata[15:0];

   wire  [7:0] LOAD_byte =
	       loadstore_addr[0] ? LOAD_halfword[15:8] : LOAD_halfword[7:0];
   //selecting among load word/halfword/byte
   wire mem_byteAccess     = funct3[1:0] == 2'b00;
   wire mem_halfwordAccess = funct3[1:0] == 2'b01;

   wire [31:0] LOAD_data =
         mem_byteAccess ? LOAD_byte     :
     mem_halfwordAccess ? LOAD_halfword :
                          mem_rdata     ;
   //sign expansion                       
   wire LOAD_sign = !funct3[2] & (mem_byteAccess ? LOAD_byte[7] : LOAD_halfword[15]);

   wire [31:0] LOAD_data =
         mem_byteAccess ? {{24{LOAD_sign}},     LOAD_byte} :
     mem_halfwordAccess ? {{16{LOAD_sign}}, LOAD_halfword} :
                          mem_rdata ;
   // The ALU
   wire [31:0] aluIn1 = rs1; //first input
   wire [31:0] aluIn2 = isALUreg | isBranch ? rs2 : Iimm; //second input, register or immidiate value
   wire [4:0] shamt = isALUreg ? rs2[4:0] : instr[24:20]; // shift amount
   reg [31:0]  aluOut;
   wire [31:0] aluPlus = aluIn1 + aluIn2;
   wire [32:0] aluMinus = {1'b1, ~aluIn2} + {1'b0,aluIn1} + 33'b1;
   wire        LT  = (aluIn1[31] ^ aluIn2[31]) ? aluIn1[31] : aluMinus[32];
   wire        LTU = aluMinus[32];
   wire        EQ  = (aluMinus[31:0] == 0);
   
   
   always @(*) begin
      case(funct3)
	3'b000: aluOut = (funct7[5] & instr[5]) ? aluMinus[31:0] : aluPlus; //sub or add, depends on func7
	3'b001: aluOut = aluIn1 << shamt; //left shift
	3'b010: aluOut = {31'b0, LT}; //signed comparison
	3'b011: aluOut = {31'b0, LTU};//unsigned comparison
	3'b100: aluOut = (aluIn1 ^ aluIn2); //xor
	3'b101: aluOut = funct7[5]? ($signed(aluIn1) >>> shamt) : ($signed(aluIn1) >> shamt); //arithmetic/logical right shift
	3'b110: aluOut = (aluIn1 | aluIn2); //or
	3'b111: aluOut = (aluIn1 & aluIn2);	//and
      endcase
   end
   
   // branch instructions
      reg takeBranch;
   always @(*) begin
      case(funct3)
	3'b000: takeBranch = EQ; //BEQ
	3'b001: takeBranch = !EQ; //BNE
	3'b100: takeBranch = LT; //BLT
	3'b101: takeBranch = !LT; //BGE
	3'b110: takeBranch = LTU; //BLTU
	3'b111: takeBranch = !LTU; // BGEU
	default: takeBranch = 1'b0;
      endcase
   end
   
   //state machine for implementing 3 steps:
   //fetching the instruction from memory
   //fetching the values of source registers 
   //computing source registers based on operation
   
   wire [31:0] PCplusImm = PC + ( instr[3] ? Jimm[31:0] :
				  instr[4] ? Uimm[31:0] :
				             Bimm[31:0] );
   wire [31:0] PCplus4 = PC+4;
   
   localparam FETCH_INSTR = 0;
   localparam WAIT_INSTR  = 1;
   localparam FETCH_REGS  = 2;
   localparam EXECUTE     = 3;
   localparam LOAD = 4;
   localparam WAIT_DATA = 5;
   localparam STORE = 6;
   
   reg [1:0] state = FETCH_INSTR;
   
   assign writeBackData = (isJAL || isJALR) ? PCplus4 :
			      isLUI         ? Uimm :
			      isAUIPC       ? PCplusImm : 
			                      aluOut;
   
   assign writeBackEn = (state == EXECUTE && !isBranch && !isStore && !isLoad) || (store ==WAIT_DATA);
   
   wire [31:0] nextPC = ((isBranch && takeBranch) || isJAL) ? PCplusImm  :	       
	                isJALR                              ? {aluPlus[31:1],1'b0}:
	                PCplus4;
   
   assign mem_wdata[ 7: 0] = rs2[7:0];
   assign mem_wdata[15: 8] = loadstore_addr[0] ? rs2[7:0]  : rs2[15: 8];
   assign mem_wdata[23:16] = loadstore_addr[1] ? rs2[7:0]  : rs2[23:16];
   assign mem_wdata[31:24] = loadstore_addr[0] ? rs2[7:0]  :
			     loadstore_addr[1] ? rs2[15:8] : rs2[31:24];
			 
			 
    wire [3:0] STORE_wmask =
	      mem_byteAccess      ?
	            (loadstore_addr[1] ?
		          (loadstore_addr[0] ? 4'b1000 : 4'b0100) :
		          (loadstore_addr[0] ? 4'b0010 : 4'b0001)
                    ) :
	      mem_halfwordAccess ?
	            (loadstore_addr[1] ? 4'b1100 : 4'b0011) :
              4'b1111;
   //implementing 4th step - storing the reult in destination register
   always @(posedge clk) begin
      if(RESET) begin
	   PC    <= 0;
	   state <= FETCH_INSTR;
      end 
      else begin
	   if(writeBackEn && rdId != 0) begin
	   RegisterBank[rdId] <= writeBackData;
	 end

    //state machine
	 case(state)
	   FETCH_INSTR: begin
	      state <= WAIT_INSTR;
	   end
	   WAIT_INSTR: begin
	   instr <= mem_rdata;
	   state <= FETCH_REGS;
	   end
	   FETCH_REGS: begin
	      rs1 <= RegisterBank[rs1Id];
	      rs2 <= RegisterBank[rs2Id];
	      state <= EXECUTE;
	   end
	   EXECUTE: begin
	      PC <= nextPC;
	      state <= FETCH_INSTR;	      
	   end
	   LOAD: begin
	       state <= WAIT_DATA;
	   end
	   WAIT_DATA: begin
	       state <= FETCH_INSTR;
	   end
	   STORE: begin
	       state <= FETCH_INSTR;
	   end
	 endcase
      end 
   end 
   
   assign mem_addr = (state == WAIT_INSTR || state == FETCH_INSTR) ? PC : loadstore_addr;
   assign mem_rstrb = (state == FETCH_INSTR || state == LOAD);
   assign mem_wmask = {4{(state == STORE)}} & STORE_wmask;
   
   
   

   assign TXD  = 1'b0; // not used for now   
endmodule
