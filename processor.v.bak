module processor (
	input [9:0] SW,
	input [1:0] KEY,
	input CLOCK_50,

	output [6:0] HEX5,
	output [6:0] HEX4,
	output [6:0] HEX3,
	output [6:0] HEX2,
	output [6:0] HEX1,
	output [6:0] HEX0,
	output [9:0] LEDR 
	);

    reg                 processorCLOCK;

    // Control Wires
    wire                PCwrite, AddrSel, MemRead, MemWrite, IRload, MDRload, RASel, RFWrite;
    wire                ABLD, ALU_A, RegIn, FlagWrite, ALUoutLD;
    wire    [2:0]       ALU_B, ALUop;

    // Datapath Output / FSM Input
    wire    [3:0]       instruction;
    wire    [7:0]       IRout;
    wire                N, Z;

    // Datapath Internal Wires
    wire    [7:0]       ALU_PC, PC_Addr, ADDR, MemOut;
    wire    [1:0]       IR_A, RAmux, IR_B;
    wire    [7:0]       MDRout, Imm4, Imm5, Imm2, dataA, dataB, dataAr, dataBr;
    wire    [7:0]       RegIn_W, ALU_Ain, ALU_Bin, ALU_RegIn;
    wire                ALU_N, ALU_Z;


    // FSM Internal Wires
    wire Reset;
    wire [2:0] currState, nextState;
    wire done;

    // Other Wires
    wire [9:0] LEDs;

    datapath Datapath(
        // Inputs
        .CLOCK_50(processorCLOCK),
        .PCwrite(PCwrite),
        .AddrSel(AddrSel),
        .MemRead(MemRead),
        .MemWrite(MemWrite),
        .IRload(IRload),
        .MDRload(MDRload),
        .RASel(RASel),
        .RFWrite(RFWrite),
        .ABLD(ABLD),
        .ALU_A(ALU_A),
        .ALU_B(ALU_B),
        .RegIn(RegIn),
        .ALUop(ALUop),
        .FlagWrite(FlagWrite),
        .ALUoutLD(ALUoutLD),  

        // Outputs
        .instruction(instruction),
        .IRout(IRout),
        .N(N),
        .Z(Z),

        // Internal Wires/Reg (set as output for debugging)
        .ALU_PC(ALU_PC),
        .PC_Addr(PC_Addr),
        .ADDR(ADDR),
        .MemOut(MemOut),
        .IR_A(),
        .RAmux(),
        .IR_B(),
        .MDRout(),
        .Imm4(Imm4),
        .Imm5(),
        .Imm2(),
        .dataA(),
        .dataB(),
        .dataAr(),
        .dataBr(),
        .RegIn_W(),
        .ALU_Ain(),
        .ALU_Bin(),
        .ALU_N(),
        .ALU_Z(),
        .ALU_RegIn()
    );

    FSM FiniteStateMachine(
        // Inputs
        .CLOCK_50(processorCLOCK),
        .Reset(Reset),
        .instruction(instruction),
        .IRout(IRout),
        .N(N),
        .Z(Z),

        // Outputs
        .PCwrite(PCwrite),
        .AddrSel(AddrSel),
        .MemRead(MemRead),
        .MemWrite(MemWrite),
        .IRload(IRload),
        .MDRload(MDRload),
        .RASel(RASel),
        .RFWrite(RFWrite),
        .ABLD(ABLD),
        .ALU_A(ALU_A),
        .ALU_B(ALU_B),
        .RegIn(RegIn),
        .ALUop(ALUop),
        .FlagWrite(FlagWrite),
        .ALUoutLD(ALUoutLD),  

        // Internal Wires/Reg (set as output for debugging)
        .currState(currState),
        .nextState(nextState),
        .done(done)
    );

    // CLOCK Divider

    reg     [31:0]      counter;

    always @ (posedge CLOCK_50) begin
        if (Reset) begin
            counter <= 0;
        end
        else if (counter >= 10000000) begin
            counter <= 0;
            processorCLOCK = ~processorCLOCK;
        end
        else begin  
            counter <= counter + 1;
        end
    end
    
    assign Reset = ~KEY[1];
    assign enable = 1;
    assign LEDs = {N, Z, MemRead, MemWrite, IRload, 4'b0, done};

	reg_LED REGLED (.CLOCK_50(CLOCK_50), .EN(enable), .Q(LEDs), .LEDR(LEDR[9:0]));
	
	reg_HEX H5(.CLOCK_50(CLOCK_50), .EN(enable), .hex(PC_Addr[7:4]), .display(HEX5));
	reg_HEX H4(.CLOCK_50(CLOCK_50), .EN(enable), .hex(PC_Addr[3:0]), .display(HEX4));
	reg_HEX H3(.CLOCK_50(CLOCK_50), .EN(enable), .hex(IRout[7:4]), .display(HEX3));
	reg_HEX H2(.CLOCK_50(CLOCK_50), .EN(enable), .hex(IRout[3:0]), .display(HEX2));
	reg_HEX H1(.CLOCK_50(CLOCK_50), .EN(enable), .hex(ALUop[2:0]), .display(HEX1));
	reg_HEX H0(.CLOCK_50(CLOCK_50), .EN(enable), .hex({1'b0, currState[2:0]}), .display(HEX0));
endmodule



module datapath (
    // Inputs
    input                   CLOCK_50,
    input                   PCwrite,
    input                   AddrSel,
    input                   MemRead,
    input                   MemWrite,
    input                   IRload,
    input                   MDRload,
    input                   RASel,
    input                   RFWrite,
    input                   ABLD,
    input                   ALU_A,
    input       [2:0]       ALU_B,
    input                   RegIn,
    input       [2:0]       ALUop,
    input                   FlagWrite,
    input                   ALUoutLD,  

    // Outputs
    output      [3:0]       instruction,
    output      [7:0]       IRout,
    output reg              N,
    output reg              Z,

    // Internal Wires/Reg (set as output for debugging)
    output      [7:0]       ALU_PC,
    output      [7:0]       PC_Addr,
    output reg  [7:0]       ADDR,
    output      [7:0]       MemOut,
    output      [1:0]       IR_A,
    output reg  [1:0]       RAmux,
    output      [1:0]       IR_B,
    output reg  [7:0]       MDRout,
    output      [7:0]       Imm4,
    output      [7:0]       Imm5,
    output      [7:0]       Imm2,
    output      [7:0]       dataA,
    output      [7:0]       dataB,
    output reg  [7:0]       dataAr,
    output reg  [7:0]       dataBr,
    output reg  [7:0]       RegIn_W,
    output reg  [7:0]       ALU_Ain,
    output reg  [7:0]       ALU_Bin,
    output                  ALU_N,
    output                  ALU_Z,
    output reg  [7:0]       ALU_RegIn
    );

    // Program Counter
    PC ProgramCounter (
        .CLK(CLOCK_50),
        .PCin(ALU_PC),
        .PCwrite(PCwrite),
        .PCout(PC_Addr)
    );

    // Address Select
    always @ (*) begin
        if (AddrSel) begin
            ADDR = PC_Addr;
        end
        else begin
            ADDR = dataBr;
        end
    end

    // Memory
    memory # (.INIT_FILE("machine_code.txt")) Memory (
        .CLK(CLOCK_50),
        .MemRead(MemRead),
        .MemWrite(MemWrite),
        .ADDR(ADDR),
        .Data_in(dataAr),
        .Data_out(MemOut)
    );

    // Instruction Register
    IR InstructionRegister (
        .CLK(CLOCK_50),
        .IRin(MemOut),
        .IRload(IRload),

        .RA(IR_A),
        .RB(IR_B),
        .instruction(instruction),
        .Imm4SE(Imm4),
        .Imm5ZE(Imm5),
        .Imm2ZE(Imm2),
        .IRout(IRout)  
    );

    // Memory Data Register
    initial begin
        MDRout = 0;
    end

    always @ (posedge CLOCK_50) begin
        if (MDRload) begin
            MDRout <= MemOut;
        end
    end

    // RASel Mux
    always @ (*) begin
        if (RASel) begin
            RAmux = 2'b01;
        end
        else begin
            RAmux = IR_A;
        end
    end

    // Register File
    register_file RegisterFile( 
        .CLOCK_50(CLOCK_50),
        .RFWrite(RFWrite),
        .regA(RAmux),
        .regB(IR_B),
        .regW(RAmux),
        .dataW(RegIn_W),

        .dataA(dataA),
        .dataB(dataB)
	);

    // A and B Load Registers
    initial begin
        dataAr = 0;
        dataBr = 0;
    end
    always @ (posedge CLOCK_50) begin
        if (ABLD) begin
            dataAr <= dataA;
            dataBr <= dataB;
        end
    end

    // RegIn Mux
    always @ (*) begin
        if (RegIn) begin
            RegIn_W = MDRout;
        end
        else begin
            RegIn_W = ALU_RegIn;
        end
    end

    // ALU_A Mux
    always @ (*) begin
        if (ALU_A) begin
            ALU_Ain = dataAr;
        end
        else begin
            ALU_Ain = PC_Addr;
        end
    end

    // ALU_B Mux
    always @ (*) begin
        case (ALU_B)
            3'b000:     ALU_Bin = dataBr;
            3'b001:     ALU_Bin = 8'b00000001;
            3'b010:     ALU_Bin = Imm4;
            3'b011:     ALU_Bin = Imm5;
            3'b100:     ALU_Bin = Imm2;
            default:    ALU_Bin = 0;
        endcase
    end

    // Arithmethic Logic Unit
    ALU ALU(
        .ALUop(ALUop),
        .A(ALU_Ain),
        .B(ALU_Bin),
        .N(ALU_N),
        .Z(ALU_Z),
        .ALUout(ALU_PC)
    );

    // ALU out Register
    initial begin
        ALU_RegIn = 0;
    end
    always @ (posedge CLOCK_50) begin
        if (ALUoutLD) begin
            ALU_RegIn <= ALU_PC;
        end
    end

    // FlagWrite Register
    initial begin
        N = 0;
        Z = 0;
    end
    always @ (posedge CLOCK_50) begin
        if (FlagWrite) begin
            N <= ALU_N;
            Z <= ALU_Z;
        end
    end
endmodule

module FSM (
    // Inputs
    input                   CLOCK_50,
    input                   Reset,
    input       [3:0]       instruction,
    input       [7:0]       IRout,
    input                   N,
    input                   Z,

    // Outputs
    output reg              PCwrite,
    output reg              AddrSel,
    output reg              MemRead,
    output reg              MemWrite,
    output reg              IRload,
    output reg              MDRload,
    output reg              RASel,
    output reg              RFWrite,
    output reg              ABLD,
    output reg              ALU_A,
    output reg  [2:0]       ALU_B,
    output reg              RegIn,
    output reg  [2:0]       ALUop,
    output reg              FlagWrite,
    output reg              ALUoutLD,  

    // Internal Wires/Reg (set as output for debugging)
    output reg  [2:0]       currState,
    output reg  [2:0]       nextState,
    output reg              done
    );

    parameter IDLE = 3'b000, CYCLE1 = 3'b001, CYCLE2 = 3'b010, CYCLE3 = 3'b011, CYCLE4 = 3'b100, CYCLE5 = 3'b101;
    
    // Instruction OpCodes (ORi, SHIFT are 3 bit)
    parameter ADD = 4'b0100, SUB = 4'b0110, NAND = 4'b1000, ORi = 3'b111, LOAD = 4'b0000;
    parameter STORE = 4'b0010, BNZ = 4'b1001, BPZ = 4'b0101, BZ = 4'b1010, SHIFT = 3'b011;
    parameter SLEFT = 1'b1, SRIGHT = 1'b0, J = 4'b0001;

    initial begin
        currState = IDLE;
    end

    // Output Logic
    always @ (*) begin

        PCwrite = 0;  AddrSel = 0;  MemRead = 0;  
        MemWrite = 0;  IRload = 0;  MDRload = 0;     
        RASel = 0;  RFWrite = 0;  RegIn = 0;
        ABLD = 0;  ALU_A = 0;  ALU_B = 0; 
        ALUop = 0;  FlagWrite = 0;  ALUoutLD = 0;
        done = 0;

        nextState = currState;

        case(currState)
            IDLE: begin
                MemRead = 1;
                AddrSel = 1;
                nextState = CYCLE1;
            end

            CYCLE1: begin
                    // IR <- Mem(PC)
                    AddrSel = 1;
                    MemRead = 1;
                    IRload = 1;

                    // PC <- PC + 1
                    ALU_A = 0;
                    ALU_B = 3'b001;
                    ALUop = 3'b000;
                    PCwrite = 1;

                    nextState = CYCLE2;
            end

            CYCLE2: begin
                    // regA <- IR[7:6], A/B <- RF DataA/B 
                    RASel = 0;
                    ABLD = 1;

                    nextState = CYCLE3;
            end

            CYCLE3: begin
                if (instruction == ADD | instruction == SUB | instruction == NAND) begin
                    ALU_A = 1;       // Select reg A
                    ALU_B = 3'b000;  // Select reg B
                    case (instruction) 
                        ADD: ALUop = 3'b000;
                        SUB: ALUop = 3'b001;
                        NAND: ALUop = 3'b011;
                        default: ALUop = 3'b000;
                    endcase
                    ALUoutLD = 1;
                    FlagWrite = 1;

                    nextState = CYCLE4;
                end 

                if (instruction[2:0] == SHIFT) begin
                    ALU_A = 1;       // Select reg A
                    ALU_B = 3'b100;  // ALU B <- ZE(Imm2)
                    ALUoutLD = 1;
                    FlagWrite = 1;
                    case (IRout[5]) 
                        SLEFT: ALUop = 3'b100;
                        SRIGHT: ALUop = 3'b101;
                        default: ALUop = 3'b100;
                    endcase

                    nextState = CYCLE4;
                end 

                if (instruction == LOAD) begin
                    AddrSel = 0;    // Select B to be address
                    MDRload = 1;    // Output of memory to MDR
                    MemRead = 1;    // Read memory

                    nextState = CYCLE4;
                end

                if (instruction == STORE) begin
                    AddrSel = 0;    // Select B to be address
                    MemWrite = 1;    // Write to memory
                    done = 1;

                    nextState = CYCLE4;
                end

                if (instruction == BNZ | instruction == BPZ | instruction == BZ | instruction == J) begin
                    ALU_A = 0; // ALU A <- PC
                    ALU_B = 3'b010; // ALU B <- SE(Imm4)
                    ALUop = 3'b000;
                    case (instruction) 
                        BPZ: if (!N) PCwrite = 1;
                        BZ: if (Z) PCwrite = 1;
                        BNZ: if (!Z) PCwrite = 1;
                        J: PCwrite = 1;
                    endcase

                    done = 1;
                    nextState = CYCLE1;
                end

                if (instruction[2:0] == ORi) begin
                    RASel = 1; // RA <- 2'b01

                    nextState = CYCLE4;
                end

            end

            CYCLE4: begin
                if (instruction == ADD | instruction == SUB | instruction == NAND | instruction[2:0] == SHIFT) begin
                    RegIn = 0;      // dataW <- ALU
                    RFWrite = 1;

                    done = 1;
                    nextState = CYCLE1;
                end 

                if (instruction == LOAD) begin
                    RegIn = 1;      // dataW <- MDR
                    RFWrite = 1;

                    done = 1;
                    nextState = CYCLE1;
                end

                if (instruction[2:0] == ORi) begin
                    ALU_A = 1;       // Select reg A
                    ALU_B = 3'b011;  // ALU B <- ZE(Imm5)
                    ALUoutLD = 1;
                    FlagWrite = 1;
                    ALUop = 3'b010;

                    nextState = CYCLE5;
                end
            end 

            CYCLE5: begin // only for ORi
                RegIn = 0;      // dataW <- ALU
                RFWrite = 1;

                done = 1;
                nextState =CYCLE1;
            end
                
            default: nextState = currState;
        endcase
    end

    // FSM Flip Flops

    always @ (posedge CLOCK_50) begin
        if (Reset) begin
            currState <= IDLE;
        end
        else begin
            currState <= nextState;
        end
    end

endmodule

module memory # (
    // Parameters
    parameter   INIT_FILE = ""
    )(
    input                   CLK,
    input                   MemRead,
    input                   MemWrite,
    input       [7:0]       ADDR,
    input       [7:0]       Data_in,
    output reg  [7:0]       Data_out
    );

    reg [7:0] mem [0:255];      // Internal Memory

    // Initialization
    integer i;
    initial begin
        if (INIT_FILE) begin
        $readmemb(INIT_FILE, mem);
        for (i = 0; i < 16; i = i + 1) 
            $display("mem[%0d] = %b", i, mem[i]);
        end
    end

    always @ (posedge CLK) begin
        if (MemWrite) begin
            mem[ADDR] <= Data_in;
        end
    end

    always @ (*) begin
        if (MemRead) begin
            Data_out = mem[ADDR];
        end
        else begin
            Data_out = 0;
        end
    end
endmodule

module register_file (
	input                   CLOCK_50,
	input                   RFWrite,
	input       [1:0]       regA,
	input       [1:0]       regB,
	input       [1:0]       regW,
	input       [7:0]       dataW,

	output reg  [7:0]       dataA,
	output reg  [7:0]       dataB
	);

	reg [7:0] r0, r1, r2, r3;
	parameter R0 = 2'b00, R1 = 2'b01, R2 = 2'b10, R3 = 2'b11;

	initial begin
		r0 = 0;
		r1 = 0;
		r2 = 0;
		r3 = 0;
	end	

	always @ (posedge CLOCK_50) begin
		if (RFWrite) begin
			case(regW[1:0])
				R0: r0 <= dataW;
				R1: r1 <= dataW;
				R2: r2 <= dataW;
				R3: r3 <= dataW;
			endcase
		end
	end

	always @ (*) begin
			case(regA[1:0])
				R0: dataA = r0;
				R1: dataA = r1;
				R2: dataA = r2;
				R3: dataA = r3;
			endcase

			case(regB[1:0])
				R0: dataB = r0;
				R1: dataB = r1;
				R2: dataB = r2;
				R3: dataB = r3;
			endcase
	end
endmodule

module PC (
    input                   CLK,
    input       [7:0]       PCin,
    input                   PCwrite,
    output reg  [7:0]       PCout
    );

    initial begin
        PCout = 0;
    end

    always @(posedge CLK) begin
        if (PCwrite) begin
            PCout <= PCin;
        end
    end
endmodule

module IR (
    input                   CLK,
    input       [7:0]       IRin,
    input                   IRload,

    output      [1:0]       RA,
    output      [1:0]       RB,
    output      [3:0]       instruction,
    output      [7:0]       Imm4SE,
    output      [7:0]       Imm5ZE,
    output      [7:0]       Imm2ZE,
    output      [7:0]       IRout  
    );

    reg         [7:0]       IRreg;
    
    always @ (posedge CLK) begin
        if (IRload) begin
            IRreg <= IRin;
        end
    end

    assign IRout = IRreg;
    assign RA = IRreg[7:6];
    assign RB = IRreg[5:4];
    assign instruction = IRreg[3:0];
    assign Imm4SE = {{4{IRreg[7]}}, IRreg[7:4]};
    assign Imm5ZE = {3'b000, IRreg[7:3]};
    assign Imm2ZE = {6'b000000, IRreg[4:3]};
endmodule

module ALU(
    input       [2:0]       ALUop,
    input       [7:0]       A,
    input       [7:0]       B,
    output                  N,
    output                  Z,
    output reg  [7:0]       ALUout
    );

    parameter ADD = 3'b000, SUB = 3'b001, OR = 3'b010, NAND = 3'b011, ShiftLeft = 3'b100, ShiftRight = 3'b101;
    always @ (*) begin
        case(ALUop) 
            ADD:        ALUout = A + B;
            SUB:        ALUout = A - B;
            OR:         ALUout = A | B;
            NAND:       ALUout = ~(A & B);
            ShiftLeft:  ALUout = A << B;
            ShiftRight: ALUout = A >> B;
            default:    ALUout = 8'b0;
        endcase
    end

    assign N = ALUout[7];
    assign Z = ~(|ALUout);
endmodule

module reg_LED(input CLOCK_50, input EN, input [9:0] Q, output reg [9:0] LEDR);
	always @ (posedge CLOCK_50) begin
		if (EN)
			LEDR <= Q;
		else
			LEDR <= LEDR;
	end
endmodule

module reg_HEX(input CLOCK_50, input EN, input [3:0] hex, output reg [6:0] display);
	wire [6:0] data;
	hex7seg SEG(.hex(hex), .display(data));
	always @ (posedge CLOCK_50) begin
		if (EN)
			display <= data;
		else
			display <= display;
	end
endmodule	

module hex7seg (hex, display);
    input [3:0] hex;
    output [6:0] display;

    reg [6:0] display;

    /*
     *       0  
     *      ---  
     *     |   |
     *    5|   |1
     *     | 6 |
     *      ---  
     *     |   |
     *    4|   |2
     *     |   |
     *      ---  
     *       3  
     */
    always @ (hex)
        case (hex)
            4'h0: display = 7'b1000000;
            4'h1: display = 7'b1111001;
            4'h2: display = 7'b0100100;
            4'h3: display = 7'b0110000;
            4'h4: display = 7'b0011001;
            4'h5: display = 7'b0010010;
            4'h6: display = 7'b0000010;
            4'h7: display = 7'b1111000;
            4'h8: display = 7'b0000000;
            4'h9: display = 7'b0011000;
            4'hA: display = 7'b0001000;
            4'hB: display = 7'b0000011;
            4'hC: display = 7'b1000110;
            4'hD: display = 7'b0100001;
            4'hE: display = 7'b0000110;
            4'hF: display = 7'b0001110;
        endcase
endmodule
	
	

	