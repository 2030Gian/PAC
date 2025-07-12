// arm_multi_complete.v
// Complete implementation of ARM multicycle processor with MUL and FPU support

module top (
    clk,
    reset,
    WriteData,
    Adr,
    MemWrite
);
    input wire clk;
    input wire reset;
    output wire [31:0] WriteData;
    output wire [31:0] Adr;
    output wire MemWrite;
    
    wire [31:0] ReadData;
    
    // instantiate processor and shared memory
    arm arm(
        .clk(clk),
        .reset(reset),
        .MemWrite(MemWrite),
        .Adr(Adr),
        .WriteData(WriteData),
        .ReadData(ReadData)
    );
    
    mem mem(
        .clk(clk),
        .we(MemWrite),
        .a(Adr),
        .wd(WriteData),
        .rd(ReadData)
    );
endmodule

module mem (
    clk,
    we,
    a,
    wd,
    rd
);
    input wire clk;
    input wire we;
    input wire [31:0] a;
    input wire [31:0] wd;
    output wire [31:0] rd;
    
    reg [31:0] RAM [63:0];
    
    initial $readmemh("memfile.dat", RAM);
    
    assign rd = RAM[a[31:2]]; // word aligned
    
    always @(posedge clk)
        if (we)
            RAM[a[31:2]] <= wd;
endmodule

module arm (
    clk,
    reset,
    MemWrite,
    Adr,
    WriteData,
    ReadData
);
    input wire clk;
    input wire reset;
    output wire MemWrite;
    output wire [31:0] Adr;
    output wire [31:0] WriteData;
    input wire [31:0] ReadData;
    
    wire [31:0] Instr;
    wire [3:0] ALUFlags;
    wire PCWrite;
    wire RegWrite;
    wire IRWrite;
    wire AdrSrc;
    wire [1:0] RegSrc;
    wire [1:0] ALUSrcA;
    wire [1:0] ALUSrcB;
    wire [1:0] ImmSrc;
    wire [1:0] ALUControl;
    wire [1:0] ResultSrc;
    wire MulStart;
    wire MulReady;
    wire FPUStart;
    wire FPUReady;
    wire [1:0] MulControl;
    wire [1:0] FPUControl;
    
    controller c(
        .clk(clk),
        .reset(reset),
        .Instr(Instr[31:12]),
        .ALUFlags(ALUFlags),
        .MulReady(MulReady),
        .FPUReady(FPUReady),
        .PCWrite(PCWrite),
        .MemWrite(MemWrite),
        .RegWrite(RegWrite),
        .IRWrite(IRWrite),
        .AdrSrc(AdrSrc),
        .RegSrc(RegSrc),
        .ALUSrcA(ALUSrcA),
        .ALUSrcB(ALUSrcB),
        .ResultSrc(ResultSrc),
        .ImmSrc(ImmSrc),
        .ALUControl(ALUControl),
        .MulStart(MulStart),
        .MulControl(MulControl),
        .FPUStart(FPUStart),
        .FPUControl(FPUControl)
    );
    
    datapath dp(
        .clk(clk),
        .reset(reset),
        .Adr(Adr),
        .WriteData(WriteData),
        .ReadData(ReadData),
        .Instr(Instr),
        .ALUFlags(ALUFlags),
        .PCWrite(PCWrite),
        .RegWrite(RegWrite),
        .IRWrite(IRWrite),
        .AdrSrc(AdrSrc),
        .RegSrc(RegSrc),
        .ALUSrcA(ALUSrcA),
        .ALUSrcB(ALUSrcB),
        .ResultSrc(ResultSrc),
        .ImmSrc(ImmSrc),
        .ALUControl(ALUControl),
        .MulStart(MulStart),
        .MulReady(MulReady),
        .MulControl(MulControl),
        .FPUStart(FPUStart),
        .FPUReady(FPUReady),
        .FPUControl(FPUControl)
    );
endmodule

module controller (
    clk,
    reset,
    Instr,
    ALUFlags,
    MulReady,
    FPUReady,
    PCWrite,
    MemWrite,
    RegWrite,
    IRWrite,
    AdrSrc,
    RegSrc,
    ALUSrcA,
    ALUSrcB,
    ResultSrc,
    ImmSrc,
    ALUControl,
    MulStart,
    MulControl,
    FPUStart,
    FPUControl
);
    input wire clk;
    input wire reset;
    input wire [31:12] Instr;
    input wire [3:0] ALUFlags;
    input wire MulReady;
    input wire FPUReady;
    output wire PCWrite;
    output wire MemWrite;
    output wire RegWrite;
    output wire IRWrite;
    output wire AdrSrc;
    output wire [1:0] RegSrc;
    output wire [1:0] ALUSrcA;
    output wire [1:0] ALUSrcB;
    output wire [1:0] ResultSrc;
    output wire [1:0] ImmSrc;
    output wire [1:0] ALUControl;
    output wire MulStart;
    output wire [1:0] MulControl;
    output wire FPUStart;
    output wire [1:0] FPUControl;
    
    wire [1:0] FlagW;
    wire PCS;
    wire NextPC;
    wire RegW;
    wire MemW;
    
    decode dec(
        .clk(clk),
        .reset(reset),
        .Op(Instr[27:26]),
        .Funct(Instr[25:20]),
        .Rd(Instr[15:12]),
        .MulReady(MulReady),
        .FPUReady(FPUReady),
        .FlagW(FlagW),
        .PCS(PCS),
        .NextPC(NextPC),
        .RegW(RegW),
        .MemW(MemW),
        .IRWrite(IRWrite),
        .AdrSrc(AdrSrc),
        .ResultSrc(ResultSrc),
        .ALUSrcA(ALUSrcA),
        .ALUSrcB(ALUSrcB),
        .ImmSrc(ImmSrc),
        .RegSrc(RegSrc),
        .ALUControl(ALUControl),
        .MulStart(MulStart),
        .MulControl(MulControl),
        .FPUStart(FPUStart),
        .FPUControl(FPUControl)
    );
    
    condlogic cl(
        .clk(clk),
        .reset(reset),
        .Cond(Instr[31:28]),
        .ALUFlags(ALUFlags),
        .FlagW(FlagW),
        .PCS(PCS),
        .NextPC(NextPC),
        .RegW(RegW),
        .MemW(MemW),
        .PCWrite(PCWrite),
        .RegWrite(RegWrite),
        .MemWrite(MemWrite)
    );
endmodule

module decode (
    clk,
    reset,
    Op,
    Funct,
    Rd,
    MulReady,
    FPUReady,
    FlagW,
    PCS,
    NextPC,
    RegW,
    MemW,
    IRWrite,
    AdrSrc,
    ResultSrc,
    ALUSrcA,
    ALUSrcB,
    ImmSrc,
    RegSrc,
    ALUControl,
    MulStart,
    MulControl,
    FPUStart,
    FPUControl
);
    input wire clk;
    input wire reset;
    input wire [1:0] Op;
    input wire [5:0] Funct;
    input wire [3:0] Rd;
    input wire MulReady;
    input wire FPUReady;
    output wire [1:0] FlagW;
    output wire PCS;
    output wire NextPC;
    output wire RegW;
    output wire MemW;
    output wire IRWrite;
    output wire AdrSrc;
    output wire [1:0] ResultSrc;
    output wire [1:0] ALUSrcA;
    output wire [1:0] ALUSrcB;
    output wire [1:0] ImmSrc;
    output wire [1:0] RegSrc;
    output wire [1:0] ALUControl;
    output wire MulStart;
    output wire [1:0] MulControl;
    output wire FPUStart;
    output wire [1:0] FPUControl;
    
    wire Branch;
    wire ALUOp;
    wire [1:0] ALUControlInt;
    
    // Main FSM
    mainfsm fsm(
        .clk(clk),
        .reset(reset),
        .Op(Op),
        .Funct(Funct),
        .MulReady(MulReady),
        .FPUReady(FPUReady),
        .IRWrite(IRWrite),
        .AdrSrc(AdrSrc),
        .ALUSrcA(ALUSrcA),
        .ALUSrcB(ALUSrcB),
        .ResultSrc(ResultSrc),
        .NextPC(NextPC),
        .RegW(RegW),
        .MemW(MemW),
        .Branch(Branch),
        .ALUOp(ALUOp),
        .MulStart(MulStart),
        .FPUStart(FPUStart)
    );
    
    // ALU Decoder
    aludec ad(
        .funct(Funct[4:0]),
        .aluop(ALUOp),
        .alucontrol(ALUControlInt),
        .flagw(FlagW)
    );
    
    assign ALUControl = ALUControlInt;
    
    // PC Logic
    assign PCS = ((Rd == 4'b1111) & RegW);
    
    // Instruction Decoder
    assign ImmSrc = Op;
    
    // RegSrc Logic
    assign RegSrc[0] = (Op == 2'b10);
    assign RegSrc[1] = (Op == 2'b01);
    
    // Multiplication Control
    assign MulControl = (Op == 2'b00) ? Funct[1:0] : 2'b00;
    
    // FPU Control
    assign FPUControl = (Op == 2'b11) ? Funct[1:0] : 2'b00;
endmodule

module mainfsm (
    clk,
    reset,
    Op,
    Funct,
    MulReady,
    FPUReady,
    IRWrite,
    AdrSrc,
    ALUSrcA,
    ALUSrcB,
    ResultSrc,
    NextPC,
    RegW,
    MemW,
    Branch,
    ALUOp,
    MulStart,
    FPUStart
);
    input wire clk;
    input wire reset;
    input wire [1:0] Op;
    input wire [5:0] Funct;
    input wire MulReady;
    input wire FPUReady;
    output reg IRWrite;
    output reg AdrSrc;
    output reg [1:0] ALUSrcA;
    output reg [1:0] ALUSrcB;
    output reg [1:0] ResultSrc;
    output reg NextPC;
    output reg RegW;
    output reg MemW;
    output reg Branch;
    output reg ALUOp;
    output reg MulStart;
    output reg FPUStart;
    
    reg [3:0] state;
    reg [3:0] nextstate;
    
    // State definitions
    localparam [3:0] FETCH = 4'd0;
    localparam [3:0] DECODE = 4'd1;
    localparam [3:0] MEMADR = 4'd2;
    localparam [3:0] MEMRD = 4'd3;
    localparam [3:0] MEMWB = 4'd4;
    localparam [3:0] MEMWR = 4'd5;
    localparam [3:0] EXECUTER = 4'd6;
    localparam [3:0] EXECUTEI = 4'd7;
    localparam [3:0] ALUWB = 4'd8;
    localparam [3:0] BRANCH = 4'd9;
    localparam [3:0] MULEXEC = 4'd10;
    localparam [3:0] MULWB = 4'd11;
    localparam [3:0] FPUEXEC = 4'd12;
    localparam [3:0] FPUWB = 4'd13;
    
    // state register
    always @(posedge clk or posedge reset)
        if (reset)
            state <= FETCH;
        else
            state <= nextstate;
    
    // next state logic
    always @(*)
        case (state)
            FETCH: nextstate = DECODE;
            DECODE:
                case (Op)
                    2'b00: begin
                        // Check for multiplication instructions
                        if (Funct[5:4] == 2'b00 && Funct[3:0] == 4'b1001) // MUL family
                            nextstate = MULEXEC;
                        else if (Funct[5])
                            nextstate = EXECUTEI;
                        else
                            nextstate = EXECUTER;
                    end
                    2'b01: nextstate = MEMADR;
                    2'b10: nextstate = BRANCH;
                    2'b11: nextstate = FPUEXEC; // FPU instructions
                    default: nextstate = FETCH;
                endcase
            EXECUTER: nextstate = ALUWB;
            EXECUTEI: nextstate = ALUWB;
            ALUWB: nextstate = FETCH;
            MEMADR: nextstate = Funct[0] ? MEMRD : MEMWR;
            MEMRD: nextstate = MEMWB;
            MEMWB: nextstate = FETCH;
            MEMWR: nextstate = FETCH;
            BRANCH: nextstate = FETCH;
            MULEXEC: nextstate = MulReady ? MULWB : MULEXEC;
            MULWB: nextstate = FETCH;
            FPUEXEC: nextstate = FPUReady ? FPUWB : FPUEXEC;
            FPUWB: nextstate = FETCH;
            default: nextstate = FETCH;
        endcase
    
    // state-dependent output logic
    always @(*)
        case (state)
            FETCH: begin
                IRWrite = 1'b1; AdrSrc = 1'b0; ALUSrcA = 2'b00; ALUSrcB = 2'b10;
                ResultSrc = 2'b10; NextPC = 1'b1; RegW = 1'b0; MemW = 1'b0;
                Branch = 1'b0; ALUOp = 1'b0; MulStart = 1'b0; FPUStart = 1'b0;
            end
            DECODE: begin
                IRWrite = 1'b0; AdrSrc = 1'b0; ALUSrcA = 2'b00; ALUSrcB = 2'b10;
                ResultSrc = 2'b10; NextPC = 1'b0; RegW = 1'b0; MemW = 1'b0;
                Branch = 1'b0; ALUOp = 1'b0; MulStart = 1'b0; FPUStart = 1'b0;
            end
            EXECUTER: begin
                IRWrite = 1'b0; AdrSrc = 1'b0; ALUSrcA = 2'b01; ALUSrcB = 2'b00;
                ResultSrc = 2'b00; NextPC = 1'b0; RegW = 1'b0; MemW = 1'b0;
                Branch = 1'b0; ALUOp = 1'b1; MulStart = 1'b0; FPUStart = 1'b0;
            end
            EXECUTEI: begin
                IRWrite = 1'b0; AdrSrc = 1'b0; ALUSrcA = 2'b01; ALUSrcB = 2'b01;
                ResultSrc = 2'b00; NextPC = 1'b0; RegW = 1'b0; MemW = 1'b0;
                Branch = 1'b0; ALUOp = 1'b1; MulStart = 1'b0; FPUStart = 1'b0;
            end
            ALUWB: begin
                IRWrite = 1'b0; AdrSrc = 1'b0; ALUSrcA = 2'b01; ALUSrcB = 2'b00;
                ResultSrc = 2'b00; NextPC = 1'b0; RegW = 1'b1; MemW = 1'b0;
                Branch = 1'b0; ALUOp = 1'b0; MulStart = 1'b0; FPUStart = 1'b0;
            end
            MEMADR: begin
                IRWrite = 1'b0; AdrSrc = 1'b0; ALUSrcA = 2'b01; ALUSrcB = 2'b01;
                ResultSrc = 2'b00; NextPC = 1'b0; RegW = 1'b0; MemW = 1'b0;
                Branch = 1'b0; ALUOp = 1'b0; MulStart = 1'b0; FPUStart = 1'b0;
            end
            MEMWR: begin
                IRWrite = 1'b0; AdrSrc = 1'b1; ALUSrcA = 2'b01; ALUSrcB = 2'b01;
                ResultSrc = 2'b00; NextPC = 1'b0; RegW = 1'b0; MemW = 1'b1;
                Branch = 1'b0; ALUOp = 1'b0; MulStart = 1'b0; FPUStart = 1'b0;
            end
            MEMRD: begin
                IRWrite = 1'b0; AdrSrc = 1'b1; ALUSrcA = 2'b01; ALUSrcB = 2'b01;
                ResultSrc = 2'b00; NextPC = 1'b0; RegW = 1'b0; MemW = 1'b0;
                Branch = 1'b0; ALUOp = 1'b0; MulStart = 1'b0; FPUStart = 1'b0;
            end
            MEMWB: begin
                IRWrite = 1'b0; AdrSrc = 1'b0; ALUSrcA = 2'b01; ALUSrcB = 2'b01;
                ResultSrc = 2'b01; NextPC = 1'b0; RegW = 1'b1; MemW = 1'b0;
                Branch = 1'b0; ALUOp = 1'b0; MulStart = 1'b0; FPUStart = 1'b0;
            end
            BRANCH: begin
                IRWrite = 1'b0; AdrSrc = 1'b0; ALUSrcA = 2'b00; ALUSrcB = 2'b01;
                ResultSrc = 2'b00; NextPC = 1'b0; RegW = 1'b0; MemW = 1'b0;
                Branch = 1'b1; ALUOp = 1'b0; MulStart = 1'b0; FPUStart = 1'b0;
            end
            MULEXEC: begin
                IRWrite = 1'b0; AdrSrc = 1'b0; ALUSrcA = 2'b01; ALUSrcB = 2'b00;
                ResultSrc = 2'b11; NextPC = 1'b0; RegW = 1'b0; MemW = 1'b0;
                Branch = 1'b0; ALUOp = 1'b0; MulStart = 1'b1; FPUStart = 1'b0;
            end
            MULWB: begin
                IRWrite = 1'b0; AdrSrc = 1'b0; ALUSrcA = 2'b01; ALUSrcB = 2'b00;
                ResultSrc = 2'b11; NextPC = 1'b0; RegW = 1'b1; MemW = 1'b0;
                Branch = 1'b0; ALUOp = 1'b0; MulStart = 1'b0; FPUStart = 1'b0;
            end
            FPUEXEC: begin
                IRWrite = 1'b0; AdrSrc = 1'b0; ALUSrcA = 2'b01; ALUSrcB = 2'b00;
                ResultSrc = 2'b11; NextPC = 1'b0; RegW = 1'b0; MemW = 1'b0;
                Branch = 1'b0; ALUOp = 1'b0; MulStart = 1'b0; FPUStart = 1'b1;
            end
            FPUWB: begin
                IRWrite = 1'b0; AdrSrc = 1'b0; ALUSrcA = 2'b01; ALUSrcB = 2'b00;
                ResultSrc = 2'b11; NextPC = 1'b0; RegW = 1'b1; MemW = 1'b0;
                Branch = 1'b0; ALUOp = 1'b0; MulStart = 1'b0; FPUStart = 1'b0;
            end
            default: begin
                IRWrite = 1'b0; AdrSrc = 1'b0; ALUSrcA = 2'b00; ALUSrcB = 2'b00;
                ResultSrc = 2'b00; NextPC = 1'b0; RegW = 1'b0; MemW = 1'b0;
                Branch = 1'b0; ALUOp = 1'b0; MulStart = 1'b0; FPUStart = 1'b0;
            end
        endcase
endmodule

module aludec (
    funct,
    aluop,
    alucontrol,
    flagw
);
    input wire [4:0] funct;
    input wire aluop;
    output reg [1:0] alucontrol;
    output reg [1:0] flagw;
    
    always @(*)
        if (aluop) begin
            case (funct[4:1])
                4'b0100: begin
                    alucontrol = 2'b00; // ADD
                    flagw = funct[0] ? 2'b11 : 2'b00;
                end
                4'b0010: begin
                    alucontrol = 2'b01; // SUB
                    flagw = funct[0] ? 2'b11 : 2'b00;
                end
                4'b0000: begin
                    alucontrol = 2'b10; // AND
                    flagw = funct[0] ? 2'b10 : 2'b00;
                end
                4'b1100: begin
                    alucontrol = 2'b11; // ORR
                    flagw = funct[0] ? 2'b10 : 2'b00;
                end
                default: begin
                    alucontrol = 2'b00;
                    flagw = 2'b00;
                end
            endcase
        end else begin
            alucontrol = 2'b00; // ADD for non-ALU ops
            flagw = 2'b00;
        end
endmodule

module condlogic (
    clk,
    reset,
    Cond,
    ALUFlags,
    FlagW,
    PCS,
    NextPC,
    RegW,
    MemW,
    PCWrite,
    RegWrite,
    MemWrite
);
    input wire clk;
    input wire reset;
    input wire [3:0] Cond;
    input wire [3:0] ALUFlags;
    input wire [1:0] FlagW;
    input wire PCS;
    input wire NextPC;
    input wire RegW;
    input wire MemW;
    output wire PCWrite;
    output wire RegWrite;
    output wire MemWrite;
    
    wire [1:0] FlagWrite;
    wire [3:0] Flags;
    wire CondEx;
    
    // Delay writing flags until ALUWB state
    flopr #(2) flagwritereg(
        .clk(clk),
        .reset(reset),
        .d(FlagW & {2{CondEx}}),
        .q(FlagWrite)
    );
    
    // Flag register
    flopenr #(2) flagreg(
        .clk(clk),
        .reset(reset),
        .en(FlagWrite[1]),
        .d(ALUFlags[3:2]),
        .q(Flags[3:2])
    );
    
    flopenr #(2) flagreg2(
        .clk(clk),
        .reset(reset),
        .en(FlagWrite[0]),
        .d(ALUFlags[1:0]),
        .q(Flags[1:0])
    );
    
    // Condition check
    condcheck cc(
        .Cond(Cond),
        .Flags(Flags),
        .CondEx(CondEx)
    );
    
    // Output logic
    assign PCWrite = (PCS & CondEx) | NextPC;
    assign RegWrite = RegW & CondEx;
    assign MemWrite = MemW & CondEx;
endmodule

module condcheck (
    Cond,
    Flags,
    CondEx
);
    input wire [3:0] Cond;
    input wire [3:0] Flags;
    output reg CondEx;
    
    wire neg, zero, carry, overflow, ge;
    
    assign {neg, zero, carry, overflow} = Flags;
    assign ge = (neg == overflow);
    
    always @(*)
        case (Cond)
            4'b0000: CondEx = zero;             // EQ
            4'b0001: CondEx = ~zero;            // NE
            4'b0010: CondEx = carry;            // CS
            4'b0011: CondEx = ~carry;           // CC
            4'b0100: CondEx = neg;              // MI
            4'b0101: CondEx = ~neg;             // PL
            4'b0110: CondEx = overflow;         // VS
            4'b0111: CondEx = ~overflow;        // VC
            4'b1000: CondEx = carry & ~zero;    // HI
            4'b1001: CondEx = ~(carry & ~zero); // LS
            4'b1010: CondEx = ge;               // GE
            4'b1011: CondEx = ~ge;              // LT
            4'b1100: CondEx = ~zero & ge;       // GT
            4'b1101: CondEx = ~(~zero & ge);    // LE
            4'b1110: CondEx = 1'b1;             // Always
            default: CondEx = 1'bx;
        endcase
endmodule

module datapath (
    clk,
    reset,
    Adr,
    WriteData,
    ReadData,
    Instr,
    ALUFlags,
    PCWrite,
    RegWrite,
    IRWrite,
    AdrSrc,
    RegSrc,
    ALUSrcA,
    ALUSrcB,
    ResultSrc,
    ImmSrc,
    ALUControl,
    MulStart,
    MulReady,
    MulControl,
    FPUStart,
    FPUReady,
    FPUControl
);
    input wire clk;
    input wire reset;
    output wire [31:0] Adr;
    output wire [31:0] WriteData;
    input wire [31:0] ReadData;
    output wire [31:0] Instr;
    output wire [3:0] ALUFlags;
    input wire PCWrite;
    input wire RegWrite;
    input wire IRWrite;
    input wire AdrSrc;
    input wire [1:0] RegSrc;
    input wire [1:0] ALUSrcA;
    input wire [1:0] ALUSrcB;
    input wire [1:0] ResultSrc;
    input wire [1:0] ImmSrc;
    input wire [1:0] ALUControl;
    input wire MulStart;
    output wire MulReady;
    input wire [1:0] MulControl;
    input wire FPUStart;
    output wire FPUReady;
    input wire [1:0] FPUControl;
    
    wire [31:0] PCNext;
    wire [31:0] PC;
    wire [31:0] ExtImm;
    wire [31:0] SrcA;
    wire [31:0] SrcB;
    wire [31:0] Result;
    wire [31:0] Data;
    wire [31:0] RD1;
    wire [31:0] RD2;
    wire [31:0] A;
    wire [31:0] ALUResult
