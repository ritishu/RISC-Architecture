module fsm (start, clock, op, PcWr, ALUSrcA, ALUSrcB, ALUOp, Output, RegA, RegB, Read3, RegDst, MemtoReg, RegWr, MemR, MemW, EQbar,PcSrc);

    input [3:0] op;
    input clock, start;                                                             //start is needed to intially reset fsm to to state 0
    output reg PcWr, ALUSrcA, Output, RegB, Read3, RegDst, MemtoReg, RegWr, MemR, MemW, EQbar, PcSrc;
    output reg [1:0] ALUOp, RegA;
    output reg [2:0] ALUSrcB;
    wire [15:0] control;                                                            //decoded 4 bit opcode
    wire [21:0] current_state;
    reg [21:0] next_state;

    decoder_4x16 ID (.In(op), .Out(control));
    dff_22 state (.clk(clock), .d(next_state), .q(current_state), .rst(start));

    always @ (*) begin

        //NEXT STATE LOGIC
        next_state[0] = current_state[6] | current_state[14] | current_state[15] | current_state[19] | current_state[20];

        next_state[1] = current_state[0] & (control[4] | control[5] | control[8] | control[11] | control[12] | control[15]);

        next_state[2] = current_state[1] & control[8];
        
        next_state[3] = current_state[1] & control[12];
        
        next_state[4] = current_state[1] & control[11];
        
        next_state[5] = current_state[1] & control[15];
        
        next_state[6] = current_state[2] | current_state[3] | current_state[4] | current_state[5] | current_state[8] | current_state[9] | current_state[10] | current_state[11] | current_state[12] | current_state[13] | current_state[21];

        next_state[7] = current_state[0] & (control[0] | control[6] | control[7] | control[9] | control[10] | control[13] | control[14]);
        
        next_state[8] = current_state[7] & control[9];
        
        next_state[9] = current_state[7] & control[10];
        
        next_state[10] = current_state[7] & control[13];
        
        next_state[11] = current_state[7] & control[14];
        
        next_state[12] = current_state[7] & control[7];
        
        next_state[13] = current_state[7] & control[6];

        next_state[14] = current_state[1] & (control[4] | control[5]);
        
        next_state[15] = current_state[0] & control[3];
        
        next_state[16] = current_state[0] & (control[1] | control[2]);
        
        next_state[17] = current_state[16];
        
        next_state[18] = current_state[17] & control[1];
        
        next_state[19] = current_state[18];
        
        next_state[20] = current_state[17] & control[2];
        
        next_state[21] = current_state[7] & control[0];
        
        //CONTROL SIGNAL LOGIC

        PcWr = current_state[0] | current_state[15];

        ALUSrcA = current_state[2] | current_state[3] | current_state[4] | current_state[5] | current_state[8] | current_state[9] | current_state[10] | current_state[11] | current_state[12] | current_state[13] | current_state[14] | current_state[17];

        ALUSrcB[2] =  current_state[15] | current_state[17];

        ALUSrcB[1] = current_state[8] | current_state[9] | current_state[10] | current_state[11] | current_state[12] | current_state[13];
        
        ALUSrcB[0] = current_state[2] | current_state[3] | current_state[4] | current_state[5] | current_state[9] | current_state[11] | current_state[14] | current_state[15];
         
        ALUOp[1] = current_state[4] | current_state[5] | current_state[12] | current_state[13];

        ALUOp[0] = current_state[3] | current_state[5] | current_state[10] | current_state[11] | current_state[13] | current_state[14];

        Output = current_state[21];

        RegA[1] = current_state[16];

        RegA[0] = current_state[7];

        RegB = RegA[1];

        Read3 = current_state[1];

        RegDst = current_state[19];

        MemtoReg = RegDst;                                                                    

        RegWr = current_state[6] | current_state[19];        

        MemR = current_state[18];

        MemW = current_state[20];

        EQbar = op[0];

        PcSrc = current_state[14];

    end

endmodule

module dff_22 (clk, d, q, rst);

    input clk, rst;
    input [21:0] d;
    output reg [21:0] q;

    always @ (posedge clk or negedge rst)

        if (!rst)
            q <= 22'b00000_00000_00000_00000_01;
        else
            q <= d;

endmodule

module decoder_4x16(In, Out);

    input [3:0] In;
    output reg [15:0] Out;

    always @ (In) begin 
        case(In)
        4'b0000: Out = 16'b0000_0000_0000_0001;
        4'b0001: Out = 16'b0000_0000_0000_0010;
        4'b0010: Out = 16'b0000_0000_0000_0100;
        4'b0011: Out = 16'b0000_0000_0000_1000;
        4'b0100: Out = 16'b0000_0000_0001_0000;
        4'b0101: Out = 16'b0000_0000_0010_0000;
        4'b0110: Out = 16'b0000_0000_0100_0000;
        4'b0111: Out = 16'b0000_0000_1000_0000;
        4'b1000: Out = 16'b0000_0001_0000_0000;
        4'b1001: Out = 16'b0000_0010_0000_0000;
        4'b1010: Out = 16'b0000_0100_0000_0000;
        4'b1011: Out = 16'b0000_1000_0000_0000;
        4'b1100: Out = 16'b0001_0000_0000_0000;
        4'b1101: Out = 16'b0010_0000_0000_0000;
        4'b1110: Out = 16'b0100_0000_0000_0000;
        4'b1111: Out = 16'b1000_0000_0000_0000;
        endcase
    end

endmodule