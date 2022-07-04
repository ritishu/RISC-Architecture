module instruct_mem(ir, pc_in, pc_wr, pc, clk, rst_pc, rst_ir, op);//Instruction memory 
    output reg[15:0]ir;                                             //With Instruction register 
    output reg[15:0]pc;    
    output [7:0]op; 
    // reg [15:0] dummy;                                         //And program counter

    input clk;
    input pc_wr;
    input rst_pc;
    input rst_ir;

    input [15:0]pc_in;

    reg [7:0]mem_even[0:32767]; //Stored in little endian format
    reg [7:0]mem_odd[0:32767];  //With even and odd banks

    wire [14:0]address;

    assign address = pc[15:1];
    assign op = mem_odd[address];

    always @(posedge clk or posedge rst_pc) begin
        if(rst_pc) begin
            pc <= 16'h0000;
        end
        else if(pc_wr) begin 
            pc <= pc_in;
        end
    end
    always @(posedge clk or posedge rst_ir) begin
        if(rst_ir) begin
            ir <= 16'h0000;
        end
        else if(pc_wr)                                      
            ir <= {mem_odd[address], mem_even[address]};
    end
endmodule
/*
module PC(pc, clk, pc_wr1, pc_in);   //Program counter register
    output reg[15:0]pc;

    input clk;
    input pc_wr1;
    input [15:0]pc_in;

    always @(posedge clk ) begin
        if(pc_wr1)
            pc <= pc_in;
    end
endmodule

module IR(ir, clk, ir_in);          //Instruction register
    output [15:0]ir;

    input clk;
    input [15:0]ir_in;

    reg [15:0]IR;

    assign ir = IR;

    always @(posedge clk ) begin
        IR <= ir_in;
    end
endmodule
*/
module mux_reg_A(rn1, regA, ir7_4, ir11_8); //Mux before RN1
    output [3:0]rn1;    //First read register address in reg_file 

    input [3:0]ir7_4; // IR[7:4]
    input [3:0]ir11_8; // IR[11:8]

    input [1:0]regA;  //RegA

    assign rn1 = regA[1] ? (!regA[0] ? {2'b10, ir11_8[1:0]} : 4'hz) : (regA[0] ? ir11_8 : ir7_4);
endmodule

module mux_reg_B(rn2, in0, in1, regB);    //Mux before RN2
    output [3:0]rn2;    //Second read register address in reg_file

    input [3:0]in0; // IR[3:0]
    input [3:0]in1; // {2'b11, IR[11:10]} 

    input regB;   //RegB

    assign rn2 = regB ? in1 : in0;
endmodule

module mux_reg_dst(wr, ir11_8, reg_dst);
    output [3:0]wr; //Write register address in reg_file

    input [3:0]ir11_8; //ir[11:8]

    input reg_dst;

    assign wr = reg_dst ? {2'b11, ir11_8[3:2]} : ir11_8;
endmodule

module reg_file(A, B, C, rn1, rn2, rn3, read3, wr, wd,  reg_wr, clk, rst_ABC);
    output reg [15:0]A;
    output reg [15:0]B;
    output reg [15:0]C;

    input [15:0]wd;

    input [3:0]rn1;
    input [3:0]rn2;
    input [3:0]rn3;

    input [3:0]wr;

    input [2:0]rst_ABC;

    input clk;
    input reg_wr;
    input read3;

    reg [15:0]register_file[0:15];

    initial register_file[0] = 16'h0000; //$0 is hard coded to 0

    always @(posedge clk or posedge rst_ABC[0]) begin
        if(rst_ABC[0])
            A <= 16'h0000;
        else
            A <= register_file[rn1];
        if((reg_wr) & (|wr))              //$0 is not valid write location
            register_file[wr] <= wd;
    end

    always @(posedge clk or posedge rst_ABC[1]) begin
        if(rst_ABC[1])
            B <= 16'h0000;
        else
            B <= register_file[rn2];
    end

    always @(posedge clk or posedge rst_ABC[2]) begin
        if(rst_ABC[2])
            C <= 16'h0000;
        else if(read3)                       //only true for Branch statement
            C <= register_file[rn3];
    end
endmodule

module sign_ext_8to16(out, in);
    output [15:0]out;
    input [7:0]in;

    assign out = {{8{in[7]}}, in[7:0]};
endmodule

module mux_alu_A(alu_A, pc, A, alu_srcA);
    output [15:0]alu_A;

    input [15:0]pc;
    input [15:0]A;

    input alu_srcA;

    assign alu_A = alu_srcA ? A : pc;
endmodule

module mux_alu_B(alu_B, ir11_0, B, alu_srcB); //send in ir[11:0]
    output [15:0]alu_B;
    
    input [11:0]ir11_0;                     //ir[11:0]
    input [15:0]B;
    input [2:0]alu_srcB;

    wire [15:0]ir_sign_ext;

    reg [15:0]alu_B1;

    sign_ext_8to16 U0(.out(ir_sign_ext), .in(ir11_0[7:0]));

    always @(*) begin
        case(alu_srcB)
        3'b000 : alu_B1 <= 16'h0002;
        3'b001 : alu_B1 <= B;
        3'b010 : alu_B1 <= ir_sign_ext;
        3'b011 : alu_B1 <= {8'h00, ir11_0[7:0]};
        3'b100 : alu_B1 <= (ir_sign_ext << 1);
        3'b101 : alu_B1 <= {4'h0, ir11_0};
        default: alu_B1 <= 16'hzzzz;
        endcase
    end

    assign alu_B = alu_B1;
endmodule

module shift_control(sh_op, opcode, ffield);    
    output [1:0]sh_op;

    input [3:0]opcode;  //opcode = ir[15:12]
    input [3:0]ffield;  //ffield = ir[3:0] , ffield is function field

    assign sh_op = (~(|opcode)) ? ffield[1:0] : 2'b00;
endmodule

module mux_alu_out(alu_out, alu, shifter, output_cont, clk, rst_alu_out); 
    output reg [15:0]alu_out;

    input [15:0]alu;
    input [15:0]shifter;

    input rst_alu_out;

    input clk;
    input output_cont; //output control signal

    wire [15:0]out;

    assign out = output_cont ? shifter : alu;

    always @(posedge clk or posedge rst_alu_out) begin
        if(rst_alu_out)
            alu_out <= 16'h0000;
        else
            alu_out <= out;
    end
endmodule

module mux_pc_src(pc_in, pc_src, alu, C);
    output [15:0]pc_in;

    input [15:0]alu;
    input [15:0]C;

    input pc_src;

    assign pc_in = pc_src ? C : alu;
endmodule

module data_mem(mdr, addr, data_in, memr, memw, clk, rst_mdr);
    output reg [15:0]mdr;

    input [15:0]addr;   //input address is always even!!
                        //Comes from alu_out register

    input memr;         //Memory read condition
    input memw;         //Memory write condition
    input clk;
    input rst_mdr;

    input [15:0]data_in;

    reg [7:0]mem_even[0:32767];
    reg [7:0]mem_odd[0:32767];

    wire [14:0]address;

    assign address = addr[15:1];

    always @(posedge clk or posedge rst_mdr) begin
        if(rst_mdr) begin
            mdr <= 16'h0000;
        end
        else if(memr&(!memw))begin
            $display("%h", address);
            mdr <= {mem_odd[address], mem_even[address]};
        end
        else if(memw&(!memr)) begin
            mem_even[address] <= data_in[7:0];
            mem_odd[address] <= data_in[15:8];
        end
    end
endmodule

module mux_mem_to_reg(wd, alu_out, mdr, mem_to_reg);
    output [15:0]wd;

    input [15:0]mdr;
    input [15:0]alu_out;
    input mem_to_reg;

    assign wd = mem_to_reg ? mdr : alu_out;
endmodule

module pc_wr_control(pc_wr1, pc_wr, pc_src, eqb, zf);
    output pc_wr1;

    input pc_wr;
    input pc_src;
    input eqb;
    input zf;

    assign pc_wr1 = pc_wr | (pc_src & (eqb^zf));
endmodule


module ALU(alu, zf,alu_A, alu_B, alu_op);
    output [15:0]alu;
    output zf;

    input [15:0]alu_A, alu_B;
    input [1:0]alu_op;

    wire [15:0]alu_addsub;
    wire [15:0]alu_nand;
    wire [15:0]alu_or;

    wire sign_B;

    assign sign_B = ((~alu_op[1])&alu_op[0]);

    assign alu_addsub = sign_B ? (alu_A - alu_B) : alu_A + alu_B;

    assign alu_nand = ~(alu_A & alu_B);

    assign alu_or = (alu_A | alu_B);

    assign alu = alu_op[1] ? (alu_op[0] ? alu_or : alu_nand) : alu_addsub;

    assign zf = ~(|alu);
endmodule

module bar_shift_lr_16b (a, sel, lr, b);

    input [15:0] a;                             //Input
    input [3:0] sel;                            //shift amount
    input [1:0]lr;                              //sh op from flowchart
    output [15:0] b;                            //Output

    wire [15:0] w1, w2;

    muxflip_2x1_16b switch (a, lr, w1);
    bar_shift_16b shift (w1, sel, w2, lr);
    muxflip_2x1_16b restore (w2, lr, b);

endmodule

module bar_shift_16b (a, sel, b, a_l);

    input [15:0] a;
    input [3:0] sel;
    input [1:0] a_l;
    output [15:0] b;    

    wire [15:0] x, y, z;
    wire bit_a = &a_l ? a[15] : 1'b0;
    wire bit_x = &a_l ? x[15] : 1'b0;
    wire bit_y = &a_l ? y[15] : 1'b0;
    wire bit_z = &a_l ? z[15] : 1'b0;
    
    mux_2x1_1b o3_0 (a[0], a[8], sel[3], x[0]); 
    mux_2x1_1b o3_1 (a[1], a[9], sel[3], x[1]); 
    mux_2x1_1b o3_2 (a[2], a[10], sel[3], x[2]); 
    mux_2x1_1b o3_3 (a[3], a[11], sel[3], x[3]); 
    mux_2x1_1b o3_4 (a[4], a[12], sel[3], x[4]); 
    mux_2x1_1b o3_5 (a[5], a[13], sel[3], x[5]); 
    mux_2x1_1b o3_6 (a[6], a[14], sel[3], x[6]); 
    mux_2x1_1b o3_7 (a[7], a[15], sel[3], x[7]); 
    mux_2x1_1b o3_8 (a[8], bit_a, sel[3], x[8]); 
    mux_2x1_1b o3_9 (a[9], bit_a, sel[3], x[9]); 
    mux_2x1_1b o3_10 (a[10], bit_a, sel[3], x[10]); 
    mux_2x1_1b o3_11 (a[11], bit_a, sel[3], x[11]); 
    mux_2x1_1b o3_12 (a[12], bit_a, sel[3], x[12]); 
    mux_2x1_1b o3_13 (a[13], bit_a, sel[3], x[13]); 
    mux_2x1_1b o3_14 (a[14], bit_a, sel[3], x[14]); 
    mux_2x1_1b o3_15 (a[15], bit_a, sel[3], x[15]); 

    mux_2x1_1b o2_0 (x[0], x[4], sel[2], y[0]); 
    mux_2x1_1b o2_1 (x[1], x[5], sel[2], y[1]); 
    mux_2x1_1b o2_2 (x[2], x[6], sel[2], y[2]); 
    mux_2x1_1b o2_3 (x[3], x[7], sel[2], y[3]); 
    mux_2x1_1b o2_4 (x[4], x[8], sel[2], y[4]); 
    mux_2x1_1b o2_5 (x[5], x[9], sel[2], y[5]); 
    mux_2x1_1b o2_6 (x[6], x[10], sel[2], y[6]); 
    mux_2x1_1b o2_7 (x[7], x[11], sel[2], y[7]); 
    mux_2x1_1b o2_8 (x[8], x[12], sel[2], y[8]); 
    mux_2x1_1b o2_9 (x[9], x[13], sel[2], y[9]); 
    mux_2x1_1b o2_10 (x[10], x[14], sel[2], y[10]); 
    mux_2x1_1b o2_11 (x[11], x[15], sel[2], y[11]); 
    mux_2x1_1b o2_12 (x[12], bit_x, sel[2], y[12]); 
    mux_2x1_1b o2_13 (x[13], bit_x, sel[2], y[13]); 
    mux_2x1_1b o2_14 (x[14], bit_x, sel[2], y[14]); 
    mux_2x1_1b o2_15 (x[15], bit_x, sel[2], y[15]); 

    mux_2x1_1b o1_0 (y[0], y[2], sel[1], z[0]); 
    mux_2x1_1b o1_1 (y[1], y[3], sel[1], z[1]); 
    mux_2x1_1b o1_2 (y[2], y[4], sel[1], z[2]); 
    mux_2x1_1b o1_3 (y[3], y[5], sel[1], z[3]); 
    mux_2x1_1b o1_4 (y[4], y[6], sel[1], z[4]); 
    mux_2x1_1b o1_5 (y[5], y[7], sel[1], z[5]); 
    mux_2x1_1b o1_6 (y[6], y[8], sel[1], z[6]); 
    mux_2x1_1b o1_7 (y[7], y[9], sel[1], z[7]); 
    mux_2x1_1b o1_8 (y[8], y[10], sel[1], z[8]); 
    mux_2x1_1b o1_9 (y[9], y[11], sel[1], z[9]); 
    mux_2x1_1b o1_10 (y[10], y[12], sel[1], z[10]); 
    mux_2x1_1b o1_11 (y[11], y[13], sel[1], z[11]); 
    mux_2x1_1b o1_12 (y[12], y[14], sel[1], z[12]); 
    mux_2x1_1b o1_13 (y[13], y[15], sel[1], z[13]); 
    mux_2x1_1b o1_14 (y[14], bit_y, sel[1], z[14]); 
    mux_2x1_1b o1_15 (y[15], bit_y, sel[1], z[15]); 

    mux_2x1_1b o0_0 (z[0], z[1], sel[0], b[0]); 
    mux_2x1_1b o0_1 (z[1], z[2], sel[0], b[1]); 
    mux_2x1_1b o0_2 (z[2], z[3], sel[0], b[2]); 
    mux_2x1_1b o0_3 (z[3], z[4], sel[0], b[3]); 
    mux_2x1_1b o0_4 (z[4], z[5], sel[0], b[4]); 
    mux_2x1_1b o0_5 (z[5], z[6], sel[0], b[5]); 
    mux_2x1_1b o0_6 (z[6], z[7], sel[0], b[6]); 
    mux_2x1_1b o0_7 (z[7], z[8], sel[0], b[7]); 
    mux_2x1_1b o0_8 (z[8], z[9], sel[0], b[8]); 
    mux_2x1_1b o0_9 (z[9], z[10], sel[0], b[9]); 
    mux_2x1_1b o0_10 (z[10], z[11], sel[0], b[10]); 
    mux_2x1_1b o0_11 (z[11], z[12], sel[0], b[11]); 
    mux_2x1_1b o0_12 (z[12], z[13], sel[0], b[12]); 
    mux_2x1_1b o0_13 (z[13], z[14], sel[0], b[13]); 
    mux_2x1_1b o0_14 (z[14], z[15], sel[0], b[14]); 
    mux_2x1_1b o0_15 (z[15], bit_z, sel[0], b[15]); 

endmodule


module mux_2x1_1b (a, b, sel, out);

    input a, b;
    input sel;
    output out;
    
    assign out = sel ? b : a;

endmodule

module muxflip_2x1_16b (in, lr, out);

    input [15:0] in;
    input [1:0]lr;                           //right shift for barrel shifter when lr is 0
    output [15:0] out;

    mux_2x1_1b m0 (in[0], in[15], ~lr[1], out[0]);
    mux_2x1_1b m1 (in[1], in[14], ~lr[1], out[1]);
    mux_2x1_1b m2 (in[2], in[13], ~lr[1], out[2]);
    mux_2x1_1b m3 (in[3], in[12], ~lr[1], out[3]);
    mux_2x1_1b m4 (in[4], in[11], ~lr[1], out[4]);
    mux_2x1_1b m5 (in[5], in[10], ~lr[1], out[5]);
    mux_2x1_1b m6 (in[6], in[9], ~lr[1], out[6]);
    mux_2x1_1b m7 (in[7], in[8], ~lr[1], out[7]);
    mux_2x1_1b m8 (in[8], in[7], ~lr[1], out[8]);
    mux_2x1_1b m9 (in[9], in[6], ~lr[1], out[9]);
    mux_2x1_1b m10 (in[10], in[5], ~lr[1], out[10]);
    mux_2x1_1b m11 (in[11], in[4], ~lr[1], out[11]);
    mux_2x1_1b m12 (in[12], in[3], ~lr[1], out[12]);
    mux_2x1_1b m13 (in[13], in[2], ~lr[1], out[13]);
    mux_2x1_1b m14 (in[14], in[1], ~lr[1], out[14]);
    mux_2x1_1b m15 (in[15], in[0], ~lr[1], out[15]);

endmodule

module fsm (start, clock, op1, op2, PcWr, ALUSrcA, ALUSrcB, ALUOp, Output, RegA, RegB, Read3, RegDst, MemtoReg, RegWr, MemR, MemW, EQbar,PcSrc);

    input [3:0] op1, op2;
    input clock, start;                                                             //start is needed to intially reset fsm to to state 0
    output reg PcWr, ALUSrcA, Output, RegB, Read3, RegDst, MemtoReg, RegWr, MemR, MemW, EQbar, PcSrc;
    output reg [1:0] ALUOp, RegA;
    output reg [2:0] ALUSrcB;
    wire [15:0] control;                                                            //decoded 4 bit opcode
    wire [21:0] current_state, current_state_lag;
    wire [21:0] next_state;
    wire [3:0] op;

    assign op = current_state[0] ? op2 : op1; 

    decoder_4x16 ID (.In(op), .Out(control));
    dff_22_pos state (.clk(clock), .d(next_state), .q(current_state), .rst(start));
    //dff_22_neg nstate (.clk(clock), .d(current_state), .q(current_state_lag), .rst(start));
    
    assign next_state[0] = current_state[6] | current_state[14] | current_state[15] | current_state[19] | current_state[20];

    assign next_state[1] = current_state[0] & (control[4] | control[5] | control[8] | control[11] | control[12] | control[15]);

    assign next_state[2] = current_state[1] & control[8];
    
    assign next_state[3] = current_state[1] & control[12];
    
    assign next_state[4] = current_state[1] & control[11];
    
    assign next_state[5] = current_state[1] & control[15];
    
    assign next_state[6] = current_state[2] | current_state[3] | current_state[4] | current_state[5] | current_state[8] | current_state[9] | current_state[10] | current_state[11] | current_state[12] | current_state[13] | current_state[21];

    assign next_state[7] = current_state[0] & (control[0] | control[6] | control[7] | control[9] | control[10] | control[13] | control[14]);
    
    assign next_state[8] = current_state[7] & control[9];
    
    assign next_state[9] = current_state[7] & control[10];
    
    assign next_state[10] = current_state[7] & control[13];
    
    assign next_state[11] = current_state[7] & control[14];
    
    assign next_state[12] = current_state[7] & control[7];
    
    assign next_state[13] = current_state[7] & control[6];

    assign next_state[14] = current_state[1] & (control[4] | control[5]);
    
    assign next_state[15] = current_state[0] & control[3];
    
    assign next_state[16] = current_state[0] & (control[1] | control[2]);
    
    assign next_state[17] = current_state[16];
    
    assign next_state[18] = current_state[17] & control[1];
    
    assign next_state[19] = current_state[18];
    
    assign next_state[20] = current_state[17] & control[2];
    
    assign next_state[21] = current_state[7] & control[0];

    always @ (*) begin

        //NEXT STATE LOGIC
        
        
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

module dff_22_pos (clk, d, q, rst);

    input clk, rst;
    input [21:0] d;
    output reg [21:0] q;

    always @ (posedge clk or negedge rst) begin

        if (!rst)
            q <= 22'b00000_00000_00000_00000_01;
        else
            q <= d;
    end

endmodule

module dff_22_neg (clk, d, q, rst);

    input clk, rst;
    input [21:0] d;
    output reg [21:0] q;

    always @ (negedge clk or negedge rst) begin

        if (!rst)
            q <= 22'b00000_00000_00000_00000_01;
        else
            q <= d;

    end

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

module data_path(   input   clk, 
                            strt,
                            rst_ir,
                            rst_pc,
                            rst_alu_out,
                            rst_mdr,
                            
                    input [2:0]  rst_ABC,

                    output [15:0]   pc, 
                                    A,
                                    B,
                                    C,
                                    mdr, 
                                    alu_out,
                                    ir);
    
    wire pc_wr, regB, reg_dst, read3, reg_wr, alu_srcA, output_cont, pc_src, mem_to_reg, eqb, memr, memw;
    wire [1:0] regA, alu_op;
    wire [2:0] alu_srcB;

    wire [15:0]pc_in;
    wire pc_wr1;
    wire zf;            //Connect to ALU module zf output

    wire [3:0]rn1, rn2, rn3, wr;

    wire [1:0]sh_op;

    wire [7:0]op;

    wire [15:0]wd;
    wire [15:0]ir_sign_ext;
    wire [15:0]alu_A, alu_B;    //Input A and B to ALU
    wire [15:0]alu;             //Output of ALU
    wire [15:0]shifter;         //Output of shifter

    assign rn3 = ir[11:8];

    pc_wr_control U0(.pc_wr1(pc_wr1), .pc_wr(pc_wr), .pc_src(pc_src), .eqb(eqb), .zf(zf));

    instruct_mem U1(.ir(ir), 
                    .pc(pc), 
                    .clk(clk),  
                    .pc_wr(pc_wr1), 
                    .pc_in(pc_in),
                    .rst_ir(rst_ir),
                    .rst_pc(rst_pc),
                    .op(op));
    //ir, pc_in, pc_wr1, pc, clk

    mux_reg_A U2(.rn1(rn1), .regA(regA), .ir7_4(ir[7:4]), .ir11_8(ir[11:8]));

    mux_reg_B U3(.rn2(rn2), .regB(regB), .in0(ir[3:0]), .in1({2'b11, ir[11:10]}));

    mux_reg_dst U4(.wr(wr), .ir11_8(ir[11:8]), .reg_dst(reg_dst));

    // mux_mem_to_reg U5(.wd(wd), .alu_out(alu_out), .mdr(mdr));

    reg_file U6(    .A(A), 
                    .B(B), 
                    .C(C), 
                    .rn1(rn1), 
                    .rn2(rn2), 
                    .rn3(rn3), 
                    .read3(read3), 
                    .wr(wr), 
                    .wd(wd),  
                    .reg_wr(reg_wr), 
                    .clk(clk),
                    .rst_ABC(rst_ABC));

    mux_alu_A U8(.alu_A(alu_A), .pc(pc), .A(A), .alu_srcA(alu_srcA));

    mux_alu_B U9(.alu_B(alu_B), .ir11_0(ir[11:0]), .B(B), .alu_srcB(alu_srcB));

    ALU U10(.alu(alu), .zf(zf), .alu_A(alu_A), .alu_B(alu_B), .alu_op(alu_op));

    shift_control U11(.sh_op(sh_op), .opcode(ir[15:12]), .ffield(ir[3:0]));

    bar_shift_lr_16b U12(.a(A), .sel(ir[7:4]), .b(shifter), .lr(sh_op));

    mux_alu_out U13(.alu_out(alu_out), 
                    .alu(alu), 
                    .shifter(shifter), 
                    .output_cont(output_cont), 
                    .clk(clk),
                    .rst_alu_out(rst_alu_out));

    mux_pc_src U14(.pc_in(pc_in), .pc_src(pc_src), .alu(alu), .C(C));

    data_mem U15(   .mdr(mdr), 
                    .addr(alu_out), 
                    .data_in(B),
                    .memr(memr), 
                    .memw(memw), 
                    .clk(clk), 
                    .rst_mdr(rst_mdr));

    mux_mem_to_reg U16(.wd(wd), .alu_out(alu_out), .mdr(mdr), .mem_to_reg(mem_to_reg));

    fsm controller(.start(strt), .op1(ir[15:12]), .op2(op[7:4]) , .clock(clk), .PcWr(pc_wr), .ALUSrcA(alu_srcA), .ALUSrcB(alu_srcB), .ALUOp(alu_op), .Output(output_cont), .RegA(regA), .RegB(regB), .Read3(read3), .RegDst(reg_dst), .MemtoReg(mem_to_reg), .RegWr(reg_wr), .MemR(memr), .MemW(memw), .EQbar(eqb), .PcSrc(pc_src));
 
endmodule


// fsm controller(.start(strt), .clock(clk), .PcWr(pc_wr), .ALUSrcA(alu_srcA), .ALUSrcB(alu_srcB), .ALUOp(alu_op), .Output(output_cont), .RegA(regA), .RegB(regB), .Read3(read3), .RegDst(reg_dst), .MemtoReg(mem_to_reg), .RegWr(reg_wr), .MemR(memr), .MemW(memw), .EQbar(eqb), .PcSrc(pc_src));
    