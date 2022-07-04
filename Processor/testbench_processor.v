module testbench_processor();
    
    reg test_clk,test_pc_wr,test_regB,test_reg_dst,test_read3,test_reg_wr,test_alu_srcA,test_output_cont,test_pc_src,test_mem_to_reg,test_eqb,test_rst_ir,test_rst_pc,test_rst_alu_out,test_rst_mdr,test_memr,test_memw,test_strt;
    reg [1:0]test_regA,test_alu_op;
    reg [2:0]test_alu_srcB,test_rst_ABC;
    wire [15:0]test_pc,test_A,test_B,test_C,test_mdr,test_alu_out,test_ir;
    reg [3:0]test_rn1,test_rn2,test_rn3;
    reg [15:0]test_wd;
    reg [3:0]test_wr;
    reg [15:0]test_pc_in;
    data_path uut(.clk(test_clk),.strt(test_strt),.rst_ir(test_rst_ir),.rst_pc(test_rst_pc),.rst_alu_out(test_rst_alu_out),.rst_mdr(test_rst_mdr),.rst_ABC(test_rst_ABC),.pc(test_pc),.A(test_A),.B(test_B),.C(test_C),.mdr(test_mdr),.alu_out(test_alu_out),.ir(test_ir));

   
    initial begin
        $readmemh("instruction.txt",uut.U1.mem,0,65535);
        $readmemh("memory.txt",uut.U15.mem,0,65535);
        // uut.U1.mem_odd[0] = 8'h83;
        // uut.U1.mem_even[0] = 8'h12;
        // uut.U1.mem_odd[1] = 8'hC1;
        // uut.U1.mem_even[1] = 8'h34;
    end

    initial begin
        test_clk = 1'b0;
        forever #1 test_clk = ~test_clk;
    end

    initial begin
        uut.U6.register_file[1] = 16'h0001;
        uut.U6.register_file[2] = 16'h0002;
        uut.U6.register_file[3] = 16'h0005;
        uut.U6.register_file[4] = 16'h0003;
        uut.U6.register_file[5] = 16'h0005;
        uut.U6.register_file[6] = 16'h0006;
        uut.U6.register_file[7] = 16'h0007;
        uut.U6.register_file[8] = 16'h0008;
        uut.U6.register_file[9] = 16'h0009;
        uut.U6.register_file[10] = 16'h000A;
        uut.U6.register_file[11] = 16'h000B;
        uut.U6.register_file[12] = 16'h000C;
        uut.U6.register_file[13] = 16'h000D;
        uut.U6.register_file[14] = 16'h000E;
        uut.U6.register_file[15] = 16'h000F;
        // uut.U15.mem_odd[32681] = 16'hff;
        // uut.U15.mem_even[32681] = 16'hff;
    end

    

    initial begin

        begin
            test_rst_ir = 1'b1;
            test_rst_pc = 1'b1;
            test_rst_alu_out = 1'b1;
            test_rst_mdr = 1'b1;
            test_rst_ABC =3'b111;
            test_eqb = 1'b0;
            test_strt = 1'b0;
        end

        #1 begin  //2
            test_rst_ir = 1'b0;
            test_rst_pc = 1'b0;
            test_rst_alu_out = 1'b0;
            test_rst_mdr = 1'b0;
            test_rst_ABC =3'b000;
            test_strt = 1'b1;
        end
    end


    initial begin
        $monitor("Time = %3d, Clock =%b, Current_State = %h, Next State = %h,Ir =%h,PC =%h,A =%h,B=%h,C=%h,ALUout = %h, op =%b , %h",$time,test_clk,uut.controller.current_state,uut.controller.next_state,test_ir,test_pc,test_A,test_B,test_C,test_alu_out, uut.controller.op2, uut.U15.address);
    end

    initial begin
        #40 $finish;
    end

endmodule