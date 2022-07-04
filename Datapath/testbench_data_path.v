module testbench_data_path();
    
    reg test_clk,test_pc_wr,test_regB,test_reg_dst,test_read3,test_reg_wr,test_alu_srcA,test_output_cont,test_pc_src,test_mem_to_reg,test_eqb,test_rst_ir,test_rst_pc,test_rst_alu_out,test_rst_mdr,test_memr,test_memw;
    reg [1:0]test_regA,test_alu_op;
    reg [2:0]test_alu_srcB,test_rst_ABC;
    wire [15:0]test_pc,test_A,test_B,test_C,test_mdr,test_alu_out,test_ir;
    reg [3:0]test_rn1,test_rn2,test_rn3;
    reg [15:0]test_wd;
    reg [3:0]test_wr;
    reg [15:0]test_pc_in;
    data_path uut(.clk(test_clk),.pc_wr(test_pc_wr),.regB(test_regB),.reg_dst(test_reg_dst),.read3  (test_read3),.reg_wr(test_reg_wr),.alu_srcA(test_alu_srcA),.output_cont(test_output_cont),.pc_src(test_pc_src),.mem_to_reg(test_mem_to_reg),.eqb(test_eqb),.regA(test_regA),.alu_op(test_alu_op),.alu_srcB(test_alu_srcB),.pc(test_pc),.A(test_A),.B(test_B),.C(test_C),.mdr(test_mdr),.alu_out(test_alu_out),.ir(test_ir),.rst_ir(test_rst_ir),.rst_pc(test_rst_pc),.rst_alu_out(test_rst_alu_out),
    .rst_mdr(test_rst_mdr),.rst_ABC(test_rst_ABC),.memr(test_memr),.memw(test_memw));

    initial begin
        uut.U1.mem_odd[0] = 8'h83;
        uut.U1.mem_even[0] = 8'h12;
        uut.U1.mem_odd[1] = 8'hC1;
        uut.U1.mem_even[1] = 8'h34;
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
        uut.U15.mem_odd[32681] = 16'hff;
        uut.U15.mem_even[32681] = 16'hff;
    end

    initial begin

        begin
            test_rst_ir = 1'b1;
            test_rst_pc = 1'b1;
            test_rst_alu_out = 1'b1;
            test_rst_mdr = 1'b1;
            test_rst_ABC =3'b111;
            test_eqb = 1'b0;
        end

        #2 begin  //2
            test_rst_ir = 1'b0;
            test_rst_pc = 1'b0;
            test_rst_alu_out = 1'b0;
            test_rst_mdr = 1'b0;
            test_rst_ABC =3'b000;
            test_pc_wr = 1'b1;
            test_alu_srcA = 0;
            test_alu_srcB = 3'b000;
            test_alu_op = 2'b00;
            test_pc_src = 1'b0;
        end

        #2 begin  //4
            test_pc_wr = 1'b0;
            test_regA = 2'b00;
            test_regB = 1'b0;
            test_read3 = 1'b1;
        end

        #2 begin  //6
            test_alu_srcA = 1'b1;
            test_alu_srcB = 3'b001;
            test_alu_op = 2'b00;
            test_output_cont =1'b0;
            // test_pc_src = 1'b1;
            // test_pc_wr = 1'b1;
        end

        // #2 begin  //8
        //     test_memr =1'b0;
        //     test_memw = 1'b1;
        // end

        #2 begin  //10
            test_reg_dst = 1'b0;
            test_mem_to_reg = 1'b0;
            test_reg_wr =1'b1;
        end

        #2 begin  //12
            test_pc_wr = 1'b1;
            test_alu_srcA = 0;
            test_alu_srcB = 3'b000;
            test_alu_op = 2'b00;
            test_pc_src = 1'b0;
        end

        #2 begin  //4
            test_pc_wr = 1'b0;
            test_regA = 2'b00;
            test_regB = 1'b0;
            test_read3 = 1'b1;
        end

        #2 begin  //6
            test_alu_srcA = 1'b1;
            test_alu_srcB = 3'b001;
            test_alu_op = 2'b01;
            test_output_cont =1'b0;
            // test_pc_src = 1'b1;
            // test_pc_wr = 1'b1;
        end

        #2 begin  //10
            test_reg_dst = 1'b0;
            test_mem_to_reg = 1'b0;
            test_reg_wr =1'b1;
        end

    end


    initial begin
        $monitor("Time = %3d,Clock = %b,PC_wr = %b,IR = %h,pc = %h,A =%h,B =%h,C=%h,ALUOut = %b, pc_in = %h %h %h ",$time,test_clk,test_pc_wr,test_ir,test_pc,test_A,test_B,test_C,test_alu_out,test_pc_in,uut.U6.register_file[3],uut.U6.register_file[1]);
    end

    initial begin
        #20 $finish;
    end

endmodule