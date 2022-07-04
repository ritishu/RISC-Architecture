module testbench_mux_reg_dst();

    reg [3:0]test_ir11_8;
    reg test_reg_dst;
    wire [3:0]test_wr;
    mux_reg_dst uut(.wr(test_wr),.reg_dst(test_reg_dst),.ir11_8(test_ir11_8));

    initial
    test_ir11_8 = 4'b0010;

    initial begin
        test_reg_dst = 1'b0;
        #5 test_reg_dst = 1'b1;
    end

    initial
    $monitor("Reg_dst = %b, Wr = %b",test_reg_dst,test_wr);


endmodule