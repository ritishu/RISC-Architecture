module testbench_mux_reg_A();
    
    reg [3:0]test_ir7_4;
    reg [3:0]test_ir11_8;
    reg [1:0]test_regA;
    wire [3:0]test_rn1;
    mux_reg_A uut(.rn1(test_rn1),.ir7_4(test_ir7_4),.ir11_8(test_ir11_8),.regA(test_regA));

    initial begin
        test_ir11_8 = 4'b0111;
        test_ir7_4 = 4'b0101;
    end

    initial begin
        test_regA = 2'b00;
        #5 test_regA = 2'b01;
        #5 test_regA = 2'b10;
        #5 test_regA = 2'b11;
    end

    initial
    $monitor("RegA = %b,Rn1 = %b",test_regA,test_rn1);

endmodule