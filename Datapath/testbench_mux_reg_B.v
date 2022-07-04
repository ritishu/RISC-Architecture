module testbench_mux_reg_B();
    
    reg  [3:0]test_in0;
    reg  [3:0]test_in1;
    wire [3:0]test_rn2;
    reg test_regB;
    mux_reg_B uut(.in0(test_in0),.in1(test_in1),.regB(test_regB),.rn2(test_rn2));

    initial begin
        test_in0 = 4'b1010;
        test_in1 = 4'b1101;
    end

    initial begin
        test_regB = 1'b0;
        #5 test_regB = 1'b1;
    end

    initial begin
        $monitor("RegB = %b, Rn_2 = %b",test_regB,test_rn2);
    end


endmodule