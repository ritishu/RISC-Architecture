module testbench_mux_alu_B();
    wire [15:0]test_alu_B;
    reg [11:0]test_ir11_0;                    
    reg [15:0]test_B;
    reg [2:0]test_alu_srcB;
    mux_alu_B uut(.alu_B(test_alu_B),.ir11_0(test_ir11_0),.B(test_B),.alu_srcB(test_alu_srcB));

    initial begin
        test_B = 16'hF231;
        test_ir11_0 = 12'h123;
    end

    initial begin
        test_alu_srcB = 3'b000;
        #1 test_alu_srcB = 3'b001;
        #1 test_alu_srcB = 3'b010;
        #1 test_alu_srcB = 3'b011;
        #1 test_alu_srcB = 3'b100;
        #1 test_alu_srcB = 3'b101;
        #1 test_alu_srcB = 3'b110;
        #1 test_alu_srcB = 3'b111;
    end

    initial
    $monitor("Time = %3d,Alu_src_B = %b,Alu_B = %h",$time,test_alu_srcB,test_alu_B);

endmodule