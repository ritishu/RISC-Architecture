module testbench_mux_alu_out();

    wire [15:0]test_alu_out;
    reg [15:0]test_alu;
    reg [15:0]test_shifter;
    reg test_clk;
    reg test_output_cont;
    mux_alu_out uut(.alu_out(test_alu_out),.alu(test_alu),.shifter(test_shifter),.clk(test_clk),.output_cont(test_output_cont));

    initial begin
        test_alu <= 16'h1234;
        test_shifter <= 16'hABCD;
    end

    initial begin
        test_clk <= 1'b0;
        forever #1 test_clk <= ~test_clk;
    end

    initial begin
        test_output_cont = 1'b0;
        #4 test_output_cont = 1'b1;
    end

    initial
    $monitor("Time =%3d,Clock =%b,Output_Cont = %b,Alu_out =%h",$time,test_clk,test_output_cont,test_alu_out);

    initial
    #10 $finish;
endmodule