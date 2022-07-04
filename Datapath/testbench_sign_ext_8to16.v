module testbench_sign_ext_8to16();

    reg [7:0]test_in;
    wire [15:0]test_out;
    sign_ext_8to16 uut(.in(test_in),.out(test_out));

    initial begin
        test_in <= 8'b1100_0011;
        #5 test_in <= 8'b0100_0011;
    end

    initial
    $monitor("Time = %3d, Input = %h, Output =%h",$time,test_in,test_out);
endmodule