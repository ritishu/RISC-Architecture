module testbench_ALU();
    
    wire [15:0]test_alu;
    wire test_zf;
    reg [15:0]test_alu_A,test_alu_B;
    reg [1:0] test_alu_op;
    ALU uut(.alu(test_alu),.zf(test_zf),.alu_A(test_alu_A),.alu_B(test_alu_B),
                .alu_op(test_alu_op));

    initial begin
        #1 begin
            test_alu_A <= 16'h0DCA;
            test_alu_B <= 16'h0234;
        end

        #4 begin
            test_alu_A <= 16'hFDCA;
            test_alu_B <= 16'h0234;
        end

        #4 begin
            test_alu_A <= 16'h0DCA;
            test_alu_B <= 16'hF234;
        end

        #4 begin
            test_alu_A <= 16'hf234;
            test_alu_B <= 16'hf234;
        end
    end

    initial begin
        forever begin
            #1 test_alu_op = 2'b00;
            #1 test_alu_op = 2'b01;
            #1 test_alu_op = 2'b10;
            #1 test_alu_op = 2'b11;
        end
    end

    initial 
        $monitor("Time = %3d,ALU_op = %b,ALU_A = %h,ALU_B = %h,ALU = %h,Zero Flag = %b",
                    $time,test_alu_op,test_alu_A,test_alu_B,test_alu,test_zf);

    initial
    #16 $finish;
endmodule