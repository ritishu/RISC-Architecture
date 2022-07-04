module testbench_fsm();

    reg [3:0]test_op;
    reg test_clock,test_start;
    wire test_PcWr,test_ALUSrcA,test_Output,test_RegB,test_Read3,test_RegDst,test_MemtoReg,
        test_RegWr,test_MemR,test_MemW,test_EQbar,test_PcSrc;
    wire [1:0]test_ALUOp,test_RegA;
    wire [2:0]test_ALUSrcB;
    fsm uut(.op(test_op),.clock(test_clock),.start(test_start),.PcWr(test_PcWr),.ALUSrcA(test_ALUSrcA),
             .Output(test_Output),.RegB(test_RegB),.Read3(test_Read3),.RegDst(test_RegDst),.MemtoReg(test_MemtoReg),
             .RegWr(test_RegWr),.MemR(test_MemR),.MemW(test_MemW),.EQbar(test_EQbar),.ALUOp(test_ALUOp),
             .RegA(test_RegA),.ALUSrcB(test_ALUSrcB),.PcSrc(test_PcSrc));
    reg [21:0]test_current_state;
    reg [21:0]test_next_state;
    
    
    initial begin
        test_op = 4'b1000;
        test_clock = 1'b0;
        forever #1 test_clock = ~test_clock;
        
    end
    initial begin
        test_start = 1'b0;
        #1 test_start = 1'b1;
    end

    initial begin
        $display("Operation = %b",test_op);
        $monitor("Clock = %b,Current State = %h,Next State = %h, PcWr = %b,ALUSrcA = %b,ALUSrcB = %b,ALUOp = %b, Output = %b,RegA = %b,RegB = %b,Read3 = %b, RegDst = %b,MemtoReg = %b, RegWr =%b, MemR =%b, MemW = %b,EQbar =%b,PcSrc = %b",
                    test_clock,uut.current_state, uut.next_state, test_PcWr,test_ALUSrcA,test_ALUSrcB,test_ALUOp,test_Output,test_RegA,test_RegB,
                    test_Read3,test_RegDst,test_MemtoReg,test_RegWr,test_MemR,test_MemW,test_EQbar,test_PcSrc);
    end

    initial
    #10 $finish;

endmodule