module testbench_instruct_mem();

    reg tclk;
    reg tinstr_wr;
    reg tpc_wr1;
    reg [15:0] tinstr_in;
    reg [15:0] tpc_in;
    wire [15:0] tir;
    wire [15:0] tpc;
    instruct_mem uut(.clk(tclk),.instr_wr(tinstr_wr),.pc_wr1(tpc_wr1),.instr_in(tinstr_in),.pc_in(tpc_in),.ir(tir),.pc(tpc));

    initial begin
        tclk <= 1'b0;
        forever #1 tclk <= ~tclk;
    end

    initial begin
    tinstr_in <= 16'hA8B7;
    tpc_in <= 16'hB7A8;
    end

    initial begin
        // begin
        //         tinstr_wr = 1'b0;tpc_wr1 =1'b0;
        // end
        forever  begin
            #4 begin
                tinstr_wr = 1'b0;tpc_wr1 =1'b0;
            end

            #4 begin
                tinstr_wr = 1'b0;tpc_wr1 =1'b1;
            end

            #4 begin
                tinstr_wr = 1'b1;tpc_wr1 =1'b0;
            end

            #4 begin
                tinstr_wr = 1'b1;tpc_wr1 =1'b1;
            end
        end
    end

    initial begin
        $monitor("Time = %3d,Clk = %b,Instr_wr = %b,Pc_wr = %b,IR = %h,PC=%h"
                    ,$time,tclk,tinstr_wr,tpc_wr1,tir,tpc);
    end

    initial
        #50 $finish;


endmodule