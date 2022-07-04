module testbench_data_mem();

    wire [15:0]test_mdr;
    reg [15:0]test_addr;
    reg test_memr;
    reg test_memw;
    reg test_clk;
    reg [15:0]test_data_in;
    reg [15:0]written_data;
    reg [14:0]interm_addr;
    data_mem uut(.mdr(test_mdr),.memr(test_memr),.memw(test_memw),
                    .clk(test_clk),.data_in(test_data_in),.addr(test_addr));
    
    
    initial begin
        test_clk <= 1'b0;
        forever #1 test_clk <= ~test_clk;
    end

    initial begin
        test_data_in <= 16'hABCD;
        test_addr <= 16'h2340;
    end

    initial begin
        forever  begin
            #4 begin
                test_memr = 1'b0;test_memw =1'b0;
            end

            #4 begin
                test_memr = 1'b0;test_memw =1'b1;
            end

            #4 begin
                test_memr = 1'b1;test_memw =1'b0;
            end

            #4 begin
                test_memr = 1'b1;test_memw =1'b1;
            end
        end
    end

    always@(*)begin
        interm_addr = test_addr[15:1];
        written_data <= {uut.mem_odd[interm_addr],uut.mem_even[interm_addr]};
    end

    initial begin
        $monitor("Time = %3d, Clock = %b,MEMR = %b, MEMW = %b,MDR = %h,Written data = %h",
                    $time,test_clk,test_memr,test_memw,test_mdr,written_data);
    end

    initial
    #40 $finish;

endmodule