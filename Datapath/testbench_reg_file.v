module testbench_reg_file();

    wire [15:0]test_A;
    wire [15:0]test_B;
    wire [15:0]test_C;
    reg [15:0]written_data;

    reg [15:0]test_wd;
    reg [3:0]test_rn1;
    reg [3:0]test_rn2;
    reg [3:0]test_rn3;
    reg [3:0]test_wr;
    reg test_clk;
    reg test_reg_wr;
    reg test_read3;
    reg_file uut(.A(test_A),.B(test_B),.C(test_C),.wd(test_wd),.rn1(test_rn1),
                    .rn2(test_rn2),.rn3(test_rn3),.wr(test_wr),.clk(test_clk),
                    .reg_wr(test_reg_wr),.read3(test_read3));
    
    initial begin
        test_rn1 <= 4'b0010; 
        test_rn2 <= 4'b0100;
        test_rn3 <= 4'b0001;
        test_wr  <= 4'b1111;//Checked for 4'b0000 as well, then test_wd = 16'h0000
        test_wd  <= 16'hFFFF;
    end

    // initial begin
    //     uut.register_file[test_rn1] <= 16'hABCD;
    //     uut.register_file[test_rn2] <= 16'h3579;
    //     uut.register_file[test_rn3] <= 16'h2468;
    // end

    initial begin
        test_clk <= 1'b0;
        forever #1 test_clk <= ~test_clk;
    end

    initial begin
        forever  begin
            #4 begin
                test_reg_wr <= 1'b0;test_read3 <= 1'b0;
            end

            #4 begin
                test_reg_wr <= 1'b0;test_read3 <= 1'b1;
            end

            #4 begin
                test_reg_wr <= 1'b1;test_read3 <=1'b0;
            end

            #4 begin
                test_reg_wr <= 1'b1;test_read3 <=1'b1;
            end
        end
    end

    always@(*)begin
        written_data <= uut.register_file[test_wr];
    end

    initial begin
        $monitor("Time = %3d,Clk = %b,Reg_wr = %b, Read3 = %b,A = %h, B = %h, C = %h,Written Data = %h"
                    ,$time,test_clk,test_reg_wr,test_read3,test_A,test_B,test_C,written_data);
    end
    
    initial
        #50 $finish;


endmodule