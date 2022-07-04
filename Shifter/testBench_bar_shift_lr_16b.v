module testBench_bar_shift_lr_16b();

    wire [15:0]tb_out; // CUT Output 16 bit
    wire [15:0]tb_inp; // CUT Input 16 bit
    reg [3:0]tb_sel; // CUT Input Shift amount
    reg [1:0]tb_lr; // CUT Input type of shift operation : 01-logiacl left,10-logical right,11-arithmetic right shift
    reg [15:0]Expected_output; 
    reg check_bit;
    bar_shift_lr_16b uut(.b(tb_out),.a(tb_inp),.sel(tb_sel),.lr(tb_lr));
    assign tb_inp = 16'b1010_0101_1010_0101; // Input stimulus
    initial begin // Checking different shift operation
        tb_lr <= 2'b01;
        #11 tb_lr <= 2'b10;
        #10 tb_lr <= 2'b11;
    end
    initial begin // Checking different combination of shamt(shift amount)
        tb_sel = 4'b0000;
         forever #1 begin
         tb_sel = 4'b0000;
         #1 tb_sel = 4'b0001; 
         #1 tb_sel = 4'b0010;
         #1 tb_sel = 4'b0100;
         #1 tb_sel = 4'b1000;
         #1 tb_sel = 4'b1100;
         #1 tb_sel = 4'b1010;
         #1 tb_sel = 4'b0011;
         #1 tb_sel = 4'b0110;
         #1 tb_sel = 4'b0101;
         end
    end
    always@(*) begin // Calculating Expected output and check bit to see whether expected output matches that of CUT output
        case(tb_lr)
            2'b01:  Expected_output = tb_inp << tb_sel[3:0];
            2'b10:  Expected_output = tb_inp >> tb_sel[3:0];
            2'b11:  Expected_output = $signed(tb_inp) >>> tb_sel[3:0];
            default:  Expected_output = 16'bX;
        endcase
        check_bit = (Expected_output==tb_out) ;
    end
    initial begin // Displaying results
        $monitor("Time = %3d,Input = %b,Shift Operation = %b,Shamt = %b,Output = %b,Expected Output = %b,Check_bit = %b",
                    $time,tb_inp,tb_lr,tb_sel,tb_out,Expected_output,check_bit);
    end
    initial begin
        #30 $finish;
    end
endmodule