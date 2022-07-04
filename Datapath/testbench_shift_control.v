module testbench_shift_control();
    
    wire [1:0]test_sh_op;
    reg [3:0]test_opcode;
    reg [3:0]test_ffield;
    shift_control uut(.sh_op(test_sh_op),.opcode(test_opcode),.ffield(test_ffield));

    initial begin
        
        begin
            test_opcode = 4'b0000;
            test_ffield = 4'b0000;
        end

        #2 begin
            test_opcode = 4'b0000;
            test_ffield = 4'b0001;
        end

        #2 begin
            test_opcode = 4'b0000;
            test_ffield = 4'b0010;
        end

        #2 begin
            test_opcode = 4'b0000;
            test_ffield = 4'b0011;
        end

        #2 begin
            test_opcode = 4'b0000;
            test_ffield = 4'b0101;
        end

        #2 begin
            test_opcode = 4'b0000;
            test_ffield = 4'b1111;
        end

        #2 begin
            test_opcode = 4'b0001;
            test_ffield = 4'b0001;
        end

        #2 begin
            test_opcode = 4'b0010;
            test_ffield = 4'b0001;
        end

        #2 begin
            test_opcode = 4'b0100;
            test_ffield = 4'b0001;
        end

        #2 begin
            test_opcode = 4'b1000;
            test_ffield = 4'b0001;
        end

    end
    
    
    initial begin
    $monitor("Time = %3d,opcode = %b,ffield = %b,sh_op = %b",
                $time,test_opcode,test_ffield,test_sh_op);
    end


endmodule