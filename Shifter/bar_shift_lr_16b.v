module bar_shift_lr_16b (a, sel, lr, b);

    input [15:0] a;                             //Input
    input [3:0] sel;                            //shift amount
    input [1:0]lr;                              //sh op from flowchart
    output [15:0] b;                            //Output

    wire [15:0] w1, w2;

    muxflip_2x1_16b switch (a, lr, w1);
    bar_shift_16b shift (w1, sel, w2, lr);
    muxflip_2x1_16b restore (w2, lr, b);

endmodule

module bar_shift_16b (a, sel, b, a_l);

    input [15:0] a;
    input [3:0] sel;
    input [1:0] a_l;
    output [15:0] b;    

    wire [15:0] x, y, z;
    wire bit_a = &a_l ? a[15] : 1'b0;
    wire bit_x = &a_l ? x[15] : 1'b0;
    wire bit_y = &a_l ? y[15] : 1'b0;
    wire bit_z = &a_l ? z[15] : 1'b0;
    
    mux_2x1_1b o3_0 (a[0], a[8], sel[3], x[0]); 
    mux_2x1_1b o3_1 (a[1], a[9], sel[3], x[1]); 
    mux_2x1_1b o3_2 (a[2], a[10], sel[3], x[2]); 
    mux_2x1_1b o3_3 (a[3], a[11], sel[3], x[3]); 
    mux_2x1_1b o3_4 (a[4], a[12], sel[3], x[4]); 
    mux_2x1_1b o3_5 (a[5], a[13], sel[3], x[5]); 
    mux_2x1_1b o3_6 (a[6], a[14], sel[3], x[6]); 
    mux_2x1_1b o3_7 (a[7], a[15], sel[3], x[7]); 
    mux_2x1_1b o3_8 (a[8], bit_a, sel[3], x[8]); 
    mux_2x1_1b o3_9 (a[9], bit_a, sel[3], x[9]); 
    mux_2x1_1b o3_10 (a[10], bit_a, sel[3], x[10]); 
    mux_2x1_1b o3_11 (a[11], bit_a, sel[3], x[11]); 
    mux_2x1_1b o3_12 (a[12], bit_a, sel[3], x[12]); 
    mux_2x1_1b o3_13 (a[13], bit_a, sel[3], x[13]); 
    mux_2x1_1b o3_14 (a[14], bit_a, sel[3], x[14]); 
    mux_2x1_1b o3_15 (a[15], bit_a, sel[3], x[15]); 

    mux_2x1_1b o2_0 (x[0], x[4], sel[2], y[0]); 
    mux_2x1_1b o2_1 (x[1], x[5], sel[2], y[1]); 
    mux_2x1_1b o2_2 (x[2], x[6], sel[2], y[2]); 
    mux_2x1_1b o2_3 (x[3], x[7], sel[2], y[3]); 
    mux_2x1_1b o2_4 (x[4], x[8], sel[2], y[4]); 
    mux_2x1_1b o2_5 (x[5], x[9], sel[2], y[5]); 
    mux_2x1_1b o2_6 (x[6], x[10], sel[2], y[6]); 
    mux_2x1_1b o2_7 (x[7], x[11], sel[2], y[7]); 
    mux_2x1_1b o2_8 (x[8], x[12], sel[2], y[8]); 
    mux_2x1_1b o2_9 (x[9], x[13], sel[2], y[9]); 
    mux_2x1_1b o2_10 (x[10], x[14], sel[2], y[10]); 
    mux_2x1_1b o2_11 (x[11], x[15], sel[2], y[11]); 
    mux_2x1_1b o2_12 (x[12], bit_x, sel[2], y[12]); 
    mux_2x1_1b o2_13 (x[13], bit_x, sel[2], y[13]); 
    mux_2x1_1b o2_14 (x[14], bit_x, sel[2], y[14]); 
    mux_2x1_1b o2_15 (x[15], bit_x, sel[2], y[15]); 

    mux_2x1_1b o1_0 (y[0], y[2], sel[1], z[0]); 
    mux_2x1_1b o1_1 (y[1], y[3], sel[1], z[1]); 
    mux_2x1_1b o1_2 (y[2], y[4], sel[1], z[2]); 
    mux_2x1_1b o1_3 (y[3], y[5], sel[1], z[3]); 
    mux_2x1_1b o1_4 (y[4], y[6], sel[1], z[4]); 
    mux_2x1_1b o1_5 (y[5], y[7], sel[1], z[5]); 
    mux_2x1_1b o1_6 (y[6], y[8], sel[1], z[6]); 
    mux_2x1_1b o1_7 (y[7], y[9], sel[1], z[7]); 
    mux_2x1_1b o1_8 (y[8], y[10], sel[1], z[8]); 
    mux_2x1_1b o1_9 (y[9], y[11], sel[1], z[9]); 
    mux_2x1_1b o1_10 (y[10], y[12], sel[1], z[10]); 
    mux_2x1_1b o1_11 (y[11], y[13], sel[1], z[11]); 
    mux_2x1_1b o1_12 (y[12], y[14], sel[1], z[12]); 
    mux_2x1_1b o1_13 (y[13], y[15], sel[1], z[13]); 
    mux_2x1_1b o1_14 (y[14], bit_y, sel[1], z[14]); 
    mux_2x1_1b o1_15 (y[15], bit_y, sel[1], z[15]); 

    mux_2x1_1b o0_0 (z[0], z[1], sel[0], b[0]); 
    mux_2x1_1b o0_1 (z[1], z[2], sel[0], b[1]); 
    mux_2x1_1b o0_2 (z[2], z[3], sel[0], b[2]); 
    mux_2x1_1b o0_3 (z[3], z[4], sel[0], b[3]); 
    mux_2x1_1b o0_4 (z[4], z[5], sel[0], b[4]); 
    mux_2x1_1b o0_5 (z[5], z[6], sel[0], b[5]); 
    mux_2x1_1b o0_6 (z[6], z[7], sel[0], b[6]); 
    mux_2x1_1b o0_7 (z[7], z[8], sel[0], b[7]); 
    mux_2x1_1b o0_8 (z[8], z[9], sel[0], b[8]); 
    mux_2x1_1b o0_9 (z[9], z[10], sel[0], b[9]); 
    mux_2x1_1b o0_10 (z[10], z[11], sel[0], b[10]); 
    mux_2x1_1b o0_11 (z[11], z[12], sel[0], b[11]); 
    mux_2x1_1b o0_12 (z[12], z[13], sel[0], b[12]); 
    mux_2x1_1b o0_13 (z[13], z[14], sel[0], b[13]); 
    mux_2x1_1b o0_14 (z[14], z[15], sel[0], b[14]); 
    mux_2x1_1b o0_15 (z[15], bit_z, sel[0], b[15]); 

endmodule


module mux_2x1_1b (a, b, sel, out);

    input a, b;
    input sel;
    output out;
    
    assign out = sel ? b : a;

endmodule

module muxflip_2x1_16b (in, lr, out);

    input [15:0] in;
    input [1:0]lr;                           //right shift for barrel shifter when lr is 0
    output [15:0] out;

    mux_2x1_1b m0 (in[0], in[15], ~lr[1], out[0]);
    mux_2x1_1b m1 (in[1], in[14], ~lr[1], out[1]);
    mux_2x1_1b m2 (in[2], in[13], ~lr[1], out[2]);
    mux_2x1_1b m3 (in[3], in[12], ~lr[1], out[3]);
    mux_2x1_1b m4 (in[4], in[11], ~lr[1], out[4]);
    mux_2x1_1b m5 (in[5], in[10], ~lr[1], out[5]);
    mux_2x1_1b m6 (in[6], in[9], ~lr[1], out[6]);
    mux_2x1_1b m7 (in[7], in[8], ~lr[1], out[7]);
    mux_2x1_1b m8 (in[8], in[7], ~lr[1], out[8]);
    mux_2x1_1b m9 (in[9], in[6], ~lr[1], out[9]);
    mux_2x1_1b m10 (in[10], in[5], ~lr[1], out[10]);
    mux_2x1_1b m11 (in[11], in[4], ~lr[1], out[11]);
    mux_2x1_1b m12 (in[12], in[3], ~lr[1], out[12]);
    mux_2x1_1b m13 (in[13], in[2], ~lr[1], out[13]);
    mux_2x1_1b m14 (in[14], in[1], ~lr[1], out[14]);
    mux_2x1_1b m15 (in[15], in[0], ~lr[1], out[15]);

endmodule
