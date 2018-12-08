module noise_generator (clk, enable, Q);
	
	input clk, enable;
	output [23:0] Q;

	
	reg [2:0] count;

	always @(posedge clk)
	begin
		if (enable)
			count <= count + 1'b1;
	end		
	
	assign Q = {{10{count[2]}}, count, 11'd0};

endmodule