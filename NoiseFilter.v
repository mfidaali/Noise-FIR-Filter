module NoiseFilter (CLOCK_50, CLOCK2_50, KEY, FPGA_I2C_SCLK, FPGA_I2C_SDAT, AUD_XCK, 
		        AUD_DACLRCK, AUD_ADCLRCK, AUD_BCLK, AUD_ADCDAT, AUD_DACDAT);

	input CLOCK_50, CLOCK2_50;
	input [0:0] KEY;
	
	// I2C Audio/Video config interface
	output FPGA_I2C_SCLK;
	inout FPGA_I2C_SDAT;
	
	// Audio CODEC
	input AUD_DACLRCK, AUD_ADCLRCK, AUD_BCLK;
	input AUD_ADCDAT;
	output AUD_XCK;
	output AUD_DACDAT;


//logic declaration
	wire reset = ~KEY[0];

	
	wire read_ready, write_ready;
	reg read, write;
	wire [23:0] readdata_left, readdata_right;
	wire [23:0] readdata_left_with_noise, readdata_right_with_noise;
	reg [23:0] writedata_left, writedata_right;
	//reg [23:0] writedatareg_left, writedatareg_right;
	
	reg [23:0] shiftregdata_left[0:6];  //array of 7 registers holding 24 bit values each
	reg [23:0] shiftregdata_right[0:6]; //array of 7 registers holding 24 bit values each

	wire [23:0] filterresult_left;
	wire [23:0] filterresult_right;
	
	
	//Noise generator
	//Output of noise generator added to sample value to produce artificial noise in sample (this has been done to test the efficacy of the FIR filter)
	wire noise_en;
	wire [23:0] supplement_noise; // Added to audio sample value from Codec
	
	assign noise_en = read_ready & read; //enable noise generator only 
	noise_generator noisy (CLOCK_50, noise_en, supplement_noise); 
	
	
	assign readdata_left_with_noise = readdata_left + supplement_noise; //Added noise into next sample coming into filter
	assign readdata_right_with_noise = readdata_right + supplement_noise; //Added noise into next sample coming into filter
	
	//Read into shift register on read_ready
	integer i;
	always @(posedge CLOCK_50 or posedge reset)
	begin
		if (reset) begin
			for (i=0; i<7; i=i+1) begin
				shiftregdata_left[i] <= 24'b0;
				shiftregdata_right[i] <= 24'b0;
			end
			read <= 0; 	
		end		
		else begin
			if (read_ready) begin
				for (i=1; i<6; i=i+1) begin
					shiftregdata_left[i+1]  <= shiftregdata_left[i];
					shiftregdata_right[i+1] <= shiftregdata_right[i];
				end
				shiftregdata_left[0] <=  readdata_left_with_noise;
				shiftregdata_right[0] <= readdata_right_with_noise;
					//writedatareg_left  <= filterresult_left;
					//writedatareg_right <= filterresult_right;
 				read <= 1; //1 or 0?
			end
		end		
	end
	
	//Output filtered signal on write_ready
	always @(posedge CLOCK_50 or posedge reset)
	begin
		if (reset)
			begin
				writedata_left <= 0;
				writedata_right <= 0;
				write <= 0;
			end
		else
			begin
				if (write_ready)
					begin
						writedata_left <=  filterresult_left;
						writedata_right <= filterresult_right;
						write <= 1;
					end
				else
					write <= 0; 
			end
	end
	
	
	//Average moving filter for sample values (average last 8 samples)
	assign filterresult_left = 	readdata_left_with_noise>>3 + 
											shiftregdata_left[0]>>3 + 
											shiftregdata_left[1]>>3 + 
											shiftregdata_left[2]>>3 + 
											shiftregdata_left[3]>>3 +
											shiftregdata_left[4]>>3 + 
											shiftregdata_left[5]>>3 + 
											shiftregdata_left[6]>>3;

	assign filterresult_right = 	readdata_right_with_noise>>3 + 
											shiftregdata_right[0]>>3 + 
											shiftregdata_right[1]>>3 + 
											shiftregdata_right[2]>>3 + 
											shiftregdata_right[3]>>3 +
											shiftregdata_right[4]>>3 + 
											shiftregdata_right[5]>>3 + 
											shiftregdata_right[6]>>3;

/////////////////////////////////////////////////////////////////////////////////
// Audio CODEC interface. 
//
// The interface consists of the following wires:
// read_ready, write_ready - CODEC ready for read/write operation 
// readdata_left, readdata_right - left and right channel data from the CODEC
// read - send data from the CODEC (both channels)
// writedata_left, writedata_right - left and right channel data to the CODEC
// write - send data to the CODEC (both channels)
// AUD_* - should connect to top-level entity I/O of the same name.
//         These signals go directly to the Audio CODEC
// I2C_* - should connect to top-level entity I/O of the same name.
//         These signals go directly to the Audio/Video Config module
/////////////////////////////////////////////////////////////////////////////////
	clock_generator my_clock_gen(
		// inputs
		CLOCK2_50,
		reset,

		// outputs
		AUD_XCK
	);

	audio_and_video_config cfg(
		// Inputs
		CLOCK_50,
		reset,

		// Bidirectionals
		FPGA_I2C_SDAT,
		FPGA_I2C_SCLK
	);

	audio_codec codec(
		// Inputs
		CLOCK_50,
		reset,

		read,	write,
		writedata_left, writedata_right,

		AUD_ADCDAT,

		// Bidirectionals
		AUD_BCLK,
		AUD_ADCLRCK,
		AUD_DACLRCK,

		// Outputs
		read_ready, write_ready,
		readdata_left, readdata_right,
		AUD_DACDAT
	);

endmodule


