`timescale 1 ns / 1 ps

module system (
	input            clk,
	input            resetn,
	output           trap,
	output [7:0]     out_byte,
	output           out_byte_en,
	input            sw0,
	output [7:0]     anodos, catodos
);

	wire clk_div;
	wire [31:0] num_to_screen;

	// Interfaz cache Pircov
	wire mem_valid;
	wire mem_instr;
	wire mem_ready;
	wire mem_ready_delay;
	wire [31:0] mem_addr;
	wire [31:0] mem_wdata;
	wire [3:0]  mem_wstrb;
	wire [31:0] mem_rdata;

	wire mem_la_read;
	wire mem_la_write;
	wire [31:0] mem_la_addr;
	wire [31:0] mem_la_wdata;
	wire [3:0]  mem_la_wstrb;

	// Interfaz cache MP
	wire mem_valid_MP;
	wire mem_ready_MP;
	wire [31:0] mem_addr_MP;
	wire [31:0] mem_wdata_MP;
	wire [3:0] mem_wstrb_MP;
	wire [31:0] mem_rdata_MP;

	picorv32 picorv32_core (
		.clk         (clk            ),
		.resetn      (resetn         ),
		.trap        (trap           ),
		.mem_valid   (mem_valid      ),
		.mem_instr   (mem_instr      ),
		.mem_ready   (mem_ready),
		.mem_addr    (mem_addr       ),
		.mem_wdata   (mem_wdata      ),
		.mem_wstrb   (mem_wstrb      ),
		.mem_rdata   (mem_rdata      ),
		.mem_la_read (mem_la_read    ),
		.mem_la_write(mem_la_write   ),
		.mem_la_addr (mem_la_addr    ),
		.mem_la_wdata(mem_la_wdata   ),
		.mem_la_wstrb(mem_la_wstrb   )
	);

	cache_directo cache ( .clk(clk), .resetn(resetn),
		// Pircov
		.mem_valid(mem_valid),
		.mem_instr(mem_instr),
		.mem_ready(mem_ready),
		.mem_addr (mem_addr ),
		.mem_wdata(mem_wdata),
		.mem_wstrb(mem_wstrb),
		.mem_rdata(mem_rdata),
		// MP
		.mem_wdata_MP(mem_wdata_MP),
		.mem_wstrb_MP(mem_wstrb_MP),
		.mem_rdata_MP(mem_rdata_MP),
		.mem_valid_MP(mem_valid_MP),
		.mem_ready_MP(mem_ready_delay),
		.mem_addr_MP(mem_addr_MP ),
		.mem_instr_MP(mem_instr_MP)
	);

	mem_prin principal (
		.clk            (clk            ),
		.resetn         (resetn         ),
		.mem_valid      (mem_valid_MP      ),
		.mem_instr      (mem_instr_MP      ),
		.mem_ready      (mem_ready_MP      ),
		.mem_ready_delay(mem_ready_delay),
		.mem_addr       (mem_addr_MP       ),
		.mem_wdata      (mem_wdata_MP      ),
		.mem_wstrb      (mem_wstrb_MP      ),
		.mem_rdata      (mem_rdata_MP      ),
		.mem_la_read    (mem_la_read_MP    ),
		.mem_la_write   (mem_la_write_MP   ),
		.mem_la_addr    (mem_la_addr    ),
		.mem_la_wdata   (mem_la_wdata   ),
		.mem_la_wstrb   (mem_la_wstrb   ),
		.out_byte       (out_byte       ),
		.out_byte_en    (out_byte_en    ),
		.num_to_screen  (num_to_screen )
	);

	clock_divider clkk (
		.clk         (clk         ),
		.resetn      (resetn      ),
		.clk_div     (clk_div     )
	);

	seven_segment_hex screen (
		.sw0          (sw0          ),
		.clk          (clk_div      ),
		.resetn       (resetn       ),
		.num_to_screen(num_to_screen),
		.catodos      (catodos      ),
		.anodos       (anodos       )
	);

endmodule

module cache_directo #( parameter CACHE_SIZE = 1024 , parameter BLOCK_BYTES = 8, parameter ASOCIATIVITY = 2
)(						// CACHE SIZE Y BLOCK_BYTES EN BYTES
	// Señales con PIRCOV
	input 			  clk, resetn,
	output reg [31:0] num_to_screen,
	output reg [7:0 ] out_byte,
	output reg        out_byte_en, mem_ready, mem_ready_delay,
	input 			  mem_valid, mem_instr,
	input      [31:0] mem_addr, mem_wdata,
	input      [3:0 ] mem_wstrb, mem_la_wstrb,
	output reg [31:0] mem_rdata,
	input      [31:0] mem_la_addr, mem_la_wdata,
	input 			  mem_la_read, mem_la_write,


	// Señales con MEM_PRN
	input             mem_ready_MP, mem_ready_delay_MP,
	output reg 		  mem_valid_MP, mem_instr_MP,
	output reg [31:0] mem_addr_MP, mem_wdata_MP,
	output reg [3:0 ] mem_wstrb_MP, mem_la_wstrb_MP,
	input      [31:0] mem_rdata_MP, 
	output reg [31:0] mem_la_addr_MP, mem_la_wdata_MP,
	output reg 		  mem_la_read_MP, mem_la_write_MP
);
	// Definicion de parametros del cache
	parameter WORD_BYTES = 4; // BYTES POR PALABRA
    parameter WORD_BITS  = 8*WORD_BYTES; // BITS POR PALABRA

	parameter ADDR_BITS = 32; // BITS POR ADDRESS

	parameter BLOCK_WORDS = BLOCK_BYTES/WORD_BYTES; // PALABRAS POR BLOQUE
	parameter BLOCK_BITS  = 8*BLOCK_BYTES; // BITS POR BLOQUE

    parameter NUM_BLOCK  = CACHE_SIZE/BLOCK_BYTES; // NUMERO DE BLOQUES
	parameter NUM_FILES = NUM_BLOCK/ASOCIATIVITY;
	
	// Definicion del tamaño de señales
    parameter OFFSET_SIZE = $clog2(BLOCK_BYTES); // OFFSET PARA BYTES POR BLOQUE
	parameter INDEX_SIZE  = $clog2(NUM_BLOCK); // INDICE POR BLOQUE
	parameter TAG_SIZE    = 32-INDEX_SIZE-OFFSET_SIZE; // TAG
    
	// Arreglo de la caches
	reg                     dirty [0:NUM_BLOCK-1];
	reg                     valid [0:NUM_BLOCK-1];
	reg [TAG_SIZE-1:0  ]    tag   [0:NUM_BLOCK-1];
	reg [BLOCK_BITS-1:0]    data  [0:NUM_BLOCK-1];

	// Contadores hit y miss, asi como sus banderas
	reg hit_flag = 0;
	reg miss_flag = 0;
	reg [31:0] hits = 0;
	reg [31:0] miss = 0;
	reg [31:0] accesos = 0;
	reg [31:0] w_mem_address;

	// Asignacion de tag, offset, index
	wire [TAG_SIZE-1:0   ] tag_w;
    wire [INDEX_SIZE-1:0 ] index;
    wire [OFFSET_SIZE-1:0] offset;

	assign offset = w_mem_address[OFFSET_SIZE-1:0                     ];
    assign index  = w_mem_address[INDEX_SIZE+OFFSET_SIZE-1:OFFSET_SIZE];
    assign tag_w    = w_mem_address[ADDR_BITS-1:INDEX_SIZE+OFFSET_SIZE  ];

	// Enteros para los for loop
	integer i,j; 

	initial begin
		hit_flag  <= 0;
		miss_flag <= 0;
		hits      <= 0;
		miss    <= 0;
		accesos   <= 0;
		state     <= IDLE;
		for (i = 0; i < NUM_BLOCK; i = i + 1) begin
			data [i] <= 0;  
			dirty[i] <= 0;
			valid[i] <= 0;
			tag  [i] <= 0;
		end
	end
	// Offset para WRITEBACK y MEM_ACCESS
	reg [OFFSET_SIZE-1:0] offsetMP;

	// Definicion de estados ONE-HOT
	reg [7:0] state, next_state;

	parameter IDLE  = 1;
    parameter READ  = 2;
    parameter WRITE = 4;
    parameter MEM_ACCESS  = 8;
    parameter WRITE_BACK  = 16;

	initial next_state = IDLE;
	initial w_mem_address = 1;

	always @(posedge clk) begin
        if (resetn == 0) begin
            hit_flag  <= 0;
        end
		else begin
			state <= next_state;
		end
	end

	always @(*) begin
		case (state)
		// COMIENZA ESTADO IDLE
			IDLE: begin
				hit_flag  = 0;
				miss_flag = 0;
				mem_ready = 0; 
				if (mem_valid && w_mem_address != mem_addr) begin
					w_mem_address = mem_addr;
					if (|mem_wstrb) begin
						next_state = WRITE;
					end 
					else if (!mem_wstrb) begin
						next_state = READ;
					end
				end
			end

		// COMIENZA ESTADO READ
				// Si tag coincide y el bit de valido esta activado
				// Se completa la transaccion
			READ: begin
				if ((tag[index] == tag_w) && valid[index]) begin 
					if (!miss_flag) begin
						hit_flag  = 1;
						hits = hits + 1;
						accesos   = accesos   + 1;
					end
					mem_rdata = data[index][(offset>>2)*32+:32]; // NOTA: El byte offset se pasa											 
					next_state     = IDLE;						      // a word offset
					mem_ready = 1;
				end else begin
					// MISS
					mem_ready = 0;
					offsetMP  = 0;
					miss_flag = 1;
					miss      = miss + 1;
					accesos   = accesos   + 1;
					// Se revisa el dirty bit
					if (dirty[index] == 0) begin
						mem_addr_MP = w_mem_address & (32'hFFFFFFFF<<(OFFSET_SIZE)); // Se eliminan los offset bit 
						next_state = MEM_ACCESS;  							  						// de la direccion
					end 													 						// Esto nos deja iniciar en la
					else begin  	
						mem_addr_MP = {tag[index], index, offset} & (32'hFFFFFFFF<<(OFFSET_SIZE));	// primera palabra en MP
						next_state = WRITE_BACK;
					end
				end
			end
		// COMIENZA ESTADO WRITE
			WRITE: begin
				if ((tag[index] == tag_w) && valid[index]) begin
					if (!miss_flag) begin
						hit_flag     = 1;
						hits    = hits + 1;
						accesos      = accesos   + 1;
					end
					dirty[index] = 1;
					data[index][(offset>>2)*32+:32] = mem_wdata; // NOTA: El byte offset se pasa		
					next_state     = IDLE;						      // a word offset
					mem_ready = 1;						    
				end else begin									   
					// MISS
					mem_ready = 0;
					offsetMP  = 0;
					miss_flag = 1;
					miss      = miss + 1;
					accesos   = accesos   + 1;
					// Se revisa el dirty bit
					if (dirty[index] == 0) begin
						mem_addr_MP = w_mem_address & (32'hFFFFFFFF<<(OFFSET_SIZE)); // Se eliminan los offset bit 
						next_state = MEM_ACCESS;  								  // de la direccion
					end 													 // Esto nos deja iniciar en la
					else begin  	
						mem_addr_MP = {tag[index], index, offset} & (32'hFFFFFFFF<<(OFFSET_SIZE));	// primera palabra en MP
						next_state = WRITE_BACK;
					end
				end
			end
		// COMIENZA ESTADO MEM_ACCESS
			MEM_ACCESS: begin
				mem_wstrb_MP = 0;
				mem_valid_MP = 1;
				mem_wdata_MP = mem_wdata;
				if (mem_ready_MP) begin
					// Se debe traer todo el bloque
					data[index][(offsetMP)*32+:32] = mem_rdata_MP;
					offsetMP = offsetMP + 1;
					mem_addr_MP = mem_addr_MP + 4;
					if (offsetMP == BLOCK_WORDS) begin
						valid[index] = 1;  
						dirty[index] = 0; 
						tag  [index] = tag_w;   
						mem_valid_MP = 0;  
						if (|mem_wstrb) begin
							next_state = WRITE;
						end 
						else if (!mem_wstrb) begin
							next_state = READ;
						end
					end
				end 
			end
		// COMIENZA ESTADO WRITE_BACK
			WRITE_BACK: begin
				mem_valid_MP = 1; 
				mem_wstrb_MP = 4'hF;
				mem_wdata_MP = data[offsetMP][index];
				if (mem_ready_MP) begin
					mem_addr_MP = mem_addr_MP + 4;
					offsetMP = offsetMP + 1;
					if (offsetMP == BLOCK_WORDS) begin
						//valid[index] <= 1;  
						//dirty[index] <= 0; 
						//tag  [index] <= tag_w;
						//mem_valid_MP <= 0;
						//if (|mem_wstrb) begin
						//	state <= WRITE;
						//end 
						//else if (!mem_wstrb) begin
						//	state <= READ;
						next_state = MEM_ACCESS;
					end
				end
			end
		endcase
	end 

endmodule


module mem_prin (
	input 			  clk, resetn,
	output reg [31:0] num_to_screen,
	output reg [7:0]  out_byte,
	output reg        out_byte_en, mem_ready, mem_ready_delay,
	input 			  mem_valid, mem_instr,
	input      [31:0] mem_addr, mem_wdata,
	input      [3:0]  mem_wstrb, mem_la_wstrb,
	output reg [31:0] mem_rdata, 
	input      [31:0] mem_la_addr, mem_la_wdata,
	input 			  mem_la_read, mem_la_write
);
	// set this to 0 for better timing but less performance/MHz
	parameter FAST_MEMORY = 0;

	// 16384 32bit words = 64kB memory
	parameter MEM_SIZE = 16384;

	// Se crea la memoria
	reg [31:0] memory [0:MEM_SIZE-1];

	integer i;
	initial begin
		for (i = 4096; i < MEM_SIZE; i = i + 1) begin
			memory[i] <= 0;
		end
	end



`ifdef SYNTHESIS
    initial $readmemh("../firmware/firmware.hex", memory);
`else
	initial $readmemh("firmware.hex", memory);
`endif

	reg [31:0] m_read_data;
	reg m_read_en;

	reg [3:0] delay;
	reg [3:0] delay_cnt;

	generate if (FAST_MEMORY) begin
		always @(posedge clk) begin
			mem_ready <= 1;
			out_byte_en <= 0;
			mem_rdata <= memory[mem_la_addr >> 2];
			if (mem_la_write && (mem_la_addr >> 2) < MEM_SIZE) begin
				if (mem_la_wstrb[0]) memory[mem_la_addr >> 2][ 7: 0] <= mem_la_wdata[ 7: 0];
				if (mem_la_wstrb[1]) memory[mem_la_addr >> 2][15: 8] <= mem_la_wdata[15: 8];
				if (mem_la_wstrb[2]) memory[mem_la_addr >> 2][23:16] <= mem_la_wdata[23:16];
				if (mem_la_wstrb[3]) memory[mem_la_addr >> 2][31:24] <= mem_la_wdata[31:24];
			end
			else
			if (mem_la_write && mem_la_addr == 32'h1000_0000) begin
				out_byte_en <= 1;
				out_byte <= mem_la_wdata;
			end
		end
	end else begin
		always @(posedge clk) begin
			m_read_en <= 0;
			mem_ready <= mem_valid && !mem_ready && m_read_en;

			m_read_data <= memory[mem_addr >> 2];
			mem_rdata <= m_read_data;

			out_byte_en <= 0;

			if (mem_valid) begin
				if (mem_wstrb) delay <= 4'b1100;
				else delay <= 4'b0111;
				if (delay_cnt == delay) begin
					delay_cnt <= 0;
					mem_ready_delay <= 1;
				end
				else begin 
					delay_cnt <= delay_cnt + 1;
					mem_ready_delay <= 0;
				end 
			end 
			else begin
				mem_ready_delay <= 0;
				delay_cnt <= 0;
			end

			(* parallel_case *)
			case (1)
				// Lectura
				mem_valid && !mem_ready && !mem_wstrb && (mem_addr >> 2) < MEM_SIZE: begin
					m_read_en <= 1;
					mem_ready <= 1;
					mem_rdata <= memory[mem_addr >> 2];
				end
				// Escritura a memoria
				mem_valid && !mem_ready && |mem_wstrb && (mem_addr >> 2) < MEM_SIZE: begin
						if (mem_wstrb[0]) memory[mem_addr >> 2][ 7: 0] <= mem_wdata[ 7: 0];
						if (mem_wstrb[1]) memory[mem_addr >> 2][15: 8] <= mem_wdata[15: 8];
						if (mem_wstrb[2]) memory[mem_addr >> 2][23:16] <= mem_wdata[23:16];
						if (mem_wstrb[3]) memory[mem_addr >> 2][31:24] <= mem_wdata[31:24];
						mem_ready <= 1;
				end
				// Leds
				mem_valid && !mem_ready && |mem_wstrb && mem_addr == 32'h1000_0000: begin
						out_byte_en <= 1;
						out_byte <= mem_wdata;
						num_to_screen <= mem_la_wdata;
						mem_ready <= 1;
				end
			endcase
		end
	end endgenerate
endmodule

module seven_segment_hex (
	input clk, resetn, sw0,
	input [31:0] num_to_screen,
	output reg [7:0] catodos, anodos
);

reg [2:0] i, nxt_i;
reg [7:0] nxt_catodos;

initial nxt_i = 0;
initial i = 0;

always @ (posedge clk) begin
	if (sw0 == 1) begin
		anodos <= 8'hFF;
	end else 
	begin
		anodos = 8'hFF;
		anodos[i] <= 0;
		i <= nxt_i;
		catodos <= nxt_catodos;
	end
end

always @ (*) begin
	case (num_to_screen[i*4+:4])
		4'h0: nxt_catodos <= 8'b11000000;
		4'h1: nxt_catodos <= 8'b11111001;
		4'h2: nxt_catodos <= 8'b10100100;
		4'h3: nxt_catodos <= 8'b10110000;
		4'h4: nxt_catodos <= 8'b10011001;
		4'h5: nxt_catodos <= 8'b10010010;
		4'h6: nxt_catodos <= 8'b10000010;
		4'h7: nxt_catodos <= 8'b11111000;
		4'h8: nxt_catodos <= 8'b10000000;
		4'h9: nxt_catodos <= 8'B10010000;
		4'ha: nxt_catodos <= 8'b10001000;
		4'hb: nxt_catodos <= 8'b10000011;
		4'hc: nxt_catodos <= 8'b11000110;
		4'hd: nxt_catodos <= 8'b10100001;
		4'he: nxt_catodos <= 8'b10000110;
		4'hf: nxt_catodos <= 8'b10001110;

		default: nxt_catodos <= 8'b11111111;

	endcase

	nxt_i = (i < 7)? i+1 : 0;
end

endmodule


module clock_divider (
	input clk, resetn,
	output reg clk_div
);

reg [19:0] counter;

parameter limit = 20'b10000000000000000000;

initial counter = 0;
initial clk_div = 0;


always @ (posedge clk) begin
	counter <= counter + 20'b00000000000001000000;
	if (counter == limit)begin
		clk_div = ~clk_div;
		counter = 0;
	end 
end

endmodule