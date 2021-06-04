// Top module of your design, you cannot modify this module!!
`include "./core/pipeline_risc.v"
`include "./core/i_cache.v"
`include "./core/d_cache.v"
`include "./core/l2_cache.v"

module CHIP (	clk,
				rst_n,
//----------for slow_memD------------
				mem_read_D,
				mem_write_D,
				mem_addr_D,
				mem_wdata_D,
				mem_rdata_D,
				mem_ready_D,
//----------for slow_memI------------
				mem_read_I,
				mem_write_I,
				mem_addr_I,
				mem_wdata_I,
				mem_rdata_I,
				mem_ready_I,
//----------for TestBed--------------				
				DCACHE_addr, 
				DCACHE_wdata,
				DCACHE_wen   
			);
input			clk, rst_n;
//--------------------------

output			mem_read_D;
output			mem_write_D;
output	[31:4]	mem_addr_D;
output	[127:0]	mem_wdata_D;
input	[127:0]	mem_rdata_D;
input			mem_ready_D;
//--------------------------
output			mem_read_I;
output			mem_write_I;
output	[31:4]	mem_addr_I;
output	[127:0]	mem_wdata_I;
input	[127:0]	mem_rdata_I;
input			mem_ready_I;
//----------for TestBed--------------
output	[29:0]	DCACHE_addr;
output	[31:0]	DCACHE_wdata;
output			DCACHE_wen;
//--------------------------

// wire declaration
wire        ICACHE_ren;
wire        ICACHE_wen;
wire [29:0] ICACHE_addr;
wire [31:0] ICACHE_wdata;
wire        ICACHE_stall;
wire [31:0] ICACHE_rdata;

wire        DCACHE_ren;
wire        DCACHE_wen;
wire [29:0] DCACHE_addr;
wire [31:0] DCACHE_wdata;
wire        DCACHE_stall;
wire [31:0] DCACHE_rdata;

wire        l1i_read;
wire        l1i_write;
wire [31:4] l1i_addr;
wire [127:0]l1i_wdata;
wire [127:0]l1i_rdata;
wire        l1i_ready;

wire        l1d_read;
wire        l1d_write;
wire [31:4] l1d_addr;
wire [127:0]l1d_wdata;
wire [127:0]l1d_rdata;
wire        l1d_ready;


//=========================================
	// Note that the overall design of your RISCV includes:
	// 1. pipelined RISCV processor
	// 2. data cache
	// 3. instruction cache


	RISCV_Pipeline i_RISCV(
		// control interface
		.clk            (clk)           , 
		.rst_n          (rst_n)         ,
//----------I cache interface-------		
		.ICACHE_ren     (ICACHE_ren)    ,
		.ICACHE_wen     (ICACHE_wen)    ,
		.ICACHE_addr    (ICACHE_addr)   ,
		.ICACHE_wdata   (ICACHE_wdata)  ,
		.ICACHE_stall   (ICACHE_stall)  ,
		.ICACHE_rdata   (ICACHE_rdata)  ,
//----------D cache interface-------
		.DCACHE_ren     (DCACHE_ren)    ,
		.DCACHE_wen     (DCACHE_wen)    ,
		.DCACHE_addr    (DCACHE_addr)   ,
		.DCACHE_wdata   (DCACHE_wdata)  ,
		.DCACHE_stall   (DCACHE_stall)  ,
		.DCACHE_rdata   (DCACHE_rdata)
	);
	

	d_cache D_cache(
        .clk        (clk)         ,
        .proc_reset (~rst_n)      ,
        .proc_read  (DCACHE_ren)  ,
        .proc_write (DCACHE_wen)  ,
        .proc_addr  (DCACHE_addr) ,
        .proc_rdata (DCACHE_rdata),
        .proc_wdata (DCACHE_wdata),
        .proc_stall (DCACHE_stall),
        .mem_read   (l1d_read )  ,
        .mem_write  (l1d_write) ,
        .mem_addr   (l1d_addr )  ,
        .mem_wdata  (l1d_wdata) ,
        .mem_rdata  (l1d_rdata) ,
        .mem_ready  (l1d_ready)
	);

	i_cache I_cache(
        .clk        (clk)         ,
        .proc_reset (~rst_n)      ,
        .proc_read  (ICACHE_ren)  ,
        .proc_write (ICACHE_wen)  ,
        .proc_addr  (ICACHE_addr) ,
        .proc_rdata (ICACHE_rdata),
        .proc_wdata (ICACHE_wdata),
        .proc_stall (ICACHE_stall),
        .mem_read   (l1i_read )  ,
        .mem_write  (l1i_write) ,
        .mem_addr   (l1i_addr )  ,
        .mem_wdata  (l1i_wdata) ,
        .mem_rdata  (l1i_rdata) ,
        .mem_ready  (l1i_ready)
	);

    l2_cache L2(
        .clk        (clk),
        .proc_reset (~rst_n),
        // between l1i cache
        .l1i_read   (l1i_read),
        .l1i_write  (l1i_write),
        .l1i_addr   (l1i_addr),
        .l1i_wdata  (l1i_wdata),
        .l1i_rdata  (l1i_rdata),
        .l1i_ready  (l1i_ready),
        // between l1d cache
        .l1d_read   (l1d_read),
        .l1d_write  (l1d_write),
        .l1d_addr   (l1d_addr),
        .l1d_wdata  (l1d_wdata),
        .l1d_rdata  (l1d_rdata),
        .l1d_ready  (l1d_ready),
        // between slow memory i
        .memi_read  (mem_read_I),
        .memi_addr  (mem_addr_I),
        .memi_write (mem_write_I),
        .memi_wdata (mem_wdata_I),
        .memi_rdata (mem_rdata_I),
        .memi_ready (mem_ready_I),
        // between slow memory d
        .memd_read  (mem_read_D),
        .memd_addr  (mem_addr_D),
        .memd_write (mem_write_D),
        .memd_wdata (mem_wdata_D),
        .memd_rdata (mem_rdata_D),
        .memd_ready (mem_ready_D)
    );
endmodule
