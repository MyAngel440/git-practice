module control(
	input wire clk,
	input wire rst_n,
	
	input wire [3:0] crl_order,	//控制器的各种指令，用于操作读写什么
	input wire crl_wr,			//crl_wr为0是写数据，为1是读数据
	input wire crl_sw,			//crl_sw为0时crl_wr不可用，为1时才可用
	
	input wire [31:0] crl_wdata,	//控制器32位
	output reg [31:0] crl_rdata_z,
	
	
	input wire rx_pad,
	output wire tx_pad
);






//crl_order
parameter [3:0]	CRL_OD_RB		= 4'b0001;
parameter [3:0]	CRL_OD_IER		= 4'b0010;
parameter [3:0] CRL_OD_CTR		= 4'b0011;
parameter [3:0]	CRL_OD_STR		= 4'b0100;
parameter [3:0]	CRL_OD_DL		= 4'b0101;
parameter [3:0]	CRL_OD_DBG		= 4'b0110;
parameter [3:0]	CRL_OD_TB		= 4'b0111;
				
//tx&rx state
reg tf_push;		
wire tf_push_n;
wire tf_overrun;
wire tf_full;
wire tf_empty;
wire [4:0] tf_state;
wire [7:0] tf_number;

reg rf_pop;		
wire rf_pop_n;
wire rf_overrun;
wire rf_full;
wire rf_empty;
wire rf_pari_err;
wire [4:0] rf_state;
wire [7:0] rf_number;
wire [7:0] rf_data_out;

//order & wr_en
wire crl_w_en;
wire crl_r_en;

//reg
reg [31:0] dl;
reg [7:0] ier;
reg [7:0] ctr;
reg [7:0] txbuf;
reg [7:0] rxbuf;
reg [31:0] dbg_reg;
reg [31:0] baud_div;
reg function_enable;

wire [7:0] txbuf_n;
wire [7:0] ier_n;
wire [7:0] ctr_n;

wire [31:0] dl_n;
wire [31:0] dbg_reg_rd;

// access
wire dl_access;
wire normal_reg_access;

//rdata wdata

reg [7:0] crl_rdata0;
reg [7:0] crl_rdata1;
reg [7:0] crl_rdata2;
reg [7:0] crl_rdata3;
wire [7:0] crl_wdata0;
wire [7:0] crl_wdata1;
wire [7:0] crl_wdata2;
wire [7:0] crl_wdata3;

//str
wire str0,str1,str2,str3,str4,str5,str6;
reg [7:0]str;
assign str0 = (rf_number != 0);
assign str1 = rf_overrun;
assign str2 = rf_pari_err;
assign str3 = tf_empty;
assign str4 = rf_full;
assign str5 = tf_full;
assign str6 = tf_overrun;

always@(*)
begin
	str = {1'b0, str6, str5, str4, str3, str2, str1, str0};
end

//控制器写数据 使能
assign crl_w_en = crl_wr & crl_sw;								//begin和end之间是串行；assign则是并行，相当于连线！
assign crl_wdata0 = crl_w_en ? crl_wdata[7:0] : {8{1'bz}};
assign crl_wdata1 = crl_w_en ? crl_wdata[15:8] : {8{1'bz}};
assign crl_wdata2 = crl_w_en ? crl_wdata[23:16] : {8{1'bz}};
assign crl_wdata3 = crl_w_en ? crl_wdata[31:24] : {8{1'bz}};

//控制器读数据 使能
wire [31:0] crl_rdata;
assign crl_r_en = (~crl_wr) & crl_sw;
assign crl_rdata = crl_r_en ? {crl_rdata3, crl_rdata2, crl_rdata1, crl_rdata0} : {32{1'b0}};
always@(*)begin
	crl_rdata_z <= crl_rdata;
end



//tx_int
uart_tx uart_tx_int
(
	.clk(clk),
	.rst_n(rst_n),
	.din(txbuf),
	.ctr(ctr),
	.baud_rate(baud_div),
	.tf_push(tf_push),
	.tf_state(tf_state),
	.tf_number(tf_number),
	.tf_overrun(tf_overrun),
	.tf_full(tf_full),
	.tf_empty(tf_empty)
);
//rx_int
uart_rx uart_rx_int
(
	.clk(clk),
	.rst_n(rst_n),
	.ctr(ctr),
	.baud_rate(baud_div),
	.rf_pop(rf_pop),
	.rf_number(rf_number),
	.rf_data_out(rf_data_out),
	.rf_state(rf_state),
	.rf_pari_err(rf_pari_err),
	.rf_overrun(rf_overrun),
	.rf_full(rf_full),
	.rf_empty(rf_empty),
	.enable(function_enable),
	.rx_pad(rx_pad)
);
//dl_access and rx_enable
assign dl_access = ctr[5];
assign normal_reg_access = ~dl_access;
always@(posedge normal_reg_access or negedge rst_n)begin
	if(!rst_n)
		function_enable <= 1'b0;
	else
		function_enable <= 1'b1;
end

//baud_div
always@(*)begin
	if(dl_access)begin
		baud_div[31:16] = dl[31:16];
		baud_div[15:0] = dl[15:0];
	end
end

//tf_push and rf_pop
assign tf_push_n = (crl_w_en & (crl_order == CRL_OD_TB)) & (~tf_full) ? 1'b1 : 1'b0;
always@(posedge clk or negedge rst_n)begin
	if(!rst_n)
		tf_push <= 1'b0;
	else
		tf_push <= tf_push_n;
end

assign rf_pop_n = (crl_r_en & (crl_order == CRL_OD_RB)) & (~rf_empty) ? 1'b1 : 1'b0;
always@(posedge clk or negedge rst_n)begin
	if(!rst_n)
		rf_pop <= 1'b0;
	else
		rf_pop <= rf_pop_n;
end

//↓↓↓从接收机收得数据↓↓↓
always@(*)begin
	rxbuf <= rf_data_out;
end
//↑↑↑从接收机收得数据↑↑↑


//读操作，可以读接收机数据，也可以读寄存器状态
always@(*)begin
	if(crl_r_en)begin
		case(crl_order)
			CRL_OD_RB:begin	//接收数据
				crl_rdata0 <= rxbuf;
				{crl_rdata3, crl_rdata2, crl_rdata1} <= {24{1'b0}};
			end
			
			CRL_OD_IER:begin	//读ier
				crl_rdata0 <= ier;
				{crl_rdata3, crl_rdata2, crl_rdata1} <= {24{1'b0}};
			end
			
			CRL_OD_CTR:begin	//读ctr
				crl_rdata0 <= ctr;
				{crl_rdata3, crl_rdata2, crl_rdata1} <= {24{1'b0}};
			end
			
			CRL_OD_STR:begin	//读str
				crl_rdata0 <= str;
				{crl_rdata3, crl_rdata2, crl_rdata1} <= {24{1'b0}};
			end
			
			CRL_OD_DL:begin	//读dl
				crl_rdata0 <= dl[7:0];
				crl_rdata1 <= dl[15:8];
				crl_rdata2 <= dl[23:16];
				crl_rdata3 <= dl[31:24];
			end
			
			CRL_OD_DBG:begin	//读dbg_reg
				crl_rdata0 <= dbg_reg[7:0];
				crl_rdata1 <= dbg_reg[15:8];
				crl_rdata2 <= dbg_reg[23:16];
				crl_rdata3 <= dbg_reg[31:24];
			end
			
			default:begin	//其他 保留数据
				crl_rdata0 <= crl_rdata0;
				crl_rdata1 <= crl_rdata1;
				crl_rdata2 <= crl_rdata2;
				crl_rdata3 <= crl_rdata3;
			end
		endcase
	end
end

//写操作
assign dl_n = (crl_sw & crl_wr & (crl_order == CRL_OD_DL) & dl_access) ? crl_wdata : dl;
always@(posedge clk or negedge rst_n)begin
	if(!rst_n)
		dl <= 32'b0;
	else
		dl <= dl_n;
end

assign txbuf_n = (crl_sw & crl_wr & (crl_order == CRL_OD_TB)) ? crl_wdata0 : txbuf;
always@(posedge clk or negedge rst_n)begin
	if(!rst_n)
		txbuf <= 8'b0;
	else
		txbuf <= txbuf_n;
end

assign ier_n = (crl_sw & crl_wr & (crl_order == CRL_OD_IER)) ? crl_wdata0 : ier;
always@(posedge clk or negedge rst_n)begin
	if(!rst_n)
		ier <= 3'b0;
	else
		ier <= ier_n;
end

assign ctr_n = (crl_sw & crl_wr & (crl_order == CRL_OD_CTR)) ? crl_wdata0 : ctr;
always@(posedge clk or negedge rst_n)begin
	if(!rst_n)
		ctr <= 8'b0000_0011;
	else
		ctr <= ctr_n;
end

assign dbg_reg_rd = (crl_sw && (crl_wr == 0) && (crl_order == CRL_OD_DBG));
always@(*)begin
	if(dbg_reg_rd)
		dbg_reg = {5'b0, rf_number, tf_number, rf_state, tf_state, ier, str};
	else
		dbg_reg <= 32'b0;
end

//中断

endmodule