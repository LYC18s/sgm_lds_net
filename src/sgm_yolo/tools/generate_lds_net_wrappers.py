"""Generate interface-only Verilog wrappers; all datapaths remain handwritten.

AMD interface attributes allow module-reference BD integration without packaging
an IP or using HLS. Regeneration does not touch model arithmetic or netlist ROMs.
"""
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'hardware/rtl/lds_net'


def signal(direction,width,name,attribute=''):
    bits=f'[{width-1}:0] ' if width>1 else ''
    return f'    {attribute}{direction} wire {bits}{name}'


def clock(name,reset,buses,hz):
    return [signal('input',1,name,f'(* X_INTERFACE_INFO="xilinx.com:signal:clock:1.0 {name} CLK", X_INTERFACE_PARAMETER="XIL_INTERFACENAME {name}, ASSOCIATED_BUSIF {buses}, ASSOCIATED_RESET {reset}, FREQ_HZ {hz}" *) '),
            signal('input',1,reset,f'(* X_INTERFACE_INFO="xilinx.com:signal:reset:1.0 {reset} RST", X_INTERFACE_PARAMETER="XIL_INTERFACENAME {reset}, POLARITY ACTIVE_LOW" *) ')]


AXIL=[('input',12,'awaddr'),('input',3,'awprot'),('input',1,'awvalid'),('output',1,'awready'),
      ('input',32,'wdata'),('input',4,'wstrb'),('input',1,'wvalid'),('output',1,'wready'),
      ('output',2,'bresp'),('output',1,'bvalid'),('input',1,'bready'),
      ('input',12,'araddr'),('input',3,'arprot'),('input',1,'arvalid'),('output',1,'arready'),
      ('output',32,'rdata'),('output',2,'rresp'),('output',1,'rvalid'),('input',1,'rready')]
READ=[('output',32,'araddr'),('output',8,'arlen'),('output',3,'arsize'),('output',2,'arburst'),('output',1,'arlock'),('output',4,'arcache'),('output',3,'arprot'),('output',4,'arqos'),('output',1,'arvalid'),('input',1,'arready'),
      ('input',64,'rdata'),('input',2,'rresp'),('input',1,'rlast'),('input',1,'rvalid'),('output',1,'rready')]
WRITE=[('output',32,'awaddr'),('output',8,'awlen'),('output',3,'awsize'),('output',2,'awburst'),('output',1,'awlock'),('output',4,'awcache'),('output',3,'awprot'),('output',4,'awqos'),('output',1,'awvalid'),('input',1,'awready'),
       ('output',64,'wdata'),('output',8,'wstrb'),('output',1,'wlast'),('output',1,'wvalid'),('input',1,'wready'),('input',2,'bresp'),('input',1,'bvalid'),('output',1,'bready')]
AXIS=[('input',24,'tdata'),('input',3,'tkeep'),('input',1,'tuser'),('input',1,'tlast'),('input',1,'tvalid'),('output',1,'tready')]


def interface(bus,prefix,fields,kind,parameters=''):
    result=[]
    for index,(direction,width,suffix) in enumerate(fields):
        attr=f'(* X_INTERFACE_INFO="xilinx.com:interface:{kind}:1.0 {bus} {suffix.upper()}"'
        if index==0 and parameters:attr+=f', X_INTERFACE_PARAMETER="XIL_INTERFACENAME {bus}, {parameters}"'
        result.append(signal(direction,width,prefix+suffix,attr+' *) '))
    return result


def axil():
    return interface('S_AXI','s_axi_',AXIL,'aximm','PROTOCOL AXI4LITE, DATA_WIDTH 32, ADDR_WIDTH 12')


def master(bus,prefix,mode):
    fields=(READ if mode!='WRITE_ONLY' else [])+(WRITE if mode!='READ_ONLY' else [])
    params=f'PROTOCOL AXI4, DATA_WIDTH 64, ADDR_WIDTH 32, ID_WIDTH 0, READ_WRITE_MODE {mode}, HAS_BURST 1, SUPPORTS_NARROW_BURST 0, MAX_BURST_LENGTH 16, NUM_READ_OUTSTANDING 4, NUM_WRITE_OUTSTANDING 4'
    return interface(bus,prefix,fields,'aximm',params)


def stream(bus,prefix,output=False):
    fields=[(('output' if d=='input' else 'input') if output else d,w,n) for d,w,n in AXIS]
    return interface(bus,prefix,fields,'axis','TDATA_NUM_BYTES 3, HAS_TKEEP 1, HAS_TLAST 1, TUSER_WIDTH 1')


def connections(fields,prefix,renames=None):
    return [f'.{name}({prefix+suffix})' for name,suffix in (renames or [(s,s) for _,_,s in fields])]


def transport():
    pairs=[f'.s_axi_{s}(s_axi_{s})' for _,_,s in AXIL]
    pairs+=['.clk(aclk)', '.rst_n(aresetn)', '.write_enable(we)', '.write_address(wa)', '.write_data(wd)', '.write_strobe(ws)', '.write_reject(reject)', '.read_address(ra)', '.read_data(rd)']
    return 'wire we,reject;wire [11:0] wa,ra;wire [31:0] wd,rd;wire [3:0] ws;\nlds_net_axil control_bus(\n    '+',\n    '.join(pairs)+');\n'


def constants(prefix,mode):
    result=[]
    for ch in (['ar'] if mode=='READ_ONLY' else ['aw'] if mode=='WRITE_ONLY' else ['ar','aw']):
        for suffix,value in [('size',"3'd3"),('burst',"2'b01"),('lock',"1'b0"),('cache',"4'b0011"),('prot',"3'd0"),('qos',"4'd0")]:
            result.append(f'assign {prefix}{ch}{suffix}={value};')
    return '\n'.join(result)+'\n'


PUBLISH=[('input',1,'authorize_valid'),('output',1,'authorize_ready'),('input',1,'authorize_slot'),('input',32,'authorize_frame'),('input',32,'authorize_base'),('output',1,'authorize_accept'),
         ('input',1,'display_event_valid'),('output',1,'display_event_ready'),('input',1,'display_event_clear'),('input',1,'display_event_slot'),('input',32,'display_event_frame')]


def perception():
    ports=clock('aclk','aresetn','S_AXI:M_NET:M_CAPTURE:M_MASK:S_VIDEO',100000000)+axil()+stream('S_VIDEO','s_video_')
    ports+=[signal(d,w,n) for d,w,n in PUBLISH+[('input',1,'branch_cancel'),('input',32,'raw_drops'),('output',1,'tap_enable')]]
    ports+=master('M_NET','m_net_','READ_WRITE')+master('M_CAPTURE','m_capture_','WRITE_ONLY')+master('M_MASK','m_mask_','READ_WRITE')
    body=transport()
    pairs=['.clk(aclk)','.rst_n(aresetn)','.write_enable(we)','.write_address(wa)','.write_data(wd)','.write_strobe(ws)','.write_reject(reject)','.read_address(ra)','.read_data(rd)',
           '.s_data(s_video_tdata)','.s_sof(s_video_tuser)','.s_eol(s_video_tlast)','.s_valid(s_video_tvalid)','.s_ready(s_video_tready)',
           ".branch_cancel(branch_cancel || (s_video_tvalid && s_video_tkeep!=3'b111))",'.raw_drops(raw_drops)','.tap_enable(tap_enable)']
    pairs += [f'.{n}({n})' for _,_,n in PUBLISH]
    for core,prefix,mode in [('net','m_net_','READ_WRITE'),('cap','m_capture_','WRITE_ONLY'),('mask','m_mask_','READ_WRITE')]:
        fields=(READ if mode!='WRITE_ONLY' else [])+WRITE
        for _,_,s in fields:
            if not any(s==ch+tail for ch in ['ar','aw'] for tail in ['size','burst','lock','cache','prot','qos']):pairs.append(f'.{core}_{s}({prefix+s})')
        body+=constants(prefix,mode)
    body+='lds_net_perception_core core(\n    '+',\n    '.join(pairs)+');\n'
    save('lds_net_perception',ports,body)


def display():
    ports='''    (* X_INTERFACE_INFO="xilinx.com:signal:clock:1.0 aclk CLK", X_INTERFACE_PARAMETER="XIL_INTERFACENAME aclk, ASSOCIATED_BUSIF S_AXI:S_AXIS:M_AXIS, ASSOCIATED_RESET aresetn, FREQ_HZ 142857132" *) input wire aclk
    (* X_INTERFACE_INFO="xilinx.com:signal:reset:1.0 aresetn RST", X_INTERFACE_PARAMETER="XIL_INTERFACENAME aresetn, POLARITY ACTIVE_LOW" *) input wire aresetn
    (* X_INTERFACE_INFO="xilinx.com:signal:clock:1.0 nn_clk CLK", X_INTERFACE_PARAMETER="XIL_INTERFACENAME nn_clk, ASSOCIATED_BUSIF M_AXI:M_UI_AXI, ASSOCIATED_RESET nn_rst_n, FREQ_HZ 100000000" *) input wire nn_clk
    (* X_INTERFACE_INFO="xilinx.com:signal:reset:1.0 nn_rst_n RST", X_INTERFACE_PARAMETER="XIL_INTERFACENAME nn_rst_n, POLARITY ACTIVE_LOW" *) input wire nn_rst_n
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 S_AXI AWADDR", X_INTERFACE_PARAMETER="XIL_INTERFACENAME S_AXI, PROTOCOL AXI4LITE, DATA_WIDTH 32, ADDR_WIDTH 12" *) input wire [11:0] s_axi_awaddr
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 S_AXI AWPROT" *) input wire [2:0] s_axi_awprot
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 S_AXI AWVALID" *) input wire s_axi_awvalid
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 S_AXI AWREADY" *) output wire s_axi_awready
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 S_AXI WDATA" *) input wire [31:0] s_axi_wdata
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 S_AXI WSTRB" *) input wire [3:0] s_axi_wstrb
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 S_AXI WVALID" *) input wire s_axi_wvalid
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 S_AXI WREADY" *) output wire s_axi_wready
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 S_AXI BRESP" *) output wire [1:0] s_axi_bresp
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 S_AXI BVALID" *) output wire s_axi_bvalid
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 S_AXI BREADY" *) input wire s_axi_bready
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 S_AXI ARADDR" *) input wire [11:0] s_axi_araddr
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 S_AXI ARPROT" *) input wire [2:0] s_axi_arprot
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 S_AXI ARVALID" *) input wire s_axi_arvalid
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 S_AXI ARREADY" *) output wire s_axi_arready
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 S_AXI RDATA" *) output wire [31:0] s_axi_rdata
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 S_AXI RRESP" *) output wire [1:0] s_axi_rresp
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 S_AXI RVALID" *) output wire s_axi_rvalid
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 S_AXI RREADY" *) input wire s_axi_rready
    (* X_INTERFACE_INFO="xilinx.com:interface:axis:1.0 S_AXIS TDATA", X_INTERFACE_PARAMETER="XIL_INTERFACENAME S_AXIS, TDATA_NUM_BYTES 3, HAS_TKEEP 1, HAS_TLAST 1, TUSER_WIDTH 1" *) input wire [23:0] s_axis_tdata
    (* X_INTERFACE_INFO="xilinx.com:interface:axis:1.0 S_AXIS TKEEP" *) input wire [2:0] s_axis_tkeep
    (* X_INTERFACE_INFO="xilinx.com:interface:axis:1.0 S_AXIS TUSER" *) input wire s_axis_tuser
    (* X_INTERFACE_INFO="xilinx.com:interface:axis:1.0 S_AXIS TLAST" *) input wire s_axis_tlast
    (* X_INTERFACE_INFO="xilinx.com:interface:axis:1.0 S_AXIS TVALID" *) input wire s_axis_tvalid
    (* X_INTERFACE_INFO="xilinx.com:interface:axis:1.0 S_AXIS TREADY" *) output wire s_axis_tready
    (* X_INTERFACE_INFO="xilinx.com:interface:axis:1.0 M_AXIS TDATA", X_INTERFACE_PARAMETER="XIL_INTERFACENAME M_AXIS, TDATA_NUM_BYTES 3, HAS_TKEEP 1, HAS_TLAST 1, TUSER_WIDTH 1" *) output wire [23:0] m_axis_tdata
    (* X_INTERFACE_INFO="xilinx.com:interface:axis:1.0 M_AXIS TKEEP" *) output wire [2:0] m_axis_tkeep
    (* X_INTERFACE_INFO="xilinx.com:interface:axis:1.0 M_AXIS TUSER" *) output wire m_axis_tuser
    (* X_INTERFACE_INFO="xilinx.com:interface:axis:1.0 M_AXIS TLAST" *) output wire m_axis_tlast
    (* X_INTERFACE_INFO="xilinx.com:interface:axis:1.0 M_AXIS TVALID" *) output wire m_axis_tvalid
    (* X_INTERFACE_INFO="xilinx.com:interface:axis:1.0 M_AXIS TREADY" *) input wire m_axis_tready
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 M_AXI ARADDR", X_INTERFACE_PARAMETER="XIL_INTERFACENAME M_AXI, PROTOCOL AXI4, DATA_WIDTH 64, ADDR_WIDTH 32, ID_WIDTH 0, READ_WRITE_MODE READ_ONLY, HAS_BURST 1, SUPPORTS_NARROW_BURST 0, MAX_BURST_LENGTH 16, NUM_READ_OUTSTANDING 4, NUM_WRITE_OUTSTANDING 4" *) output wire [31:0] m_axi_araddr
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 M_AXI ARLEN" *) output wire [7:0] m_axi_arlen
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 M_AXI ARSIZE" *) output wire [2:0] m_axi_arsize
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 M_AXI ARBURST" *) output wire [1:0] m_axi_arburst
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 M_AXI ARLOCK" *) output wire m_axi_arlock
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 M_AXI ARCACHE" *) output wire [3:0] m_axi_arcache
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 M_AXI ARPROT" *) output wire [2:0] m_axi_arprot
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 M_AXI ARQOS" *) output wire [3:0] m_axi_arqos
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 M_AXI ARVALID" *) output wire m_axi_arvalid
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 M_AXI ARREADY" *) input wire m_axi_arready
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 M_AXI RDATA" *) input wire [63:0] m_axi_rdata
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 M_AXI RRESP" *) input wire [1:0] m_axi_rresp
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 M_AXI RLAST" *) input wire m_axi_rlast
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 M_AXI RVALID" *) input wire m_axi_rvalid
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 M_AXI RREADY" *) output wire m_axi_rready
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 M_UI_AXI ARADDR", X_INTERFACE_PARAMETER="XIL_INTERFACENAME M_UI_AXI, PROTOCOL AXI4, DATA_WIDTH 64, ADDR_WIDTH 32, ID_WIDTH 0, READ_WRITE_MODE READ_ONLY, HAS_BURST 1, SUPPORTS_NARROW_BURST 0, MAX_BURST_LENGTH 16, NUM_READ_OUTSTANDING 4, NUM_WRITE_OUTSTANDING 4" *) output wire [31:0] m_ui_axi_araddr
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 M_UI_AXI ARLEN" *) output wire [7:0] m_ui_axi_arlen
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 M_UI_AXI ARSIZE" *) output wire [2:0] m_ui_axi_arsize
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 M_UI_AXI ARBURST" *) output wire [1:0] m_ui_axi_arburst
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 M_UI_AXI ARLOCK" *) output wire m_ui_axi_arlock
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 M_UI_AXI ARCACHE" *) output wire [3:0] m_ui_axi_arcache
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 M_UI_AXI ARPROT" *) output wire [2:0] m_ui_axi_arprot
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 M_UI_AXI ARQOS" *) output wire [3:0] m_ui_axi_arqos
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 M_UI_AXI ARVALID" *) output wire m_ui_axi_arvalid
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 M_UI_AXI ARREADY" *) input wire m_ui_axi_arready
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 M_UI_AXI RDATA" *) input wire [63:0] m_ui_axi_rdata
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 M_UI_AXI RRESP" *) input wire [1:0] m_ui_axi_rresp
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 M_UI_AXI RLAST" *) input wire m_ui_axi_rlast
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 M_UI_AXI RVALID" *) input wire m_ui_axi_rvalid
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 M_UI_AXI RREADY" *) output wire m_ui_axi_rready
    output wire authorize_valid
    input wire authorize_ready
    output wire authorize_slot
    output wire [31:0] authorize_frame
    output wire [31:0] authorize_base
    input wire authorize_accept
    output wire display_event_valid
    input wire display_event_ready
    output wire display_event_clear
    output wire display_event_slot
    output wire [31:0] display_event_frame'''.splitlines()
    body='''wire we,reject;wire [11:0] wa,ra;wire [31:0] wd,rd;wire [3:0] ws;
wire ui_we,ui_re,ui_ready;wire [11:0] ui_address;wire [31:0] ui_wd;wire [3:0] ui_ws;
lds_net_ui_axil control_bus(
    .s_axi_awaddr(s_axi_awaddr),
    .s_axi_awprot(s_axi_awprot),
    .s_axi_awvalid(s_axi_awvalid),
    .s_axi_awready(s_axi_awready),
    .s_axi_wdata(s_axi_wdata),
    .s_axi_wstrb(s_axi_wstrb),
    .s_axi_wvalid(s_axi_wvalid),
    .s_axi_wready(s_axi_wready),
    .s_axi_bresp(s_axi_bresp),
    .s_axi_bvalid(s_axi_bvalid),
    .s_axi_bready(s_axi_bready),
    .s_axi_araddr(s_axi_araddr),
    .s_axi_arprot(s_axi_arprot),
    .s_axi_arvalid(s_axi_arvalid),
    .s_axi_arready(s_axi_arready),
    .s_axi_rdata(s_axi_rdata),
    .s_axi_rresp(s_axi_rresp),
    .s_axi_rvalid(s_axi_rvalid),
    .s_axi_rready(s_axi_rready),
    .clk(aclk),
    .rst_n(aresetn),
    .memory_clk(nn_clk),.memory_rst_n(nn_rst_n),
    .ui_write_enable(ui_we),.ui_read_enable(ui_re),.ui_address(ui_address),
    .ui_write_data(ui_wd),.ui_write_strobe(ui_ws),.ui_write_reject(ui_reject),.ui_read_data(ui_rd),.ui_ready(ui_ready),
    .write_enable(we),
    .write_address(wa),
    .write_data(wd),
    .write_strobe(ws),
    .write_reject(reject),
    .read_address(ra),
    .read_data(rd));
assign m_axi_arsize=3'd3;
assign m_axi_arburst=2'b01;
assign m_axi_arlock=1'b0;
assign m_axi_arcache=4'b0011;
assign m_axi_arprot=3'd0;
assign m_axi_arqos=4'd0;
wire box_write,count_write,shadow_locked,submit_valid,submit_ready,submit_slot,clear;
wire [4:0] box_address,box_count;wire [31:0] box_data,submit_frame,submit_base;wire [3:0] box_strobe;
wire committed,clear_committed,committed_slot,active_valid,active_slot,pending,pending_error;
wire [31:0] committed_frame,active_frame,cache_failures,read_errors,mask_misses;
wire [23:0] overlay_data;wire [2:0] overlay_keep;
wire overlay_sof,overlay_eol,overlay_valid,overlay_ready;
wire [31:0] legacy_rd,ui_rd;wire legacy_reject,ui_reject;
assign reject=(wa>=12'h200 && wa<12'h264)?we:legacy_reject;
assign rd=(ra==12'h030) ? 32'h55494631 : (ra==12'h034) ? 32'h434f4c31 : (ra==12'h038) ? 32'h48554431 : legacy_rd;
assign m_ui_axi_arsize=3'd3;assign m_ui_axi_arburst=2'b01;assign m_ui_axi_arlock=1'b0;
assign m_ui_axi_arcache=4'b0011;assign m_ui_axi_arprot=3'd0;assign m_ui_axi_arqos=4'd0;
lds_net_display_control registers(
    .video_clk(aclk),
    .video_rst_n(aresetn),
    .nn_clk(nn_clk),
    .nn_rst_n(nn_rst_n),
    .box_write(box_write),
    .box_address(box_address),
    .box_data(box_data),
    .box_strobe(box_strobe),
    .count_write(count_write),
    .box_count(box_count),
    .shadow_locked(shadow_locked),
    .submit_valid(submit_valid),
    .submit_ready(submit_ready),
    .submit_slot(submit_slot),
    .submit_frame(submit_frame),
    .submit_base(submit_base),
    .clear(clear),
    .committed(committed),
    .clear_committed(clear_committed),
    .committed_slot(committed_slot),
    .committed_frame(committed_frame),
    .active_valid(active_valid),
    .active_slot(active_slot),
    .active_frame(active_frame),
    .pending_error(pending_error),
    .cache_failures(cache_failures),
    .read_errors(read_errors),
    .mask_misses(mask_misses),
    .write_enable(we && !(wa>=12'h200 && wa<12'h264)),
    .write_address(wa),
    .write_data(wd),
    .write_strobe(ws),
    .write_reject(legacy_reject),
    .read_address(ra),
    .read_data(legacy_rd),
    .authorize_valid(authorize_valid),
    .authorize_ready(authorize_ready),
    .authorize_slot(authorize_slot),
    .authorize_frame(authorize_frame),
    .authorize_base(authorize_base),
    .authorize_accept(authorize_accept),
    .event_valid(display_event_valid),
    .event_ready(display_event_ready),
    .event_clear(display_event_clear),
    .event_slot(display_event_slot),
    .event_frame(display_event_frame));
lds_net_display_core #(.PANEL_Y(300)) core(
    .video_clk(aclk),
    .video_rst_n(aresetn),
    .nn_clk(nn_clk),
    .nn_rst_n(nn_rst_n),
    .box_write(box_write),
    .box_address(box_address),
    .box_data(box_data),
    .box_strobe(box_strobe),
    .count_write(count_write),
    .box_count(box_count),
    .shadow_locked(shadow_locked),
    .submit_valid(submit_valid),
    .submit_ready(submit_ready),
    .submit_slot(submit_slot),
    .submit_frame(submit_frame),
    .submit_base(submit_base),
    .clear(clear),
    .committed(committed),
    .clear_committed(clear_committed),
    .committed_slot(committed_slot),
    .committed_frame(committed_frame),
    .active_valid(active_valid),
    .active_slot(active_slot),
    .active_frame(active_frame),
    .pending_error(pending_error),
    .cache_failures(cache_failures),
    .read_errors(read_errors),
    .mask_misses(mask_misses),
    .pending(pending),
    .s_data(s_axis_tdata),
    .s_keep(s_axis_tkeep),
    .s_sof(s_axis_tuser),
    .s_eol(s_axis_tlast),
    .s_valid(s_axis_tvalid),
    .s_ready(s_axis_tready),
    .m_data(overlay_data),
    .m_keep(overlay_keep),
    .m_sof(overlay_sof),
    .m_eol(overlay_eol),
    .m_valid(overlay_valid),
    .m_ready(overlay_ready),
    .araddr(m_axi_araddr),
    .arlen(m_axi_arlen),
    .arvalid(m_axi_arvalid),
    .arready(m_axi_arready),
    .rdata(m_axi_rdata),
    .rresp(m_axi_rresp),
    .rlast(m_axi_rlast),
    .rvalid(m_axi_rvalid),
    .rready(m_axi_rready));
wire [23:0] ui_pixels;wire [2:0] ui_keep;wire ui_sof,ui_eol,ui_valid,ui_output_ready;
lds_net_ui ui(.clk(aclk),.rst_n(aresetn),.memory_clk(nn_clk),.memory_rst_n(nn_rst_n),
    .write_enable(ui_we),.read_enable(ui_re),.operation_ready(ui_ready),.write_address(ui_address),.read_address(ui_address),.write_data(ui_wd),.write_strobe(ui_ws),
    .write_reject(ui_reject),.read_data(ui_rd),
    .s_data(overlay_data),.s_keep(overlay_keep),.s_sof(overlay_sof),.s_eol(overlay_eol),.s_valid(overlay_valid),.s_ready(overlay_ready),
    .m_data(ui_pixels),.m_keep(ui_keep),.m_sof(ui_sof),.m_eol(ui_eol),.m_valid(ui_valid),.m_ready(ui_output_ready),
    .araddr(m_ui_axi_araddr),.arlen(m_ui_axi_arlen),.arvalid(m_ui_axi_arvalid),.arready(m_ui_axi_arready),
    .rdata(m_ui_axi_rdata),.rresp(m_ui_axi_rresp),.rlast(m_ui_axi_rlast),.rvalid(m_ui_axi_rvalid),.rready(m_ui_axi_rready));
lds_net_ui_elastic output_queue(.clk(aclk),.rst_n(aresetn),
    .s_data(ui_pixels),.s_keep(ui_keep),.s_sof(ui_sof),.s_eol(ui_eol),.s_valid(ui_valid),.s_ready(ui_output_ready),
    .m_data(m_axis_tdata),.m_keep(m_axis_tkeep),.m_sof(m_axis_tuser),.m_eol(m_axis_tlast),.m_valid(m_axis_tvalid),.m_ready(m_axis_tready));
'''
    save('lds_net_display',ports,body)


def tap():
    ports=clock('aclk','aresetn','S_AXIS:M_AXIS:NN_AXIS',142857132)+clock('nn_clk','nn_rst_n','',100000000)
    ports+=stream('S_AXIS','s_axis_')+stream('M_AXIS','m_axis_',True)+stream('NN_AXIS','nn_axis_',True)
    ports += [signal(d,w,n) for d,w,n in [('input',1,'enable_nn'),('output',1,'branch_reset_n'),('output',1,'branch_cancel_nn'),('output',32,'dropped_frames_nn')]]
    body='''(* ASYNC_REG="TRUE" *) reg enable0,enable1,cancel0,cancel1;
(* ASYNC_REG="TRUE" *) reg [31:0] drops0,drops1;
wire cancel_source;wire [31:0] dropped_source;
reg [31:0] gray_source;
always @(posedge aclk)begin
    if(!aresetn)begin enable0<=0;enable1<=0;gray_source<=0;end
    else begin enable0<=enable_nn;enable1<=enable0;gray_source<=(dropped_source>>1)^dropped_source;end
end
always @(posedge nn_clk)begin
    if(!nn_rst_n)begin cancel0<=1;cancel1<=1;drops0<=0;drops1<=0;end
    else begin cancel0<=cancel_source;cancel1<=cancel0;drops0<=gray_source;drops1<=drops0;end
end
assign branch_cancel_nn=cancel1;
function [31:0] gray_to_binary;
    input [31:0] value;integer i;
    begin gray_to_binary[31]=value[31];for(i=30;i>=0;i=i-1)gray_to_binary[i]=gray_to_binary[i+1]^value[i];end
endfunction
assign dropped_frames_nn=gray_to_binary(drops1);
'''
    pairs=['.clk(aclk)','.rst_n(aresetn)','.enable(enable1)','.branch_reset_n(branch_reset_n)','.branch_cancel(cancel_source)','.dropped_frames(dropped_source)']
    for prefix,core in [('s_axis_','s'),('m_axis_','m'),('nn_axis_','nn')]:
        for short,axis in [('data','tdata'),('keep','tkeep'),('sof','tuser'),('eol','tlast'),('valid','tvalid'),('ready','tready')]:pairs.append(f'.{core}_{short}({prefix+axis})')
    body+='lds_net_camera_tap tap(\n    '+',\n    '.join(pairs)+');\n'
    save('lds_net_video_tap',ports,body)


def save(name,ports,body):
    text='`timescale 1ns/1ps\n// Generated by tools/generate_lds_net_wrappers.py; interface wiring only.\n'
    text+=f'module {name}(\n'+',\n'.join(ports)+'\n);\n'+body+'endmodule\n'
    path=OUT/(name+'.v')
    if not path.exists() or path.read_text(encoding='utf-8')!=text:path.write_text(text,encoding='utf-8')
    print(f'Generated {path.relative_to(ROOT)}')


if __name__=='__main__':
    perception();display();tap()
