 // MiniS08 starter file
module minis08(clk50,rxd,resetin,pbin,clksel,clkdisp,txd,addr,data,stateout,IRout);
input clk50, rxd, resetin, pbin;
input [2:0] clksel;
output txd;
output [20:0] addr;  // three 7-segment displays
output [13:0] data;  // two 7-segment displays
output [13:0] IRout;
output [6:0] stateout;
wire ldIR, ldA, ldHX, ldMARH, ldMARL, clrMARH;
wire ldPC, oeIncPC, oePCH, oePCL;
wire br, jmp, jsr, rts;
wire Imm, Ix, Inh, Stk, Dir, Ext, Rel;
wire st0, st1, st2, st3, st4;
wire N, Z;
wire oeA, oeH, oeX, oeHX, oeMAR;
wire incSP, decSP, oeSP;
wire modA, stoA, modHX;
wire pula, psha, pulx, pshx, pulh, pshh, aix, add, sub, And, ora, eor;
wire lsra, asra, lsla, lda, ldx, sta, stx;
wire bra, bcc, bcs, bpl, bmi, bne, beq, BCT;
wire Read, Write;
wire IOaddr, RAMaddr, ROMaddr, FPUaddr;
wire [7:0] sciout;
wire [7:0] FPUout;
wire [7:0] ROMout, RAMout;
wire [9:0] abus;
wire [7:0] dbus;
wire Reset, Co;
wire [9:0] PCLU, HXLU;
wire [8:0] ALU;
reg [2:0] CPUstate;
reg [27:0] clkdiv;
reg resetout, gotoreset, pbout, pbreg, S08clk, C;
output clkdisp;
reg [7:0] A, SP, IR;
reg [9:0] HX, PC, MAR;

sevenseg A2({2'd0,abus[9:8]},addr[20:14]);
sevenseg A1(abus[7:4],addr[13:7]);
sevenseg A0(abus[3:0],addr[6:0]);
sevenseg D1(dbus[7:4],data[13:7]);
sevenseg D0(dbus[3:0],data[6:0]);
//sevenseg D1(A[7:4],data[13:7]);     //***** debugging
//sevenseg D0(A[3:0],data[6:0]);      //***** debugging

sevenseg IR1(IR[7:4], IRout[13:7]);
sevenseg IR0(IR[3:0], IRout[6:0]);
//sevenseg IR1(SP[7:0], IRout[13:7]);     //***** debugging
//sevenseg IR0(SP[3:0], IRout[6:0]);      //***** debugging
//sevenseg IR1(0, IRout[13:7]);            //***** debugging
//sevenseg IR1({2'b0,HX[9:8]}, IRout[13:7]);//***** debugging
sevenseg ST({1'b0,CPUstate}-1,stateout);

sci S08sci(clk50, dbus, sciout, IOaddr, abus[2:0], Read, Write, rxd, txd);
S08ram S08ram(abus[7:0],clk50,dbus,Write&RAMaddr,RAMout);
S08rom S08rom(abus,clk50,ROMout);
fpu S08fpu(clk50, dbus, FPUout, FPUaddr, abus[1:0], Read, Write);

assign clkdisp = clksel==0 ? pbreg : clksel==1 ? clkdiv[27] 
	   : clksel==2 ? clkdiv[24] : clksel==3 ? clkdiv[21]
           : clksel==4 ? clkdiv[17] : clksel==5 ? clkdiv[15]
	   : clksel==6 ? clkdiv[13] : clkdiv[2];

always @(posedge clk50)
   begin
   clkdiv <= clkdiv+1;
   gotoreset <= resetout;
   resetout <= ~resetin;
   pbreg <= pbout;
   pbout <= ~pbin;
   S08clk <= clkdisp;
   end

always @(posedge S08clk)
   begin
   A <= ldA ? ALU[7:0] : A;
   HX <= ldHX ? HXLU : HX;
   SP <= Reset ? 'hFF : incSP  ? (SP + 1) : decSP ? (SP - 1) : SP;
   PC <= Reset ? 'h100 : ldPC ? PCLU : oeIncPC ? PC+1 : PC;
   IR <= ldIR ? dbus : IR;
   C <= ldA&(add|sub|lsla|lsra|asra) ? Co : C;
   // 2'X0, where X is any base (doesn't matter)
   MAR <= ldMARL&clrMARH ? {2'h0, dbus[7:0]} : ldMARH ? {dbus[1:0], MAR[7:0]}
	: ldMARL ? {MAR[9:8], dbus[7:0]} : clrMARH ? {2'h0, MAR[7:0]} : MAR;

   CPUstate <= gotoreset ? 0 : Reset ? 1 : st0 ? 2
             : st1 ? ((Inh|Imm|Ix|Rel|psha|pshh|pshx) ? 1: 3)
             : st2 ? (((Ext&!jmp)|rts) ? 4 : 1)
             : st3 ? (jsr ? 5 : 1)
				 : st4 ? 1 : st0;
   end

assign Reset = CPUstate==0;
assign st0 = CPUstate==1;
assign st1 = CPUstate==2;
assign st2 = CPUstate==3;
assign st3 = CPUstate==4;
assign st4 = CPUstate==5;

assign N = A[7];
assign Z = A==0;
assign ALU = lda ? dbus : pula ? dbus : add ? ({1'b0, A} + {1'b0, dbus}) 
		: sub ? ({1'b0, A} - {1'b0, dbus}) : And ? (A & dbus) : ora ? A | dbus 
		: eor ? A ^ dbus : lsra ? ({A[0], A >> 1}) : asra ? ({A[0], A >>> 1})
		: lsla ? ({A[7], A << 1}) : 0;

assign Co = ALU[8];

assign HXLU = ldx ? {HX[9:8], dbus} : pulx ? {HX[9:8], dbus} 
	    : pulh ? {dbus[1:0], HX[7:0]} : aix ? (HX + {dbus[7], dbus[7], dbus}) : 0;

assign PCLU =  BCT ? (PC+{dbus[7], dbus[7], dbus}+1)
					: jsr ? MAR
					: {MAR[9:8], dbus};

assign abus = oeHX ? HX : oeSP ? {2'h0, SP[7:0]} : oeMAR ? MAR : oeIncPC ? PC : 0;
assign dbus = oeA ? A : oeX ? HX[7:0] : oeH ? HX[9:8] : oePCL ? PC[7:0] 
				: oePCH ? PC[9:8] : Read&IOaddr ? sciout 
				: Read&ROMaddr ? ROMout : Read&RAMaddr ? RAMout 
				: Read&FPUaddr ? FPUout : 0;

assign ldIR = st0;
assign ldA = modA&Inh&st1 | modA&Stk&st2 | modA&Imm&st1 
		| modA&Dir&st2 | modA&Ext&st3 | modA&Ix&st1;
assign oeA = stoA&Stk&st1 | stoA&Dir&st2 | stoA&Ext&st3 | stoA&Ix&st1;
assign ldHX = modHX&Stk&st2 | modHX&Imm&st1 | modHX&Dir&st2 
		| modHX&Ext&st3 | modHX&Stk&Ix;
assign oeH = pshh&st1;
assign oeX = pshx&st1 | stx&Dir&st2 | stx&Ext&st3 | stx&Ix&st1;
assign oeHX = modA&Ix&st1 | stoA&Ix&st1 | modHX&Ix&st1 | stx&Ix&st1;
assign incSP = modA&Stk&st1 | modHX&Stk&st1 | rts&st1 | rts&st2;
assign decSP = stoA&Stk&st1 | stx&Stk&st1 | pshx&st1 | pshh&st1 |  jsr&st3 | jsr&st4;
assign oeSP = decSP | modA&Stk&st2 | modHX&Stk&st2 | rts&st2 | rts&st3;
assign ldMARH = modA&Ext&st1 | stoA&Ext&st1 | modHX&Ext&st1 
		| stx&Ext&st1 | jmp&st1 | jsr&st1 | rts&st2;
assign ldMARL = modA&Dir&st1 | modA&Ext&st2 | stoA&Dir&st1 
		| stoA&Ext&st2 | modHX&Dir&st1 | modHX&Ext&st2
		| stx&Dir&st1 | stx&Ext&st2 | jsr&st2;
assign clrMARH = modA&Dir&st1 | stoA&Dir&st1 | modHX&Dir&st1 | stx&Dir&st1;

assign oeMAR = modA&Dir&st2 | modA&Ext&st3 | | stoA&Dir&st2 
	| stoA&Ext&st3 | modHX&Dir&st2 | modHX&Ext&st3 | stx&Dir&st2 | stx&Ext&st3;
assign ldPC = BCT&br&st1 | jmp&st2 | jsr&st4 | rts&st3;
assign oeIncPC = st0 | modA&Imm&st1 | modA&Ext&st1 | modA&Ext&st2 | modA&Dir&st1
							 | stoA&Dir&st1 | stoA&Ext&st1 | stoA&Ext&st2 | modHX&Imm&st1
							 | modHX&Dir&st1 | modHX&Ext&st1 | modHX&Ext&st2 | stx&Dir&st1
							 | stx&Ext&st1 | stx&Ext&st2 | br&st1 | jmp&st1
							 | jmp&st2 | jsr&st1 | jsr&st2;
assign oePCH = jsr&st4;
assign oePCL = jsr&st3;
assign Write = stoA&Stk&st1 | stoA&Dir&st2 | stoA&Ext&st3 | stoA&Ix&st1
     | stx&Stk&st1 | pshx&Stk&st1 | stx&Dir&st2 | stx&Ext&st3 | stx&Ix&st1 
     | pshh&Stk&st1| jsr&st3 | jsr&st4;
assign Read = st0 | modA&Stk&st2 | modA&Imm&st1 | modA&Dir&st1 | modA&Dir&st2
     | modA&Ext&st1 | modA&Ext&st2 | modA&Ext&st3 | modA&Ix&st1
     | stoA&Dir&st1 | stoA&Ext&st1 | stoA&Ext&st2 | modHX&Stk&st2
     | modHX&Imm&st1 | modHX&Dir&st1 | modHX&Dir&st2 | modHX&Ext&st1
	 | modHX&Ext&st2 | modHX&Ext&st3 | modHX&Ix&st1  | stx&Dir&st1
     | stx&Ext&st1 | stx&Ext&st2 | br&st1 | jmp&st1 | jmp&st2
	 | jsr&st1 | jsr&st2 | rts&st2 | rts&st3;
assign IOaddr = (abus < 'h8)&(!FPUaddr);
assign RAMaddr = !IOaddr & !ROMaddr & !FPUaddr;
assign ROMaddr = (abus > 'hFF);
assign FPUaddr = (abus < 'h4);
assign modA = lda | pula | add | sub | And | ora | eor | lsra | asra | lsla;
assign stoA = sta | psha;
assign modHX = ldx | pulx | pulh | aix;
assign br = IR[7:4] == 'h2;
assign jmp = IR=='hCC;
assign jsr = IR=='hCD;
assign rts = IR=='h81;
assign add = (IR == 'hAB) | (IR == 'hBB) | (IR == 'hCB) | (IR == 'hFB); 
assign sub = (IR == 'hA0) | (IR == 'hB0) | (IR == 'hC0) | (IR == 'hF0); 
assign And = (IR == 'hA4) | (IR == 'hB4) | (IR == 'hC4) | (IR == 'hF4); 
assign ora = (IR == 'hAA) | (IR == 'hBA) | (IR == 'hCA) | (IR == 'hFA); 
assign eor = (IR == 'hA8) | (IR == 'hB8) | (IR == 'hC8) | (IR == 'hF8); 
assign lsra = (IR == 'h44);
assign asra = (IR == 'h47);
assign lsla = (IR == 'h48);
assign Imm = IR[7:4] == 'hA;
assign Ix = IR[7:4] == 'hF;
assign Inh = IR[7:4] == 'h4;
assign Rel = bra | bcc | bcs | bpl | bmi | bne | beq;
assign Dir = IR[7:4] == 'hB;
assign Ext = IR[7:4] == 'hC;
assign Stk = IR[7:4] == 'h8;
assign psha = IR=='h87;
assign pshh = IR=='h8B;
assign pshx = IR=='h89;
assign lda = (IR == 'hA6) | (IR == 'hB6) | (IR == 'hC6) | (IR == 'hF6);
assign ldx = (IR == 'hAE) | (IR == 'hBE) | (IR == 'hCE) | (IR == 'hFE);
assign pula = IR=='h86;
assign pulx = IR=='h88;
assign pulh = IR=='h8A;
assign aix = IR=='hAF;
assign BCT = bra | bcc&!C | bcs&C | bpl&!N | bmi&N | bne&!Z | beq&Z;
assign bra = IR=='h20;
assign bcc = IR=='h24;
assign bcs = IR=='h25;
assign bpl = IR=='h2A;
assign bmi = IR=='h2B;
assign bne = IR=='h26;
assign beq = IR=='h27;
assign sta = (IR == 'hB7) | (IR == 'hC7) | (IR == 'hF7);
assign stx = (IR == 'hBF) | (IR == 'hCF) | (IR == 'hFF);

endmodule

module sevenseg(hex, segs);
input [3:0] hex;
output [6:0] segs;
assign segs = hex==0  ? 'b0000001   // active-low segments
            : hex==1  ? 'b1001111
            : hex==2  ? 'b0010010
            : hex==3  ? 'b0000110
            : hex==4  ? 'b1001100
            : hex==5  ? 'b0100100
            : hex==6  ? 'b0100000
            : hex==7  ? 'b0001111
            : hex==8  ? 'b0000000
            : hex==9  ? 'b0001100
            : hex==10 ? 'b0001000
            : hex==11 ? 'b1100000
            : hex==12 ? 'b0110001
            : hex==13 ? 'b1000010
            : hex==14 ? 'b0110000
            :           'b0111000;
endmodule
