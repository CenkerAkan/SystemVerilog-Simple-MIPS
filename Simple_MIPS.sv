`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 02/09/2023 10:47:27 AM
// Design Name: 
// Module Name: Simple_MIPS
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module Simple_MIPS();

endmodule

module denemeProje(
input logic instruction,
input logic [11:0]instr,
input logic clk,
output logic [15:0]displayed_number,
output logic [6:0] LED_out,
output logic [3:0]Anode_Activate);
logic newclk;
debounce d1(instruction,clk,newclk);
regFDataMemConnection rfdmc(newclk, instr,displayed_number);
reg [19:0] refresh_counter; 
wire [1:0] LED_activating_counter; 
logic [4:0]LED_BCD;
always @(posedge clk)
begin 
  refresh_counter <= refresh_counter + 1;
end 
assign LED_activating_counter = refresh_counter[19:18];

    always @(*)
    begin
        case(LED_activating_counter)
        2'b00: begin
            Anode_Activate = 4'b0111; 
            LED_BCD = displayed_number[15:11];
             end
        2'b01: begin
            Anode_Activate = 4'b1011; 
            LED_BCD = displayed_number[10:8];
                end
        2'b10: begin
            Anode_Activate = 4'b1101; 
            LED_BCD = displayed_number[7:4];
              end
        2'b11: begin
            Anode_Activate = 4'b1110;
             LED_BCD = displayed_number[3:0];
               end   
        default:begin
             Anode_Activate = 4'b0111; 
            LED_BCD = displayed_number[15:11];
            end
        endcase
    end
    always @(*)
    begin
     case(LED_BCD)
     4'b0000: LED_out = 7'b0000001; // "0"  
     4'b0001: LED_out = 7'b1001111; // "1" 
     4'b0010: LED_out = 7'b0010010; // "2" 
     4'b0011: LED_out = 7'b0000110; // "3" 
     4'b0100: LED_out = 7'b1001100; // "4" 
     4'b0101: LED_out = 7'b0100100; // "5" 
     4'b0110: LED_out = 7'b0100000; // "6" 
     4'b0111: LED_out = 7'b0001111; // "7" 
     4'b1000: LED_out = 7'b0000000; // "8"  
     4'b1001: LED_out = 7'b0000100; // "9" 
     default: LED_out = 7'b0000001; // "0"
     endcase
    end
endmodule

module regfile(input logic clk,
input logic we3,
input logic [2:0] ra1, ra2, wa3,
input logic [3:0] wd3,
input logic sortPermission,
input logic [3:0] sortedarr[7:0],
output logic [3:0] rd1, rd2,
output logic [3:0] rf[7:0],
output logic [15:0] firstfour
);
initial begin
rf[7]=4'b0111;
rf[6]=4'b0110;
rf[5]=4'b0101;
rf[4]=4'b0100;
rf[3]=4'b0011;
rf[2]=4'b0010;
rf[1]=4'b0001;
rf[0]=4'b0000;
end
always_ff @(posedge clk)begin
if (we3)begin
rf[wa3] <= wd3;
end else if(sortPermission)begin
rf<=sortedarr;
end else begin
rf<=rf;
end
end
assign rd1 = (ra1 != 0) ? rf[ra1] : 0;
assign rd2 = (ra2 != 0) ? rf[ra2] : 0;
assign firstfour[3:0]=rf[0];
assign firstfour[7:4]=rf[1];
assign firstfour[11:8]=rf[2];
assign firstfour[15:12]=rf[3];
endmodule

module regFDataMemConnection(
input logic clk,
input logic[11:0] instr,
output logic [15:0] firstfour
);
logic [3:0] rfile[7:0];
logic [3:0] RAM[15:0];
logic zero,memtoreg, memwrite,pcsrc,regdst,regwrite,jump,we3,we;
logic loadOrStoreSignal;
logic[2:0] alucontrol;
logic [3:0] writeData; 
logic [3:0] rd1,rd2;
logic [2:0]wa3,ra1;
logic[3:0] aluOut;
logic[3:0] result;
logic[3:0] sortretarray[7:0];
logic sortPermission;
controller ctrl(instr[11:9],zero,memtoreg,we, pcsrc,regdst,we3,jump,alucontrol,loadOrStoreSignal,sortPermission);

mux2 #(4) separeter(instr[8:6],instr[6:4],loadOrStoreSignal,wa3);

mux2 #(4) secondsepareter(instr[2:0],instr[6:4],loadOrStoreSignal,ra1);

mux2 #(4) wa3separeter(aluOut,writeData,loadOrStoreSignal,result);

regfile rf(clk,we3,ra1, instr[5:3],wa3,result,sortPermission,sortretarray,rd1,rd2,rfile,firstfour);// write data data memden gelecek

alu4 ALU(clk,rfile,instr,rd1,rd2,alucontrol,aluOut,sortretarray);

dmem dataMemory(clk,we,instr[3:0],aluOut,writeData,RAM);
endmodule

module alu4(input logic clk,
input logic[3:0] rf[7:0],
input logic[11:0] instr,
input logic signed[3:0] A, B, 
 input logic [2:0] F, 
 output logic signed [3:0] Y,
 output logic[3:0] retarray[7:0]
 );
 logic [3:0] tmp[7:0];
 logic [3:0] dtmp[7:0];
 ascSort sorter(clk,rf,instr[2:0],instr[5:3],instr[8:6],tmp);//retarray);
 descSort dsorter(clk,rf,instr[2:0],instr[5:3],instr[8:6],dtmp);
 always_comb
 case (F[2:0])
 3'b000: begin
 Y <= A;// load
 end
 3'b001: begin
 Y <= A; //store
 end
 3'b010:begin
 Y <= (B-A);//sub
 end
 3'b011:begin
 Y <=(A+B);//add
 end
 3'b100:begin// asc sort
 retarray<=tmp;
 end
 3'b101:begin
 retarray<=dtmp;
 end
// 3'b110:begin
// end
// 3'b111:begin
// end
 default:Y<=4'bxxxx;
 endcase
endmodule

module mux2 #(parameter WIDTH = 4)
(input logic [WIDTH-1:0] d0, d1,
input logic s,
output logic [WIDTH-1:0] y);
assign y = s ? d1 : d0;
endmodule

module controller(input logic [2:0] op,
input logic zero,
output logic memtoreg, memwrite,// memwrite= we
output logic pcsrc,
output logic regdst, regwrite,// regwrite = we3
output logic jump,
output logic [2:0] alucontrol,
output logic loadOrStoreSignal,// tamamen ben ekledim
output logic sortPermission,
output logic pc
);
logic branch;

maindec md(op, memtoreg, memwrite, branch, regdst, regwrite, jump,alucontrol,loadOrStoreSignal,sortPermission);

assign pcsrc = branch & zero;
endmodule

module maindec(input logic [2:0] op,
output logic memtoreg, memwrite,// load ya da store için
output logic branch,
output logic regdst, regwrite,
output logic jump,
output logic [2:0] alucontrol,
output logic loadOrStoreSignal,// tamamen ben ekledim
output logic sortPermission
);
logic [7:0] controls;
assign {regwrite, regdst, branch, memwrite,
memtoreg, jump,loadOrStoreSignal,sortPermission} = controls;// MEMTOREG SÝGNALÝNAÝNE DAHA SONRA DÝKKAT ET YENÝDEN DÜZENLE
always_comb
case(op)
3'b000:begin 
controls <= 8'b110_010_10; // RTYPE// aslýnda LW 
alucontrol<=op;
end
3'b001:begin 
controls <= 8'b000_100_10; // SW son lsb den emin deðilim
alucontrol<=op;
end
3'b010:begin 
controls <= 8'b100_000_00; // Sub omasý lazým
alucontrol<=op;
end
3'b011:begin 
controls <= 8'b100_000_00; // Bu da add olacak
alucontrol<=op;
end
3'b100:begin 
controls <= 8'b001_000_01; // BEQ asc olacak
alucontrol<=op;
end
3'b101:begin 
controls <= 8'b001_000_01; // ADDI desc olacak
alucontrol<=op;
end
3'b110:begin 
controls <= 8'b000_001_00; // J disp olacak
alucontrol<=op;
end
default: controls <= 8'bxxxxxxxx; // illegal op
endcase
endmodule

module insmemo(input logic [2:0]pc, input logic[11:0] ins,output logic[11:0] instr);
endmodule
module dmem(input logic clk, we,
input logic [3:0] a, wd,
output logic [3:0] rd,
output logic [3:0] RAM[15:0]
);
assign rd = RAM[a[3:0]]; 

always_ff @(posedge clk)
if (we) begin RAM[a[3:0]] <= wd;
end
endmodule
module decSort(
input logic clk, input logic [3:0] rf[7:0],
input logic[2:0] length,readadress,writeadress,
output logic[3:0] retarray[7:0]
);
logic [3:0] rftmp[7:0]=rf;
dsort s(clk,rf,length,readadress,writeadress,rftmp);
assigner a(clk, rftmp,length,readadress,writeadress,retarray);

endmodule

module ascSort(
input logic clk, input logic [3:0] rf[7:0],
input logic[2:0] length,readadress,writeadress,
output logic[3:0] retarray[7:0]
);
logic [3:0] rftmp[7:0]=rf;
//logic[3:0] retarray[7:0];
sort s(clk,rf,length,readadress,writeadress,rftmp);
assigner a(clk, rftmp,length,readadress,writeadress,retarray);

endmodule
module assigner(
input logic clk,input logic [3:0] retarray[7:0],
input logic[2:0] length,readadress,writeadress,
output logic [3:0] rf[7:0]
);

always_ff@(posedge clk)begin
for(int j=0;j<8;j++)begin
rf[j]=retarray[j];
end

for(int i=0;i<length;i++)begin
rf[writeadress+i]=retarray[readadress+i];
end
end
endmodule

module sort (
  input clk,
  input logic signed [3:0] array[7:0],
  input logic[2:0] length,readadress,writeadress,
  output logic signed [3:0] sorted[7:0]
);
    logic signed [3:0] min;
    logic [3:0] min_index;
    logic [7:0] used; 
    always_ff @(posedge clk) begin
        for (int i = 0; i < 8; i++) begin
          used[i] = 1'b0;
          sorted[i]=array[i];
        end
        for (int i = readadress; (i < readadress+length)&&(i<8); i++) begin
            min = array[readadress];
            min_index = readadress;
            for (int j = readadress; (j < readadress+length)&&(j<8); j++) begin
                if (used[j] == 1'b0 && array[j] < min) begin
                    min = array[j];
                    min_index = j;
                end
            end
            sorted[i] = min;
            used[min_index] = 1'b1;
        end
    end
endmodule
module dsort (
  input clk,
  input logic signed [3:0] array[7:0],
  input logic[2:0] length,readadress,writeadress,
  output logic signed [3:0]sorted[7:0]
);
    logic signed [3:0] max;
    logic [3:0] max_index;
    logic [7:0] used; 
    always_ff @(posedge clk) begin
        for (int i = 0; i < 8; i++) begin
          used[i] = 1'b0;
          sorted[i]=array[i];
        end
        for (int i = readadress; i < readadress+length; i++) begin
            max = 4'b1001;
            max_index = readadress;
            for (int j = readadress; j < readadress+length; j++) begin
                if (used[j] == 1'b0 && !($signed(array[j]) < $signed(max))) begin
                    max = $signed(array[j]);
                    max_index = j;
                end
            end
            sorted[i] =max;
            used[max_index] = 1'b1;
        end
    end
endmodule

module debounce(input pb_1,clk,output pb_out);
logic slow_clk;
logic  Q1,Q2,Q2_bar,Q0;
clock_div u1(clk,slow_clk);
my_dff d0(slow_clk, pb_1,Q0 );

my_dff d1(slow_clk, Q0,Q1 );
my_dff d2(slow_clk, Q1,Q2 );
assign Q2_bar = ~Q2;
assign pb_out = Q1 & Q2_bar;
endmodule
module clock_div(input logic Clk_100M, output logic slow_clk);
    logic [31:0]counter=0;
    always @(posedge Clk_100M)
    begin
        counter <= (counter>=2499990)?0:counter+1;
        slow_clk <= (counter < 1250000)?1'b0:1'b1;
    end
endmodule
module my_dff(input DFF_CLOCK, D, output reg Q);
    always @ (posedge DFF_CLOCK) begin
        Q <= D;
    end
endmodule

module imem(input logic [2:0] a,
output logic [11:0] rd);
logic [11:0] RAM[7:0];
assign rd = RAM[a]; 
endmodule