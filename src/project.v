/*
 * Copyright (c) 2024 Your Name
 * SPDX-License-Identifier: Apache-2.0
 */

`default_nettype none

// module tt_um_example (
//     input  wire [7:0] ui_in,    // Dedicated inputs
//     output wire [7:0] uo_out,   // Dedicated outputs
//     input  wire [7:0] uio_in,   // IOs: Input path
//     output wire [7:0] uio_out,  // IOs: Output path
//     output wire [7:0] uio_oe,   // IOs: Enable path (active high: 0=input, 1=output)
//     input  wire       ena,      // always 1 when the design is powered, so you can ignore it
//     input  wire       clk,      // clock
//     input  wire       rst_n     // reset_n - low to reset
// );

//   // All output pins must be assigned. If not used, assign to 0.
//   assign uo_out  = ui_in + uio_in;  // Example: ou_out is the sum of ui_in and uio_in
//   assign uio_out = 0;
//   assign uio_oe  = 0;

//   // List all unused inputs to prevent warnings
//   wire _unused = &{ena, clk, rst_n, 1'b0};

// endmodule






/*
 * Tiny 8-bit CPU for TinyTapeout
 * SPDX-License-Identifier: Apache-2.0
 */

`default_nettype none

// ============================================================
// TinyTapeout Wrapper (Top-level module)
// ============================================================
module tt_um_tinycpu (
    input  wire [7:0] ui_in,    // Dedicated inputs
    output wire [7:0] uo_out,   // Dedicated outputs
    input  wire [7:0] uio_in,   // IOs: Input path
    output wire [7:0] uio_out,  // IOs: Output path
    output wire [7:0] uio_oe,   // IOs: Enable path (0=input, 1=output)
    input  wire       ena,      // always 1 when powered
    input  wire       clk,      // clock
    input  wire       rst_n     // reset_n - low to reset
);

    // ---------------------------
    // Instantiate CPU core
    // ---------------------------
    tiny_cpu_top u_cpu (
        .clk    (clk),
        .reset_n(rst_n),
        .io_in  (ui_in),    // External input mapped to ui_in
        .io_out (uo_out)    // CPU drives uo_out
    );

    // ---------------------------
    // Unused IOs must be tied off
    // ---------------------------
    assign uio_out = 8'b0;
    assign uio_oe  = 8'b0;

    // Prevent unused warnings
    wire _unused = &{ena, uio_in, 1'b0};

endmodule

// ============================================================
// Tiny 8-bit CPU Core
// ============================================================
module tiny_cpu_top (
    input  wire       clk,
    input  wire       reset_n,
    input  wire [7:0] io_in,     // External input
    output reg  [7:0] io_out     // External output
);

    // ------------------------
    // CPU State
    // ------------------------
    reg [7:0]  A;                 // Accumulator
    reg        Z;                 // Zero flag
    reg [4:0]  PC;                // Program counter (0..31)
    reg [7:0]  IR;                // Instruction register

    localparam S_FETCH = 1'b0;
    localparam S_EXEC  = 1'b1;
    reg        state;

    // Data RAM (30 general addresses, 30=input, 31=output)
    reg [7:0] RAM [0:29];

    // Decode
    wire [2:0] opcode = IR[7:5];
    wire [4:0] imm5   = IR[4:0];

    // ROM
    wire [7:0] rom_dout;
    tiny_cpu_rom u_rom (
        .addr(PC),
        .data(rom_dout)
    );

    // RAM/IO read mux
    wire [7:0] mem_read_data =
        (imm5 == 5'd30) ? io_in  :   // Input
        (imm5 == 5'd31) ? io_out :   // Output (readback)
                           RAM[imm5];

    // ------------------------
    // Main FSM
    // ------------------------
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            A      <= 8'd0;
            Z      <= 1'b0;
            PC     <= 5'd0;
            IR     <= 8'd0;
            io_out <= 8'd0;
            state  <= S_FETCH;
        end else begin
            case (state)
                // FETCH
                S_FETCH: begin
                    IR    <= rom_dout;
                    PC    <= PC + 5'd1;
                    state <= S_EXEC;
                end

                // EXEC
                S_EXEC: begin
                    case (opcode)
                        3'b000: begin
                            // NOP
                        end
                        3'b001: begin
                            // LDI imm
                            A <= {3'b000, imm5};
                            Z <= ({3'b000, imm5} == 8'd0);
                        end
                        3'b010: begin
                            // LDA addr
                            A <= mem_read_data;
                            Z <= (mem_read_data == 8'd0);
                        end
                        3'b011: begin
                            // STA addr
                            if (imm5 == 5'd31) begin
                                io_out <= A;
                            end else if (imm5 < 5'd30) begin
                                RAM[imm5] <= A;
                            end
                        end
                        3'b100: begin
                            // ADD addr
                            A <= A + mem_read_data;
                            Z <= ((A + mem_read_data) == 8'd0);
                        end
                        3'b101: begin
                            // JMP addr
                            PC <= imm5;
                        end
                        3'b110: begin
                            // BEQ addr
                            if (Z) PC <= imm5;
                        end
                        3'b111: begin
                            // BNE addr
                            if (!Z) PC <= imm5;
                        end
                    endcase
                    state <= S_FETCH;
                end
            endcase
        end
    end

endmodule

// ============================================================
// Program ROM (32 x 8)
// Demo: count up and display on uo_out
// ============================================================
module tiny_cpu_rom (
    input  wire [4:0] addr,
    output reg  [7:0] data
);
    // Opcodes
    localparam NOP = 3'b000;
    localparam LDI = 3'b001;
    localparam LDA = 3'b010;
    localparam STA = 3'b011;
    localparam ADD = 3'b100;
    localparam JMP = 3'b101;
    localparam BEQ = 3'b110;
    localparam BNE = 3'b111;

    function [7:0] I;
        input [2:0] op;
        input [4:0] imm5;
        begin
            I = {op, imm5};
        end
    endfunction

    always @* begin
        case (addr)
            // Init: store 1 at RAM[29]
            5'd0:  data = I(LDI, 5'd1);   // A=1
            5'd1:  data = I(STA, 5'd29);  // RAM[29]=1
            5'd2:  data = I(LDI, 5'd0);   // A=0

            // Main loop: increment and output
            5'd3:  data = I(STA, 5'd31);  // OUT=A
            5'd4:  data = I(ADD, 5'd29);  // A=A+1
            5'd5:  data = I(JMP, 5'd3);   // loop

            default: data = I(NOP, 5'd0);
        endcase
    end
endmodule

