// TEC-8 中断控制器
// 约束：EPM7128SLC84-15，无 LIAR/IABUS，断点保存至 R3
// 中断请求：PULSE 高电平有效，latch 锁存 IS_INT，STOP 停机
// 中断允许：INTI_FLAG（上电=1，响应后自动关，RETI 后重开）
// 新增指令：RETI (4'b1111) — 从 R3 恢复 PC，重新开中断
// 中断向量：由 R2 预置（程序开始时写入 08H）
// 断点保存方案：取指拍同时 PCINC + R3 INC，R3 始终等于当前指令地址
module interrupt_controller(
    input       CLR_n,
    input       C,
    input       Z,
    input [3:0] IR,
    input [2:0] SW,
    input       PULSE,
    input       T3,
    input [2:0] W,
    output reg  LPC,
    output reg  PCINC,
    output reg  PCADD,
    output reg  LAR,
    output reg  ARINC,
    output reg  LIR,
    output reg  DRW,
    output reg  LDZ,
    output reg  LDC,
    output reg [3:0] S,
    output reg  M,
    output reg  CIN,
    output reg  MEMW,
    output reg  ABUS,
    output reg  SBUS,
    output reg  MBUS,
    output reg [3:0] SEL,
    output reg  SELCTL,
    output      STOP,
    output reg  SHORT,
    output reg  LONG,
    output      INT_IND
);
    localparam NOP=4'b0000, ADD=4'b0001, SUB=4'b0010, AND=4'b0011,
               INC=4'b0100, LD =4'b0101, ST =4'b0110, JC =4'b0111,
               JZ =4'b1000, JMP=4'b1001, OUT=4'b1010, MOV=4'b1011,
               CMP=4'b1100, NOT=4'b1101, STP=4'b1110, RETI=4'b1111;
    localparam FETCH=3'b000, WRITE_MEMORY=3'b001, READ_MEMORY=3'b010,
               READ_REG=3'b011, WRITE_REG=3'b100;
    localparam W0=3'b000, W1=3'b001, W2=3'b010, W3=3'b100;

    reg FLAG;       // 初始化完成标志，0=初始化阶段，1=正常运行阶段
    reg INTI_FLAG;  // 中断允许标志，上电=1，响应中断后清0，RETI后置1
    reg IS_INT;     // 中断请求锁存，PULSE高电平置1并保持，响应后清0
    reg INT_CLR;    // 中断响应拍置1，用于清零IS_INT latch
    reg STOP_r;     // 时序逻辑驱动的STOP，STP指令和初始化阶段停机用
    reg [2:0] w;    // 内部节拍状态，W1/W2/W3/W0

    assign STOP = STOP_r | (PULSE & INTI_FLAG);
    assign INT_IND = IS_INT;

    // IS_INT latch：PULSE 高电平置1并保持，INT_CLR 或复位清零
    always @(PULSE, INT_CLR, CLR_n, INTI_FLAG) begin
        if (!CLR_n)
            IS_INT = 0;
        else if (INT_CLR)
            IS_INT = 0;
        else if (PULSE && INTI_FLAG)
            IS_INT = 1;
    end

    task reset_outputs;
    begin
        LPC<=0; PCINC<=0; PCADD<=0; LAR<=0; ARINC<=0; LIR<=0; DRW<=0;
        LDZ<=0; LDC<=0; S<=4'b0000; M<=0; CIN<=0;
        MEMW<=0; ABUS<=0; SBUS<=0; MBUS<=0;
        SEL<=4'b0000; SELCTL<=0; STOP_r<=0; SHORT<=0; LONG<=0; INT_CLR<=0;
    end
    endtask

    always @(negedge T3, negedge CLR_n) begin
        if (!CLR_n) begin
            reset_outputs;
            w<=W1; FLAG<=0; INTI_FLAG<=1; INT_CLR<=0;
        end
        else if (w == W0) begin
            reset_outputs;
            w<=W1; FLAG<=0;
        end
        else begin
            case (SW)

                //------ WRITE_REG ------
                WRITE_REG: begin
                    case (FLAG)
                        1'b0: case (w)
                            W1: begin reset_outputs; DRW<=1; SBUS<=1; SEL<=4'b0011; SELCTL<=1; STOP_r<=1; w<=W2; FLAG<=0; end
                            W2: begin reset_outputs; DRW<=1; SBUS<=1; SEL<=4'b0100; SELCTL<=1; STOP_r<=1; w<=W1; FLAG<=1; end
                            default:; endcase
                        1'b1: case (w)
                            W1: begin reset_outputs; DRW<=1; SBUS<=1; SEL<=4'b1001; SELCTL<=1; STOP_r<=1; w<=W2; FLAG<=1; end
                            W2: begin reset_outputs; DRW<=1; S<=4'b1110; SBUS<=1; SEL<=4'b1110; SELCTL<=1; STOP_r<=1; LONG<=1; w<=W0; FLAG<=1; end
                            default:; endcase
                        default:; endcase end

                //------ READ_REG ------
                READ_REG: begin
                    case (w)
                        W1: begin reset_outputs; SEL<=4'b0001; SELCTL<=1; STOP_r<=1; w<=W2; FLAG<=0; end
                        W2: begin reset_outputs; SEL<=4'b1011; SELCTL<=1; STOP_r<=1; LONG<=1; w<=W0; FLAG<=0; end
                        default:; endcase end

                //------ READ_MEMORY ------
                READ_MEMORY: begin
                    case (FLAG)
                        1'b0: begin reset_outputs; LAR<=1; SBUS<=1; SELCTL<=1; STOP_r<=1; SHORT<=1; w<=W1; FLAG<=1; end
                        1'b1: begin reset_outputs; ARINC<=1; MBUS<=1; SELCTL<=1; STOP_r<=1; SHORT<=1; w<=W1; FLAG<=1; end
                        default:; endcase end

                //------ WRITE_MEMORY ------
                WRITE_MEMORY: begin
                    case (FLAG)
                        1'b0: begin reset_outputs; LAR<=1; SBUS<=1; SELCTL<=1; STOP_r<=1; SHORT<=1; w<=W1; FLAG<=1; end
                        1'b1: begin reset_outputs; ARINC<=1; MEMW<=1; SBUS<=1; SELCTL<=1; STOP_r<=1; SHORT<=1; w<=W1; FLAG<=1; end
                        default:; endcase end

                //------ FETCH ------
                FETCH: begin
                    case (FLAG)
                        1'b0: case (w)
                            // 初始化：W1 停机等待，W2 从 SD 写入 PC 并同步 R3
                            W1: begin reset_outputs; STOP_r<=1; w<=W2; FLAG<=0; end
                            W2: begin reset_outputs; LPC<=1; SBUS<=1; DRW<=1; SEL<=4'b1110; w<=W1; FLAG<=1; end
                            default:; endcase
                        1'b1: begin
                            if (w == W1) begin
                                if (IS_INT && INTI_FLAG) begin
                                    // 中断响应：ALU 直通 R2（预置为 08H）写入 PC
                                    reset_outputs;
                                    LPC<=1; S<=4'b1111; M<=1; ABUS<=1;
                                    SEL<=4'b1010; SELCTL<=1;
                                    INT_CLR<=1; INTI_FLAG<=0;
                                    w<=W1; FLAG<=1;
                                end else begin
                                    // 正常取指：PCINC + LIR，INTI_FLAG=1时同步R3 INC
                                    reset_outputs;
                                    PCINC<=1; LIR<=1;
                                    if (INTI_FLAG) begin
                                        DRW<=1; S<=4'b0000; M<=0; CIN<=0; ABUS<=1;
                                        SEL<=4'b1110; SELCTL<=1;
                                    end
                                    INT_CLR<=0;
                                    w<=W2; FLAG<=1;
                                end
                            end
                            else begin
                                case (IR)
                                    NOP:  begin reset_outputs; w<=W1; FLAG<=1; end
                                    ADD:  begin reset_outputs; DRW<=1; LDZ<=1; LDC<=1; S<=4'b1001; M<=0; CIN<=1; ABUS<=1; w<=W1; FLAG<=1; end
                                    SUB:  begin reset_outputs; DRW<=1; LDZ<=1; LDC<=1; S<=4'b0110; M<=0; CIN<=0; ABUS<=1; w<=W1; FLAG<=1; end
                                    AND:  begin reset_outputs; DRW<=1; LDZ<=1; S<=4'b1011; M<=1; ABUS<=1; w<=W1; FLAG<=1; end
                                    INC:  begin reset_outputs; DRW<=1; LDZ<=1; LDC<=1; S<=4'b0000; M<=0; ABUS<=1; w<=W1; FLAG<=1; end
                                    LD: case (w)
                                        W2: begin reset_outputs; LAR<=1; S<=4'b1010; M<=1; ABUS<=1; LONG<=1; w<=W3; FLAG<=1; end
                                        W3: begin reset_outputs; DRW<=1; MBUS<=1; w<=W1; FLAG<=1; end
                                        default:; endcase
                                    ST: case (w)
                                        W2: begin reset_outputs; LAR<=1; S<=4'b1111; M<=1; ABUS<=1; LONG<=1; w<=W3; FLAG<=1; end
                                        W3: begin reset_outputs; MEMW<=1; S<=4'b1010; M<=1; ABUS<=1; w<=W1; FLAG<=1; end
                                        default:; endcase
                                    JC: case (C)
                                        1'b0: begin reset_outputs; w<=W1; FLAG<=1; end
                                        1'b1: begin reset_outputs; PCADD<=1; w<=W1; FLAG<=1; end
                                        default:; endcase
                                    JZ: case (Z)
                                        1'b0: begin reset_outputs; w<=W1; FLAG<=1; end
                                        1'b1: begin reset_outputs; PCADD<=1; w<=W1; FLAG<=1; end
                                        default:; endcase
                                    JMP:  begin reset_outputs; LPC<=1; S<=4'b1111; M<=1; ABUS<=1; w<=W1; FLAG<=1; end
                                    OUT:  begin reset_outputs; S<=4'b1010; M<=1; ABUS<=1; w<=W1; FLAG<=1; end
                                    MOV:  begin reset_outputs; DRW<=1; S<=4'b1010; M<=1; ABUS<=1; w<=W1; FLAG<=1; end
                                    CMP:  begin reset_outputs; LDZ<=1; LDC<=1; S<=4'b0110; M<=0; ABUS<=1; w<=W1; FLAG<=1; end
                                    NOT:  begin reset_outputs; DRW<=1; LDC<=1; S<=4'b0000; M<=1; ABUS<=1; w<=W1; FLAG<=1; end
                                    STP:  begin reset_outputs; STOP_r<=1; w<=W1; FLAG<=1; end
                                    RETI: begin
                                        // 从 R3 恢复 PC，重新开中断
                                        reset_outputs;
                                        LPC<=1; S<=4'b1111; M<=1; ABUS<=1;
                                        SEL<=4'b1110; SELCTL<=1; INTI_FLAG<=1;
                                        w<=W1; FLAG<=1;
                                    end
                                    default:; endcase
                            end
                        end
                        default:; endcase end

                default:; endcase
        end
    end

endmodule
