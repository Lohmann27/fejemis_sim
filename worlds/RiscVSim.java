import java.io.*;
import java.util.Scanner;

public class RiscVSim {

    static int pc;
    static int instr;
    static int opcode;
    static int rd;
    static int funct3;
    static int funct7;
    static int rs1;
    static int rs2;
    static int imm;
    static int addOffset;
    static int byteOffset;
    static int base;
    static long tL1; //temporary longs
    static long tL2;
    static int[] reg = new int[32];
    static int[] mem = new int[0x100000];
    static int sp = 0x1000000; // Initialize stack pointer to 1 MB
    static String[] registerNames = {"x0","ra","sp","gp","tp",
                                     "t0","t1","t2",
                                     "s0","s1",
                                     "a0","a1","a2","a3","a4","a5","a6","a7",
                                     "s2","s3","s4","s5","s6","s7","s8","s9","s10","s11",
                                     "t3","t4","t5","t6"
                                    };
    
    //Troubleshooting variables
    static boolean pcstop = false; // set true if you want the program to stop at a specific pc
    static int insStop = 25; //The pc that the program stops at if pcstop is set true
    static boolean step = false; //If step is set true, the program prompts the user to continue after each instruction
    static int next = 1;
    static int instrCount = 0;
    static Scanner input = new Scanner(System.in); //counts amnt of instr executed
    public static void main(String[] args) {
        int instrCount = loadIns(args[0]);
        if (args.length == 0) {
            System.out.println("Usage: java RiscVSim ./addlarge.res");
            System.exit(1);
        }
        pc = 0;

        System.out.println("Program Start");

        
/* 
        for (int i = 0; i < progr.length; i++){
            System.out.print("Instr. " + i + ": ");
            for (int j = 31; j > -1; j--){
                System.out.print((progr[i]>>j) & 0x1);
            }
            System.out.println();
        }
 */
        while (true) {

            //instr = (mem[pc]) + (mem[pc + 1]<<8) + (mem[pc + 2]<<16) + (mem[pc + 3]<<24);
            
            instr = progr[pc >> 2];
            //System.out.println("pc: " + ((pc >> 2) + 1) + " Instruction: " + Integer.toHexString(progr[pc>>2]));

            if ((pc >> 2) + 1 == insStop && pcstop){
                break;
            }
            opcode = instr & 0x7f;
            rd = (instr >> 7) & 0x01f;
            funct3 = (instr >> 12) & 0x7;
            funct7 = (instr >> 25) & 0x7f;
            rs1 = (instr >> 15) & 0x01f;
            rs2 = (instr >> 20) & 0x01f;
            byteOffset = 0;
            addOffset = 0;

            switch (opcode) {
                case 0x3: //Load
                    loadOps();
                    break;
                case 0x13: //I-type operations
                    immOps();
                    break;
                case 0x17: //AUI
                    auiOp();
                    break;
                case 0x23: //Store
                    storeOps();
                    break;
                case 0x33: //R-type operations
                    aritOrLogOps();
                    break;
                case 0x37: //LUI
                    luiOp();
                    break;
                case 0x63: //B-type operations
                    branchOps();
                    break;
                case 0x67: //JALR
                    jalOp();
                    break;
                case 0x6f: //JAL
                    jalROp();
                    break;
                case 0x73: //ecall
                    ecall();
                    break;
                default:
                    System.out.println("Opcode " + Integer.toHexString(opcode) + " not yet implemented");
                    break;
            }
            if (rd == 0){
                reg[rd] = 0; //Register x0 is hardwired to zero
            }
            
            instrCount++;
            
            if (stopCheck(instrCount)){
                break;
            }
            if (opcode != 0x63 && opcode != 0x67 && opcode != 0x6f){ //If a non-branching instruction has been executed, the pc should'nt imncremnt
                pc += 4; // One instruction is four bytes
            }
            if (step){
                System.out.println("Next instruction? (y: 1 / n: 0)");
                next = input.nextInt();
                if (next == 0){
                    break;
                }
            }
            if (byteOffset % 4 == 0) {
                pc += byteOffset;
            } else {
                System.out.println("Branch misalignment exception");
            }
        }
        outputDump();
        System.out.println("Program exit");
        //System.out.println("Amount of instructions executed: " + instrCount);
        input.close();
    }
    public static void loadOps(){
        addOffset = instr >> 20;
        base = reg[rs1];
        switch (funct3){
            case 0x0: //LB
                reg[rd] = (mem[base + addOffset]<<24)>>24;
                break;
            case 0x1: //LH
                reg[rd] = ((mem[base + addOffset] + ((mem[base + addOffset + 1]) << 8)) << 16) >> 16;
                break;
            case 0x2: //LW
                reg[rd] = mem[base + addOffset] + (mem[base + addOffset + 1] << 8) + (mem[base + addOffset + 2] << 16) + (mem[base + addOffset + 3] << 24);
                break;
            case 0x4: //LBU
                reg[rd] = mem[base + addOffset];
                break;
            case 0x5: //LHU
                reg[rd] = mem[base + addOffset] + (mem[base + addOffset + 1] << 8);
                break;
            default:
                break;
        }
    }

    public static void storeOps(){
        addOffset = ((instr & 0xfe000000) >> 20) + ((instr >> 7) & 0x01f);
        base = reg[rs1];
        switch(funct3){
            case 0x0: //SB
                mem[base + addOffset] = reg[rs2] & 0xff;
                break;
            case 0x1: //SH
                mem[base + addOffset]     =  reg[rs2] & 0xff;
                mem[base + addOffset + 1] = (reg[rs2] >> 8) & 0xff;
                break;
            case 0x2: //SW
                mem[base + addOffset]     =  reg[rs2] & 0xff;
                mem[base + addOffset + 1] = (reg[rs2] >> 8)  & 0xff;
                mem[base + addOffset + 2] = (reg[rs2] >> 16) & 0xff;
                mem[base + addOffset + 3] = (reg[rs2] >> 24) & 0xff;
                break;
            default:
                System.out.println("Wrong Instruction " + (pc >> 2) + " failed to save to memory");
                break;
        }
    }

    public static void immOps(){
        imm = (instr >> 20);
        switch (funct3) {
            case 0x0: //ADDI
                reg[rd] = reg[rs1] + imm;
                break;
            case 0x1: //SLLI
                reg[rd] = reg[rs1] << imm;
                break;
            case 0x2: //SLTI
                reg[rd] = (reg[rs1] < imm) ? 1 : 0; 
                break;
            case 0x3: //SLTIU
                tL1 = imm >= 0 ? (long) imm : ((long) imm) & (0xffffffffL); //if imm negative? Make the long version positive
                tL2 = reg[rs1] >= 0 ? (long) reg[rs1] : ((long) reg[rs1]) & (0xffffffffL);
                reg[rd] = reg[rd] = (tL2 < tL1) ? 1 : 0;
                break;
            case 0x4: //XORI
                reg[rd] = reg[rs1] ^ imm; 
                break;
            case 0x5: // SRLI/SRAI
                imm &= 0x1F;
                if (funct7 == 0) {
                    reg[rd] = reg[rs1] >>> imm;
                } else if (funct7 == 0x20) {
                    reg[rd] = reg[rs1] >> imm; // Arithmetic right shift
                } else {
                    System.out.println("Funct7 " + funct7 + " at opcode " + opcode + " has not been implemented yet");
                }
                break;
            case 0x6: //ORI
                reg[rd] = reg[rs1] | imm; 
                break;
            case 0x7: //ANDI
                reg[rd] = reg[rs1] & imm; 
                break;
            default:
                System.out.println("Funct3 " + Integer.toHexString(funct3) + " at opcode "+ Integer.toHexString(opcode) + " has not been implemented yet");
                break;
        }
    }

    public static void aritOrLogOps(){
        switch (funct3){
            // case 0x0: //ADD
            //     reg[rd] = (funct7 == 0) ? reg[rs1] + reg[rs2] : reg[rs1] - reg[rs2];
            //     break;
            // case 0x0: // ADD
            //     reg[rd] = reg[rs1] + reg[rs2];
            //     break;
            case 0x0: //ADD/SUB
                reg[rd] = (funct7 == 0) ? reg[rs1] + reg[rs2] : reg[rs1] - reg[rs2];
                break;
            case 0x1: //SLL
                reg[rd] = reg[rs1] << reg[rs2];
                break;
            case 0x2: //SLT
                reg[rd] = (reg[rs1] < reg[rs2]) ? 1 : 0;
                break;
            case 0x3: //SLTU
                tL1 = reg[rs1] >= 0 ? (long) reg[rs1] : ((long) reg[rs1]) & (0xffffffffL);
                tL2 = reg[rs2] >= 0 ? (long) reg[rs2] : ((long) reg[rs2]) & (0xffffffffL);
                reg[rd] = (tL1 < tL2) ? 1 : 0; 
                break;
            case 0x4:
                reg[rd] = reg[rs1] ^ reg[rs2]; //XOR
                break;
            case 0x5: //SRL/SRA
                if (funct7 == 0){
                    reg[rd] = reg[rs1] >>> reg[rs2];
                }else{
                    reg[rd] = reg[rs1] >> reg[rs2];
                }
                break;
            case 0x6: //OR
                reg[rd] = reg[rs1] | reg[rs2];
                break;
            case 0x7: //AND
                reg[rd] = reg[rs1] & reg[rs2];
                break;
            case 0x20: //SUB
                reg[rd] = reg[rs1] - reg[rs2];
                break;
            default:
                System.out.println("Funct3 " + funct3 + " at opcode "+ opcode + " has not been implemented yet");
                break;
        }
    }

    public static void auiOp() {
        imm = (instr >> 12) << 12;
        reg[rd] = pc + imm;
    }

    public static void luiOp() {
        imm = (instr >> 12) << 12;
        reg[rd] = imm;
    }

    public static void branchOps() {
        byteOffset = ((((instr >> 20) & 0xfffff800) + ((instr << 4) & 0x400) + ((instr >> 21) & 0x3f0) + ((instr >> 8) & 0xf)) & ~(0x1)) << 1;
        if (byteOffset%4 == 0){
            switch(funct3){
                case 0x0: //BEQ
                    if (reg[rs1] == reg[rs2]){ 
                        pc += byteOffset;
                    }else{
                        pc += 4;
                    }
                    break;
                case 0x1: //BNE
                    if (reg[rs1] != reg[rs2]){
                        pc += byteOffset;
                    }else{
                        pc += 4;
                    }
                    break;
                case 0x4: //BLT
                    if (reg[rs1] < reg[rs2]){
                        pc += byteOffset;
                    }else{
                        pc += 4;
                    }
                    break;
                case 0x5: //BGE
                    if (reg[rs1] >= reg[rs2]){
                        pc += byteOffset;
                    }else{
                        pc += 4;
                    }
                    break;
                case 0x6: //BLTU
                    tL1 = reg[rs1] >= 0 ? (long) reg[rs1] : ((long) reg[rs1]) & (0xffffffffL);
                    tL2 = reg[rs2] >= 0 ? (long) reg[rs2] : ((long) reg[rs2]) & (0xffffffffL);
                    if (tL1 < tL2){
                        pc += byteOffset;
                    }else{
                        pc += 4;
                    }
                    break;
                case 0x7: //BGEU
                    tL1 = reg[rs1] >= 0 ? (long) reg[rs1] : ((long) reg[rs1]) & (0xffffffffL);
                    tL2 = reg[rs2] >= 0 ? (long) reg[rs2] : ((long) reg[rs2]) & (0xffffffffL);
                    if (tL1 >= tL2){
                        pc += byteOffset;
                    }else{
                        pc += 4;
                    }
                    break;
                default:
                    System.out.println("Funct3 " + funct3 + " at opcode " + opcode + " has not been implemented yet");
                    break;
            }
        }
    }

    public static void jalOp() {
        byteOffset = ((instr >> 20) + reg[rs1]) & ~0x1;
        if(byteOffset%4 == 0){
            reg[rd] = pc + 4;
            pc = byteOffset;
        }
        sp -= 4; // Update stack pointer
    }

    public static void jalROp() {
        byteOffset = (((instr >> 31) << 20) + (instr & 0xff000) + ((instr & 0x100000) >> 9) + ((instr & 0x7fffffff) >> 20)) & ~0x1;
        if(byteOffset%4 == 0){
            reg[rd] = pc + 4;
            pc += byteOffset;
        }
        sp -= 4; // Update stack pointer
    }

    public static void ecall() {
        if (reg[17] == 10) {
            System.out.println("Ecall 10 called, ending program.");
            System.exit(0);
        } else if (reg[17] == 1) {
            // ECALL 1: Print integer (assuming integer is in a0 register)
            System.out.println("Ecall 1 called. Printing integer: " + reg[10]);
        } else {
            System.out.println("Unsupported ECALL: " + reg[17]);
            // Handle other ECALLs if needed
        }
    }

    public static boolean stopCheck(int progrlength) {
        if (opcode == 0x73 && reg[17] == 10){// Ecall 10: End Program
            System.out.println("Ecall 10 called, ending program.");
            return true;
        }else if ((pc >> 2) >= progrlength) {//End program if there are no more instructions
            System.out.println("No more instructions, ending program.");
            return true;
        }else if (byteOffset%4 != 0) {
            System.out.println("Offset Misalignment Exception, ending program.");
            return true;
        }else if (sp < 0) {
            System.out.println("Stack overflow");
            return true;
        }else {
            return false;
        }
    }

    public static int loadIns(String inputFile){
            try (
                    InputStream inputStream = new FileInputStream(inputFile)
            ) {
                int fileSize = (int) new File(inputFile).length();
                int inputByte;
                int i = 0;
                while ((inputByte = inputStream.read()) != -1) {
                    mem[i] = (int) inputByte;
                    i++;
                }
                return fileSize/4;
            } catch (IOException ex) {
                ex.printStackTrace();
            }
            return 0;
        }

    public static void outputDump(){
        for (int i = 0; i < reg.length; i++) {
            System.out.println(registerNames[i] + ": " + Integer.toHexString(reg[i]));
            //System.out.println("x" + i + " " + reg[i]);
        }
        /* 
        for (int i = 0; i < 32; i++){
            System.out.print(registerNames[i] + ": ");
            for (int j = 31; j > -1; j--){
                System.out.print((reg[i]>>j) & 0x1);
            }
            System.out.println();
        }*/
    }
}