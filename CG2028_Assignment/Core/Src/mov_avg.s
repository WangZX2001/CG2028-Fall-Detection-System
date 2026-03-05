/*
 * mov_avg.s
 *
 * Created on: 2/2/2026
 * Author: Hitesh B, Hou Linxin
 */
.syntax unified
 .cpu cortex-m4
 .thumb
 .global mov_avg
 .equ N_MAX, 8
 .bss
 .align 4

 .text
 .align 2
@ CG2028 Assignment, Sem 2, AY 2025/26
@ (c) ECE NUS, 2025
@ Write Student 1’s Name here: ABCD (A1234567R)
@ Write Student 2’s Name here: WXYZ (A0000007X)
@ You could create a look-up table of registers here:
@ R0 ...
@ R1 ...
@ write your program from here:
mov_avg:
 	PUSH {r2-r11, lr}

    // Input:
    // R0 = N (buffer size)
    // R1 = accel_buff (pointer to integer array)

    // Safety check: Prevent division by zero
    CMP     R0, #0
    BLE     .L_return_zero  // If N <= 0, branch to return 0

    MOV     R2, #0          // R2 = Accumulating sum (initialize to 0)
    MOV     R3, R0          // R3 = Save the original N for the final division
    MOV     R12, R0         // R12 = Loop counter (starts at N)

.L_sum_loop:
    LDR     R0, [R1], #4    // Load current int into R0, then post-increment R1 by 4 bytes
    ADD     R2, R2, R0      // Add the loaded value to the sum (R2 += R0)
    SUBS    R12, R12, #1    // Decrement loop counter and set condition flags
    BNE     .L_sum_loop     // If counter != 0, branch back to the start of the loop

    // Compute average
    SDIV    R0, R2, R3      // Signed divide: R0 = R2 (sum) / R3 (N)
	POP {r2-r11, pc}

.L_return_zero:
    MOV     R0, #0          // Set return value to 0
	POP {r2-r11, pc}


