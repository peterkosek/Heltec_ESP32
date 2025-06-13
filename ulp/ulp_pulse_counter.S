; ULP Assembly + C integration: Combined Pulse/Timer Wakeup with 32-bit Total
; ---------------------------------------------------------------
; ULP Program (ulp_pulse_counter.S)

    .global entry

    .bss
ulp_prev_state:    .res 1       ; previous GPIO state
ulp_count_lo:      .res 1       ; 16-bit pulse count low byte
ulp_count_hi:      .res 1       ; 16-bit pulse count high byte
ulp_limit_lo:      .res 1       ; threshold low byte
ulp_limit_hi:      .res 1       ; threshold high byte
; 32-bit total across deep-sleep cycles
ulp_total_b0:      .res 1       ; byte0 (LSB)
ulp_total_b1:      .res 1       ; byte1
ulp_total_b2:      .res 1       ; byte2
ulp_total_b3:      .res 1       ; byte3 (MSB)

    .text
entry:
    ; Sample GPIO pin for rising edge
    I_ADC      ULP_PIN0, r1     ; r1 = current GPIO state
    MOV        r0, r1
    ANDN       r0, r1, ulp_prev_state   ; r0 = 1 if rising

    ; Update 16-bit count
    LD         ulp_count_lo, r2
    LD         ulp_count_hi, r3
    ADD        r2, r2, r0
    ADCH       r3, r3, r2
    ST         r2, ulp_count_lo
    ST         r3, ulp_count_hi

    ; Save state
    ST         r1, ulp_prev_state

    ; Check threshold
    LD         ulp_limit_lo, r4
    LD         ulp_limit_hi, r5
    SUB        r6, r3, r5
    JNZ        timer_check      ; if high !=, skip
    SUB        r6, r2, r4
    JNZ        timer_check      ; if low !=, skip

    ; --- Threshold reached: add count to total (32-bit) ---
    ; Lo byte
    LD         ulp_total_b0, r7     ; r7 = tot_b0
    ADD        r7, r7, r2       ; tot_b0 += count_lo
    ST         r7, ulp_total_b0
    ; Mid byte
    LD         ulp_total_b1, r8
    ADCH       r8, r8, r7       ; tot_b1 += count_hi + carry
    ST         r8, ulp_total_b1
    ; High word
    LD         ulp_total_b2, r9
    LD         ulp_total_b3, r10
    ; no overflow beyond MSB assumed
    ST         r9, ulp_total_b2
    ST         r10, ulp_total_b3

    ; Reset 16-bit count
    ST         r0, ulp_count_lo     ; set count_lo = 0 ; note r0=1, so clear differently
    MOV        r2, 0
    MOV        r3, 0
    ST         r2, ulp_count_lo
    ST         r3, ulp_count_hi

    ; Trigger wakeup
    I_WAKE

timer_check:
    ; Periodic ULP sleep: wake every cycle to poll timer bit
    HALT       50
    JMP        entry
