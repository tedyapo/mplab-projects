;;;
;;; main_loop.asm: decade-flashlight-v2
;;;

#include <xc.inc>  
#include <pic12lf1572.inc>

;;;
;;; LED pulse spacing macro -- add NOPs between pulses to recapture
;;;   energy in LC ringing
;;; 
        NOP_SPACE       macro
        NOP
        endm

;;;
;;; external unit16_t: 65536 - _reset_count flashes before auto reset
;;; 
        GLOBAL          _reset_count
        

;;;
;;; flashlight main loop: wake from sleep based on PWM1 period interrupts
;;;   if awakened by WDT, reset immediately
;;;   else, reset after set number of flashes
;;; 
        GLOBAL          _main_loop
        SIGNAT          _main_loop, 88
        PSECT           main_loop_text,local,class=CODE,delta=2
_main_loop:
        BANKSEL(LATA)
        REPT            5
        bsf             LATA, 5         ; begin inductor current ramp
        bcf             LATA, 5         ; end inductor current ramp
        NOP_SPACE                       ; delay to allow LED to flash
        ENDM
        bsf             LATA, 5         ; begin inductor current ramp
        bcf             LATA, 5         ; end inductor current ramp
        SLEEP                           ; sleep; wake by PWM1 or WDT
        BTFSS           nTO             ; if awakened by WDT, something is wrong
        RESET                           ;     so, reset immediately
        BANKSEL(PWM1INTF)
        BCF             PWM1PRIF        ; clear PWM1 interrupt flag
        BANKSEL(_reset_count)
        INCFSZ          _reset_count    ; increment low byte of reset count
        BRA             _main_loop      ; continue loop if no overflow
        INCFSZ          _reset_count+1  ; increment high byte of reset count
        BRA             _main_loop      ; continue loop if no overflow
        RESET                           ; reset flash count met: reset/re-init
