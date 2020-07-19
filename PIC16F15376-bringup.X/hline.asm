#include <xc.inc>

GLOBAL _hline
SIGNAT _hline,

PSECT mytext, local, class=CODE, delta=2

_hline:
        MOVIW   FSR0++
        MOVWF   PORTB
        
