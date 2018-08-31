/***************************************************************************
 *   Copyright (C) 2011 by Andreas Fritiofson                              *
 *   andreas.fritiofson@gmail.com                                          *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.           *
 ***************************************************************************/

/***************************************************************************
 *   Modified at 2018 by Reinforce-II <fate@eastal.com>                    *
 ***************************************************************************/


.text
.syntax unified
.cpu cortex-m0
.thumb

/* Params:
* r0 - flash base (in), status (out)
* r1 - count (bytes)
* r2 - workarea start
* r3 - workarea end
* r4 - target address
* Clobbered:
* r5 - rp
* r6 - wp, tmp
* r7 - tmp
*/

.thumb_func
.global _start
_start:
wait_fifo:
    ldr     r6, [r2, #0]    /* read wp */
    cmp     r6, #0          /* abort if wp == 0 */
    beq     exit
    ldr     r5, [r2, #4]    /* read rp */
    cmp     r5, r6          /* wait until rp != wp */
    beq     wait_fifo
    ldrb    r6, [r5]        /* "*target_address++ = *rp++" */
    strb    r6, [r4]
    adds    r5, #1
    adds    r4, #1
busy:
    ldr     r6, [r0, #0x20] /* wait until BUSY flag is reset */
    movs    r7, #0x10
    tst     r6, r7
    bne     busy
    cmp     r5, r3          /* wrap rp at end of buffer */
    bcc     no_wrap
    mov     r5, r2
    adds    r5, #8
no_wrap:
    str     r5, [r2, #4]    /* store rp */
    subs    r1, r1, #1      /* decrement halfword count */
    cmp     r1, #0
    beq     exit            /* loop if not done */
    b       wait_fifo
error:
    movs    r0, #0
    str     r0, [r2, #4]    /* set rp = 0 on error */
exit:
    mov     r0, r6       /* return status in r0 */
    bkpt    #0
