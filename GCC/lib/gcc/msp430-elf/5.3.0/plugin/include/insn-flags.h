/* Generated automatically by the program `genflags'
   from the machine description file `md'.  */

#ifndef GCC_INSN_FLAGS_H
#define GCC_INSN_FLAGS_H

#define HAVE_push 1
#define HAVE_pusha (TARGET_LARGE)
#define HAVE_pushm 1
#define HAVE_pop 1
#define HAVE_popa (TARGET_LARGE)
#define HAVE_popm 1
#define HAVE_grow_and_swap 1
#define HAVE_swap_and_shrink 1
#define HAVE_movqihi 1
#define HAVE_movqi_topbyte (msp430x)
#define HAVE_movqi 1
#define HAVE_movhi 1
#define HAVE_movsi_s 1
#define HAVE_movsi_x 1
#define HAVE_movpsi 1
#define HAVE_movsipsi2 (msp430x)
#define HAVE_addpsi3 1
#define HAVE_addqi3 1
#define HAVE_addhi3 1
#define HAVE_addsipsi3 1
#define HAVE_addsi3 1
#define HAVE_addhi3_cy 1
#define HAVE_addhi3_cy_i 1
#define HAVE_addchi4_cy 1
#define HAVE_subpsi3 1
#define HAVE_subqi3 1
#define HAVE_subhi3 1
#define HAVE_subsi3 1
#define HAVE_bicqi3 1
#define HAVE_bichi3 1
#define HAVE_bicpsi3 1
#define HAVE_andqi3 1
#define HAVE_andhi3 1
#define HAVE_andpsi3 1
#define HAVE_iorqi3 1
#define HAVE_iorhi3 1
#define HAVE_iorpsi3 1
#define HAVE_xorqi3 1
#define HAVE_xorhi3 1
#define HAVE_xorpsi3 1
#define HAVE_one_cmplqi2 1
#define HAVE_one_cmplhi2 1
#define HAVE_one_cmplpsi2 1
#define HAVE_extendqihi2 1
#define HAVE_zero_extendqihi2 1
#define HAVE_zero_extendhipsi2 1
#define HAVE_truncpsihi2 1
#define HAVE_extendhisi2 1
#define HAVE_extendhipsi2 (msp430x)
#define HAVE_zero_extendqisi2 (msp430x)
#define HAVE_zero_extendhisi2 (msp430x)
#define HAVE_zero_extendhisipsi2 (msp430x)
#define HAVE_extend_and_shift1_hipsi2 (msp430x)
#define HAVE_extend_and_shift2_hipsi2 (msp430x)
#define HAVE_zero_extendpsisi2 1
#define HAVE_extendpsisi2 (msp430x)
#define HAVE_truncsipsi2 1
#define HAVE_slli_1 1
#define HAVE_430x_shift_left (msp430x)
#define HAVE_slll_1 1
#define HAVE_slll_2 1
#define HAVE_srai_1 1
#define HAVE_430x_arithmetic_shift_right (msp430x)
#define HAVE_srap_1 (msp430x)
#define HAVE_srap_2 (msp430x)
#define HAVE_sral_1 1
#define HAVE_sral_2 1
#define HAVE_srli_1 1
#define HAVE_430x_logical_shift_right (msp430x)
#define HAVE_srlp_1 1
#define HAVE_srll_1 1
#define HAVE_srll_2x (msp430x)
#define HAVE_epilogue_helper 1
#define HAVE_prologue_start_marker 1
#define HAVE_prologue_end_marker 1
#define HAVE_epilogue_start_marker 1
#define HAVE_msp430_refsym_need_exit 1
#define HAVE_call_internal 1
#define HAVE_call_value_internal 1
#define HAVE_msp_return 1
#define HAVE_msp430_eh_epilogue 1
#define HAVE_jump 1
#define HAVE_indirect_jump 1
#define HAVE_cbranchpsi4_real 1
#define HAVE_cbranchqi4_real 1
#define HAVE_cbranchhi4_real 1
#define HAVE_cbranchpsi4_reversed 1
#define HAVE_cbranchqi4_reversed 1
#define HAVE_cbranchhi4_reversed 1
#define HAVE_nop 1
#define HAVE_disable_interrupts 1
#define HAVE_enable_interrupts 1
#define HAVE_push_intr_state 1
#define HAVE_pop_intr_state 1
#define HAVE_bic_SR 1
#define HAVE_bis_SR 1
#define HAVE_andneghi3 1
#define HAVE_delay_cycles_start 1
#define HAVE_delay_cycles_end 1
#define HAVE_delay_cycles_32 1
#define HAVE_delay_cycles_32x 1
#define HAVE_delay_cycles_16 1
#define HAVE_delay_cycles_16x 1
#define HAVE_delay_cycles_2 1
#define HAVE_delay_cycles_1 1
#define HAVE_mulhisi3 (optimize > 2 && msp430_hwmult_type != NONE)
#define HAVE_umulhisi3 (optimize > 2 && msp430_hwmult_type != NONE)
#define HAVE_mulsidi3 (optimize > 2 && msp430_hwmult_type != NONE)
#define HAVE_umulsidi3 (optimize > 2 && msp430_hwmult_type != NONE)
#define HAVE_movsi 1
#define HAVE_ashlhi3 1
#define HAVE_ashlsi3 1
#define HAVE_ashrhi3 1
#define HAVE_ashrsi3 1
#define HAVE_lshrhi3 1
#define HAVE_lshrsi3 1
#define HAVE_prologue 1
#define HAVE_epilogue 1
#define HAVE_call 1
#define HAVE_call_value 1
#define HAVE_eh_return 1
#define HAVE_cbranchqi4 1
#define HAVE_cbranchhi4 1
#define HAVE_cbranchpsi4 1
extern rtx        gen_push                        (rtx);
extern rtx        gen_pusha                       (rtx);
extern rtx        gen_pushm                       (rtx, rtx);
extern rtx        gen_pop                         (rtx);
extern rtx        gen_popa                        (rtx);
extern rtx        gen_popm                        (rtx, rtx, rtx);
extern rtx        gen_grow_and_swap               (void);
extern rtx        gen_swap_and_shrink             (void);
extern rtx        gen_movqihi                     (rtx, rtx);
extern rtx        gen_movqi_topbyte               (rtx, rtx);
extern rtx        gen_movqi                       (rtx, rtx);
extern rtx        gen_movhi                       (rtx, rtx);
extern rtx        gen_movsi_s                     (rtx, rtx);
extern rtx        gen_movsi_x                     (rtx, rtx);
extern rtx        gen_movpsi                      (rtx, rtx);
extern rtx        gen_movsipsi2                   (rtx, rtx);
extern rtx        gen_addpsi3                     (rtx, rtx, rtx);
extern rtx        gen_addqi3                      (rtx, rtx, rtx);
extern rtx        gen_addhi3                      (rtx, rtx, rtx);
extern rtx        gen_addsipsi3                   (rtx, rtx, rtx);
extern rtx        gen_addsi3                      (rtx, rtx, rtx);
extern rtx        gen_addhi3_cy                   (rtx, rtx, rtx);
extern rtx        gen_addhi3_cy_i                 (rtx, rtx, rtx, rtx);
extern rtx        gen_addchi4_cy                  (rtx, rtx, rtx);
extern rtx        gen_subpsi3                     (rtx, rtx, rtx);
extern rtx        gen_subqi3                      (rtx, rtx, rtx);
extern rtx        gen_subhi3                      (rtx, rtx, rtx);
extern rtx        gen_subsi3                      (rtx, rtx, rtx);
extern rtx        gen_bicqi3                      (rtx, rtx, rtx);
extern rtx        gen_bichi3                      (rtx, rtx, rtx);
extern rtx        gen_bicpsi3                     (rtx, rtx, rtx);
extern rtx        gen_andqi3                      (rtx, rtx, rtx);
extern rtx        gen_andhi3                      (rtx, rtx, rtx);
extern rtx        gen_andpsi3                     (rtx, rtx, rtx);
extern rtx        gen_iorqi3                      (rtx, rtx, rtx);
extern rtx        gen_iorhi3                      (rtx, rtx, rtx);
extern rtx        gen_iorpsi3                     (rtx, rtx, rtx);
extern rtx        gen_xorqi3                      (rtx, rtx, rtx);
extern rtx        gen_xorhi3                      (rtx, rtx, rtx);
extern rtx        gen_xorpsi3                     (rtx, rtx, rtx);
extern rtx        gen_one_cmplqi2                 (rtx, rtx);
extern rtx        gen_one_cmplhi2                 (rtx, rtx);
extern rtx        gen_one_cmplpsi2                (rtx, rtx);
extern rtx        gen_extendqihi2                 (rtx, rtx);
extern rtx        gen_zero_extendqihi2            (rtx, rtx);
extern rtx        gen_zero_extendhipsi2           (rtx, rtx);
extern rtx        gen_truncpsihi2                 (rtx, rtx);
extern rtx        gen_extendhisi2                 (rtx, rtx);
extern rtx        gen_extendhipsi2                (rtx, rtx);
extern rtx        gen_zero_extendqisi2            (rtx, rtx);
extern rtx        gen_zero_extendhisi2            (rtx, rtx);
extern rtx        gen_zero_extendhisipsi2         (rtx, rtx);
extern rtx        gen_extend_and_shift1_hipsi2    (rtx, rtx);
extern rtx        gen_extend_and_shift2_hipsi2    (rtx, rtx);
extern rtx        gen_zero_extendpsisi2           (rtx, rtx);
extern rtx        gen_extendpsisi2                (rtx, rtx);
extern rtx        gen_truncsipsi2                 (rtx, rtx);
extern rtx        gen_slli_1                      (rtx, rtx);
extern rtx        gen_430x_shift_left             (rtx, rtx, rtx);
extern rtx        gen_slll_1                      (rtx, rtx);
extern rtx        gen_slll_2                      (rtx, rtx);
extern rtx        gen_srai_1                      (rtx, rtx);
extern rtx        gen_430x_arithmetic_shift_right (rtx, rtx, rtx);
extern rtx        gen_srap_1                      (rtx, rtx);
extern rtx        gen_srap_2                      (rtx, rtx);
extern rtx        gen_sral_1                      (rtx, rtx);
extern rtx        gen_sral_2                      (rtx, rtx);
extern rtx        gen_srli_1                      (rtx, rtx);
extern rtx        gen_430x_logical_shift_right    (rtx, rtx, rtx);
extern rtx        gen_srlp_1                      (rtx, rtx);
extern rtx        gen_srll_1                      (rtx, rtx);
extern rtx        gen_srll_2x                     (rtx, rtx);
extern rtx        gen_epilogue_helper             (rtx);
extern rtx        gen_prologue_start_marker       (void);
extern rtx        gen_prologue_end_marker         (void);
extern rtx        gen_epilogue_start_marker       (void);
extern rtx        gen_msp430_refsym_need_exit     (void);
extern rtx        gen_call_internal               (rtx, rtx);
extern rtx        gen_call_value_internal         (rtx, rtx, rtx);
extern rtx        gen_msp_return                  (void);
extern rtx        gen_msp430_eh_epilogue          (void);
extern rtx        gen_jump                        (rtx);
extern rtx        gen_indirect_jump               (rtx);
extern rtx        gen_cbranchpsi4_real            (rtx, rtx, rtx, rtx);
extern rtx        gen_cbranchqi4_real             (rtx, rtx, rtx, rtx);
extern rtx        gen_cbranchhi4_real             (rtx, rtx, rtx, rtx);
extern rtx        gen_cbranchpsi4_reversed        (rtx, rtx, rtx, rtx);
extern rtx        gen_cbranchqi4_reversed         (rtx, rtx, rtx, rtx);
extern rtx        gen_cbranchhi4_reversed         (rtx, rtx, rtx, rtx);
extern rtx        gen_nop                         (void);
extern rtx        gen_disable_interrupts          (void);
extern rtx        gen_enable_interrupts           (void);
extern rtx        gen_push_intr_state             (void);
extern rtx        gen_pop_intr_state              (void);
extern rtx        gen_bic_SR                      (rtx);
extern rtx        gen_bis_SR                      (rtx);
extern rtx        gen_andneghi3                   (rtx, rtx, rtx);
extern rtx        gen_delay_cycles_start          (rtx);
extern rtx        gen_delay_cycles_end            (rtx);
extern rtx        gen_delay_cycles_32             (rtx, rtx);
extern rtx        gen_delay_cycles_32x            (rtx, rtx);
extern rtx        gen_delay_cycles_16             (rtx, rtx);
extern rtx        gen_delay_cycles_16x            (rtx, rtx);
extern rtx        gen_delay_cycles_2              (void);
extern rtx        gen_delay_cycles_1              (void);
extern rtx        gen_mulhisi3                    (rtx, rtx, rtx);
extern rtx        gen_umulhisi3                   (rtx, rtx, rtx);
extern rtx        gen_mulsidi3                    (rtx, rtx, rtx);
extern rtx        gen_umulsidi3                   (rtx, rtx, rtx);
extern rtx        gen_movsi                       (rtx, rtx);
extern rtx        gen_ashlhi3                     (rtx, rtx, rtx);
extern rtx        gen_ashlsi3                     (rtx, rtx, rtx);
extern rtx        gen_ashrhi3                     (rtx, rtx, rtx);
extern rtx        gen_ashrsi3                     (rtx, rtx, rtx);
extern rtx        gen_lshrhi3                     (rtx, rtx, rtx);
extern rtx        gen_lshrsi3                     (rtx, rtx, rtx);
extern rtx        gen_prologue                    (void);
extern rtx        gen_epilogue                    (void);
#define GEN_CALL(A, B, C, D) gen_call ((A), (B))
extern rtx        gen_call                        (rtx, rtx);
#define GEN_CALL_VALUE(A, B, C, D, E) gen_call_value ((A), (B), (C))
extern rtx        gen_call_value                  (rtx, rtx, rtx);
extern rtx        gen_eh_return                   (rtx);
extern rtx        gen_cbranchqi4                  (rtx, rtx, rtx, rtx);
extern rtx        gen_cbranchhi4                  (rtx, rtx, rtx, rtx);
extern rtx        gen_cbranchpsi4                 (rtx, rtx, rtx, rtx);

#endif /* GCC_INSN_FLAGS_H */
