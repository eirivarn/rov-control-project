#ifndef rov_kinematics_level3_acc_h_
#define rov_kinematics_level3_acc_h_
#ifndef rov_kinematics_level3_acc_COMMON_INCLUDES_
#define rov_kinematics_level3_acc_COMMON_INCLUDES_
#include <stdlib.h>
#define S_FUNCTION_NAME simulink_only_sfcn
#define S_FUNCTION_LEVEL 2
#ifndef RTW_GENERATED_S_FUNCTION
#define RTW_GENERATED_S_FUNCTION
#endif
#include "sl_AsyncioQueue/AsyncioQueueCAPI.h"
#include "rtwtypes.h"
#include "simstruc.h"
#include "fixedpoint.h"
#include "rt_nonfinite.h"
#include "math.h"
#endif
#include "rov_kinematics_level3_acc_types.h"
#include <stddef.h>
#include "rt_defines.h"
typedef struct { real_T B_3_0_0 [ 6 ] ; real_T B_3_6_48 [ 8 ] ; real_T
B_3_14_112 [ 6 ] ; real_T B_3_20_160 [ 6 ] ; real_T B_3_26_208 [ 20 ] ;
real_T B_3_46_368 [ 20 ] ; real_T B_3_66_528 [ 8 ] ; real_T B_3_74_592 [ 8 ]
; real_T B_3_82_656 ; real_T B_3_83_664 ; real_T B_3_84_672 ; real_T
B_3_85_680 [ 8 ] ; real_T B_3_93_744 [ 8 ] ; real_T B_3_101_808 [ 8 ] ;
real_T B_3_109_872 [ 8 ] ; real_T B_3_117_936 ; real_T B_3_118_944 ; real_T
B_3_119_952 ; real_T B_3_120_960 ; real_T B_3_121_968 ; real_T B_3_122_976 ;
real_T B_3_123_984 ; real_T B_3_124_992 ; real_T B_3_125_1000 ; real_T
B_3_126_1008 ; real_T B_3_127_1016 ; real_T B_3_128_1024 ; real_T
B_3_129_1032 [ 6 ] ; real_T B_3_135_1080 ; real_T B_3_136_1088 ; real_T
B_3_137_1096 ; real_T B_3_138_1104 ; real_T B_3_139_1112 ; real_T
B_3_140_1120 ; real_T B_3_141_1128 ; real_T B_3_142_1136 ; real_T
B_3_143_1144 [ 8 ] ; real_T B_3_151_1208 [ 8 ] ; real_T B_3_159_1272 [ 8 ] ;
real_T B_3_167_1336 [ 6 ] ; real_T B_3_173_1384 [ 6 ] ; real_T B_3_179_1432 [
6 ] ; real_T B_3_185_1480 [ 6 ] ; real_T B_3_191_1528 ; real_T B_3_192_1536 ;
real_T B_3_193_1544 ; real_T B_3_194_1552 ; real_T B_3_195_1560 ; real_T
B_3_196_1568 ; real_T B_3_197_1576 ; real_T B_3_198_1584 ; real_T
B_3_199_1592 ; real_T B_3_200_1600 [ 6 ] ; real_T B_3_206_1648 [ 6 ] ; real_T
B_3_212_1696 ; real_T B_3_213_1704 ; real_T B_3_214_1712 [ 6 ] ; real_T
B_3_220_1760 ; real_T B_3_221_1768 ; real_T B_3_222_1776 ; real_T
B_3_223_1784 ; real_T B_3_224_1792 [ 6 ] ; real_T B_3_230_1840 ; real_T
B_3_231_1848 ; real_T B_3_232_1856 ; real_T B_3_233_1864 ; real_T
B_3_234_1872 ; real_T B_3_235_1880 ; real_T B_3_236_1888 ; real_T
B_3_237_1896 ; real_T B_3_238_1904 ; real_T B_3_239_1912 ; real_T
B_3_240_1920 ; real_T B_3_241_1928 ; real_T B_3_242_1936 ; real_T
B_3_243_1944 ; real_T B_3_244_1952 ; real_T B_3_245_1960 ; real_T
B_3_246_1968 ; real_T B_3_247_1976 ; real_T B_3_248_1984 ; real_T
B_3_249_1992 ; real_T B_3_250_2000 ; real_T B_3_251_2008 ; real_T
B_3_252_2016 ; real_T B_3_253_2024 ; real_T B_3_254_2032 ; real_T
B_3_255_2040 ; real_T B_3_256_2048 ; real_T B_3_257_2056 ; real_T
B_3_258_2064 ; real_T B_3_259_2072 ; real_T B_3_260_2080 ; real_T
B_3_261_2088 ; real_T B_3_262_2096 ; real_T B_3_263_2104 ; real_T
B_3_264_2112 ; real_T B_3_265_2120 [ 16 ] ; real_T B_3_281_2248 [ 8 ] ;
real_T B_3_289_2312 [ 6 ] ; real_T B_3_295_2360 [ 6 ] ; real_T B_3_301_2408 ;
real_T B_3_302_2416 ; real_T B_3_303_2424 ; real_T B_3_304_2432 ; real_T
B_3_305_2440 [ 648 ] ; real_T B_3_953_7624 [ 34560 ] ; real_T
B_3_35513_284104 [ 10368 ] ; real_T B_3_45881_367048 ; real_T
B_3_45882_367056 ; real_T B_2_45883_367064 [ 36 ] ; real_T B_1_45919_367352 [
6 ] ; real_T B_0_45925_367400 [ 160 ] ; real_T B_0_46085_368680 [ 48 ] ;
real_T B_3_46133_369064 [ 12 ] ; real_T B_3_46145_369160 [ 6 ] ; }
B_rov_kinematics_level3_T ; typedef struct { real_T NextOutput ; real_T
NextOutput_l ; real_T NextOutput_i ; real_T NextOutput_l1 ; real_T
NextOutput_m ; real_T NextOutput_p ; real_T NextOutput_k ; real_T
NextOutput_n ; real_T NextOutput_c ; real_T NextOutput_e ; real_T
NextOutput_j ; real_T NextOutput_km ; struct { real_T modelTStart ; }
TransportDelay_RWORK ; struct { real_T modelTStart ; } TransportDelay_RWORK_j
; struct { real_T modelTStart ; } TransportDelay_RWORK_c ; struct { real_T
modelTStart ; } TransportDelay_RWORK_a ; struct { real_T modelTStart ; }
TransportDelay_RWORK_c2 ; struct { real_T modelTStart ; }
TransportDelay_RWORK_h ; struct { real_T modelTStart ; }
TransportDelay_RWORK_k ; struct { real_T modelTStart ; }
TransportDelay_RWORK_n ; void * Scope_PWORK ; void * Scope1_PWORK ; void *
Scope10_PWORK ; void * Scope11_PWORK ; void * Scope2_PWORK ; void *
Scope3_PWORK ; void * Scope5_PWORK [ 6 ] ; void * Scope7_PWORK ; void *
Scope8_PWORK [ 6 ] ; void * Scope9_PWORK ; struct { void * AQHandles ; }
_asyncqueue_inserted_for_ToWorkspace1_PWORK ; struct { void * AQHandles ; }
_asyncqueue_inserted_for_ToWorkspace3_PWORK ; struct { void * TUbufferPtrs [
2 ] ; } TransportDelay_PWORK ; struct { void * TUbufferPtrs [ 2 ] ; }
TransportDelay_PWORK_e ; struct { void * TUbufferPtrs [ 2 ] ; }
TransportDelay_PWORK_i ; struct { void * TUbufferPtrs [ 2 ] ; }
TransportDelay_PWORK_o ; struct { void * TUbufferPtrs [ 2 ] ; }
TransportDelay_PWORK_e5 ; struct { void * TUbufferPtrs [ 2 ] ; }
TransportDelay_PWORK_m ; struct { void * TUbufferPtrs [ 2 ] ; }
TransportDelay_PWORK_n ; struct { void * TUbufferPtrs [ 2 ] ; }
TransportDelay_PWORK_d ; struct { void * AQHandles ; }
_asyncqueue_inserted_for_ToWorkspace_PWORK ; int32_T
MATLABFunction2_sysIdxToRun ; int32_T MATLABFunction1_sysIdxToRun ; int32_T
MATLABFunction3_sysIdxToRun ; uint32_T RandSeed ; uint32_T RandSeed_i ;
uint32_T RandSeed_m ; uint32_T RandSeed_a ; uint32_T RandSeed_h ; uint32_T
RandSeed_il ; uint32_T RandSeed_c ; uint32_T RandSeed_i2 ; uint32_T
RandSeed_d ; uint32_T RandSeed_i5 ; uint32_T RandSeed_j ; uint32_T
RandSeed_al ; struct { int_T Tail ; int_T Head ; int_T Last ; int_T
CircularBufSize ; int_T MaxNewBufSize ; } TransportDelay_IWORK ; struct {
int_T Tail ; int_T Head ; int_T Last ; int_T CircularBufSize ; int_T
MaxNewBufSize ; } TransportDelay_IWORK_j ; struct { int_T Tail ; int_T Head ;
int_T Last ; int_T CircularBufSize ; int_T MaxNewBufSize ; }
TransportDelay_IWORK_k ; struct { int_T Tail ; int_T Head ; int_T Last ;
int_T CircularBufSize ; int_T MaxNewBufSize ; } TransportDelay_IWORK_h ;
struct { int_T Tail ; int_T Head ; int_T Last ; int_T CircularBufSize ; int_T
MaxNewBufSize ; } TransportDelay_IWORK_d ; struct { int_T Tail ; int_T Head ;
int_T Last ; int_T CircularBufSize ; int_T MaxNewBufSize ; }
TransportDelay_IWORK_hm ; struct { int_T Tail ; int_T Head ; int_T Last ;
int_T CircularBufSize ; int_T MaxNewBufSize ; } TransportDelay_IWORK_d0 ;
struct { int_T Tail ; int_T Head ; int_T Last ; int_T CircularBufSize ; int_T
MaxNewBufSize ; } TransportDelay_IWORK_p ; int_T Step_MODE ; int_T
Saturation_MODE [ 8 ] ; int_T Step_MODE_n ; char_T pad_Step_MODE_n [ 4 ] ; }
DW_rov_kinematics_level3_T ; typedef struct { real_T Integrator3_CSTATE [ 6 ]
; real_T Integrator2_CSTATE [ 6 ] ; real_T Integrator1_CSTATE [ 6 ] ; real_T
StateSpace_CSTATE [ 20 ] ; real_T TransferFcn_CSTATE ; real_T
TransferFcn_CSTATE_a ; real_T TransferFcn_CSTATE_e ; real_T
TransferFcn_CSTATE_p ; real_T TransferFcn1_CSTATE ; real_T
TransferFcn2_CSTATE ; real_T TransferFcn3_CSTATE ; real_T TransferFcn4_CSTATE
; real_T TransferFcn5_CSTATE ; real_T TransferFcn6_CSTATE ; real_T
TransferFcn7_CSTATE ; real_T Integrator4_CSTATE ; real_T Integrator5_CSTATE ;
real_T TransferFcn_CSTATE_f ; real_T TransferFcn_CSTATE_g ; real_T
TransferFcn_CSTATE_l ; real_T TransferFcn_CSTATE_d ; real_T
TransferFcn_CSTATE_n ; } X_rov_kinematics_level3_T ; typedef struct { real_T
Integrator3_CSTATE [ 6 ] ; real_T Integrator2_CSTATE [ 6 ] ; real_T
Integrator1_CSTATE [ 6 ] ; real_T StateSpace_CSTATE [ 20 ] ; real_T
TransferFcn_CSTATE ; real_T TransferFcn_CSTATE_a ; real_T
TransferFcn_CSTATE_e ; real_T TransferFcn_CSTATE_p ; real_T
TransferFcn1_CSTATE ; real_T TransferFcn2_CSTATE ; real_T TransferFcn3_CSTATE
; real_T TransferFcn4_CSTATE ; real_T TransferFcn5_CSTATE ; real_T
TransferFcn6_CSTATE ; real_T TransferFcn7_CSTATE ; real_T Integrator4_CSTATE
; real_T Integrator5_CSTATE ; real_T TransferFcn_CSTATE_f ; real_T
TransferFcn_CSTATE_g ; real_T TransferFcn_CSTATE_l ; real_T
TransferFcn_CSTATE_d ; real_T TransferFcn_CSTATE_n ; }
XDot_rov_kinematics_level3_T ; typedef struct { boolean_T Integrator3_CSTATE
[ 6 ] ; boolean_T Integrator2_CSTATE [ 6 ] ; boolean_T Integrator1_CSTATE [ 6
] ; boolean_T StateSpace_CSTATE [ 20 ] ; boolean_T TransferFcn_CSTATE ;
boolean_T TransferFcn_CSTATE_a ; boolean_T TransferFcn_CSTATE_e ; boolean_T
TransferFcn_CSTATE_p ; boolean_T TransferFcn1_CSTATE ; boolean_T
TransferFcn2_CSTATE ; boolean_T TransferFcn3_CSTATE ; boolean_T
TransferFcn4_CSTATE ; boolean_T TransferFcn5_CSTATE ; boolean_T
TransferFcn6_CSTATE ; boolean_T TransferFcn7_CSTATE ; boolean_T
Integrator4_CSTATE ; boolean_T Integrator5_CSTATE ; boolean_T
TransferFcn_CSTATE_f ; boolean_T TransferFcn_CSTATE_g ; boolean_T
TransferFcn_CSTATE_l ; boolean_T TransferFcn_CSTATE_d ; boolean_T
TransferFcn_CSTATE_n ; } XDis_rov_kinematics_level3_T ; typedef struct {
real_T Integrator3_CSTATE [ 6 ] ; real_T Integrator2_CSTATE [ 6 ] ; real_T
Integrator1_CSTATE [ 6 ] ; real_T StateSpace_CSTATE [ 20 ] ; real_T
TransferFcn_CSTATE ; real_T TransferFcn_CSTATE_a ; real_T
TransferFcn_CSTATE_e ; real_T TransferFcn_CSTATE_p ; real_T
TransferFcn1_CSTATE ; real_T TransferFcn2_CSTATE ; real_T TransferFcn3_CSTATE
; real_T TransferFcn4_CSTATE ; real_T TransferFcn5_CSTATE ; real_T
TransferFcn6_CSTATE ; real_T TransferFcn7_CSTATE ; real_T Integrator4_CSTATE
; real_T Integrator5_CSTATE ; real_T TransferFcn_CSTATE_f ; real_T
TransferFcn_CSTATE_g ; real_T TransferFcn_CSTATE_l ; real_T
TransferFcn_CSTATE_d ; real_T TransferFcn_CSTATE_n ; }
CStateAbsTol_rov_kinematics_level3_T ; typedef struct { real_T
Integrator3_CSTATE [ 6 ] ; real_T Integrator2_CSTATE [ 6 ] ; real_T
Integrator1_CSTATE [ 6 ] ; real_T StateSpace_CSTATE [ 20 ] ; real_T
TransferFcn_CSTATE ; real_T TransferFcn_CSTATE_a ; real_T
TransferFcn_CSTATE_e ; real_T TransferFcn_CSTATE_p ; real_T
TransferFcn1_CSTATE ; real_T TransferFcn2_CSTATE ; real_T TransferFcn3_CSTATE
; real_T TransferFcn4_CSTATE ; real_T TransferFcn5_CSTATE ; real_T
TransferFcn6_CSTATE ; real_T TransferFcn7_CSTATE ; real_T Integrator4_CSTATE
; real_T Integrator5_CSTATE ; real_T TransferFcn_CSTATE_f ; real_T
TransferFcn_CSTATE_g ; real_T TransferFcn_CSTATE_l ; real_T
TransferFcn_CSTATE_d ; real_T TransferFcn_CSTATE_n ; }
CXPtMin_rov_kinematics_level3_T ; typedef struct { real_T Integrator3_CSTATE
[ 6 ] ; real_T Integrator2_CSTATE [ 6 ] ; real_T Integrator1_CSTATE [ 6 ] ;
real_T StateSpace_CSTATE [ 20 ] ; real_T TransferFcn_CSTATE ; real_T
TransferFcn_CSTATE_a ; real_T TransferFcn_CSTATE_e ; real_T
TransferFcn_CSTATE_p ; real_T TransferFcn1_CSTATE ; real_T
TransferFcn2_CSTATE ; real_T TransferFcn3_CSTATE ; real_T TransferFcn4_CSTATE
; real_T TransferFcn5_CSTATE ; real_T TransferFcn6_CSTATE ; real_T
TransferFcn7_CSTATE ; real_T Integrator4_CSTATE ; real_T Integrator5_CSTATE ;
real_T TransferFcn_CSTATE_f ; real_T TransferFcn_CSTATE_g ; real_T
TransferFcn_CSTATE_l ; real_T TransferFcn_CSTATE_d ; real_T
TransferFcn_CSTATE_n ; } CXPtMax_rov_kinematics_level3_T ; typedef struct {
real_T Step_StepTime_ZC ; real_T Saturation_UprLim_ZC [ 8 ] ; real_T
Saturation_LwrLim_ZC [ 8 ] ; real_T Step_StepTime_ZC_f ; }
ZCV_rov_kinematics_level3_T ; typedef struct { ZCSigState Step_StepTime_ZCE ;
ZCSigState Saturation_UprLim_ZCE [ 8 ] ; ZCSigState Saturation_LwrLim_ZCE [ 8
] ; ZCSigState Step_StepTime_ZCE_l ; } PrevZCX_rov_kinematics_level3_T ;
struct P_rov_kinematics_level3_T_ { real_T P_0 ; real_T P_1 [ 48 ] ; real_T
P_2 ; real_T P_3 ; real_T P_4 [ 198 ] ; real_T P_5 [ 168 ] ; real_T P_6 [ 20
] ; real_T P_7 ; real_T P_8 [ 160 ] ; real_T P_9 ; real_T P_10 ; real_T P_11
; real_T P_12 ; real_T P_13 ; real_T P_14 ; real_T P_15 ; real_T P_16 ;
real_T P_17 ; real_T P_18 ; real_T P_19 ; real_T P_20 ; real_T P_21 ; real_T
P_22 ; real_T P_23 ; real_T P_24 ; real_T P_25 ; real_T P_26 ; real_T P_27 ;
real_T P_28 ; real_T P_29 ; real_T P_30 ; real_T P_31 ; real_T P_32 ; real_T
P_33 ; real_T P_34 ; real_T P_35 ; real_T P_36 ; real_T P_37 ; real_T P_38 ;
real_T P_39 ; real_T P_40 ; real_T P_41 ; real_T P_42 ; real_T P_43 ; real_T
P_44 ; real_T P_45 ; real_T P_46 ; real_T P_47 ; real_T P_48 ; real_T P_49 ;
real_T P_50 ; real_T P_51 ; real_T P_52 ; real_T P_53 ; real_T P_54 ; real_T
P_55 ; real_T P_56 ; real_T P_57 ; real_T P_58 ; real_T P_59 ; real_T P_60 ;
real_T P_61 ; real_T P_62 ; real_T P_63 ; real_T P_64 [ 48 ] ; real_T P_65 [
120 ] ; real_T P_66 ; real_T P_67 ; real_T P_68 ; real_T P_69 ; real_T P_70 ;
real_T P_71 ; real_T P_72 ; real_T P_73 ; real_T P_74 ; real_T P_75 ; real_T
P_76 ; real_T P_77 ; real_T P_78 ; real_T P_79 ; real_T P_80 ; real_T P_81 ;
real_T P_82 ; real_T P_83 ; real_T P_84 ; real_T P_85 ; real_T P_86 ; real_T
P_87 ; real_T P_88 ; real_T P_89 ; real_T P_90 ; real_T P_91 ; real_T P_92 ;
real_T P_93 ; real_T P_94 ; real_T P_95 ; real_T P_96 ; real_T P_97 ; real_T
P_98 ; real_T P_99 ; real_T P_100 ; real_T P_101 ; real_T P_102 ; real_T
P_103 ; real_T P_104 ; real_T P_105 ; real_T P_106 ; real_T P_107 ; real_T
P_108 ; real_T P_109 ; real_T P_110 ; real_T P_111 ; real_T P_112 ; real_T
P_113 ; real_T P_114 ; real_T P_115 ; real_T P_116 ; real_T P_117 ; real_T
P_118 ; real_T P_119 ; real_T P_120 ; real_T P_121 ; real_T P_122 ; real_T
P_123 ; real_T P_124 ; real_T P_125 ; real_T P_126 ; real_T P_127 ; real_T
P_128 ; real_T P_129 ; real_T P_130 ; real_T P_131 ; real_T P_132 ; real_T
P_133 ; real_T P_134 ; real_T P_135 ; real_T P_136 ; real_T P_137 ; real_T
P_138 ; real_T P_139 ; real_T P_140 ; real_T P_141 ; real_T P_142 ; real_T
P_143 ; real_T P_144 ; real_T P_145 ; real_T P_146 ; real_T P_147 ; real_T
P_148 [ 8 ] ; real_T P_149 [ 6 ] ; real_T P_150 [ 6 ] ; real_T P_151 ; real_T
P_152 ; real_T P_153 ; real_T P_154 ; real_T P_155 [ 648 ] ; real_T P_156 [
34560 ] ; real_T P_157 [ 10368 ] ; real_T P_158 ; real_T P_159 ; uint32_T
P_160 [ 198 ] ; uint32_T P_161 [ 21 ] ; uint32_T P_162 [ 168 ] ; uint32_T
P_163 [ 17 ] ; uint32_T P_164 [ 20 ] ; uint32_T P_165 [ 21 ] ; uint8_T P_166
; uint8_T P_167 ; uint8_T P_168 ; uint8_T P_169 ; uint8_T P_170 ; uint8_T
P_171 ; uint8_T P_172 ; uint8_T P_173 ; uint8_T P_174 ; uint8_T P_175 ;
uint8_T P_176 ; uint8_T P_177 ; uint8_T P_178 ; uint8_T P_179 ; uint8_T P_180
; uint8_T P_181 ; uint8_T P_182 ; uint8_T P_183 ; uint8_T P_184 ; uint8_T
P_185 ; uint8_T P_186 ; char_T pad_P_186 [ 7 ] ; } ; extern
P_rov_kinematics_level3_T rov_kinematics_level3_rtDefaultP ;
#endif
