/*
 *    This file was auto-generated using the ACADO Toolkit.
 *    
 *    While ACADO Toolkit is free software released under the terms of
 *    the GNU Lesser General Public License (LGPL), the generated code
 *    as such remains the property of the user who used ACADO Toolkit
 *    to generate this code. In particular, user dependent data of the code
 *    do not inherit the GNU LGPL license. On the other hand, parts of the
 *    generated code that are a direct copy of source code from the
 *    ACADO Toolkit or the software tools it is based on, remain, as derived
 *    work, automatically covered by the LGPL license.
 *    
 *    ACADO Toolkit is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *    
 */


#include "nmpc_common.h"




/******************************************************************************/
/*                                                                            */
/* ACADO code generation                                                      */
/*                                                                            */
/******************************************************************************/


int nmpc_modelSimulation(  )
{
int ret;

int lRun1;
int lRun2;
ret = 0;
for (lRun1 = 0; lRun1 < 30; ++lRun1)
{
nmpcWorkspace.state[0] = nmpcVariables.x[lRun1 * 12];
nmpcWorkspace.state[1] = nmpcVariables.x[lRun1 * 12 + 1];
nmpcWorkspace.state[2] = nmpcVariables.x[lRun1 * 12 + 2];
nmpcWorkspace.state[3] = nmpcVariables.x[lRun1 * 12 + 3];
nmpcWorkspace.state[4] = nmpcVariables.x[lRun1 * 12 + 4];
nmpcWorkspace.state[5] = nmpcVariables.x[lRun1 * 12 + 5];
nmpcWorkspace.state[6] = nmpcVariables.x[lRun1 * 12 + 6];
nmpcWorkspace.state[7] = nmpcVariables.x[lRun1 * 12 + 7];
nmpcWorkspace.state[8] = nmpcVariables.x[lRun1 * 12 + 8];
nmpcWorkspace.state[9] = nmpcVariables.x[lRun1 * 12 + 9];
nmpcWorkspace.state[10] = nmpcVariables.x[lRun1 * 12 + 10];
nmpcWorkspace.state[11] = nmpcVariables.x[lRun1 * 12 + 11];

nmpcWorkspace.state[204] = nmpcVariables.u[lRun1 * 4];
nmpcWorkspace.state[205] = nmpcVariables.u[lRun1 * 4 + 1];
nmpcWorkspace.state[206] = nmpcVariables.u[lRun1 * 4 + 2];
nmpcWorkspace.state[207] = nmpcVariables.u[lRun1 * 4 + 3];
nmpcWorkspace.state[208] = nmpcVariables.od[lRun1 * 12];
nmpcWorkspace.state[209] = nmpcVariables.od[lRun1 * 12 + 1];
nmpcWorkspace.state[210] = nmpcVariables.od[lRun1 * 12 + 2];
nmpcWorkspace.state[211] = nmpcVariables.od[lRun1 * 12 + 3];
nmpcWorkspace.state[212] = nmpcVariables.od[lRun1 * 12 + 4];
nmpcWorkspace.state[213] = nmpcVariables.od[lRun1 * 12 + 5];
nmpcWorkspace.state[214] = nmpcVariables.od[lRun1 * 12 + 6];
nmpcWorkspace.state[215] = nmpcVariables.od[lRun1 * 12 + 7];
nmpcWorkspace.state[216] = nmpcVariables.od[lRun1 * 12 + 8];
nmpcWorkspace.state[217] = nmpcVariables.od[lRun1 * 12 + 9];
nmpcWorkspace.state[218] = nmpcVariables.od[lRun1 * 12 + 10];
nmpcWorkspace.state[219] = nmpcVariables.od[lRun1 * 12 + 11];

ret = nmpc_integrate(nmpcWorkspace.state, 1);

nmpcWorkspace.d[lRun1 * 12] = nmpcWorkspace.state[0] - nmpcVariables.x[lRun1 * 12 + 12];
nmpcWorkspace.d[lRun1 * 12 + 1] = nmpcWorkspace.state[1] - nmpcVariables.x[lRun1 * 12 + 13];
nmpcWorkspace.d[lRun1 * 12 + 2] = nmpcWorkspace.state[2] - nmpcVariables.x[lRun1 * 12 + 14];
nmpcWorkspace.d[lRun1 * 12 + 3] = nmpcWorkspace.state[3] - nmpcVariables.x[lRun1 * 12 + 15];
nmpcWorkspace.d[lRun1 * 12 + 4] = nmpcWorkspace.state[4] - nmpcVariables.x[lRun1 * 12 + 16];
nmpcWorkspace.d[lRun1 * 12 + 5] = nmpcWorkspace.state[5] - nmpcVariables.x[lRun1 * 12 + 17];
nmpcWorkspace.d[lRun1 * 12 + 6] = nmpcWorkspace.state[6] - nmpcVariables.x[lRun1 * 12 + 18];
nmpcWorkspace.d[lRun1 * 12 + 7] = nmpcWorkspace.state[7] - nmpcVariables.x[lRun1 * 12 + 19];
nmpcWorkspace.d[lRun1 * 12 + 8] = nmpcWorkspace.state[8] - nmpcVariables.x[lRun1 * 12 + 20];
nmpcWorkspace.d[lRun1 * 12 + 9] = nmpcWorkspace.state[9] - nmpcVariables.x[lRun1 * 12 + 21];
nmpcWorkspace.d[lRun1 * 12 + 10] = nmpcWorkspace.state[10] - nmpcVariables.x[lRun1 * 12 + 22];
nmpcWorkspace.d[lRun1 * 12 + 11] = nmpcWorkspace.state[11] - nmpcVariables.x[lRun1 * 12 + 23];

for (lRun2 = 0; lRun2 < 144; ++lRun2)
nmpcWorkspace.evGx[(0) + ((lRun2) + (lRun1 * 144))] = nmpcWorkspace.state[lRun2 + 12];


nmpcWorkspace.evGu[lRun1 * 48] = nmpcWorkspace.state[156];
nmpcWorkspace.evGu[lRun1 * 48 + 1] = nmpcWorkspace.state[157];
nmpcWorkspace.evGu[lRun1 * 48 + 2] = nmpcWorkspace.state[158];
nmpcWorkspace.evGu[lRun1 * 48 + 3] = nmpcWorkspace.state[159];
nmpcWorkspace.evGu[lRun1 * 48 + 4] = nmpcWorkspace.state[160];
nmpcWorkspace.evGu[lRun1 * 48 + 5] = nmpcWorkspace.state[161];
nmpcWorkspace.evGu[lRun1 * 48 + 6] = nmpcWorkspace.state[162];
nmpcWorkspace.evGu[lRun1 * 48 + 7] = nmpcWorkspace.state[163];
nmpcWorkspace.evGu[lRun1 * 48 + 8] = nmpcWorkspace.state[164];
nmpcWorkspace.evGu[lRun1 * 48 + 9] = nmpcWorkspace.state[165];
nmpcWorkspace.evGu[lRun1 * 48 + 10] = nmpcWorkspace.state[166];
nmpcWorkspace.evGu[lRun1 * 48 + 11] = nmpcWorkspace.state[167];
nmpcWorkspace.evGu[lRun1 * 48 + 12] = nmpcWorkspace.state[168];
nmpcWorkspace.evGu[lRun1 * 48 + 13] = nmpcWorkspace.state[169];
nmpcWorkspace.evGu[lRun1 * 48 + 14] = nmpcWorkspace.state[170];
nmpcWorkspace.evGu[lRun1 * 48 + 15] = nmpcWorkspace.state[171];
nmpcWorkspace.evGu[lRun1 * 48 + 16] = nmpcWorkspace.state[172];
nmpcWorkspace.evGu[lRun1 * 48 + 17] = nmpcWorkspace.state[173];
nmpcWorkspace.evGu[lRun1 * 48 + 18] = nmpcWorkspace.state[174];
nmpcWorkspace.evGu[lRun1 * 48 + 19] = nmpcWorkspace.state[175];
nmpcWorkspace.evGu[lRun1 * 48 + 20] = nmpcWorkspace.state[176];
nmpcWorkspace.evGu[lRun1 * 48 + 21] = nmpcWorkspace.state[177];
nmpcWorkspace.evGu[lRun1 * 48 + 22] = nmpcWorkspace.state[178];
nmpcWorkspace.evGu[lRun1 * 48 + 23] = nmpcWorkspace.state[179];
nmpcWorkspace.evGu[lRun1 * 48 + 24] = nmpcWorkspace.state[180];
nmpcWorkspace.evGu[lRun1 * 48 + 25] = nmpcWorkspace.state[181];
nmpcWorkspace.evGu[lRun1 * 48 + 26] = nmpcWorkspace.state[182];
nmpcWorkspace.evGu[lRun1 * 48 + 27] = nmpcWorkspace.state[183];
nmpcWorkspace.evGu[lRun1 * 48 + 28] = nmpcWorkspace.state[184];
nmpcWorkspace.evGu[lRun1 * 48 + 29] = nmpcWorkspace.state[185];
nmpcWorkspace.evGu[lRun1 * 48 + 30] = nmpcWorkspace.state[186];
nmpcWorkspace.evGu[lRun1 * 48 + 31] = nmpcWorkspace.state[187];
nmpcWorkspace.evGu[lRun1 * 48 + 32] = nmpcWorkspace.state[188];
nmpcWorkspace.evGu[lRun1 * 48 + 33] = nmpcWorkspace.state[189];
nmpcWorkspace.evGu[lRun1 * 48 + 34] = nmpcWorkspace.state[190];
nmpcWorkspace.evGu[lRun1 * 48 + 35] = nmpcWorkspace.state[191];
nmpcWorkspace.evGu[lRun1 * 48 + 36] = nmpcWorkspace.state[192];
nmpcWorkspace.evGu[lRun1 * 48 + 37] = nmpcWorkspace.state[193];
nmpcWorkspace.evGu[lRun1 * 48 + 38] = nmpcWorkspace.state[194];
nmpcWorkspace.evGu[lRun1 * 48 + 39] = nmpcWorkspace.state[195];
nmpcWorkspace.evGu[lRun1 * 48 + 40] = nmpcWorkspace.state[196];
nmpcWorkspace.evGu[lRun1 * 48 + 41] = nmpcWorkspace.state[197];
nmpcWorkspace.evGu[lRun1 * 48 + 42] = nmpcWorkspace.state[198];
nmpcWorkspace.evGu[lRun1 * 48 + 43] = nmpcWorkspace.state[199];
nmpcWorkspace.evGu[lRun1 * 48 + 44] = nmpcWorkspace.state[200];
nmpcWorkspace.evGu[lRun1 * 48 + 45] = nmpcWorkspace.state[201];
nmpcWorkspace.evGu[lRun1 * 48 + 46] = nmpcWorkspace.state[202];
nmpcWorkspace.evGu[lRun1 * 48 + 47] = nmpcWorkspace.state[203];
}
return ret;
}

void nmpc_evaluateLSQ(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* u = in + 12;
const real_t* od = in + 16;
/* Vector of auxiliary variables; number of elements: 108. */
real_t* a = nmpcWorkspace.objAuxVar;

/* Compute intermediate quantities: */
a[0] = (od[6]-xd[0]);
a[1] = (od[7]-xd[1]);
a[2] = (od[8]-xd[2]);
a[3] = (sqrt((((a[0]*a[0])+(a[1]*a[1]))+(a[2]*a[2]))));
a[4] = (a[3]+(real_t)(1.0000000000000000e-03));
a[5] = (cos(u[2]));
a[6] = (sin(u[2]));
a[7] = (((real_t)(1.0000000000000000e+00)/a[4])*((a[5]*a[0])+(a[6]*a[1])));
a[8] = (sin(u[2]));
a[9] = (cos(u[2]));
a[10] = (cos(u[2]));
a[11] = (sin(u[2]));
a[12] = (((real_t)(1.0000000000000000e+00)/a[4])*(((((((real_t)(0.0000000000000000e+00)-a[8])*od[2])*a[0])+(a[9]*((real_t)(0.0000000000000000e+00)-xd[3])))+((a[10]*od[2])*a[1]))+(a[11]*((real_t)(0.0000000000000000e+00)-xd[4]))));
a[13] = (((od[9]*a[0])+(od[10]*a[1]))+(od[11]*a[2]));
a[14] = ((real_t)(0.0000000000000000e+00)-(real_t)(1.0000000000000000e+00));
a[15] = (1.0/sqrt((((a[0]*a[0])+(a[1]*a[1]))+(a[2]*a[2]))));
a[16] = (a[15]*(real_t)(5.0000000000000000e-01));
a[17] = (((a[14]*a[0])+(a[0]*a[14]))*a[16]);
a[18] = ((real_t)(1.0000000000000000e+00)/a[4]);
a[19] = (a[18]*a[18]);
a[20] = ((((real_t)(0.0000000000000000e+00)-(a[17]*a[19]))*((a[5]*a[0])+(a[6]*a[1])))+(((real_t)(1.0000000000000000e+00)/a[4])*(a[5]*a[14])));
a[21] = ((real_t)(0.0000000000000000e+00)-(real_t)(1.0000000000000000e+00));
a[22] = (((a[21]*a[1])+(a[1]*a[21]))*a[16]);
a[23] = ((((real_t)(0.0000000000000000e+00)-(a[22]*a[19]))*((a[5]*a[0])+(a[6]*a[1])))+(((real_t)(1.0000000000000000e+00)/a[4])*(a[6]*a[21])));
a[24] = ((real_t)(0.0000000000000000e+00)-(real_t)(1.0000000000000000e+00));
a[25] = (((a[24]*a[2])+(a[2]*a[24]))*a[16]);
a[26] = (((real_t)(0.0000000000000000e+00)-(a[25]*a[19]))*((a[5]*a[0])+(a[6]*a[1])));
a[27] = (real_t)(0.0000000000000000e+00);
a[28] = (real_t)(0.0000000000000000e+00);
a[29] = (real_t)(0.0000000000000000e+00);
a[30] = (real_t)(0.0000000000000000e+00);
a[31] = (real_t)(0.0000000000000000e+00);
a[32] = (real_t)(0.0000000000000000e+00);
a[33] = (real_t)(0.0000000000000000e+00);
a[34] = (real_t)(0.0000000000000000e+00);
a[35] = (real_t)(0.0000000000000000e+00);
a[36] = ((real_t)(0.0000000000000000e+00)-(real_t)(1.0000000000000000e+00));
a[37] = (((a[36]*a[0])+(a[0]*a[36]))*a[16]);
a[38] = ((real_t)(1.0000000000000000e+00)/a[4]);
a[39] = (a[38]*a[38]);
a[40] = ((((real_t)(0.0000000000000000e+00)-(a[37]*a[39]))*(((((((real_t)(0.0000000000000000e+00)-a[8])*od[2])*a[0])+(a[9]*((real_t)(0.0000000000000000e+00)-xd[3])))+((a[10]*od[2])*a[1]))+(a[11]*((real_t)(0.0000000000000000e+00)-xd[4]))))+(((real_t)(1.0000000000000000e+00)/a[4])*((((real_t)(0.0000000000000000e+00)-a[8])*od[2])*a[36])));
a[41] = ((real_t)(0.0000000000000000e+00)-(real_t)(1.0000000000000000e+00));
a[42] = (((a[41]*a[1])+(a[1]*a[41]))*a[16]);
a[43] = ((((real_t)(0.0000000000000000e+00)-(a[42]*a[39]))*(((((((real_t)(0.0000000000000000e+00)-a[8])*od[2])*a[0])+(a[9]*((real_t)(0.0000000000000000e+00)-xd[3])))+((a[10]*od[2])*a[1]))+(a[11]*((real_t)(0.0000000000000000e+00)-xd[4]))))+(((real_t)(1.0000000000000000e+00)/a[4])*((a[10]*od[2])*a[41])));
a[44] = ((real_t)(0.0000000000000000e+00)-(real_t)(1.0000000000000000e+00));
a[45] = (((a[44]*a[2])+(a[2]*a[44]))*a[16]);
a[46] = (((real_t)(0.0000000000000000e+00)-(a[45]*a[39]))*(((((((real_t)(0.0000000000000000e+00)-a[8])*od[2])*a[0])+(a[9]*((real_t)(0.0000000000000000e+00)-xd[3])))+((a[10]*od[2])*a[1]))+(a[11]*((real_t)(0.0000000000000000e+00)-xd[4]))));
a[47] = (((real_t)(1.0000000000000000e+00)/a[4])*(a[9]*((real_t)(0.0000000000000000e+00)-(real_t)(1.0000000000000000e+00))));
a[48] = (((real_t)(1.0000000000000000e+00)/a[4])*(a[11]*((real_t)(0.0000000000000000e+00)-(real_t)(1.0000000000000000e+00))));
a[49] = (real_t)(0.0000000000000000e+00);
a[50] = (real_t)(0.0000000000000000e+00);
a[51] = (real_t)(0.0000000000000000e+00);
a[52] = (real_t)(0.0000000000000000e+00);
a[53] = (real_t)(0.0000000000000000e+00);
a[54] = (real_t)(0.0000000000000000e+00);
a[55] = (real_t)(0.0000000000000000e+00);
a[56] = ((real_t)(0.0000000000000000e+00)-(real_t)(1.0000000000000000e+00));
a[57] = (((a[56]*a[0])+(a[0]*a[56]))*a[16]);
a[58] = ((real_t)(0.0000000000000000e+00)-(real_t)(1.0000000000000000e+00));
a[59] = (((a[58]*a[1])+(a[1]*a[58]))*a[16]);
a[60] = ((real_t)(0.0000000000000000e+00)-(real_t)(1.0000000000000000e+00));
a[61] = (((a[60]*a[2])+(a[2]*a[60]))*a[16]);
a[62] = (real_t)(0.0000000000000000e+00);
a[63] = (real_t)(0.0000000000000000e+00);
a[64] = (real_t)(0.0000000000000000e+00);
a[65] = (real_t)(0.0000000000000000e+00);
a[66] = (real_t)(0.0000000000000000e+00);
a[67] = (real_t)(0.0000000000000000e+00);
a[68] = (real_t)(0.0000000000000000e+00);
a[69] = (real_t)(0.0000000000000000e+00);
a[70] = (real_t)(0.0000000000000000e+00);
a[71] = ((real_t)(0.0000000000000000e+00)-(real_t)(1.0000000000000000e+00));
a[72] = (od[9]*a[71]);
a[73] = ((real_t)(0.0000000000000000e+00)-(real_t)(1.0000000000000000e+00));
a[74] = (od[10]*a[73]);
a[75] = ((real_t)(0.0000000000000000e+00)-(real_t)(1.0000000000000000e+00));
a[76] = (od[11]*a[75]);
a[77] = (real_t)(0.0000000000000000e+00);
a[78] = (real_t)(0.0000000000000000e+00);
a[79] = (real_t)(0.0000000000000000e+00);
a[80] = (real_t)(0.0000000000000000e+00);
a[81] = (real_t)(0.0000000000000000e+00);
a[82] = (real_t)(0.0000000000000000e+00);
a[83] = (real_t)(0.0000000000000000e+00);
a[84] = (real_t)(0.0000000000000000e+00);
a[85] = (real_t)(0.0000000000000000e+00);
a[86] = (real_t)(0.0000000000000000e+00);
a[87] = (real_t)(0.0000000000000000e+00);
a[88] = ((real_t)(-1.0000000000000000e+00)*(sin(u[2])));
a[89] = (cos(u[2]));
a[90] = (((real_t)(1.0000000000000000e+00)/a[4])*((a[88]*a[0])+(a[89]*a[1])));
a[91] = (real_t)(0.0000000000000000e+00);
a[92] = (real_t)(0.0000000000000000e+00);
a[93] = (real_t)(0.0000000000000000e+00);
a[94] = (cos(u[2]));
a[95] = ((real_t)(-1.0000000000000000e+00)*(sin(u[2])));
a[96] = ((real_t)(-1.0000000000000000e+00)*(sin(u[2])));
a[97] = (cos(u[2]));
a[98] = (((real_t)(1.0000000000000000e+00)/a[4])*(((((((real_t)(0.0000000000000000e+00)-a[94])*od[2])*a[0])+(a[95]*((real_t)(0.0000000000000000e+00)-xd[3])))+((a[96]*od[2])*a[1]))+(a[97]*((real_t)(0.0000000000000000e+00)-xd[4]))));
a[99] = (real_t)(0.0000000000000000e+00);
a[100] = (real_t)(0.0000000000000000e+00);
a[101] = (real_t)(0.0000000000000000e+00);
a[102] = (real_t)(0.0000000000000000e+00);
a[103] = (real_t)(0.0000000000000000e+00);
a[104] = (real_t)(0.0000000000000000e+00);
a[105] = (real_t)(0.0000000000000000e+00);
a[106] = (real_t)(0.0000000000000000e+00);
a[107] = (real_t)(0.0000000000000000e+00);

/* Compute outputs: */
out[0] = xd[0];
out[1] = xd[1];
out[2] = xd[2];
out[3] = xd[3];
out[4] = xd[4];
out[5] = xd[5];
out[6] = a[7];
out[7] = a[12];
out[8] = a[4];
out[9] = a[13];
out[10] = u[0];
out[11] = u[1];
out[12] = u[2];
out[13] = u[3];
out[14] = (real_t)(1.0000000000000000e+00);
out[15] = (real_t)(0.0000000000000000e+00);
out[16] = (real_t)(0.0000000000000000e+00);
out[17] = (real_t)(0.0000000000000000e+00);
out[18] = (real_t)(0.0000000000000000e+00);
out[19] = (real_t)(0.0000000000000000e+00);
out[20] = (real_t)(0.0000000000000000e+00);
out[21] = (real_t)(0.0000000000000000e+00);
out[22] = (real_t)(0.0000000000000000e+00);
out[23] = (real_t)(0.0000000000000000e+00);
out[24] = (real_t)(0.0000000000000000e+00);
out[25] = (real_t)(0.0000000000000000e+00);
out[26] = (real_t)(0.0000000000000000e+00);
out[27] = (real_t)(1.0000000000000000e+00);
out[28] = (real_t)(0.0000000000000000e+00);
out[29] = (real_t)(0.0000000000000000e+00);
out[30] = (real_t)(0.0000000000000000e+00);
out[31] = (real_t)(0.0000000000000000e+00);
out[32] = (real_t)(0.0000000000000000e+00);
out[33] = (real_t)(0.0000000000000000e+00);
out[34] = (real_t)(0.0000000000000000e+00);
out[35] = (real_t)(0.0000000000000000e+00);
out[36] = (real_t)(0.0000000000000000e+00);
out[37] = (real_t)(0.0000000000000000e+00);
out[38] = (real_t)(0.0000000000000000e+00);
out[39] = (real_t)(0.0000000000000000e+00);
out[40] = (real_t)(1.0000000000000000e+00);
out[41] = (real_t)(0.0000000000000000e+00);
out[42] = (real_t)(0.0000000000000000e+00);
out[43] = (real_t)(0.0000000000000000e+00);
out[44] = (real_t)(0.0000000000000000e+00);
out[45] = (real_t)(0.0000000000000000e+00);
out[46] = (real_t)(0.0000000000000000e+00);
out[47] = (real_t)(0.0000000000000000e+00);
out[48] = (real_t)(0.0000000000000000e+00);
out[49] = (real_t)(0.0000000000000000e+00);
out[50] = (real_t)(0.0000000000000000e+00);
out[51] = (real_t)(0.0000000000000000e+00);
out[52] = (real_t)(0.0000000000000000e+00);
out[53] = (real_t)(1.0000000000000000e+00);
out[54] = (real_t)(0.0000000000000000e+00);
out[55] = (real_t)(0.0000000000000000e+00);
out[56] = (real_t)(0.0000000000000000e+00);
out[57] = (real_t)(0.0000000000000000e+00);
out[58] = (real_t)(0.0000000000000000e+00);
out[59] = (real_t)(0.0000000000000000e+00);
out[60] = (real_t)(0.0000000000000000e+00);
out[61] = (real_t)(0.0000000000000000e+00);
out[62] = (real_t)(0.0000000000000000e+00);
out[63] = (real_t)(0.0000000000000000e+00);
out[64] = (real_t)(0.0000000000000000e+00);
out[65] = (real_t)(0.0000000000000000e+00);
out[66] = (real_t)(1.0000000000000000e+00);
out[67] = (real_t)(0.0000000000000000e+00);
out[68] = (real_t)(0.0000000000000000e+00);
out[69] = (real_t)(0.0000000000000000e+00);
out[70] = (real_t)(0.0000000000000000e+00);
out[71] = (real_t)(0.0000000000000000e+00);
out[72] = (real_t)(0.0000000000000000e+00);
out[73] = (real_t)(0.0000000000000000e+00);
out[74] = (real_t)(0.0000000000000000e+00);
out[75] = (real_t)(0.0000000000000000e+00);
out[76] = (real_t)(0.0000000000000000e+00);
out[77] = (real_t)(0.0000000000000000e+00);
out[78] = (real_t)(0.0000000000000000e+00);
out[79] = (real_t)(1.0000000000000000e+00);
out[80] = (real_t)(0.0000000000000000e+00);
out[81] = (real_t)(0.0000000000000000e+00);
out[82] = (real_t)(0.0000000000000000e+00);
out[83] = (real_t)(0.0000000000000000e+00);
out[84] = (real_t)(0.0000000000000000e+00);
out[85] = (real_t)(0.0000000000000000e+00);
out[86] = a[20];
out[87] = a[23];
out[88] = a[26];
out[89] = a[27];
out[90] = a[28];
out[91] = a[29];
out[92] = a[30];
out[93] = a[31];
out[94] = a[32];
out[95] = a[33];
out[96] = a[34];
out[97] = a[35];
out[98] = a[40];
out[99] = a[43];
out[100] = a[46];
out[101] = a[47];
out[102] = a[48];
out[103] = a[49];
out[104] = a[50];
out[105] = a[51];
out[106] = a[52];
out[107] = a[53];
out[108] = a[54];
out[109] = a[55];
out[110] = a[57];
out[111] = a[59];
out[112] = a[61];
out[113] = a[62];
out[114] = a[63];
out[115] = a[64];
out[116] = a[65];
out[117] = a[66];
out[118] = a[67];
out[119] = a[68];
out[120] = a[69];
out[121] = a[70];
out[122] = a[72];
out[123] = a[74];
out[124] = a[76];
out[125] = a[77];
out[126] = a[78];
out[127] = a[79];
out[128] = a[80];
out[129] = a[81];
out[130] = a[82];
out[131] = a[83];
out[132] = a[84];
out[133] = a[85];
out[134] = (real_t)(0.0000000000000000e+00);
out[135] = (real_t)(0.0000000000000000e+00);
out[136] = (real_t)(0.0000000000000000e+00);
out[137] = (real_t)(0.0000000000000000e+00);
out[138] = (real_t)(0.0000000000000000e+00);
out[139] = (real_t)(0.0000000000000000e+00);
out[140] = (real_t)(0.0000000000000000e+00);
out[141] = (real_t)(0.0000000000000000e+00);
out[142] = (real_t)(0.0000000000000000e+00);
out[143] = (real_t)(0.0000000000000000e+00);
out[144] = (real_t)(0.0000000000000000e+00);
out[145] = (real_t)(0.0000000000000000e+00);
out[146] = (real_t)(0.0000000000000000e+00);
out[147] = (real_t)(0.0000000000000000e+00);
out[148] = (real_t)(0.0000000000000000e+00);
out[149] = (real_t)(0.0000000000000000e+00);
out[150] = (real_t)(0.0000000000000000e+00);
out[151] = (real_t)(0.0000000000000000e+00);
out[152] = (real_t)(0.0000000000000000e+00);
out[153] = (real_t)(0.0000000000000000e+00);
out[154] = (real_t)(0.0000000000000000e+00);
out[155] = (real_t)(0.0000000000000000e+00);
out[156] = (real_t)(0.0000000000000000e+00);
out[157] = (real_t)(0.0000000000000000e+00);
out[158] = (real_t)(0.0000000000000000e+00);
out[159] = (real_t)(0.0000000000000000e+00);
out[160] = (real_t)(0.0000000000000000e+00);
out[161] = (real_t)(0.0000000000000000e+00);
out[162] = (real_t)(0.0000000000000000e+00);
out[163] = (real_t)(0.0000000000000000e+00);
out[164] = (real_t)(0.0000000000000000e+00);
out[165] = (real_t)(0.0000000000000000e+00);
out[166] = (real_t)(0.0000000000000000e+00);
out[167] = (real_t)(0.0000000000000000e+00);
out[168] = (real_t)(0.0000000000000000e+00);
out[169] = (real_t)(0.0000000000000000e+00);
out[170] = (real_t)(0.0000000000000000e+00);
out[171] = (real_t)(0.0000000000000000e+00);
out[172] = (real_t)(0.0000000000000000e+00);
out[173] = (real_t)(0.0000000000000000e+00);
out[174] = (real_t)(0.0000000000000000e+00);
out[175] = (real_t)(0.0000000000000000e+00);
out[176] = (real_t)(0.0000000000000000e+00);
out[177] = (real_t)(0.0000000000000000e+00);
out[178] = (real_t)(0.0000000000000000e+00);
out[179] = (real_t)(0.0000000000000000e+00);
out[180] = (real_t)(0.0000000000000000e+00);
out[181] = (real_t)(0.0000000000000000e+00);
out[182] = (real_t)(0.0000000000000000e+00);
out[183] = (real_t)(0.0000000000000000e+00);
out[184] = (real_t)(0.0000000000000000e+00);
out[185] = (real_t)(0.0000000000000000e+00);
out[186] = (real_t)(0.0000000000000000e+00);
out[187] = (real_t)(0.0000000000000000e+00);
out[188] = (real_t)(0.0000000000000000e+00);
out[189] = (real_t)(0.0000000000000000e+00);
out[190] = (real_t)(0.0000000000000000e+00);
out[191] = (real_t)(0.0000000000000000e+00);
out[192] = (real_t)(0.0000000000000000e+00);
out[193] = (real_t)(0.0000000000000000e+00);
out[194] = (real_t)(0.0000000000000000e+00);
out[195] = (real_t)(0.0000000000000000e+00);
out[196] = (real_t)(0.0000000000000000e+00);
out[197] = (real_t)(0.0000000000000000e+00);
out[198] = (real_t)(0.0000000000000000e+00);
out[199] = (real_t)(0.0000000000000000e+00);
out[200] = (real_t)(0.0000000000000000e+00);
out[201] = (real_t)(0.0000000000000000e+00);
out[202] = (real_t)(0.0000000000000000e+00);
out[203] = (real_t)(0.0000000000000000e+00);
out[204] = (real_t)(0.0000000000000000e+00);
out[205] = (real_t)(0.0000000000000000e+00);
out[206] = a[86];
out[207] = a[87];
out[208] = a[90];
out[209] = a[91];
out[210] = a[92];
out[211] = a[93];
out[212] = a[98];
out[213] = a[99];
out[214] = a[100];
out[215] = a[101];
out[216] = a[102];
out[217] = a[103];
out[218] = a[104];
out[219] = a[105];
out[220] = a[106];
out[221] = a[107];
out[222] = (real_t)(1.0000000000000000e+00);
out[223] = (real_t)(0.0000000000000000e+00);
out[224] = (real_t)(0.0000000000000000e+00);
out[225] = (real_t)(0.0000000000000000e+00);
out[226] = (real_t)(0.0000000000000000e+00);
out[227] = (real_t)(1.0000000000000000e+00);
out[228] = (real_t)(0.0000000000000000e+00);
out[229] = (real_t)(0.0000000000000000e+00);
out[230] = (real_t)(0.0000000000000000e+00);
out[231] = (real_t)(0.0000000000000000e+00);
out[232] = (real_t)(1.0000000000000000e+00);
out[233] = (real_t)(0.0000000000000000e+00);
out[234] = (real_t)(0.0000000000000000e+00);
out[235] = (real_t)(0.0000000000000000e+00);
out[236] = (real_t)(0.0000000000000000e+00);
out[237] = (real_t)(1.0000000000000000e+00);
}

void nmpc_evaluateLSQEndTerm(const real_t* in, real_t* out)
{
const real_t* xd = in;

/* Compute outputs: */
out[0] = xd[0];
out[1] = xd[1];
out[2] = xd[2];
out[3] = xd[3];
out[4] = xd[4];
out[5] = xd[5];
}

void nmpc_setObjQ1Q2( real_t* const tmpFx, real_t* const tmpObjS, real_t* const tmpQ1, real_t* const tmpQ2 )
{
tmpQ2[0] = + tmpFx[0]*tmpObjS[0] + tmpFx[12]*tmpObjS[14] + tmpFx[24]*tmpObjS[28] + tmpFx[36]*tmpObjS[42] + tmpFx[48]*tmpObjS[56] + tmpFx[60]*tmpObjS[70] + tmpFx[72]*tmpObjS[84] + tmpFx[84]*tmpObjS[98] + tmpFx[96]*tmpObjS[112] + tmpFx[108]*tmpObjS[126] + tmpFx[120]*tmpObjS[140] + tmpFx[132]*tmpObjS[154] + tmpFx[144]*tmpObjS[168] + tmpFx[156]*tmpObjS[182];
tmpQ2[1] = + tmpFx[0]*tmpObjS[1] + tmpFx[12]*tmpObjS[15] + tmpFx[24]*tmpObjS[29] + tmpFx[36]*tmpObjS[43] + tmpFx[48]*tmpObjS[57] + tmpFx[60]*tmpObjS[71] + tmpFx[72]*tmpObjS[85] + tmpFx[84]*tmpObjS[99] + tmpFx[96]*tmpObjS[113] + tmpFx[108]*tmpObjS[127] + tmpFx[120]*tmpObjS[141] + tmpFx[132]*tmpObjS[155] + tmpFx[144]*tmpObjS[169] + tmpFx[156]*tmpObjS[183];
tmpQ2[2] = + tmpFx[0]*tmpObjS[2] + tmpFx[12]*tmpObjS[16] + tmpFx[24]*tmpObjS[30] + tmpFx[36]*tmpObjS[44] + tmpFx[48]*tmpObjS[58] + tmpFx[60]*tmpObjS[72] + tmpFx[72]*tmpObjS[86] + tmpFx[84]*tmpObjS[100] + tmpFx[96]*tmpObjS[114] + tmpFx[108]*tmpObjS[128] + tmpFx[120]*tmpObjS[142] + tmpFx[132]*tmpObjS[156] + tmpFx[144]*tmpObjS[170] + tmpFx[156]*tmpObjS[184];
tmpQ2[3] = + tmpFx[0]*tmpObjS[3] + tmpFx[12]*tmpObjS[17] + tmpFx[24]*tmpObjS[31] + tmpFx[36]*tmpObjS[45] + tmpFx[48]*tmpObjS[59] + tmpFx[60]*tmpObjS[73] + tmpFx[72]*tmpObjS[87] + tmpFx[84]*tmpObjS[101] + tmpFx[96]*tmpObjS[115] + tmpFx[108]*tmpObjS[129] + tmpFx[120]*tmpObjS[143] + tmpFx[132]*tmpObjS[157] + tmpFx[144]*tmpObjS[171] + tmpFx[156]*tmpObjS[185];
tmpQ2[4] = + tmpFx[0]*tmpObjS[4] + tmpFx[12]*tmpObjS[18] + tmpFx[24]*tmpObjS[32] + tmpFx[36]*tmpObjS[46] + tmpFx[48]*tmpObjS[60] + tmpFx[60]*tmpObjS[74] + tmpFx[72]*tmpObjS[88] + tmpFx[84]*tmpObjS[102] + tmpFx[96]*tmpObjS[116] + tmpFx[108]*tmpObjS[130] + tmpFx[120]*tmpObjS[144] + tmpFx[132]*tmpObjS[158] + tmpFx[144]*tmpObjS[172] + tmpFx[156]*tmpObjS[186];
tmpQ2[5] = + tmpFx[0]*tmpObjS[5] + tmpFx[12]*tmpObjS[19] + tmpFx[24]*tmpObjS[33] + tmpFx[36]*tmpObjS[47] + tmpFx[48]*tmpObjS[61] + tmpFx[60]*tmpObjS[75] + tmpFx[72]*tmpObjS[89] + tmpFx[84]*tmpObjS[103] + tmpFx[96]*tmpObjS[117] + tmpFx[108]*tmpObjS[131] + tmpFx[120]*tmpObjS[145] + tmpFx[132]*tmpObjS[159] + tmpFx[144]*tmpObjS[173] + tmpFx[156]*tmpObjS[187];
tmpQ2[6] = + tmpFx[0]*tmpObjS[6] + tmpFx[12]*tmpObjS[20] + tmpFx[24]*tmpObjS[34] + tmpFx[36]*tmpObjS[48] + tmpFx[48]*tmpObjS[62] + tmpFx[60]*tmpObjS[76] + tmpFx[72]*tmpObjS[90] + tmpFx[84]*tmpObjS[104] + tmpFx[96]*tmpObjS[118] + tmpFx[108]*tmpObjS[132] + tmpFx[120]*tmpObjS[146] + tmpFx[132]*tmpObjS[160] + tmpFx[144]*tmpObjS[174] + tmpFx[156]*tmpObjS[188];
tmpQ2[7] = + tmpFx[0]*tmpObjS[7] + tmpFx[12]*tmpObjS[21] + tmpFx[24]*tmpObjS[35] + tmpFx[36]*tmpObjS[49] + tmpFx[48]*tmpObjS[63] + tmpFx[60]*tmpObjS[77] + tmpFx[72]*tmpObjS[91] + tmpFx[84]*tmpObjS[105] + tmpFx[96]*tmpObjS[119] + tmpFx[108]*tmpObjS[133] + tmpFx[120]*tmpObjS[147] + tmpFx[132]*tmpObjS[161] + tmpFx[144]*tmpObjS[175] + tmpFx[156]*tmpObjS[189];
tmpQ2[8] = + tmpFx[0]*tmpObjS[8] + tmpFx[12]*tmpObjS[22] + tmpFx[24]*tmpObjS[36] + tmpFx[36]*tmpObjS[50] + tmpFx[48]*tmpObjS[64] + tmpFx[60]*tmpObjS[78] + tmpFx[72]*tmpObjS[92] + tmpFx[84]*tmpObjS[106] + tmpFx[96]*tmpObjS[120] + tmpFx[108]*tmpObjS[134] + tmpFx[120]*tmpObjS[148] + tmpFx[132]*tmpObjS[162] + tmpFx[144]*tmpObjS[176] + tmpFx[156]*tmpObjS[190];
tmpQ2[9] = + tmpFx[0]*tmpObjS[9] + tmpFx[12]*tmpObjS[23] + tmpFx[24]*tmpObjS[37] + tmpFx[36]*tmpObjS[51] + tmpFx[48]*tmpObjS[65] + tmpFx[60]*tmpObjS[79] + tmpFx[72]*tmpObjS[93] + tmpFx[84]*tmpObjS[107] + tmpFx[96]*tmpObjS[121] + tmpFx[108]*tmpObjS[135] + tmpFx[120]*tmpObjS[149] + tmpFx[132]*tmpObjS[163] + tmpFx[144]*tmpObjS[177] + tmpFx[156]*tmpObjS[191];
tmpQ2[10] = + tmpFx[0]*tmpObjS[10] + tmpFx[12]*tmpObjS[24] + tmpFx[24]*tmpObjS[38] + tmpFx[36]*tmpObjS[52] + tmpFx[48]*tmpObjS[66] + tmpFx[60]*tmpObjS[80] + tmpFx[72]*tmpObjS[94] + tmpFx[84]*tmpObjS[108] + tmpFx[96]*tmpObjS[122] + tmpFx[108]*tmpObjS[136] + tmpFx[120]*tmpObjS[150] + tmpFx[132]*tmpObjS[164] + tmpFx[144]*tmpObjS[178] + tmpFx[156]*tmpObjS[192];
tmpQ2[11] = + tmpFx[0]*tmpObjS[11] + tmpFx[12]*tmpObjS[25] + tmpFx[24]*tmpObjS[39] + tmpFx[36]*tmpObjS[53] + tmpFx[48]*tmpObjS[67] + tmpFx[60]*tmpObjS[81] + tmpFx[72]*tmpObjS[95] + tmpFx[84]*tmpObjS[109] + tmpFx[96]*tmpObjS[123] + tmpFx[108]*tmpObjS[137] + tmpFx[120]*tmpObjS[151] + tmpFx[132]*tmpObjS[165] + tmpFx[144]*tmpObjS[179] + tmpFx[156]*tmpObjS[193];
tmpQ2[12] = + tmpFx[0]*tmpObjS[12] + tmpFx[12]*tmpObjS[26] + tmpFx[24]*tmpObjS[40] + tmpFx[36]*tmpObjS[54] + tmpFx[48]*tmpObjS[68] + tmpFx[60]*tmpObjS[82] + tmpFx[72]*tmpObjS[96] + tmpFx[84]*tmpObjS[110] + tmpFx[96]*tmpObjS[124] + tmpFx[108]*tmpObjS[138] + tmpFx[120]*tmpObjS[152] + tmpFx[132]*tmpObjS[166] + tmpFx[144]*tmpObjS[180] + tmpFx[156]*tmpObjS[194];
tmpQ2[13] = + tmpFx[0]*tmpObjS[13] + tmpFx[12]*tmpObjS[27] + tmpFx[24]*tmpObjS[41] + tmpFx[36]*tmpObjS[55] + tmpFx[48]*tmpObjS[69] + tmpFx[60]*tmpObjS[83] + tmpFx[72]*tmpObjS[97] + tmpFx[84]*tmpObjS[111] + tmpFx[96]*tmpObjS[125] + tmpFx[108]*tmpObjS[139] + tmpFx[120]*tmpObjS[153] + tmpFx[132]*tmpObjS[167] + tmpFx[144]*tmpObjS[181] + tmpFx[156]*tmpObjS[195];
tmpQ2[14] = + tmpFx[1]*tmpObjS[0] + tmpFx[13]*tmpObjS[14] + tmpFx[25]*tmpObjS[28] + tmpFx[37]*tmpObjS[42] + tmpFx[49]*tmpObjS[56] + tmpFx[61]*tmpObjS[70] + tmpFx[73]*tmpObjS[84] + tmpFx[85]*tmpObjS[98] + tmpFx[97]*tmpObjS[112] + tmpFx[109]*tmpObjS[126] + tmpFx[121]*tmpObjS[140] + tmpFx[133]*tmpObjS[154] + tmpFx[145]*tmpObjS[168] + tmpFx[157]*tmpObjS[182];
tmpQ2[15] = + tmpFx[1]*tmpObjS[1] + tmpFx[13]*tmpObjS[15] + tmpFx[25]*tmpObjS[29] + tmpFx[37]*tmpObjS[43] + tmpFx[49]*tmpObjS[57] + tmpFx[61]*tmpObjS[71] + tmpFx[73]*tmpObjS[85] + tmpFx[85]*tmpObjS[99] + tmpFx[97]*tmpObjS[113] + tmpFx[109]*tmpObjS[127] + tmpFx[121]*tmpObjS[141] + tmpFx[133]*tmpObjS[155] + tmpFx[145]*tmpObjS[169] + tmpFx[157]*tmpObjS[183];
tmpQ2[16] = + tmpFx[1]*tmpObjS[2] + tmpFx[13]*tmpObjS[16] + tmpFx[25]*tmpObjS[30] + tmpFx[37]*tmpObjS[44] + tmpFx[49]*tmpObjS[58] + tmpFx[61]*tmpObjS[72] + tmpFx[73]*tmpObjS[86] + tmpFx[85]*tmpObjS[100] + tmpFx[97]*tmpObjS[114] + tmpFx[109]*tmpObjS[128] + tmpFx[121]*tmpObjS[142] + tmpFx[133]*tmpObjS[156] + tmpFx[145]*tmpObjS[170] + tmpFx[157]*tmpObjS[184];
tmpQ2[17] = + tmpFx[1]*tmpObjS[3] + tmpFx[13]*tmpObjS[17] + tmpFx[25]*tmpObjS[31] + tmpFx[37]*tmpObjS[45] + tmpFx[49]*tmpObjS[59] + tmpFx[61]*tmpObjS[73] + tmpFx[73]*tmpObjS[87] + tmpFx[85]*tmpObjS[101] + tmpFx[97]*tmpObjS[115] + tmpFx[109]*tmpObjS[129] + tmpFx[121]*tmpObjS[143] + tmpFx[133]*tmpObjS[157] + tmpFx[145]*tmpObjS[171] + tmpFx[157]*tmpObjS[185];
tmpQ2[18] = + tmpFx[1]*tmpObjS[4] + tmpFx[13]*tmpObjS[18] + tmpFx[25]*tmpObjS[32] + tmpFx[37]*tmpObjS[46] + tmpFx[49]*tmpObjS[60] + tmpFx[61]*tmpObjS[74] + tmpFx[73]*tmpObjS[88] + tmpFx[85]*tmpObjS[102] + tmpFx[97]*tmpObjS[116] + tmpFx[109]*tmpObjS[130] + tmpFx[121]*tmpObjS[144] + tmpFx[133]*tmpObjS[158] + tmpFx[145]*tmpObjS[172] + tmpFx[157]*tmpObjS[186];
tmpQ2[19] = + tmpFx[1]*tmpObjS[5] + tmpFx[13]*tmpObjS[19] + tmpFx[25]*tmpObjS[33] + tmpFx[37]*tmpObjS[47] + tmpFx[49]*tmpObjS[61] + tmpFx[61]*tmpObjS[75] + tmpFx[73]*tmpObjS[89] + tmpFx[85]*tmpObjS[103] + tmpFx[97]*tmpObjS[117] + tmpFx[109]*tmpObjS[131] + tmpFx[121]*tmpObjS[145] + tmpFx[133]*tmpObjS[159] + tmpFx[145]*tmpObjS[173] + tmpFx[157]*tmpObjS[187];
tmpQ2[20] = + tmpFx[1]*tmpObjS[6] + tmpFx[13]*tmpObjS[20] + tmpFx[25]*tmpObjS[34] + tmpFx[37]*tmpObjS[48] + tmpFx[49]*tmpObjS[62] + tmpFx[61]*tmpObjS[76] + tmpFx[73]*tmpObjS[90] + tmpFx[85]*tmpObjS[104] + tmpFx[97]*tmpObjS[118] + tmpFx[109]*tmpObjS[132] + tmpFx[121]*tmpObjS[146] + tmpFx[133]*tmpObjS[160] + tmpFx[145]*tmpObjS[174] + tmpFx[157]*tmpObjS[188];
tmpQ2[21] = + tmpFx[1]*tmpObjS[7] + tmpFx[13]*tmpObjS[21] + tmpFx[25]*tmpObjS[35] + tmpFx[37]*tmpObjS[49] + tmpFx[49]*tmpObjS[63] + tmpFx[61]*tmpObjS[77] + tmpFx[73]*tmpObjS[91] + tmpFx[85]*tmpObjS[105] + tmpFx[97]*tmpObjS[119] + tmpFx[109]*tmpObjS[133] + tmpFx[121]*tmpObjS[147] + tmpFx[133]*tmpObjS[161] + tmpFx[145]*tmpObjS[175] + tmpFx[157]*tmpObjS[189];
tmpQ2[22] = + tmpFx[1]*tmpObjS[8] + tmpFx[13]*tmpObjS[22] + tmpFx[25]*tmpObjS[36] + tmpFx[37]*tmpObjS[50] + tmpFx[49]*tmpObjS[64] + tmpFx[61]*tmpObjS[78] + tmpFx[73]*tmpObjS[92] + tmpFx[85]*tmpObjS[106] + tmpFx[97]*tmpObjS[120] + tmpFx[109]*tmpObjS[134] + tmpFx[121]*tmpObjS[148] + tmpFx[133]*tmpObjS[162] + tmpFx[145]*tmpObjS[176] + tmpFx[157]*tmpObjS[190];
tmpQ2[23] = + tmpFx[1]*tmpObjS[9] + tmpFx[13]*tmpObjS[23] + tmpFx[25]*tmpObjS[37] + tmpFx[37]*tmpObjS[51] + tmpFx[49]*tmpObjS[65] + tmpFx[61]*tmpObjS[79] + tmpFx[73]*tmpObjS[93] + tmpFx[85]*tmpObjS[107] + tmpFx[97]*tmpObjS[121] + tmpFx[109]*tmpObjS[135] + tmpFx[121]*tmpObjS[149] + tmpFx[133]*tmpObjS[163] + tmpFx[145]*tmpObjS[177] + tmpFx[157]*tmpObjS[191];
tmpQ2[24] = + tmpFx[1]*tmpObjS[10] + tmpFx[13]*tmpObjS[24] + tmpFx[25]*tmpObjS[38] + tmpFx[37]*tmpObjS[52] + tmpFx[49]*tmpObjS[66] + tmpFx[61]*tmpObjS[80] + tmpFx[73]*tmpObjS[94] + tmpFx[85]*tmpObjS[108] + tmpFx[97]*tmpObjS[122] + tmpFx[109]*tmpObjS[136] + tmpFx[121]*tmpObjS[150] + tmpFx[133]*tmpObjS[164] + tmpFx[145]*tmpObjS[178] + tmpFx[157]*tmpObjS[192];
tmpQ2[25] = + tmpFx[1]*tmpObjS[11] + tmpFx[13]*tmpObjS[25] + tmpFx[25]*tmpObjS[39] + tmpFx[37]*tmpObjS[53] + tmpFx[49]*tmpObjS[67] + tmpFx[61]*tmpObjS[81] + tmpFx[73]*tmpObjS[95] + tmpFx[85]*tmpObjS[109] + tmpFx[97]*tmpObjS[123] + tmpFx[109]*tmpObjS[137] + tmpFx[121]*tmpObjS[151] + tmpFx[133]*tmpObjS[165] + tmpFx[145]*tmpObjS[179] + tmpFx[157]*tmpObjS[193];
tmpQ2[26] = + tmpFx[1]*tmpObjS[12] + tmpFx[13]*tmpObjS[26] + tmpFx[25]*tmpObjS[40] + tmpFx[37]*tmpObjS[54] + tmpFx[49]*tmpObjS[68] + tmpFx[61]*tmpObjS[82] + tmpFx[73]*tmpObjS[96] + tmpFx[85]*tmpObjS[110] + tmpFx[97]*tmpObjS[124] + tmpFx[109]*tmpObjS[138] + tmpFx[121]*tmpObjS[152] + tmpFx[133]*tmpObjS[166] + tmpFx[145]*tmpObjS[180] + tmpFx[157]*tmpObjS[194];
tmpQ2[27] = + tmpFx[1]*tmpObjS[13] + tmpFx[13]*tmpObjS[27] + tmpFx[25]*tmpObjS[41] + tmpFx[37]*tmpObjS[55] + tmpFx[49]*tmpObjS[69] + tmpFx[61]*tmpObjS[83] + tmpFx[73]*tmpObjS[97] + tmpFx[85]*tmpObjS[111] + tmpFx[97]*tmpObjS[125] + tmpFx[109]*tmpObjS[139] + tmpFx[121]*tmpObjS[153] + tmpFx[133]*tmpObjS[167] + tmpFx[145]*tmpObjS[181] + tmpFx[157]*tmpObjS[195];
tmpQ2[28] = + tmpFx[2]*tmpObjS[0] + tmpFx[14]*tmpObjS[14] + tmpFx[26]*tmpObjS[28] + tmpFx[38]*tmpObjS[42] + tmpFx[50]*tmpObjS[56] + tmpFx[62]*tmpObjS[70] + tmpFx[74]*tmpObjS[84] + tmpFx[86]*tmpObjS[98] + tmpFx[98]*tmpObjS[112] + tmpFx[110]*tmpObjS[126] + tmpFx[122]*tmpObjS[140] + tmpFx[134]*tmpObjS[154] + tmpFx[146]*tmpObjS[168] + tmpFx[158]*tmpObjS[182];
tmpQ2[29] = + tmpFx[2]*tmpObjS[1] + tmpFx[14]*tmpObjS[15] + tmpFx[26]*tmpObjS[29] + tmpFx[38]*tmpObjS[43] + tmpFx[50]*tmpObjS[57] + tmpFx[62]*tmpObjS[71] + tmpFx[74]*tmpObjS[85] + tmpFx[86]*tmpObjS[99] + tmpFx[98]*tmpObjS[113] + tmpFx[110]*tmpObjS[127] + tmpFx[122]*tmpObjS[141] + tmpFx[134]*tmpObjS[155] + tmpFx[146]*tmpObjS[169] + tmpFx[158]*tmpObjS[183];
tmpQ2[30] = + tmpFx[2]*tmpObjS[2] + tmpFx[14]*tmpObjS[16] + tmpFx[26]*tmpObjS[30] + tmpFx[38]*tmpObjS[44] + tmpFx[50]*tmpObjS[58] + tmpFx[62]*tmpObjS[72] + tmpFx[74]*tmpObjS[86] + tmpFx[86]*tmpObjS[100] + tmpFx[98]*tmpObjS[114] + tmpFx[110]*tmpObjS[128] + tmpFx[122]*tmpObjS[142] + tmpFx[134]*tmpObjS[156] + tmpFx[146]*tmpObjS[170] + tmpFx[158]*tmpObjS[184];
tmpQ2[31] = + tmpFx[2]*tmpObjS[3] + tmpFx[14]*tmpObjS[17] + tmpFx[26]*tmpObjS[31] + tmpFx[38]*tmpObjS[45] + tmpFx[50]*tmpObjS[59] + tmpFx[62]*tmpObjS[73] + tmpFx[74]*tmpObjS[87] + tmpFx[86]*tmpObjS[101] + tmpFx[98]*tmpObjS[115] + tmpFx[110]*tmpObjS[129] + tmpFx[122]*tmpObjS[143] + tmpFx[134]*tmpObjS[157] + tmpFx[146]*tmpObjS[171] + tmpFx[158]*tmpObjS[185];
tmpQ2[32] = + tmpFx[2]*tmpObjS[4] + tmpFx[14]*tmpObjS[18] + tmpFx[26]*tmpObjS[32] + tmpFx[38]*tmpObjS[46] + tmpFx[50]*tmpObjS[60] + tmpFx[62]*tmpObjS[74] + tmpFx[74]*tmpObjS[88] + tmpFx[86]*tmpObjS[102] + tmpFx[98]*tmpObjS[116] + tmpFx[110]*tmpObjS[130] + tmpFx[122]*tmpObjS[144] + tmpFx[134]*tmpObjS[158] + tmpFx[146]*tmpObjS[172] + tmpFx[158]*tmpObjS[186];
tmpQ2[33] = + tmpFx[2]*tmpObjS[5] + tmpFx[14]*tmpObjS[19] + tmpFx[26]*tmpObjS[33] + tmpFx[38]*tmpObjS[47] + tmpFx[50]*tmpObjS[61] + tmpFx[62]*tmpObjS[75] + tmpFx[74]*tmpObjS[89] + tmpFx[86]*tmpObjS[103] + tmpFx[98]*tmpObjS[117] + tmpFx[110]*tmpObjS[131] + tmpFx[122]*tmpObjS[145] + tmpFx[134]*tmpObjS[159] + tmpFx[146]*tmpObjS[173] + tmpFx[158]*tmpObjS[187];
tmpQ2[34] = + tmpFx[2]*tmpObjS[6] + tmpFx[14]*tmpObjS[20] + tmpFx[26]*tmpObjS[34] + tmpFx[38]*tmpObjS[48] + tmpFx[50]*tmpObjS[62] + tmpFx[62]*tmpObjS[76] + tmpFx[74]*tmpObjS[90] + tmpFx[86]*tmpObjS[104] + tmpFx[98]*tmpObjS[118] + tmpFx[110]*tmpObjS[132] + tmpFx[122]*tmpObjS[146] + tmpFx[134]*tmpObjS[160] + tmpFx[146]*tmpObjS[174] + tmpFx[158]*tmpObjS[188];
tmpQ2[35] = + tmpFx[2]*tmpObjS[7] + tmpFx[14]*tmpObjS[21] + tmpFx[26]*tmpObjS[35] + tmpFx[38]*tmpObjS[49] + tmpFx[50]*tmpObjS[63] + tmpFx[62]*tmpObjS[77] + tmpFx[74]*tmpObjS[91] + tmpFx[86]*tmpObjS[105] + tmpFx[98]*tmpObjS[119] + tmpFx[110]*tmpObjS[133] + tmpFx[122]*tmpObjS[147] + tmpFx[134]*tmpObjS[161] + tmpFx[146]*tmpObjS[175] + tmpFx[158]*tmpObjS[189];
tmpQ2[36] = + tmpFx[2]*tmpObjS[8] + tmpFx[14]*tmpObjS[22] + tmpFx[26]*tmpObjS[36] + tmpFx[38]*tmpObjS[50] + tmpFx[50]*tmpObjS[64] + tmpFx[62]*tmpObjS[78] + tmpFx[74]*tmpObjS[92] + tmpFx[86]*tmpObjS[106] + tmpFx[98]*tmpObjS[120] + tmpFx[110]*tmpObjS[134] + tmpFx[122]*tmpObjS[148] + tmpFx[134]*tmpObjS[162] + tmpFx[146]*tmpObjS[176] + tmpFx[158]*tmpObjS[190];
tmpQ2[37] = + tmpFx[2]*tmpObjS[9] + tmpFx[14]*tmpObjS[23] + tmpFx[26]*tmpObjS[37] + tmpFx[38]*tmpObjS[51] + tmpFx[50]*tmpObjS[65] + tmpFx[62]*tmpObjS[79] + tmpFx[74]*tmpObjS[93] + tmpFx[86]*tmpObjS[107] + tmpFx[98]*tmpObjS[121] + tmpFx[110]*tmpObjS[135] + tmpFx[122]*tmpObjS[149] + tmpFx[134]*tmpObjS[163] + tmpFx[146]*tmpObjS[177] + tmpFx[158]*tmpObjS[191];
tmpQ2[38] = + tmpFx[2]*tmpObjS[10] + tmpFx[14]*tmpObjS[24] + tmpFx[26]*tmpObjS[38] + tmpFx[38]*tmpObjS[52] + tmpFx[50]*tmpObjS[66] + tmpFx[62]*tmpObjS[80] + tmpFx[74]*tmpObjS[94] + tmpFx[86]*tmpObjS[108] + tmpFx[98]*tmpObjS[122] + tmpFx[110]*tmpObjS[136] + tmpFx[122]*tmpObjS[150] + tmpFx[134]*tmpObjS[164] + tmpFx[146]*tmpObjS[178] + tmpFx[158]*tmpObjS[192];
tmpQ2[39] = + tmpFx[2]*tmpObjS[11] + tmpFx[14]*tmpObjS[25] + tmpFx[26]*tmpObjS[39] + tmpFx[38]*tmpObjS[53] + tmpFx[50]*tmpObjS[67] + tmpFx[62]*tmpObjS[81] + tmpFx[74]*tmpObjS[95] + tmpFx[86]*tmpObjS[109] + tmpFx[98]*tmpObjS[123] + tmpFx[110]*tmpObjS[137] + tmpFx[122]*tmpObjS[151] + tmpFx[134]*tmpObjS[165] + tmpFx[146]*tmpObjS[179] + tmpFx[158]*tmpObjS[193];
tmpQ2[40] = + tmpFx[2]*tmpObjS[12] + tmpFx[14]*tmpObjS[26] + tmpFx[26]*tmpObjS[40] + tmpFx[38]*tmpObjS[54] + tmpFx[50]*tmpObjS[68] + tmpFx[62]*tmpObjS[82] + tmpFx[74]*tmpObjS[96] + tmpFx[86]*tmpObjS[110] + tmpFx[98]*tmpObjS[124] + tmpFx[110]*tmpObjS[138] + tmpFx[122]*tmpObjS[152] + tmpFx[134]*tmpObjS[166] + tmpFx[146]*tmpObjS[180] + tmpFx[158]*tmpObjS[194];
tmpQ2[41] = + tmpFx[2]*tmpObjS[13] + tmpFx[14]*tmpObjS[27] + tmpFx[26]*tmpObjS[41] + tmpFx[38]*tmpObjS[55] + tmpFx[50]*tmpObjS[69] + tmpFx[62]*tmpObjS[83] + tmpFx[74]*tmpObjS[97] + tmpFx[86]*tmpObjS[111] + tmpFx[98]*tmpObjS[125] + tmpFx[110]*tmpObjS[139] + tmpFx[122]*tmpObjS[153] + tmpFx[134]*tmpObjS[167] + tmpFx[146]*tmpObjS[181] + tmpFx[158]*tmpObjS[195];
tmpQ2[42] = + tmpFx[3]*tmpObjS[0] + tmpFx[15]*tmpObjS[14] + tmpFx[27]*tmpObjS[28] + tmpFx[39]*tmpObjS[42] + tmpFx[51]*tmpObjS[56] + tmpFx[63]*tmpObjS[70] + tmpFx[75]*tmpObjS[84] + tmpFx[87]*tmpObjS[98] + tmpFx[99]*tmpObjS[112] + tmpFx[111]*tmpObjS[126] + tmpFx[123]*tmpObjS[140] + tmpFx[135]*tmpObjS[154] + tmpFx[147]*tmpObjS[168] + tmpFx[159]*tmpObjS[182];
tmpQ2[43] = + tmpFx[3]*tmpObjS[1] + tmpFx[15]*tmpObjS[15] + tmpFx[27]*tmpObjS[29] + tmpFx[39]*tmpObjS[43] + tmpFx[51]*tmpObjS[57] + tmpFx[63]*tmpObjS[71] + tmpFx[75]*tmpObjS[85] + tmpFx[87]*tmpObjS[99] + tmpFx[99]*tmpObjS[113] + tmpFx[111]*tmpObjS[127] + tmpFx[123]*tmpObjS[141] + tmpFx[135]*tmpObjS[155] + tmpFx[147]*tmpObjS[169] + tmpFx[159]*tmpObjS[183];
tmpQ2[44] = + tmpFx[3]*tmpObjS[2] + tmpFx[15]*tmpObjS[16] + tmpFx[27]*tmpObjS[30] + tmpFx[39]*tmpObjS[44] + tmpFx[51]*tmpObjS[58] + tmpFx[63]*tmpObjS[72] + tmpFx[75]*tmpObjS[86] + tmpFx[87]*tmpObjS[100] + tmpFx[99]*tmpObjS[114] + tmpFx[111]*tmpObjS[128] + tmpFx[123]*tmpObjS[142] + tmpFx[135]*tmpObjS[156] + tmpFx[147]*tmpObjS[170] + tmpFx[159]*tmpObjS[184];
tmpQ2[45] = + tmpFx[3]*tmpObjS[3] + tmpFx[15]*tmpObjS[17] + tmpFx[27]*tmpObjS[31] + tmpFx[39]*tmpObjS[45] + tmpFx[51]*tmpObjS[59] + tmpFx[63]*tmpObjS[73] + tmpFx[75]*tmpObjS[87] + tmpFx[87]*tmpObjS[101] + tmpFx[99]*tmpObjS[115] + tmpFx[111]*tmpObjS[129] + tmpFx[123]*tmpObjS[143] + tmpFx[135]*tmpObjS[157] + tmpFx[147]*tmpObjS[171] + tmpFx[159]*tmpObjS[185];
tmpQ2[46] = + tmpFx[3]*tmpObjS[4] + tmpFx[15]*tmpObjS[18] + tmpFx[27]*tmpObjS[32] + tmpFx[39]*tmpObjS[46] + tmpFx[51]*tmpObjS[60] + tmpFx[63]*tmpObjS[74] + tmpFx[75]*tmpObjS[88] + tmpFx[87]*tmpObjS[102] + tmpFx[99]*tmpObjS[116] + tmpFx[111]*tmpObjS[130] + tmpFx[123]*tmpObjS[144] + tmpFx[135]*tmpObjS[158] + tmpFx[147]*tmpObjS[172] + tmpFx[159]*tmpObjS[186];
tmpQ2[47] = + tmpFx[3]*tmpObjS[5] + tmpFx[15]*tmpObjS[19] + tmpFx[27]*tmpObjS[33] + tmpFx[39]*tmpObjS[47] + tmpFx[51]*tmpObjS[61] + tmpFx[63]*tmpObjS[75] + tmpFx[75]*tmpObjS[89] + tmpFx[87]*tmpObjS[103] + tmpFx[99]*tmpObjS[117] + tmpFx[111]*tmpObjS[131] + tmpFx[123]*tmpObjS[145] + tmpFx[135]*tmpObjS[159] + tmpFx[147]*tmpObjS[173] + tmpFx[159]*tmpObjS[187];
tmpQ2[48] = + tmpFx[3]*tmpObjS[6] + tmpFx[15]*tmpObjS[20] + tmpFx[27]*tmpObjS[34] + tmpFx[39]*tmpObjS[48] + tmpFx[51]*tmpObjS[62] + tmpFx[63]*tmpObjS[76] + tmpFx[75]*tmpObjS[90] + tmpFx[87]*tmpObjS[104] + tmpFx[99]*tmpObjS[118] + tmpFx[111]*tmpObjS[132] + tmpFx[123]*tmpObjS[146] + tmpFx[135]*tmpObjS[160] + tmpFx[147]*tmpObjS[174] + tmpFx[159]*tmpObjS[188];
tmpQ2[49] = + tmpFx[3]*tmpObjS[7] + tmpFx[15]*tmpObjS[21] + tmpFx[27]*tmpObjS[35] + tmpFx[39]*tmpObjS[49] + tmpFx[51]*tmpObjS[63] + tmpFx[63]*tmpObjS[77] + tmpFx[75]*tmpObjS[91] + tmpFx[87]*tmpObjS[105] + tmpFx[99]*tmpObjS[119] + tmpFx[111]*tmpObjS[133] + tmpFx[123]*tmpObjS[147] + tmpFx[135]*tmpObjS[161] + tmpFx[147]*tmpObjS[175] + tmpFx[159]*tmpObjS[189];
tmpQ2[50] = + tmpFx[3]*tmpObjS[8] + tmpFx[15]*tmpObjS[22] + tmpFx[27]*tmpObjS[36] + tmpFx[39]*tmpObjS[50] + tmpFx[51]*tmpObjS[64] + tmpFx[63]*tmpObjS[78] + tmpFx[75]*tmpObjS[92] + tmpFx[87]*tmpObjS[106] + tmpFx[99]*tmpObjS[120] + tmpFx[111]*tmpObjS[134] + tmpFx[123]*tmpObjS[148] + tmpFx[135]*tmpObjS[162] + tmpFx[147]*tmpObjS[176] + tmpFx[159]*tmpObjS[190];
tmpQ2[51] = + tmpFx[3]*tmpObjS[9] + tmpFx[15]*tmpObjS[23] + tmpFx[27]*tmpObjS[37] + tmpFx[39]*tmpObjS[51] + tmpFx[51]*tmpObjS[65] + tmpFx[63]*tmpObjS[79] + tmpFx[75]*tmpObjS[93] + tmpFx[87]*tmpObjS[107] + tmpFx[99]*tmpObjS[121] + tmpFx[111]*tmpObjS[135] + tmpFx[123]*tmpObjS[149] + tmpFx[135]*tmpObjS[163] + tmpFx[147]*tmpObjS[177] + tmpFx[159]*tmpObjS[191];
tmpQ2[52] = + tmpFx[3]*tmpObjS[10] + tmpFx[15]*tmpObjS[24] + tmpFx[27]*tmpObjS[38] + tmpFx[39]*tmpObjS[52] + tmpFx[51]*tmpObjS[66] + tmpFx[63]*tmpObjS[80] + tmpFx[75]*tmpObjS[94] + tmpFx[87]*tmpObjS[108] + tmpFx[99]*tmpObjS[122] + tmpFx[111]*tmpObjS[136] + tmpFx[123]*tmpObjS[150] + tmpFx[135]*tmpObjS[164] + tmpFx[147]*tmpObjS[178] + tmpFx[159]*tmpObjS[192];
tmpQ2[53] = + tmpFx[3]*tmpObjS[11] + tmpFx[15]*tmpObjS[25] + tmpFx[27]*tmpObjS[39] + tmpFx[39]*tmpObjS[53] + tmpFx[51]*tmpObjS[67] + tmpFx[63]*tmpObjS[81] + tmpFx[75]*tmpObjS[95] + tmpFx[87]*tmpObjS[109] + tmpFx[99]*tmpObjS[123] + tmpFx[111]*tmpObjS[137] + tmpFx[123]*tmpObjS[151] + tmpFx[135]*tmpObjS[165] + tmpFx[147]*tmpObjS[179] + tmpFx[159]*tmpObjS[193];
tmpQ2[54] = + tmpFx[3]*tmpObjS[12] + tmpFx[15]*tmpObjS[26] + tmpFx[27]*tmpObjS[40] + tmpFx[39]*tmpObjS[54] + tmpFx[51]*tmpObjS[68] + tmpFx[63]*tmpObjS[82] + tmpFx[75]*tmpObjS[96] + tmpFx[87]*tmpObjS[110] + tmpFx[99]*tmpObjS[124] + tmpFx[111]*tmpObjS[138] + tmpFx[123]*tmpObjS[152] + tmpFx[135]*tmpObjS[166] + tmpFx[147]*tmpObjS[180] + tmpFx[159]*tmpObjS[194];
tmpQ2[55] = + tmpFx[3]*tmpObjS[13] + tmpFx[15]*tmpObjS[27] + tmpFx[27]*tmpObjS[41] + tmpFx[39]*tmpObjS[55] + tmpFx[51]*tmpObjS[69] + tmpFx[63]*tmpObjS[83] + tmpFx[75]*tmpObjS[97] + tmpFx[87]*tmpObjS[111] + tmpFx[99]*tmpObjS[125] + tmpFx[111]*tmpObjS[139] + tmpFx[123]*tmpObjS[153] + tmpFx[135]*tmpObjS[167] + tmpFx[147]*tmpObjS[181] + tmpFx[159]*tmpObjS[195];
tmpQ2[56] = + tmpFx[4]*tmpObjS[0] + tmpFx[16]*tmpObjS[14] + tmpFx[28]*tmpObjS[28] + tmpFx[40]*tmpObjS[42] + tmpFx[52]*tmpObjS[56] + tmpFx[64]*tmpObjS[70] + tmpFx[76]*tmpObjS[84] + tmpFx[88]*tmpObjS[98] + tmpFx[100]*tmpObjS[112] + tmpFx[112]*tmpObjS[126] + tmpFx[124]*tmpObjS[140] + tmpFx[136]*tmpObjS[154] + tmpFx[148]*tmpObjS[168] + tmpFx[160]*tmpObjS[182];
tmpQ2[57] = + tmpFx[4]*tmpObjS[1] + tmpFx[16]*tmpObjS[15] + tmpFx[28]*tmpObjS[29] + tmpFx[40]*tmpObjS[43] + tmpFx[52]*tmpObjS[57] + tmpFx[64]*tmpObjS[71] + tmpFx[76]*tmpObjS[85] + tmpFx[88]*tmpObjS[99] + tmpFx[100]*tmpObjS[113] + tmpFx[112]*tmpObjS[127] + tmpFx[124]*tmpObjS[141] + tmpFx[136]*tmpObjS[155] + tmpFx[148]*tmpObjS[169] + tmpFx[160]*tmpObjS[183];
tmpQ2[58] = + tmpFx[4]*tmpObjS[2] + tmpFx[16]*tmpObjS[16] + tmpFx[28]*tmpObjS[30] + tmpFx[40]*tmpObjS[44] + tmpFx[52]*tmpObjS[58] + tmpFx[64]*tmpObjS[72] + tmpFx[76]*tmpObjS[86] + tmpFx[88]*tmpObjS[100] + tmpFx[100]*tmpObjS[114] + tmpFx[112]*tmpObjS[128] + tmpFx[124]*tmpObjS[142] + tmpFx[136]*tmpObjS[156] + tmpFx[148]*tmpObjS[170] + tmpFx[160]*tmpObjS[184];
tmpQ2[59] = + tmpFx[4]*tmpObjS[3] + tmpFx[16]*tmpObjS[17] + tmpFx[28]*tmpObjS[31] + tmpFx[40]*tmpObjS[45] + tmpFx[52]*tmpObjS[59] + tmpFx[64]*tmpObjS[73] + tmpFx[76]*tmpObjS[87] + tmpFx[88]*tmpObjS[101] + tmpFx[100]*tmpObjS[115] + tmpFx[112]*tmpObjS[129] + tmpFx[124]*tmpObjS[143] + tmpFx[136]*tmpObjS[157] + tmpFx[148]*tmpObjS[171] + tmpFx[160]*tmpObjS[185];
tmpQ2[60] = + tmpFx[4]*tmpObjS[4] + tmpFx[16]*tmpObjS[18] + tmpFx[28]*tmpObjS[32] + tmpFx[40]*tmpObjS[46] + tmpFx[52]*tmpObjS[60] + tmpFx[64]*tmpObjS[74] + tmpFx[76]*tmpObjS[88] + tmpFx[88]*tmpObjS[102] + tmpFx[100]*tmpObjS[116] + tmpFx[112]*tmpObjS[130] + tmpFx[124]*tmpObjS[144] + tmpFx[136]*tmpObjS[158] + tmpFx[148]*tmpObjS[172] + tmpFx[160]*tmpObjS[186];
tmpQ2[61] = + tmpFx[4]*tmpObjS[5] + tmpFx[16]*tmpObjS[19] + tmpFx[28]*tmpObjS[33] + tmpFx[40]*tmpObjS[47] + tmpFx[52]*tmpObjS[61] + tmpFx[64]*tmpObjS[75] + tmpFx[76]*tmpObjS[89] + tmpFx[88]*tmpObjS[103] + tmpFx[100]*tmpObjS[117] + tmpFx[112]*tmpObjS[131] + tmpFx[124]*tmpObjS[145] + tmpFx[136]*tmpObjS[159] + tmpFx[148]*tmpObjS[173] + tmpFx[160]*tmpObjS[187];
tmpQ2[62] = + tmpFx[4]*tmpObjS[6] + tmpFx[16]*tmpObjS[20] + tmpFx[28]*tmpObjS[34] + tmpFx[40]*tmpObjS[48] + tmpFx[52]*tmpObjS[62] + tmpFx[64]*tmpObjS[76] + tmpFx[76]*tmpObjS[90] + tmpFx[88]*tmpObjS[104] + tmpFx[100]*tmpObjS[118] + tmpFx[112]*tmpObjS[132] + tmpFx[124]*tmpObjS[146] + tmpFx[136]*tmpObjS[160] + tmpFx[148]*tmpObjS[174] + tmpFx[160]*tmpObjS[188];
tmpQ2[63] = + tmpFx[4]*tmpObjS[7] + tmpFx[16]*tmpObjS[21] + tmpFx[28]*tmpObjS[35] + tmpFx[40]*tmpObjS[49] + tmpFx[52]*tmpObjS[63] + tmpFx[64]*tmpObjS[77] + tmpFx[76]*tmpObjS[91] + tmpFx[88]*tmpObjS[105] + tmpFx[100]*tmpObjS[119] + tmpFx[112]*tmpObjS[133] + tmpFx[124]*tmpObjS[147] + tmpFx[136]*tmpObjS[161] + tmpFx[148]*tmpObjS[175] + tmpFx[160]*tmpObjS[189];
tmpQ2[64] = + tmpFx[4]*tmpObjS[8] + tmpFx[16]*tmpObjS[22] + tmpFx[28]*tmpObjS[36] + tmpFx[40]*tmpObjS[50] + tmpFx[52]*tmpObjS[64] + tmpFx[64]*tmpObjS[78] + tmpFx[76]*tmpObjS[92] + tmpFx[88]*tmpObjS[106] + tmpFx[100]*tmpObjS[120] + tmpFx[112]*tmpObjS[134] + tmpFx[124]*tmpObjS[148] + tmpFx[136]*tmpObjS[162] + tmpFx[148]*tmpObjS[176] + tmpFx[160]*tmpObjS[190];
tmpQ2[65] = + tmpFx[4]*tmpObjS[9] + tmpFx[16]*tmpObjS[23] + tmpFx[28]*tmpObjS[37] + tmpFx[40]*tmpObjS[51] + tmpFx[52]*tmpObjS[65] + tmpFx[64]*tmpObjS[79] + tmpFx[76]*tmpObjS[93] + tmpFx[88]*tmpObjS[107] + tmpFx[100]*tmpObjS[121] + tmpFx[112]*tmpObjS[135] + tmpFx[124]*tmpObjS[149] + tmpFx[136]*tmpObjS[163] + tmpFx[148]*tmpObjS[177] + tmpFx[160]*tmpObjS[191];
tmpQ2[66] = + tmpFx[4]*tmpObjS[10] + tmpFx[16]*tmpObjS[24] + tmpFx[28]*tmpObjS[38] + tmpFx[40]*tmpObjS[52] + tmpFx[52]*tmpObjS[66] + tmpFx[64]*tmpObjS[80] + tmpFx[76]*tmpObjS[94] + tmpFx[88]*tmpObjS[108] + tmpFx[100]*tmpObjS[122] + tmpFx[112]*tmpObjS[136] + tmpFx[124]*tmpObjS[150] + tmpFx[136]*tmpObjS[164] + tmpFx[148]*tmpObjS[178] + tmpFx[160]*tmpObjS[192];
tmpQ2[67] = + tmpFx[4]*tmpObjS[11] + tmpFx[16]*tmpObjS[25] + tmpFx[28]*tmpObjS[39] + tmpFx[40]*tmpObjS[53] + tmpFx[52]*tmpObjS[67] + tmpFx[64]*tmpObjS[81] + tmpFx[76]*tmpObjS[95] + tmpFx[88]*tmpObjS[109] + tmpFx[100]*tmpObjS[123] + tmpFx[112]*tmpObjS[137] + tmpFx[124]*tmpObjS[151] + tmpFx[136]*tmpObjS[165] + tmpFx[148]*tmpObjS[179] + tmpFx[160]*tmpObjS[193];
tmpQ2[68] = + tmpFx[4]*tmpObjS[12] + tmpFx[16]*tmpObjS[26] + tmpFx[28]*tmpObjS[40] + tmpFx[40]*tmpObjS[54] + tmpFx[52]*tmpObjS[68] + tmpFx[64]*tmpObjS[82] + tmpFx[76]*tmpObjS[96] + tmpFx[88]*tmpObjS[110] + tmpFx[100]*tmpObjS[124] + tmpFx[112]*tmpObjS[138] + tmpFx[124]*tmpObjS[152] + tmpFx[136]*tmpObjS[166] + tmpFx[148]*tmpObjS[180] + tmpFx[160]*tmpObjS[194];
tmpQ2[69] = + tmpFx[4]*tmpObjS[13] + tmpFx[16]*tmpObjS[27] + tmpFx[28]*tmpObjS[41] + tmpFx[40]*tmpObjS[55] + tmpFx[52]*tmpObjS[69] + tmpFx[64]*tmpObjS[83] + tmpFx[76]*tmpObjS[97] + tmpFx[88]*tmpObjS[111] + tmpFx[100]*tmpObjS[125] + tmpFx[112]*tmpObjS[139] + tmpFx[124]*tmpObjS[153] + tmpFx[136]*tmpObjS[167] + tmpFx[148]*tmpObjS[181] + tmpFx[160]*tmpObjS[195];
tmpQ2[70] = + tmpFx[5]*tmpObjS[0] + tmpFx[17]*tmpObjS[14] + tmpFx[29]*tmpObjS[28] + tmpFx[41]*tmpObjS[42] + tmpFx[53]*tmpObjS[56] + tmpFx[65]*tmpObjS[70] + tmpFx[77]*tmpObjS[84] + tmpFx[89]*tmpObjS[98] + tmpFx[101]*tmpObjS[112] + tmpFx[113]*tmpObjS[126] + tmpFx[125]*tmpObjS[140] + tmpFx[137]*tmpObjS[154] + tmpFx[149]*tmpObjS[168] + tmpFx[161]*tmpObjS[182];
tmpQ2[71] = + tmpFx[5]*tmpObjS[1] + tmpFx[17]*tmpObjS[15] + tmpFx[29]*tmpObjS[29] + tmpFx[41]*tmpObjS[43] + tmpFx[53]*tmpObjS[57] + tmpFx[65]*tmpObjS[71] + tmpFx[77]*tmpObjS[85] + tmpFx[89]*tmpObjS[99] + tmpFx[101]*tmpObjS[113] + tmpFx[113]*tmpObjS[127] + tmpFx[125]*tmpObjS[141] + tmpFx[137]*tmpObjS[155] + tmpFx[149]*tmpObjS[169] + tmpFx[161]*tmpObjS[183];
tmpQ2[72] = + tmpFx[5]*tmpObjS[2] + tmpFx[17]*tmpObjS[16] + tmpFx[29]*tmpObjS[30] + tmpFx[41]*tmpObjS[44] + tmpFx[53]*tmpObjS[58] + tmpFx[65]*tmpObjS[72] + tmpFx[77]*tmpObjS[86] + tmpFx[89]*tmpObjS[100] + tmpFx[101]*tmpObjS[114] + tmpFx[113]*tmpObjS[128] + tmpFx[125]*tmpObjS[142] + tmpFx[137]*tmpObjS[156] + tmpFx[149]*tmpObjS[170] + tmpFx[161]*tmpObjS[184];
tmpQ2[73] = + tmpFx[5]*tmpObjS[3] + tmpFx[17]*tmpObjS[17] + tmpFx[29]*tmpObjS[31] + tmpFx[41]*tmpObjS[45] + tmpFx[53]*tmpObjS[59] + tmpFx[65]*tmpObjS[73] + tmpFx[77]*tmpObjS[87] + tmpFx[89]*tmpObjS[101] + tmpFx[101]*tmpObjS[115] + tmpFx[113]*tmpObjS[129] + tmpFx[125]*tmpObjS[143] + tmpFx[137]*tmpObjS[157] + tmpFx[149]*tmpObjS[171] + tmpFx[161]*tmpObjS[185];
tmpQ2[74] = + tmpFx[5]*tmpObjS[4] + tmpFx[17]*tmpObjS[18] + tmpFx[29]*tmpObjS[32] + tmpFx[41]*tmpObjS[46] + tmpFx[53]*tmpObjS[60] + tmpFx[65]*tmpObjS[74] + tmpFx[77]*tmpObjS[88] + tmpFx[89]*tmpObjS[102] + tmpFx[101]*tmpObjS[116] + tmpFx[113]*tmpObjS[130] + tmpFx[125]*tmpObjS[144] + tmpFx[137]*tmpObjS[158] + tmpFx[149]*tmpObjS[172] + tmpFx[161]*tmpObjS[186];
tmpQ2[75] = + tmpFx[5]*tmpObjS[5] + tmpFx[17]*tmpObjS[19] + tmpFx[29]*tmpObjS[33] + tmpFx[41]*tmpObjS[47] + tmpFx[53]*tmpObjS[61] + tmpFx[65]*tmpObjS[75] + tmpFx[77]*tmpObjS[89] + tmpFx[89]*tmpObjS[103] + tmpFx[101]*tmpObjS[117] + tmpFx[113]*tmpObjS[131] + tmpFx[125]*tmpObjS[145] + tmpFx[137]*tmpObjS[159] + tmpFx[149]*tmpObjS[173] + tmpFx[161]*tmpObjS[187];
tmpQ2[76] = + tmpFx[5]*tmpObjS[6] + tmpFx[17]*tmpObjS[20] + tmpFx[29]*tmpObjS[34] + tmpFx[41]*tmpObjS[48] + tmpFx[53]*tmpObjS[62] + tmpFx[65]*tmpObjS[76] + tmpFx[77]*tmpObjS[90] + tmpFx[89]*tmpObjS[104] + tmpFx[101]*tmpObjS[118] + tmpFx[113]*tmpObjS[132] + tmpFx[125]*tmpObjS[146] + tmpFx[137]*tmpObjS[160] + tmpFx[149]*tmpObjS[174] + tmpFx[161]*tmpObjS[188];
tmpQ2[77] = + tmpFx[5]*tmpObjS[7] + tmpFx[17]*tmpObjS[21] + tmpFx[29]*tmpObjS[35] + tmpFx[41]*tmpObjS[49] + tmpFx[53]*tmpObjS[63] + tmpFx[65]*tmpObjS[77] + tmpFx[77]*tmpObjS[91] + tmpFx[89]*tmpObjS[105] + tmpFx[101]*tmpObjS[119] + tmpFx[113]*tmpObjS[133] + tmpFx[125]*tmpObjS[147] + tmpFx[137]*tmpObjS[161] + tmpFx[149]*tmpObjS[175] + tmpFx[161]*tmpObjS[189];
tmpQ2[78] = + tmpFx[5]*tmpObjS[8] + tmpFx[17]*tmpObjS[22] + tmpFx[29]*tmpObjS[36] + tmpFx[41]*tmpObjS[50] + tmpFx[53]*tmpObjS[64] + tmpFx[65]*tmpObjS[78] + tmpFx[77]*tmpObjS[92] + tmpFx[89]*tmpObjS[106] + tmpFx[101]*tmpObjS[120] + tmpFx[113]*tmpObjS[134] + tmpFx[125]*tmpObjS[148] + tmpFx[137]*tmpObjS[162] + tmpFx[149]*tmpObjS[176] + tmpFx[161]*tmpObjS[190];
tmpQ2[79] = + tmpFx[5]*tmpObjS[9] + tmpFx[17]*tmpObjS[23] + tmpFx[29]*tmpObjS[37] + tmpFx[41]*tmpObjS[51] + tmpFx[53]*tmpObjS[65] + tmpFx[65]*tmpObjS[79] + tmpFx[77]*tmpObjS[93] + tmpFx[89]*tmpObjS[107] + tmpFx[101]*tmpObjS[121] + tmpFx[113]*tmpObjS[135] + tmpFx[125]*tmpObjS[149] + tmpFx[137]*tmpObjS[163] + tmpFx[149]*tmpObjS[177] + tmpFx[161]*tmpObjS[191];
tmpQ2[80] = + tmpFx[5]*tmpObjS[10] + tmpFx[17]*tmpObjS[24] + tmpFx[29]*tmpObjS[38] + tmpFx[41]*tmpObjS[52] + tmpFx[53]*tmpObjS[66] + tmpFx[65]*tmpObjS[80] + tmpFx[77]*tmpObjS[94] + tmpFx[89]*tmpObjS[108] + tmpFx[101]*tmpObjS[122] + tmpFx[113]*tmpObjS[136] + tmpFx[125]*tmpObjS[150] + tmpFx[137]*tmpObjS[164] + tmpFx[149]*tmpObjS[178] + tmpFx[161]*tmpObjS[192];
tmpQ2[81] = + tmpFx[5]*tmpObjS[11] + tmpFx[17]*tmpObjS[25] + tmpFx[29]*tmpObjS[39] + tmpFx[41]*tmpObjS[53] + tmpFx[53]*tmpObjS[67] + tmpFx[65]*tmpObjS[81] + tmpFx[77]*tmpObjS[95] + tmpFx[89]*tmpObjS[109] + tmpFx[101]*tmpObjS[123] + tmpFx[113]*tmpObjS[137] + tmpFx[125]*tmpObjS[151] + tmpFx[137]*tmpObjS[165] + tmpFx[149]*tmpObjS[179] + tmpFx[161]*tmpObjS[193];
tmpQ2[82] = + tmpFx[5]*tmpObjS[12] + tmpFx[17]*tmpObjS[26] + tmpFx[29]*tmpObjS[40] + tmpFx[41]*tmpObjS[54] + tmpFx[53]*tmpObjS[68] + tmpFx[65]*tmpObjS[82] + tmpFx[77]*tmpObjS[96] + tmpFx[89]*tmpObjS[110] + tmpFx[101]*tmpObjS[124] + tmpFx[113]*tmpObjS[138] + tmpFx[125]*tmpObjS[152] + tmpFx[137]*tmpObjS[166] + tmpFx[149]*tmpObjS[180] + tmpFx[161]*tmpObjS[194];
tmpQ2[83] = + tmpFx[5]*tmpObjS[13] + tmpFx[17]*tmpObjS[27] + tmpFx[29]*tmpObjS[41] + tmpFx[41]*tmpObjS[55] + tmpFx[53]*tmpObjS[69] + tmpFx[65]*tmpObjS[83] + tmpFx[77]*tmpObjS[97] + tmpFx[89]*tmpObjS[111] + tmpFx[101]*tmpObjS[125] + tmpFx[113]*tmpObjS[139] + tmpFx[125]*tmpObjS[153] + tmpFx[137]*tmpObjS[167] + tmpFx[149]*tmpObjS[181] + tmpFx[161]*tmpObjS[195];
tmpQ2[84] = + tmpFx[6]*tmpObjS[0] + tmpFx[18]*tmpObjS[14] + tmpFx[30]*tmpObjS[28] + tmpFx[42]*tmpObjS[42] + tmpFx[54]*tmpObjS[56] + tmpFx[66]*tmpObjS[70] + tmpFx[78]*tmpObjS[84] + tmpFx[90]*tmpObjS[98] + tmpFx[102]*tmpObjS[112] + tmpFx[114]*tmpObjS[126] + tmpFx[126]*tmpObjS[140] + tmpFx[138]*tmpObjS[154] + tmpFx[150]*tmpObjS[168] + tmpFx[162]*tmpObjS[182];
tmpQ2[85] = + tmpFx[6]*tmpObjS[1] + tmpFx[18]*tmpObjS[15] + tmpFx[30]*tmpObjS[29] + tmpFx[42]*tmpObjS[43] + tmpFx[54]*tmpObjS[57] + tmpFx[66]*tmpObjS[71] + tmpFx[78]*tmpObjS[85] + tmpFx[90]*tmpObjS[99] + tmpFx[102]*tmpObjS[113] + tmpFx[114]*tmpObjS[127] + tmpFx[126]*tmpObjS[141] + tmpFx[138]*tmpObjS[155] + tmpFx[150]*tmpObjS[169] + tmpFx[162]*tmpObjS[183];
tmpQ2[86] = + tmpFx[6]*tmpObjS[2] + tmpFx[18]*tmpObjS[16] + tmpFx[30]*tmpObjS[30] + tmpFx[42]*tmpObjS[44] + tmpFx[54]*tmpObjS[58] + tmpFx[66]*tmpObjS[72] + tmpFx[78]*tmpObjS[86] + tmpFx[90]*tmpObjS[100] + tmpFx[102]*tmpObjS[114] + tmpFx[114]*tmpObjS[128] + tmpFx[126]*tmpObjS[142] + tmpFx[138]*tmpObjS[156] + tmpFx[150]*tmpObjS[170] + tmpFx[162]*tmpObjS[184];
tmpQ2[87] = + tmpFx[6]*tmpObjS[3] + tmpFx[18]*tmpObjS[17] + tmpFx[30]*tmpObjS[31] + tmpFx[42]*tmpObjS[45] + tmpFx[54]*tmpObjS[59] + tmpFx[66]*tmpObjS[73] + tmpFx[78]*tmpObjS[87] + tmpFx[90]*tmpObjS[101] + tmpFx[102]*tmpObjS[115] + tmpFx[114]*tmpObjS[129] + tmpFx[126]*tmpObjS[143] + tmpFx[138]*tmpObjS[157] + tmpFx[150]*tmpObjS[171] + tmpFx[162]*tmpObjS[185];
tmpQ2[88] = + tmpFx[6]*tmpObjS[4] + tmpFx[18]*tmpObjS[18] + tmpFx[30]*tmpObjS[32] + tmpFx[42]*tmpObjS[46] + tmpFx[54]*tmpObjS[60] + tmpFx[66]*tmpObjS[74] + tmpFx[78]*tmpObjS[88] + tmpFx[90]*tmpObjS[102] + tmpFx[102]*tmpObjS[116] + tmpFx[114]*tmpObjS[130] + tmpFx[126]*tmpObjS[144] + tmpFx[138]*tmpObjS[158] + tmpFx[150]*tmpObjS[172] + tmpFx[162]*tmpObjS[186];
tmpQ2[89] = + tmpFx[6]*tmpObjS[5] + tmpFx[18]*tmpObjS[19] + tmpFx[30]*tmpObjS[33] + tmpFx[42]*tmpObjS[47] + tmpFx[54]*tmpObjS[61] + tmpFx[66]*tmpObjS[75] + tmpFx[78]*tmpObjS[89] + tmpFx[90]*tmpObjS[103] + tmpFx[102]*tmpObjS[117] + tmpFx[114]*tmpObjS[131] + tmpFx[126]*tmpObjS[145] + tmpFx[138]*tmpObjS[159] + tmpFx[150]*tmpObjS[173] + tmpFx[162]*tmpObjS[187];
tmpQ2[90] = + tmpFx[6]*tmpObjS[6] + tmpFx[18]*tmpObjS[20] + tmpFx[30]*tmpObjS[34] + tmpFx[42]*tmpObjS[48] + tmpFx[54]*tmpObjS[62] + tmpFx[66]*tmpObjS[76] + tmpFx[78]*tmpObjS[90] + tmpFx[90]*tmpObjS[104] + tmpFx[102]*tmpObjS[118] + tmpFx[114]*tmpObjS[132] + tmpFx[126]*tmpObjS[146] + tmpFx[138]*tmpObjS[160] + tmpFx[150]*tmpObjS[174] + tmpFx[162]*tmpObjS[188];
tmpQ2[91] = + tmpFx[6]*tmpObjS[7] + tmpFx[18]*tmpObjS[21] + tmpFx[30]*tmpObjS[35] + tmpFx[42]*tmpObjS[49] + tmpFx[54]*tmpObjS[63] + tmpFx[66]*tmpObjS[77] + tmpFx[78]*tmpObjS[91] + tmpFx[90]*tmpObjS[105] + tmpFx[102]*tmpObjS[119] + tmpFx[114]*tmpObjS[133] + tmpFx[126]*tmpObjS[147] + tmpFx[138]*tmpObjS[161] + tmpFx[150]*tmpObjS[175] + tmpFx[162]*tmpObjS[189];
tmpQ2[92] = + tmpFx[6]*tmpObjS[8] + tmpFx[18]*tmpObjS[22] + tmpFx[30]*tmpObjS[36] + tmpFx[42]*tmpObjS[50] + tmpFx[54]*tmpObjS[64] + tmpFx[66]*tmpObjS[78] + tmpFx[78]*tmpObjS[92] + tmpFx[90]*tmpObjS[106] + tmpFx[102]*tmpObjS[120] + tmpFx[114]*tmpObjS[134] + tmpFx[126]*tmpObjS[148] + tmpFx[138]*tmpObjS[162] + tmpFx[150]*tmpObjS[176] + tmpFx[162]*tmpObjS[190];
tmpQ2[93] = + tmpFx[6]*tmpObjS[9] + tmpFx[18]*tmpObjS[23] + tmpFx[30]*tmpObjS[37] + tmpFx[42]*tmpObjS[51] + tmpFx[54]*tmpObjS[65] + tmpFx[66]*tmpObjS[79] + tmpFx[78]*tmpObjS[93] + tmpFx[90]*tmpObjS[107] + tmpFx[102]*tmpObjS[121] + tmpFx[114]*tmpObjS[135] + tmpFx[126]*tmpObjS[149] + tmpFx[138]*tmpObjS[163] + tmpFx[150]*tmpObjS[177] + tmpFx[162]*tmpObjS[191];
tmpQ2[94] = + tmpFx[6]*tmpObjS[10] + tmpFx[18]*tmpObjS[24] + tmpFx[30]*tmpObjS[38] + tmpFx[42]*tmpObjS[52] + tmpFx[54]*tmpObjS[66] + tmpFx[66]*tmpObjS[80] + tmpFx[78]*tmpObjS[94] + tmpFx[90]*tmpObjS[108] + tmpFx[102]*tmpObjS[122] + tmpFx[114]*tmpObjS[136] + tmpFx[126]*tmpObjS[150] + tmpFx[138]*tmpObjS[164] + tmpFx[150]*tmpObjS[178] + tmpFx[162]*tmpObjS[192];
tmpQ2[95] = + tmpFx[6]*tmpObjS[11] + tmpFx[18]*tmpObjS[25] + tmpFx[30]*tmpObjS[39] + tmpFx[42]*tmpObjS[53] + tmpFx[54]*tmpObjS[67] + tmpFx[66]*tmpObjS[81] + tmpFx[78]*tmpObjS[95] + tmpFx[90]*tmpObjS[109] + tmpFx[102]*tmpObjS[123] + tmpFx[114]*tmpObjS[137] + tmpFx[126]*tmpObjS[151] + tmpFx[138]*tmpObjS[165] + tmpFx[150]*tmpObjS[179] + tmpFx[162]*tmpObjS[193];
tmpQ2[96] = + tmpFx[6]*tmpObjS[12] + tmpFx[18]*tmpObjS[26] + tmpFx[30]*tmpObjS[40] + tmpFx[42]*tmpObjS[54] + tmpFx[54]*tmpObjS[68] + tmpFx[66]*tmpObjS[82] + tmpFx[78]*tmpObjS[96] + tmpFx[90]*tmpObjS[110] + tmpFx[102]*tmpObjS[124] + tmpFx[114]*tmpObjS[138] + tmpFx[126]*tmpObjS[152] + tmpFx[138]*tmpObjS[166] + tmpFx[150]*tmpObjS[180] + tmpFx[162]*tmpObjS[194];
tmpQ2[97] = + tmpFx[6]*tmpObjS[13] + tmpFx[18]*tmpObjS[27] + tmpFx[30]*tmpObjS[41] + tmpFx[42]*tmpObjS[55] + tmpFx[54]*tmpObjS[69] + tmpFx[66]*tmpObjS[83] + tmpFx[78]*tmpObjS[97] + tmpFx[90]*tmpObjS[111] + tmpFx[102]*tmpObjS[125] + tmpFx[114]*tmpObjS[139] + tmpFx[126]*tmpObjS[153] + tmpFx[138]*tmpObjS[167] + tmpFx[150]*tmpObjS[181] + tmpFx[162]*tmpObjS[195];
tmpQ2[98] = + tmpFx[7]*tmpObjS[0] + tmpFx[19]*tmpObjS[14] + tmpFx[31]*tmpObjS[28] + tmpFx[43]*tmpObjS[42] + tmpFx[55]*tmpObjS[56] + tmpFx[67]*tmpObjS[70] + tmpFx[79]*tmpObjS[84] + tmpFx[91]*tmpObjS[98] + tmpFx[103]*tmpObjS[112] + tmpFx[115]*tmpObjS[126] + tmpFx[127]*tmpObjS[140] + tmpFx[139]*tmpObjS[154] + tmpFx[151]*tmpObjS[168] + tmpFx[163]*tmpObjS[182];
tmpQ2[99] = + tmpFx[7]*tmpObjS[1] + tmpFx[19]*tmpObjS[15] + tmpFx[31]*tmpObjS[29] + tmpFx[43]*tmpObjS[43] + tmpFx[55]*tmpObjS[57] + tmpFx[67]*tmpObjS[71] + tmpFx[79]*tmpObjS[85] + tmpFx[91]*tmpObjS[99] + tmpFx[103]*tmpObjS[113] + tmpFx[115]*tmpObjS[127] + tmpFx[127]*tmpObjS[141] + tmpFx[139]*tmpObjS[155] + tmpFx[151]*tmpObjS[169] + tmpFx[163]*tmpObjS[183];
tmpQ2[100] = + tmpFx[7]*tmpObjS[2] + tmpFx[19]*tmpObjS[16] + tmpFx[31]*tmpObjS[30] + tmpFx[43]*tmpObjS[44] + tmpFx[55]*tmpObjS[58] + tmpFx[67]*tmpObjS[72] + tmpFx[79]*tmpObjS[86] + tmpFx[91]*tmpObjS[100] + tmpFx[103]*tmpObjS[114] + tmpFx[115]*tmpObjS[128] + tmpFx[127]*tmpObjS[142] + tmpFx[139]*tmpObjS[156] + tmpFx[151]*tmpObjS[170] + tmpFx[163]*tmpObjS[184];
tmpQ2[101] = + tmpFx[7]*tmpObjS[3] + tmpFx[19]*tmpObjS[17] + tmpFx[31]*tmpObjS[31] + tmpFx[43]*tmpObjS[45] + tmpFx[55]*tmpObjS[59] + tmpFx[67]*tmpObjS[73] + tmpFx[79]*tmpObjS[87] + tmpFx[91]*tmpObjS[101] + tmpFx[103]*tmpObjS[115] + tmpFx[115]*tmpObjS[129] + tmpFx[127]*tmpObjS[143] + tmpFx[139]*tmpObjS[157] + tmpFx[151]*tmpObjS[171] + tmpFx[163]*tmpObjS[185];
tmpQ2[102] = + tmpFx[7]*tmpObjS[4] + tmpFx[19]*tmpObjS[18] + tmpFx[31]*tmpObjS[32] + tmpFx[43]*tmpObjS[46] + tmpFx[55]*tmpObjS[60] + tmpFx[67]*tmpObjS[74] + tmpFx[79]*tmpObjS[88] + tmpFx[91]*tmpObjS[102] + tmpFx[103]*tmpObjS[116] + tmpFx[115]*tmpObjS[130] + tmpFx[127]*tmpObjS[144] + tmpFx[139]*tmpObjS[158] + tmpFx[151]*tmpObjS[172] + tmpFx[163]*tmpObjS[186];
tmpQ2[103] = + tmpFx[7]*tmpObjS[5] + tmpFx[19]*tmpObjS[19] + tmpFx[31]*tmpObjS[33] + tmpFx[43]*tmpObjS[47] + tmpFx[55]*tmpObjS[61] + tmpFx[67]*tmpObjS[75] + tmpFx[79]*tmpObjS[89] + tmpFx[91]*tmpObjS[103] + tmpFx[103]*tmpObjS[117] + tmpFx[115]*tmpObjS[131] + tmpFx[127]*tmpObjS[145] + tmpFx[139]*tmpObjS[159] + tmpFx[151]*tmpObjS[173] + tmpFx[163]*tmpObjS[187];
tmpQ2[104] = + tmpFx[7]*tmpObjS[6] + tmpFx[19]*tmpObjS[20] + tmpFx[31]*tmpObjS[34] + tmpFx[43]*tmpObjS[48] + tmpFx[55]*tmpObjS[62] + tmpFx[67]*tmpObjS[76] + tmpFx[79]*tmpObjS[90] + tmpFx[91]*tmpObjS[104] + tmpFx[103]*tmpObjS[118] + tmpFx[115]*tmpObjS[132] + tmpFx[127]*tmpObjS[146] + tmpFx[139]*tmpObjS[160] + tmpFx[151]*tmpObjS[174] + tmpFx[163]*tmpObjS[188];
tmpQ2[105] = + tmpFx[7]*tmpObjS[7] + tmpFx[19]*tmpObjS[21] + tmpFx[31]*tmpObjS[35] + tmpFx[43]*tmpObjS[49] + tmpFx[55]*tmpObjS[63] + tmpFx[67]*tmpObjS[77] + tmpFx[79]*tmpObjS[91] + tmpFx[91]*tmpObjS[105] + tmpFx[103]*tmpObjS[119] + tmpFx[115]*tmpObjS[133] + tmpFx[127]*tmpObjS[147] + tmpFx[139]*tmpObjS[161] + tmpFx[151]*tmpObjS[175] + tmpFx[163]*tmpObjS[189];
tmpQ2[106] = + tmpFx[7]*tmpObjS[8] + tmpFx[19]*tmpObjS[22] + tmpFx[31]*tmpObjS[36] + tmpFx[43]*tmpObjS[50] + tmpFx[55]*tmpObjS[64] + tmpFx[67]*tmpObjS[78] + tmpFx[79]*tmpObjS[92] + tmpFx[91]*tmpObjS[106] + tmpFx[103]*tmpObjS[120] + tmpFx[115]*tmpObjS[134] + tmpFx[127]*tmpObjS[148] + tmpFx[139]*tmpObjS[162] + tmpFx[151]*tmpObjS[176] + tmpFx[163]*tmpObjS[190];
tmpQ2[107] = + tmpFx[7]*tmpObjS[9] + tmpFx[19]*tmpObjS[23] + tmpFx[31]*tmpObjS[37] + tmpFx[43]*tmpObjS[51] + tmpFx[55]*tmpObjS[65] + tmpFx[67]*tmpObjS[79] + tmpFx[79]*tmpObjS[93] + tmpFx[91]*tmpObjS[107] + tmpFx[103]*tmpObjS[121] + tmpFx[115]*tmpObjS[135] + tmpFx[127]*tmpObjS[149] + tmpFx[139]*tmpObjS[163] + tmpFx[151]*tmpObjS[177] + tmpFx[163]*tmpObjS[191];
tmpQ2[108] = + tmpFx[7]*tmpObjS[10] + tmpFx[19]*tmpObjS[24] + tmpFx[31]*tmpObjS[38] + tmpFx[43]*tmpObjS[52] + tmpFx[55]*tmpObjS[66] + tmpFx[67]*tmpObjS[80] + tmpFx[79]*tmpObjS[94] + tmpFx[91]*tmpObjS[108] + tmpFx[103]*tmpObjS[122] + tmpFx[115]*tmpObjS[136] + tmpFx[127]*tmpObjS[150] + tmpFx[139]*tmpObjS[164] + tmpFx[151]*tmpObjS[178] + tmpFx[163]*tmpObjS[192];
tmpQ2[109] = + tmpFx[7]*tmpObjS[11] + tmpFx[19]*tmpObjS[25] + tmpFx[31]*tmpObjS[39] + tmpFx[43]*tmpObjS[53] + tmpFx[55]*tmpObjS[67] + tmpFx[67]*tmpObjS[81] + tmpFx[79]*tmpObjS[95] + tmpFx[91]*tmpObjS[109] + tmpFx[103]*tmpObjS[123] + tmpFx[115]*tmpObjS[137] + tmpFx[127]*tmpObjS[151] + tmpFx[139]*tmpObjS[165] + tmpFx[151]*tmpObjS[179] + tmpFx[163]*tmpObjS[193];
tmpQ2[110] = + tmpFx[7]*tmpObjS[12] + tmpFx[19]*tmpObjS[26] + tmpFx[31]*tmpObjS[40] + tmpFx[43]*tmpObjS[54] + tmpFx[55]*tmpObjS[68] + tmpFx[67]*tmpObjS[82] + tmpFx[79]*tmpObjS[96] + tmpFx[91]*tmpObjS[110] + tmpFx[103]*tmpObjS[124] + tmpFx[115]*tmpObjS[138] + tmpFx[127]*tmpObjS[152] + tmpFx[139]*tmpObjS[166] + tmpFx[151]*tmpObjS[180] + tmpFx[163]*tmpObjS[194];
tmpQ2[111] = + tmpFx[7]*tmpObjS[13] + tmpFx[19]*tmpObjS[27] + tmpFx[31]*tmpObjS[41] + tmpFx[43]*tmpObjS[55] + tmpFx[55]*tmpObjS[69] + tmpFx[67]*tmpObjS[83] + tmpFx[79]*tmpObjS[97] + tmpFx[91]*tmpObjS[111] + tmpFx[103]*tmpObjS[125] + tmpFx[115]*tmpObjS[139] + tmpFx[127]*tmpObjS[153] + tmpFx[139]*tmpObjS[167] + tmpFx[151]*tmpObjS[181] + tmpFx[163]*tmpObjS[195];
tmpQ2[112] = + tmpFx[8]*tmpObjS[0] + tmpFx[20]*tmpObjS[14] + tmpFx[32]*tmpObjS[28] + tmpFx[44]*tmpObjS[42] + tmpFx[56]*tmpObjS[56] + tmpFx[68]*tmpObjS[70] + tmpFx[80]*tmpObjS[84] + tmpFx[92]*tmpObjS[98] + tmpFx[104]*tmpObjS[112] + tmpFx[116]*tmpObjS[126] + tmpFx[128]*tmpObjS[140] + tmpFx[140]*tmpObjS[154] + tmpFx[152]*tmpObjS[168] + tmpFx[164]*tmpObjS[182];
tmpQ2[113] = + tmpFx[8]*tmpObjS[1] + tmpFx[20]*tmpObjS[15] + tmpFx[32]*tmpObjS[29] + tmpFx[44]*tmpObjS[43] + tmpFx[56]*tmpObjS[57] + tmpFx[68]*tmpObjS[71] + tmpFx[80]*tmpObjS[85] + tmpFx[92]*tmpObjS[99] + tmpFx[104]*tmpObjS[113] + tmpFx[116]*tmpObjS[127] + tmpFx[128]*tmpObjS[141] + tmpFx[140]*tmpObjS[155] + tmpFx[152]*tmpObjS[169] + tmpFx[164]*tmpObjS[183];
tmpQ2[114] = + tmpFx[8]*tmpObjS[2] + tmpFx[20]*tmpObjS[16] + tmpFx[32]*tmpObjS[30] + tmpFx[44]*tmpObjS[44] + tmpFx[56]*tmpObjS[58] + tmpFx[68]*tmpObjS[72] + tmpFx[80]*tmpObjS[86] + tmpFx[92]*tmpObjS[100] + tmpFx[104]*tmpObjS[114] + tmpFx[116]*tmpObjS[128] + tmpFx[128]*tmpObjS[142] + tmpFx[140]*tmpObjS[156] + tmpFx[152]*tmpObjS[170] + tmpFx[164]*tmpObjS[184];
tmpQ2[115] = + tmpFx[8]*tmpObjS[3] + tmpFx[20]*tmpObjS[17] + tmpFx[32]*tmpObjS[31] + tmpFx[44]*tmpObjS[45] + tmpFx[56]*tmpObjS[59] + tmpFx[68]*tmpObjS[73] + tmpFx[80]*tmpObjS[87] + tmpFx[92]*tmpObjS[101] + tmpFx[104]*tmpObjS[115] + tmpFx[116]*tmpObjS[129] + tmpFx[128]*tmpObjS[143] + tmpFx[140]*tmpObjS[157] + tmpFx[152]*tmpObjS[171] + tmpFx[164]*tmpObjS[185];
tmpQ2[116] = + tmpFx[8]*tmpObjS[4] + tmpFx[20]*tmpObjS[18] + tmpFx[32]*tmpObjS[32] + tmpFx[44]*tmpObjS[46] + tmpFx[56]*tmpObjS[60] + tmpFx[68]*tmpObjS[74] + tmpFx[80]*tmpObjS[88] + tmpFx[92]*tmpObjS[102] + tmpFx[104]*tmpObjS[116] + tmpFx[116]*tmpObjS[130] + tmpFx[128]*tmpObjS[144] + tmpFx[140]*tmpObjS[158] + tmpFx[152]*tmpObjS[172] + tmpFx[164]*tmpObjS[186];
tmpQ2[117] = + tmpFx[8]*tmpObjS[5] + tmpFx[20]*tmpObjS[19] + tmpFx[32]*tmpObjS[33] + tmpFx[44]*tmpObjS[47] + tmpFx[56]*tmpObjS[61] + tmpFx[68]*tmpObjS[75] + tmpFx[80]*tmpObjS[89] + tmpFx[92]*tmpObjS[103] + tmpFx[104]*tmpObjS[117] + tmpFx[116]*tmpObjS[131] + tmpFx[128]*tmpObjS[145] + tmpFx[140]*tmpObjS[159] + tmpFx[152]*tmpObjS[173] + tmpFx[164]*tmpObjS[187];
tmpQ2[118] = + tmpFx[8]*tmpObjS[6] + tmpFx[20]*tmpObjS[20] + tmpFx[32]*tmpObjS[34] + tmpFx[44]*tmpObjS[48] + tmpFx[56]*tmpObjS[62] + tmpFx[68]*tmpObjS[76] + tmpFx[80]*tmpObjS[90] + tmpFx[92]*tmpObjS[104] + tmpFx[104]*tmpObjS[118] + tmpFx[116]*tmpObjS[132] + tmpFx[128]*tmpObjS[146] + tmpFx[140]*tmpObjS[160] + tmpFx[152]*tmpObjS[174] + tmpFx[164]*tmpObjS[188];
tmpQ2[119] = + tmpFx[8]*tmpObjS[7] + tmpFx[20]*tmpObjS[21] + tmpFx[32]*tmpObjS[35] + tmpFx[44]*tmpObjS[49] + tmpFx[56]*tmpObjS[63] + tmpFx[68]*tmpObjS[77] + tmpFx[80]*tmpObjS[91] + tmpFx[92]*tmpObjS[105] + tmpFx[104]*tmpObjS[119] + tmpFx[116]*tmpObjS[133] + tmpFx[128]*tmpObjS[147] + tmpFx[140]*tmpObjS[161] + tmpFx[152]*tmpObjS[175] + tmpFx[164]*tmpObjS[189];
tmpQ2[120] = + tmpFx[8]*tmpObjS[8] + tmpFx[20]*tmpObjS[22] + tmpFx[32]*tmpObjS[36] + tmpFx[44]*tmpObjS[50] + tmpFx[56]*tmpObjS[64] + tmpFx[68]*tmpObjS[78] + tmpFx[80]*tmpObjS[92] + tmpFx[92]*tmpObjS[106] + tmpFx[104]*tmpObjS[120] + tmpFx[116]*tmpObjS[134] + tmpFx[128]*tmpObjS[148] + tmpFx[140]*tmpObjS[162] + tmpFx[152]*tmpObjS[176] + tmpFx[164]*tmpObjS[190];
tmpQ2[121] = + tmpFx[8]*tmpObjS[9] + tmpFx[20]*tmpObjS[23] + tmpFx[32]*tmpObjS[37] + tmpFx[44]*tmpObjS[51] + tmpFx[56]*tmpObjS[65] + tmpFx[68]*tmpObjS[79] + tmpFx[80]*tmpObjS[93] + tmpFx[92]*tmpObjS[107] + tmpFx[104]*tmpObjS[121] + tmpFx[116]*tmpObjS[135] + tmpFx[128]*tmpObjS[149] + tmpFx[140]*tmpObjS[163] + tmpFx[152]*tmpObjS[177] + tmpFx[164]*tmpObjS[191];
tmpQ2[122] = + tmpFx[8]*tmpObjS[10] + tmpFx[20]*tmpObjS[24] + tmpFx[32]*tmpObjS[38] + tmpFx[44]*tmpObjS[52] + tmpFx[56]*tmpObjS[66] + tmpFx[68]*tmpObjS[80] + tmpFx[80]*tmpObjS[94] + tmpFx[92]*tmpObjS[108] + tmpFx[104]*tmpObjS[122] + tmpFx[116]*tmpObjS[136] + tmpFx[128]*tmpObjS[150] + tmpFx[140]*tmpObjS[164] + tmpFx[152]*tmpObjS[178] + tmpFx[164]*tmpObjS[192];
tmpQ2[123] = + tmpFx[8]*tmpObjS[11] + tmpFx[20]*tmpObjS[25] + tmpFx[32]*tmpObjS[39] + tmpFx[44]*tmpObjS[53] + tmpFx[56]*tmpObjS[67] + tmpFx[68]*tmpObjS[81] + tmpFx[80]*tmpObjS[95] + tmpFx[92]*tmpObjS[109] + tmpFx[104]*tmpObjS[123] + tmpFx[116]*tmpObjS[137] + tmpFx[128]*tmpObjS[151] + tmpFx[140]*tmpObjS[165] + tmpFx[152]*tmpObjS[179] + tmpFx[164]*tmpObjS[193];
tmpQ2[124] = + tmpFx[8]*tmpObjS[12] + tmpFx[20]*tmpObjS[26] + tmpFx[32]*tmpObjS[40] + tmpFx[44]*tmpObjS[54] + tmpFx[56]*tmpObjS[68] + tmpFx[68]*tmpObjS[82] + tmpFx[80]*tmpObjS[96] + tmpFx[92]*tmpObjS[110] + tmpFx[104]*tmpObjS[124] + tmpFx[116]*tmpObjS[138] + tmpFx[128]*tmpObjS[152] + tmpFx[140]*tmpObjS[166] + tmpFx[152]*tmpObjS[180] + tmpFx[164]*tmpObjS[194];
tmpQ2[125] = + tmpFx[8]*tmpObjS[13] + tmpFx[20]*tmpObjS[27] + tmpFx[32]*tmpObjS[41] + tmpFx[44]*tmpObjS[55] + tmpFx[56]*tmpObjS[69] + tmpFx[68]*tmpObjS[83] + tmpFx[80]*tmpObjS[97] + tmpFx[92]*tmpObjS[111] + tmpFx[104]*tmpObjS[125] + tmpFx[116]*tmpObjS[139] + tmpFx[128]*tmpObjS[153] + tmpFx[140]*tmpObjS[167] + tmpFx[152]*tmpObjS[181] + tmpFx[164]*tmpObjS[195];
tmpQ2[126] = + tmpFx[9]*tmpObjS[0] + tmpFx[21]*tmpObjS[14] + tmpFx[33]*tmpObjS[28] + tmpFx[45]*tmpObjS[42] + tmpFx[57]*tmpObjS[56] + tmpFx[69]*tmpObjS[70] + tmpFx[81]*tmpObjS[84] + tmpFx[93]*tmpObjS[98] + tmpFx[105]*tmpObjS[112] + tmpFx[117]*tmpObjS[126] + tmpFx[129]*tmpObjS[140] + tmpFx[141]*tmpObjS[154] + tmpFx[153]*tmpObjS[168] + tmpFx[165]*tmpObjS[182];
tmpQ2[127] = + tmpFx[9]*tmpObjS[1] + tmpFx[21]*tmpObjS[15] + tmpFx[33]*tmpObjS[29] + tmpFx[45]*tmpObjS[43] + tmpFx[57]*tmpObjS[57] + tmpFx[69]*tmpObjS[71] + tmpFx[81]*tmpObjS[85] + tmpFx[93]*tmpObjS[99] + tmpFx[105]*tmpObjS[113] + tmpFx[117]*tmpObjS[127] + tmpFx[129]*tmpObjS[141] + tmpFx[141]*tmpObjS[155] + tmpFx[153]*tmpObjS[169] + tmpFx[165]*tmpObjS[183];
tmpQ2[128] = + tmpFx[9]*tmpObjS[2] + tmpFx[21]*tmpObjS[16] + tmpFx[33]*tmpObjS[30] + tmpFx[45]*tmpObjS[44] + tmpFx[57]*tmpObjS[58] + tmpFx[69]*tmpObjS[72] + tmpFx[81]*tmpObjS[86] + tmpFx[93]*tmpObjS[100] + tmpFx[105]*tmpObjS[114] + tmpFx[117]*tmpObjS[128] + tmpFx[129]*tmpObjS[142] + tmpFx[141]*tmpObjS[156] + tmpFx[153]*tmpObjS[170] + tmpFx[165]*tmpObjS[184];
tmpQ2[129] = + tmpFx[9]*tmpObjS[3] + tmpFx[21]*tmpObjS[17] + tmpFx[33]*tmpObjS[31] + tmpFx[45]*tmpObjS[45] + tmpFx[57]*tmpObjS[59] + tmpFx[69]*tmpObjS[73] + tmpFx[81]*tmpObjS[87] + tmpFx[93]*tmpObjS[101] + tmpFx[105]*tmpObjS[115] + tmpFx[117]*tmpObjS[129] + tmpFx[129]*tmpObjS[143] + tmpFx[141]*tmpObjS[157] + tmpFx[153]*tmpObjS[171] + tmpFx[165]*tmpObjS[185];
tmpQ2[130] = + tmpFx[9]*tmpObjS[4] + tmpFx[21]*tmpObjS[18] + tmpFx[33]*tmpObjS[32] + tmpFx[45]*tmpObjS[46] + tmpFx[57]*tmpObjS[60] + tmpFx[69]*tmpObjS[74] + tmpFx[81]*tmpObjS[88] + tmpFx[93]*tmpObjS[102] + tmpFx[105]*tmpObjS[116] + tmpFx[117]*tmpObjS[130] + tmpFx[129]*tmpObjS[144] + tmpFx[141]*tmpObjS[158] + tmpFx[153]*tmpObjS[172] + tmpFx[165]*tmpObjS[186];
tmpQ2[131] = + tmpFx[9]*tmpObjS[5] + tmpFx[21]*tmpObjS[19] + tmpFx[33]*tmpObjS[33] + tmpFx[45]*tmpObjS[47] + tmpFx[57]*tmpObjS[61] + tmpFx[69]*tmpObjS[75] + tmpFx[81]*tmpObjS[89] + tmpFx[93]*tmpObjS[103] + tmpFx[105]*tmpObjS[117] + tmpFx[117]*tmpObjS[131] + tmpFx[129]*tmpObjS[145] + tmpFx[141]*tmpObjS[159] + tmpFx[153]*tmpObjS[173] + tmpFx[165]*tmpObjS[187];
tmpQ2[132] = + tmpFx[9]*tmpObjS[6] + tmpFx[21]*tmpObjS[20] + tmpFx[33]*tmpObjS[34] + tmpFx[45]*tmpObjS[48] + tmpFx[57]*tmpObjS[62] + tmpFx[69]*tmpObjS[76] + tmpFx[81]*tmpObjS[90] + tmpFx[93]*tmpObjS[104] + tmpFx[105]*tmpObjS[118] + tmpFx[117]*tmpObjS[132] + tmpFx[129]*tmpObjS[146] + tmpFx[141]*tmpObjS[160] + tmpFx[153]*tmpObjS[174] + tmpFx[165]*tmpObjS[188];
tmpQ2[133] = + tmpFx[9]*tmpObjS[7] + tmpFx[21]*tmpObjS[21] + tmpFx[33]*tmpObjS[35] + tmpFx[45]*tmpObjS[49] + tmpFx[57]*tmpObjS[63] + tmpFx[69]*tmpObjS[77] + tmpFx[81]*tmpObjS[91] + tmpFx[93]*tmpObjS[105] + tmpFx[105]*tmpObjS[119] + tmpFx[117]*tmpObjS[133] + tmpFx[129]*tmpObjS[147] + tmpFx[141]*tmpObjS[161] + tmpFx[153]*tmpObjS[175] + tmpFx[165]*tmpObjS[189];
tmpQ2[134] = + tmpFx[9]*tmpObjS[8] + tmpFx[21]*tmpObjS[22] + tmpFx[33]*tmpObjS[36] + tmpFx[45]*tmpObjS[50] + tmpFx[57]*tmpObjS[64] + tmpFx[69]*tmpObjS[78] + tmpFx[81]*tmpObjS[92] + tmpFx[93]*tmpObjS[106] + tmpFx[105]*tmpObjS[120] + tmpFx[117]*tmpObjS[134] + tmpFx[129]*tmpObjS[148] + tmpFx[141]*tmpObjS[162] + tmpFx[153]*tmpObjS[176] + tmpFx[165]*tmpObjS[190];
tmpQ2[135] = + tmpFx[9]*tmpObjS[9] + tmpFx[21]*tmpObjS[23] + tmpFx[33]*tmpObjS[37] + tmpFx[45]*tmpObjS[51] + tmpFx[57]*tmpObjS[65] + tmpFx[69]*tmpObjS[79] + tmpFx[81]*tmpObjS[93] + tmpFx[93]*tmpObjS[107] + tmpFx[105]*tmpObjS[121] + tmpFx[117]*tmpObjS[135] + tmpFx[129]*tmpObjS[149] + tmpFx[141]*tmpObjS[163] + tmpFx[153]*tmpObjS[177] + tmpFx[165]*tmpObjS[191];
tmpQ2[136] = + tmpFx[9]*tmpObjS[10] + tmpFx[21]*tmpObjS[24] + tmpFx[33]*tmpObjS[38] + tmpFx[45]*tmpObjS[52] + tmpFx[57]*tmpObjS[66] + tmpFx[69]*tmpObjS[80] + tmpFx[81]*tmpObjS[94] + tmpFx[93]*tmpObjS[108] + tmpFx[105]*tmpObjS[122] + tmpFx[117]*tmpObjS[136] + tmpFx[129]*tmpObjS[150] + tmpFx[141]*tmpObjS[164] + tmpFx[153]*tmpObjS[178] + tmpFx[165]*tmpObjS[192];
tmpQ2[137] = + tmpFx[9]*tmpObjS[11] + tmpFx[21]*tmpObjS[25] + tmpFx[33]*tmpObjS[39] + tmpFx[45]*tmpObjS[53] + tmpFx[57]*tmpObjS[67] + tmpFx[69]*tmpObjS[81] + tmpFx[81]*tmpObjS[95] + tmpFx[93]*tmpObjS[109] + tmpFx[105]*tmpObjS[123] + tmpFx[117]*tmpObjS[137] + tmpFx[129]*tmpObjS[151] + tmpFx[141]*tmpObjS[165] + tmpFx[153]*tmpObjS[179] + tmpFx[165]*tmpObjS[193];
tmpQ2[138] = + tmpFx[9]*tmpObjS[12] + tmpFx[21]*tmpObjS[26] + tmpFx[33]*tmpObjS[40] + tmpFx[45]*tmpObjS[54] + tmpFx[57]*tmpObjS[68] + tmpFx[69]*tmpObjS[82] + tmpFx[81]*tmpObjS[96] + tmpFx[93]*tmpObjS[110] + tmpFx[105]*tmpObjS[124] + tmpFx[117]*tmpObjS[138] + tmpFx[129]*tmpObjS[152] + tmpFx[141]*tmpObjS[166] + tmpFx[153]*tmpObjS[180] + tmpFx[165]*tmpObjS[194];
tmpQ2[139] = + tmpFx[9]*tmpObjS[13] + tmpFx[21]*tmpObjS[27] + tmpFx[33]*tmpObjS[41] + tmpFx[45]*tmpObjS[55] + tmpFx[57]*tmpObjS[69] + tmpFx[69]*tmpObjS[83] + tmpFx[81]*tmpObjS[97] + tmpFx[93]*tmpObjS[111] + tmpFx[105]*tmpObjS[125] + tmpFx[117]*tmpObjS[139] + tmpFx[129]*tmpObjS[153] + tmpFx[141]*tmpObjS[167] + tmpFx[153]*tmpObjS[181] + tmpFx[165]*tmpObjS[195];
tmpQ2[140] = + tmpFx[10]*tmpObjS[0] + tmpFx[22]*tmpObjS[14] + tmpFx[34]*tmpObjS[28] + tmpFx[46]*tmpObjS[42] + tmpFx[58]*tmpObjS[56] + tmpFx[70]*tmpObjS[70] + tmpFx[82]*tmpObjS[84] + tmpFx[94]*tmpObjS[98] + tmpFx[106]*tmpObjS[112] + tmpFx[118]*tmpObjS[126] + tmpFx[130]*tmpObjS[140] + tmpFx[142]*tmpObjS[154] + tmpFx[154]*tmpObjS[168] + tmpFx[166]*tmpObjS[182];
tmpQ2[141] = + tmpFx[10]*tmpObjS[1] + tmpFx[22]*tmpObjS[15] + tmpFx[34]*tmpObjS[29] + tmpFx[46]*tmpObjS[43] + tmpFx[58]*tmpObjS[57] + tmpFx[70]*tmpObjS[71] + tmpFx[82]*tmpObjS[85] + tmpFx[94]*tmpObjS[99] + tmpFx[106]*tmpObjS[113] + tmpFx[118]*tmpObjS[127] + tmpFx[130]*tmpObjS[141] + tmpFx[142]*tmpObjS[155] + tmpFx[154]*tmpObjS[169] + tmpFx[166]*tmpObjS[183];
tmpQ2[142] = + tmpFx[10]*tmpObjS[2] + tmpFx[22]*tmpObjS[16] + tmpFx[34]*tmpObjS[30] + tmpFx[46]*tmpObjS[44] + tmpFx[58]*tmpObjS[58] + tmpFx[70]*tmpObjS[72] + tmpFx[82]*tmpObjS[86] + tmpFx[94]*tmpObjS[100] + tmpFx[106]*tmpObjS[114] + tmpFx[118]*tmpObjS[128] + tmpFx[130]*tmpObjS[142] + tmpFx[142]*tmpObjS[156] + tmpFx[154]*tmpObjS[170] + tmpFx[166]*tmpObjS[184];
tmpQ2[143] = + tmpFx[10]*tmpObjS[3] + tmpFx[22]*tmpObjS[17] + tmpFx[34]*tmpObjS[31] + tmpFx[46]*tmpObjS[45] + tmpFx[58]*tmpObjS[59] + tmpFx[70]*tmpObjS[73] + tmpFx[82]*tmpObjS[87] + tmpFx[94]*tmpObjS[101] + tmpFx[106]*tmpObjS[115] + tmpFx[118]*tmpObjS[129] + tmpFx[130]*tmpObjS[143] + tmpFx[142]*tmpObjS[157] + tmpFx[154]*tmpObjS[171] + tmpFx[166]*tmpObjS[185];
tmpQ2[144] = + tmpFx[10]*tmpObjS[4] + tmpFx[22]*tmpObjS[18] + tmpFx[34]*tmpObjS[32] + tmpFx[46]*tmpObjS[46] + tmpFx[58]*tmpObjS[60] + tmpFx[70]*tmpObjS[74] + tmpFx[82]*tmpObjS[88] + tmpFx[94]*tmpObjS[102] + tmpFx[106]*tmpObjS[116] + tmpFx[118]*tmpObjS[130] + tmpFx[130]*tmpObjS[144] + tmpFx[142]*tmpObjS[158] + tmpFx[154]*tmpObjS[172] + tmpFx[166]*tmpObjS[186];
tmpQ2[145] = + tmpFx[10]*tmpObjS[5] + tmpFx[22]*tmpObjS[19] + tmpFx[34]*tmpObjS[33] + tmpFx[46]*tmpObjS[47] + tmpFx[58]*tmpObjS[61] + tmpFx[70]*tmpObjS[75] + tmpFx[82]*tmpObjS[89] + tmpFx[94]*tmpObjS[103] + tmpFx[106]*tmpObjS[117] + tmpFx[118]*tmpObjS[131] + tmpFx[130]*tmpObjS[145] + tmpFx[142]*tmpObjS[159] + tmpFx[154]*tmpObjS[173] + tmpFx[166]*tmpObjS[187];
tmpQ2[146] = + tmpFx[10]*tmpObjS[6] + tmpFx[22]*tmpObjS[20] + tmpFx[34]*tmpObjS[34] + tmpFx[46]*tmpObjS[48] + tmpFx[58]*tmpObjS[62] + tmpFx[70]*tmpObjS[76] + tmpFx[82]*tmpObjS[90] + tmpFx[94]*tmpObjS[104] + tmpFx[106]*tmpObjS[118] + tmpFx[118]*tmpObjS[132] + tmpFx[130]*tmpObjS[146] + tmpFx[142]*tmpObjS[160] + tmpFx[154]*tmpObjS[174] + tmpFx[166]*tmpObjS[188];
tmpQ2[147] = + tmpFx[10]*tmpObjS[7] + tmpFx[22]*tmpObjS[21] + tmpFx[34]*tmpObjS[35] + tmpFx[46]*tmpObjS[49] + tmpFx[58]*tmpObjS[63] + tmpFx[70]*tmpObjS[77] + tmpFx[82]*tmpObjS[91] + tmpFx[94]*tmpObjS[105] + tmpFx[106]*tmpObjS[119] + tmpFx[118]*tmpObjS[133] + tmpFx[130]*tmpObjS[147] + tmpFx[142]*tmpObjS[161] + tmpFx[154]*tmpObjS[175] + tmpFx[166]*tmpObjS[189];
tmpQ2[148] = + tmpFx[10]*tmpObjS[8] + tmpFx[22]*tmpObjS[22] + tmpFx[34]*tmpObjS[36] + tmpFx[46]*tmpObjS[50] + tmpFx[58]*tmpObjS[64] + tmpFx[70]*tmpObjS[78] + tmpFx[82]*tmpObjS[92] + tmpFx[94]*tmpObjS[106] + tmpFx[106]*tmpObjS[120] + tmpFx[118]*tmpObjS[134] + tmpFx[130]*tmpObjS[148] + tmpFx[142]*tmpObjS[162] + tmpFx[154]*tmpObjS[176] + tmpFx[166]*tmpObjS[190];
tmpQ2[149] = + tmpFx[10]*tmpObjS[9] + tmpFx[22]*tmpObjS[23] + tmpFx[34]*tmpObjS[37] + tmpFx[46]*tmpObjS[51] + tmpFx[58]*tmpObjS[65] + tmpFx[70]*tmpObjS[79] + tmpFx[82]*tmpObjS[93] + tmpFx[94]*tmpObjS[107] + tmpFx[106]*tmpObjS[121] + tmpFx[118]*tmpObjS[135] + tmpFx[130]*tmpObjS[149] + tmpFx[142]*tmpObjS[163] + tmpFx[154]*tmpObjS[177] + tmpFx[166]*tmpObjS[191];
tmpQ2[150] = + tmpFx[10]*tmpObjS[10] + tmpFx[22]*tmpObjS[24] + tmpFx[34]*tmpObjS[38] + tmpFx[46]*tmpObjS[52] + tmpFx[58]*tmpObjS[66] + tmpFx[70]*tmpObjS[80] + tmpFx[82]*tmpObjS[94] + tmpFx[94]*tmpObjS[108] + tmpFx[106]*tmpObjS[122] + tmpFx[118]*tmpObjS[136] + tmpFx[130]*tmpObjS[150] + tmpFx[142]*tmpObjS[164] + tmpFx[154]*tmpObjS[178] + tmpFx[166]*tmpObjS[192];
tmpQ2[151] = + tmpFx[10]*tmpObjS[11] + tmpFx[22]*tmpObjS[25] + tmpFx[34]*tmpObjS[39] + tmpFx[46]*tmpObjS[53] + tmpFx[58]*tmpObjS[67] + tmpFx[70]*tmpObjS[81] + tmpFx[82]*tmpObjS[95] + tmpFx[94]*tmpObjS[109] + tmpFx[106]*tmpObjS[123] + tmpFx[118]*tmpObjS[137] + tmpFx[130]*tmpObjS[151] + tmpFx[142]*tmpObjS[165] + tmpFx[154]*tmpObjS[179] + tmpFx[166]*tmpObjS[193];
tmpQ2[152] = + tmpFx[10]*tmpObjS[12] + tmpFx[22]*tmpObjS[26] + tmpFx[34]*tmpObjS[40] + tmpFx[46]*tmpObjS[54] + tmpFx[58]*tmpObjS[68] + tmpFx[70]*tmpObjS[82] + tmpFx[82]*tmpObjS[96] + tmpFx[94]*tmpObjS[110] + tmpFx[106]*tmpObjS[124] + tmpFx[118]*tmpObjS[138] + tmpFx[130]*tmpObjS[152] + tmpFx[142]*tmpObjS[166] + tmpFx[154]*tmpObjS[180] + tmpFx[166]*tmpObjS[194];
tmpQ2[153] = + tmpFx[10]*tmpObjS[13] + tmpFx[22]*tmpObjS[27] + tmpFx[34]*tmpObjS[41] + tmpFx[46]*tmpObjS[55] + tmpFx[58]*tmpObjS[69] + tmpFx[70]*tmpObjS[83] + tmpFx[82]*tmpObjS[97] + tmpFx[94]*tmpObjS[111] + tmpFx[106]*tmpObjS[125] + tmpFx[118]*tmpObjS[139] + tmpFx[130]*tmpObjS[153] + tmpFx[142]*tmpObjS[167] + tmpFx[154]*tmpObjS[181] + tmpFx[166]*tmpObjS[195];
tmpQ2[154] = + tmpFx[11]*tmpObjS[0] + tmpFx[23]*tmpObjS[14] + tmpFx[35]*tmpObjS[28] + tmpFx[47]*tmpObjS[42] + tmpFx[59]*tmpObjS[56] + tmpFx[71]*tmpObjS[70] + tmpFx[83]*tmpObjS[84] + tmpFx[95]*tmpObjS[98] + tmpFx[107]*tmpObjS[112] + tmpFx[119]*tmpObjS[126] + tmpFx[131]*tmpObjS[140] + tmpFx[143]*tmpObjS[154] + tmpFx[155]*tmpObjS[168] + tmpFx[167]*tmpObjS[182];
tmpQ2[155] = + tmpFx[11]*tmpObjS[1] + tmpFx[23]*tmpObjS[15] + tmpFx[35]*tmpObjS[29] + tmpFx[47]*tmpObjS[43] + tmpFx[59]*tmpObjS[57] + tmpFx[71]*tmpObjS[71] + tmpFx[83]*tmpObjS[85] + tmpFx[95]*tmpObjS[99] + tmpFx[107]*tmpObjS[113] + tmpFx[119]*tmpObjS[127] + tmpFx[131]*tmpObjS[141] + tmpFx[143]*tmpObjS[155] + tmpFx[155]*tmpObjS[169] + tmpFx[167]*tmpObjS[183];
tmpQ2[156] = + tmpFx[11]*tmpObjS[2] + tmpFx[23]*tmpObjS[16] + tmpFx[35]*tmpObjS[30] + tmpFx[47]*tmpObjS[44] + tmpFx[59]*tmpObjS[58] + tmpFx[71]*tmpObjS[72] + tmpFx[83]*tmpObjS[86] + tmpFx[95]*tmpObjS[100] + tmpFx[107]*tmpObjS[114] + tmpFx[119]*tmpObjS[128] + tmpFx[131]*tmpObjS[142] + tmpFx[143]*tmpObjS[156] + tmpFx[155]*tmpObjS[170] + tmpFx[167]*tmpObjS[184];
tmpQ2[157] = + tmpFx[11]*tmpObjS[3] + tmpFx[23]*tmpObjS[17] + tmpFx[35]*tmpObjS[31] + tmpFx[47]*tmpObjS[45] + tmpFx[59]*tmpObjS[59] + tmpFx[71]*tmpObjS[73] + tmpFx[83]*tmpObjS[87] + tmpFx[95]*tmpObjS[101] + tmpFx[107]*tmpObjS[115] + tmpFx[119]*tmpObjS[129] + tmpFx[131]*tmpObjS[143] + tmpFx[143]*tmpObjS[157] + tmpFx[155]*tmpObjS[171] + tmpFx[167]*tmpObjS[185];
tmpQ2[158] = + tmpFx[11]*tmpObjS[4] + tmpFx[23]*tmpObjS[18] + tmpFx[35]*tmpObjS[32] + tmpFx[47]*tmpObjS[46] + tmpFx[59]*tmpObjS[60] + tmpFx[71]*tmpObjS[74] + tmpFx[83]*tmpObjS[88] + tmpFx[95]*tmpObjS[102] + tmpFx[107]*tmpObjS[116] + tmpFx[119]*tmpObjS[130] + tmpFx[131]*tmpObjS[144] + tmpFx[143]*tmpObjS[158] + tmpFx[155]*tmpObjS[172] + tmpFx[167]*tmpObjS[186];
tmpQ2[159] = + tmpFx[11]*tmpObjS[5] + tmpFx[23]*tmpObjS[19] + tmpFx[35]*tmpObjS[33] + tmpFx[47]*tmpObjS[47] + tmpFx[59]*tmpObjS[61] + tmpFx[71]*tmpObjS[75] + tmpFx[83]*tmpObjS[89] + tmpFx[95]*tmpObjS[103] + tmpFx[107]*tmpObjS[117] + tmpFx[119]*tmpObjS[131] + tmpFx[131]*tmpObjS[145] + tmpFx[143]*tmpObjS[159] + tmpFx[155]*tmpObjS[173] + tmpFx[167]*tmpObjS[187];
tmpQ2[160] = + tmpFx[11]*tmpObjS[6] + tmpFx[23]*tmpObjS[20] + tmpFx[35]*tmpObjS[34] + tmpFx[47]*tmpObjS[48] + tmpFx[59]*tmpObjS[62] + tmpFx[71]*tmpObjS[76] + tmpFx[83]*tmpObjS[90] + tmpFx[95]*tmpObjS[104] + tmpFx[107]*tmpObjS[118] + tmpFx[119]*tmpObjS[132] + tmpFx[131]*tmpObjS[146] + tmpFx[143]*tmpObjS[160] + tmpFx[155]*tmpObjS[174] + tmpFx[167]*tmpObjS[188];
tmpQ2[161] = + tmpFx[11]*tmpObjS[7] + tmpFx[23]*tmpObjS[21] + tmpFx[35]*tmpObjS[35] + tmpFx[47]*tmpObjS[49] + tmpFx[59]*tmpObjS[63] + tmpFx[71]*tmpObjS[77] + tmpFx[83]*tmpObjS[91] + tmpFx[95]*tmpObjS[105] + tmpFx[107]*tmpObjS[119] + tmpFx[119]*tmpObjS[133] + tmpFx[131]*tmpObjS[147] + tmpFx[143]*tmpObjS[161] + tmpFx[155]*tmpObjS[175] + tmpFx[167]*tmpObjS[189];
tmpQ2[162] = + tmpFx[11]*tmpObjS[8] + tmpFx[23]*tmpObjS[22] + tmpFx[35]*tmpObjS[36] + tmpFx[47]*tmpObjS[50] + tmpFx[59]*tmpObjS[64] + tmpFx[71]*tmpObjS[78] + tmpFx[83]*tmpObjS[92] + tmpFx[95]*tmpObjS[106] + tmpFx[107]*tmpObjS[120] + tmpFx[119]*tmpObjS[134] + tmpFx[131]*tmpObjS[148] + tmpFx[143]*tmpObjS[162] + tmpFx[155]*tmpObjS[176] + tmpFx[167]*tmpObjS[190];
tmpQ2[163] = + tmpFx[11]*tmpObjS[9] + tmpFx[23]*tmpObjS[23] + tmpFx[35]*tmpObjS[37] + tmpFx[47]*tmpObjS[51] + tmpFx[59]*tmpObjS[65] + tmpFx[71]*tmpObjS[79] + tmpFx[83]*tmpObjS[93] + tmpFx[95]*tmpObjS[107] + tmpFx[107]*tmpObjS[121] + tmpFx[119]*tmpObjS[135] + tmpFx[131]*tmpObjS[149] + tmpFx[143]*tmpObjS[163] + tmpFx[155]*tmpObjS[177] + tmpFx[167]*tmpObjS[191];
tmpQ2[164] = + tmpFx[11]*tmpObjS[10] + tmpFx[23]*tmpObjS[24] + tmpFx[35]*tmpObjS[38] + tmpFx[47]*tmpObjS[52] + tmpFx[59]*tmpObjS[66] + tmpFx[71]*tmpObjS[80] + tmpFx[83]*tmpObjS[94] + tmpFx[95]*tmpObjS[108] + tmpFx[107]*tmpObjS[122] + tmpFx[119]*tmpObjS[136] + tmpFx[131]*tmpObjS[150] + tmpFx[143]*tmpObjS[164] + tmpFx[155]*tmpObjS[178] + tmpFx[167]*tmpObjS[192];
tmpQ2[165] = + tmpFx[11]*tmpObjS[11] + tmpFx[23]*tmpObjS[25] + tmpFx[35]*tmpObjS[39] + tmpFx[47]*tmpObjS[53] + tmpFx[59]*tmpObjS[67] + tmpFx[71]*tmpObjS[81] + tmpFx[83]*tmpObjS[95] + tmpFx[95]*tmpObjS[109] + tmpFx[107]*tmpObjS[123] + tmpFx[119]*tmpObjS[137] + tmpFx[131]*tmpObjS[151] + tmpFx[143]*tmpObjS[165] + tmpFx[155]*tmpObjS[179] + tmpFx[167]*tmpObjS[193];
tmpQ2[166] = + tmpFx[11]*tmpObjS[12] + tmpFx[23]*tmpObjS[26] + tmpFx[35]*tmpObjS[40] + tmpFx[47]*tmpObjS[54] + tmpFx[59]*tmpObjS[68] + tmpFx[71]*tmpObjS[82] + tmpFx[83]*tmpObjS[96] + tmpFx[95]*tmpObjS[110] + tmpFx[107]*tmpObjS[124] + tmpFx[119]*tmpObjS[138] + tmpFx[131]*tmpObjS[152] + tmpFx[143]*tmpObjS[166] + tmpFx[155]*tmpObjS[180] + tmpFx[167]*tmpObjS[194];
tmpQ2[167] = + tmpFx[11]*tmpObjS[13] + tmpFx[23]*tmpObjS[27] + tmpFx[35]*tmpObjS[41] + tmpFx[47]*tmpObjS[55] + tmpFx[59]*tmpObjS[69] + tmpFx[71]*tmpObjS[83] + tmpFx[83]*tmpObjS[97] + tmpFx[95]*tmpObjS[111] + tmpFx[107]*tmpObjS[125] + tmpFx[119]*tmpObjS[139] + tmpFx[131]*tmpObjS[153] + tmpFx[143]*tmpObjS[167] + tmpFx[155]*tmpObjS[181] + tmpFx[167]*tmpObjS[195];
tmpQ1[0] = + tmpQ2[0]*tmpFx[0] + tmpQ2[1]*tmpFx[12] + tmpQ2[2]*tmpFx[24] + tmpQ2[3]*tmpFx[36] + tmpQ2[4]*tmpFx[48] + tmpQ2[5]*tmpFx[60] + tmpQ2[6]*tmpFx[72] + tmpQ2[7]*tmpFx[84] + tmpQ2[8]*tmpFx[96] + tmpQ2[9]*tmpFx[108] + tmpQ2[10]*tmpFx[120] + tmpQ2[11]*tmpFx[132] + tmpQ2[12]*tmpFx[144] + tmpQ2[13]*tmpFx[156];
tmpQ1[1] = + tmpQ2[0]*tmpFx[1] + tmpQ2[1]*tmpFx[13] + tmpQ2[2]*tmpFx[25] + tmpQ2[3]*tmpFx[37] + tmpQ2[4]*tmpFx[49] + tmpQ2[5]*tmpFx[61] + tmpQ2[6]*tmpFx[73] + tmpQ2[7]*tmpFx[85] + tmpQ2[8]*tmpFx[97] + tmpQ2[9]*tmpFx[109] + tmpQ2[10]*tmpFx[121] + tmpQ2[11]*tmpFx[133] + tmpQ2[12]*tmpFx[145] + tmpQ2[13]*tmpFx[157];
tmpQ1[2] = + tmpQ2[0]*tmpFx[2] + tmpQ2[1]*tmpFx[14] + tmpQ2[2]*tmpFx[26] + tmpQ2[3]*tmpFx[38] + tmpQ2[4]*tmpFx[50] + tmpQ2[5]*tmpFx[62] + tmpQ2[6]*tmpFx[74] + tmpQ2[7]*tmpFx[86] + tmpQ2[8]*tmpFx[98] + tmpQ2[9]*tmpFx[110] + tmpQ2[10]*tmpFx[122] + tmpQ2[11]*tmpFx[134] + tmpQ2[12]*tmpFx[146] + tmpQ2[13]*tmpFx[158];
tmpQ1[3] = + tmpQ2[0]*tmpFx[3] + tmpQ2[1]*tmpFx[15] + tmpQ2[2]*tmpFx[27] + tmpQ2[3]*tmpFx[39] + tmpQ2[4]*tmpFx[51] + tmpQ2[5]*tmpFx[63] + tmpQ2[6]*tmpFx[75] + tmpQ2[7]*tmpFx[87] + tmpQ2[8]*tmpFx[99] + tmpQ2[9]*tmpFx[111] + tmpQ2[10]*tmpFx[123] + tmpQ2[11]*tmpFx[135] + tmpQ2[12]*tmpFx[147] + tmpQ2[13]*tmpFx[159];
tmpQ1[4] = + tmpQ2[0]*tmpFx[4] + tmpQ2[1]*tmpFx[16] + tmpQ2[2]*tmpFx[28] + tmpQ2[3]*tmpFx[40] + tmpQ2[4]*tmpFx[52] + tmpQ2[5]*tmpFx[64] + tmpQ2[6]*tmpFx[76] + tmpQ2[7]*tmpFx[88] + tmpQ2[8]*tmpFx[100] + tmpQ2[9]*tmpFx[112] + tmpQ2[10]*tmpFx[124] + tmpQ2[11]*tmpFx[136] + tmpQ2[12]*tmpFx[148] + tmpQ2[13]*tmpFx[160];
tmpQ1[5] = + tmpQ2[0]*tmpFx[5] + tmpQ2[1]*tmpFx[17] + tmpQ2[2]*tmpFx[29] + tmpQ2[3]*tmpFx[41] + tmpQ2[4]*tmpFx[53] + tmpQ2[5]*tmpFx[65] + tmpQ2[6]*tmpFx[77] + tmpQ2[7]*tmpFx[89] + tmpQ2[8]*tmpFx[101] + tmpQ2[9]*tmpFx[113] + tmpQ2[10]*tmpFx[125] + tmpQ2[11]*tmpFx[137] + tmpQ2[12]*tmpFx[149] + tmpQ2[13]*tmpFx[161];
tmpQ1[6] = + tmpQ2[0]*tmpFx[6] + tmpQ2[1]*tmpFx[18] + tmpQ2[2]*tmpFx[30] + tmpQ2[3]*tmpFx[42] + tmpQ2[4]*tmpFx[54] + tmpQ2[5]*tmpFx[66] + tmpQ2[6]*tmpFx[78] + tmpQ2[7]*tmpFx[90] + tmpQ2[8]*tmpFx[102] + tmpQ2[9]*tmpFx[114] + tmpQ2[10]*tmpFx[126] + tmpQ2[11]*tmpFx[138] + tmpQ2[12]*tmpFx[150] + tmpQ2[13]*tmpFx[162];
tmpQ1[7] = + tmpQ2[0]*tmpFx[7] + tmpQ2[1]*tmpFx[19] + tmpQ2[2]*tmpFx[31] + tmpQ2[3]*tmpFx[43] + tmpQ2[4]*tmpFx[55] + tmpQ2[5]*tmpFx[67] + tmpQ2[6]*tmpFx[79] + tmpQ2[7]*tmpFx[91] + tmpQ2[8]*tmpFx[103] + tmpQ2[9]*tmpFx[115] + tmpQ2[10]*tmpFx[127] + tmpQ2[11]*tmpFx[139] + tmpQ2[12]*tmpFx[151] + tmpQ2[13]*tmpFx[163];
tmpQ1[8] = + tmpQ2[0]*tmpFx[8] + tmpQ2[1]*tmpFx[20] + tmpQ2[2]*tmpFx[32] + tmpQ2[3]*tmpFx[44] + tmpQ2[4]*tmpFx[56] + tmpQ2[5]*tmpFx[68] + tmpQ2[6]*tmpFx[80] + tmpQ2[7]*tmpFx[92] + tmpQ2[8]*tmpFx[104] + tmpQ2[9]*tmpFx[116] + tmpQ2[10]*tmpFx[128] + tmpQ2[11]*tmpFx[140] + tmpQ2[12]*tmpFx[152] + tmpQ2[13]*tmpFx[164];
tmpQ1[9] = + tmpQ2[0]*tmpFx[9] + tmpQ2[1]*tmpFx[21] + tmpQ2[2]*tmpFx[33] + tmpQ2[3]*tmpFx[45] + tmpQ2[4]*tmpFx[57] + tmpQ2[5]*tmpFx[69] + tmpQ2[6]*tmpFx[81] + tmpQ2[7]*tmpFx[93] + tmpQ2[8]*tmpFx[105] + tmpQ2[9]*tmpFx[117] + tmpQ2[10]*tmpFx[129] + tmpQ2[11]*tmpFx[141] + tmpQ2[12]*tmpFx[153] + tmpQ2[13]*tmpFx[165];
tmpQ1[10] = + tmpQ2[0]*tmpFx[10] + tmpQ2[1]*tmpFx[22] + tmpQ2[2]*tmpFx[34] + tmpQ2[3]*tmpFx[46] + tmpQ2[4]*tmpFx[58] + tmpQ2[5]*tmpFx[70] + tmpQ2[6]*tmpFx[82] + tmpQ2[7]*tmpFx[94] + tmpQ2[8]*tmpFx[106] + tmpQ2[9]*tmpFx[118] + tmpQ2[10]*tmpFx[130] + tmpQ2[11]*tmpFx[142] + tmpQ2[12]*tmpFx[154] + tmpQ2[13]*tmpFx[166];
tmpQ1[11] = + tmpQ2[0]*tmpFx[11] + tmpQ2[1]*tmpFx[23] + tmpQ2[2]*tmpFx[35] + tmpQ2[3]*tmpFx[47] + tmpQ2[4]*tmpFx[59] + tmpQ2[5]*tmpFx[71] + tmpQ2[6]*tmpFx[83] + tmpQ2[7]*tmpFx[95] + tmpQ2[8]*tmpFx[107] + tmpQ2[9]*tmpFx[119] + tmpQ2[10]*tmpFx[131] + tmpQ2[11]*tmpFx[143] + tmpQ2[12]*tmpFx[155] + tmpQ2[13]*tmpFx[167];
tmpQ1[12] = + tmpQ2[14]*tmpFx[0] + tmpQ2[15]*tmpFx[12] + tmpQ2[16]*tmpFx[24] + tmpQ2[17]*tmpFx[36] + tmpQ2[18]*tmpFx[48] + tmpQ2[19]*tmpFx[60] + tmpQ2[20]*tmpFx[72] + tmpQ2[21]*tmpFx[84] + tmpQ2[22]*tmpFx[96] + tmpQ2[23]*tmpFx[108] + tmpQ2[24]*tmpFx[120] + tmpQ2[25]*tmpFx[132] + tmpQ2[26]*tmpFx[144] + tmpQ2[27]*tmpFx[156];
tmpQ1[13] = + tmpQ2[14]*tmpFx[1] + tmpQ2[15]*tmpFx[13] + tmpQ2[16]*tmpFx[25] + tmpQ2[17]*tmpFx[37] + tmpQ2[18]*tmpFx[49] + tmpQ2[19]*tmpFx[61] + tmpQ2[20]*tmpFx[73] + tmpQ2[21]*tmpFx[85] + tmpQ2[22]*tmpFx[97] + tmpQ2[23]*tmpFx[109] + tmpQ2[24]*tmpFx[121] + tmpQ2[25]*tmpFx[133] + tmpQ2[26]*tmpFx[145] + tmpQ2[27]*tmpFx[157];
tmpQ1[14] = + tmpQ2[14]*tmpFx[2] + tmpQ2[15]*tmpFx[14] + tmpQ2[16]*tmpFx[26] + tmpQ2[17]*tmpFx[38] + tmpQ2[18]*tmpFx[50] + tmpQ2[19]*tmpFx[62] + tmpQ2[20]*tmpFx[74] + tmpQ2[21]*tmpFx[86] + tmpQ2[22]*tmpFx[98] + tmpQ2[23]*tmpFx[110] + tmpQ2[24]*tmpFx[122] + tmpQ2[25]*tmpFx[134] + tmpQ2[26]*tmpFx[146] + tmpQ2[27]*tmpFx[158];
tmpQ1[15] = + tmpQ2[14]*tmpFx[3] + tmpQ2[15]*tmpFx[15] + tmpQ2[16]*tmpFx[27] + tmpQ2[17]*tmpFx[39] + tmpQ2[18]*tmpFx[51] + tmpQ2[19]*tmpFx[63] + tmpQ2[20]*tmpFx[75] + tmpQ2[21]*tmpFx[87] + tmpQ2[22]*tmpFx[99] + tmpQ2[23]*tmpFx[111] + tmpQ2[24]*tmpFx[123] + tmpQ2[25]*tmpFx[135] + tmpQ2[26]*tmpFx[147] + tmpQ2[27]*tmpFx[159];
tmpQ1[16] = + tmpQ2[14]*tmpFx[4] + tmpQ2[15]*tmpFx[16] + tmpQ2[16]*tmpFx[28] + tmpQ2[17]*tmpFx[40] + tmpQ2[18]*tmpFx[52] + tmpQ2[19]*tmpFx[64] + tmpQ2[20]*tmpFx[76] + tmpQ2[21]*tmpFx[88] + tmpQ2[22]*tmpFx[100] + tmpQ2[23]*tmpFx[112] + tmpQ2[24]*tmpFx[124] + tmpQ2[25]*tmpFx[136] + tmpQ2[26]*tmpFx[148] + tmpQ2[27]*tmpFx[160];
tmpQ1[17] = + tmpQ2[14]*tmpFx[5] + tmpQ2[15]*tmpFx[17] + tmpQ2[16]*tmpFx[29] + tmpQ2[17]*tmpFx[41] + tmpQ2[18]*tmpFx[53] + tmpQ2[19]*tmpFx[65] + tmpQ2[20]*tmpFx[77] + tmpQ2[21]*tmpFx[89] + tmpQ2[22]*tmpFx[101] + tmpQ2[23]*tmpFx[113] + tmpQ2[24]*tmpFx[125] + tmpQ2[25]*tmpFx[137] + tmpQ2[26]*tmpFx[149] + tmpQ2[27]*tmpFx[161];
tmpQ1[18] = + tmpQ2[14]*tmpFx[6] + tmpQ2[15]*tmpFx[18] + tmpQ2[16]*tmpFx[30] + tmpQ2[17]*tmpFx[42] + tmpQ2[18]*tmpFx[54] + tmpQ2[19]*tmpFx[66] + tmpQ2[20]*tmpFx[78] + tmpQ2[21]*tmpFx[90] + tmpQ2[22]*tmpFx[102] + tmpQ2[23]*tmpFx[114] + tmpQ2[24]*tmpFx[126] + tmpQ2[25]*tmpFx[138] + tmpQ2[26]*tmpFx[150] + tmpQ2[27]*tmpFx[162];
tmpQ1[19] = + tmpQ2[14]*tmpFx[7] + tmpQ2[15]*tmpFx[19] + tmpQ2[16]*tmpFx[31] + tmpQ2[17]*tmpFx[43] + tmpQ2[18]*tmpFx[55] + tmpQ2[19]*tmpFx[67] + tmpQ2[20]*tmpFx[79] + tmpQ2[21]*tmpFx[91] + tmpQ2[22]*tmpFx[103] + tmpQ2[23]*tmpFx[115] + tmpQ2[24]*tmpFx[127] + tmpQ2[25]*tmpFx[139] + tmpQ2[26]*tmpFx[151] + tmpQ2[27]*tmpFx[163];
tmpQ1[20] = + tmpQ2[14]*tmpFx[8] + tmpQ2[15]*tmpFx[20] + tmpQ2[16]*tmpFx[32] + tmpQ2[17]*tmpFx[44] + tmpQ2[18]*tmpFx[56] + tmpQ2[19]*tmpFx[68] + tmpQ2[20]*tmpFx[80] + tmpQ2[21]*tmpFx[92] + tmpQ2[22]*tmpFx[104] + tmpQ2[23]*tmpFx[116] + tmpQ2[24]*tmpFx[128] + tmpQ2[25]*tmpFx[140] + tmpQ2[26]*tmpFx[152] + tmpQ2[27]*tmpFx[164];
tmpQ1[21] = + tmpQ2[14]*tmpFx[9] + tmpQ2[15]*tmpFx[21] + tmpQ2[16]*tmpFx[33] + tmpQ2[17]*tmpFx[45] + tmpQ2[18]*tmpFx[57] + tmpQ2[19]*tmpFx[69] + tmpQ2[20]*tmpFx[81] + tmpQ2[21]*tmpFx[93] + tmpQ2[22]*tmpFx[105] + tmpQ2[23]*tmpFx[117] + tmpQ2[24]*tmpFx[129] + tmpQ2[25]*tmpFx[141] + tmpQ2[26]*tmpFx[153] + tmpQ2[27]*tmpFx[165];
tmpQ1[22] = + tmpQ2[14]*tmpFx[10] + tmpQ2[15]*tmpFx[22] + tmpQ2[16]*tmpFx[34] + tmpQ2[17]*tmpFx[46] + tmpQ2[18]*tmpFx[58] + tmpQ2[19]*tmpFx[70] + tmpQ2[20]*tmpFx[82] + tmpQ2[21]*tmpFx[94] + tmpQ2[22]*tmpFx[106] + tmpQ2[23]*tmpFx[118] + tmpQ2[24]*tmpFx[130] + tmpQ2[25]*tmpFx[142] + tmpQ2[26]*tmpFx[154] + tmpQ2[27]*tmpFx[166];
tmpQ1[23] = + tmpQ2[14]*tmpFx[11] + tmpQ2[15]*tmpFx[23] + tmpQ2[16]*tmpFx[35] + tmpQ2[17]*tmpFx[47] + tmpQ2[18]*tmpFx[59] + tmpQ2[19]*tmpFx[71] + tmpQ2[20]*tmpFx[83] + tmpQ2[21]*tmpFx[95] + tmpQ2[22]*tmpFx[107] + tmpQ2[23]*tmpFx[119] + tmpQ2[24]*tmpFx[131] + tmpQ2[25]*tmpFx[143] + tmpQ2[26]*tmpFx[155] + tmpQ2[27]*tmpFx[167];
tmpQ1[24] = + tmpQ2[28]*tmpFx[0] + tmpQ2[29]*tmpFx[12] + tmpQ2[30]*tmpFx[24] + tmpQ2[31]*tmpFx[36] + tmpQ2[32]*tmpFx[48] + tmpQ2[33]*tmpFx[60] + tmpQ2[34]*tmpFx[72] + tmpQ2[35]*tmpFx[84] + tmpQ2[36]*tmpFx[96] + tmpQ2[37]*tmpFx[108] + tmpQ2[38]*tmpFx[120] + tmpQ2[39]*tmpFx[132] + tmpQ2[40]*tmpFx[144] + tmpQ2[41]*tmpFx[156];
tmpQ1[25] = + tmpQ2[28]*tmpFx[1] + tmpQ2[29]*tmpFx[13] + tmpQ2[30]*tmpFx[25] + tmpQ2[31]*tmpFx[37] + tmpQ2[32]*tmpFx[49] + tmpQ2[33]*tmpFx[61] + tmpQ2[34]*tmpFx[73] + tmpQ2[35]*tmpFx[85] + tmpQ2[36]*tmpFx[97] + tmpQ2[37]*tmpFx[109] + tmpQ2[38]*tmpFx[121] + tmpQ2[39]*tmpFx[133] + tmpQ2[40]*tmpFx[145] + tmpQ2[41]*tmpFx[157];
tmpQ1[26] = + tmpQ2[28]*tmpFx[2] + tmpQ2[29]*tmpFx[14] + tmpQ2[30]*tmpFx[26] + tmpQ2[31]*tmpFx[38] + tmpQ2[32]*tmpFx[50] + tmpQ2[33]*tmpFx[62] + tmpQ2[34]*tmpFx[74] + tmpQ2[35]*tmpFx[86] + tmpQ2[36]*tmpFx[98] + tmpQ2[37]*tmpFx[110] + tmpQ2[38]*tmpFx[122] + tmpQ2[39]*tmpFx[134] + tmpQ2[40]*tmpFx[146] + tmpQ2[41]*tmpFx[158];
tmpQ1[27] = + tmpQ2[28]*tmpFx[3] + tmpQ2[29]*tmpFx[15] + tmpQ2[30]*tmpFx[27] + tmpQ2[31]*tmpFx[39] + tmpQ2[32]*tmpFx[51] + tmpQ2[33]*tmpFx[63] + tmpQ2[34]*tmpFx[75] + tmpQ2[35]*tmpFx[87] + tmpQ2[36]*tmpFx[99] + tmpQ2[37]*tmpFx[111] + tmpQ2[38]*tmpFx[123] + tmpQ2[39]*tmpFx[135] + tmpQ2[40]*tmpFx[147] + tmpQ2[41]*tmpFx[159];
tmpQ1[28] = + tmpQ2[28]*tmpFx[4] + tmpQ2[29]*tmpFx[16] + tmpQ2[30]*tmpFx[28] + tmpQ2[31]*tmpFx[40] + tmpQ2[32]*tmpFx[52] + tmpQ2[33]*tmpFx[64] + tmpQ2[34]*tmpFx[76] + tmpQ2[35]*tmpFx[88] + tmpQ2[36]*tmpFx[100] + tmpQ2[37]*tmpFx[112] + tmpQ2[38]*tmpFx[124] + tmpQ2[39]*tmpFx[136] + tmpQ2[40]*tmpFx[148] + tmpQ2[41]*tmpFx[160];
tmpQ1[29] = + tmpQ2[28]*tmpFx[5] + tmpQ2[29]*tmpFx[17] + tmpQ2[30]*tmpFx[29] + tmpQ2[31]*tmpFx[41] + tmpQ2[32]*tmpFx[53] + tmpQ2[33]*tmpFx[65] + tmpQ2[34]*tmpFx[77] + tmpQ2[35]*tmpFx[89] + tmpQ2[36]*tmpFx[101] + tmpQ2[37]*tmpFx[113] + tmpQ2[38]*tmpFx[125] + tmpQ2[39]*tmpFx[137] + tmpQ2[40]*tmpFx[149] + tmpQ2[41]*tmpFx[161];
tmpQ1[30] = + tmpQ2[28]*tmpFx[6] + tmpQ2[29]*tmpFx[18] + tmpQ2[30]*tmpFx[30] + tmpQ2[31]*tmpFx[42] + tmpQ2[32]*tmpFx[54] + tmpQ2[33]*tmpFx[66] + tmpQ2[34]*tmpFx[78] + tmpQ2[35]*tmpFx[90] + tmpQ2[36]*tmpFx[102] + tmpQ2[37]*tmpFx[114] + tmpQ2[38]*tmpFx[126] + tmpQ2[39]*tmpFx[138] + tmpQ2[40]*tmpFx[150] + tmpQ2[41]*tmpFx[162];
tmpQ1[31] = + tmpQ2[28]*tmpFx[7] + tmpQ2[29]*tmpFx[19] + tmpQ2[30]*tmpFx[31] + tmpQ2[31]*tmpFx[43] + tmpQ2[32]*tmpFx[55] + tmpQ2[33]*tmpFx[67] + tmpQ2[34]*tmpFx[79] + tmpQ2[35]*tmpFx[91] + tmpQ2[36]*tmpFx[103] + tmpQ2[37]*tmpFx[115] + tmpQ2[38]*tmpFx[127] + tmpQ2[39]*tmpFx[139] + tmpQ2[40]*tmpFx[151] + tmpQ2[41]*tmpFx[163];
tmpQ1[32] = + tmpQ2[28]*tmpFx[8] + tmpQ2[29]*tmpFx[20] + tmpQ2[30]*tmpFx[32] + tmpQ2[31]*tmpFx[44] + tmpQ2[32]*tmpFx[56] + tmpQ2[33]*tmpFx[68] + tmpQ2[34]*tmpFx[80] + tmpQ2[35]*tmpFx[92] + tmpQ2[36]*tmpFx[104] + tmpQ2[37]*tmpFx[116] + tmpQ2[38]*tmpFx[128] + tmpQ2[39]*tmpFx[140] + tmpQ2[40]*tmpFx[152] + tmpQ2[41]*tmpFx[164];
tmpQ1[33] = + tmpQ2[28]*tmpFx[9] + tmpQ2[29]*tmpFx[21] + tmpQ2[30]*tmpFx[33] + tmpQ2[31]*tmpFx[45] + tmpQ2[32]*tmpFx[57] + tmpQ2[33]*tmpFx[69] + tmpQ2[34]*tmpFx[81] + tmpQ2[35]*tmpFx[93] + tmpQ2[36]*tmpFx[105] + tmpQ2[37]*tmpFx[117] + tmpQ2[38]*tmpFx[129] + tmpQ2[39]*tmpFx[141] + tmpQ2[40]*tmpFx[153] + tmpQ2[41]*tmpFx[165];
tmpQ1[34] = + tmpQ2[28]*tmpFx[10] + tmpQ2[29]*tmpFx[22] + tmpQ2[30]*tmpFx[34] + tmpQ2[31]*tmpFx[46] + tmpQ2[32]*tmpFx[58] + tmpQ2[33]*tmpFx[70] + tmpQ2[34]*tmpFx[82] + tmpQ2[35]*tmpFx[94] + tmpQ2[36]*tmpFx[106] + tmpQ2[37]*tmpFx[118] + tmpQ2[38]*tmpFx[130] + tmpQ2[39]*tmpFx[142] + tmpQ2[40]*tmpFx[154] + tmpQ2[41]*tmpFx[166];
tmpQ1[35] = + tmpQ2[28]*tmpFx[11] + tmpQ2[29]*tmpFx[23] + tmpQ2[30]*tmpFx[35] + tmpQ2[31]*tmpFx[47] + tmpQ2[32]*tmpFx[59] + tmpQ2[33]*tmpFx[71] + tmpQ2[34]*tmpFx[83] + tmpQ2[35]*tmpFx[95] + tmpQ2[36]*tmpFx[107] + tmpQ2[37]*tmpFx[119] + tmpQ2[38]*tmpFx[131] + tmpQ2[39]*tmpFx[143] + tmpQ2[40]*tmpFx[155] + tmpQ2[41]*tmpFx[167];
tmpQ1[36] = + tmpQ2[42]*tmpFx[0] + tmpQ2[43]*tmpFx[12] + tmpQ2[44]*tmpFx[24] + tmpQ2[45]*tmpFx[36] + tmpQ2[46]*tmpFx[48] + tmpQ2[47]*tmpFx[60] + tmpQ2[48]*tmpFx[72] + tmpQ2[49]*tmpFx[84] + tmpQ2[50]*tmpFx[96] + tmpQ2[51]*tmpFx[108] + tmpQ2[52]*tmpFx[120] + tmpQ2[53]*tmpFx[132] + tmpQ2[54]*tmpFx[144] + tmpQ2[55]*tmpFx[156];
tmpQ1[37] = + tmpQ2[42]*tmpFx[1] + tmpQ2[43]*tmpFx[13] + tmpQ2[44]*tmpFx[25] + tmpQ2[45]*tmpFx[37] + tmpQ2[46]*tmpFx[49] + tmpQ2[47]*tmpFx[61] + tmpQ2[48]*tmpFx[73] + tmpQ2[49]*tmpFx[85] + tmpQ2[50]*tmpFx[97] + tmpQ2[51]*tmpFx[109] + tmpQ2[52]*tmpFx[121] + tmpQ2[53]*tmpFx[133] + tmpQ2[54]*tmpFx[145] + tmpQ2[55]*tmpFx[157];
tmpQ1[38] = + tmpQ2[42]*tmpFx[2] + tmpQ2[43]*tmpFx[14] + tmpQ2[44]*tmpFx[26] + tmpQ2[45]*tmpFx[38] + tmpQ2[46]*tmpFx[50] + tmpQ2[47]*tmpFx[62] + tmpQ2[48]*tmpFx[74] + tmpQ2[49]*tmpFx[86] + tmpQ2[50]*tmpFx[98] + tmpQ2[51]*tmpFx[110] + tmpQ2[52]*tmpFx[122] + tmpQ2[53]*tmpFx[134] + tmpQ2[54]*tmpFx[146] + tmpQ2[55]*tmpFx[158];
tmpQ1[39] = + tmpQ2[42]*tmpFx[3] + tmpQ2[43]*tmpFx[15] + tmpQ2[44]*tmpFx[27] + tmpQ2[45]*tmpFx[39] + tmpQ2[46]*tmpFx[51] + tmpQ2[47]*tmpFx[63] + tmpQ2[48]*tmpFx[75] + tmpQ2[49]*tmpFx[87] + tmpQ2[50]*tmpFx[99] + tmpQ2[51]*tmpFx[111] + tmpQ2[52]*tmpFx[123] + tmpQ2[53]*tmpFx[135] + tmpQ2[54]*tmpFx[147] + tmpQ2[55]*tmpFx[159];
tmpQ1[40] = + tmpQ2[42]*tmpFx[4] + tmpQ2[43]*tmpFx[16] + tmpQ2[44]*tmpFx[28] + tmpQ2[45]*tmpFx[40] + tmpQ2[46]*tmpFx[52] + tmpQ2[47]*tmpFx[64] + tmpQ2[48]*tmpFx[76] + tmpQ2[49]*tmpFx[88] + tmpQ2[50]*tmpFx[100] + tmpQ2[51]*tmpFx[112] + tmpQ2[52]*tmpFx[124] + tmpQ2[53]*tmpFx[136] + tmpQ2[54]*tmpFx[148] + tmpQ2[55]*tmpFx[160];
tmpQ1[41] = + tmpQ2[42]*tmpFx[5] + tmpQ2[43]*tmpFx[17] + tmpQ2[44]*tmpFx[29] + tmpQ2[45]*tmpFx[41] + tmpQ2[46]*tmpFx[53] + tmpQ2[47]*tmpFx[65] + tmpQ2[48]*tmpFx[77] + tmpQ2[49]*tmpFx[89] + tmpQ2[50]*tmpFx[101] + tmpQ2[51]*tmpFx[113] + tmpQ2[52]*tmpFx[125] + tmpQ2[53]*tmpFx[137] + tmpQ2[54]*tmpFx[149] + tmpQ2[55]*tmpFx[161];
tmpQ1[42] = + tmpQ2[42]*tmpFx[6] + tmpQ2[43]*tmpFx[18] + tmpQ2[44]*tmpFx[30] + tmpQ2[45]*tmpFx[42] + tmpQ2[46]*tmpFx[54] + tmpQ2[47]*tmpFx[66] + tmpQ2[48]*tmpFx[78] + tmpQ2[49]*tmpFx[90] + tmpQ2[50]*tmpFx[102] + tmpQ2[51]*tmpFx[114] + tmpQ2[52]*tmpFx[126] + tmpQ2[53]*tmpFx[138] + tmpQ2[54]*tmpFx[150] + tmpQ2[55]*tmpFx[162];
tmpQ1[43] = + tmpQ2[42]*tmpFx[7] + tmpQ2[43]*tmpFx[19] + tmpQ2[44]*tmpFx[31] + tmpQ2[45]*tmpFx[43] + tmpQ2[46]*tmpFx[55] + tmpQ2[47]*tmpFx[67] + tmpQ2[48]*tmpFx[79] + tmpQ2[49]*tmpFx[91] + tmpQ2[50]*tmpFx[103] + tmpQ2[51]*tmpFx[115] + tmpQ2[52]*tmpFx[127] + tmpQ2[53]*tmpFx[139] + tmpQ2[54]*tmpFx[151] + tmpQ2[55]*tmpFx[163];
tmpQ1[44] = + tmpQ2[42]*tmpFx[8] + tmpQ2[43]*tmpFx[20] + tmpQ2[44]*tmpFx[32] + tmpQ2[45]*tmpFx[44] + tmpQ2[46]*tmpFx[56] + tmpQ2[47]*tmpFx[68] + tmpQ2[48]*tmpFx[80] + tmpQ2[49]*tmpFx[92] + tmpQ2[50]*tmpFx[104] + tmpQ2[51]*tmpFx[116] + tmpQ2[52]*tmpFx[128] + tmpQ2[53]*tmpFx[140] + tmpQ2[54]*tmpFx[152] + tmpQ2[55]*tmpFx[164];
tmpQ1[45] = + tmpQ2[42]*tmpFx[9] + tmpQ2[43]*tmpFx[21] + tmpQ2[44]*tmpFx[33] + tmpQ2[45]*tmpFx[45] + tmpQ2[46]*tmpFx[57] + tmpQ2[47]*tmpFx[69] + tmpQ2[48]*tmpFx[81] + tmpQ2[49]*tmpFx[93] + tmpQ2[50]*tmpFx[105] + tmpQ2[51]*tmpFx[117] + tmpQ2[52]*tmpFx[129] + tmpQ2[53]*tmpFx[141] + tmpQ2[54]*tmpFx[153] + tmpQ2[55]*tmpFx[165];
tmpQ1[46] = + tmpQ2[42]*tmpFx[10] + tmpQ2[43]*tmpFx[22] + tmpQ2[44]*tmpFx[34] + tmpQ2[45]*tmpFx[46] + tmpQ2[46]*tmpFx[58] + tmpQ2[47]*tmpFx[70] + tmpQ2[48]*tmpFx[82] + tmpQ2[49]*tmpFx[94] + tmpQ2[50]*tmpFx[106] + tmpQ2[51]*tmpFx[118] + tmpQ2[52]*tmpFx[130] + tmpQ2[53]*tmpFx[142] + tmpQ2[54]*tmpFx[154] + tmpQ2[55]*tmpFx[166];
tmpQ1[47] = + tmpQ2[42]*tmpFx[11] + tmpQ2[43]*tmpFx[23] + tmpQ2[44]*tmpFx[35] + tmpQ2[45]*tmpFx[47] + tmpQ2[46]*tmpFx[59] + tmpQ2[47]*tmpFx[71] + tmpQ2[48]*tmpFx[83] + tmpQ2[49]*tmpFx[95] + tmpQ2[50]*tmpFx[107] + tmpQ2[51]*tmpFx[119] + tmpQ2[52]*tmpFx[131] + tmpQ2[53]*tmpFx[143] + tmpQ2[54]*tmpFx[155] + tmpQ2[55]*tmpFx[167];
tmpQ1[48] = + tmpQ2[56]*tmpFx[0] + tmpQ2[57]*tmpFx[12] + tmpQ2[58]*tmpFx[24] + tmpQ2[59]*tmpFx[36] + tmpQ2[60]*tmpFx[48] + tmpQ2[61]*tmpFx[60] + tmpQ2[62]*tmpFx[72] + tmpQ2[63]*tmpFx[84] + tmpQ2[64]*tmpFx[96] + tmpQ2[65]*tmpFx[108] + tmpQ2[66]*tmpFx[120] + tmpQ2[67]*tmpFx[132] + tmpQ2[68]*tmpFx[144] + tmpQ2[69]*tmpFx[156];
tmpQ1[49] = + tmpQ2[56]*tmpFx[1] + tmpQ2[57]*tmpFx[13] + tmpQ2[58]*tmpFx[25] + tmpQ2[59]*tmpFx[37] + tmpQ2[60]*tmpFx[49] + tmpQ2[61]*tmpFx[61] + tmpQ2[62]*tmpFx[73] + tmpQ2[63]*tmpFx[85] + tmpQ2[64]*tmpFx[97] + tmpQ2[65]*tmpFx[109] + tmpQ2[66]*tmpFx[121] + tmpQ2[67]*tmpFx[133] + tmpQ2[68]*tmpFx[145] + tmpQ2[69]*tmpFx[157];
tmpQ1[50] = + tmpQ2[56]*tmpFx[2] + tmpQ2[57]*tmpFx[14] + tmpQ2[58]*tmpFx[26] + tmpQ2[59]*tmpFx[38] + tmpQ2[60]*tmpFx[50] + tmpQ2[61]*tmpFx[62] + tmpQ2[62]*tmpFx[74] + tmpQ2[63]*tmpFx[86] + tmpQ2[64]*tmpFx[98] + tmpQ2[65]*tmpFx[110] + tmpQ2[66]*tmpFx[122] + tmpQ2[67]*tmpFx[134] + tmpQ2[68]*tmpFx[146] + tmpQ2[69]*tmpFx[158];
tmpQ1[51] = + tmpQ2[56]*tmpFx[3] + tmpQ2[57]*tmpFx[15] + tmpQ2[58]*tmpFx[27] + tmpQ2[59]*tmpFx[39] + tmpQ2[60]*tmpFx[51] + tmpQ2[61]*tmpFx[63] + tmpQ2[62]*tmpFx[75] + tmpQ2[63]*tmpFx[87] + tmpQ2[64]*tmpFx[99] + tmpQ2[65]*tmpFx[111] + tmpQ2[66]*tmpFx[123] + tmpQ2[67]*tmpFx[135] + tmpQ2[68]*tmpFx[147] + tmpQ2[69]*tmpFx[159];
tmpQ1[52] = + tmpQ2[56]*tmpFx[4] + tmpQ2[57]*tmpFx[16] + tmpQ2[58]*tmpFx[28] + tmpQ2[59]*tmpFx[40] + tmpQ2[60]*tmpFx[52] + tmpQ2[61]*tmpFx[64] + tmpQ2[62]*tmpFx[76] + tmpQ2[63]*tmpFx[88] + tmpQ2[64]*tmpFx[100] + tmpQ2[65]*tmpFx[112] + tmpQ2[66]*tmpFx[124] + tmpQ2[67]*tmpFx[136] + tmpQ2[68]*tmpFx[148] + tmpQ2[69]*tmpFx[160];
tmpQ1[53] = + tmpQ2[56]*tmpFx[5] + tmpQ2[57]*tmpFx[17] + tmpQ2[58]*tmpFx[29] + tmpQ2[59]*tmpFx[41] + tmpQ2[60]*tmpFx[53] + tmpQ2[61]*tmpFx[65] + tmpQ2[62]*tmpFx[77] + tmpQ2[63]*tmpFx[89] + tmpQ2[64]*tmpFx[101] + tmpQ2[65]*tmpFx[113] + tmpQ2[66]*tmpFx[125] + tmpQ2[67]*tmpFx[137] + tmpQ2[68]*tmpFx[149] + tmpQ2[69]*tmpFx[161];
tmpQ1[54] = + tmpQ2[56]*tmpFx[6] + tmpQ2[57]*tmpFx[18] + tmpQ2[58]*tmpFx[30] + tmpQ2[59]*tmpFx[42] + tmpQ2[60]*tmpFx[54] + tmpQ2[61]*tmpFx[66] + tmpQ2[62]*tmpFx[78] + tmpQ2[63]*tmpFx[90] + tmpQ2[64]*tmpFx[102] + tmpQ2[65]*tmpFx[114] + tmpQ2[66]*tmpFx[126] + tmpQ2[67]*tmpFx[138] + tmpQ2[68]*tmpFx[150] + tmpQ2[69]*tmpFx[162];
tmpQ1[55] = + tmpQ2[56]*tmpFx[7] + tmpQ2[57]*tmpFx[19] + tmpQ2[58]*tmpFx[31] + tmpQ2[59]*tmpFx[43] + tmpQ2[60]*tmpFx[55] + tmpQ2[61]*tmpFx[67] + tmpQ2[62]*tmpFx[79] + tmpQ2[63]*tmpFx[91] + tmpQ2[64]*tmpFx[103] + tmpQ2[65]*tmpFx[115] + tmpQ2[66]*tmpFx[127] + tmpQ2[67]*tmpFx[139] + tmpQ2[68]*tmpFx[151] + tmpQ2[69]*tmpFx[163];
tmpQ1[56] = + tmpQ2[56]*tmpFx[8] + tmpQ2[57]*tmpFx[20] + tmpQ2[58]*tmpFx[32] + tmpQ2[59]*tmpFx[44] + tmpQ2[60]*tmpFx[56] + tmpQ2[61]*tmpFx[68] + tmpQ2[62]*tmpFx[80] + tmpQ2[63]*tmpFx[92] + tmpQ2[64]*tmpFx[104] + tmpQ2[65]*tmpFx[116] + tmpQ2[66]*tmpFx[128] + tmpQ2[67]*tmpFx[140] + tmpQ2[68]*tmpFx[152] + tmpQ2[69]*tmpFx[164];
tmpQ1[57] = + tmpQ2[56]*tmpFx[9] + tmpQ2[57]*tmpFx[21] + tmpQ2[58]*tmpFx[33] + tmpQ2[59]*tmpFx[45] + tmpQ2[60]*tmpFx[57] + tmpQ2[61]*tmpFx[69] + tmpQ2[62]*tmpFx[81] + tmpQ2[63]*tmpFx[93] + tmpQ2[64]*tmpFx[105] + tmpQ2[65]*tmpFx[117] + tmpQ2[66]*tmpFx[129] + tmpQ2[67]*tmpFx[141] + tmpQ2[68]*tmpFx[153] + tmpQ2[69]*tmpFx[165];
tmpQ1[58] = + tmpQ2[56]*tmpFx[10] + tmpQ2[57]*tmpFx[22] + tmpQ2[58]*tmpFx[34] + tmpQ2[59]*tmpFx[46] + tmpQ2[60]*tmpFx[58] + tmpQ2[61]*tmpFx[70] + tmpQ2[62]*tmpFx[82] + tmpQ2[63]*tmpFx[94] + tmpQ2[64]*tmpFx[106] + tmpQ2[65]*tmpFx[118] + tmpQ2[66]*tmpFx[130] + tmpQ2[67]*tmpFx[142] + tmpQ2[68]*tmpFx[154] + tmpQ2[69]*tmpFx[166];
tmpQ1[59] = + tmpQ2[56]*tmpFx[11] + tmpQ2[57]*tmpFx[23] + tmpQ2[58]*tmpFx[35] + tmpQ2[59]*tmpFx[47] + tmpQ2[60]*tmpFx[59] + tmpQ2[61]*tmpFx[71] + tmpQ2[62]*tmpFx[83] + tmpQ2[63]*tmpFx[95] + tmpQ2[64]*tmpFx[107] + tmpQ2[65]*tmpFx[119] + tmpQ2[66]*tmpFx[131] + tmpQ2[67]*tmpFx[143] + tmpQ2[68]*tmpFx[155] + tmpQ2[69]*tmpFx[167];
tmpQ1[60] = + tmpQ2[70]*tmpFx[0] + tmpQ2[71]*tmpFx[12] + tmpQ2[72]*tmpFx[24] + tmpQ2[73]*tmpFx[36] + tmpQ2[74]*tmpFx[48] + tmpQ2[75]*tmpFx[60] + tmpQ2[76]*tmpFx[72] + tmpQ2[77]*tmpFx[84] + tmpQ2[78]*tmpFx[96] + tmpQ2[79]*tmpFx[108] + tmpQ2[80]*tmpFx[120] + tmpQ2[81]*tmpFx[132] + tmpQ2[82]*tmpFx[144] + tmpQ2[83]*tmpFx[156];
tmpQ1[61] = + tmpQ2[70]*tmpFx[1] + tmpQ2[71]*tmpFx[13] + tmpQ2[72]*tmpFx[25] + tmpQ2[73]*tmpFx[37] + tmpQ2[74]*tmpFx[49] + tmpQ2[75]*tmpFx[61] + tmpQ2[76]*tmpFx[73] + tmpQ2[77]*tmpFx[85] + tmpQ2[78]*tmpFx[97] + tmpQ2[79]*tmpFx[109] + tmpQ2[80]*tmpFx[121] + tmpQ2[81]*tmpFx[133] + tmpQ2[82]*tmpFx[145] + tmpQ2[83]*tmpFx[157];
tmpQ1[62] = + tmpQ2[70]*tmpFx[2] + tmpQ2[71]*tmpFx[14] + tmpQ2[72]*tmpFx[26] + tmpQ2[73]*tmpFx[38] + tmpQ2[74]*tmpFx[50] + tmpQ2[75]*tmpFx[62] + tmpQ2[76]*tmpFx[74] + tmpQ2[77]*tmpFx[86] + tmpQ2[78]*tmpFx[98] + tmpQ2[79]*tmpFx[110] + tmpQ2[80]*tmpFx[122] + tmpQ2[81]*tmpFx[134] + tmpQ2[82]*tmpFx[146] + tmpQ2[83]*tmpFx[158];
tmpQ1[63] = + tmpQ2[70]*tmpFx[3] + tmpQ2[71]*tmpFx[15] + tmpQ2[72]*tmpFx[27] + tmpQ2[73]*tmpFx[39] + tmpQ2[74]*tmpFx[51] + tmpQ2[75]*tmpFx[63] + tmpQ2[76]*tmpFx[75] + tmpQ2[77]*tmpFx[87] + tmpQ2[78]*tmpFx[99] + tmpQ2[79]*tmpFx[111] + tmpQ2[80]*tmpFx[123] + tmpQ2[81]*tmpFx[135] + tmpQ2[82]*tmpFx[147] + tmpQ2[83]*tmpFx[159];
tmpQ1[64] = + tmpQ2[70]*tmpFx[4] + tmpQ2[71]*tmpFx[16] + tmpQ2[72]*tmpFx[28] + tmpQ2[73]*tmpFx[40] + tmpQ2[74]*tmpFx[52] + tmpQ2[75]*tmpFx[64] + tmpQ2[76]*tmpFx[76] + tmpQ2[77]*tmpFx[88] + tmpQ2[78]*tmpFx[100] + tmpQ2[79]*tmpFx[112] + tmpQ2[80]*tmpFx[124] + tmpQ2[81]*tmpFx[136] + tmpQ2[82]*tmpFx[148] + tmpQ2[83]*tmpFx[160];
tmpQ1[65] = + tmpQ2[70]*tmpFx[5] + tmpQ2[71]*tmpFx[17] + tmpQ2[72]*tmpFx[29] + tmpQ2[73]*tmpFx[41] + tmpQ2[74]*tmpFx[53] + tmpQ2[75]*tmpFx[65] + tmpQ2[76]*tmpFx[77] + tmpQ2[77]*tmpFx[89] + tmpQ2[78]*tmpFx[101] + tmpQ2[79]*tmpFx[113] + tmpQ2[80]*tmpFx[125] + tmpQ2[81]*tmpFx[137] + tmpQ2[82]*tmpFx[149] + tmpQ2[83]*tmpFx[161];
tmpQ1[66] = + tmpQ2[70]*tmpFx[6] + tmpQ2[71]*tmpFx[18] + tmpQ2[72]*tmpFx[30] + tmpQ2[73]*tmpFx[42] + tmpQ2[74]*tmpFx[54] + tmpQ2[75]*tmpFx[66] + tmpQ2[76]*tmpFx[78] + tmpQ2[77]*tmpFx[90] + tmpQ2[78]*tmpFx[102] + tmpQ2[79]*tmpFx[114] + tmpQ2[80]*tmpFx[126] + tmpQ2[81]*tmpFx[138] + tmpQ2[82]*tmpFx[150] + tmpQ2[83]*tmpFx[162];
tmpQ1[67] = + tmpQ2[70]*tmpFx[7] + tmpQ2[71]*tmpFx[19] + tmpQ2[72]*tmpFx[31] + tmpQ2[73]*tmpFx[43] + tmpQ2[74]*tmpFx[55] + tmpQ2[75]*tmpFx[67] + tmpQ2[76]*tmpFx[79] + tmpQ2[77]*tmpFx[91] + tmpQ2[78]*tmpFx[103] + tmpQ2[79]*tmpFx[115] + tmpQ2[80]*tmpFx[127] + tmpQ2[81]*tmpFx[139] + tmpQ2[82]*tmpFx[151] + tmpQ2[83]*tmpFx[163];
tmpQ1[68] = + tmpQ2[70]*tmpFx[8] + tmpQ2[71]*tmpFx[20] + tmpQ2[72]*tmpFx[32] + tmpQ2[73]*tmpFx[44] + tmpQ2[74]*tmpFx[56] + tmpQ2[75]*tmpFx[68] + tmpQ2[76]*tmpFx[80] + tmpQ2[77]*tmpFx[92] + tmpQ2[78]*tmpFx[104] + tmpQ2[79]*tmpFx[116] + tmpQ2[80]*tmpFx[128] + tmpQ2[81]*tmpFx[140] + tmpQ2[82]*tmpFx[152] + tmpQ2[83]*tmpFx[164];
tmpQ1[69] = + tmpQ2[70]*tmpFx[9] + tmpQ2[71]*tmpFx[21] + tmpQ2[72]*tmpFx[33] + tmpQ2[73]*tmpFx[45] + tmpQ2[74]*tmpFx[57] + tmpQ2[75]*tmpFx[69] + tmpQ2[76]*tmpFx[81] + tmpQ2[77]*tmpFx[93] + tmpQ2[78]*tmpFx[105] + tmpQ2[79]*tmpFx[117] + tmpQ2[80]*tmpFx[129] + tmpQ2[81]*tmpFx[141] + tmpQ2[82]*tmpFx[153] + tmpQ2[83]*tmpFx[165];
tmpQ1[70] = + tmpQ2[70]*tmpFx[10] + tmpQ2[71]*tmpFx[22] + tmpQ2[72]*tmpFx[34] + tmpQ2[73]*tmpFx[46] + tmpQ2[74]*tmpFx[58] + tmpQ2[75]*tmpFx[70] + tmpQ2[76]*tmpFx[82] + tmpQ2[77]*tmpFx[94] + tmpQ2[78]*tmpFx[106] + tmpQ2[79]*tmpFx[118] + tmpQ2[80]*tmpFx[130] + tmpQ2[81]*tmpFx[142] + tmpQ2[82]*tmpFx[154] + tmpQ2[83]*tmpFx[166];
tmpQ1[71] = + tmpQ2[70]*tmpFx[11] + tmpQ2[71]*tmpFx[23] + tmpQ2[72]*tmpFx[35] + tmpQ2[73]*tmpFx[47] + tmpQ2[74]*tmpFx[59] + tmpQ2[75]*tmpFx[71] + tmpQ2[76]*tmpFx[83] + tmpQ2[77]*tmpFx[95] + tmpQ2[78]*tmpFx[107] + tmpQ2[79]*tmpFx[119] + tmpQ2[80]*tmpFx[131] + tmpQ2[81]*tmpFx[143] + tmpQ2[82]*tmpFx[155] + tmpQ2[83]*tmpFx[167];
tmpQ1[72] = + tmpQ2[84]*tmpFx[0] + tmpQ2[85]*tmpFx[12] + tmpQ2[86]*tmpFx[24] + tmpQ2[87]*tmpFx[36] + tmpQ2[88]*tmpFx[48] + tmpQ2[89]*tmpFx[60] + tmpQ2[90]*tmpFx[72] + tmpQ2[91]*tmpFx[84] + tmpQ2[92]*tmpFx[96] + tmpQ2[93]*tmpFx[108] + tmpQ2[94]*tmpFx[120] + tmpQ2[95]*tmpFx[132] + tmpQ2[96]*tmpFx[144] + tmpQ2[97]*tmpFx[156];
tmpQ1[73] = + tmpQ2[84]*tmpFx[1] + tmpQ2[85]*tmpFx[13] + tmpQ2[86]*tmpFx[25] + tmpQ2[87]*tmpFx[37] + tmpQ2[88]*tmpFx[49] + tmpQ2[89]*tmpFx[61] + tmpQ2[90]*tmpFx[73] + tmpQ2[91]*tmpFx[85] + tmpQ2[92]*tmpFx[97] + tmpQ2[93]*tmpFx[109] + tmpQ2[94]*tmpFx[121] + tmpQ2[95]*tmpFx[133] + tmpQ2[96]*tmpFx[145] + tmpQ2[97]*tmpFx[157];
tmpQ1[74] = + tmpQ2[84]*tmpFx[2] + tmpQ2[85]*tmpFx[14] + tmpQ2[86]*tmpFx[26] + tmpQ2[87]*tmpFx[38] + tmpQ2[88]*tmpFx[50] + tmpQ2[89]*tmpFx[62] + tmpQ2[90]*tmpFx[74] + tmpQ2[91]*tmpFx[86] + tmpQ2[92]*tmpFx[98] + tmpQ2[93]*tmpFx[110] + tmpQ2[94]*tmpFx[122] + tmpQ2[95]*tmpFx[134] + tmpQ2[96]*tmpFx[146] + tmpQ2[97]*tmpFx[158];
tmpQ1[75] = + tmpQ2[84]*tmpFx[3] + tmpQ2[85]*tmpFx[15] + tmpQ2[86]*tmpFx[27] + tmpQ2[87]*tmpFx[39] + tmpQ2[88]*tmpFx[51] + tmpQ2[89]*tmpFx[63] + tmpQ2[90]*tmpFx[75] + tmpQ2[91]*tmpFx[87] + tmpQ2[92]*tmpFx[99] + tmpQ2[93]*tmpFx[111] + tmpQ2[94]*tmpFx[123] + tmpQ2[95]*tmpFx[135] + tmpQ2[96]*tmpFx[147] + tmpQ2[97]*tmpFx[159];
tmpQ1[76] = + tmpQ2[84]*tmpFx[4] + tmpQ2[85]*tmpFx[16] + tmpQ2[86]*tmpFx[28] + tmpQ2[87]*tmpFx[40] + tmpQ2[88]*tmpFx[52] + tmpQ2[89]*tmpFx[64] + tmpQ2[90]*tmpFx[76] + tmpQ2[91]*tmpFx[88] + tmpQ2[92]*tmpFx[100] + tmpQ2[93]*tmpFx[112] + tmpQ2[94]*tmpFx[124] + tmpQ2[95]*tmpFx[136] + tmpQ2[96]*tmpFx[148] + tmpQ2[97]*tmpFx[160];
tmpQ1[77] = + tmpQ2[84]*tmpFx[5] + tmpQ2[85]*tmpFx[17] + tmpQ2[86]*tmpFx[29] + tmpQ2[87]*tmpFx[41] + tmpQ2[88]*tmpFx[53] + tmpQ2[89]*tmpFx[65] + tmpQ2[90]*tmpFx[77] + tmpQ2[91]*tmpFx[89] + tmpQ2[92]*tmpFx[101] + tmpQ2[93]*tmpFx[113] + tmpQ2[94]*tmpFx[125] + tmpQ2[95]*tmpFx[137] + tmpQ2[96]*tmpFx[149] + tmpQ2[97]*tmpFx[161];
tmpQ1[78] = + tmpQ2[84]*tmpFx[6] + tmpQ2[85]*tmpFx[18] + tmpQ2[86]*tmpFx[30] + tmpQ2[87]*tmpFx[42] + tmpQ2[88]*tmpFx[54] + tmpQ2[89]*tmpFx[66] + tmpQ2[90]*tmpFx[78] + tmpQ2[91]*tmpFx[90] + tmpQ2[92]*tmpFx[102] + tmpQ2[93]*tmpFx[114] + tmpQ2[94]*tmpFx[126] + tmpQ2[95]*tmpFx[138] + tmpQ2[96]*tmpFx[150] + tmpQ2[97]*tmpFx[162];
tmpQ1[79] = + tmpQ2[84]*tmpFx[7] + tmpQ2[85]*tmpFx[19] + tmpQ2[86]*tmpFx[31] + tmpQ2[87]*tmpFx[43] + tmpQ2[88]*tmpFx[55] + tmpQ2[89]*tmpFx[67] + tmpQ2[90]*tmpFx[79] + tmpQ2[91]*tmpFx[91] + tmpQ2[92]*tmpFx[103] + tmpQ2[93]*tmpFx[115] + tmpQ2[94]*tmpFx[127] + tmpQ2[95]*tmpFx[139] + tmpQ2[96]*tmpFx[151] + tmpQ2[97]*tmpFx[163];
tmpQ1[80] = + tmpQ2[84]*tmpFx[8] + tmpQ2[85]*tmpFx[20] + tmpQ2[86]*tmpFx[32] + tmpQ2[87]*tmpFx[44] + tmpQ2[88]*tmpFx[56] + tmpQ2[89]*tmpFx[68] + tmpQ2[90]*tmpFx[80] + tmpQ2[91]*tmpFx[92] + tmpQ2[92]*tmpFx[104] + tmpQ2[93]*tmpFx[116] + tmpQ2[94]*tmpFx[128] + tmpQ2[95]*tmpFx[140] + tmpQ2[96]*tmpFx[152] + tmpQ2[97]*tmpFx[164];
tmpQ1[81] = + tmpQ2[84]*tmpFx[9] + tmpQ2[85]*tmpFx[21] + tmpQ2[86]*tmpFx[33] + tmpQ2[87]*tmpFx[45] + tmpQ2[88]*tmpFx[57] + tmpQ2[89]*tmpFx[69] + tmpQ2[90]*tmpFx[81] + tmpQ2[91]*tmpFx[93] + tmpQ2[92]*tmpFx[105] + tmpQ2[93]*tmpFx[117] + tmpQ2[94]*tmpFx[129] + tmpQ2[95]*tmpFx[141] + tmpQ2[96]*tmpFx[153] + tmpQ2[97]*tmpFx[165];
tmpQ1[82] = + tmpQ2[84]*tmpFx[10] + tmpQ2[85]*tmpFx[22] + tmpQ2[86]*tmpFx[34] + tmpQ2[87]*tmpFx[46] + tmpQ2[88]*tmpFx[58] + tmpQ2[89]*tmpFx[70] + tmpQ2[90]*tmpFx[82] + tmpQ2[91]*tmpFx[94] + tmpQ2[92]*tmpFx[106] + tmpQ2[93]*tmpFx[118] + tmpQ2[94]*tmpFx[130] + tmpQ2[95]*tmpFx[142] + tmpQ2[96]*tmpFx[154] + tmpQ2[97]*tmpFx[166];
tmpQ1[83] = + tmpQ2[84]*tmpFx[11] + tmpQ2[85]*tmpFx[23] + tmpQ2[86]*tmpFx[35] + tmpQ2[87]*tmpFx[47] + tmpQ2[88]*tmpFx[59] + tmpQ2[89]*tmpFx[71] + tmpQ2[90]*tmpFx[83] + tmpQ2[91]*tmpFx[95] + tmpQ2[92]*tmpFx[107] + tmpQ2[93]*tmpFx[119] + tmpQ2[94]*tmpFx[131] + tmpQ2[95]*tmpFx[143] + tmpQ2[96]*tmpFx[155] + tmpQ2[97]*tmpFx[167];
tmpQ1[84] = + tmpQ2[98]*tmpFx[0] + tmpQ2[99]*tmpFx[12] + tmpQ2[100]*tmpFx[24] + tmpQ2[101]*tmpFx[36] + tmpQ2[102]*tmpFx[48] + tmpQ2[103]*tmpFx[60] + tmpQ2[104]*tmpFx[72] + tmpQ2[105]*tmpFx[84] + tmpQ2[106]*tmpFx[96] + tmpQ2[107]*tmpFx[108] + tmpQ2[108]*tmpFx[120] + tmpQ2[109]*tmpFx[132] + tmpQ2[110]*tmpFx[144] + tmpQ2[111]*tmpFx[156];
tmpQ1[85] = + tmpQ2[98]*tmpFx[1] + tmpQ2[99]*tmpFx[13] + tmpQ2[100]*tmpFx[25] + tmpQ2[101]*tmpFx[37] + tmpQ2[102]*tmpFx[49] + tmpQ2[103]*tmpFx[61] + tmpQ2[104]*tmpFx[73] + tmpQ2[105]*tmpFx[85] + tmpQ2[106]*tmpFx[97] + tmpQ2[107]*tmpFx[109] + tmpQ2[108]*tmpFx[121] + tmpQ2[109]*tmpFx[133] + tmpQ2[110]*tmpFx[145] + tmpQ2[111]*tmpFx[157];
tmpQ1[86] = + tmpQ2[98]*tmpFx[2] + tmpQ2[99]*tmpFx[14] + tmpQ2[100]*tmpFx[26] + tmpQ2[101]*tmpFx[38] + tmpQ2[102]*tmpFx[50] + tmpQ2[103]*tmpFx[62] + tmpQ2[104]*tmpFx[74] + tmpQ2[105]*tmpFx[86] + tmpQ2[106]*tmpFx[98] + tmpQ2[107]*tmpFx[110] + tmpQ2[108]*tmpFx[122] + tmpQ2[109]*tmpFx[134] + tmpQ2[110]*tmpFx[146] + tmpQ2[111]*tmpFx[158];
tmpQ1[87] = + tmpQ2[98]*tmpFx[3] + tmpQ2[99]*tmpFx[15] + tmpQ2[100]*tmpFx[27] + tmpQ2[101]*tmpFx[39] + tmpQ2[102]*tmpFx[51] + tmpQ2[103]*tmpFx[63] + tmpQ2[104]*tmpFx[75] + tmpQ2[105]*tmpFx[87] + tmpQ2[106]*tmpFx[99] + tmpQ2[107]*tmpFx[111] + tmpQ2[108]*tmpFx[123] + tmpQ2[109]*tmpFx[135] + tmpQ2[110]*tmpFx[147] + tmpQ2[111]*tmpFx[159];
tmpQ1[88] = + tmpQ2[98]*tmpFx[4] + tmpQ2[99]*tmpFx[16] + tmpQ2[100]*tmpFx[28] + tmpQ2[101]*tmpFx[40] + tmpQ2[102]*tmpFx[52] + tmpQ2[103]*tmpFx[64] + tmpQ2[104]*tmpFx[76] + tmpQ2[105]*tmpFx[88] + tmpQ2[106]*tmpFx[100] + tmpQ2[107]*tmpFx[112] + tmpQ2[108]*tmpFx[124] + tmpQ2[109]*tmpFx[136] + tmpQ2[110]*tmpFx[148] + tmpQ2[111]*tmpFx[160];
tmpQ1[89] = + tmpQ2[98]*tmpFx[5] + tmpQ2[99]*tmpFx[17] + tmpQ2[100]*tmpFx[29] + tmpQ2[101]*tmpFx[41] + tmpQ2[102]*tmpFx[53] + tmpQ2[103]*tmpFx[65] + tmpQ2[104]*tmpFx[77] + tmpQ2[105]*tmpFx[89] + tmpQ2[106]*tmpFx[101] + tmpQ2[107]*tmpFx[113] + tmpQ2[108]*tmpFx[125] + tmpQ2[109]*tmpFx[137] + tmpQ2[110]*tmpFx[149] + tmpQ2[111]*tmpFx[161];
tmpQ1[90] = + tmpQ2[98]*tmpFx[6] + tmpQ2[99]*tmpFx[18] + tmpQ2[100]*tmpFx[30] + tmpQ2[101]*tmpFx[42] + tmpQ2[102]*tmpFx[54] + tmpQ2[103]*tmpFx[66] + tmpQ2[104]*tmpFx[78] + tmpQ2[105]*tmpFx[90] + tmpQ2[106]*tmpFx[102] + tmpQ2[107]*tmpFx[114] + tmpQ2[108]*tmpFx[126] + tmpQ2[109]*tmpFx[138] + tmpQ2[110]*tmpFx[150] + tmpQ2[111]*tmpFx[162];
tmpQ1[91] = + tmpQ2[98]*tmpFx[7] + tmpQ2[99]*tmpFx[19] + tmpQ2[100]*tmpFx[31] + tmpQ2[101]*tmpFx[43] + tmpQ2[102]*tmpFx[55] + tmpQ2[103]*tmpFx[67] + tmpQ2[104]*tmpFx[79] + tmpQ2[105]*tmpFx[91] + tmpQ2[106]*tmpFx[103] + tmpQ2[107]*tmpFx[115] + tmpQ2[108]*tmpFx[127] + tmpQ2[109]*tmpFx[139] + tmpQ2[110]*tmpFx[151] + tmpQ2[111]*tmpFx[163];
tmpQ1[92] = + tmpQ2[98]*tmpFx[8] + tmpQ2[99]*tmpFx[20] + tmpQ2[100]*tmpFx[32] + tmpQ2[101]*tmpFx[44] + tmpQ2[102]*tmpFx[56] + tmpQ2[103]*tmpFx[68] + tmpQ2[104]*tmpFx[80] + tmpQ2[105]*tmpFx[92] + tmpQ2[106]*tmpFx[104] + tmpQ2[107]*tmpFx[116] + tmpQ2[108]*tmpFx[128] + tmpQ2[109]*tmpFx[140] + tmpQ2[110]*tmpFx[152] + tmpQ2[111]*tmpFx[164];
tmpQ1[93] = + tmpQ2[98]*tmpFx[9] + tmpQ2[99]*tmpFx[21] + tmpQ2[100]*tmpFx[33] + tmpQ2[101]*tmpFx[45] + tmpQ2[102]*tmpFx[57] + tmpQ2[103]*tmpFx[69] + tmpQ2[104]*tmpFx[81] + tmpQ2[105]*tmpFx[93] + tmpQ2[106]*tmpFx[105] + tmpQ2[107]*tmpFx[117] + tmpQ2[108]*tmpFx[129] + tmpQ2[109]*tmpFx[141] + tmpQ2[110]*tmpFx[153] + tmpQ2[111]*tmpFx[165];
tmpQ1[94] = + tmpQ2[98]*tmpFx[10] + tmpQ2[99]*tmpFx[22] + tmpQ2[100]*tmpFx[34] + tmpQ2[101]*tmpFx[46] + tmpQ2[102]*tmpFx[58] + tmpQ2[103]*tmpFx[70] + tmpQ2[104]*tmpFx[82] + tmpQ2[105]*tmpFx[94] + tmpQ2[106]*tmpFx[106] + tmpQ2[107]*tmpFx[118] + tmpQ2[108]*tmpFx[130] + tmpQ2[109]*tmpFx[142] + tmpQ2[110]*tmpFx[154] + tmpQ2[111]*tmpFx[166];
tmpQ1[95] = + tmpQ2[98]*tmpFx[11] + tmpQ2[99]*tmpFx[23] + tmpQ2[100]*tmpFx[35] + tmpQ2[101]*tmpFx[47] + tmpQ2[102]*tmpFx[59] + tmpQ2[103]*tmpFx[71] + tmpQ2[104]*tmpFx[83] + tmpQ2[105]*tmpFx[95] + tmpQ2[106]*tmpFx[107] + tmpQ2[107]*tmpFx[119] + tmpQ2[108]*tmpFx[131] + tmpQ2[109]*tmpFx[143] + tmpQ2[110]*tmpFx[155] + tmpQ2[111]*tmpFx[167];
tmpQ1[96] = + tmpQ2[112]*tmpFx[0] + tmpQ2[113]*tmpFx[12] + tmpQ2[114]*tmpFx[24] + tmpQ2[115]*tmpFx[36] + tmpQ2[116]*tmpFx[48] + tmpQ2[117]*tmpFx[60] + tmpQ2[118]*tmpFx[72] + tmpQ2[119]*tmpFx[84] + tmpQ2[120]*tmpFx[96] + tmpQ2[121]*tmpFx[108] + tmpQ2[122]*tmpFx[120] + tmpQ2[123]*tmpFx[132] + tmpQ2[124]*tmpFx[144] + tmpQ2[125]*tmpFx[156];
tmpQ1[97] = + tmpQ2[112]*tmpFx[1] + tmpQ2[113]*tmpFx[13] + tmpQ2[114]*tmpFx[25] + tmpQ2[115]*tmpFx[37] + tmpQ2[116]*tmpFx[49] + tmpQ2[117]*tmpFx[61] + tmpQ2[118]*tmpFx[73] + tmpQ2[119]*tmpFx[85] + tmpQ2[120]*tmpFx[97] + tmpQ2[121]*tmpFx[109] + tmpQ2[122]*tmpFx[121] + tmpQ2[123]*tmpFx[133] + tmpQ2[124]*tmpFx[145] + tmpQ2[125]*tmpFx[157];
tmpQ1[98] = + tmpQ2[112]*tmpFx[2] + tmpQ2[113]*tmpFx[14] + tmpQ2[114]*tmpFx[26] + tmpQ2[115]*tmpFx[38] + tmpQ2[116]*tmpFx[50] + tmpQ2[117]*tmpFx[62] + tmpQ2[118]*tmpFx[74] + tmpQ2[119]*tmpFx[86] + tmpQ2[120]*tmpFx[98] + tmpQ2[121]*tmpFx[110] + tmpQ2[122]*tmpFx[122] + tmpQ2[123]*tmpFx[134] + tmpQ2[124]*tmpFx[146] + tmpQ2[125]*tmpFx[158];
tmpQ1[99] = + tmpQ2[112]*tmpFx[3] + tmpQ2[113]*tmpFx[15] + tmpQ2[114]*tmpFx[27] + tmpQ2[115]*tmpFx[39] + tmpQ2[116]*tmpFx[51] + tmpQ2[117]*tmpFx[63] + tmpQ2[118]*tmpFx[75] + tmpQ2[119]*tmpFx[87] + tmpQ2[120]*tmpFx[99] + tmpQ2[121]*tmpFx[111] + tmpQ2[122]*tmpFx[123] + tmpQ2[123]*tmpFx[135] + tmpQ2[124]*tmpFx[147] + tmpQ2[125]*tmpFx[159];
tmpQ1[100] = + tmpQ2[112]*tmpFx[4] + tmpQ2[113]*tmpFx[16] + tmpQ2[114]*tmpFx[28] + tmpQ2[115]*tmpFx[40] + tmpQ2[116]*tmpFx[52] + tmpQ2[117]*tmpFx[64] + tmpQ2[118]*tmpFx[76] + tmpQ2[119]*tmpFx[88] + tmpQ2[120]*tmpFx[100] + tmpQ2[121]*tmpFx[112] + tmpQ2[122]*tmpFx[124] + tmpQ2[123]*tmpFx[136] + tmpQ2[124]*tmpFx[148] + tmpQ2[125]*tmpFx[160];
tmpQ1[101] = + tmpQ2[112]*tmpFx[5] + tmpQ2[113]*tmpFx[17] + tmpQ2[114]*tmpFx[29] + tmpQ2[115]*tmpFx[41] + tmpQ2[116]*tmpFx[53] + tmpQ2[117]*tmpFx[65] + tmpQ2[118]*tmpFx[77] + tmpQ2[119]*tmpFx[89] + tmpQ2[120]*tmpFx[101] + tmpQ2[121]*tmpFx[113] + tmpQ2[122]*tmpFx[125] + tmpQ2[123]*tmpFx[137] + tmpQ2[124]*tmpFx[149] + tmpQ2[125]*tmpFx[161];
tmpQ1[102] = + tmpQ2[112]*tmpFx[6] + tmpQ2[113]*tmpFx[18] + tmpQ2[114]*tmpFx[30] + tmpQ2[115]*tmpFx[42] + tmpQ2[116]*tmpFx[54] + tmpQ2[117]*tmpFx[66] + tmpQ2[118]*tmpFx[78] + tmpQ2[119]*tmpFx[90] + tmpQ2[120]*tmpFx[102] + tmpQ2[121]*tmpFx[114] + tmpQ2[122]*tmpFx[126] + tmpQ2[123]*tmpFx[138] + tmpQ2[124]*tmpFx[150] + tmpQ2[125]*tmpFx[162];
tmpQ1[103] = + tmpQ2[112]*tmpFx[7] + tmpQ2[113]*tmpFx[19] + tmpQ2[114]*tmpFx[31] + tmpQ2[115]*tmpFx[43] + tmpQ2[116]*tmpFx[55] + tmpQ2[117]*tmpFx[67] + tmpQ2[118]*tmpFx[79] + tmpQ2[119]*tmpFx[91] + tmpQ2[120]*tmpFx[103] + tmpQ2[121]*tmpFx[115] + tmpQ2[122]*tmpFx[127] + tmpQ2[123]*tmpFx[139] + tmpQ2[124]*tmpFx[151] + tmpQ2[125]*tmpFx[163];
tmpQ1[104] = + tmpQ2[112]*tmpFx[8] + tmpQ2[113]*tmpFx[20] + tmpQ2[114]*tmpFx[32] + tmpQ2[115]*tmpFx[44] + tmpQ2[116]*tmpFx[56] + tmpQ2[117]*tmpFx[68] + tmpQ2[118]*tmpFx[80] + tmpQ2[119]*tmpFx[92] + tmpQ2[120]*tmpFx[104] + tmpQ2[121]*tmpFx[116] + tmpQ2[122]*tmpFx[128] + tmpQ2[123]*tmpFx[140] + tmpQ2[124]*tmpFx[152] + tmpQ2[125]*tmpFx[164];
tmpQ1[105] = + tmpQ2[112]*tmpFx[9] + tmpQ2[113]*tmpFx[21] + tmpQ2[114]*tmpFx[33] + tmpQ2[115]*tmpFx[45] + tmpQ2[116]*tmpFx[57] + tmpQ2[117]*tmpFx[69] + tmpQ2[118]*tmpFx[81] + tmpQ2[119]*tmpFx[93] + tmpQ2[120]*tmpFx[105] + tmpQ2[121]*tmpFx[117] + tmpQ2[122]*tmpFx[129] + tmpQ2[123]*tmpFx[141] + tmpQ2[124]*tmpFx[153] + tmpQ2[125]*tmpFx[165];
tmpQ1[106] = + tmpQ2[112]*tmpFx[10] + tmpQ2[113]*tmpFx[22] + tmpQ2[114]*tmpFx[34] + tmpQ2[115]*tmpFx[46] + tmpQ2[116]*tmpFx[58] + tmpQ2[117]*tmpFx[70] + tmpQ2[118]*tmpFx[82] + tmpQ2[119]*tmpFx[94] + tmpQ2[120]*tmpFx[106] + tmpQ2[121]*tmpFx[118] + tmpQ2[122]*tmpFx[130] + tmpQ2[123]*tmpFx[142] + tmpQ2[124]*tmpFx[154] + tmpQ2[125]*tmpFx[166];
tmpQ1[107] = + tmpQ2[112]*tmpFx[11] + tmpQ2[113]*tmpFx[23] + tmpQ2[114]*tmpFx[35] + tmpQ2[115]*tmpFx[47] + tmpQ2[116]*tmpFx[59] + tmpQ2[117]*tmpFx[71] + tmpQ2[118]*tmpFx[83] + tmpQ2[119]*tmpFx[95] + tmpQ2[120]*tmpFx[107] + tmpQ2[121]*tmpFx[119] + tmpQ2[122]*tmpFx[131] + tmpQ2[123]*tmpFx[143] + tmpQ2[124]*tmpFx[155] + tmpQ2[125]*tmpFx[167];
tmpQ1[108] = + tmpQ2[126]*tmpFx[0] + tmpQ2[127]*tmpFx[12] + tmpQ2[128]*tmpFx[24] + tmpQ2[129]*tmpFx[36] + tmpQ2[130]*tmpFx[48] + tmpQ2[131]*tmpFx[60] + tmpQ2[132]*tmpFx[72] + tmpQ2[133]*tmpFx[84] + tmpQ2[134]*tmpFx[96] + tmpQ2[135]*tmpFx[108] + tmpQ2[136]*tmpFx[120] + tmpQ2[137]*tmpFx[132] + tmpQ2[138]*tmpFx[144] + tmpQ2[139]*tmpFx[156];
tmpQ1[109] = + tmpQ2[126]*tmpFx[1] + tmpQ2[127]*tmpFx[13] + tmpQ2[128]*tmpFx[25] + tmpQ2[129]*tmpFx[37] + tmpQ2[130]*tmpFx[49] + tmpQ2[131]*tmpFx[61] + tmpQ2[132]*tmpFx[73] + tmpQ2[133]*tmpFx[85] + tmpQ2[134]*tmpFx[97] + tmpQ2[135]*tmpFx[109] + tmpQ2[136]*tmpFx[121] + tmpQ2[137]*tmpFx[133] + tmpQ2[138]*tmpFx[145] + tmpQ2[139]*tmpFx[157];
tmpQ1[110] = + tmpQ2[126]*tmpFx[2] + tmpQ2[127]*tmpFx[14] + tmpQ2[128]*tmpFx[26] + tmpQ2[129]*tmpFx[38] + tmpQ2[130]*tmpFx[50] + tmpQ2[131]*tmpFx[62] + tmpQ2[132]*tmpFx[74] + tmpQ2[133]*tmpFx[86] + tmpQ2[134]*tmpFx[98] + tmpQ2[135]*tmpFx[110] + tmpQ2[136]*tmpFx[122] + tmpQ2[137]*tmpFx[134] + tmpQ2[138]*tmpFx[146] + tmpQ2[139]*tmpFx[158];
tmpQ1[111] = + tmpQ2[126]*tmpFx[3] + tmpQ2[127]*tmpFx[15] + tmpQ2[128]*tmpFx[27] + tmpQ2[129]*tmpFx[39] + tmpQ2[130]*tmpFx[51] + tmpQ2[131]*tmpFx[63] + tmpQ2[132]*tmpFx[75] + tmpQ2[133]*tmpFx[87] + tmpQ2[134]*tmpFx[99] + tmpQ2[135]*tmpFx[111] + tmpQ2[136]*tmpFx[123] + tmpQ2[137]*tmpFx[135] + tmpQ2[138]*tmpFx[147] + tmpQ2[139]*tmpFx[159];
tmpQ1[112] = + tmpQ2[126]*tmpFx[4] + tmpQ2[127]*tmpFx[16] + tmpQ2[128]*tmpFx[28] + tmpQ2[129]*tmpFx[40] + tmpQ2[130]*tmpFx[52] + tmpQ2[131]*tmpFx[64] + tmpQ2[132]*tmpFx[76] + tmpQ2[133]*tmpFx[88] + tmpQ2[134]*tmpFx[100] + tmpQ2[135]*tmpFx[112] + tmpQ2[136]*tmpFx[124] + tmpQ2[137]*tmpFx[136] + tmpQ2[138]*tmpFx[148] + tmpQ2[139]*tmpFx[160];
tmpQ1[113] = + tmpQ2[126]*tmpFx[5] + tmpQ2[127]*tmpFx[17] + tmpQ2[128]*tmpFx[29] + tmpQ2[129]*tmpFx[41] + tmpQ2[130]*tmpFx[53] + tmpQ2[131]*tmpFx[65] + tmpQ2[132]*tmpFx[77] + tmpQ2[133]*tmpFx[89] + tmpQ2[134]*tmpFx[101] + tmpQ2[135]*tmpFx[113] + tmpQ2[136]*tmpFx[125] + tmpQ2[137]*tmpFx[137] + tmpQ2[138]*tmpFx[149] + tmpQ2[139]*tmpFx[161];
tmpQ1[114] = + tmpQ2[126]*tmpFx[6] + tmpQ2[127]*tmpFx[18] + tmpQ2[128]*tmpFx[30] + tmpQ2[129]*tmpFx[42] + tmpQ2[130]*tmpFx[54] + tmpQ2[131]*tmpFx[66] + tmpQ2[132]*tmpFx[78] + tmpQ2[133]*tmpFx[90] + tmpQ2[134]*tmpFx[102] + tmpQ2[135]*tmpFx[114] + tmpQ2[136]*tmpFx[126] + tmpQ2[137]*tmpFx[138] + tmpQ2[138]*tmpFx[150] + tmpQ2[139]*tmpFx[162];
tmpQ1[115] = + tmpQ2[126]*tmpFx[7] + tmpQ2[127]*tmpFx[19] + tmpQ2[128]*tmpFx[31] + tmpQ2[129]*tmpFx[43] + tmpQ2[130]*tmpFx[55] + tmpQ2[131]*tmpFx[67] + tmpQ2[132]*tmpFx[79] + tmpQ2[133]*tmpFx[91] + tmpQ2[134]*tmpFx[103] + tmpQ2[135]*tmpFx[115] + tmpQ2[136]*tmpFx[127] + tmpQ2[137]*tmpFx[139] + tmpQ2[138]*tmpFx[151] + tmpQ2[139]*tmpFx[163];
tmpQ1[116] = + tmpQ2[126]*tmpFx[8] + tmpQ2[127]*tmpFx[20] + tmpQ2[128]*tmpFx[32] + tmpQ2[129]*tmpFx[44] + tmpQ2[130]*tmpFx[56] + tmpQ2[131]*tmpFx[68] + tmpQ2[132]*tmpFx[80] + tmpQ2[133]*tmpFx[92] + tmpQ2[134]*tmpFx[104] + tmpQ2[135]*tmpFx[116] + tmpQ2[136]*tmpFx[128] + tmpQ2[137]*tmpFx[140] + tmpQ2[138]*tmpFx[152] + tmpQ2[139]*tmpFx[164];
tmpQ1[117] = + tmpQ2[126]*tmpFx[9] + tmpQ2[127]*tmpFx[21] + tmpQ2[128]*tmpFx[33] + tmpQ2[129]*tmpFx[45] + tmpQ2[130]*tmpFx[57] + tmpQ2[131]*tmpFx[69] + tmpQ2[132]*tmpFx[81] + tmpQ2[133]*tmpFx[93] + tmpQ2[134]*tmpFx[105] + tmpQ2[135]*tmpFx[117] + tmpQ2[136]*tmpFx[129] + tmpQ2[137]*tmpFx[141] + tmpQ2[138]*tmpFx[153] + tmpQ2[139]*tmpFx[165];
tmpQ1[118] = + tmpQ2[126]*tmpFx[10] + tmpQ2[127]*tmpFx[22] + tmpQ2[128]*tmpFx[34] + tmpQ2[129]*tmpFx[46] + tmpQ2[130]*tmpFx[58] + tmpQ2[131]*tmpFx[70] + tmpQ2[132]*tmpFx[82] + tmpQ2[133]*tmpFx[94] + tmpQ2[134]*tmpFx[106] + tmpQ2[135]*tmpFx[118] + tmpQ2[136]*tmpFx[130] + tmpQ2[137]*tmpFx[142] + tmpQ2[138]*tmpFx[154] + tmpQ2[139]*tmpFx[166];
tmpQ1[119] = + tmpQ2[126]*tmpFx[11] + tmpQ2[127]*tmpFx[23] + tmpQ2[128]*tmpFx[35] + tmpQ2[129]*tmpFx[47] + tmpQ2[130]*tmpFx[59] + tmpQ2[131]*tmpFx[71] + tmpQ2[132]*tmpFx[83] + tmpQ2[133]*tmpFx[95] + tmpQ2[134]*tmpFx[107] + tmpQ2[135]*tmpFx[119] + tmpQ2[136]*tmpFx[131] + tmpQ2[137]*tmpFx[143] + tmpQ2[138]*tmpFx[155] + tmpQ2[139]*tmpFx[167];
tmpQ1[120] = + tmpQ2[140]*tmpFx[0] + tmpQ2[141]*tmpFx[12] + tmpQ2[142]*tmpFx[24] + tmpQ2[143]*tmpFx[36] + tmpQ2[144]*tmpFx[48] + tmpQ2[145]*tmpFx[60] + tmpQ2[146]*tmpFx[72] + tmpQ2[147]*tmpFx[84] + tmpQ2[148]*tmpFx[96] + tmpQ2[149]*tmpFx[108] + tmpQ2[150]*tmpFx[120] + tmpQ2[151]*tmpFx[132] + tmpQ2[152]*tmpFx[144] + tmpQ2[153]*tmpFx[156];
tmpQ1[121] = + tmpQ2[140]*tmpFx[1] + tmpQ2[141]*tmpFx[13] + tmpQ2[142]*tmpFx[25] + tmpQ2[143]*tmpFx[37] + tmpQ2[144]*tmpFx[49] + tmpQ2[145]*tmpFx[61] + tmpQ2[146]*tmpFx[73] + tmpQ2[147]*tmpFx[85] + tmpQ2[148]*tmpFx[97] + tmpQ2[149]*tmpFx[109] + tmpQ2[150]*tmpFx[121] + tmpQ2[151]*tmpFx[133] + tmpQ2[152]*tmpFx[145] + tmpQ2[153]*tmpFx[157];
tmpQ1[122] = + tmpQ2[140]*tmpFx[2] + tmpQ2[141]*tmpFx[14] + tmpQ2[142]*tmpFx[26] + tmpQ2[143]*tmpFx[38] + tmpQ2[144]*tmpFx[50] + tmpQ2[145]*tmpFx[62] + tmpQ2[146]*tmpFx[74] + tmpQ2[147]*tmpFx[86] + tmpQ2[148]*tmpFx[98] + tmpQ2[149]*tmpFx[110] + tmpQ2[150]*tmpFx[122] + tmpQ2[151]*tmpFx[134] + tmpQ2[152]*tmpFx[146] + tmpQ2[153]*tmpFx[158];
tmpQ1[123] = + tmpQ2[140]*tmpFx[3] + tmpQ2[141]*tmpFx[15] + tmpQ2[142]*tmpFx[27] + tmpQ2[143]*tmpFx[39] + tmpQ2[144]*tmpFx[51] + tmpQ2[145]*tmpFx[63] + tmpQ2[146]*tmpFx[75] + tmpQ2[147]*tmpFx[87] + tmpQ2[148]*tmpFx[99] + tmpQ2[149]*tmpFx[111] + tmpQ2[150]*tmpFx[123] + tmpQ2[151]*tmpFx[135] + tmpQ2[152]*tmpFx[147] + tmpQ2[153]*tmpFx[159];
tmpQ1[124] = + tmpQ2[140]*tmpFx[4] + tmpQ2[141]*tmpFx[16] + tmpQ2[142]*tmpFx[28] + tmpQ2[143]*tmpFx[40] + tmpQ2[144]*tmpFx[52] + tmpQ2[145]*tmpFx[64] + tmpQ2[146]*tmpFx[76] + tmpQ2[147]*tmpFx[88] + tmpQ2[148]*tmpFx[100] + tmpQ2[149]*tmpFx[112] + tmpQ2[150]*tmpFx[124] + tmpQ2[151]*tmpFx[136] + tmpQ2[152]*tmpFx[148] + tmpQ2[153]*tmpFx[160];
tmpQ1[125] = + tmpQ2[140]*tmpFx[5] + tmpQ2[141]*tmpFx[17] + tmpQ2[142]*tmpFx[29] + tmpQ2[143]*tmpFx[41] + tmpQ2[144]*tmpFx[53] + tmpQ2[145]*tmpFx[65] + tmpQ2[146]*tmpFx[77] + tmpQ2[147]*tmpFx[89] + tmpQ2[148]*tmpFx[101] + tmpQ2[149]*tmpFx[113] + tmpQ2[150]*tmpFx[125] + tmpQ2[151]*tmpFx[137] + tmpQ2[152]*tmpFx[149] + tmpQ2[153]*tmpFx[161];
tmpQ1[126] = + tmpQ2[140]*tmpFx[6] + tmpQ2[141]*tmpFx[18] + tmpQ2[142]*tmpFx[30] + tmpQ2[143]*tmpFx[42] + tmpQ2[144]*tmpFx[54] + tmpQ2[145]*tmpFx[66] + tmpQ2[146]*tmpFx[78] + tmpQ2[147]*tmpFx[90] + tmpQ2[148]*tmpFx[102] + tmpQ2[149]*tmpFx[114] + tmpQ2[150]*tmpFx[126] + tmpQ2[151]*tmpFx[138] + tmpQ2[152]*tmpFx[150] + tmpQ2[153]*tmpFx[162];
tmpQ1[127] = + tmpQ2[140]*tmpFx[7] + tmpQ2[141]*tmpFx[19] + tmpQ2[142]*tmpFx[31] + tmpQ2[143]*tmpFx[43] + tmpQ2[144]*tmpFx[55] + tmpQ2[145]*tmpFx[67] + tmpQ2[146]*tmpFx[79] + tmpQ2[147]*tmpFx[91] + tmpQ2[148]*tmpFx[103] + tmpQ2[149]*tmpFx[115] + tmpQ2[150]*tmpFx[127] + tmpQ2[151]*tmpFx[139] + tmpQ2[152]*tmpFx[151] + tmpQ2[153]*tmpFx[163];
tmpQ1[128] = + tmpQ2[140]*tmpFx[8] + tmpQ2[141]*tmpFx[20] + tmpQ2[142]*tmpFx[32] + tmpQ2[143]*tmpFx[44] + tmpQ2[144]*tmpFx[56] + tmpQ2[145]*tmpFx[68] + tmpQ2[146]*tmpFx[80] + tmpQ2[147]*tmpFx[92] + tmpQ2[148]*tmpFx[104] + tmpQ2[149]*tmpFx[116] + tmpQ2[150]*tmpFx[128] + tmpQ2[151]*tmpFx[140] + tmpQ2[152]*tmpFx[152] + tmpQ2[153]*tmpFx[164];
tmpQ1[129] = + tmpQ2[140]*tmpFx[9] + tmpQ2[141]*tmpFx[21] + tmpQ2[142]*tmpFx[33] + tmpQ2[143]*tmpFx[45] + tmpQ2[144]*tmpFx[57] + tmpQ2[145]*tmpFx[69] + tmpQ2[146]*tmpFx[81] + tmpQ2[147]*tmpFx[93] + tmpQ2[148]*tmpFx[105] + tmpQ2[149]*tmpFx[117] + tmpQ2[150]*tmpFx[129] + tmpQ2[151]*tmpFx[141] + tmpQ2[152]*tmpFx[153] + tmpQ2[153]*tmpFx[165];
tmpQ1[130] = + tmpQ2[140]*tmpFx[10] + tmpQ2[141]*tmpFx[22] + tmpQ2[142]*tmpFx[34] + tmpQ2[143]*tmpFx[46] + tmpQ2[144]*tmpFx[58] + tmpQ2[145]*tmpFx[70] + tmpQ2[146]*tmpFx[82] + tmpQ2[147]*tmpFx[94] + tmpQ2[148]*tmpFx[106] + tmpQ2[149]*tmpFx[118] + tmpQ2[150]*tmpFx[130] + tmpQ2[151]*tmpFx[142] + tmpQ2[152]*tmpFx[154] + tmpQ2[153]*tmpFx[166];
tmpQ1[131] = + tmpQ2[140]*tmpFx[11] + tmpQ2[141]*tmpFx[23] + tmpQ2[142]*tmpFx[35] + tmpQ2[143]*tmpFx[47] + tmpQ2[144]*tmpFx[59] + tmpQ2[145]*tmpFx[71] + tmpQ2[146]*tmpFx[83] + tmpQ2[147]*tmpFx[95] + tmpQ2[148]*tmpFx[107] + tmpQ2[149]*tmpFx[119] + tmpQ2[150]*tmpFx[131] + tmpQ2[151]*tmpFx[143] + tmpQ2[152]*tmpFx[155] + tmpQ2[153]*tmpFx[167];
tmpQ1[132] = + tmpQ2[154]*tmpFx[0] + tmpQ2[155]*tmpFx[12] + tmpQ2[156]*tmpFx[24] + tmpQ2[157]*tmpFx[36] + tmpQ2[158]*tmpFx[48] + tmpQ2[159]*tmpFx[60] + tmpQ2[160]*tmpFx[72] + tmpQ2[161]*tmpFx[84] + tmpQ2[162]*tmpFx[96] + tmpQ2[163]*tmpFx[108] + tmpQ2[164]*tmpFx[120] + tmpQ2[165]*tmpFx[132] + tmpQ2[166]*tmpFx[144] + tmpQ2[167]*tmpFx[156];
tmpQ1[133] = + tmpQ2[154]*tmpFx[1] + tmpQ2[155]*tmpFx[13] + tmpQ2[156]*tmpFx[25] + tmpQ2[157]*tmpFx[37] + tmpQ2[158]*tmpFx[49] + tmpQ2[159]*tmpFx[61] + tmpQ2[160]*tmpFx[73] + tmpQ2[161]*tmpFx[85] + tmpQ2[162]*tmpFx[97] + tmpQ2[163]*tmpFx[109] + tmpQ2[164]*tmpFx[121] + tmpQ2[165]*tmpFx[133] + tmpQ2[166]*tmpFx[145] + tmpQ2[167]*tmpFx[157];
tmpQ1[134] = + tmpQ2[154]*tmpFx[2] + tmpQ2[155]*tmpFx[14] + tmpQ2[156]*tmpFx[26] + tmpQ2[157]*tmpFx[38] + tmpQ2[158]*tmpFx[50] + tmpQ2[159]*tmpFx[62] + tmpQ2[160]*tmpFx[74] + tmpQ2[161]*tmpFx[86] + tmpQ2[162]*tmpFx[98] + tmpQ2[163]*tmpFx[110] + tmpQ2[164]*tmpFx[122] + tmpQ2[165]*tmpFx[134] + tmpQ2[166]*tmpFx[146] + tmpQ2[167]*tmpFx[158];
tmpQ1[135] = + tmpQ2[154]*tmpFx[3] + tmpQ2[155]*tmpFx[15] + tmpQ2[156]*tmpFx[27] + tmpQ2[157]*tmpFx[39] + tmpQ2[158]*tmpFx[51] + tmpQ2[159]*tmpFx[63] + tmpQ2[160]*tmpFx[75] + tmpQ2[161]*tmpFx[87] + tmpQ2[162]*tmpFx[99] + tmpQ2[163]*tmpFx[111] + tmpQ2[164]*tmpFx[123] + tmpQ2[165]*tmpFx[135] + tmpQ2[166]*tmpFx[147] + tmpQ2[167]*tmpFx[159];
tmpQ1[136] = + tmpQ2[154]*tmpFx[4] + tmpQ2[155]*tmpFx[16] + tmpQ2[156]*tmpFx[28] + tmpQ2[157]*tmpFx[40] + tmpQ2[158]*tmpFx[52] + tmpQ2[159]*tmpFx[64] + tmpQ2[160]*tmpFx[76] + tmpQ2[161]*tmpFx[88] + tmpQ2[162]*tmpFx[100] + tmpQ2[163]*tmpFx[112] + tmpQ2[164]*tmpFx[124] + tmpQ2[165]*tmpFx[136] + tmpQ2[166]*tmpFx[148] + tmpQ2[167]*tmpFx[160];
tmpQ1[137] = + tmpQ2[154]*tmpFx[5] + tmpQ2[155]*tmpFx[17] + tmpQ2[156]*tmpFx[29] + tmpQ2[157]*tmpFx[41] + tmpQ2[158]*tmpFx[53] + tmpQ2[159]*tmpFx[65] + tmpQ2[160]*tmpFx[77] + tmpQ2[161]*tmpFx[89] + tmpQ2[162]*tmpFx[101] + tmpQ2[163]*tmpFx[113] + tmpQ2[164]*tmpFx[125] + tmpQ2[165]*tmpFx[137] + tmpQ2[166]*tmpFx[149] + tmpQ2[167]*tmpFx[161];
tmpQ1[138] = + tmpQ2[154]*tmpFx[6] + tmpQ2[155]*tmpFx[18] + tmpQ2[156]*tmpFx[30] + tmpQ2[157]*tmpFx[42] + tmpQ2[158]*tmpFx[54] + tmpQ2[159]*tmpFx[66] + tmpQ2[160]*tmpFx[78] + tmpQ2[161]*tmpFx[90] + tmpQ2[162]*tmpFx[102] + tmpQ2[163]*tmpFx[114] + tmpQ2[164]*tmpFx[126] + tmpQ2[165]*tmpFx[138] + tmpQ2[166]*tmpFx[150] + tmpQ2[167]*tmpFx[162];
tmpQ1[139] = + tmpQ2[154]*tmpFx[7] + tmpQ2[155]*tmpFx[19] + tmpQ2[156]*tmpFx[31] + tmpQ2[157]*tmpFx[43] + tmpQ2[158]*tmpFx[55] + tmpQ2[159]*tmpFx[67] + tmpQ2[160]*tmpFx[79] + tmpQ2[161]*tmpFx[91] + tmpQ2[162]*tmpFx[103] + tmpQ2[163]*tmpFx[115] + tmpQ2[164]*tmpFx[127] + tmpQ2[165]*tmpFx[139] + tmpQ2[166]*tmpFx[151] + tmpQ2[167]*tmpFx[163];
tmpQ1[140] = + tmpQ2[154]*tmpFx[8] + tmpQ2[155]*tmpFx[20] + tmpQ2[156]*tmpFx[32] + tmpQ2[157]*tmpFx[44] + tmpQ2[158]*tmpFx[56] + tmpQ2[159]*tmpFx[68] + tmpQ2[160]*tmpFx[80] + tmpQ2[161]*tmpFx[92] + tmpQ2[162]*tmpFx[104] + tmpQ2[163]*tmpFx[116] + tmpQ2[164]*tmpFx[128] + tmpQ2[165]*tmpFx[140] + tmpQ2[166]*tmpFx[152] + tmpQ2[167]*tmpFx[164];
tmpQ1[141] = + tmpQ2[154]*tmpFx[9] + tmpQ2[155]*tmpFx[21] + tmpQ2[156]*tmpFx[33] + tmpQ2[157]*tmpFx[45] + tmpQ2[158]*tmpFx[57] + tmpQ2[159]*tmpFx[69] + tmpQ2[160]*tmpFx[81] + tmpQ2[161]*tmpFx[93] + tmpQ2[162]*tmpFx[105] + tmpQ2[163]*tmpFx[117] + tmpQ2[164]*tmpFx[129] + tmpQ2[165]*tmpFx[141] + tmpQ2[166]*tmpFx[153] + tmpQ2[167]*tmpFx[165];
tmpQ1[142] = + tmpQ2[154]*tmpFx[10] + tmpQ2[155]*tmpFx[22] + tmpQ2[156]*tmpFx[34] + tmpQ2[157]*tmpFx[46] + tmpQ2[158]*tmpFx[58] + tmpQ2[159]*tmpFx[70] + tmpQ2[160]*tmpFx[82] + tmpQ2[161]*tmpFx[94] + tmpQ2[162]*tmpFx[106] + tmpQ2[163]*tmpFx[118] + tmpQ2[164]*tmpFx[130] + tmpQ2[165]*tmpFx[142] + tmpQ2[166]*tmpFx[154] + tmpQ2[167]*tmpFx[166];
tmpQ1[143] = + tmpQ2[154]*tmpFx[11] + tmpQ2[155]*tmpFx[23] + tmpQ2[156]*tmpFx[35] + tmpQ2[157]*tmpFx[47] + tmpQ2[158]*tmpFx[59] + tmpQ2[159]*tmpFx[71] + tmpQ2[160]*tmpFx[83] + tmpQ2[161]*tmpFx[95] + tmpQ2[162]*tmpFx[107] + tmpQ2[163]*tmpFx[119] + tmpQ2[164]*tmpFx[131] + tmpQ2[165]*tmpFx[143] + tmpQ2[166]*tmpFx[155] + tmpQ2[167]*tmpFx[167];
}

void nmpc_setObjR1R2( real_t* const tmpFu, real_t* const tmpObjS, real_t* const tmpR1, real_t* const tmpR2 )
{
tmpR2[0] = + tmpFu[0]*tmpObjS[0] + tmpFu[4]*tmpObjS[14] + tmpFu[8]*tmpObjS[28] + tmpFu[12]*tmpObjS[42] + tmpFu[16]*tmpObjS[56] + tmpFu[20]*tmpObjS[70] + tmpFu[24]*tmpObjS[84] + tmpFu[28]*tmpObjS[98] + tmpFu[32]*tmpObjS[112] + tmpFu[36]*tmpObjS[126] + tmpFu[40]*tmpObjS[140] + tmpFu[44]*tmpObjS[154] + tmpFu[48]*tmpObjS[168] + tmpFu[52]*tmpObjS[182];
tmpR2[1] = + tmpFu[0]*tmpObjS[1] + tmpFu[4]*tmpObjS[15] + tmpFu[8]*tmpObjS[29] + tmpFu[12]*tmpObjS[43] + tmpFu[16]*tmpObjS[57] + tmpFu[20]*tmpObjS[71] + tmpFu[24]*tmpObjS[85] + tmpFu[28]*tmpObjS[99] + tmpFu[32]*tmpObjS[113] + tmpFu[36]*tmpObjS[127] + tmpFu[40]*tmpObjS[141] + tmpFu[44]*tmpObjS[155] + tmpFu[48]*tmpObjS[169] + tmpFu[52]*tmpObjS[183];
tmpR2[2] = + tmpFu[0]*tmpObjS[2] + tmpFu[4]*tmpObjS[16] + tmpFu[8]*tmpObjS[30] + tmpFu[12]*tmpObjS[44] + tmpFu[16]*tmpObjS[58] + tmpFu[20]*tmpObjS[72] + tmpFu[24]*tmpObjS[86] + tmpFu[28]*tmpObjS[100] + tmpFu[32]*tmpObjS[114] + tmpFu[36]*tmpObjS[128] + tmpFu[40]*tmpObjS[142] + tmpFu[44]*tmpObjS[156] + tmpFu[48]*tmpObjS[170] + tmpFu[52]*tmpObjS[184];
tmpR2[3] = + tmpFu[0]*tmpObjS[3] + tmpFu[4]*tmpObjS[17] + tmpFu[8]*tmpObjS[31] + tmpFu[12]*tmpObjS[45] + tmpFu[16]*tmpObjS[59] + tmpFu[20]*tmpObjS[73] + tmpFu[24]*tmpObjS[87] + tmpFu[28]*tmpObjS[101] + tmpFu[32]*tmpObjS[115] + tmpFu[36]*tmpObjS[129] + tmpFu[40]*tmpObjS[143] + tmpFu[44]*tmpObjS[157] + tmpFu[48]*tmpObjS[171] + tmpFu[52]*tmpObjS[185];
tmpR2[4] = + tmpFu[0]*tmpObjS[4] + tmpFu[4]*tmpObjS[18] + tmpFu[8]*tmpObjS[32] + tmpFu[12]*tmpObjS[46] + tmpFu[16]*tmpObjS[60] + tmpFu[20]*tmpObjS[74] + tmpFu[24]*tmpObjS[88] + tmpFu[28]*tmpObjS[102] + tmpFu[32]*tmpObjS[116] + tmpFu[36]*tmpObjS[130] + tmpFu[40]*tmpObjS[144] + tmpFu[44]*tmpObjS[158] + tmpFu[48]*tmpObjS[172] + tmpFu[52]*tmpObjS[186];
tmpR2[5] = + tmpFu[0]*tmpObjS[5] + tmpFu[4]*tmpObjS[19] + tmpFu[8]*tmpObjS[33] + tmpFu[12]*tmpObjS[47] + tmpFu[16]*tmpObjS[61] + tmpFu[20]*tmpObjS[75] + tmpFu[24]*tmpObjS[89] + tmpFu[28]*tmpObjS[103] + tmpFu[32]*tmpObjS[117] + tmpFu[36]*tmpObjS[131] + tmpFu[40]*tmpObjS[145] + tmpFu[44]*tmpObjS[159] + tmpFu[48]*tmpObjS[173] + tmpFu[52]*tmpObjS[187];
tmpR2[6] = + tmpFu[0]*tmpObjS[6] + tmpFu[4]*tmpObjS[20] + tmpFu[8]*tmpObjS[34] + tmpFu[12]*tmpObjS[48] + tmpFu[16]*tmpObjS[62] + tmpFu[20]*tmpObjS[76] + tmpFu[24]*tmpObjS[90] + tmpFu[28]*tmpObjS[104] + tmpFu[32]*tmpObjS[118] + tmpFu[36]*tmpObjS[132] + tmpFu[40]*tmpObjS[146] + tmpFu[44]*tmpObjS[160] + tmpFu[48]*tmpObjS[174] + tmpFu[52]*tmpObjS[188];
tmpR2[7] = + tmpFu[0]*tmpObjS[7] + tmpFu[4]*tmpObjS[21] + tmpFu[8]*tmpObjS[35] + tmpFu[12]*tmpObjS[49] + tmpFu[16]*tmpObjS[63] + tmpFu[20]*tmpObjS[77] + tmpFu[24]*tmpObjS[91] + tmpFu[28]*tmpObjS[105] + tmpFu[32]*tmpObjS[119] + tmpFu[36]*tmpObjS[133] + tmpFu[40]*tmpObjS[147] + tmpFu[44]*tmpObjS[161] + tmpFu[48]*tmpObjS[175] + tmpFu[52]*tmpObjS[189];
tmpR2[8] = + tmpFu[0]*tmpObjS[8] + tmpFu[4]*tmpObjS[22] + tmpFu[8]*tmpObjS[36] + tmpFu[12]*tmpObjS[50] + tmpFu[16]*tmpObjS[64] + tmpFu[20]*tmpObjS[78] + tmpFu[24]*tmpObjS[92] + tmpFu[28]*tmpObjS[106] + tmpFu[32]*tmpObjS[120] + tmpFu[36]*tmpObjS[134] + tmpFu[40]*tmpObjS[148] + tmpFu[44]*tmpObjS[162] + tmpFu[48]*tmpObjS[176] + tmpFu[52]*tmpObjS[190];
tmpR2[9] = + tmpFu[0]*tmpObjS[9] + tmpFu[4]*tmpObjS[23] + tmpFu[8]*tmpObjS[37] + tmpFu[12]*tmpObjS[51] + tmpFu[16]*tmpObjS[65] + tmpFu[20]*tmpObjS[79] + tmpFu[24]*tmpObjS[93] + tmpFu[28]*tmpObjS[107] + tmpFu[32]*tmpObjS[121] + tmpFu[36]*tmpObjS[135] + tmpFu[40]*tmpObjS[149] + tmpFu[44]*tmpObjS[163] + tmpFu[48]*tmpObjS[177] + tmpFu[52]*tmpObjS[191];
tmpR2[10] = + tmpFu[0]*tmpObjS[10] + tmpFu[4]*tmpObjS[24] + tmpFu[8]*tmpObjS[38] + tmpFu[12]*tmpObjS[52] + tmpFu[16]*tmpObjS[66] + tmpFu[20]*tmpObjS[80] + tmpFu[24]*tmpObjS[94] + tmpFu[28]*tmpObjS[108] + tmpFu[32]*tmpObjS[122] + tmpFu[36]*tmpObjS[136] + tmpFu[40]*tmpObjS[150] + tmpFu[44]*tmpObjS[164] + tmpFu[48]*tmpObjS[178] + tmpFu[52]*tmpObjS[192];
tmpR2[11] = + tmpFu[0]*tmpObjS[11] + tmpFu[4]*tmpObjS[25] + tmpFu[8]*tmpObjS[39] + tmpFu[12]*tmpObjS[53] + tmpFu[16]*tmpObjS[67] + tmpFu[20]*tmpObjS[81] + tmpFu[24]*tmpObjS[95] + tmpFu[28]*tmpObjS[109] + tmpFu[32]*tmpObjS[123] + tmpFu[36]*tmpObjS[137] + tmpFu[40]*tmpObjS[151] + tmpFu[44]*tmpObjS[165] + tmpFu[48]*tmpObjS[179] + tmpFu[52]*tmpObjS[193];
tmpR2[12] = + tmpFu[0]*tmpObjS[12] + tmpFu[4]*tmpObjS[26] + tmpFu[8]*tmpObjS[40] + tmpFu[12]*tmpObjS[54] + tmpFu[16]*tmpObjS[68] + tmpFu[20]*tmpObjS[82] + tmpFu[24]*tmpObjS[96] + tmpFu[28]*tmpObjS[110] + tmpFu[32]*tmpObjS[124] + tmpFu[36]*tmpObjS[138] + tmpFu[40]*tmpObjS[152] + tmpFu[44]*tmpObjS[166] + tmpFu[48]*tmpObjS[180] + tmpFu[52]*tmpObjS[194];
tmpR2[13] = + tmpFu[0]*tmpObjS[13] + tmpFu[4]*tmpObjS[27] + tmpFu[8]*tmpObjS[41] + tmpFu[12]*tmpObjS[55] + tmpFu[16]*tmpObjS[69] + tmpFu[20]*tmpObjS[83] + tmpFu[24]*tmpObjS[97] + tmpFu[28]*tmpObjS[111] + tmpFu[32]*tmpObjS[125] + tmpFu[36]*tmpObjS[139] + tmpFu[40]*tmpObjS[153] + tmpFu[44]*tmpObjS[167] + tmpFu[48]*tmpObjS[181] + tmpFu[52]*tmpObjS[195];
tmpR2[14] = + tmpFu[1]*tmpObjS[0] + tmpFu[5]*tmpObjS[14] + tmpFu[9]*tmpObjS[28] + tmpFu[13]*tmpObjS[42] + tmpFu[17]*tmpObjS[56] + tmpFu[21]*tmpObjS[70] + tmpFu[25]*tmpObjS[84] + tmpFu[29]*tmpObjS[98] + tmpFu[33]*tmpObjS[112] + tmpFu[37]*tmpObjS[126] + tmpFu[41]*tmpObjS[140] + tmpFu[45]*tmpObjS[154] + tmpFu[49]*tmpObjS[168] + tmpFu[53]*tmpObjS[182];
tmpR2[15] = + tmpFu[1]*tmpObjS[1] + tmpFu[5]*tmpObjS[15] + tmpFu[9]*tmpObjS[29] + tmpFu[13]*tmpObjS[43] + tmpFu[17]*tmpObjS[57] + tmpFu[21]*tmpObjS[71] + tmpFu[25]*tmpObjS[85] + tmpFu[29]*tmpObjS[99] + tmpFu[33]*tmpObjS[113] + tmpFu[37]*tmpObjS[127] + tmpFu[41]*tmpObjS[141] + tmpFu[45]*tmpObjS[155] + tmpFu[49]*tmpObjS[169] + tmpFu[53]*tmpObjS[183];
tmpR2[16] = + tmpFu[1]*tmpObjS[2] + tmpFu[5]*tmpObjS[16] + tmpFu[9]*tmpObjS[30] + tmpFu[13]*tmpObjS[44] + tmpFu[17]*tmpObjS[58] + tmpFu[21]*tmpObjS[72] + tmpFu[25]*tmpObjS[86] + tmpFu[29]*tmpObjS[100] + tmpFu[33]*tmpObjS[114] + tmpFu[37]*tmpObjS[128] + tmpFu[41]*tmpObjS[142] + tmpFu[45]*tmpObjS[156] + tmpFu[49]*tmpObjS[170] + tmpFu[53]*tmpObjS[184];
tmpR2[17] = + tmpFu[1]*tmpObjS[3] + tmpFu[5]*tmpObjS[17] + tmpFu[9]*tmpObjS[31] + tmpFu[13]*tmpObjS[45] + tmpFu[17]*tmpObjS[59] + tmpFu[21]*tmpObjS[73] + tmpFu[25]*tmpObjS[87] + tmpFu[29]*tmpObjS[101] + tmpFu[33]*tmpObjS[115] + tmpFu[37]*tmpObjS[129] + tmpFu[41]*tmpObjS[143] + tmpFu[45]*tmpObjS[157] + tmpFu[49]*tmpObjS[171] + tmpFu[53]*tmpObjS[185];
tmpR2[18] = + tmpFu[1]*tmpObjS[4] + tmpFu[5]*tmpObjS[18] + tmpFu[9]*tmpObjS[32] + tmpFu[13]*tmpObjS[46] + tmpFu[17]*tmpObjS[60] + tmpFu[21]*tmpObjS[74] + tmpFu[25]*tmpObjS[88] + tmpFu[29]*tmpObjS[102] + tmpFu[33]*tmpObjS[116] + tmpFu[37]*tmpObjS[130] + tmpFu[41]*tmpObjS[144] + tmpFu[45]*tmpObjS[158] + tmpFu[49]*tmpObjS[172] + tmpFu[53]*tmpObjS[186];
tmpR2[19] = + tmpFu[1]*tmpObjS[5] + tmpFu[5]*tmpObjS[19] + tmpFu[9]*tmpObjS[33] + tmpFu[13]*tmpObjS[47] + tmpFu[17]*tmpObjS[61] + tmpFu[21]*tmpObjS[75] + tmpFu[25]*tmpObjS[89] + tmpFu[29]*tmpObjS[103] + tmpFu[33]*tmpObjS[117] + tmpFu[37]*tmpObjS[131] + tmpFu[41]*tmpObjS[145] + tmpFu[45]*tmpObjS[159] + tmpFu[49]*tmpObjS[173] + tmpFu[53]*tmpObjS[187];
tmpR2[20] = + tmpFu[1]*tmpObjS[6] + tmpFu[5]*tmpObjS[20] + tmpFu[9]*tmpObjS[34] + tmpFu[13]*tmpObjS[48] + tmpFu[17]*tmpObjS[62] + tmpFu[21]*tmpObjS[76] + tmpFu[25]*tmpObjS[90] + tmpFu[29]*tmpObjS[104] + tmpFu[33]*tmpObjS[118] + tmpFu[37]*tmpObjS[132] + tmpFu[41]*tmpObjS[146] + tmpFu[45]*tmpObjS[160] + tmpFu[49]*tmpObjS[174] + tmpFu[53]*tmpObjS[188];
tmpR2[21] = + tmpFu[1]*tmpObjS[7] + tmpFu[5]*tmpObjS[21] + tmpFu[9]*tmpObjS[35] + tmpFu[13]*tmpObjS[49] + tmpFu[17]*tmpObjS[63] + tmpFu[21]*tmpObjS[77] + tmpFu[25]*tmpObjS[91] + tmpFu[29]*tmpObjS[105] + tmpFu[33]*tmpObjS[119] + tmpFu[37]*tmpObjS[133] + tmpFu[41]*tmpObjS[147] + tmpFu[45]*tmpObjS[161] + tmpFu[49]*tmpObjS[175] + tmpFu[53]*tmpObjS[189];
tmpR2[22] = + tmpFu[1]*tmpObjS[8] + tmpFu[5]*tmpObjS[22] + tmpFu[9]*tmpObjS[36] + tmpFu[13]*tmpObjS[50] + tmpFu[17]*tmpObjS[64] + tmpFu[21]*tmpObjS[78] + tmpFu[25]*tmpObjS[92] + tmpFu[29]*tmpObjS[106] + tmpFu[33]*tmpObjS[120] + tmpFu[37]*tmpObjS[134] + tmpFu[41]*tmpObjS[148] + tmpFu[45]*tmpObjS[162] + tmpFu[49]*tmpObjS[176] + tmpFu[53]*tmpObjS[190];
tmpR2[23] = + tmpFu[1]*tmpObjS[9] + tmpFu[5]*tmpObjS[23] + tmpFu[9]*tmpObjS[37] + tmpFu[13]*tmpObjS[51] + tmpFu[17]*tmpObjS[65] + tmpFu[21]*tmpObjS[79] + tmpFu[25]*tmpObjS[93] + tmpFu[29]*tmpObjS[107] + tmpFu[33]*tmpObjS[121] + tmpFu[37]*tmpObjS[135] + tmpFu[41]*tmpObjS[149] + tmpFu[45]*tmpObjS[163] + tmpFu[49]*tmpObjS[177] + tmpFu[53]*tmpObjS[191];
tmpR2[24] = + tmpFu[1]*tmpObjS[10] + tmpFu[5]*tmpObjS[24] + tmpFu[9]*tmpObjS[38] + tmpFu[13]*tmpObjS[52] + tmpFu[17]*tmpObjS[66] + tmpFu[21]*tmpObjS[80] + tmpFu[25]*tmpObjS[94] + tmpFu[29]*tmpObjS[108] + tmpFu[33]*tmpObjS[122] + tmpFu[37]*tmpObjS[136] + tmpFu[41]*tmpObjS[150] + tmpFu[45]*tmpObjS[164] + tmpFu[49]*tmpObjS[178] + tmpFu[53]*tmpObjS[192];
tmpR2[25] = + tmpFu[1]*tmpObjS[11] + tmpFu[5]*tmpObjS[25] + tmpFu[9]*tmpObjS[39] + tmpFu[13]*tmpObjS[53] + tmpFu[17]*tmpObjS[67] + tmpFu[21]*tmpObjS[81] + tmpFu[25]*tmpObjS[95] + tmpFu[29]*tmpObjS[109] + tmpFu[33]*tmpObjS[123] + tmpFu[37]*tmpObjS[137] + tmpFu[41]*tmpObjS[151] + tmpFu[45]*tmpObjS[165] + tmpFu[49]*tmpObjS[179] + tmpFu[53]*tmpObjS[193];
tmpR2[26] = + tmpFu[1]*tmpObjS[12] + tmpFu[5]*tmpObjS[26] + tmpFu[9]*tmpObjS[40] + tmpFu[13]*tmpObjS[54] + tmpFu[17]*tmpObjS[68] + tmpFu[21]*tmpObjS[82] + tmpFu[25]*tmpObjS[96] + tmpFu[29]*tmpObjS[110] + tmpFu[33]*tmpObjS[124] + tmpFu[37]*tmpObjS[138] + tmpFu[41]*tmpObjS[152] + tmpFu[45]*tmpObjS[166] + tmpFu[49]*tmpObjS[180] + tmpFu[53]*tmpObjS[194];
tmpR2[27] = + tmpFu[1]*tmpObjS[13] + tmpFu[5]*tmpObjS[27] + tmpFu[9]*tmpObjS[41] + tmpFu[13]*tmpObjS[55] + tmpFu[17]*tmpObjS[69] + tmpFu[21]*tmpObjS[83] + tmpFu[25]*tmpObjS[97] + tmpFu[29]*tmpObjS[111] + tmpFu[33]*tmpObjS[125] + tmpFu[37]*tmpObjS[139] + tmpFu[41]*tmpObjS[153] + tmpFu[45]*tmpObjS[167] + tmpFu[49]*tmpObjS[181] + tmpFu[53]*tmpObjS[195];
tmpR2[28] = + tmpFu[2]*tmpObjS[0] + tmpFu[6]*tmpObjS[14] + tmpFu[10]*tmpObjS[28] + tmpFu[14]*tmpObjS[42] + tmpFu[18]*tmpObjS[56] + tmpFu[22]*tmpObjS[70] + tmpFu[26]*tmpObjS[84] + tmpFu[30]*tmpObjS[98] + tmpFu[34]*tmpObjS[112] + tmpFu[38]*tmpObjS[126] + tmpFu[42]*tmpObjS[140] + tmpFu[46]*tmpObjS[154] + tmpFu[50]*tmpObjS[168] + tmpFu[54]*tmpObjS[182];
tmpR2[29] = + tmpFu[2]*tmpObjS[1] + tmpFu[6]*tmpObjS[15] + tmpFu[10]*tmpObjS[29] + tmpFu[14]*tmpObjS[43] + tmpFu[18]*tmpObjS[57] + tmpFu[22]*tmpObjS[71] + tmpFu[26]*tmpObjS[85] + tmpFu[30]*tmpObjS[99] + tmpFu[34]*tmpObjS[113] + tmpFu[38]*tmpObjS[127] + tmpFu[42]*tmpObjS[141] + tmpFu[46]*tmpObjS[155] + tmpFu[50]*tmpObjS[169] + tmpFu[54]*tmpObjS[183];
tmpR2[30] = + tmpFu[2]*tmpObjS[2] + tmpFu[6]*tmpObjS[16] + tmpFu[10]*tmpObjS[30] + tmpFu[14]*tmpObjS[44] + tmpFu[18]*tmpObjS[58] + tmpFu[22]*tmpObjS[72] + tmpFu[26]*tmpObjS[86] + tmpFu[30]*tmpObjS[100] + tmpFu[34]*tmpObjS[114] + tmpFu[38]*tmpObjS[128] + tmpFu[42]*tmpObjS[142] + tmpFu[46]*tmpObjS[156] + tmpFu[50]*tmpObjS[170] + tmpFu[54]*tmpObjS[184];
tmpR2[31] = + tmpFu[2]*tmpObjS[3] + tmpFu[6]*tmpObjS[17] + tmpFu[10]*tmpObjS[31] + tmpFu[14]*tmpObjS[45] + tmpFu[18]*tmpObjS[59] + tmpFu[22]*tmpObjS[73] + tmpFu[26]*tmpObjS[87] + tmpFu[30]*tmpObjS[101] + tmpFu[34]*tmpObjS[115] + tmpFu[38]*tmpObjS[129] + tmpFu[42]*tmpObjS[143] + tmpFu[46]*tmpObjS[157] + tmpFu[50]*tmpObjS[171] + tmpFu[54]*tmpObjS[185];
tmpR2[32] = + tmpFu[2]*tmpObjS[4] + tmpFu[6]*tmpObjS[18] + tmpFu[10]*tmpObjS[32] + tmpFu[14]*tmpObjS[46] + tmpFu[18]*tmpObjS[60] + tmpFu[22]*tmpObjS[74] + tmpFu[26]*tmpObjS[88] + tmpFu[30]*tmpObjS[102] + tmpFu[34]*tmpObjS[116] + tmpFu[38]*tmpObjS[130] + tmpFu[42]*tmpObjS[144] + tmpFu[46]*tmpObjS[158] + tmpFu[50]*tmpObjS[172] + tmpFu[54]*tmpObjS[186];
tmpR2[33] = + tmpFu[2]*tmpObjS[5] + tmpFu[6]*tmpObjS[19] + tmpFu[10]*tmpObjS[33] + tmpFu[14]*tmpObjS[47] + tmpFu[18]*tmpObjS[61] + tmpFu[22]*tmpObjS[75] + tmpFu[26]*tmpObjS[89] + tmpFu[30]*tmpObjS[103] + tmpFu[34]*tmpObjS[117] + tmpFu[38]*tmpObjS[131] + tmpFu[42]*tmpObjS[145] + tmpFu[46]*tmpObjS[159] + tmpFu[50]*tmpObjS[173] + tmpFu[54]*tmpObjS[187];
tmpR2[34] = + tmpFu[2]*tmpObjS[6] + tmpFu[6]*tmpObjS[20] + tmpFu[10]*tmpObjS[34] + tmpFu[14]*tmpObjS[48] + tmpFu[18]*tmpObjS[62] + tmpFu[22]*tmpObjS[76] + tmpFu[26]*tmpObjS[90] + tmpFu[30]*tmpObjS[104] + tmpFu[34]*tmpObjS[118] + tmpFu[38]*tmpObjS[132] + tmpFu[42]*tmpObjS[146] + tmpFu[46]*tmpObjS[160] + tmpFu[50]*tmpObjS[174] + tmpFu[54]*tmpObjS[188];
tmpR2[35] = + tmpFu[2]*tmpObjS[7] + tmpFu[6]*tmpObjS[21] + tmpFu[10]*tmpObjS[35] + tmpFu[14]*tmpObjS[49] + tmpFu[18]*tmpObjS[63] + tmpFu[22]*tmpObjS[77] + tmpFu[26]*tmpObjS[91] + tmpFu[30]*tmpObjS[105] + tmpFu[34]*tmpObjS[119] + tmpFu[38]*tmpObjS[133] + tmpFu[42]*tmpObjS[147] + tmpFu[46]*tmpObjS[161] + tmpFu[50]*tmpObjS[175] + tmpFu[54]*tmpObjS[189];
tmpR2[36] = + tmpFu[2]*tmpObjS[8] + tmpFu[6]*tmpObjS[22] + tmpFu[10]*tmpObjS[36] + tmpFu[14]*tmpObjS[50] + tmpFu[18]*tmpObjS[64] + tmpFu[22]*tmpObjS[78] + tmpFu[26]*tmpObjS[92] + tmpFu[30]*tmpObjS[106] + tmpFu[34]*tmpObjS[120] + tmpFu[38]*tmpObjS[134] + tmpFu[42]*tmpObjS[148] + tmpFu[46]*tmpObjS[162] + tmpFu[50]*tmpObjS[176] + tmpFu[54]*tmpObjS[190];
tmpR2[37] = + tmpFu[2]*tmpObjS[9] + tmpFu[6]*tmpObjS[23] + tmpFu[10]*tmpObjS[37] + tmpFu[14]*tmpObjS[51] + tmpFu[18]*tmpObjS[65] + tmpFu[22]*tmpObjS[79] + tmpFu[26]*tmpObjS[93] + tmpFu[30]*tmpObjS[107] + tmpFu[34]*tmpObjS[121] + tmpFu[38]*tmpObjS[135] + tmpFu[42]*tmpObjS[149] + tmpFu[46]*tmpObjS[163] + tmpFu[50]*tmpObjS[177] + tmpFu[54]*tmpObjS[191];
tmpR2[38] = + tmpFu[2]*tmpObjS[10] + tmpFu[6]*tmpObjS[24] + tmpFu[10]*tmpObjS[38] + tmpFu[14]*tmpObjS[52] + tmpFu[18]*tmpObjS[66] + tmpFu[22]*tmpObjS[80] + tmpFu[26]*tmpObjS[94] + tmpFu[30]*tmpObjS[108] + tmpFu[34]*tmpObjS[122] + tmpFu[38]*tmpObjS[136] + tmpFu[42]*tmpObjS[150] + tmpFu[46]*tmpObjS[164] + tmpFu[50]*tmpObjS[178] + tmpFu[54]*tmpObjS[192];
tmpR2[39] = + tmpFu[2]*tmpObjS[11] + tmpFu[6]*tmpObjS[25] + tmpFu[10]*tmpObjS[39] + tmpFu[14]*tmpObjS[53] + tmpFu[18]*tmpObjS[67] + tmpFu[22]*tmpObjS[81] + tmpFu[26]*tmpObjS[95] + tmpFu[30]*tmpObjS[109] + tmpFu[34]*tmpObjS[123] + tmpFu[38]*tmpObjS[137] + tmpFu[42]*tmpObjS[151] + tmpFu[46]*tmpObjS[165] + tmpFu[50]*tmpObjS[179] + tmpFu[54]*tmpObjS[193];
tmpR2[40] = + tmpFu[2]*tmpObjS[12] + tmpFu[6]*tmpObjS[26] + tmpFu[10]*tmpObjS[40] + tmpFu[14]*tmpObjS[54] + tmpFu[18]*tmpObjS[68] + tmpFu[22]*tmpObjS[82] + tmpFu[26]*tmpObjS[96] + tmpFu[30]*tmpObjS[110] + tmpFu[34]*tmpObjS[124] + tmpFu[38]*tmpObjS[138] + tmpFu[42]*tmpObjS[152] + tmpFu[46]*tmpObjS[166] + tmpFu[50]*tmpObjS[180] + tmpFu[54]*tmpObjS[194];
tmpR2[41] = + tmpFu[2]*tmpObjS[13] + tmpFu[6]*tmpObjS[27] + tmpFu[10]*tmpObjS[41] + tmpFu[14]*tmpObjS[55] + tmpFu[18]*tmpObjS[69] + tmpFu[22]*tmpObjS[83] + tmpFu[26]*tmpObjS[97] + tmpFu[30]*tmpObjS[111] + tmpFu[34]*tmpObjS[125] + tmpFu[38]*tmpObjS[139] + tmpFu[42]*tmpObjS[153] + tmpFu[46]*tmpObjS[167] + tmpFu[50]*tmpObjS[181] + tmpFu[54]*tmpObjS[195];
tmpR2[42] = + tmpFu[3]*tmpObjS[0] + tmpFu[7]*tmpObjS[14] + tmpFu[11]*tmpObjS[28] + tmpFu[15]*tmpObjS[42] + tmpFu[19]*tmpObjS[56] + tmpFu[23]*tmpObjS[70] + tmpFu[27]*tmpObjS[84] + tmpFu[31]*tmpObjS[98] + tmpFu[35]*tmpObjS[112] + tmpFu[39]*tmpObjS[126] + tmpFu[43]*tmpObjS[140] + tmpFu[47]*tmpObjS[154] + tmpFu[51]*tmpObjS[168] + tmpFu[55]*tmpObjS[182];
tmpR2[43] = + tmpFu[3]*tmpObjS[1] + tmpFu[7]*tmpObjS[15] + tmpFu[11]*tmpObjS[29] + tmpFu[15]*tmpObjS[43] + tmpFu[19]*tmpObjS[57] + tmpFu[23]*tmpObjS[71] + tmpFu[27]*tmpObjS[85] + tmpFu[31]*tmpObjS[99] + tmpFu[35]*tmpObjS[113] + tmpFu[39]*tmpObjS[127] + tmpFu[43]*tmpObjS[141] + tmpFu[47]*tmpObjS[155] + tmpFu[51]*tmpObjS[169] + tmpFu[55]*tmpObjS[183];
tmpR2[44] = + tmpFu[3]*tmpObjS[2] + tmpFu[7]*tmpObjS[16] + tmpFu[11]*tmpObjS[30] + tmpFu[15]*tmpObjS[44] + tmpFu[19]*tmpObjS[58] + tmpFu[23]*tmpObjS[72] + tmpFu[27]*tmpObjS[86] + tmpFu[31]*tmpObjS[100] + tmpFu[35]*tmpObjS[114] + tmpFu[39]*tmpObjS[128] + tmpFu[43]*tmpObjS[142] + tmpFu[47]*tmpObjS[156] + tmpFu[51]*tmpObjS[170] + tmpFu[55]*tmpObjS[184];
tmpR2[45] = + tmpFu[3]*tmpObjS[3] + tmpFu[7]*tmpObjS[17] + tmpFu[11]*tmpObjS[31] + tmpFu[15]*tmpObjS[45] + tmpFu[19]*tmpObjS[59] + tmpFu[23]*tmpObjS[73] + tmpFu[27]*tmpObjS[87] + tmpFu[31]*tmpObjS[101] + tmpFu[35]*tmpObjS[115] + tmpFu[39]*tmpObjS[129] + tmpFu[43]*tmpObjS[143] + tmpFu[47]*tmpObjS[157] + tmpFu[51]*tmpObjS[171] + tmpFu[55]*tmpObjS[185];
tmpR2[46] = + tmpFu[3]*tmpObjS[4] + tmpFu[7]*tmpObjS[18] + tmpFu[11]*tmpObjS[32] + tmpFu[15]*tmpObjS[46] + tmpFu[19]*tmpObjS[60] + tmpFu[23]*tmpObjS[74] + tmpFu[27]*tmpObjS[88] + tmpFu[31]*tmpObjS[102] + tmpFu[35]*tmpObjS[116] + tmpFu[39]*tmpObjS[130] + tmpFu[43]*tmpObjS[144] + tmpFu[47]*tmpObjS[158] + tmpFu[51]*tmpObjS[172] + tmpFu[55]*tmpObjS[186];
tmpR2[47] = + tmpFu[3]*tmpObjS[5] + tmpFu[7]*tmpObjS[19] + tmpFu[11]*tmpObjS[33] + tmpFu[15]*tmpObjS[47] + tmpFu[19]*tmpObjS[61] + tmpFu[23]*tmpObjS[75] + tmpFu[27]*tmpObjS[89] + tmpFu[31]*tmpObjS[103] + tmpFu[35]*tmpObjS[117] + tmpFu[39]*tmpObjS[131] + tmpFu[43]*tmpObjS[145] + tmpFu[47]*tmpObjS[159] + tmpFu[51]*tmpObjS[173] + tmpFu[55]*tmpObjS[187];
tmpR2[48] = + tmpFu[3]*tmpObjS[6] + tmpFu[7]*tmpObjS[20] + tmpFu[11]*tmpObjS[34] + tmpFu[15]*tmpObjS[48] + tmpFu[19]*tmpObjS[62] + tmpFu[23]*tmpObjS[76] + tmpFu[27]*tmpObjS[90] + tmpFu[31]*tmpObjS[104] + tmpFu[35]*tmpObjS[118] + tmpFu[39]*tmpObjS[132] + tmpFu[43]*tmpObjS[146] + tmpFu[47]*tmpObjS[160] + tmpFu[51]*tmpObjS[174] + tmpFu[55]*tmpObjS[188];
tmpR2[49] = + tmpFu[3]*tmpObjS[7] + tmpFu[7]*tmpObjS[21] + tmpFu[11]*tmpObjS[35] + tmpFu[15]*tmpObjS[49] + tmpFu[19]*tmpObjS[63] + tmpFu[23]*tmpObjS[77] + tmpFu[27]*tmpObjS[91] + tmpFu[31]*tmpObjS[105] + tmpFu[35]*tmpObjS[119] + tmpFu[39]*tmpObjS[133] + tmpFu[43]*tmpObjS[147] + tmpFu[47]*tmpObjS[161] + tmpFu[51]*tmpObjS[175] + tmpFu[55]*tmpObjS[189];
tmpR2[50] = + tmpFu[3]*tmpObjS[8] + tmpFu[7]*tmpObjS[22] + tmpFu[11]*tmpObjS[36] + tmpFu[15]*tmpObjS[50] + tmpFu[19]*tmpObjS[64] + tmpFu[23]*tmpObjS[78] + tmpFu[27]*tmpObjS[92] + tmpFu[31]*tmpObjS[106] + tmpFu[35]*tmpObjS[120] + tmpFu[39]*tmpObjS[134] + tmpFu[43]*tmpObjS[148] + tmpFu[47]*tmpObjS[162] + tmpFu[51]*tmpObjS[176] + tmpFu[55]*tmpObjS[190];
tmpR2[51] = + tmpFu[3]*tmpObjS[9] + tmpFu[7]*tmpObjS[23] + tmpFu[11]*tmpObjS[37] + tmpFu[15]*tmpObjS[51] + tmpFu[19]*tmpObjS[65] + tmpFu[23]*tmpObjS[79] + tmpFu[27]*tmpObjS[93] + tmpFu[31]*tmpObjS[107] + tmpFu[35]*tmpObjS[121] + tmpFu[39]*tmpObjS[135] + tmpFu[43]*tmpObjS[149] + tmpFu[47]*tmpObjS[163] + tmpFu[51]*tmpObjS[177] + tmpFu[55]*tmpObjS[191];
tmpR2[52] = + tmpFu[3]*tmpObjS[10] + tmpFu[7]*tmpObjS[24] + tmpFu[11]*tmpObjS[38] + tmpFu[15]*tmpObjS[52] + tmpFu[19]*tmpObjS[66] + tmpFu[23]*tmpObjS[80] + tmpFu[27]*tmpObjS[94] + tmpFu[31]*tmpObjS[108] + tmpFu[35]*tmpObjS[122] + tmpFu[39]*tmpObjS[136] + tmpFu[43]*tmpObjS[150] + tmpFu[47]*tmpObjS[164] + tmpFu[51]*tmpObjS[178] + tmpFu[55]*tmpObjS[192];
tmpR2[53] = + tmpFu[3]*tmpObjS[11] + tmpFu[7]*tmpObjS[25] + tmpFu[11]*tmpObjS[39] + tmpFu[15]*tmpObjS[53] + tmpFu[19]*tmpObjS[67] + tmpFu[23]*tmpObjS[81] + tmpFu[27]*tmpObjS[95] + tmpFu[31]*tmpObjS[109] + tmpFu[35]*tmpObjS[123] + tmpFu[39]*tmpObjS[137] + tmpFu[43]*tmpObjS[151] + tmpFu[47]*tmpObjS[165] + tmpFu[51]*tmpObjS[179] + tmpFu[55]*tmpObjS[193];
tmpR2[54] = + tmpFu[3]*tmpObjS[12] + tmpFu[7]*tmpObjS[26] + tmpFu[11]*tmpObjS[40] + tmpFu[15]*tmpObjS[54] + tmpFu[19]*tmpObjS[68] + tmpFu[23]*tmpObjS[82] + tmpFu[27]*tmpObjS[96] + tmpFu[31]*tmpObjS[110] + tmpFu[35]*tmpObjS[124] + tmpFu[39]*tmpObjS[138] + tmpFu[43]*tmpObjS[152] + tmpFu[47]*tmpObjS[166] + tmpFu[51]*tmpObjS[180] + tmpFu[55]*tmpObjS[194];
tmpR2[55] = + tmpFu[3]*tmpObjS[13] + tmpFu[7]*tmpObjS[27] + tmpFu[11]*tmpObjS[41] + tmpFu[15]*tmpObjS[55] + tmpFu[19]*tmpObjS[69] + tmpFu[23]*tmpObjS[83] + tmpFu[27]*tmpObjS[97] + tmpFu[31]*tmpObjS[111] + tmpFu[35]*tmpObjS[125] + tmpFu[39]*tmpObjS[139] + tmpFu[43]*tmpObjS[153] + tmpFu[47]*tmpObjS[167] + tmpFu[51]*tmpObjS[181] + tmpFu[55]*tmpObjS[195];
tmpR1[0] = + tmpR2[0]*tmpFu[0] + tmpR2[1]*tmpFu[4] + tmpR2[2]*tmpFu[8] + tmpR2[3]*tmpFu[12] + tmpR2[4]*tmpFu[16] + tmpR2[5]*tmpFu[20] + tmpR2[6]*tmpFu[24] + tmpR2[7]*tmpFu[28] + tmpR2[8]*tmpFu[32] + tmpR2[9]*tmpFu[36] + tmpR2[10]*tmpFu[40] + tmpR2[11]*tmpFu[44] + tmpR2[12]*tmpFu[48] + tmpR2[13]*tmpFu[52];
tmpR1[1] = + tmpR2[0]*tmpFu[1] + tmpR2[1]*tmpFu[5] + tmpR2[2]*tmpFu[9] + tmpR2[3]*tmpFu[13] + tmpR2[4]*tmpFu[17] + tmpR2[5]*tmpFu[21] + tmpR2[6]*tmpFu[25] + tmpR2[7]*tmpFu[29] + tmpR2[8]*tmpFu[33] + tmpR2[9]*tmpFu[37] + tmpR2[10]*tmpFu[41] + tmpR2[11]*tmpFu[45] + tmpR2[12]*tmpFu[49] + tmpR2[13]*tmpFu[53];
tmpR1[2] = + tmpR2[0]*tmpFu[2] + tmpR2[1]*tmpFu[6] + tmpR2[2]*tmpFu[10] + tmpR2[3]*tmpFu[14] + tmpR2[4]*tmpFu[18] + tmpR2[5]*tmpFu[22] + tmpR2[6]*tmpFu[26] + tmpR2[7]*tmpFu[30] + tmpR2[8]*tmpFu[34] + tmpR2[9]*tmpFu[38] + tmpR2[10]*tmpFu[42] + tmpR2[11]*tmpFu[46] + tmpR2[12]*tmpFu[50] + tmpR2[13]*tmpFu[54];
tmpR1[3] = + tmpR2[0]*tmpFu[3] + tmpR2[1]*tmpFu[7] + tmpR2[2]*tmpFu[11] + tmpR2[3]*tmpFu[15] + tmpR2[4]*tmpFu[19] + tmpR2[5]*tmpFu[23] + tmpR2[6]*tmpFu[27] + tmpR2[7]*tmpFu[31] + tmpR2[8]*tmpFu[35] + tmpR2[9]*tmpFu[39] + tmpR2[10]*tmpFu[43] + tmpR2[11]*tmpFu[47] + tmpR2[12]*tmpFu[51] + tmpR2[13]*tmpFu[55];
tmpR1[4] = + tmpR2[14]*tmpFu[0] + tmpR2[15]*tmpFu[4] + tmpR2[16]*tmpFu[8] + tmpR2[17]*tmpFu[12] + tmpR2[18]*tmpFu[16] + tmpR2[19]*tmpFu[20] + tmpR2[20]*tmpFu[24] + tmpR2[21]*tmpFu[28] + tmpR2[22]*tmpFu[32] + tmpR2[23]*tmpFu[36] + tmpR2[24]*tmpFu[40] + tmpR2[25]*tmpFu[44] + tmpR2[26]*tmpFu[48] + tmpR2[27]*tmpFu[52];
tmpR1[5] = + tmpR2[14]*tmpFu[1] + tmpR2[15]*tmpFu[5] + tmpR2[16]*tmpFu[9] + tmpR2[17]*tmpFu[13] + tmpR2[18]*tmpFu[17] + tmpR2[19]*tmpFu[21] + tmpR2[20]*tmpFu[25] + tmpR2[21]*tmpFu[29] + tmpR2[22]*tmpFu[33] + tmpR2[23]*tmpFu[37] + tmpR2[24]*tmpFu[41] + tmpR2[25]*tmpFu[45] + tmpR2[26]*tmpFu[49] + tmpR2[27]*tmpFu[53];
tmpR1[6] = + tmpR2[14]*tmpFu[2] + tmpR2[15]*tmpFu[6] + tmpR2[16]*tmpFu[10] + tmpR2[17]*tmpFu[14] + tmpR2[18]*tmpFu[18] + tmpR2[19]*tmpFu[22] + tmpR2[20]*tmpFu[26] + tmpR2[21]*tmpFu[30] + tmpR2[22]*tmpFu[34] + tmpR2[23]*tmpFu[38] + tmpR2[24]*tmpFu[42] + tmpR2[25]*tmpFu[46] + tmpR2[26]*tmpFu[50] + tmpR2[27]*tmpFu[54];
tmpR1[7] = + tmpR2[14]*tmpFu[3] + tmpR2[15]*tmpFu[7] + tmpR2[16]*tmpFu[11] + tmpR2[17]*tmpFu[15] + tmpR2[18]*tmpFu[19] + tmpR2[19]*tmpFu[23] + tmpR2[20]*tmpFu[27] + tmpR2[21]*tmpFu[31] + tmpR2[22]*tmpFu[35] + tmpR2[23]*tmpFu[39] + tmpR2[24]*tmpFu[43] + tmpR2[25]*tmpFu[47] + tmpR2[26]*tmpFu[51] + tmpR2[27]*tmpFu[55];
tmpR1[8] = + tmpR2[28]*tmpFu[0] + tmpR2[29]*tmpFu[4] + tmpR2[30]*tmpFu[8] + tmpR2[31]*tmpFu[12] + tmpR2[32]*tmpFu[16] + tmpR2[33]*tmpFu[20] + tmpR2[34]*tmpFu[24] + tmpR2[35]*tmpFu[28] + tmpR2[36]*tmpFu[32] + tmpR2[37]*tmpFu[36] + tmpR2[38]*tmpFu[40] + tmpR2[39]*tmpFu[44] + tmpR2[40]*tmpFu[48] + tmpR2[41]*tmpFu[52];
tmpR1[9] = + tmpR2[28]*tmpFu[1] + tmpR2[29]*tmpFu[5] + tmpR2[30]*tmpFu[9] + tmpR2[31]*tmpFu[13] + tmpR2[32]*tmpFu[17] + tmpR2[33]*tmpFu[21] + tmpR2[34]*tmpFu[25] + tmpR2[35]*tmpFu[29] + tmpR2[36]*tmpFu[33] + tmpR2[37]*tmpFu[37] + tmpR2[38]*tmpFu[41] + tmpR2[39]*tmpFu[45] + tmpR2[40]*tmpFu[49] + tmpR2[41]*tmpFu[53];
tmpR1[10] = + tmpR2[28]*tmpFu[2] + tmpR2[29]*tmpFu[6] + tmpR2[30]*tmpFu[10] + tmpR2[31]*tmpFu[14] + tmpR2[32]*tmpFu[18] + tmpR2[33]*tmpFu[22] + tmpR2[34]*tmpFu[26] + tmpR2[35]*tmpFu[30] + tmpR2[36]*tmpFu[34] + tmpR2[37]*tmpFu[38] + tmpR2[38]*tmpFu[42] + tmpR2[39]*tmpFu[46] + tmpR2[40]*tmpFu[50] + tmpR2[41]*tmpFu[54];
tmpR1[11] = + tmpR2[28]*tmpFu[3] + tmpR2[29]*tmpFu[7] + tmpR2[30]*tmpFu[11] + tmpR2[31]*tmpFu[15] + tmpR2[32]*tmpFu[19] + tmpR2[33]*tmpFu[23] + tmpR2[34]*tmpFu[27] + tmpR2[35]*tmpFu[31] + tmpR2[36]*tmpFu[35] + tmpR2[37]*tmpFu[39] + tmpR2[38]*tmpFu[43] + tmpR2[39]*tmpFu[47] + tmpR2[40]*tmpFu[51] + tmpR2[41]*tmpFu[55];
tmpR1[12] = + tmpR2[42]*tmpFu[0] + tmpR2[43]*tmpFu[4] + tmpR2[44]*tmpFu[8] + tmpR2[45]*tmpFu[12] + tmpR2[46]*tmpFu[16] + tmpR2[47]*tmpFu[20] + tmpR2[48]*tmpFu[24] + tmpR2[49]*tmpFu[28] + tmpR2[50]*tmpFu[32] + tmpR2[51]*tmpFu[36] + tmpR2[52]*tmpFu[40] + tmpR2[53]*tmpFu[44] + tmpR2[54]*tmpFu[48] + tmpR2[55]*tmpFu[52];
tmpR1[13] = + tmpR2[42]*tmpFu[1] + tmpR2[43]*tmpFu[5] + tmpR2[44]*tmpFu[9] + tmpR2[45]*tmpFu[13] + tmpR2[46]*tmpFu[17] + tmpR2[47]*tmpFu[21] + tmpR2[48]*tmpFu[25] + tmpR2[49]*tmpFu[29] + tmpR2[50]*tmpFu[33] + tmpR2[51]*tmpFu[37] + tmpR2[52]*tmpFu[41] + tmpR2[53]*tmpFu[45] + tmpR2[54]*tmpFu[49] + tmpR2[55]*tmpFu[53];
tmpR1[14] = + tmpR2[42]*tmpFu[2] + tmpR2[43]*tmpFu[6] + tmpR2[44]*tmpFu[10] + tmpR2[45]*tmpFu[14] + tmpR2[46]*tmpFu[18] + tmpR2[47]*tmpFu[22] + tmpR2[48]*tmpFu[26] + tmpR2[49]*tmpFu[30] + tmpR2[50]*tmpFu[34] + tmpR2[51]*tmpFu[38] + tmpR2[52]*tmpFu[42] + tmpR2[53]*tmpFu[46] + tmpR2[54]*tmpFu[50] + tmpR2[55]*tmpFu[54];
tmpR1[15] = + tmpR2[42]*tmpFu[3] + tmpR2[43]*tmpFu[7] + tmpR2[44]*tmpFu[11] + tmpR2[45]*tmpFu[15] + tmpR2[46]*tmpFu[19] + tmpR2[47]*tmpFu[23] + tmpR2[48]*tmpFu[27] + tmpR2[49]*tmpFu[31] + tmpR2[50]*tmpFu[35] + tmpR2[51]*tmpFu[39] + tmpR2[52]*tmpFu[43] + tmpR2[53]*tmpFu[47] + tmpR2[54]*tmpFu[51] + tmpR2[55]*tmpFu[55];
}

void nmpc_setObjQN1QN2( real_t* const tmpObjSEndTerm, real_t* const tmpQN1, real_t* const tmpQN2 )
{
tmpQN2[0] = +tmpObjSEndTerm[0];
tmpQN2[1] = +tmpObjSEndTerm[1];
tmpQN2[2] = +tmpObjSEndTerm[2];
tmpQN2[3] = +tmpObjSEndTerm[3];
tmpQN2[4] = +tmpObjSEndTerm[4];
tmpQN2[5] = +tmpObjSEndTerm[5];
tmpQN2[6] = +tmpObjSEndTerm[6];
tmpQN2[7] = +tmpObjSEndTerm[7];
tmpQN2[8] = +tmpObjSEndTerm[8];
tmpQN2[9] = +tmpObjSEndTerm[9];
tmpQN2[10] = +tmpObjSEndTerm[10];
tmpQN2[11] = +tmpObjSEndTerm[11];
tmpQN2[12] = +tmpObjSEndTerm[12];
tmpQN2[13] = +tmpObjSEndTerm[13];
tmpQN2[14] = +tmpObjSEndTerm[14];
tmpQN2[15] = +tmpObjSEndTerm[15];
tmpQN2[16] = +tmpObjSEndTerm[16];
tmpQN2[17] = +tmpObjSEndTerm[17];
tmpQN2[18] = +tmpObjSEndTerm[18];
tmpQN2[19] = +tmpObjSEndTerm[19];
tmpQN2[20] = +tmpObjSEndTerm[20];
tmpQN2[21] = +tmpObjSEndTerm[21];
tmpQN2[22] = +tmpObjSEndTerm[22];
tmpQN2[23] = +tmpObjSEndTerm[23];
tmpQN2[24] = +tmpObjSEndTerm[24];
tmpQN2[25] = +tmpObjSEndTerm[25];
tmpQN2[26] = +tmpObjSEndTerm[26];
tmpQN2[27] = +tmpObjSEndTerm[27];
tmpQN2[28] = +tmpObjSEndTerm[28];
tmpQN2[29] = +tmpObjSEndTerm[29];
tmpQN2[30] = +tmpObjSEndTerm[30];
tmpQN2[31] = +tmpObjSEndTerm[31];
tmpQN2[32] = +tmpObjSEndTerm[32];
tmpQN2[33] = +tmpObjSEndTerm[33];
tmpQN2[34] = +tmpObjSEndTerm[34];
tmpQN2[35] = +tmpObjSEndTerm[35];
tmpQN2[36] = 0.0;
;
tmpQN2[37] = 0.0;
;
tmpQN2[38] = 0.0;
;
tmpQN2[39] = 0.0;
;
tmpQN2[40] = 0.0;
;
tmpQN2[41] = 0.0;
;
tmpQN2[42] = 0.0;
;
tmpQN2[43] = 0.0;
;
tmpQN2[44] = 0.0;
;
tmpQN2[45] = 0.0;
;
tmpQN2[46] = 0.0;
;
tmpQN2[47] = 0.0;
;
tmpQN2[48] = 0.0;
;
tmpQN2[49] = 0.0;
;
tmpQN2[50] = 0.0;
;
tmpQN2[51] = 0.0;
;
tmpQN2[52] = 0.0;
;
tmpQN2[53] = 0.0;
;
tmpQN2[54] = 0.0;
;
tmpQN2[55] = 0.0;
;
tmpQN2[56] = 0.0;
;
tmpQN2[57] = 0.0;
;
tmpQN2[58] = 0.0;
;
tmpQN2[59] = 0.0;
;
tmpQN2[60] = 0.0;
;
tmpQN2[61] = 0.0;
;
tmpQN2[62] = 0.0;
;
tmpQN2[63] = 0.0;
;
tmpQN2[64] = 0.0;
;
tmpQN2[65] = 0.0;
;
tmpQN2[66] = 0.0;
;
tmpQN2[67] = 0.0;
;
tmpQN2[68] = 0.0;
;
tmpQN2[69] = 0.0;
;
tmpQN2[70] = 0.0;
;
tmpQN2[71] = 0.0;
;
tmpQN1[0] = + tmpQN2[0];
tmpQN1[1] = + tmpQN2[1];
tmpQN1[2] = + tmpQN2[2];
tmpQN1[3] = + tmpQN2[3];
tmpQN1[4] = + tmpQN2[4];
tmpQN1[5] = + tmpQN2[5];
tmpQN1[6] = 0.0;
;
tmpQN1[7] = 0.0;
;
tmpQN1[8] = 0.0;
;
tmpQN1[9] = 0.0;
;
tmpQN1[10] = 0.0;
;
tmpQN1[11] = 0.0;
;
tmpQN1[12] = + tmpQN2[6];
tmpQN1[13] = + tmpQN2[7];
tmpQN1[14] = + tmpQN2[8];
tmpQN1[15] = + tmpQN2[9];
tmpQN1[16] = + tmpQN2[10];
tmpQN1[17] = + tmpQN2[11];
tmpQN1[18] = 0.0;
;
tmpQN1[19] = 0.0;
;
tmpQN1[20] = 0.0;
;
tmpQN1[21] = 0.0;
;
tmpQN1[22] = 0.0;
;
tmpQN1[23] = 0.0;
;
tmpQN1[24] = + tmpQN2[12];
tmpQN1[25] = + tmpQN2[13];
tmpQN1[26] = + tmpQN2[14];
tmpQN1[27] = + tmpQN2[15];
tmpQN1[28] = + tmpQN2[16];
tmpQN1[29] = + tmpQN2[17];
tmpQN1[30] = 0.0;
;
tmpQN1[31] = 0.0;
;
tmpQN1[32] = 0.0;
;
tmpQN1[33] = 0.0;
;
tmpQN1[34] = 0.0;
;
tmpQN1[35] = 0.0;
;
tmpQN1[36] = + tmpQN2[18];
tmpQN1[37] = + tmpQN2[19];
tmpQN1[38] = + tmpQN2[20];
tmpQN1[39] = + tmpQN2[21];
tmpQN1[40] = + tmpQN2[22];
tmpQN1[41] = + tmpQN2[23];
tmpQN1[42] = 0.0;
;
tmpQN1[43] = 0.0;
;
tmpQN1[44] = 0.0;
;
tmpQN1[45] = 0.0;
;
tmpQN1[46] = 0.0;
;
tmpQN1[47] = 0.0;
;
tmpQN1[48] = + tmpQN2[24];
tmpQN1[49] = + tmpQN2[25];
tmpQN1[50] = + tmpQN2[26];
tmpQN1[51] = + tmpQN2[27];
tmpQN1[52] = + tmpQN2[28];
tmpQN1[53] = + tmpQN2[29];
tmpQN1[54] = 0.0;
;
tmpQN1[55] = 0.0;
;
tmpQN1[56] = 0.0;
;
tmpQN1[57] = 0.0;
;
tmpQN1[58] = 0.0;
;
tmpQN1[59] = 0.0;
;
tmpQN1[60] = + tmpQN2[30];
tmpQN1[61] = + tmpQN2[31];
tmpQN1[62] = + tmpQN2[32];
tmpQN1[63] = + tmpQN2[33];
tmpQN1[64] = + tmpQN2[34];
tmpQN1[65] = + tmpQN2[35];
tmpQN1[66] = 0.0;
;
tmpQN1[67] = 0.0;
;
tmpQN1[68] = 0.0;
;
tmpQN1[69] = 0.0;
;
tmpQN1[70] = 0.0;
;
tmpQN1[71] = 0.0;
;
tmpQN1[72] = + tmpQN2[36];
tmpQN1[73] = + tmpQN2[37];
tmpQN1[74] = + tmpQN2[38];
tmpQN1[75] = + tmpQN2[39];
tmpQN1[76] = + tmpQN2[40];
tmpQN1[77] = + tmpQN2[41];
tmpQN1[78] = 0.0;
;
tmpQN1[79] = 0.0;
;
tmpQN1[80] = 0.0;
;
tmpQN1[81] = 0.0;
;
tmpQN1[82] = 0.0;
;
tmpQN1[83] = 0.0;
;
tmpQN1[84] = + tmpQN2[42];
tmpQN1[85] = + tmpQN2[43];
tmpQN1[86] = + tmpQN2[44];
tmpQN1[87] = + tmpQN2[45];
tmpQN1[88] = + tmpQN2[46];
tmpQN1[89] = + tmpQN2[47];
tmpQN1[90] = 0.0;
;
tmpQN1[91] = 0.0;
;
tmpQN1[92] = 0.0;
;
tmpQN1[93] = 0.0;
;
tmpQN1[94] = 0.0;
;
tmpQN1[95] = 0.0;
;
tmpQN1[96] = + tmpQN2[48];
tmpQN1[97] = + tmpQN2[49];
tmpQN1[98] = + tmpQN2[50];
tmpQN1[99] = + tmpQN2[51];
tmpQN1[100] = + tmpQN2[52];
tmpQN1[101] = + tmpQN2[53];
tmpQN1[102] = 0.0;
;
tmpQN1[103] = 0.0;
;
tmpQN1[104] = 0.0;
;
tmpQN1[105] = 0.0;
;
tmpQN1[106] = 0.0;
;
tmpQN1[107] = 0.0;
;
tmpQN1[108] = + tmpQN2[54];
tmpQN1[109] = + tmpQN2[55];
tmpQN1[110] = + tmpQN2[56];
tmpQN1[111] = + tmpQN2[57];
tmpQN1[112] = + tmpQN2[58];
tmpQN1[113] = + tmpQN2[59];
tmpQN1[114] = 0.0;
;
tmpQN1[115] = 0.0;
;
tmpQN1[116] = 0.0;
;
tmpQN1[117] = 0.0;
;
tmpQN1[118] = 0.0;
;
tmpQN1[119] = 0.0;
;
tmpQN1[120] = + tmpQN2[60];
tmpQN1[121] = + tmpQN2[61];
tmpQN1[122] = + tmpQN2[62];
tmpQN1[123] = + tmpQN2[63];
tmpQN1[124] = + tmpQN2[64];
tmpQN1[125] = + tmpQN2[65];
tmpQN1[126] = 0.0;
;
tmpQN1[127] = 0.0;
;
tmpQN1[128] = 0.0;
;
tmpQN1[129] = 0.0;
;
tmpQN1[130] = 0.0;
;
tmpQN1[131] = 0.0;
;
tmpQN1[132] = + tmpQN2[66];
tmpQN1[133] = + tmpQN2[67];
tmpQN1[134] = + tmpQN2[68];
tmpQN1[135] = + tmpQN2[69];
tmpQN1[136] = + tmpQN2[70];
tmpQN1[137] = + tmpQN2[71];
tmpQN1[138] = 0.0;
;
tmpQN1[139] = 0.0;
;
tmpQN1[140] = 0.0;
;
tmpQN1[141] = 0.0;
;
tmpQN1[142] = 0.0;
;
tmpQN1[143] = 0.0;
;
}

void nmpc_evaluateObjective(  )
{
int runObj;
for (runObj = 0; runObj < 30; ++runObj)
{
nmpcWorkspace.objValueIn[0] = nmpcVariables.x[runObj * 12];
nmpcWorkspace.objValueIn[1] = nmpcVariables.x[runObj * 12 + 1];
nmpcWorkspace.objValueIn[2] = nmpcVariables.x[runObj * 12 + 2];
nmpcWorkspace.objValueIn[3] = nmpcVariables.x[runObj * 12 + 3];
nmpcWorkspace.objValueIn[4] = nmpcVariables.x[runObj * 12 + 4];
nmpcWorkspace.objValueIn[5] = nmpcVariables.x[runObj * 12 + 5];
nmpcWorkspace.objValueIn[6] = nmpcVariables.x[runObj * 12 + 6];
nmpcWorkspace.objValueIn[7] = nmpcVariables.x[runObj * 12 + 7];
nmpcWorkspace.objValueIn[8] = nmpcVariables.x[runObj * 12 + 8];
nmpcWorkspace.objValueIn[9] = nmpcVariables.x[runObj * 12 + 9];
nmpcWorkspace.objValueIn[10] = nmpcVariables.x[runObj * 12 + 10];
nmpcWorkspace.objValueIn[11] = nmpcVariables.x[runObj * 12 + 11];
nmpcWorkspace.objValueIn[12] = nmpcVariables.u[runObj * 4];
nmpcWorkspace.objValueIn[13] = nmpcVariables.u[runObj * 4 + 1];
nmpcWorkspace.objValueIn[14] = nmpcVariables.u[runObj * 4 + 2];
nmpcWorkspace.objValueIn[15] = nmpcVariables.u[runObj * 4 + 3];
nmpcWorkspace.objValueIn[16] = nmpcVariables.od[runObj * 12];
nmpcWorkspace.objValueIn[17] = nmpcVariables.od[runObj * 12 + 1];
nmpcWorkspace.objValueIn[18] = nmpcVariables.od[runObj * 12 + 2];
nmpcWorkspace.objValueIn[19] = nmpcVariables.od[runObj * 12 + 3];
nmpcWorkspace.objValueIn[20] = nmpcVariables.od[runObj * 12 + 4];
nmpcWorkspace.objValueIn[21] = nmpcVariables.od[runObj * 12 + 5];
nmpcWorkspace.objValueIn[22] = nmpcVariables.od[runObj * 12 + 6];
nmpcWorkspace.objValueIn[23] = nmpcVariables.od[runObj * 12 + 7];
nmpcWorkspace.objValueIn[24] = nmpcVariables.od[runObj * 12 + 8];
nmpcWorkspace.objValueIn[25] = nmpcVariables.od[runObj * 12 + 9];
nmpcWorkspace.objValueIn[26] = nmpcVariables.od[runObj * 12 + 10];
nmpcWorkspace.objValueIn[27] = nmpcVariables.od[runObj * 12 + 11];

nmpc_evaluateLSQ( nmpcWorkspace.objValueIn, nmpcWorkspace.objValueOut );
nmpcWorkspace.Dy[runObj * 14] = nmpcWorkspace.objValueOut[0];
nmpcWorkspace.Dy[runObj * 14 + 1] = nmpcWorkspace.objValueOut[1];
nmpcWorkspace.Dy[runObj * 14 + 2] = nmpcWorkspace.objValueOut[2];
nmpcWorkspace.Dy[runObj * 14 + 3] = nmpcWorkspace.objValueOut[3];
nmpcWorkspace.Dy[runObj * 14 + 4] = nmpcWorkspace.objValueOut[4];
nmpcWorkspace.Dy[runObj * 14 + 5] = nmpcWorkspace.objValueOut[5];
nmpcWorkspace.Dy[runObj * 14 + 6] = nmpcWorkspace.objValueOut[6];
nmpcWorkspace.Dy[runObj * 14 + 7] = nmpcWorkspace.objValueOut[7];
nmpcWorkspace.Dy[runObj * 14 + 8] = nmpcWorkspace.objValueOut[8];
nmpcWorkspace.Dy[runObj * 14 + 9] = nmpcWorkspace.objValueOut[9];
nmpcWorkspace.Dy[runObj * 14 + 10] = nmpcWorkspace.objValueOut[10];
nmpcWorkspace.Dy[runObj * 14 + 11] = nmpcWorkspace.objValueOut[11];
nmpcWorkspace.Dy[runObj * 14 + 12] = nmpcWorkspace.objValueOut[12];
nmpcWorkspace.Dy[runObj * 14 + 13] = nmpcWorkspace.objValueOut[13];

nmpc_setObjQ1Q2( &(nmpcWorkspace.objValueOut[ 14 ]), nmpcVariables.W, &(nmpcWorkspace.Q1[ runObj * 144 ]), &(nmpcWorkspace.Q2[ runObj * 168 ]) );

nmpc_setObjR1R2( &(nmpcWorkspace.objValueOut[ 182 ]), nmpcVariables.W, &(nmpcWorkspace.R1[ runObj * 16 ]), &(nmpcWorkspace.R2[ runObj * 56 ]) );

}
nmpcWorkspace.objValueIn[0] = nmpcVariables.x[360];
nmpcWorkspace.objValueIn[1] = nmpcVariables.x[361];
nmpcWorkspace.objValueIn[2] = nmpcVariables.x[362];
nmpcWorkspace.objValueIn[3] = nmpcVariables.x[363];
nmpcWorkspace.objValueIn[4] = nmpcVariables.x[364];
nmpcWorkspace.objValueIn[5] = nmpcVariables.x[365];
nmpcWorkspace.objValueIn[6] = nmpcVariables.x[366];
nmpcWorkspace.objValueIn[7] = nmpcVariables.x[367];
nmpcWorkspace.objValueIn[8] = nmpcVariables.x[368];
nmpcWorkspace.objValueIn[9] = nmpcVariables.x[369];
nmpcWorkspace.objValueIn[10] = nmpcVariables.x[370];
nmpcWorkspace.objValueIn[11] = nmpcVariables.x[371];
nmpcWorkspace.objValueIn[12] = nmpcVariables.od[360];
nmpcWorkspace.objValueIn[13] = nmpcVariables.od[361];
nmpcWorkspace.objValueIn[14] = nmpcVariables.od[362];
nmpcWorkspace.objValueIn[15] = nmpcVariables.od[363];
nmpcWorkspace.objValueIn[16] = nmpcVariables.od[364];
nmpcWorkspace.objValueIn[17] = nmpcVariables.od[365];
nmpcWorkspace.objValueIn[18] = nmpcVariables.od[366];
nmpcWorkspace.objValueIn[19] = nmpcVariables.od[367];
nmpcWorkspace.objValueIn[20] = nmpcVariables.od[368];
nmpcWorkspace.objValueIn[21] = nmpcVariables.od[369];
nmpcWorkspace.objValueIn[22] = nmpcVariables.od[370];
nmpcWorkspace.objValueIn[23] = nmpcVariables.od[371];
nmpc_evaluateLSQEndTerm( nmpcWorkspace.objValueIn, nmpcWorkspace.objValueOut );

nmpcWorkspace.DyN[0] = nmpcWorkspace.objValueOut[0];
nmpcWorkspace.DyN[1] = nmpcWorkspace.objValueOut[1];
nmpcWorkspace.DyN[2] = nmpcWorkspace.objValueOut[2];
nmpcWorkspace.DyN[3] = nmpcWorkspace.objValueOut[3];
nmpcWorkspace.DyN[4] = nmpcWorkspace.objValueOut[4];
nmpcWorkspace.DyN[5] = nmpcWorkspace.objValueOut[5];

nmpc_setObjQN1QN2( nmpcVariables.WN, nmpcWorkspace.QN1, nmpcWorkspace.QN2 );

}

void nmpc_multGxd( real_t* const dOld, real_t* const Gx1, real_t* const dNew )
{
dNew[0] += + Gx1[0]*dOld[0] + Gx1[1]*dOld[1] + Gx1[2]*dOld[2] + Gx1[3]*dOld[3] + Gx1[4]*dOld[4] + Gx1[5]*dOld[5] + Gx1[6]*dOld[6] + Gx1[7]*dOld[7] + Gx1[8]*dOld[8] + Gx1[9]*dOld[9] + Gx1[10]*dOld[10] + Gx1[11]*dOld[11];
dNew[1] += + Gx1[12]*dOld[0] + Gx1[13]*dOld[1] + Gx1[14]*dOld[2] + Gx1[15]*dOld[3] + Gx1[16]*dOld[4] + Gx1[17]*dOld[5] + Gx1[18]*dOld[6] + Gx1[19]*dOld[7] + Gx1[20]*dOld[8] + Gx1[21]*dOld[9] + Gx1[22]*dOld[10] + Gx1[23]*dOld[11];
dNew[2] += + Gx1[24]*dOld[0] + Gx1[25]*dOld[1] + Gx1[26]*dOld[2] + Gx1[27]*dOld[3] + Gx1[28]*dOld[4] + Gx1[29]*dOld[5] + Gx1[30]*dOld[6] + Gx1[31]*dOld[7] + Gx1[32]*dOld[8] + Gx1[33]*dOld[9] + Gx1[34]*dOld[10] + Gx1[35]*dOld[11];
dNew[3] += + Gx1[36]*dOld[0] + Gx1[37]*dOld[1] + Gx1[38]*dOld[2] + Gx1[39]*dOld[3] + Gx1[40]*dOld[4] + Gx1[41]*dOld[5] + Gx1[42]*dOld[6] + Gx1[43]*dOld[7] + Gx1[44]*dOld[8] + Gx1[45]*dOld[9] + Gx1[46]*dOld[10] + Gx1[47]*dOld[11];
dNew[4] += + Gx1[48]*dOld[0] + Gx1[49]*dOld[1] + Gx1[50]*dOld[2] + Gx1[51]*dOld[3] + Gx1[52]*dOld[4] + Gx1[53]*dOld[5] + Gx1[54]*dOld[6] + Gx1[55]*dOld[7] + Gx1[56]*dOld[8] + Gx1[57]*dOld[9] + Gx1[58]*dOld[10] + Gx1[59]*dOld[11];
dNew[5] += + Gx1[60]*dOld[0] + Gx1[61]*dOld[1] + Gx1[62]*dOld[2] + Gx1[63]*dOld[3] + Gx1[64]*dOld[4] + Gx1[65]*dOld[5] + Gx1[66]*dOld[6] + Gx1[67]*dOld[7] + Gx1[68]*dOld[8] + Gx1[69]*dOld[9] + Gx1[70]*dOld[10] + Gx1[71]*dOld[11];
dNew[6] += + Gx1[72]*dOld[0] + Gx1[73]*dOld[1] + Gx1[74]*dOld[2] + Gx1[75]*dOld[3] + Gx1[76]*dOld[4] + Gx1[77]*dOld[5] + Gx1[78]*dOld[6] + Gx1[79]*dOld[7] + Gx1[80]*dOld[8] + Gx1[81]*dOld[9] + Gx1[82]*dOld[10] + Gx1[83]*dOld[11];
dNew[7] += + Gx1[84]*dOld[0] + Gx1[85]*dOld[1] + Gx1[86]*dOld[2] + Gx1[87]*dOld[3] + Gx1[88]*dOld[4] + Gx1[89]*dOld[5] + Gx1[90]*dOld[6] + Gx1[91]*dOld[7] + Gx1[92]*dOld[8] + Gx1[93]*dOld[9] + Gx1[94]*dOld[10] + Gx1[95]*dOld[11];
dNew[8] += + Gx1[96]*dOld[0] + Gx1[97]*dOld[1] + Gx1[98]*dOld[2] + Gx1[99]*dOld[3] + Gx1[100]*dOld[4] + Gx1[101]*dOld[5] + Gx1[102]*dOld[6] + Gx1[103]*dOld[7] + Gx1[104]*dOld[8] + Gx1[105]*dOld[9] + Gx1[106]*dOld[10] + Gx1[107]*dOld[11];
dNew[9] += + Gx1[108]*dOld[0] + Gx1[109]*dOld[1] + Gx1[110]*dOld[2] + Gx1[111]*dOld[3] + Gx1[112]*dOld[4] + Gx1[113]*dOld[5] + Gx1[114]*dOld[6] + Gx1[115]*dOld[7] + Gx1[116]*dOld[8] + Gx1[117]*dOld[9] + Gx1[118]*dOld[10] + Gx1[119]*dOld[11];
dNew[10] += + Gx1[120]*dOld[0] + Gx1[121]*dOld[1] + Gx1[122]*dOld[2] + Gx1[123]*dOld[3] + Gx1[124]*dOld[4] + Gx1[125]*dOld[5] + Gx1[126]*dOld[6] + Gx1[127]*dOld[7] + Gx1[128]*dOld[8] + Gx1[129]*dOld[9] + Gx1[130]*dOld[10] + Gx1[131]*dOld[11];
dNew[11] += + Gx1[132]*dOld[0] + Gx1[133]*dOld[1] + Gx1[134]*dOld[2] + Gx1[135]*dOld[3] + Gx1[136]*dOld[4] + Gx1[137]*dOld[5] + Gx1[138]*dOld[6] + Gx1[139]*dOld[7] + Gx1[140]*dOld[8] + Gx1[141]*dOld[9] + Gx1[142]*dOld[10] + Gx1[143]*dOld[11];
}

void nmpc_moveGxT( real_t* const Gx1, real_t* const Gx2 )
{
int lRun1;
int lRun2;
for (lRun1 = 0;lRun1 < 12; ++lRun1)
for (lRun2 = 0;lRun2 < 12; ++lRun2)
Gx2[(lRun1 * 12) + (lRun2)] = Gx1[(lRun1 * 12) + (lRun2)];
}

void nmpc_multGxGx( real_t* const Gx1, real_t* const Gx2, real_t* const Gx3 )
{
Gx3[0] = + Gx1[0]*Gx2[0] + Gx1[1]*Gx2[12] + Gx1[2]*Gx2[24] + Gx1[3]*Gx2[36] + Gx1[4]*Gx2[48] + Gx1[5]*Gx2[60] + Gx1[6]*Gx2[72] + Gx1[7]*Gx2[84] + Gx1[8]*Gx2[96] + Gx1[9]*Gx2[108] + Gx1[10]*Gx2[120] + Gx1[11]*Gx2[132];
Gx3[1] = + Gx1[0]*Gx2[1] + Gx1[1]*Gx2[13] + Gx1[2]*Gx2[25] + Gx1[3]*Gx2[37] + Gx1[4]*Gx2[49] + Gx1[5]*Gx2[61] + Gx1[6]*Gx2[73] + Gx1[7]*Gx2[85] + Gx1[8]*Gx2[97] + Gx1[9]*Gx2[109] + Gx1[10]*Gx2[121] + Gx1[11]*Gx2[133];
Gx3[2] = + Gx1[0]*Gx2[2] + Gx1[1]*Gx2[14] + Gx1[2]*Gx2[26] + Gx1[3]*Gx2[38] + Gx1[4]*Gx2[50] + Gx1[5]*Gx2[62] + Gx1[6]*Gx2[74] + Gx1[7]*Gx2[86] + Gx1[8]*Gx2[98] + Gx1[9]*Gx2[110] + Gx1[10]*Gx2[122] + Gx1[11]*Gx2[134];
Gx3[3] = + Gx1[0]*Gx2[3] + Gx1[1]*Gx2[15] + Gx1[2]*Gx2[27] + Gx1[3]*Gx2[39] + Gx1[4]*Gx2[51] + Gx1[5]*Gx2[63] + Gx1[6]*Gx2[75] + Gx1[7]*Gx2[87] + Gx1[8]*Gx2[99] + Gx1[9]*Gx2[111] + Gx1[10]*Gx2[123] + Gx1[11]*Gx2[135];
Gx3[4] = + Gx1[0]*Gx2[4] + Gx1[1]*Gx2[16] + Gx1[2]*Gx2[28] + Gx1[3]*Gx2[40] + Gx1[4]*Gx2[52] + Gx1[5]*Gx2[64] + Gx1[6]*Gx2[76] + Gx1[7]*Gx2[88] + Gx1[8]*Gx2[100] + Gx1[9]*Gx2[112] + Gx1[10]*Gx2[124] + Gx1[11]*Gx2[136];
Gx3[5] = + Gx1[0]*Gx2[5] + Gx1[1]*Gx2[17] + Gx1[2]*Gx2[29] + Gx1[3]*Gx2[41] + Gx1[4]*Gx2[53] + Gx1[5]*Gx2[65] + Gx1[6]*Gx2[77] + Gx1[7]*Gx2[89] + Gx1[8]*Gx2[101] + Gx1[9]*Gx2[113] + Gx1[10]*Gx2[125] + Gx1[11]*Gx2[137];
Gx3[6] = + Gx1[0]*Gx2[6] + Gx1[1]*Gx2[18] + Gx1[2]*Gx2[30] + Gx1[3]*Gx2[42] + Gx1[4]*Gx2[54] + Gx1[5]*Gx2[66] + Gx1[6]*Gx2[78] + Gx1[7]*Gx2[90] + Gx1[8]*Gx2[102] + Gx1[9]*Gx2[114] + Gx1[10]*Gx2[126] + Gx1[11]*Gx2[138];
Gx3[7] = + Gx1[0]*Gx2[7] + Gx1[1]*Gx2[19] + Gx1[2]*Gx2[31] + Gx1[3]*Gx2[43] + Gx1[4]*Gx2[55] + Gx1[5]*Gx2[67] + Gx1[6]*Gx2[79] + Gx1[7]*Gx2[91] + Gx1[8]*Gx2[103] + Gx1[9]*Gx2[115] + Gx1[10]*Gx2[127] + Gx1[11]*Gx2[139];
Gx3[8] = + Gx1[0]*Gx2[8] + Gx1[1]*Gx2[20] + Gx1[2]*Gx2[32] + Gx1[3]*Gx2[44] + Gx1[4]*Gx2[56] + Gx1[5]*Gx2[68] + Gx1[6]*Gx2[80] + Gx1[7]*Gx2[92] + Gx1[8]*Gx2[104] + Gx1[9]*Gx2[116] + Gx1[10]*Gx2[128] + Gx1[11]*Gx2[140];
Gx3[9] = + Gx1[0]*Gx2[9] + Gx1[1]*Gx2[21] + Gx1[2]*Gx2[33] + Gx1[3]*Gx2[45] + Gx1[4]*Gx2[57] + Gx1[5]*Gx2[69] + Gx1[6]*Gx2[81] + Gx1[7]*Gx2[93] + Gx1[8]*Gx2[105] + Gx1[9]*Gx2[117] + Gx1[10]*Gx2[129] + Gx1[11]*Gx2[141];
Gx3[10] = + Gx1[0]*Gx2[10] + Gx1[1]*Gx2[22] + Gx1[2]*Gx2[34] + Gx1[3]*Gx2[46] + Gx1[4]*Gx2[58] + Gx1[5]*Gx2[70] + Gx1[6]*Gx2[82] + Gx1[7]*Gx2[94] + Gx1[8]*Gx2[106] + Gx1[9]*Gx2[118] + Gx1[10]*Gx2[130] + Gx1[11]*Gx2[142];
Gx3[11] = + Gx1[0]*Gx2[11] + Gx1[1]*Gx2[23] + Gx1[2]*Gx2[35] + Gx1[3]*Gx2[47] + Gx1[4]*Gx2[59] + Gx1[5]*Gx2[71] + Gx1[6]*Gx2[83] + Gx1[7]*Gx2[95] + Gx1[8]*Gx2[107] + Gx1[9]*Gx2[119] + Gx1[10]*Gx2[131] + Gx1[11]*Gx2[143];
Gx3[12] = + Gx1[12]*Gx2[0] + Gx1[13]*Gx2[12] + Gx1[14]*Gx2[24] + Gx1[15]*Gx2[36] + Gx1[16]*Gx2[48] + Gx1[17]*Gx2[60] + Gx1[18]*Gx2[72] + Gx1[19]*Gx2[84] + Gx1[20]*Gx2[96] + Gx1[21]*Gx2[108] + Gx1[22]*Gx2[120] + Gx1[23]*Gx2[132];
Gx3[13] = + Gx1[12]*Gx2[1] + Gx1[13]*Gx2[13] + Gx1[14]*Gx2[25] + Gx1[15]*Gx2[37] + Gx1[16]*Gx2[49] + Gx1[17]*Gx2[61] + Gx1[18]*Gx2[73] + Gx1[19]*Gx2[85] + Gx1[20]*Gx2[97] + Gx1[21]*Gx2[109] + Gx1[22]*Gx2[121] + Gx1[23]*Gx2[133];
Gx3[14] = + Gx1[12]*Gx2[2] + Gx1[13]*Gx2[14] + Gx1[14]*Gx2[26] + Gx1[15]*Gx2[38] + Gx1[16]*Gx2[50] + Gx1[17]*Gx2[62] + Gx1[18]*Gx2[74] + Gx1[19]*Gx2[86] + Gx1[20]*Gx2[98] + Gx1[21]*Gx2[110] + Gx1[22]*Gx2[122] + Gx1[23]*Gx2[134];
Gx3[15] = + Gx1[12]*Gx2[3] + Gx1[13]*Gx2[15] + Gx1[14]*Gx2[27] + Gx1[15]*Gx2[39] + Gx1[16]*Gx2[51] + Gx1[17]*Gx2[63] + Gx1[18]*Gx2[75] + Gx1[19]*Gx2[87] + Gx1[20]*Gx2[99] + Gx1[21]*Gx2[111] + Gx1[22]*Gx2[123] + Gx1[23]*Gx2[135];
Gx3[16] = + Gx1[12]*Gx2[4] + Gx1[13]*Gx2[16] + Gx1[14]*Gx2[28] + Gx1[15]*Gx2[40] + Gx1[16]*Gx2[52] + Gx1[17]*Gx2[64] + Gx1[18]*Gx2[76] + Gx1[19]*Gx2[88] + Gx1[20]*Gx2[100] + Gx1[21]*Gx2[112] + Gx1[22]*Gx2[124] + Gx1[23]*Gx2[136];
Gx3[17] = + Gx1[12]*Gx2[5] + Gx1[13]*Gx2[17] + Gx1[14]*Gx2[29] + Gx1[15]*Gx2[41] + Gx1[16]*Gx2[53] + Gx1[17]*Gx2[65] + Gx1[18]*Gx2[77] + Gx1[19]*Gx2[89] + Gx1[20]*Gx2[101] + Gx1[21]*Gx2[113] + Gx1[22]*Gx2[125] + Gx1[23]*Gx2[137];
Gx3[18] = + Gx1[12]*Gx2[6] + Gx1[13]*Gx2[18] + Gx1[14]*Gx2[30] + Gx1[15]*Gx2[42] + Gx1[16]*Gx2[54] + Gx1[17]*Gx2[66] + Gx1[18]*Gx2[78] + Gx1[19]*Gx2[90] + Gx1[20]*Gx2[102] + Gx1[21]*Gx2[114] + Gx1[22]*Gx2[126] + Gx1[23]*Gx2[138];
Gx3[19] = + Gx1[12]*Gx2[7] + Gx1[13]*Gx2[19] + Gx1[14]*Gx2[31] + Gx1[15]*Gx2[43] + Gx1[16]*Gx2[55] + Gx1[17]*Gx2[67] + Gx1[18]*Gx2[79] + Gx1[19]*Gx2[91] + Gx1[20]*Gx2[103] + Gx1[21]*Gx2[115] + Gx1[22]*Gx2[127] + Gx1[23]*Gx2[139];
Gx3[20] = + Gx1[12]*Gx2[8] + Gx1[13]*Gx2[20] + Gx1[14]*Gx2[32] + Gx1[15]*Gx2[44] + Gx1[16]*Gx2[56] + Gx1[17]*Gx2[68] + Gx1[18]*Gx2[80] + Gx1[19]*Gx2[92] + Gx1[20]*Gx2[104] + Gx1[21]*Gx2[116] + Gx1[22]*Gx2[128] + Gx1[23]*Gx2[140];
Gx3[21] = + Gx1[12]*Gx2[9] + Gx1[13]*Gx2[21] + Gx1[14]*Gx2[33] + Gx1[15]*Gx2[45] + Gx1[16]*Gx2[57] + Gx1[17]*Gx2[69] + Gx1[18]*Gx2[81] + Gx1[19]*Gx2[93] + Gx1[20]*Gx2[105] + Gx1[21]*Gx2[117] + Gx1[22]*Gx2[129] + Gx1[23]*Gx2[141];
Gx3[22] = + Gx1[12]*Gx2[10] + Gx1[13]*Gx2[22] + Gx1[14]*Gx2[34] + Gx1[15]*Gx2[46] + Gx1[16]*Gx2[58] + Gx1[17]*Gx2[70] + Gx1[18]*Gx2[82] + Gx1[19]*Gx2[94] + Gx1[20]*Gx2[106] + Gx1[21]*Gx2[118] + Gx1[22]*Gx2[130] + Gx1[23]*Gx2[142];
Gx3[23] = + Gx1[12]*Gx2[11] + Gx1[13]*Gx2[23] + Gx1[14]*Gx2[35] + Gx1[15]*Gx2[47] + Gx1[16]*Gx2[59] + Gx1[17]*Gx2[71] + Gx1[18]*Gx2[83] + Gx1[19]*Gx2[95] + Gx1[20]*Gx2[107] + Gx1[21]*Gx2[119] + Gx1[22]*Gx2[131] + Gx1[23]*Gx2[143];
Gx3[24] = + Gx1[24]*Gx2[0] + Gx1[25]*Gx2[12] + Gx1[26]*Gx2[24] + Gx1[27]*Gx2[36] + Gx1[28]*Gx2[48] + Gx1[29]*Gx2[60] + Gx1[30]*Gx2[72] + Gx1[31]*Gx2[84] + Gx1[32]*Gx2[96] + Gx1[33]*Gx2[108] + Gx1[34]*Gx2[120] + Gx1[35]*Gx2[132];
Gx3[25] = + Gx1[24]*Gx2[1] + Gx1[25]*Gx2[13] + Gx1[26]*Gx2[25] + Gx1[27]*Gx2[37] + Gx1[28]*Gx2[49] + Gx1[29]*Gx2[61] + Gx1[30]*Gx2[73] + Gx1[31]*Gx2[85] + Gx1[32]*Gx2[97] + Gx1[33]*Gx2[109] + Gx1[34]*Gx2[121] + Gx1[35]*Gx2[133];
Gx3[26] = + Gx1[24]*Gx2[2] + Gx1[25]*Gx2[14] + Gx1[26]*Gx2[26] + Gx1[27]*Gx2[38] + Gx1[28]*Gx2[50] + Gx1[29]*Gx2[62] + Gx1[30]*Gx2[74] + Gx1[31]*Gx2[86] + Gx1[32]*Gx2[98] + Gx1[33]*Gx2[110] + Gx1[34]*Gx2[122] + Gx1[35]*Gx2[134];
Gx3[27] = + Gx1[24]*Gx2[3] + Gx1[25]*Gx2[15] + Gx1[26]*Gx2[27] + Gx1[27]*Gx2[39] + Gx1[28]*Gx2[51] + Gx1[29]*Gx2[63] + Gx1[30]*Gx2[75] + Gx1[31]*Gx2[87] + Gx1[32]*Gx2[99] + Gx1[33]*Gx2[111] + Gx1[34]*Gx2[123] + Gx1[35]*Gx2[135];
Gx3[28] = + Gx1[24]*Gx2[4] + Gx1[25]*Gx2[16] + Gx1[26]*Gx2[28] + Gx1[27]*Gx2[40] + Gx1[28]*Gx2[52] + Gx1[29]*Gx2[64] + Gx1[30]*Gx2[76] + Gx1[31]*Gx2[88] + Gx1[32]*Gx2[100] + Gx1[33]*Gx2[112] + Gx1[34]*Gx2[124] + Gx1[35]*Gx2[136];
Gx3[29] = + Gx1[24]*Gx2[5] + Gx1[25]*Gx2[17] + Gx1[26]*Gx2[29] + Gx1[27]*Gx2[41] + Gx1[28]*Gx2[53] + Gx1[29]*Gx2[65] + Gx1[30]*Gx2[77] + Gx1[31]*Gx2[89] + Gx1[32]*Gx2[101] + Gx1[33]*Gx2[113] + Gx1[34]*Gx2[125] + Gx1[35]*Gx2[137];
Gx3[30] = + Gx1[24]*Gx2[6] + Gx1[25]*Gx2[18] + Gx1[26]*Gx2[30] + Gx1[27]*Gx2[42] + Gx1[28]*Gx2[54] + Gx1[29]*Gx2[66] + Gx1[30]*Gx2[78] + Gx1[31]*Gx2[90] + Gx1[32]*Gx2[102] + Gx1[33]*Gx2[114] + Gx1[34]*Gx2[126] + Gx1[35]*Gx2[138];
Gx3[31] = + Gx1[24]*Gx2[7] + Gx1[25]*Gx2[19] + Gx1[26]*Gx2[31] + Gx1[27]*Gx2[43] + Gx1[28]*Gx2[55] + Gx1[29]*Gx2[67] + Gx1[30]*Gx2[79] + Gx1[31]*Gx2[91] + Gx1[32]*Gx2[103] + Gx1[33]*Gx2[115] + Gx1[34]*Gx2[127] + Gx1[35]*Gx2[139];
Gx3[32] = + Gx1[24]*Gx2[8] + Gx1[25]*Gx2[20] + Gx1[26]*Gx2[32] + Gx1[27]*Gx2[44] + Gx1[28]*Gx2[56] + Gx1[29]*Gx2[68] + Gx1[30]*Gx2[80] + Gx1[31]*Gx2[92] + Gx1[32]*Gx2[104] + Gx1[33]*Gx2[116] + Gx1[34]*Gx2[128] + Gx1[35]*Gx2[140];
Gx3[33] = + Gx1[24]*Gx2[9] + Gx1[25]*Gx2[21] + Gx1[26]*Gx2[33] + Gx1[27]*Gx2[45] + Gx1[28]*Gx2[57] + Gx1[29]*Gx2[69] + Gx1[30]*Gx2[81] + Gx1[31]*Gx2[93] + Gx1[32]*Gx2[105] + Gx1[33]*Gx2[117] + Gx1[34]*Gx2[129] + Gx1[35]*Gx2[141];
Gx3[34] = + Gx1[24]*Gx2[10] + Gx1[25]*Gx2[22] + Gx1[26]*Gx2[34] + Gx1[27]*Gx2[46] + Gx1[28]*Gx2[58] + Gx1[29]*Gx2[70] + Gx1[30]*Gx2[82] + Gx1[31]*Gx2[94] + Gx1[32]*Gx2[106] + Gx1[33]*Gx2[118] + Gx1[34]*Gx2[130] + Gx1[35]*Gx2[142];
Gx3[35] = + Gx1[24]*Gx2[11] + Gx1[25]*Gx2[23] + Gx1[26]*Gx2[35] + Gx1[27]*Gx2[47] + Gx1[28]*Gx2[59] + Gx1[29]*Gx2[71] + Gx1[30]*Gx2[83] + Gx1[31]*Gx2[95] + Gx1[32]*Gx2[107] + Gx1[33]*Gx2[119] + Gx1[34]*Gx2[131] + Gx1[35]*Gx2[143];
Gx3[36] = + Gx1[36]*Gx2[0] + Gx1[37]*Gx2[12] + Gx1[38]*Gx2[24] + Gx1[39]*Gx2[36] + Gx1[40]*Gx2[48] + Gx1[41]*Gx2[60] + Gx1[42]*Gx2[72] + Gx1[43]*Gx2[84] + Gx1[44]*Gx2[96] + Gx1[45]*Gx2[108] + Gx1[46]*Gx2[120] + Gx1[47]*Gx2[132];
Gx3[37] = + Gx1[36]*Gx2[1] + Gx1[37]*Gx2[13] + Gx1[38]*Gx2[25] + Gx1[39]*Gx2[37] + Gx1[40]*Gx2[49] + Gx1[41]*Gx2[61] + Gx1[42]*Gx2[73] + Gx1[43]*Gx2[85] + Gx1[44]*Gx2[97] + Gx1[45]*Gx2[109] + Gx1[46]*Gx2[121] + Gx1[47]*Gx2[133];
Gx3[38] = + Gx1[36]*Gx2[2] + Gx1[37]*Gx2[14] + Gx1[38]*Gx2[26] + Gx1[39]*Gx2[38] + Gx1[40]*Gx2[50] + Gx1[41]*Gx2[62] + Gx1[42]*Gx2[74] + Gx1[43]*Gx2[86] + Gx1[44]*Gx2[98] + Gx1[45]*Gx2[110] + Gx1[46]*Gx2[122] + Gx1[47]*Gx2[134];
Gx3[39] = + Gx1[36]*Gx2[3] + Gx1[37]*Gx2[15] + Gx1[38]*Gx2[27] + Gx1[39]*Gx2[39] + Gx1[40]*Gx2[51] + Gx1[41]*Gx2[63] + Gx1[42]*Gx2[75] + Gx1[43]*Gx2[87] + Gx1[44]*Gx2[99] + Gx1[45]*Gx2[111] + Gx1[46]*Gx2[123] + Gx1[47]*Gx2[135];
Gx3[40] = + Gx1[36]*Gx2[4] + Gx1[37]*Gx2[16] + Gx1[38]*Gx2[28] + Gx1[39]*Gx2[40] + Gx1[40]*Gx2[52] + Gx1[41]*Gx2[64] + Gx1[42]*Gx2[76] + Gx1[43]*Gx2[88] + Gx1[44]*Gx2[100] + Gx1[45]*Gx2[112] + Gx1[46]*Gx2[124] + Gx1[47]*Gx2[136];
Gx3[41] = + Gx1[36]*Gx2[5] + Gx1[37]*Gx2[17] + Gx1[38]*Gx2[29] + Gx1[39]*Gx2[41] + Gx1[40]*Gx2[53] + Gx1[41]*Gx2[65] + Gx1[42]*Gx2[77] + Gx1[43]*Gx2[89] + Gx1[44]*Gx2[101] + Gx1[45]*Gx2[113] + Gx1[46]*Gx2[125] + Gx1[47]*Gx2[137];
Gx3[42] = + Gx1[36]*Gx2[6] + Gx1[37]*Gx2[18] + Gx1[38]*Gx2[30] + Gx1[39]*Gx2[42] + Gx1[40]*Gx2[54] + Gx1[41]*Gx2[66] + Gx1[42]*Gx2[78] + Gx1[43]*Gx2[90] + Gx1[44]*Gx2[102] + Gx1[45]*Gx2[114] + Gx1[46]*Gx2[126] + Gx1[47]*Gx2[138];
Gx3[43] = + Gx1[36]*Gx2[7] + Gx1[37]*Gx2[19] + Gx1[38]*Gx2[31] + Gx1[39]*Gx2[43] + Gx1[40]*Gx2[55] + Gx1[41]*Gx2[67] + Gx1[42]*Gx2[79] + Gx1[43]*Gx2[91] + Gx1[44]*Gx2[103] + Gx1[45]*Gx2[115] + Gx1[46]*Gx2[127] + Gx1[47]*Gx2[139];
Gx3[44] = + Gx1[36]*Gx2[8] + Gx1[37]*Gx2[20] + Gx1[38]*Gx2[32] + Gx1[39]*Gx2[44] + Gx1[40]*Gx2[56] + Gx1[41]*Gx2[68] + Gx1[42]*Gx2[80] + Gx1[43]*Gx2[92] + Gx1[44]*Gx2[104] + Gx1[45]*Gx2[116] + Gx1[46]*Gx2[128] + Gx1[47]*Gx2[140];
Gx3[45] = + Gx1[36]*Gx2[9] + Gx1[37]*Gx2[21] + Gx1[38]*Gx2[33] + Gx1[39]*Gx2[45] + Gx1[40]*Gx2[57] + Gx1[41]*Gx2[69] + Gx1[42]*Gx2[81] + Gx1[43]*Gx2[93] + Gx1[44]*Gx2[105] + Gx1[45]*Gx2[117] + Gx1[46]*Gx2[129] + Gx1[47]*Gx2[141];
Gx3[46] = + Gx1[36]*Gx2[10] + Gx1[37]*Gx2[22] + Gx1[38]*Gx2[34] + Gx1[39]*Gx2[46] + Gx1[40]*Gx2[58] + Gx1[41]*Gx2[70] + Gx1[42]*Gx2[82] + Gx1[43]*Gx2[94] + Gx1[44]*Gx2[106] + Gx1[45]*Gx2[118] + Gx1[46]*Gx2[130] + Gx1[47]*Gx2[142];
Gx3[47] = + Gx1[36]*Gx2[11] + Gx1[37]*Gx2[23] + Gx1[38]*Gx2[35] + Gx1[39]*Gx2[47] + Gx1[40]*Gx2[59] + Gx1[41]*Gx2[71] + Gx1[42]*Gx2[83] + Gx1[43]*Gx2[95] + Gx1[44]*Gx2[107] + Gx1[45]*Gx2[119] + Gx1[46]*Gx2[131] + Gx1[47]*Gx2[143];
Gx3[48] = + Gx1[48]*Gx2[0] + Gx1[49]*Gx2[12] + Gx1[50]*Gx2[24] + Gx1[51]*Gx2[36] + Gx1[52]*Gx2[48] + Gx1[53]*Gx2[60] + Gx1[54]*Gx2[72] + Gx1[55]*Gx2[84] + Gx1[56]*Gx2[96] + Gx1[57]*Gx2[108] + Gx1[58]*Gx2[120] + Gx1[59]*Gx2[132];
Gx3[49] = + Gx1[48]*Gx2[1] + Gx1[49]*Gx2[13] + Gx1[50]*Gx2[25] + Gx1[51]*Gx2[37] + Gx1[52]*Gx2[49] + Gx1[53]*Gx2[61] + Gx1[54]*Gx2[73] + Gx1[55]*Gx2[85] + Gx1[56]*Gx2[97] + Gx1[57]*Gx2[109] + Gx1[58]*Gx2[121] + Gx1[59]*Gx2[133];
Gx3[50] = + Gx1[48]*Gx2[2] + Gx1[49]*Gx2[14] + Gx1[50]*Gx2[26] + Gx1[51]*Gx2[38] + Gx1[52]*Gx2[50] + Gx1[53]*Gx2[62] + Gx1[54]*Gx2[74] + Gx1[55]*Gx2[86] + Gx1[56]*Gx2[98] + Gx1[57]*Gx2[110] + Gx1[58]*Gx2[122] + Gx1[59]*Gx2[134];
Gx3[51] = + Gx1[48]*Gx2[3] + Gx1[49]*Gx2[15] + Gx1[50]*Gx2[27] + Gx1[51]*Gx2[39] + Gx1[52]*Gx2[51] + Gx1[53]*Gx2[63] + Gx1[54]*Gx2[75] + Gx1[55]*Gx2[87] + Gx1[56]*Gx2[99] + Gx1[57]*Gx2[111] + Gx1[58]*Gx2[123] + Gx1[59]*Gx2[135];
Gx3[52] = + Gx1[48]*Gx2[4] + Gx1[49]*Gx2[16] + Gx1[50]*Gx2[28] + Gx1[51]*Gx2[40] + Gx1[52]*Gx2[52] + Gx1[53]*Gx2[64] + Gx1[54]*Gx2[76] + Gx1[55]*Gx2[88] + Gx1[56]*Gx2[100] + Gx1[57]*Gx2[112] + Gx1[58]*Gx2[124] + Gx1[59]*Gx2[136];
Gx3[53] = + Gx1[48]*Gx2[5] + Gx1[49]*Gx2[17] + Gx1[50]*Gx2[29] + Gx1[51]*Gx2[41] + Gx1[52]*Gx2[53] + Gx1[53]*Gx2[65] + Gx1[54]*Gx2[77] + Gx1[55]*Gx2[89] + Gx1[56]*Gx2[101] + Gx1[57]*Gx2[113] + Gx1[58]*Gx2[125] + Gx1[59]*Gx2[137];
Gx3[54] = + Gx1[48]*Gx2[6] + Gx1[49]*Gx2[18] + Gx1[50]*Gx2[30] + Gx1[51]*Gx2[42] + Gx1[52]*Gx2[54] + Gx1[53]*Gx2[66] + Gx1[54]*Gx2[78] + Gx1[55]*Gx2[90] + Gx1[56]*Gx2[102] + Gx1[57]*Gx2[114] + Gx1[58]*Gx2[126] + Gx1[59]*Gx2[138];
Gx3[55] = + Gx1[48]*Gx2[7] + Gx1[49]*Gx2[19] + Gx1[50]*Gx2[31] + Gx1[51]*Gx2[43] + Gx1[52]*Gx2[55] + Gx1[53]*Gx2[67] + Gx1[54]*Gx2[79] + Gx1[55]*Gx2[91] + Gx1[56]*Gx2[103] + Gx1[57]*Gx2[115] + Gx1[58]*Gx2[127] + Gx1[59]*Gx2[139];
Gx3[56] = + Gx1[48]*Gx2[8] + Gx1[49]*Gx2[20] + Gx1[50]*Gx2[32] + Gx1[51]*Gx2[44] + Gx1[52]*Gx2[56] + Gx1[53]*Gx2[68] + Gx1[54]*Gx2[80] + Gx1[55]*Gx2[92] + Gx1[56]*Gx2[104] + Gx1[57]*Gx2[116] + Gx1[58]*Gx2[128] + Gx1[59]*Gx2[140];
Gx3[57] = + Gx1[48]*Gx2[9] + Gx1[49]*Gx2[21] + Gx1[50]*Gx2[33] + Gx1[51]*Gx2[45] + Gx1[52]*Gx2[57] + Gx1[53]*Gx2[69] + Gx1[54]*Gx2[81] + Gx1[55]*Gx2[93] + Gx1[56]*Gx2[105] + Gx1[57]*Gx2[117] + Gx1[58]*Gx2[129] + Gx1[59]*Gx2[141];
Gx3[58] = + Gx1[48]*Gx2[10] + Gx1[49]*Gx2[22] + Gx1[50]*Gx2[34] + Gx1[51]*Gx2[46] + Gx1[52]*Gx2[58] + Gx1[53]*Gx2[70] + Gx1[54]*Gx2[82] + Gx1[55]*Gx2[94] + Gx1[56]*Gx2[106] + Gx1[57]*Gx2[118] + Gx1[58]*Gx2[130] + Gx1[59]*Gx2[142];
Gx3[59] = + Gx1[48]*Gx2[11] + Gx1[49]*Gx2[23] + Gx1[50]*Gx2[35] + Gx1[51]*Gx2[47] + Gx1[52]*Gx2[59] + Gx1[53]*Gx2[71] + Gx1[54]*Gx2[83] + Gx1[55]*Gx2[95] + Gx1[56]*Gx2[107] + Gx1[57]*Gx2[119] + Gx1[58]*Gx2[131] + Gx1[59]*Gx2[143];
Gx3[60] = + Gx1[60]*Gx2[0] + Gx1[61]*Gx2[12] + Gx1[62]*Gx2[24] + Gx1[63]*Gx2[36] + Gx1[64]*Gx2[48] + Gx1[65]*Gx2[60] + Gx1[66]*Gx2[72] + Gx1[67]*Gx2[84] + Gx1[68]*Gx2[96] + Gx1[69]*Gx2[108] + Gx1[70]*Gx2[120] + Gx1[71]*Gx2[132];
Gx3[61] = + Gx1[60]*Gx2[1] + Gx1[61]*Gx2[13] + Gx1[62]*Gx2[25] + Gx1[63]*Gx2[37] + Gx1[64]*Gx2[49] + Gx1[65]*Gx2[61] + Gx1[66]*Gx2[73] + Gx1[67]*Gx2[85] + Gx1[68]*Gx2[97] + Gx1[69]*Gx2[109] + Gx1[70]*Gx2[121] + Gx1[71]*Gx2[133];
Gx3[62] = + Gx1[60]*Gx2[2] + Gx1[61]*Gx2[14] + Gx1[62]*Gx2[26] + Gx1[63]*Gx2[38] + Gx1[64]*Gx2[50] + Gx1[65]*Gx2[62] + Gx1[66]*Gx2[74] + Gx1[67]*Gx2[86] + Gx1[68]*Gx2[98] + Gx1[69]*Gx2[110] + Gx1[70]*Gx2[122] + Gx1[71]*Gx2[134];
Gx3[63] = + Gx1[60]*Gx2[3] + Gx1[61]*Gx2[15] + Gx1[62]*Gx2[27] + Gx1[63]*Gx2[39] + Gx1[64]*Gx2[51] + Gx1[65]*Gx2[63] + Gx1[66]*Gx2[75] + Gx1[67]*Gx2[87] + Gx1[68]*Gx2[99] + Gx1[69]*Gx2[111] + Gx1[70]*Gx2[123] + Gx1[71]*Gx2[135];
Gx3[64] = + Gx1[60]*Gx2[4] + Gx1[61]*Gx2[16] + Gx1[62]*Gx2[28] + Gx1[63]*Gx2[40] + Gx1[64]*Gx2[52] + Gx1[65]*Gx2[64] + Gx1[66]*Gx2[76] + Gx1[67]*Gx2[88] + Gx1[68]*Gx2[100] + Gx1[69]*Gx2[112] + Gx1[70]*Gx2[124] + Gx1[71]*Gx2[136];
Gx3[65] = + Gx1[60]*Gx2[5] + Gx1[61]*Gx2[17] + Gx1[62]*Gx2[29] + Gx1[63]*Gx2[41] + Gx1[64]*Gx2[53] + Gx1[65]*Gx2[65] + Gx1[66]*Gx2[77] + Gx1[67]*Gx2[89] + Gx1[68]*Gx2[101] + Gx1[69]*Gx2[113] + Gx1[70]*Gx2[125] + Gx1[71]*Gx2[137];
Gx3[66] = + Gx1[60]*Gx2[6] + Gx1[61]*Gx2[18] + Gx1[62]*Gx2[30] + Gx1[63]*Gx2[42] + Gx1[64]*Gx2[54] + Gx1[65]*Gx2[66] + Gx1[66]*Gx2[78] + Gx1[67]*Gx2[90] + Gx1[68]*Gx2[102] + Gx1[69]*Gx2[114] + Gx1[70]*Gx2[126] + Gx1[71]*Gx2[138];
Gx3[67] = + Gx1[60]*Gx2[7] + Gx1[61]*Gx2[19] + Gx1[62]*Gx2[31] + Gx1[63]*Gx2[43] + Gx1[64]*Gx2[55] + Gx1[65]*Gx2[67] + Gx1[66]*Gx2[79] + Gx1[67]*Gx2[91] + Gx1[68]*Gx2[103] + Gx1[69]*Gx2[115] + Gx1[70]*Gx2[127] + Gx1[71]*Gx2[139];
Gx3[68] = + Gx1[60]*Gx2[8] + Gx1[61]*Gx2[20] + Gx1[62]*Gx2[32] + Gx1[63]*Gx2[44] + Gx1[64]*Gx2[56] + Gx1[65]*Gx2[68] + Gx1[66]*Gx2[80] + Gx1[67]*Gx2[92] + Gx1[68]*Gx2[104] + Gx1[69]*Gx2[116] + Gx1[70]*Gx2[128] + Gx1[71]*Gx2[140];
Gx3[69] = + Gx1[60]*Gx2[9] + Gx1[61]*Gx2[21] + Gx1[62]*Gx2[33] + Gx1[63]*Gx2[45] + Gx1[64]*Gx2[57] + Gx1[65]*Gx2[69] + Gx1[66]*Gx2[81] + Gx1[67]*Gx2[93] + Gx1[68]*Gx2[105] + Gx1[69]*Gx2[117] + Gx1[70]*Gx2[129] + Gx1[71]*Gx2[141];
Gx3[70] = + Gx1[60]*Gx2[10] + Gx1[61]*Gx2[22] + Gx1[62]*Gx2[34] + Gx1[63]*Gx2[46] + Gx1[64]*Gx2[58] + Gx1[65]*Gx2[70] + Gx1[66]*Gx2[82] + Gx1[67]*Gx2[94] + Gx1[68]*Gx2[106] + Gx1[69]*Gx2[118] + Gx1[70]*Gx2[130] + Gx1[71]*Gx2[142];
Gx3[71] = + Gx1[60]*Gx2[11] + Gx1[61]*Gx2[23] + Gx1[62]*Gx2[35] + Gx1[63]*Gx2[47] + Gx1[64]*Gx2[59] + Gx1[65]*Gx2[71] + Gx1[66]*Gx2[83] + Gx1[67]*Gx2[95] + Gx1[68]*Gx2[107] + Gx1[69]*Gx2[119] + Gx1[70]*Gx2[131] + Gx1[71]*Gx2[143];
Gx3[72] = + Gx1[72]*Gx2[0] + Gx1[73]*Gx2[12] + Gx1[74]*Gx2[24] + Gx1[75]*Gx2[36] + Gx1[76]*Gx2[48] + Gx1[77]*Gx2[60] + Gx1[78]*Gx2[72] + Gx1[79]*Gx2[84] + Gx1[80]*Gx2[96] + Gx1[81]*Gx2[108] + Gx1[82]*Gx2[120] + Gx1[83]*Gx2[132];
Gx3[73] = + Gx1[72]*Gx2[1] + Gx1[73]*Gx2[13] + Gx1[74]*Gx2[25] + Gx1[75]*Gx2[37] + Gx1[76]*Gx2[49] + Gx1[77]*Gx2[61] + Gx1[78]*Gx2[73] + Gx1[79]*Gx2[85] + Gx1[80]*Gx2[97] + Gx1[81]*Gx2[109] + Gx1[82]*Gx2[121] + Gx1[83]*Gx2[133];
Gx3[74] = + Gx1[72]*Gx2[2] + Gx1[73]*Gx2[14] + Gx1[74]*Gx2[26] + Gx1[75]*Gx2[38] + Gx1[76]*Gx2[50] + Gx1[77]*Gx2[62] + Gx1[78]*Gx2[74] + Gx1[79]*Gx2[86] + Gx1[80]*Gx2[98] + Gx1[81]*Gx2[110] + Gx1[82]*Gx2[122] + Gx1[83]*Gx2[134];
Gx3[75] = + Gx1[72]*Gx2[3] + Gx1[73]*Gx2[15] + Gx1[74]*Gx2[27] + Gx1[75]*Gx2[39] + Gx1[76]*Gx2[51] + Gx1[77]*Gx2[63] + Gx1[78]*Gx2[75] + Gx1[79]*Gx2[87] + Gx1[80]*Gx2[99] + Gx1[81]*Gx2[111] + Gx1[82]*Gx2[123] + Gx1[83]*Gx2[135];
Gx3[76] = + Gx1[72]*Gx2[4] + Gx1[73]*Gx2[16] + Gx1[74]*Gx2[28] + Gx1[75]*Gx2[40] + Gx1[76]*Gx2[52] + Gx1[77]*Gx2[64] + Gx1[78]*Gx2[76] + Gx1[79]*Gx2[88] + Gx1[80]*Gx2[100] + Gx1[81]*Gx2[112] + Gx1[82]*Gx2[124] + Gx1[83]*Gx2[136];
Gx3[77] = + Gx1[72]*Gx2[5] + Gx1[73]*Gx2[17] + Gx1[74]*Gx2[29] + Gx1[75]*Gx2[41] + Gx1[76]*Gx2[53] + Gx1[77]*Gx2[65] + Gx1[78]*Gx2[77] + Gx1[79]*Gx2[89] + Gx1[80]*Gx2[101] + Gx1[81]*Gx2[113] + Gx1[82]*Gx2[125] + Gx1[83]*Gx2[137];
Gx3[78] = + Gx1[72]*Gx2[6] + Gx1[73]*Gx2[18] + Gx1[74]*Gx2[30] + Gx1[75]*Gx2[42] + Gx1[76]*Gx2[54] + Gx1[77]*Gx2[66] + Gx1[78]*Gx2[78] + Gx1[79]*Gx2[90] + Gx1[80]*Gx2[102] + Gx1[81]*Gx2[114] + Gx1[82]*Gx2[126] + Gx1[83]*Gx2[138];
Gx3[79] = + Gx1[72]*Gx2[7] + Gx1[73]*Gx2[19] + Gx1[74]*Gx2[31] + Gx1[75]*Gx2[43] + Gx1[76]*Gx2[55] + Gx1[77]*Gx2[67] + Gx1[78]*Gx2[79] + Gx1[79]*Gx2[91] + Gx1[80]*Gx2[103] + Gx1[81]*Gx2[115] + Gx1[82]*Gx2[127] + Gx1[83]*Gx2[139];
Gx3[80] = + Gx1[72]*Gx2[8] + Gx1[73]*Gx2[20] + Gx1[74]*Gx2[32] + Gx1[75]*Gx2[44] + Gx1[76]*Gx2[56] + Gx1[77]*Gx2[68] + Gx1[78]*Gx2[80] + Gx1[79]*Gx2[92] + Gx1[80]*Gx2[104] + Gx1[81]*Gx2[116] + Gx1[82]*Gx2[128] + Gx1[83]*Gx2[140];
Gx3[81] = + Gx1[72]*Gx2[9] + Gx1[73]*Gx2[21] + Gx1[74]*Gx2[33] + Gx1[75]*Gx2[45] + Gx1[76]*Gx2[57] + Gx1[77]*Gx2[69] + Gx1[78]*Gx2[81] + Gx1[79]*Gx2[93] + Gx1[80]*Gx2[105] + Gx1[81]*Gx2[117] + Gx1[82]*Gx2[129] + Gx1[83]*Gx2[141];
Gx3[82] = + Gx1[72]*Gx2[10] + Gx1[73]*Gx2[22] + Gx1[74]*Gx2[34] + Gx1[75]*Gx2[46] + Gx1[76]*Gx2[58] + Gx1[77]*Gx2[70] + Gx1[78]*Gx2[82] + Gx1[79]*Gx2[94] + Gx1[80]*Gx2[106] + Gx1[81]*Gx2[118] + Gx1[82]*Gx2[130] + Gx1[83]*Gx2[142];
Gx3[83] = + Gx1[72]*Gx2[11] + Gx1[73]*Gx2[23] + Gx1[74]*Gx2[35] + Gx1[75]*Gx2[47] + Gx1[76]*Gx2[59] + Gx1[77]*Gx2[71] + Gx1[78]*Gx2[83] + Gx1[79]*Gx2[95] + Gx1[80]*Gx2[107] + Gx1[81]*Gx2[119] + Gx1[82]*Gx2[131] + Gx1[83]*Gx2[143];
Gx3[84] = + Gx1[84]*Gx2[0] + Gx1[85]*Gx2[12] + Gx1[86]*Gx2[24] + Gx1[87]*Gx2[36] + Gx1[88]*Gx2[48] + Gx1[89]*Gx2[60] + Gx1[90]*Gx2[72] + Gx1[91]*Gx2[84] + Gx1[92]*Gx2[96] + Gx1[93]*Gx2[108] + Gx1[94]*Gx2[120] + Gx1[95]*Gx2[132];
Gx3[85] = + Gx1[84]*Gx2[1] + Gx1[85]*Gx2[13] + Gx1[86]*Gx2[25] + Gx1[87]*Gx2[37] + Gx1[88]*Gx2[49] + Gx1[89]*Gx2[61] + Gx1[90]*Gx2[73] + Gx1[91]*Gx2[85] + Gx1[92]*Gx2[97] + Gx1[93]*Gx2[109] + Gx1[94]*Gx2[121] + Gx1[95]*Gx2[133];
Gx3[86] = + Gx1[84]*Gx2[2] + Gx1[85]*Gx2[14] + Gx1[86]*Gx2[26] + Gx1[87]*Gx2[38] + Gx1[88]*Gx2[50] + Gx1[89]*Gx2[62] + Gx1[90]*Gx2[74] + Gx1[91]*Gx2[86] + Gx1[92]*Gx2[98] + Gx1[93]*Gx2[110] + Gx1[94]*Gx2[122] + Gx1[95]*Gx2[134];
Gx3[87] = + Gx1[84]*Gx2[3] + Gx1[85]*Gx2[15] + Gx1[86]*Gx2[27] + Gx1[87]*Gx2[39] + Gx1[88]*Gx2[51] + Gx1[89]*Gx2[63] + Gx1[90]*Gx2[75] + Gx1[91]*Gx2[87] + Gx1[92]*Gx2[99] + Gx1[93]*Gx2[111] + Gx1[94]*Gx2[123] + Gx1[95]*Gx2[135];
Gx3[88] = + Gx1[84]*Gx2[4] + Gx1[85]*Gx2[16] + Gx1[86]*Gx2[28] + Gx1[87]*Gx2[40] + Gx1[88]*Gx2[52] + Gx1[89]*Gx2[64] + Gx1[90]*Gx2[76] + Gx1[91]*Gx2[88] + Gx1[92]*Gx2[100] + Gx1[93]*Gx2[112] + Gx1[94]*Gx2[124] + Gx1[95]*Gx2[136];
Gx3[89] = + Gx1[84]*Gx2[5] + Gx1[85]*Gx2[17] + Gx1[86]*Gx2[29] + Gx1[87]*Gx2[41] + Gx1[88]*Gx2[53] + Gx1[89]*Gx2[65] + Gx1[90]*Gx2[77] + Gx1[91]*Gx2[89] + Gx1[92]*Gx2[101] + Gx1[93]*Gx2[113] + Gx1[94]*Gx2[125] + Gx1[95]*Gx2[137];
Gx3[90] = + Gx1[84]*Gx2[6] + Gx1[85]*Gx2[18] + Gx1[86]*Gx2[30] + Gx1[87]*Gx2[42] + Gx1[88]*Gx2[54] + Gx1[89]*Gx2[66] + Gx1[90]*Gx2[78] + Gx1[91]*Gx2[90] + Gx1[92]*Gx2[102] + Gx1[93]*Gx2[114] + Gx1[94]*Gx2[126] + Gx1[95]*Gx2[138];
Gx3[91] = + Gx1[84]*Gx2[7] + Gx1[85]*Gx2[19] + Gx1[86]*Gx2[31] + Gx1[87]*Gx2[43] + Gx1[88]*Gx2[55] + Gx1[89]*Gx2[67] + Gx1[90]*Gx2[79] + Gx1[91]*Gx2[91] + Gx1[92]*Gx2[103] + Gx1[93]*Gx2[115] + Gx1[94]*Gx2[127] + Gx1[95]*Gx2[139];
Gx3[92] = + Gx1[84]*Gx2[8] + Gx1[85]*Gx2[20] + Gx1[86]*Gx2[32] + Gx1[87]*Gx2[44] + Gx1[88]*Gx2[56] + Gx1[89]*Gx2[68] + Gx1[90]*Gx2[80] + Gx1[91]*Gx2[92] + Gx1[92]*Gx2[104] + Gx1[93]*Gx2[116] + Gx1[94]*Gx2[128] + Gx1[95]*Gx2[140];
Gx3[93] = + Gx1[84]*Gx2[9] + Gx1[85]*Gx2[21] + Gx1[86]*Gx2[33] + Gx1[87]*Gx2[45] + Gx1[88]*Gx2[57] + Gx1[89]*Gx2[69] + Gx1[90]*Gx2[81] + Gx1[91]*Gx2[93] + Gx1[92]*Gx2[105] + Gx1[93]*Gx2[117] + Gx1[94]*Gx2[129] + Gx1[95]*Gx2[141];
Gx3[94] = + Gx1[84]*Gx2[10] + Gx1[85]*Gx2[22] + Gx1[86]*Gx2[34] + Gx1[87]*Gx2[46] + Gx1[88]*Gx2[58] + Gx1[89]*Gx2[70] + Gx1[90]*Gx2[82] + Gx1[91]*Gx2[94] + Gx1[92]*Gx2[106] + Gx1[93]*Gx2[118] + Gx1[94]*Gx2[130] + Gx1[95]*Gx2[142];
Gx3[95] = + Gx1[84]*Gx2[11] + Gx1[85]*Gx2[23] + Gx1[86]*Gx2[35] + Gx1[87]*Gx2[47] + Gx1[88]*Gx2[59] + Gx1[89]*Gx2[71] + Gx1[90]*Gx2[83] + Gx1[91]*Gx2[95] + Gx1[92]*Gx2[107] + Gx1[93]*Gx2[119] + Gx1[94]*Gx2[131] + Gx1[95]*Gx2[143];
Gx3[96] = + Gx1[96]*Gx2[0] + Gx1[97]*Gx2[12] + Gx1[98]*Gx2[24] + Gx1[99]*Gx2[36] + Gx1[100]*Gx2[48] + Gx1[101]*Gx2[60] + Gx1[102]*Gx2[72] + Gx1[103]*Gx2[84] + Gx1[104]*Gx2[96] + Gx1[105]*Gx2[108] + Gx1[106]*Gx2[120] + Gx1[107]*Gx2[132];
Gx3[97] = + Gx1[96]*Gx2[1] + Gx1[97]*Gx2[13] + Gx1[98]*Gx2[25] + Gx1[99]*Gx2[37] + Gx1[100]*Gx2[49] + Gx1[101]*Gx2[61] + Gx1[102]*Gx2[73] + Gx1[103]*Gx2[85] + Gx1[104]*Gx2[97] + Gx1[105]*Gx2[109] + Gx1[106]*Gx2[121] + Gx1[107]*Gx2[133];
Gx3[98] = + Gx1[96]*Gx2[2] + Gx1[97]*Gx2[14] + Gx1[98]*Gx2[26] + Gx1[99]*Gx2[38] + Gx1[100]*Gx2[50] + Gx1[101]*Gx2[62] + Gx1[102]*Gx2[74] + Gx1[103]*Gx2[86] + Gx1[104]*Gx2[98] + Gx1[105]*Gx2[110] + Gx1[106]*Gx2[122] + Gx1[107]*Gx2[134];
Gx3[99] = + Gx1[96]*Gx2[3] + Gx1[97]*Gx2[15] + Gx1[98]*Gx2[27] + Gx1[99]*Gx2[39] + Gx1[100]*Gx2[51] + Gx1[101]*Gx2[63] + Gx1[102]*Gx2[75] + Gx1[103]*Gx2[87] + Gx1[104]*Gx2[99] + Gx1[105]*Gx2[111] + Gx1[106]*Gx2[123] + Gx1[107]*Gx2[135];
Gx3[100] = + Gx1[96]*Gx2[4] + Gx1[97]*Gx2[16] + Gx1[98]*Gx2[28] + Gx1[99]*Gx2[40] + Gx1[100]*Gx2[52] + Gx1[101]*Gx2[64] + Gx1[102]*Gx2[76] + Gx1[103]*Gx2[88] + Gx1[104]*Gx2[100] + Gx1[105]*Gx2[112] + Gx1[106]*Gx2[124] + Gx1[107]*Gx2[136];
Gx3[101] = + Gx1[96]*Gx2[5] + Gx1[97]*Gx2[17] + Gx1[98]*Gx2[29] + Gx1[99]*Gx2[41] + Gx1[100]*Gx2[53] + Gx1[101]*Gx2[65] + Gx1[102]*Gx2[77] + Gx1[103]*Gx2[89] + Gx1[104]*Gx2[101] + Gx1[105]*Gx2[113] + Gx1[106]*Gx2[125] + Gx1[107]*Gx2[137];
Gx3[102] = + Gx1[96]*Gx2[6] + Gx1[97]*Gx2[18] + Gx1[98]*Gx2[30] + Gx1[99]*Gx2[42] + Gx1[100]*Gx2[54] + Gx1[101]*Gx2[66] + Gx1[102]*Gx2[78] + Gx1[103]*Gx2[90] + Gx1[104]*Gx2[102] + Gx1[105]*Gx2[114] + Gx1[106]*Gx2[126] + Gx1[107]*Gx2[138];
Gx3[103] = + Gx1[96]*Gx2[7] + Gx1[97]*Gx2[19] + Gx1[98]*Gx2[31] + Gx1[99]*Gx2[43] + Gx1[100]*Gx2[55] + Gx1[101]*Gx2[67] + Gx1[102]*Gx2[79] + Gx1[103]*Gx2[91] + Gx1[104]*Gx2[103] + Gx1[105]*Gx2[115] + Gx1[106]*Gx2[127] + Gx1[107]*Gx2[139];
Gx3[104] = + Gx1[96]*Gx2[8] + Gx1[97]*Gx2[20] + Gx1[98]*Gx2[32] + Gx1[99]*Gx2[44] + Gx1[100]*Gx2[56] + Gx1[101]*Gx2[68] + Gx1[102]*Gx2[80] + Gx1[103]*Gx2[92] + Gx1[104]*Gx2[104] + Gx1[105]*Gx2[116] + Gx1[106]*Gx2[128] + Gx1[107]*Gx2[140];
Gx3[105] = + Gx1[96]*Gx2[9] + Gx1[97]*Gx2[21] + Gx1[98]*Gx2[33] + Gx1[99]*Gx2[45] + Gx1[100]*Gx2[57] + Gx1[101]*Gx2[69] + Gx1[102]*Gx2[81] + Gx1[103]*Gx2[93] + Gx1[104]*Gx2[105] + Gx1[105]*Gx2[117] + Gx1[106]*Gx2[129] + Gx1[107]*Gx2[141];
Gx3[106] = + Gx1[96]*Gx2[10] + Gx1[97]*Gx2[22] + Gx1[98]*Gx2[34] + Gx1[99]*Gx2[46] + Gx1[100]*Gx2[58] + Gx1[101]*Gx2[70] + Gx1[102]*Gx2[82] + Gx1[103]*Gx2[94] + Gx1[104]*Gx2[106] + Gx1[105]*Gx2[118] + Gx1[106]*Gx2[130] + Gx1[107]*Gx2[142];
Gx3[107] = + Gx1[96]*Gx2[11] + Gx1[97]*Gx2[23] + Gx1[98]*Gx2[35] + Gx1[99]*Gx2[47] + Gx1[100]*Gx2[59] + Gx1[101]*Gx2[71] + Gx1[102]*Gx2[83] + Gx1[103]*Gx2[95] + Gx1[104]*Gx2[107] + Gx1[105]*Gx2[119] + Gx1[106]*Gx2[131] + Gx1[107]*Gx2[143];
Gx3[108] = + Gx1[108]*Gx2[0] + Gx1[109]*Gx2[12] + Gx1[110]*Gx2[24] + Gx1[111]*Gx2[36] + Gx1[112]*Gx2[48] + Gx1[113]*Gx2[60] + Gx1[114]*Gx2[72] + Gx1[115]*Gx2[84] + Gx1[116]*Gx2[96] + Gx1[117]*Gx2[108] + Gx1[118]*Gx2[120] + Gx1[119]*Gx2[132];
Gx3[109] = + Gx1[108]*Gx2[1] + Gx1[109]*Gx2[13] + Gx1[110]*Gx2[25] + Gx1[111]*Gx2[37] + Gx1[112]*Gx2[49] + Gx1[113]*Gx2[61] + Gx1[114]*Gx2[73] + Gx1[115]*Gx2[85] + Gx1[116]*Gx2[97] + Gx1[117]*Gx2[109] + Gx1[118]*Gx2[121] + Gx1[119]*Gx2[133];
Gx3[110] = + Gx1[108]*Gx2[2] + Gx1[109]*Gx2[14] + Gx1[110]*Gx2[26] + Gx1[111]*Gx2[38] + Gx1[112]*Gx2[50] + Gx1[113]*Gx2[62] + Gx1[114]*Gx2[74] + Gx1[115]*Gx2[86] + Gx1[116]*Gx2[98] + Gx1[117]*Gx2[110] + Gx1[118]*Gx2[122] + Gx1[119]*Gx2[134];
Gx3[111] = + Gx1[108]*Gx2[3] + Gx1[109]*Gx2[15] + Gx1[110]*Gx2[27] + Gx1[111]*Gx2[39] + Gx1[112]*Gx2[51] + Gx1[113]*Gx2[63] + Gx1[114]*Gx2[75] + Gx1[115]*Gx2[87] + Gx1[116]*Gx2[99] + Gx1[117]*Gx2[111] + Gx1[118]*Gx2[123] + Gx1[119]*Gx2[135];
Gx3[112] = + Gx1[108]*Gx2[4] + Gx1[109]*Gx2[16] + Gx1[110]*Gx2[28] + Gx1[111]*Gx2[40] + Gx1[112]*Gx2[52] + Gx1[113]*Gx2[64] + Gx1[114]*Gx2[76] + Gx1[115]*Gx2[88] + Gx1[116]*Gx2[100] + Gx1[117]*Gx2[112] + Gx1[118]*Gx2[124] + Gx1[119]*Gx2[136];
Gx3[113] = + Gx1[108]*Gx2[5] + Gx1[109]*Gx2[17] + Gx1[110]*Gx2[29] + Gx1[111]*Gx2[41] + Gx1[112]*Gx2[53] + Gx1[113]*Gx2[65] + Gx1[114]*Gx2[77] + Gx1[115]*Gx2[89] + Gx1[116]*Gx2[101] + Gx1[117]*Gx2[113] + Gx1[118]*Gx2[125] + Gx1[119]*Gx2[137];
Gx3[114] = + Gx1[108]*Gx2[6] + Gx1[109]*Gx2[18] + Gx1[110]*Gx2[30] + Gx1[111]*Gx2[42] + Gx1[112]*Gx2[54] + Gx1[113]*Gx2[66] + Gx1[114]*Gx2[78] + Gx1[115]*Gx2[90] + Gx1[116]*Gx2[102] + Gx1[117]*Gx2[114] + Gx1[118]*Gx2[126] + Gx1[119]*Gx2[138];
Gx3[115] = + Gx1[108]*Gx2[7] + Gx1[109]*Gx2[19] + Gx1[110]*Gx2[31] + Gx1[111]*Gx2[43] + Gx1[112]*Gx2[55] + Gx1[113]*Gx2[67] + Gx1[114]*Gx2[79] + Gx1[115]*Gx2[91] + Gx1[116]*Gx2[103] + Gx1[117]*Gx2[115] + Gx1[118]*Gx2[127] + Gx1[119]*Gx2[139];
Gx3[116] = + Gx1[108]*Gx2[8] + Gx1[109]*Gx2[20] + Gx1[110]*Gx2[32] + Gx1[111]*Gx2[44] + Gx1[112]*Gx2[56] + Gx1[113]*Gx2[68] + Gx1[114]*Gx2[80] + Gx1[115]*Gx2[92] + Gx1[116]*Gx2[104] + Gx1[117]*Gx2[116] + Gx1[118]*Gx2[128] + Gx1[119]*Gx2[140];
Gx3[117] = + Gx1[108]*Gx2[9] + Gx1[109]*Gx2[21] + Gx1[110]*Gx2[33] + Gx1[111]*Gx2[45] + Gx1[112]*Gx2[57] + Gx1[113]*Gx2[69] + Gx1[114]*Gx2[81] + Gx1[115]*Gx2[93] + Gx1[116]*Gx2[105] + Gx1[117]*Gx2[117] + Gx1[118]*Gx2[129] + Gx1[119]*Gx2[141];
Gx3[118] = + Gx1[108]*Gx2[10] + Gx1[109]*Gx2[22] + Gx1[110]*Gx2[34] + Gx1[111]*Gx2[46] + Gx1[112]*Gx2[58] + Gx1[113]*Gx2[70] + Gx1[114]*Gx2[82] + Gx1[115]*Gx2[94] + Gx1[116]*Gx2[106] + Gx1[117]*Gx2[118] + Gx1[118]*Gx2[130] + Gx1[119]*Gx2[142];
Gx3[119] = + Gx1[108]*Gx2[11] + Gx1[109]*Gx2[23] + Gx1[110]*Gx2[35] + Gx1[111]*Gx2[47] + Gx1[112]*Gx2[59] + Gx1[113]*Gx2[71] + Gx1[114]*Gx2[83] + Gx1[115]*Gx2[95] + Gx1[116]*Gx2[107] + Gx1[117]*Gx2[119] + Gx1[118]*Gx2[131] + Gx1[119]*Gx2[143];
Gx3[120] = + Gx1[120]*Gx2[0] + Gx1[121]*Gx2[12] + Gx1[122]*Gx2[24] + Gx1[123]*Gx2[36] + Gx1[124]*Gx2[48] + Gx1[125]*Gx2[60] + Gx1[126]*Gx2[72] + Gx1[127]*Gx2[84] + Gx1[128]*Gx2[96] + Gx1[129]*Gx2[108] + Gx1[130]*Gx2[120] + Gx1[131]*Gx2[132];
Gx3[121] = + Gx1[120]*Gx2[1] + Gx1[121]*Gx2[13] + Gx1[122]*Gx2[25] + Gx1[123]*Gx2[37] + Gx1[124]*Gx2[49] + Gx1[125]*Gx2[61] + Gx1[126]*Gx2[73] + Gx1[127]*Gx2[85] + Gx1[128]*Gx2[97] + Gx1[129]*Gx2[109] + Gx1[130]*Gx2[121] + Gx1[131]*Gx2[133];
Gx3[122] = + Gx1[120]*Gx2[2] + Gx1[121]*Gx2[14] + Gx1[122]*Gx2[26] + Gx1[123]*Gx2[38] + Gx1[124]*Gx2[50] + Gx1[125]*Gx2[62] + Gx1[126]*Gx2[74] + Gx1[127]*Gx2[86] + Gx1[128]*Gx2[98] + Gx1[129]*Gx2[110] + Gx1[130]*Gx2[122] + Gx1[131]*Gx2[134];
Gx3[123] = + Gx1[120]*Gx2[3] + Gx1[121]*Gx2[15] + Gx1[122]*Gx2[27] + Gx1[123]*Gx2[39] + Gx1[124]*Gx2[51] + Gx1[125]*Gx2[63] + Gx1[126]*Gx2[75] + Gx1[127]*Gx2[87] + Gx1[128]*Gx2[99] + Gx1[129]*Gx2[111] + Gx1[130]*Gx2[123] + Gx1[131]*Gx2[135];
Gx3[124] = + Gx1[120]*Gx2[4] + Gx1[121]*Gx2[16] + Gx1[122]*Gx2[28] + Gx1[123]*Gx2[40] + Gx1[124]*Gx2[52] + Gx1[125]*Gx2[64] + Gx1[126]*Gx2[76] + Gx1[127]*Gx2[88] + Gx1[128]*Gx2[100] + Gx1[129]*Gx2[112] + Gx1[130]*Gx2[124] + Gx1[131]*Gx2[136];
Gx3[125] = + Gx1[120]*Gx2[5] + Gx1[121]*Gx2[17] + Gx1[122]*Gx2[29] + Gx1[123]*Gx2[41] + Gx1[124]*Gx2[53] + Gx1[125]*Gx2[65] + Gx1[126]*Gx2[77] + Gx1[127]*Gx2[89] + Gx1[128]*Gx2[101] + Gx1[129]*Gx2[113] + Gx1[130]*Gx2[125] + Gx1[131]*Gx2[137];
Gx3[126] = + Gx1[120]*Gx2[6] + Gx1[121]*Gx2[18] + Gx1[122]*Gx2[30] + Gx1[123]*Gx2[42] + Gx1[124]*Gx2[54] + Gx1[125]*Gx2[66] + Gx1[126]*Gx2[78] + Gx1[127]*Gx2[90] + Gx1[128]*Gx2[102] + Gx1[129]*Gx2[114] + Gx1[130]*Gx2[126] + Gx1[131]*Gx2[138];
Gx3[127] = + Gx1[120]*Gx2[7] + Gx1[121]*Gx2[19] + Gx1[122]*Gx2[31] + Gx1[123]*Gx2[43] + Gx1[124]*Gx2[55] + Gx1[125]*Gx2[67] + Gx1[126]*Gx2[79] + Gx1[127]*Gx2[91] + Gx1[128]*Gx2[103] + Gx1[129]*Gx2[115] + Gx1[130]*Gx2[127] + Gx1[131]*Gx2[139];
Gx3[128] = + Gx1[120]*Gx2[8] + Gx1[121]*Gx2[20] + Gx1[122]*Gx2[32] + Gx1[123]*Gx2[44] + Gx1[124]*Gx2[56] + Gx1[125]*Gx2[68] + Gx1[126]*Gx2[80] + Gx1[127]*Gx2[92] + Gx1[128]*Gx2[104] + Gx1[129]*Gx2[116] + Gx1[130]*Gx2[128] + Gx1[131]*Gx2[140];
Gx3[129] = + Gx1[120]*Gx2[9] + Gx1[121]*Gx2[21] + Gx1[122]*Gx2[33] + Gx1[123]*Gx2[45] + Gx1[124]*Gx2[57] + Gx1[125]*Gx2[69] + Gx1[126]*Gx2[81] + Gx1[127]*Gx2[93] + Gx1[128]*Gx2[105] + Gx1[129]*Gx2[117] + Gx1[130]*Gx2[129] + Gx1[131]*Gx2[141];
Gx3[130] = + Gx1[120]*Gx2[10] + Gx1[121]*Gx2[22] + Gx1[122]*Gx2[34] + Gx1[123]*Gx2[46] + Gx1[124]*Gx2[58] + Gx1[125]*Gx2[70] + Gx1[126]*Gx2[82] + Gx1[127]*Gx2[94] + Gx1[128]*Gx2[106] + Gx1[129]*Gx2[118] + Gx1[130]*Gx2[130] + Gx1[131]*Gx2[142];
Gx3[131] = + Gx1[120]*Gx2[11] + Gx1[121]*Gx2[23] + Gx1[122]*Gx2[35] + Gx1[123]*Gx2[47] + Gx1[124]*Gx2[59] + Gx1[125]*Gx2[71] + Gx1[126]*Gx2[83] + Gx1[127]*Gx2[95] + Gx1[128]*Gx2[107] + Gx1[129]*Gx2[119] + Gx1[130]*Gx2[131] + Gx1[131]*Gx2[143];
Gx3[132] = + Gx1[132]*Gx2[0] + Gx1[133]*Gx2[12] + Gx1[134]*Gx2[24] + Gx1[135]*Gx2[36] + Gx1[136]*Gx2[48] + Gx1[137]*Gx2[60] + Gx1[138]*Gx2[72] + Gx1[139]*Gx2[84] + Gx1[140]*Gx2[96] + Gx1[141]*Gx2[108] + Gx1[142]*Gx2[120] + Gx1[143]*Gx2[132];
Gx3[133] = + Gx1[132]*Gx2[1] + Gx1[133]*Gx2[13] + Gx1[134]*Gx2[25] + Gx1[135]*Gx2[37] + Gx1[136]*Gx2[49] + Gx1[137]*Gx2[61] + Gx1[138]*Gx2[73] + Gx1[139]*Gx2[85] + Gx1[140]*Gx2[97] + Gx1[141]*Gx2[109] + Gx1[142]*Gx2[121] + Gx1[143]*Gx2[133];
Gx3[134] = + Gx1[132]*Gx2[2] + Gx1[133]*Gx2[14] + Gx1[134]*Gx2[26] + Gx1[135]*Gx2[38] + Gx1[136]*Gx2[50] + Gx1[137]*Gx2[62] + Gx1[138]*Gx2[74] + Gx1[139]*Gx2[86] + Gx1[140]*Gx2[98] + Gx1[141]*Gx2[110] + Gx1[142]*Gx2[122] + Gx1[143]*Gx2[134];
Gx3[135] = + Gx1[132]*Gx2[3] + Gx1[133]*Gx2[15] + Gx1[134]*Gx2[27] + Gx1[135]*Gx2[39] + Gx1[136]*Gx2[51] + Gx1[137]*Gx2[63] + Gx1[138]*Gx2[75] + Gx1[139]*Gx2[87] + Gx1[140]*Gx2[99] + Gx1[141]*Gx2[111] + Gx1[142]*Gx2[123] + Gx1[143]*Gx2[135];
Gx3[136] = + Gx1[132]*Gx2[4] + Gx1[133]*Gx2[16] + Gx1[134]*Gx2[28] + Gx1[135]*Gx2[40] + Gx1[136]*Gx2[52] + Gx1[137]*Gx2[64] + Gx1[138]*Gx2[76] + Gx1[139]*Gx2[88] + Gx1[140]*Gx2[100] + Gx1[141]*Gx2[112] + Gx1[142]*Gx2[124] + Gx1[143]*Gx2[136];
Gx3[137] = + Gx1[132]*Gx2[5] + Gx1[133]*Gx2[17] + Gx1[134]*Gx2[29] + Gx1[135]*Gx2[41] + Gx1[136]*Gx2[53] + Gx1[137]*Gx2[65] + Gx1[138]*Gx2[77] + Gx1[139]*Gx2[89] + Gx1[140]*Gx2[101] + Gx1[141]*Gx2[113] + Gx1[142]*Gx2[125] + Gx1[143]*Gx2[137];
Gx3[138] = + Gx1[132]*Gx2[6] + Gx1[133]*Gx2[18] + Gx1[134]*Gx2[30] + Gx1[135]*Gx2[42] + Gx1[136]*Gx2[54] + Gx1[137]*Gx2[66] + Gx1[138]*Gx2[78] + Gx1[139]*Gx2[90] + Gx1[140]*Gx2[102] + Gx1[141]*Gx2[114] + Gx1[142]*Gx2[126] + Gx1[143]*Gx2[138];
Gx3[139] = + Gx1[132]*Gx2[7] + Gx1[133]*Gx2[19] + Gx1[134]*Gx2[31] + Gx1[135]*Gx2[43] + Gx1[136]*Gx2[55] + Gx1[137]*Gx2[67] + Gx1[138]*Gx2[79] + Gx1[139]*Gx2[91] + Gx1[140]*Gx2[103] + Gx1[141]*Gx2[115] + Gx1[142]*Gx2[127] + Gx1[143]*Gx2[139];
Gx3[140] = + Gx1[132]*Gx2[8] + Gx1[133]*Gx2[20] + Gx1[134]*Gx2[32] + Gx1[135]*Gx2[44] + Gx1[136]*Gx2[56] + Gx1[137]*Gx2[68] + Gx1[138]*Gx2[80] + Gx1[139]*Gx2[92] + Gx1[140]*Gx2[104] + Gx1[141]*Gx2[116] + Gx1[142]*Gx2[128] + Gx1[143]*Gx2[140];
Gx3[141] = + Gx1[132]*Gx2[9] + Gx1[133]*Gx2[21] + Gx1[134]*Gx2[33] + Gx1[135]*Gx2[45] + Gx1[136]*Gx2[57] + Gx1[137]*Gx2[69] + Gx1[138]*Gx2[81] + Gx1[139]*Gx2[93] + Gx1[140]*Gx2[105] + Gx1[141]*Gx2[117] + Gx1[142]*Gx2[129] + Gx1[143]*Gx2[141];
Gx3[142] = + Gx1[132]*Gx2[10] + Gx1[133]*Gx2[22] + Gx1[134]*Gx2[34] + Gx1[135]*Gx2[46] + Gx1[136]*Gx2[58] + Gx1[137]*Gx2[70] + Gx1[138]*Gx2[82] + Gx1[139]*Gx2[94] + Gx1[140]*Gx2[106] + Gx1[141]*Gx2[118] + Gx1[142]*Gx2[130] + Gx1[143]*Gx2[142];
Gx3[143] = + Gx1[132]*Gx2[11] + Gx1[133]*Gx2[23] + Gx1[134]*Gx2[35] + Gx1[135]*Gx2[47] + Gx1[136]*Gx2[59] + Gx1[137]*Gx2[71] + Gx1[138]*Gx2[83] + Gx1[139]*Gx2[95] + Gx1[140]*Gx2[107] + Gx1[141]*Gx2[119] + Gx1[142]*Gx2[131] + Gx1[143]*Gx2[143];
}

void nmpc_multGxGu( real_t* const Gx1, real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = + Gx1[0]*Gu1[0] + Gx1[1]*Gu1[4] + Gx1[2]*Gu1[8] + Gx1[3]*Gu1[12] + Gx1[4]*Gu1[16] + Gx1[5]*Gu1[20] + Gx1[6]*Gu1[24] + Gx1[7]*Gu1[28] + Gx1[8]*Gu1[32] + Gx1[9]*Gu1[36] + Gx1[10]*Gu1[40] + Gx1[11]*Gu1[44];
Gu2[1] = + Gx1[0]*Gu1[1] + Gx1[1]*Gu1[5] + Gx1[2]*Gu1[9] + Gx1[3]*Gu1[13] + Gx1[4]*Gu1[17] + Gx1[5]*Gu1[21] + Gx1[6]*Gu1[25] + Gx1[7]*Gu1[29] + Gx1[8]*Gu1[33] + Gx1[9]*Gu1[37] + Gx1[10]*Gu1[41] + Gx1[11]*Gu1[45];
Gu2[2] = + Gx1[0]*Gu1[2] + Gx1[1]*Gu1[6] + Gx1[2]*Gu1[10] + Gx1[3]*Gu1[14] + Gx1[4]*Gu1[18] + Gx1[5]*Gu1[22] + Gx1[6]*Gu1[26] + Gx1[7]*Gu1[30] + Gx1[8]*Gu1[34] + Gx1[9]*Gu1[38] + Gx1[10]*Gu1[42] + Gx1[11]*Gu1[46];
Gu2[3] = + Gx1[0]*Gu1[3] + Gx1[1]*Gu1[7] + Gx1[2]*Gu1[11] + Gx1[3]*Gu1[15] + Gx1[4]*Gu1[19] + Gx1[5]*Gu1[23] + Gx1[6]*Gu1[27] + Gx1[7]*Gu1[31] + Gx1[8]*Gu1[35] + Gx1[9]*Gu1[39] + Gx1[10]*Gu1[43] + Gx1[11]*Gu1[47];
Gu2[4] = + Gx1[12]*Gu1[0] + Gx1[13]*Gu1[4] + Gx1[14]*Gu1[8] + Gx1[15]*Gu1[12] + Gx1[16]*Gu1[16] + Gx1[17]*Gu1[20] + Gx1[18]*Gu1[24] + Gx1[19]*Gu1[28] + Gx1[20]*Gu1[32] + Gx1[21]*Gu1[36] + Gx1[22]*Gu1[40] + Gx1[23]*Gu1[44];
Gu2[5] = + Gx1[12]*Gu1[1] + Gx1[13]*Gu1[5] + Gx1[14]*Gu1[9] + Gx1[15]*Gu1[13] + Gx1[16]*Gu1[17] + Gx1[17]*Gu1[21] + Gx1[18]*Gu1[25] + Gx1[19]*Gu1[29] + Gx1[20]*Gu1[33] + Gx1[21]*Gu1[37] + Gx1[22]*Gu1[41] + Gx1[23]*Gu1[45];
Gu2[6] = + Gx1[12]*Gu1[2] + Gx1[13]*Gu1[6] + Gx1[14]*Gu1[10] + Gx1[15]*Gu1[14] + Gx1[16]*Gu1[18] + Gx1[17]*Gu1[22] + Gx1[18]*Gu1[26] + Gx1[19]*Gu1[30] + Gx1[20]*Gu1[34] + Gx1[21]*Gu1[38] + Gx1[22]*Gu1[42] + Gx1[23]*Gu1[46];
Gu2[7] = + Gx1[12]*Gu1[3] + Gx1[13]*Gu1[7] + Gx1[14]*Gu1[11] + Gx1[15]*Gu1[15] + Gx1[16]*Gu1[19] + Gx1[17]*Gu1[23] + Gx1[18]*Gu1[27] + Gx1[19]*Gu1[31] + Gx1[20]*Gu1[35] + Gx1[21]*Gu1[39] + Gx1[22]*Gu1[43] + Gx1[23]*Gu1[47];
Gu2[8] = + Gx1[24]*Gu1[0] + Gx1[25]*Gu1[4] + Gx1[26]*Gu1[8] + Gx1[27]*Gu1[12] + Gx1[28]*Gu1[16] + Gx1[29]*Gu1[20] + Gx1[30]*Gu1[24] + Gx1[31]*Gu1[28] + Gx1[32]*Gu1[32] + Gx1[33]*Gu1[36] + Gx1[34]*Gu1[40] + Gx1[35]*Gu1[44];
Gu2[9] = + Gx1[24]*Gu1[1] + Gx1[25]*Gu1[5] + Gx1[26]*Gu1[9] + Gx1[27]*Gu1[13] + Gx1[28]*Gu1[17] + Gx1[29]*Gu1[21] + Gx1[30]*Gu1[25] + Gx1[31]*Gu1[29] + Gx1[32]*Gu1[33] + Gx1[33]*Gu1[37] + Gx1[34]*Gu1[41] + Gx1[35]*Gu1[45];
Gu2[10] = + Gx1[24]*Gu1[2] + Gx1[25]*Gu1[6] + Gx1[26]*Gu1[10] + Gx1[27]*Gu1[14] + Gx1[28]*Gu1[18] + Gx1[29]*Gu1[22] + Gx1[30]*Gu1[26] + Gx1[31]*Gu1[30] + Gx1[32]*Gu1[34] + Gx1[33]*Gu1[38] + Gx1[34]*Gu1[42] + Gx1[35]*Gu1[46];
Gu2[11] = + Gx1[24]*Gu1[3] + Gx1[25]*Gu1[7] + Gx1[26]*Gu1[11] + Gx1[27]*Gu1[15] + Gx1[28]*Gu1[19] + Gx1[29]*Gu1[23] + Gx1[30]*Gu1[27] + Gx1[31]*Gu1[31] + Gx1[32]*Gu1[35] + Gx1[33]*Gu1[39] + Gx1[34]*Gu1[43] + Gx1[35]*Gu1[47];
Gu2[12] = + Gx1[36]*Gu1[0] + Gx1[37]*Gu1[4] + Gx1[38]*Gu1[8] + Gx1[39]*Gu1[12] + Gx1[40]*Gu1[16] + Gx1[41]*Gu1[20] + Gx1[42]*Gu1[24] + Gx1[43]*Gu1[28] + Gx1[44]*Gu1[32] + Gx1[45]*Gu1[36] + Gx1[46]*Gu1[40] + Gx1[47]*Gu1[44];
Gu2[13] = + Gx1[36]*Gu1[1] + Gx1[37]*Gu1[5] + Gx1[38]*Gu1[9] + Gx1[39]*Gu1[13] + Gx1[40]*Gu1[17] + Gx1[41]*Gu1[21] + Gx1[42]*Gu1[25] + Gx1[43]*Gu1[29] + Gx1[44]*Gu1[33] + Gx1[45]*Gu1[37] + Gx1[46]*Gu1[41] + Gx1[47]*Gu1[45];
Gu2[14] = + Gx1[36]*Gu1[2] + Gx1[37]*Gu1[6] + Gx1[38]*Gu1[10] + Gx1[39]*Gu1[14] + Gx1[40]*Gu1[18] + Gx1[41]*Gu1[22] + Gx1[42]*Gu1[26] + Gx1[43]*Gu1[30] + Gx1[44]*Gu1[34] + Gx1[45]*Gu1[38] + Gx1[46]*Gu1[42] + Gx1[47]*Gu1[46];
Gu2[15] = + Gx1[36]*Gu1[3] + Gx1[37]*Gu1[7] + Gx1[38]*Gu1[11] + Gx1[39]*Gu1[15] + Gx1[40]*Gu1[19] + Gx1[41]*Gu1[23] + Gx1[42]*Gu1[27] + Gx1[43]*Gu1[31] + Gx1[44]*Gu1[35] + Gx1[45]*Gu1[39] + Gx1[46]*Gu1[43] + Gx1[47]*Gu1[47];
Gu2[16] = + Gx1[48]*Gu1[0] + Gx1[49]*Gu1[4] + Gx1[50]*Gu1[8] + Gx1[51]*Gu1[12] + Gx1[52]*Gu1[16] + Gx1[53]*Gu1[20] + Gx1[54]*Gu1[24] + Gx1[55]*Gu1[28] + Gx1[56]*Gu1[32] + Gx1[57]*Gu1[36] + Gx1[58]*Gu1[40] + Gx1[59]*Gu1[44];
Gu2[17] = + Gx1[48]*Gu1[1] + Gx1[49]*Gu1[5] + Gx1[50]*Gu1[9] + Gx1[51]*Gu1[13] + Gx1[52]*Gu1[17] + Gx1[53]*Gu1[21] + Gx1[54]*Gu1[25] + Gx1[55]*Gu1[29] + Gx1[56]*Gu1[33] + Gx1[57]*Gu1[37] + Gx1[58]*Gu1[41] + Gx1[59]*Gu1[45];
Gu2[18] = + Gx1[48]*Gu1[2] + Gx1[49]*Gu1[6] + Gx1[50]*Gu1[10] + Gx1[51]*Gu1[14] + Gx1[52]*Gu1[18] + Gx1[53]*Gu1[22] + Gx1[54]*Gu1[26] + Gx1[55]*Gu1[30] + Gx1[56]*Gu1[34] + Gx1[57]*Gu1[38] + Gx1[58]*Gu1[42] + Gx1[59]*Gu1[46];
Gu2[19] = + Gx1[48]*Gu1[3] + Gx1[49]*Gu1[7] + Gx1[50]*Gu1[11] + Gx1[51]*Gu1[15] + Gx1[52]*Gu1[19] + Gx1[53]*Gu1[23] + Gx1[54]*Gu1[27] + Gx1[55]*Gu1[31] + Gx1[56]*Gu1[35] + Gx1[57]*Gu1[39] + Gx1[58]*Gu1[43] + Gx1[59]*Gu1[47];
Gu2[20] = + Gx1[60]*Gu1[0] + Gx1[61]*Gu1[4] + Gx1[62]*Gu1[8] + Gx1[63]*Gu1[12] + Gx1[64]*Gu1[16] + Gx1[65]*Gu1[20] + Gx1[66]*Gu1[24] + Gx1[67]*Gu1[28] + Gx1[68]*Gu1[32] + Gx1[69]*Gu1[36] + Gx1[70]*Gu1[40] + Gx1[71]*Gu1[44];
Gu2[21] = + Gx1[60]*Gu1[1] + Gx1[61]*Gu1[5] + Gx1[62]*Gu1[9] + Gx1[63]*Gu1[13] + Gx1[64]*Gu1[17] + Gx1[65]*Gu1[21] + Gx1[66]*Gu1[25] + Gx1[67]*Gu1[29] + Gx1[68]*Gu1[33] + Gx1[69]*Gu1[37] + Gx1[70]*Gu1[41] + Gx1[71]*Gu1[45];
Gu2[22] = + Gx1[60]*Gu1[2] + Gx1[61]*Gu1[6] + Gx1[62]*Gu1[10] + Gx1[63]*Gu1[14] + Gx1[64]*Gu1[18] + Gx1[65]*Gu1[22] + Gx1[66]*Gu1[26] + Gx1[67]*Gu1[30] + Gx1[68]*Gu1[34] + Gx1[69]*Gu1[38] + Gx1[70]*Gu1[42] + Gx1[71]*Gu1[46];
Gu2[23] = + Gx1[60]*Gu1[3] + Gx1[61]*Gu1[7] + Gx1[62]*Gu1[11] + Gx1[63]*Gu1[15] + Gx1[64]*Gu1[19] + Gx1[65]*Gu1[23] + Gx1[66]*Gu1[27] + Gx1[67]*Gu1[31] + Gx1[68]*Gu1[35] + Gx1[69]*Gu1[39] + Gx1[70]*Gu1[43] + Gx1[71]*Gu1[47];
Gu2[24] = + Gx1[72]*Gu1[0] + Gx1[73]*Gu1[4] + Gx1[74]*Gu1[8] + Gx1[75]*Gu1[12] + Gx1[76]*Gu1[16] + Gx1[77]*Gu1[20] + Gx1[78]*Gu1[24] + Gx1[79]*Gu1[28] + Gx1[80]*Gu1[32] + Gx1[81]*Gu1[36] + Gx1[82]*Gu1[40] + Gx1[83]*Gu1[44];
Gu2[25] = + Gx1[72]*Gu1[1] + Gx1[73]*Gu1[5] + Gx1[74]*Gu1[9] + Gx1[75]*Gu1[13] + Gx1[76]*Gu1[17] + Gx1[77]*Gu1[21] + Gx1[78]*Gu1[25] + Gx1[79]*Gu1[29] + Gx1[80]*Gu1[33] + Gx1[81]*Gu1[37] + Gx1[82]*Gu1[41] + Gx1[83]*Gu1[45];
Gu2[26] = + Gx1[72]*Gu1[2] + Gx1[73]*Gu1[6] + Gx1[74]*Gu1[10] + Gx1[75]*Gu1[14] + Gx1[76]*Gu1[18] + Gx1[77]*Gu1[22] + Gx1[78]*Gu1[26] + Gx1[79]*Gu1[30] + Gx1[80]*Gu1[34] + Gx1[81]*Gu1[38] + Gx1[82]*Gu1[42] + Gx1[83]*Gu1[46];
Gu2[27] = + Gx1[72]*Gu1[3] + Gx1[73]*Gu1[7] + Gx1[74]*Gu1[11] + Gx1[75]*Gu1[15] + Gx1[76]*Gu1[19] + Gx1[77]*Gu1[23] + Gx1[78]*Gu1[27] + Gx1[79]*Gu1[31] + Gx1[80]*Gu1[35] + Gx1[81]*Gu1[39] + Gx1[82]*Gu1[43] + Gx1[83]*Gu1[47];
Gu2[28] = + Gx1[84]*Gu1[0] + Gx1[85]*Gu1[4] + Gx1[86]*Gu1[8] + Gx1[87]*Gu1[12] + Gx1[88]*Gu1[16] + Gx1[89]*Gu1[20] + Gx1[90]*Gu1[24] + Gx1[91]*Gu1[28] + Gx1[92]*Gu1[32] + Gx1[93]*Gu1[36] + Gx1[94]*Gu1[40] + Gx1[95]*Gu1[44];
Gu2[29] = + Gx1[84]*Gu1[1] + Gx1[85]*Gu1[5] + Gx1[86]*Gu1[9] + Gx1[87]*Gu1[13] + Gx1[88]*Gu1[17] + Gx1[89]*Gu1[21] + Gx1[90]*Gu1[25] + Gx1[91]*Gu1[29] + Gx1[92]*Gu1[33] + Gx1[93]*Gu1[37] + Gx1[94]*Gu1[41] + Gx1[95]*Gu1[45];
Gu2[30] = + Gx1[84]*Gu1[2] + Gx1[85]*Gu1[6] + Gx1[86]*Gu1[10] + Gx1[87]*Gu1[14] + Gx1[88]*Gu1[18] + Gx1[89]*Gu1[22] + Gx1[90]*Gu1[26] + Gx1[91]*Gu1[30] + Gx1[92]*Gu1[34] + Gx1[93]*Gu1[38] + Gx1[94]*Gu1[42] + Gx1[95]*Gu1[46];
Gu2[31] = + Gx1[84]*Gu1[3] + Gx1[85]*Gu1[7] + Gx1[86]*Gu1[11] + Gx1[87]*Gu1[15] + Gx1[88]*Gu1[19] + Gx1[89]*Gu1[23] + Gx1[90]*Gu1[27] + Gx1[91]*Gu1[31] + Gx1[92]*Gu1[35] + Gx1[93]*Gu1[39] + Gx1[94]*Gu1[43] + Gx1[95]*Gu1[47];
Gu2[32] = + Gx1[96]*Gu1[0] + Gx1[97]*Gu1[4] + Gx1[98]*Gu1[8] + Gx1[99]*Gu1[12] + Gx1[100]*Gu1[16] + Gx1[101]*Gu1[20] + Gx1[102]*Gu1[24] + Gx1[103]*Gu1[28] + Gx1[104]*Gu1[32] + Gx1[105]*Gu1[36] + Gx1[106]*Gu1[40] + Gx1[107]*Gu1[44];
Gu2[33] = + Gx1[96]*Gu1[1] + Gx1[97]*Gu1[5] + Gx1[98]*Gu1[9] + Gx1[99]*Gu1[13] + Gx1[100]*Gu1[17] + Gx1[101]*Gu1[21] + Gx1[102]*Gu1[25] + Gx1[103]*Gu1[29] + Gx1[104]*Gu1[33] + Gx1[105]*Gu1[37] + Gx1[106]*Gu1[41] + Gx1[107]*Gu1[45];
Gu2[34] = + Gx1[96]*Gu1[2] + Gx1[97]*Gu1[6] + Gx1[98]*Gu1[10] + Gx1[99]*Gu1[14] + Gx1[100]*Gu1[18] + Gx1[101]*Gu1[22] + Gx1[102]*Gu1[26] + Gx1[103]*Gu1[30] + Gx1[104]*Gu1[34] + Gx1[105]*Gu1[38] + Gx1[106]*Gu1[42] + Gx1[107]*Gu1[46];
Gu2[35] = + Gx1[96]*Gu1[3] + Gx1[97]*Gu1[7] + Gx1[98]*Gu1[11] + Gx1[99]*Gu1[15] + Gx1[100]*Gu1[19] + Gx1[101]*Gu1[23] + Gx1[102]*Gu1[27] + Gx1[103]*Gu1[31] + Gx1[104]*Gu1[35] + Gx1[105]*Gu1[39] + Gx1[106]*Gu1[43] + Gx1[107]*Gu1[47];
Gu2[36] = + Gx1[108]*Gu1[0] + Gx1[109]*Gu1[4] + Gx1[110]*Gu1[8] + Gx1[111]*Gu1[12] + Gx1[112]*Gu1[16] + Gx1[113]*Gu1[20] + Gx1[114]*Gu1[24] + Gx1[115]*Gu1[28] + Gx1[116]*Gu1[32] + Gx1[117]*Gu1[36] + Gx1[118]*Gu1[40] + Gx1[119]*Gu1[44];
Gu2[37] = + Gx1[108]*Gu1[1] + Gx1[109]*Gu1[5] + Gx1[110]*Gu1[9] + Gx1[111]*Gu1[13] + Gx1[112]*Gu1[17] + Gx1[113]*Gu1[21] + Gx1[114]*Gu1[25] + Gx1[115]*Gu1[29] + Gx1[116]*Gu1[33] + Gx1[117]*Gu1[37] + Gx1[118]*Gu1[41] + Gx1[119]*Gu1[45];
Gu2[38] = + Gx1[108]*Gu1[2] + Gx1[109]*Gu1[6] + Gx1[110]*Gu1[10] + Gx1[111]*Gu1[14] + Gx1[112]*Gu1[18] + Gx1[113]*Gu1[22] + Gx1[114]*Gu1[26] + Gx1[115]*Gu1[30] + Gx1[116]*Gu1[34] + Gx1[117]*Gu1[38] + Gx1[118]*Gu1[42] + Gx1[119]*Gu1[46];
Gu2[39] = + Gx1[108]*Gu1[3] + Gx1[109]*Gu1[7] + Gx1[110]*Gu1[11] + Gx1[111]*Gu1[15] + Gx1[112]*Gu1[19] + Gx1[113]*Gu1[23] + Gx1[114]*Gu1[27] + Gx1[115]*Gu1[31] + Gx1[116]*Gu1[35] + Gx1[117]*Gu1[39] + Gx1[118]*Gu1[43] + Gx1[119]*Gu1[47];
Gu2[40] = + Gx1[120]*Gu1[0] + Gx1[121]*Gu1[4] + Gx1[122]*Gu1[8] + Gx1[123]*Gu1[12] + Gx1[124]*Gu1[16] + Gx1[125]*Gu1[20] + Gx1[126]*Gu1[24] + Gx1[127]*Gu1[28] + Gx1[128]*Gu1[32] + Gx1[129]*Gu1[36] + Gx1[130]*Gu1[40] + Gx1[131]*Gu1[44];
Gu2[41] = + Gx1[120]*Gu1[1] + Gx1[121]*Gu1[5] + Gx1[122]*Gu1[9] + Gx1[123]*Gu1[13] + Gx1[124]*Gu1[17] + Gx1[125]*Gu1[21] + Gx1[126]*Gu1[25] + Gx1[127]*Gu1[29] + Gx1[128]*Gu1[33] + Gx1[129]*Gu1[37] + Gx1[130]*Gu1[41] + Gx1[131]*Gu1[45];
Gu2[42] = + Gx1[120]*Gu1[2] + Gx1[121]*Gu1[6] + Gx1[122]*Gu1[10] + Gx1[123]*Gu1[14] + Gx1[124]*Gu1[18] + Gx1[125]*Gu1[22] + Gx1[126]*Gu1[26] + Gx1[127]*Gu1[30] + Gx1[128]*Gu1[34] + Gx1[129]*Gu1[38] + Gx1[130]*Gu1[42] + Gx1[131]*Gu1[46];
Gu2[43] = + Gx1[120]*Gu1[3] + Gx1[121]*Gu1[7] + Gx1[122]*Gu1[11] + Gx1[123]*Gu1[15] + Gx1[124]*Gu1[19] + Gx1[125]*Gu1[23] + Gx1[126]*Gu1[27] + Gx1[127]*Gu1[31] + Gx1[128]*Gu1[35] + Gx1[129]*Gu1[39] + Gx1[130]*Gu1[43] + Gx1[131]*Gu1[47];
Gu2[44] = + Gx1[132]*Gu1[0] + Gx1[133]*Gu1[4] + Gx1[134]*Gu1[8] + Gx1[135]*Gu1[12] + Gx1[136]*Gu1[16] + Gx1[137]*Gu1[20] + Gx1[138]*Gu1[24] + Gx1[139]*Gu1[28] + Gx1[140]*Gu1[32] + Gx1[141]*Gu1[36] + Gx1[142]*Gu1[40] + Gx1[143]*Gu1[44];
Gu2[45] = + Gx1[132]*Gu1[1] + Gx1[133]*Gu1[5] + Gx1[134]*Gu1[9] + Gx1[135]*Gu1[13] + Gx1[136]*Gu1[17] + Gx1[137]*Gu1[21] + Gx1[138]*Gu1[25] + Gx1[139]*Gu1[29] + Gx1[140]*Gu1[33] + Gx1[141]*Gu1[37] + Gx1[142]*Gu1[41] + Gx1[143]*Gu1[45];
Gu2[46] = + Gx1[132]*Gu1[2] + Gx1[133]*Gu1[6] + Gx1[134]*Gu1[10] + Gx1[135]*Gu1[14] + Gx1[136]*Gu1[18] + Gx1[137]*Gu1[22] + Gx1[138]*Gu1[26] + Gx1[139]*Gu1[30] + Gx1[140]*Gu1[34] + Gx1[141]*Gu1[38] + Gx1[142]*Gu1[42] + Gx1[143]*Gu1[46];
Gu2[47] = + Gx1[132]*Gu1[3] + Gx1[133]*Gu1[7] + Gx1[134]*Gu1[11] + Gx1[135]*Gu1[15] + Gx1[136]*Gu1[19] + Gx1[137]*Gu1[23] + Gx1[138]*Gu1[27] + Gx1[139]*Gu1[31] + Gx1[140]*Gu1[35] + Gx1[141]*Gu1[39] + Gx1[142]*Gu1[43] + Gx1[143]*Gu1[47];
}

void nmpc_moveGuE( real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = Gu1[0];
Gu2[1] = Gu1[1];
Gu2[2] = Gu1[2];
Gu2[3] = Gu1[3];
Gu2[4] = Gu1[4];
Gu2[5] = Gu1[5];
Gu2[6] = Gu1[6];
Gu2[7] = Gu1[7];
Gu2[8] = Gu1[8];
Gu2[9] = Gu1[9];
Gu2[10] = Gu1[10];
Gu2[11] = Gu1[11];
Gu2[12] = Gu1[12];
Gu2[13] = Gu1[13];
Gu2[14] = Gu1[14];
Gu2[15] = Gu1[15];
Gu2[16] = Gu1[16];
Gu2[17] = Gu1[17];
Gu2[18] = Gu1[18];
Gu2[19] = Gu1[19];
Gu2[20] = Gu1[20];
Gu2[21] = Gu1[21];
Gu2[22] = Gu1[22];
Gu2[23] = Gu1[23];
Gu2[24] = Gu1[24];
Gu2[25] = Gu1[25];
Gu2[26] = Gu1[26];
Gu2[27] = Gu1[27];
Gu2[28] = Gu1[28];
Gu2[29] = Gu1[29];
Gu2[30] = Gu1[30];
Gu2[31] = Gu1[31];
Gu2[32] = Gu1[32];
Gu2[33] = Gu1[33];
Gu2[34] = Gu1[34];
Gu2[35] = Gu1[35];
Gu2[36] = Gu1[36];
Gu2[37] = Gu1[37];
Gu2[38] = Gu1[38];
Gu2[39] = Gu1[39];
Gu2[40] = Gu1[40];
Gu2[41] = Gu1[41];
Gu2[42] = Gu1[42];
Gu2[43] = Gu1[43];
Gu2[44] = Gu1[44];
Gu2[45] = Gu1[45];
Gu2[46] = Gu1[46];
Gu2[47] = Gu1[47];
}

void nmpc_setBlockH11( int iRow, int iCol, real_t* const Gu1, real_t* const Gu2 )
{
nmpcWorkspace.H[(iRow * 528 + 1584) + (iCol * 4 + 12)] += + Gu1[0]*Gu2[0] + Gu1[4]*Gu2[4] + Gu1[8]*Gu2[8] + Gu1[12]*Gu2[12] + Gu1[16]*Gu2[16] + Gu1[20]*Gu2[20] + Gu1[24]*Gu2[24] + Gu1[28]*Gu2[28] + Gu1[32]*Gu2[32] + Gu1[36]*Gu2[36] + Gu1[40]*Gu2[40] + Gu1[44]*Gu2[44];
nmpcWorkspace.H[(iRow * 528 + 1584) + (iCol * 4 + 13)] += + Gu1[0]*Gu2[1] + Gu1[4]*Gu2[5] + Gu1[8]*Gu2[9] + Gu1[12]*Gu2[13] + Gu1[16]*Gu2[17] + Gu1[20]*Gu2[21] + Gu1[24]*Gu2[25] + Gu1[28]*Gu2[29] + Gu1[32]*Gu2[33] + Gu1[36]*Gu2[37] + Gu1[40]*Gu2[41] + Gu1[44]*Gu2[45];
nmpcWorkspace.H[(iRow * 528 + 1584) + (iCol * 4 + 14)] += + Gu1[0]*Gu2[2] + Gu1[4]*Gu2[6] + Gu1[8]*Gu2[10] + Gu1[12]*Gu2[14] + Gu1[16]*Gu2[18] + Gu1[20]*Gu2[22] + Gu1[24]*Gu2[26] + Gu1[28]*Gu2[30] + Gu1[32]*Gu2[34] + Gu1[36]*Gu2[38] + Gu1[40]*Gu2[42] + Gu1[44]*Gu2[46];
nmpcWorkspace.H[(iRow * 528 + 1584) + (iCol * 4 + 15)] += + Gu1[0]*Gu2[3] + Gu1[4]*Gu2[7] + Gu1[8]*Gu2[11] + Gu1[12]*Gu2[15] + Gu1[16]*Gu2[19] + Gu1[20]*Gu2[23] + Gu1[24]*Gu2[27] + Gu1[28]*Gu2[31] + Gu1[32]*Gu2[35] + Gu1[36]*Gu2[39] + Gu1[40]*Gu2[43] + Gu1[44]*Gu2[47];
nmpcWorkspace.H[(iRow * 528 + 1716) + (iCol * 4 + 12)] += + Gu1[1]*Gu2[0] + Gu1[5]*Gu2[4] + Gu1[9]*Gu2[8] + Gu1[13]*Gu2[12] + Gu1[17]*Gu2[16] + Gu1[21]*Gu2[20] + Gu1[25]*Gu2[24] + Gu1[29]*Gu2[28] + Gu1[33]*Gu2[32] + Gu1[37]*Gu2[36] + Gu1[41]*Gu2[40] + Gu1[45]*Gu2[44];
nmpcWorkspace.H[(iRow * 528 + 1716) + (iCol * 4 + 13)] += + Gu1[1]*Gu2[1] + Gu1[5]*Gu2[5] + Gu1[9]*Gu2[9] + Gu1[13]*Gu2[13] + Gu1[17]*Gu2[17] + Gu1[21]*Gu2[21] + Gu1[25]*Gu2[25] + Gu1[29]*Gu2[29] + Gu1[33]*Gu2[33] + Gu1[37]*Gu2[37] + Gu1[41]*Gu2[41] + Gu1[45]*Gu2[45];
nmpcWorkspace.H[(iRow * 528 + 1716) + (iCol * 4 + 14)] += + Gu1[1]*Gu2[2] + Gu1[5]*Gu2[6] + Gu1[9]*Gu2[10] + Gu1[13]*Gu2[14] + Gu1[17]*Gu2[18] + Gu1[21]*Gu2[22] + Gu1[25]*Gu2[26] + Gu1[29]*Gu2[30] + Gu1[33]*Gu2[34] + Gu1[37]*Gu2[38] + Gu1[41]*Gu2[42] + Gu1[45]*Gu2[46];
nmpcWorkspace.H[(iRow * 528 + 1716) + (iCol * 4 + 15)] += + Gu1[1]*Gu2[3] + Gu1[5]*Gu2[7] + Gu1[9]*Gu2[11] + Gu1[13]*Gu2[15] + Gu1[17]*Gu2[19] + Gu1[21]*Gu2[23] + Gu1[25]*Gu2[27] + Gu1[29]*Gu2[31] + Gu1[33]*Gu2[35] + Gu1[37]*Gu2[39] + Gu1[41]*Gu2[43] + Gu1[45]*Gu2[47];
nmpcWorkspace.H[(iRow * 528 + 1848) + (iCol * 4 + 12)] += + Gu1[2]*Gu2[0] + Gu1[6]*Gu2[4] + Gu1[10]*Gu2[8] + Gu1[14]*Gu2[12] + Gu1[18]*Gu2[16] + Gu1[22]*Gu2[20] + Gu1[26]*Gu2[24] + Gu1[30]*Gu2[28] + Gu1[34]*Gu2[32] + Gu1[38]*Gu2[36] + Gu1[42]*Gu2[40] + Gu1[46]*Gu2[44];
nmpcWorkspace.H[(iRow * 528 + 1848) + (iCol * 4 + 13)] += + Gu1[2]*Gu2[1] + Gu1[6]*Gu2[5] + Gu1[10]*Gu2[9] + Gu1[14]*Gu2[13] + Gu1[18]*Gu2[17] + Gu1[22]*Gu2[21] + Gu1[26]*Gu2[25] + Gu1[30]*Gu2[29] + Gu1[34]*Gu2[33] + Gu1[38]*Gu2[37] + Gu1[42]*Gu2[41] + Gu1[46]*Gu2[45];
nmpcWorkspace.H[(iRow * 528 + 1848) + (iCol * 4 + 14)] += + Gu1[2]*Gu2[2] + Gu1[6]*Gu2[6] + Gu1[10]*Gu2[10] + Gu1[14]*Gu2[14] + Gu1[18]*Gu2[18] + Gu1[22]*Gu2[22] + Gu1[26]*Gu2[26] + Gu1[30]*Gu2[30] + Gu1[34]*Gu2[34] + Gu1[38]*Gu2[38] + Gu1[42]*Gu2[42] + Gu1[46]*Gu2[46];
nmpcWorkspace.H[(iRow * 528 + 1848) + (iCol * 4 + 15)] += + Gu1[2]*Gu2[3] + Gu1[6]*Gu2[7] + Gu1[10]*Gu2[11] + Gu1[14]*Gu2[15] + Gu1[18]*Gu2[19] + Gu1[22]*Gu2[23] + Gu1[26]*Gu2[27] + Gu1[30]*Gu2[31] + Gu1[34]*Gu2[35] + Gu1[38]*Gu2[39] + Gu1[42]*Gu2[43] + Gu1[46]*Gu2[47];
nmpcWorkspace.H[(iRow * 528 + 1980) + (iCol * 4 + 12)] += + Gu1[3]*Gu2[0] + Gu1[7]*Gu2[4] + Gu1[11]*Gu2[8] + Gu1[15]*Gu2[12] + Gu1[19]*Gu2[16] + Gu1[23]*Gu2[20] + Gu1[27]*Gu2[24] + Gu1[31]*Gu2[28] + Gu1[35]*Gu2[32] + Gu1[39]*Gu2[36] + Gu1[43]*Gu2[40] + Gu1[47]*Gu2[44];
nmpcWorkspace.H[(iRow * 528 + 1980) + (iCol * 4 + 13)] += + Gu1[3]*Gu2[1] + Gu1[7]*Gu2[5] + Gu1[11]*Gu2[9] + Gu1[15]*Gu2[13] + Gu1[19]*Gu2[17] + Gu1[23]*Gu2[21] + Gu1[27]*Gu2[25] + Gu1[31]*Gu2[29] + Gu1[35]*Gu2[33] + Gu1[39]*Gu2[37] + Gu1[43]*Gu2[41] + Gu1[47]*Gu2[45];
nmpcWorkspace.H[(iRow * 528 + 1980) + (iCol * 4 + 14)] += + Gu1[3]*Gu2[2] + Gu1[7]*Gu2[6] + Gu1[11]*Gu2[10] + Gu1[15]*Gu2[14] + Gu1[19]*Gu2[18] + Gu1[23]*Gu2[22] + Gu1[27]*Gu2[26] + Gu1[31]*Gu2[30] + Gu1[35]*Gu2[34] + Gu1[39]*Gu2[38] + Gu1[43]*Gu2[42] + Gu1[47]*Gu2[46];
nmpcWorkspace.H[(iRow * 528 + 1980) + (iCol * 4 + 15)] += + Gu1[3]*Gu2[3] + Gu1[7]*Gu2[7] + Gu1[11]*Gu2[11] + Gu1[15]*Gu2[15] + Gu1[19]*Gu2[19] + Gu1[23]*Gu2[23] + Gu1[27]*Gu2[27] + Gu1[31]*Gu2[31] + Gu1[35]*Gu2[35] + Gu1[39]*Gu2[39] + Gu1[43]*Gu2[43] + Gu1[47]*Gu2[47];
}

void nmpc_setBlockH11_R1( int iRow, int iCol, real_t* const R11 )
{
nmpcWorkspace.H[(iRow * 528 + 1584) + (iCol * 4 + 12)] = R11[0];
nmpcWorkspace.H[(iRow * 528 + 1584) + (iCol * 4 + 13)] = R11[1];
nmpcWorkspace.H[(iRow * 528 + 1584) + (iCol * 4 + 14)] = R11[2];
nmpcWorkspace.H[(iRow * 528 + 1584) + (iCol * 4 + 15)] = R11[3];
nmpcWorkspace.H[(iRow * 528 + 1716) + (iCol * 4 + 12)] = R11[4];
nmpcWorkspace.H[(iRow * 528 + 1716) + (iCol * 4 + 13)] = R11[5];
nmpcWorkspace.H[(iRow * 528 + 1716) + (iCol * 4 + 14)] = R11[6];
nmpcWorkspace.H[(iRow * 528 + 1716) + (iCol * 4 + 15)] = R11[7];
nmpcWorkspace.H[(iRow * 528 + 1848) + (iCol * 4 + 12)] = R11[8];
nmpcWorkspace.H[(iRow * 528 + 1848) + (iCol * 4 + 13)] = R11[9];
nmpcWorkspace.H[(iRow * 528 + 1848) + (iCol * 4 + 14)] = R11[10];
nmpcWorkspace.H[(iRow * 528 + 1848) + (iCol * 4 + 15)] = R11[11];
nmpcWorkspace.H[(iRow * 528 + 1980) + (iCol * 4 + 12)] = R11[12];
nmpcWorkspace.H[(iRow * 528 + 1980) + (iCol * 4 + 13)] = R11[13];
nmpcWorkspace.H[(iRow * 528 + 1980) + (iCol * 4 + 14)] = R11[14];
nmpcWorkspace.H[(iRow * 528 + 1980) + (iCol * 4 + 15)] = R11[15];
}

void nmpc_zeroBlockH11( int iRow, int iCol )
{
nmpcWorkspace.H[(iRow * 528 + 1584) + (iCol * 4 + 12)] = 0.0000000000000000e+00;
nmpcWorkspace.H[(iRow * 528 + 1584) + (iCol * 4 + 13)] = 0.0000000000000000e+00;
nmpcWorkspace.H[(iRow * 528 + 1584) + (iCol * 4 + 14)] = 0.0000000000000000e+00;
nmpcWorkspace.H[(iRow * 528 + 1584) + (iCol * 4 + 15)] = 0.0000000000000000e+00;
nmpcWorkspace.H[(iRow * 528 + 1716) + (iCol * 4 + 12)] = 0.0000000000000000e+00;
nmpcWorkspace.H[(iRow * 528 + 1716) + (iCol * 4 + 13)] = 0.0000000000000000e+00;
nmpcWorkspace.H[(iRow * 528 + 1716) + (iCol * 4 + 14)] = 0.0000000000000000e+00;
nmpcWorkspace.H[(iRow * 528 + 1716) + (iCol * 4 + 15)] = 0.0000000000000000e+00;
nmpcWorkspace.H[(iRow * 528 + 1848) + (iCol * 4 + 12)] = 0.0000000000000000e+00;
nmpcWorkspace.H[(iRow * 528 + 1848) + (iCol * 4 + 13)] = 0.0000000000000000e+00;
nmpcWorkspace.H[(iRow * 528 + 1848) + (iCol * 4 + 14)] = 0.0000000000000000e+00;
nmpcWorkspace.H[(iRow * 528 + 1848) + (iCol * 4 + 15)] = 0.0000000000000000e+00;
nmpcWorkspace.H[(iRow * 528 + 1980) + (iCol * 4 + 12)] = 0.0000000000000000e+00;
nmpcWorkspace.H[(iRow * 528 + 1980) + (iCol * 4 + 13)] = 0.0000000000000000e+00;
nmpcWorkspace.H[(iRow * 528 + 1980) + (iCol * 4 + 14)] = 0.0000000000000000e+00;
nmpcWorkspace.H[(iRow * 528 + 1980) + (iCol * 4 + 15)] = 0.0000000000000000e+00;
}

void nmpc_copyHTH( int iRow, int iCol )
{
nmpcWorkspace.H[(iRow * 528 + 1584) + (iCol * 4 + 12)] = nmpcWorkspace.H[(iCol * 528 + 1584) + (iRow * 4 + 12)];
nmpcWorkspace.H[(iRow * 528 + 1584) + (iCol * 4 + 13)] = nmpcWorkspace.H[(iCol * 528 + 1716) + (iRow * 4 + 12)];
nmpcWorkspace.H[(iRow * 528 + 1584) + (iCol * 4 + 14)] = nmpcWorkspace.H[(iCol * 528 + 1848) + (iRow * 4 + 12)];
nmpcWorkspace.H[(iRow * 528 + 1584) + (iCol * 4 + 15)] = nmpcWorkspace.H[(iCol * 528 + 1980) + (iRow * 4 + 12)];
nmpcWorkspace.H[(iRow * 528 + 1716) + (iCol * 4 + 12)] = nmpcWorkspace.H[(iCol * 528 + 1584) + (iRow * 4 + 13)];
nmpcWorkspace.H[(iRow * 528 + 1716) + (iCol * 4 + 13)] = nmpcWorkspace.H[(iCol * 528 + 1716) + (iRow * 4 + 13)];
nmpcWorkspace.H[(iRow * 528 + 1716) + (iCol * 4 + 14)] = nmpcWorkspace.H[(iCol * 528 + 1848) + (iRow * 4 + 13)];
nmpcWorkspace.H[(iRow * 528 + 1716) + (iCol * 4 + 15)] = nmpcWorkspace.H[(iCol * 528 + 1980) + (iRow * 4 + 13)];
nmpcWorkspace.H[(iRow * 528 + 1848) + (iCol * 4 + 12)] = nmpcWorkspace.H[(iCol * 528 + 1584) + (iRow * 4 + 14)];
nmpcWorkspace.H[(iRow * 528 + 1848) + (iCol * 4 + 13)] = nmpcWorkspace.H[(iCol * 528 + 1716) + (iRow * 4 + 14)];
nmpcWorkspace.H[(iRow * 528 + 1848) + (iCol * 4 + 14)] = nmpcWorkspace.H[(iCol * 528 + 1848) + (iRow * 4 + 14)];
nmpcWorkspace.H[(iRow * 528 + 1848) + (iCol * 4 + 15)] = nmpcWorkspace.H[(iCol * 528 + 1980) + (iRow * 4 + 14)];
nmpcWorkspace.H[(iRow * 528 + 1980) + (iCol * 4 + 12)] = nmpcWorkspace.H[(iCol * 528 + 1584) + (iRow * 4 + 15)];
nmpcWorkspace.H[(iRow * 528 + 1980) + (iCol * 4 + 13)] = nmpcWorkspace.H[(iCol * 528 + 1716) + (iRow * 4 + 15)];
nmpcWorkspace.H[(iRow * 528 + 1980) + (iCol * 4 + 14)] = nmpcWorkspace.H[(iCol * 528 + 1848) + (iRow * 4 + 15)];
nmpcWorkspace.H[(iRow * 528 + 1980) + (iCol * 4 + 15)] = nmpcWorkspace.H[(iCol * 528 + 1980) + (iRow * 4 + 15)];
}

void nmpc_multQ1d( real_t* const Gx1, real_t* const dOld, real_t* const dNew )
{
dNew[0] = + Gx1[0]*dOld[0] + Gx1[1]*dOld[1] + Gx1[2]*dOld[2] + Gx1[3]*dOld[3] + Gx1[4]*dOld[4] + Gx1[5]*dOld[5] + Gx1[6]*dOld[6] + Gx1[7]*dOld[7] + Gx1[8]*dOld[8] + Gx1[9]*dOld[9] + Gx1[10]*dOld[10] + Gx1[11]*dOld[11];
dNew[1] = + Gx1[12]*dOld[0] + Gx1[13]*dOld[1] + Gx1[14]*dOld[2] + Gx1[15]*dOld[3] + Gx1[16]*dOld[4] + Gx1[17]*dOld[5] + Gx1[18]*dOld[6] + Gx1[19]*dOld[7] + Gx1[20]*dOld[8] + Gx1[21]*dOld[9] + Gx1[22]*dOld[10] + Gx1[23]*dOld[11];
dNew[2] = + Gx1[24]*dOld[0] + Gx1[25]*dOld[1] + Gx1[26]*dOld[2] + Gx1[27]*dOld[3] + Gx1[28]*dOld[4] + Gx1[29]*dOld[5] + Gx1[30]*dOld[6] + Gx1[31]*dOld[7] + Gx1[32]*dOld[8] + Gx1[33]*dOld[9] + Gx1[34]*dOld[10] + Gx1[35]*dOld[11];
dNew[3] = + Gx1[36]*dOld[0] + Gx1[37]*dOld[1] + Gx1[38]*dOld[2] + Gx1[39]*dOld[3] + Gx1[40]*dOld[4] + Gx1[41]*dOld[5] + Gx1[42]*dOld[6] + Gx1[43]*dOld[7] + Gx1[44]*dOld[8] + Gx1[45]*dOld[9] + Gx1[46]*dOld[10] + Gx1[47]*dOld[11];
dNew[4] = + Gx1[48]*dOld[0] + Gx1[49]*dOld[1] + Gx1[50]*dOld[2] + Gx1[51]*dOld[3] + Gx1[52]*dOld[4] + Gx1[53]*dOld[5] + Gx1[54]*dOld[6] + Gx1[55]*dOld[7] + Gx1[56]*dOld[8] + Gx1[57]*dOld[9] + Gx1[58]*dOld[10] + Gx1[59]*dOld[11];
dNew[5] = + Gx1[60]*dOld[0] + Gx1[61]*dOld[1] + Gx1[62]*dOld[2] + Gx1[63]*dOld[3] + Gx1[64]*dOld[4] + Gx1[65]*dOld[5] + Gx1[66]*dOld[6] + Gx1[67]*dOld[7] + Gx1[68]*dOld[8] + Gx1[69]*dOld[9] + Gx1[70]*dOld[10] + Gx1[71]*dOld[11];
dNew[6] = + Gx1[72]*dOld[0] + Gx1[73]*dOld[1] + Gx1[74]*dOld[2] + Gx1[75]*dOld[3] + Gx1[76]*dOld[4] + Gx1[77]*dOld[5] + Gx1[78]*dOld[6] + Gx1[79]*dOld[7] + Gx1[80]*dOld[8] + Gx1[81]*dOld[9] + Gx1[82]*dOld[10] + Gx1[83]*dOld[11];
dNew[7] = + Gx1[84]*dOld[0] + Gx1[85]*dOld[1] + Gx1[86]*dOld[2] + Gx1[87]*dOld[3] + Gx1[88]*dOld[4] + Gx1[89]*dOld[5] + Gx1[90]*dOld[6] + Gx1[91]*dOld[7] + Gx1[92]*dOld[8] + Gx1[93]*dOld[9] + Gx1[94]*dOld[10] + Gx1[95]*dOld[11];
dNew[8] = + Gx1[96]*dOld[0] + Gx1[97]*dOld[1] + Gx1[98]*dOld[2] + Gx1[99]*dOld[3] + Gx1[100]*dOld[4] + Gx1[101]*dOld[5] + Gx1[102]*dOld[6] + Gx1[103]*dOld[7] + Gx1[104]*dOld[8] + Gx1[105]*dOld[9] + Gx1[106]*dOld[10] + Gx1[107]*dOld[11];
dNew[9] = + Gx1[108]*dOld[0] + Gx1[109]*dOld[1] + Gx1[110]*dOld[2] + Gx1[111]*dOld[3] + Gx1[112]*dOld[4] + Gx1[113]*dOld[5] + Gx1[114]*dOld[6] + Gx1[115]*dOld[7] + Gx1[116]*dOld[8] + Gx1[117]*dOld[9] + Gx1[118]*dOld[10] + Gx1[119]*dOld[11];
dNew[10] = + Gx1[120]*dOld[0] + Gx1[121]*dOld[1] + Gx1[122]*dOld[2] + Gx1[123]*dOld[3] + Gx1[124]*dOld[4] + Gx1[125]*dOld[5] + Gx1[126]*dOld[6] + Gx1[127]*dOld[7] + Gx1[128]*dOld[8] + Gx1[129]*dOld[9] + Gx1[130]*dOld[10] + Gx1[131]*dOld[11];
dNew[11] = + Gx1[132]*dOld[0] + Gx1[133]*dOld[1] + Gx1[134]*dOld[2] + Gx1[135]*dOld[3] + Gx1[136]*dOld[4] + Gx1[137]*dOld[5] + Gx1[138]*dOld[6] + Gx1[139]*dOld[7] + Gx1[140]*dOld[8] + Gx1[141]*dOld[9] + Gx1[142]*dOld[10] + Gx1[143]*dOld[11];
}

void nmpc_multQN1d( real_t* const QN1, real_t* const dOld, real_t* const dNew )
{
dNew[0] = + nmpcWorkspace.QN1[0]*dOld[0] + nmpcWorkspace.QN1[1]*dOld[1] + nmpcWorkspace.QN1[2]*dOld[2] + nmpcWorkspace.QN1[3]*dOld[3] + nmpcWorkspace.QN1[4]*dOld[4] + nmpcWorkspace.QN1[5]*dOld[5] + nmpcWorkspace.QN1[6]*dOld[6] + nmpcWorkspace.QN1[7]*dOld[7] + nmpcWorkspace.QN1[8]*dOld[8] + nmpcWorkspace.QN1[9]*dOld[9] + nmpcWorkspace.QN1[10]*dOld[10] + nmpcWorkspace.QN1[11]*dOld[11];
dNew[1] = + nmpcWorkspace.QN1[12]*dOld[0] + nmpcWorkspace.QN1[13]*dOld[1] + nmpcWorkspace.QN1[14]*dOld[2] + nmpcWorkspace.QN1[15]*dOld[3] + nmpcWorkspace.QN1[16]*dOld[4] + nmpcWorkspace.QN1[17]*dOld[5] + nmpcWorkspace.QN1[18]*dOld[6] + nmpcWorkspace.QN1[19]*dOld[7] + nmpcWorkspace.QN1[20]*dOld[8] + nmpcWorkspace.QN1[21]*dOld[9] + nmpcWorkspace.QN1[22]*dOld[10] + nmpcWorkspace.QN1[23]*dOld[11];
dNew[2] = + nmpcWorkspace.QN1[24]*dOld[0] + nmpcWorkspace.QN1[25]*dOld[1] + nmpcWorkspace.QN1[26]*dOld[2] + nmpcWorkspace.QN1[27]*dOld[3] + nmpcWorkspace.QN1[28]*dOld[4] + nmpcWorkspace.QN1[29]*dOld[5] + nmpcWorkspace.QN1[30]*dOld[6] + nmpcWorkspace.QN1[31]*dOld[7] + nmpcWorkspace.QN1[32]*dOld[8] + nmpcWorkspace.QN1[33]*dOld[9] + nmpcWorkspace.QN1[34]*dOld[10] + nmpcWorkspace.QN1[35]*dOld[11];
dNew[3] = + nmpcWorkspace.QN1[36]*dOld[0] + nmpcWorkspace.QN1[37]*dOld[1] + nmpcWorkspace.QN1[38]*dOld[2] + nmpcWorkspace.QN1[39]*dOld[3] + nmpcWorkspace.QN1[40]*dOld[4] + nmpcWorkspace.QN1[41]*dOld[5] + nmpcWorkspace.QN1[42]*dOld[6] + nmpcWorkspace.QN1[43]*dOld[7] + nmpcWorkspace.QN1[44]*dOld[8] + nmpcWorkspace.QN1[45]*dOld[9] + nmpcWorkspace.QN1[46]*dOld[10] + nmpcWorkspace.QN1[47]*dOld[11];
dNew[4] = + nmpcWorkspace.QN1[48]*dOld[0] + nmpcWorkspace.QN1[49]*dOld[1] + nmpcWorkspace.QN1[50]*dOld[2] + nmpcWorkspace.QN1[51]*dOld[3] + nmpcWorkspace.QN1[52]*dOld[4] + nmpcWorkspace.QN1[53]*dOld[5] + nmpcWorkspace.QN1[54]*dOld[6] + nmpcWorkspace.QN1[55]*dOld[7] + nmpcWorkspace.QN1[56]*dOld[8] + nmpcWorkspace.QN1[57]*dOld[9] + nmpcWorkspace.QN1[58]*dOld[10] + nmpcWorkspace.QN1[59]*dOld[11];
dNew[5] = + nmpcWorkspace.QN1[60]*dOld[0] + nmpcWorkspace.QN1[61]*dOld[1] + nmpcWorkspace.QN1[62]*dOld[2] + nmpcWorkspace.QN1[63]*dOld[3] + nmpcWorkspace.QN1[64]*dOld[4] + nmpcWorkspace.QN1[65]*dOld[5] + nmpcWorkspace.QN1[66]*dOld[6] + nmpcWorkspace.QN1[67]*dOld[7] + nmpcWorkspace.QN1[68]*dOld[8] + nmpcWorkspace.QN1[69]*dOld[9] + nmpcWorkspace.QN1[70]*dOld[10] + nmpcWorkspace.QN1[71]*dOld[11];
dNew[6] = + nmpcWorkspace.QN1[72]*dOld[0] + nmpcWorkspace.QN1[73]*dOld[1] + nmpcWorkspace.QN1[74]*dOld[2] + nmpcWorkspace.QN1[75]*dOld[3] + nmpcWorkspace.QN1[76]*dOld[4] + nmpcWorkspace.QN1[77]*dOld[5] + nmpcWorkspace.QN1[78]*dOld[6] + nmpcWorkspace.QN1[79]*dOld[7] + nmpcWorkspace.QN1[80]*dOld[8] + nmpcWorkspace.QN1[81]*dOld[9] + nmpcWorkspace.QN1[82]*dOld[10] + nmpcWorkspace.QN1[83]*dOld[11];
dNew[7] = + nmpcWorkspace.QN1[84]*dOld[0] + nmpcWorkspace.QN1[85]*dOld[1] + nmpcWorkspace.QN1[86]*dOld[2] + nmpcWorkspace.QN1[87]*dOld[3] + nmpcWorkspace.QN1[88]*dOld[4] + nmpcWorkspace.QN1[89]*dOld[5] + nmpcWorkspace.QN1[90]*dOld[6] + nmpcWorkspace.QN1[91]*dOld[7] + nmpcWorkspace.QN1[92]*dOld[8] + nmpcWorkspace.QN1[93]*dOld[9] + nmpcWorkspace.QN1[94]*dOld[10] + nmpcWorkspace.QN1[95]*dOld[11];
dNew[8] = + nmpcWorkspace.QN1[96]*dOld[0] + nmpcWorkspace.QN1[97]*dOld[1] + nmpcWorkspace.QN1[98]*dOld[2] + nmpcWorkspace.QN1[99]*dOld[3] + nmpcWorkspace.QN1[100]*dOld[4] + nmpcWorkspace.QN1[101]*dOld[5] + nmpcWorkspace.QN1[102]*dOld[6] + nmpcWorkspace.QN1[103]*dOld[7] + nmpcWorkspace.QN1[104]*dOld[8] + nmpcWorkspace.QN1[105]*dOld[9] + nmpcWorkspace.QN1[106]*dOld[10] + nmpcWorkspace.QN1[107]*dOld[11];
dNew[9] = + nmpcWorkspace.QN1[108]*dOld[0] + nmpcWorkspace.QN1[109]*dOld[1] + nmpcWorkspace.QN1[110]*dOld[2] + nmpcWorkspace.QN1[111]*dOld[3] + nmpcWorkspace.QN1[112]*dOld[4] + nmpcWorkspace.QN1[113]*dOld[5] + nmpcWorkspace.QN1[114]*dOld[6] + nmpcWorkspace.QN1[115]*dOld[7] + nmpcWorkspace.QN1[116]*dOld[8] + nmpcWorkspace.QN1[117]*dOld[9] + nmpcWorkspace.QN1[118]*dOld[10] + nmpcWorkspace.QN1[119]*dOld[11];
dNew[10] = + nmpcWorkspace.QN1[120]*dOld[0] + nmpcWorkspace.QN1[121]*dOld[1] + nmpcWorkspace.QN1[122]*dOld[2] + nmpcWorkspace.QN1[123]*dOld[3] + nmpcWorkspace.QN1[124]*dOld[4] + nmpcWorkspace.QN1[125]*dOld[5] + nmpcWorkspace.QN1[126]*dOld[6] + nmpcWorkspace.QN1[127]*dOld[7] + nmpcWorkspace.QN1[128]*dOld[8] + nmpcWorkspace.QN1[129]*dOld[9] + nmpcWorkspace.QN1[130]*dOld[10] + nmpcWorkspace.QN1[131]*dOld[11];
dNew[11] = + nmpcWorkspace.QN1[132]*dOld[0] + nmpcWorkspace.QN1[133]*dOld[1] + nmpcWorkspace.QN1[134]*dOld[2] + nmpcWorkspace.QN1[135]*dOld[3] + nmpcWorkspace.QN1[136]*dOld[4] + nmpcWorkspace.QN1[137]*dOld[5] + nmpcWorkspace.QN1[138]*dOld[6] + nmpcWorkspace.QN1[139]*dOld[7] + nmpcWorkspace.QN1[140]*dOld[8] + nmpcWorkspace.QN1[141]*dOld[9] + nmpcWorkspace.QN1[142]*dOld[10] + nmpcWorkspace.QN1[143]*dOld[11];
}

void nmpc_multRDy( real_t* const R2, real_t* const Dy1, real_t* const RDy1 )
{
RDy1[0] = + R2[0]*Dy1[0] + R2[1]*Dy1[1] + R2[2]*Dy1[2] + R2[3]*Dy1[3] + R2[4]*Dy1[4] + R2[5]*Dy1[5] + R2[6]*Dy1[6] + R2[7]*Dy1[7] + R2[8]*Dy1[8] + R2[9]*Dy1[9] + R2[10]*Dy1[10] + R2[11]*Dy1[11] + R2[12]*Dy1[12] + R2[13]*Dy1[13];
RDy1[1] = + R2[14]*Dy1[0] + R2[15]*Dy1[1] + R2[16]*Dy1[2] + R2[17]*Dy1[3] + R2[18]*Dy1[4] + R2[19]*Dy1[5] + R2[20]*Dy1[6] + R2[21]*Dy1[7] + R2[22]*Dy1[8] + R2[23]*Dy1[9] + R2[24]*Dy1[10] + R2[25]*Dy1[11] + R2[26]*Dy1[12] + R2[27]*Dy1[13];
RDy1[2] = + R2[28]*Dy1[0] + R2[29]*Dy1[1] + R2[30]*Dy1[2] + R2[31]*Dy1[3] + R2[32]*Dy1[4] + R2[33]*Dy1[5] + R2[34]*Dy1[6] + R2[35]*Dy1[7] + R2[36]*Dy1[8] + R2[37]*Dy1[9] + R2[38]*Dy1[10] + R2[39]*Dy1[11] + R2[40]*Dy1[12] + R2[41]*Dy1[13];
RDy1[3] = + R2[42]*Dy1[0] + R2[43]*Dy1[1] + R2[44]*Dy1[2] + R2[45]*Dy1[3] + R2[46]*Dy1[4] + R2[47]*Dy1[5] + R2[48]*Dy1[6] + R2[49]*Dy1[7] + R2[50]*Dy1[8] + R2[51]*Dy1[9] + R2[52]*Dy1[10] + R2[53]*Dy1[11] + R2[54]*Dy1[12] + R2[55]*Dy1[13];
}

void nmpc_multQDy( real_t* const Q2, real_t* const Dy1, real_t* const QDy1 )
{
QDy1[0] = + Q2[0]*Dy1[0] + Q2[1]*Dy1[1] + Q2[2]*Dy1[2] + Q2[3]*Dy1[3] + Q2[4]*Dy1[4] + Q2[5]*Dy1[5] + Q2[6]*Dy1[6] + Q2[7]*Dy1[7] + Q2[8]*Dy1[8] + Q2[9]*Dy1[9] + Q2[10]*Dy1[10] + Q2[11]*Dy1[11] + Q2[12]*Dy1[12] + Q2[13]*Dy1[13];
QDy1[1] = + Q2[14]*Dy1[0] + Q2[15]*Dy1[1] + Q2[16]*Dy1[2] + Q2[17]*Dy1[3] + Q2[18]*Dy1[4] + Q2[19]*Dy1[5] + Q2[20]*Dy1[6] + Q2[21]*Dy1[7] + Q2[22]*Dy1[8] + Q2[23]*Dy1[9] + Q2[24]*Dy1[10] + Q2[25]*Dy1[11] + Q2[26]*Dy1[12] + Q2[27]*Dy1[13];
QDy1[2] = + Q2[28]*Dy1[0] + Q2[29]*Dy1[1] + Q2[30]*Dy1[2] + Q2[31]*Dy1[3] + Q2[32]*Dy1[4] + Q2[33]*Dy1[5] + Q2[34]*Dy1[6] + Q2[35]*Dy1[7] + Q2[36]*Dy1[8] + Q2[37]*Dy1[9] + Q2[38]*Dy1[10] + Q2[39]*Dy1[11] + Q2[40]*Dy1[12] + Q2[41]*Dy1[13];
QDy1[3] = + Q2[42]*Dy1[0] + Q2[43]*Dy1[1] + Q2[44]*Dy1[2] + Q2[45]*Dy1[3] + Q2[46]*Dy1[4] + Q2[47]*Dy1[5] + Q2[48]*Dy1[6] + Q2[49]*Dy1[7] + Q2[50]*Dy1[8] + Q2[51]*Dy1[9] + Q2[52]*Dy1[10] + Q2[53]*Dy1[11] + Q2[54]*Dy1[12] + Q2[55]*Dy1[13];
QDy1[4] = + Q2[56]*Dy1[0] + Q2[57]*Dy1[1] + Q2[58]*Dy1[2] + Q2[59]*Dy1[3] + Q2[60]*Dy1[4] + Q2[61]*Dy1[5] + Q2[62]*Dy1[6] + Q2[63]*Dy1[7] + Q2[64]*Dy1[8] + Q2[65]*Dy1[9] + Q2[66]*Dy1[10] + Q2[67]*Dy1[11] + Q2[68]*Dy1[12] + Q2[69]*Dy1[13];
QDy1[5] = + Q2[70]*Dy1[0] + Q2[71]*Dy1[1] + Q2[72]*Dy1[2] + Q2[73]*Dy1[3] + Q2[74]*Dy1[4] + Q2[75]*Dy1[5] + Q2[76]*Dy1[6] + Q2[77]*Dy1[7] + Q2[78]*Dy1[8] + Q2[79]*Dy1[9] + Q2[80]*Dy1[10] + Q2[81]*Dy1[11] + Q2[82]*Dy1[12] + Q2[83]*Dy1[13];
QDy1[6] = + Q2[84]*Dy1[0] + Q2[85]*Dy1[1] + Q2[86]*Dy1[2] + Q2[87]*Dy1[3] + Q2[88]*Dy1[4] + Q2[89]*Dy1[5] + Q2[90]*Dy1[6] + Q2[91]*Dy1[7] + Q2[92]*Dy1[8] + Q2[93]*Dy1[9] + Q2[94]*Dy1[10] + Q2[95]*Dy1[11] + Q2[96]*Dy1[12] + Q2[97]*Dy1[13];
QDy1[7] = + Q2[98]*Dy1[0] + Q2[99]*Dy1[1] + Q2[100]*Dy1[2] + Q2[101]*Dy1[3] + Q2[102]*Dy1[4] + Q2[103]*Dy1[5] + Q2[104]*Dy1[6] + Q2[105]*Dy1[7] + Q2[106]*Dy1[8] + Q2[107]*Dy1[9] + Q2[108]*Dy1[10] + Q2[109]*Dy1[11] + Q2[110]*Dy1[12] + Q2[111]*Dy1[13];
QDy1[8] = + Q2[112]*Dy1[0] + Q2[113]*Dy1[1] + Q2[114]*Dy1[2] + Q2[115]*Dy1[3] + Q2[116]*Dy1[4] + Q2[117]*Dy1[5] + Q2[118]*Dy1[6] + Q2[119]*Dy1[7] + Q2[120]*Dy1[8] + Q2[121]*Dy1[9] + Q2[122]*Dy1[10] + Q2[123]*Dy1[11] + Q2[124]*Dy1[12] + Q2[125]*Dy1[13];
QDy1[9] = + Q2[126]*Dy1[0] + Q2[127]*Dy1[1] + Q2[128]*Dy1[2] + Q2[129]*Dy1[3] + Q2[130]*Dy1[4] + Q2[131]*Dy1[5] + Q2[132]*Dy1[6] + Q2[133]*Dy1[7] + Q2[134]*Dy1[8] + Q2[135]*Dy1[9] + Q2[136]*Dy1[10] + Q2[137]*Dy1[11] + Q2[138]*Dy1[12] + Q2[139]*Dy1[13];
QDy1[10] = + Q2[140]*Dy1[0] + Q2[141]*Dy1[1] + Q2[142]*Dy1[2] + Q2[143]*Dy1[3] + Q2[144]*Dy1[4] + Q2[145]*Dy1[5] + Q2[146]*Dy1[6] + Q2[147]*Dy1[7] + Q2[148]*Dy1[8] + Q2[149]*Dy1[9] + Q2[150]*Dy1[10] + Q2[151]*Dy1[11] + Q2[152]*Dy1[12] + Q2[153]*Dy1[13];
QDy1[11] = + Q2[154]*Dy1[0] + Q2[155]*Dy1[1] + Q2[156]*Dy1[2] + Q2[157]*Dy1[3] + Q2[158]*Dy1[4] + Q2[159]*Dy1[5] + Q2[160]*Dy1[6] + Q2[161]*Dy1[7] + Q2[162]*Dy1[8] + Q2[163]*Dy1[9] + Q2[164]*Dy1[10] + Q2[165]*Dy1[11] + Q2[166]*Dy1[12] + Q2[167]*Dy1[13];
}

void nmpc_multEQDy( real_t* const E1, real_t* const QDy1, real_t* const U1 )
{
U1[0] += + E1[0]*QDy1[0] + E1[4]*QDy1[1] + E1[8]*QDy1[2] + E1[12]*QDy1[3] + E1[16]*QDy1[4] + E1[20]*QDy1[5] + E1[24]*QDy1[6] + E1[28]*QDy1[7] + E1[32]*QDy1[8] + E1[36]*QDy1[9] + E1[40]*QDy1[10] + E1[44]*QDy1[11];
U1[1] += + E1[1]*QDy1[0] + E1[5]*QDy1[1] + E1[9]*QDy1[2] + E1[13]*QDy1[3] + E1[17]*QDy1[4] + E1[21]*QDy1[5] + E1[25]*QDy1[6] + E1[29]*QDy1[7] + E1[33]*QDy1[8] + E1[37]*QDy1[9] + E1[41]*QDy1[10] + E1[45]*QDy1[11];
U1[2] += + E1[2]*QDy1[0] + E1[6]*QDy1[1] + E1[10]*QDy1[2] + E1[14]*QDy1[3] + E1[18]*QDy1[4] + E1[22]*QDy1[5] + E1[26]*QDy1[6] + E1[30]*QDy1[7] + E1[34]*QDy1[8] + E1[38]*QDy1[9] + E1[42]*QDy1[10] + E1[46]*QDy1[11];
U1[3] += + E1[3]*QDy1[0] + E1[7]*QDy1[1] + E1[11]*QDy1[2] + E1[15]*QDy1[3] + E1[19]*QDy1[4] + E1[23]*QDy1[5] + E1[27]*QDy1[6] + E1[31]*QDy1[7] + E1[35]*QDy1[8] + E1[39]*QDy1[9] + E1[43]*QDy1[10] + E1[47]*QDy1[11];
}

void nmpc_multQETGx( real_t* const E1, real_t* const Gx1, real_t* const H101 )
{
H101[0] += + E1[0]*Gx1[0] + E1[4]*Gx1[12] + E1[8]*Gx1[24] + E1[12]*Gx1[36] + E1[16]*Gx1[48] + E1[20]*Gx1[60] + E1[24]*Gx1[72] + E1[28]*Gx1[84] + E1[32]*Gx1[96] + E1[36]*Gx1[108] + E1[40]*Gx1[120] + E1[44]*Gx1[132];
H101[1] += + E1[0]*Gx1[1] + E1[4]*Gx1[13] + E1[8]*Gx1[25] + E1[12]*Gx1[37] + E1[16]*Gx1[49] + E1[20]*Gx1[61] + E1[24]*Gx1[73] + E1[28]*Gx1[85] + E1[32]*Gx1[97] + E1[36]*Gx1[109] + E1[40]*Gx1[121] + E1[44]*Gx1[133];
H101[2] += + E1[0]*Gx1[2] + E1[4]*Gx1[14] + E1[8]*Gx1[26] + E1[12]*Gx1[38] + E1[16]*Gx1[50] + E1[20]*Gx1[62] + E1[24]*Gx1[74] + E1[28]*Gx1[86] + E1[32]*Gx1[98] + E1[36]*Gx1[110] + E1[40]*Gx1[122] + E1[44]*Gx1[134];
H101[3] += + E1[0]*Gx1[3] + E1[4]*Gx1[15] + E1[8]*Gx1[27] + E1[12]*Gx1[39] + E1[16]*Gx1[51] + E1[20]*Gx1[63] + E1[24]*Gx1[75] + E1[28]*Gx1[87] + E1[32]*Gx1[99] + E1[36]*Gx1[111] + E1[40]*Gx1[123] + E1[44]*Gx1[135];
H101[4] += + E1[0]*Gx1[4] + E1[4]*Gx1[16] + E1[8]*Gx1[28] + E1[12]*Gx1[40] + E1[16]*Gx1[52] + E1[20]*Gx1[64] + E1[24]*Gx1[76] + E1[28]*Gx1[88] + E1[32]*Gx1[100] + E1[36]*Gx1[112] + E1[40]*Gx1[124] + E1[44]*Gx1[136];
H101[5] += + E1[0]*Gx1[5] + E1[4]*Gx1[17] + E1[8]*Gx1[29] + E1[12]*Gx1[41] + E1[16]*Gx1[53] + E1[20]*Gx1[65] + E1[24]*Gx1[77] + E1[28]*Gx1[89] + E1[32]*Gx1[101] + E1[36]*Gx1[113] + E1[40]*Gx1[125] + E1[44]*Gx1[137];
H101[6] += + E1[0]*Gx1[6] + E1[4]*Gx1[18] + E1[8]*Gx1[30] + E1[12]*Gx1[42] + E1[16]*Gx1[54] + E1[20]*Gx1[66] + E1[24]*Gx1[78] + E1[28]*Gx1[90] + E1[32]*Gx1[102] + E1[36]*Gx1[114] + E1[40]*Gx1[126] + E1[44]*Gx1[138];
H101[7] += + E1[0]*Gx1[7] + E1[4]*Gx1[19] + E1[8]*Gx1[31] + E1[12]*Gx1[43] + E1[16]*Gx1[55] + E1[20]*Gx1[67] + E1[24]*Gx1[79] + E1[28]*Gx1[91] + E1[32]*Gx1[103] + E1[36]*Gx1[115] + E1[40]*Gx1[127] + E1[44]*Gx1[139];
H101[8] += + E1[0]*Gx1[8] + E1[4]*Gx1[20] + E1[8]*Gx1[32] + E1[12]*Gx1[44] + E1[16]*Gx1[56] + E1[20]*Gx1[68] + E1[24]*Gx1[80] + E1[28]*Gx1[92] + E1[32]*Gx1[104] + E1[36]*Gx1[116] + E1[40]*Gx1[128] + E1[44]*Gx1[140];
H101[9] += + E1[0]*Gx1[9] + E1[4]*Gx1[21] + E1[8]*Gx1[33] + E1[12]*Gx1[45] + E1[16]*Gx1[57] + E1[20]*Gx1[69] + E1[24]*Gx1[81] + E1[28]*Gx1[93] + E1[32]*Gx1[105] + E1[36]*Gx1[117] + E1[40]*Gx1[129] + E1[44]*Gx1[141];
H101[10] += + E1[0]*Gx1[10] + E1[4]*Gx1[22] + E1[8]*Gx1[34] + E1[12]*Gx1[46] + E1[16]*Gx1[58] + E1[20]*Gx1[70] + E1[24]*Gx1[82] + E1[28]*Gx1[94] + E1[32]*Gx1[106] + E1[36]*Gx1[118] + E1[40]*Gx1[130] + E1[44]*Gx1[142];
H101[11] += + E1[0]*Gx1[11] + E1[4]*Gx1[23] + E1[8]*Gx1[35] + E1[12]*Gx1[47] + E1[16]*Gx1[59] + E1[20]*Gx1[71] + E1[24]*Gx1[83] + E1[28]*Gx1[95] + E1[32]*Gx1[107] + E1[36]*Gx1[119] + E1[40]*Gx1[131] + E1[44]*Gx1[143];
H101[12] += + E1[1]*Gx1[0] + E1[5]*Gx1[12] + E1[9]*Gx1[24] + E1[13]*Gx1[36] + E1[17]*Gx1[48] + E1[21]*Gx1[60] + E1[25]*Gx1[72] + E1[29]*Gx1[84] + E1[33]*Gx1[96] + E1[37]*Gx1[108] + E1[41]*Gx1[120] + E1[45]*Gx1[132];
H101[13] += + E1[1]*Gx1[1] + E1[5]*Gx1[13] + E1[9]*Gx1[25] + E1[13]*Gx1[37] + E1[17]*Gx1[49] + E1[21]*Gx1[61] + E1[25]*Gx1[73] + E1[29]*Gx1[85] + E1[33]*Gx1[97] + E1[37]*Gx1[109] + E1[41]*Gx1[121] + E1[45]*Gx1[133];
H101[14] += + E1[1]*Gx1[2] + E1[5]*Gx1[14] + E1[9]*Gx1[26] + E1[13]*Gx1[38] + E1[17]*Gx1[50] + E1[21]*Gx1[62] + E1[25]*Gx1[74] + E1[29]*Gx1[86] + E1[33]*Gx1[98] + E1[37]*Gx1[110] + E1[41]*Gx1[122] + E1[45]*Gx1[134];
H101[15] += + E1[1]*Gx1[3] + E1[5]*Gx1[15] + E1[9]*Gx1[27] + E1[13]*Gx1[39] + E1[17]*Gx1[51] + E1[21]*Gx1[63] + E1[25]*Gx1[75] + E1[29]*Gx1[87] + E1[33]*Gx1[99] + E1[37]*Gx1[111] + E1[41]*Gx1[123] + E1[45]*Gx1[135];
H101[16] += + E1[1]*Gx1[4] + E1[5]*Gx1[16] + E1[9]*Gx1[28] + E1[13]*Gx1[40] + E1[17]*Gx1[52] + E1[21]*Gx1[64] + E1[25]*Gx1[76] + E1[29]*Gx1[88] + E1[33]*Gx1[100] + E1[37]*Gx1[112] + E1[41]*Gx1[124] + E1[45]*Gx1[136];
H101[17] += + E1[1]*Gx1[5] + E1[5]*Gx1[17] + E1[9]*Gx1[29] + E1[13]*Gx1[41] + E1[17]*Gx1[53] + E1[21]*Gx1[65] + E1[25]*Gx1[77] + E1[29]*Gx1[89] + E1[33]*Gx1[101] + E1[37]*Gx1[113] + E1[41]*Gx1[125] + E1[45]*Gx1[137];
H101[18] += + E1[1]*Gx1[6] + E1[5]*Gx1[18] + E1[9]*Gx1[30] + E1[13]*Gx1[42] + E1[17]*Gx1[54] + E1[21]*Gx1[66] + E1[25]*Gx1[78] + E1[29]*Gx1[90] + E1[33]*Gx1[102] + E1[37]*Gx1[114] + E1[41]*Gx1[126] + E1[45]*Gx1[138];
H101[19] += + E1[1]*Gx1[7] + E1[5]*Gx1[19] + E1[9]*Gx1[31] + E1[13]*Gx1[43] + E1[17]*Gx1[55] + E1[21]*Gx1[67] + E1[25]*Gx1[79] + E1[29]*Gx1[91] + E1[33]*Gx1[103] + E1[37]*Gx1[115] + E1[41]*Gx1[127] + E1[45]*Gx1[139];
H101[20] += + E1[1]*Gx1[8] + E1[5]*Gx1[20] + E1[9]*Gx1[32] + E1[13]*Gx1[44] + E1[17]*Gx1[56] + E1[21]*Gx1[68] + E1[25]*Gx1[80] + E1[29]*Gx1[92] + E1[33]*Gx1[104] + E1[37]*Gx1[116] + E1[41]*Gx1[128] + E1[45]*Gx1[140];
H101[21] += + E1[1]*Gx1[9] + E1[5]*Gx1[21] + E1[9]*Gx1[33] + E1[13]*Gx1[45] + E1[17]*Gx1[57] + E1[21]*Gx1[69] + E1[25]*Gx1[81] + E1[29]*Gx1[93] + E1[33]*Gx1[105] + E1[37]*Gx1[117] + E1[41]*Gx1[129] + E1[45]*Gx1[141];
H101[22] += + E1[1]*Gx1[10] + E1[5]*Gx1[22] + E1[9]*Gx1[34] + E1[13]*Gx1[46] + E1[17]*Gx1[58] + E1[21]*Gx1[70] + E1[25]*Gx1[82] + E1[29]*Gx1[94] + E1[33]*Gx1[106] + E1[37]*Gx1[118] + E1[41]*Gx1[130] + E1[45]*Gx1[142];
H101[23] += + E1[1]*Gx1[11] + E1[5]*Gx1[23] + E1[9]*Gx1[35] + E1[13]*Gx1[47] + E1[17]*Gx1[59] + E1[21]*Gx1[71] + E1[25]*Gx1[83] + E1[29]*Gx1[95] + E1[33]*Gx1[107] + E1[37]*Gx1[119] + E1[41]*Gx1[131] + E1[45]*Gx1[143];
H101[24] += + E1[2]*Gx1[0] + E1[6]*Gx1[12] + E1[10]*Gx1[24] + E1[14]*Gx1[36] + E1[18]*Gx1[48] + E1[22]*Gx1[60] + E1[26]*Gx1[72] + E1[30]*Gx1[84] + E1[34]*Gx1[96] + E1[38]*Gx1[108] + E1[42]*Gx1[120] + E1[46]*Gx1[132];
H101[25] += + E1[2]*Gx1[1] + E1[6]*Gx1[13] + E1[10]*Gx1[25] + E1[14]*Gx1[37] + E1[18]*Gx1[49] + E1[22]*Gx1[61] + E1[26]*Gx1[73] + E1[30]*Gx1[85] + E1[34]*Gx1[97] + E1[38]*Gx1[109] + E1[42]*Gx1[121] + E1[46]*Gx1[133];
H101[26] += + E1[2]*Gx1[2] + E1[6]*Gx1[14] + E1[10]*Gx1[26] + E1[14]*Gx1[38] + E1[18]*Gx1[50] + E1[22]*Gx1[62] + E1[26]*Gx1[74] + E1[30]*Gx1[86] + E1[34]*Gx1[98] + E1[38]*Gx1[110] + E1[42]*Gx1[122] + E1[46]*Gx1[134];
H101[27] += + E1[2]*Gx1[3] + E1[6]*Gx1[15] + E1[10]*Gx1[27] + E1[14]*Gx1[39] + E1[18]*Gx1[51] + E1[22]*Gx1[63] + E1[26]*Gx1[75] + E1[30]*Gx1[87] + E1[34]*Gx1[99] + E1[38]*Gx1[111] + E1[42]*Gx1[123] + E1[46]*Gx1[135];
H101[28] += + E1[2]*Gx1[4] + E1[6]*Gx1[16] + E1[10]*Gx1[28] + E1[14]*Gx1[40] + E1[18]*Gx1[52] + E1[22]*Gx1[64] + E1[26]*Gx1[76] + E1[30]*Gx1[88] + E1[34]*Gx1[100] + E1[38]*Gx1[112] + E1[42]*Gx1[124] + E1[46]*Gx1[136];
H101[29] += + E1[2]*Gx1[5] + E1[6]*Gx1[17] + E1[10]*Gx1[29] + E1[14]*Gx1[41] + E1[18]*Gx1[53] + E1[22]*Gx1[65] + E1[26]*Gx1[77] + E1[30]*Gx1[89] + E1[34]*Gx1[101] + E1[38]*Gx1[113] + E1[42]*Gx1[125] + E1[46]*Gx1[137];
H101[30] += + E1[2]*Gx1[6] + E1[6]*Gx1[18] + E1[10]*Gx1[30] + E1[14]*Gx1[42] + E1[18]*Gx1[54] + E1[22]*Gx1[66] + E1[26]*Gx1[78] + E1[30]*Gx1[90] + E1[34]*Gx1[102] + E1[38]*Gx1[114] + E1[42]*Gx1[126] + E1[46]*Gx1[138];
H101[31] += + E1[2]*Gx1[7] + E1[6]*Gx1[19] + E1[10]*Gx1[31] + E1[14]*Gx1[43] + E1[18]*Gx1[55] + E1[22]*Gx1[67] + E1[26]*Gx1[79] + E1[30]*Gx1[91] + E1[34]*Gx1[103] + E1[38]*Gx1[115] + E1[42]*Gx1[127] + E1[46]*Gx1[139];
H101[32] += + E1[2]*Gx1[8] + E1[6]*Gx1[20] + E1[10]*Gx1[32] + E1[14]*Gx1[44] + E1[18]*Gx1[56] + E1[22]*Gx1[68] + E1[26]*Gx1[80] + E1[30]*Gx1[92] + E1[34]*Gx1[104] + E1[38]*Gx1[116] + E1[42]*Gx1[128] + E1[46]*Gx1[140];
H101[33] += + E1[2]*Gx1[9] + E1[6]*Gx1[21] + E1[10]*Gx1[33] + E1[14]*Gx1[45] + E1[18]*Gx1[57] + E1[22]*Gx1[69] + E1[26]*Gx1[81] + E1[30]*Gx1[93] + E1[34]*Gx1[105] + E1[38]*Gx1[117] + E1[42]*Gx1[129] + E1[46]*Gx1[141];
H101[34] += + E1[2]*Gx1[10] + E1[6]*Gx1[22] + E1[10]*Gx1[34] + E1[14]*Gx1[46] + E1[18]*Gx1[58] + E1[22]*Gx1[70] + E1[26]*Gx1[82] + E1[30]*Gx1[94] + E1[34]*Gx1[106] + E1[38]*Gx1[118] + E1[42]*Gx1[130] + E1[46]*Gx1[142];
H101[35] += + E1[2]*Gx1[11] + E1[6]*Gx1[23] + E1[10]*Gx1[35] + E1[14]*Gx1[47] + E1[18]*Gx1[59] + E1[22]*Gx1[71] + E1[26]*Gx1[83] + E1[30]*Gx1[95] + E1[34]*Gx1[107] + E1[38]*Gx1[119] + E1[42]*Gx1[131] + E1[46]*Gx1[143];
H101[36] += + E1[3]*Gx1[0] + E1[7]*Gx1[12] + E1[11]*Gx1[24] + E1[15]*Gx1[36] + E1[19]*Gx1[48] + E1[23]*Gx1[60] + E1[27]*Gx1[72] + E1[31]*Gx1[84] + E1[35]*Gx1[96] + E1[39]*Gx1[108] + E1[43]*Gx1[120] + E1[47]*Gx1[132];
H101[37] += + E1[3]*Gx1[1] + E1[7]*Gx1[13] + E1[11]*Gx1[25] + E1[15]*Gx1[37] + E1[19]*Gx1[49] + E1[23]*Gx1[61] + E1[27]*Gx1[73] + E1[31]*Gx1[85] + E1[35]*Gx1[97] + E1[39]*Gx1[109] + E1[43]*Gx1[121] + E1[47]*Gx1[133];
H101[38] += + E1[3]*Gx1[2] + E1[7]*Gx1[14] + E1[11]*Gx1[26] + E1[15]*Gx1[38] + E1[19]*Gx1[50] + E1[23]*Gx1[62] + E1[27]*Gx1[74] + E1[31]*Gx1[86] + E1[35]*Gx1[98] + E1[39]*Gx1[110] + E1[43]*Gx1[122] + E1[47]*Gx1[134];
H101[39] += + E1[3]*Gx1[3] + E1[7]*Gx1[15] + E1[11]*Gx1[27] + E1[15]*Gx1[39] + E1[19]*Gx1[51] + E1[23]*Gx1[63] + E1[27]*Gx1[75] + E1[31]*Gx1[87] + E1[35]*Gx1[99] + E1[39]*Gx1[111] + E1[43]*Gx1[123] + E1[47]*Gx1[135];
H101[40] += + E1[3]*Gx1[4] + E1[7]*Gx1[16] + E1[11]*Gx1[28] + E1[15]*Gx1[40] + E1[19]*Gx1[52] + E1[23]*Gx1[64] + E1[27]*Gx1[76] + E1[31]*Gx1[88] + E1[35]*Gx1[100] + E1[39]*Gx1[112] + E1[43]*Gx1[124] + E1[47]*Gx1[136];
H101[41] += + E1[3]*Gx1[5] + E1[7]*Gx1[17] + E1[11]*Gx1[29] + E1[15]*Gx1[41] + E1[19]*Gx1[53] + E1[23]*Gx1[65] + E1[27]*Gx1[77] + E1[31]*Gx1[89] + E1[35]*Gx1[101] + E1[39]*Gx1[113] + E1[43]*Gx1[125] + E1[47]*Gx1[137];
H101[42] += + E1[3]*Gx1[6] + E1[7]*Gx1[18] + E1[11]*Gx1[30] + E1[15]*Gx1[42] + E1[19]*Gx1[54] + E1[23]*Gx1[66] + E1[27]*Gx1[78] + E1[31]*Gx1[90] + E1[35]*Gx1[102] + E1[39]*Gx1[114] + E1[43]*Gx1[126] + E1[47]*Gx1[138];
H101[43] += + E1[3]*Gx1[7] + E1[7]*Gx1[19] + E1[11]*Gx1[31] + E1[15]*Gx1[43] + E1[19]*Gx1[55] + E1[23]*Gx1[67] + E1[27]*Gx1[79] + E1[31]*Gx1[91] + E1[35]*Gx1[103] + E1[39]*Gx1[115] + E1[43]*Gx1[127] + E1[47]*Gx1[139];
H101[44] += + E1[3]*Gx1[8] + E1[7]*Gx1[20] + E1[11]*Gx1[32] + E1[15]*Gx1[44] + E1[19]*Gx1[56] + E1[23]*Gx1[68] + E1[27]*Gx1[80] + E1[31]*Gx1[92] + E1[35]*Gx1[104] + E1[39]*Gx1[116] + E1[43]*Gx1[128] + E1[47]*Gx1[140];
H101[45] += + E1[3]*Gx1[9] + E1[7]*Gx1[21] + E1[11]*Gx1[33] + E1[15]*Gx1[45] + E1[19]*Gx1[57] + E1[23]*Gx1[69] + E1[27]*Gx1[81] + E1[31]*Gx1[93] + E1[35]*Gx1[105] + E1[39]*Gx1[117] + E1[43]*Gx1[129] + E1[47]*Gx1[141];
H101[46] += + E1[3]*Gx1[10] + E1[7]*Gx1[22] + E1[11]*Gx1[34] + E1[15]*Gx1[46] + E1[19]*Gx1[58] + E1[23]*Gx1[70] + E1[27]*Gx1[82] + E1[31]*Gx1[94] + E1[35]*Gx1[106] + E1[39]*Gx1[118] + E1[43]*Gx1[130] + E1[47]*Gx1[142];
H101[47] += + E1[3]*Gx1[11] + E1[7]*Gx1[23] + E1[11]*Gx1[35] + E1[15]*Gx1[47] + E1[19]*Gx1[59] + E1[23]*Gx1[71] + E1[27]*Gx1[83] + E1[31]*Gx1[95] + E1[35]*Gx1[107] + E1[39]*Gx1[119] + E1[43]*Gx1[131] + E1[47]*Gx1[143];
}

void nmpc_zeroBlockH10( real_t* const H101 )
{
{ int lCopy; for (lCopy = 0; lCopy < 48; lCopy++) H101[ lCopy ] = 0; }
}

void nmpc_multEDu( real_t* const E1, real_t* const U1, real_t* const dNew )
{
dNew[0] += + E1[0]*U1[0] + E1[1]*U1[1] + E1[2]*U1[2] + E1[3]*U1[3];
dNew[1] += + E1[4]*U1[0] + E1[5]*U1[1] + E1[6]*U1[2] + E1[7]*U1[3];
dNew[2] += + E1[8]*U1[0] + E1[9]*U1[1] + E1[10]*U1[2] + E1[11]*U1[3];
dNew[3] += + E1[12]*U1[0] + E1[13]*U1[1] + E1[14]*U1[2] + E1[15]*U1[3];
dNew[4] += + E1[16]*U1[0] + E1[17]*U1[1] + E1[18]*U1[2] + E1[19]*U1[3];
dNew[5] += + E1[20]*U1[0] + E1[21]*U1[1] + E1[22]*U1[2] + E1[23]*U1[3];
dNew[6] += + E1[24]*U1[0] + E1[25]*U1[1] + E1[26]*U1[2] + E1[27]*U1[3];
dNew[7] += + E1[28]*U1[0] + E1[29]*U1[1] + E1[30]*U1[2] + E1[31]*U1[3];
dNew[8] += + E1[32]*U1[0] + E1[33]*U1[1] + E1[34]*U1[2] + E1[35]*U1[3];
dNew[9] += + E1[36]*U1[0] + E1[37]*U1[1] + E1[38]*U1[2] + E1[39]*U1[3];
dNew[10] += + E1[40]*U1[0] + E1[41]*U1[1] + E1[42]*U1[2] + E1[43]*U1[3];
dNew[11] += + E1[44]*U1[0] + E1[45]*U1[1] + E1[46]*U1[2] + E1[47]*U1[3];
}

void nmpc_zeroBlockH00(  )
{
nmpcWorkspace.H[0] = 0.0000000000000000e+00;
nmpcWorkspace.H[1] = 0.0000000000000000e+00;
nmpcWorkspace.H[2] = 0.0000000000000000e+00;
nmpcWorkspace.H[3] = 0.0000000000000000e+00;
nmpcWorkspace.H[4] = 0.0000000000000000e+00;
nmpcWorkspace.H[5] = 0.0000000000000000e+00;
nmpcWorkspace.H[6] = 0.0000000000000000e+00;
nmpcWorkspace.H[7] = 0.0000000000000000e+00;
nmpcWorkspace.H[8] = 0.0000000000000000e+00;
nmpcWorkspace.H[9] = 0.0000000000000000e+00;
nmpcWorkspace.H[10] = 0.0000000000000000e+00;
nmpcWorkspace.H[11] = 0.0000000000000000e+00;
nmpcWorkspace.H[132] = 0.0000000000000000e+00;
nmpcWorkspace.H[133] = 0.0000000000000000e+00;
nmpcWorkspace.H[134] = 0.0000000000000000e+00;
nmpcWorkspace.H[135] = 0.0000000000000000e+00;
nmpcWorkspace.H[136] = 0.0000000000000000e+00;
nmpcWorkspace.H[137] = 0.0000000000000000e+00;
nmpcWorkspace.H[138] = 0.0000000000000000e+00;
nmpcWorkspace.H[139] = 0.0000000000000000e+00;
nmpcWorkspace.H[140] = 0.0000000000000000e+00;
nmpcWorkspace.H[141] = 0.0000000000000000e+00;
nmpcWorkspace.H[142] = 0.0000000000000000e+00;
nmpcWorkspace.H[143] = 0.0000000000000000e+00;
nmpcWorkspace.H[264] = 0.0000000000000000e+00;
nmpcWorkspace.H[265] = 0.0000000000000000e+00;
nmpcWorkspace.H[266] = 0.0000000000000000e+00;
nmpcWorkspace.H[267] = 0.0000000000000000e+00;
nmpcWorkspace.H[268] = 0.0000000000000000e+00;
nmpcWorkspace.H[269] = 0.0000000000000000e+00;
nmpcWorkspace.H[270] = 0.0000000000000000e+00;
nmpcWorkspace.H[271] = 0.0000000000000000e+00;
nmpcWorkspace.H[272] = 0.0000000000000000e+00;
nmpcWorkspace.H[273] = 0.0000000000000000e+00;
nmpcWorkspace.H[274] = 0.0000000000000000e+00;
nmpcWorkspace.H[275] = 0.0000000000000000e+00;
nmpcWorkspace.H[396] = 0.0000000000000000e+00;
nmpcWorkspace.H[397] = 0.0000000000000000e+00;
nmpcWorkspace.H[398] = 0.0000000000000000e+00;
nmpcWorkspace.H[399] = 0.0000000000000000e+00;
nmpcWorkspace.H[400] = 0.0000000000000000e+00;
nmpcWorkspace.H[401] = 0.0000000000000000e+00;
nmpcWorkspace.H[402] = 0.0000000000000000e+00;
nmpcWorkspace.H[403] = 0.0000000000000000e+00;
nmpcWorkspace.H[404] = 0.0000000000000000e+00;
nmpcWorkspace.H[405] = 0.0000000000000000e+00;
nmpcWorkspace.H[406] = 0.0000000000000000e+00;
nmpcWorkspace.H[407] = 0.0000000000000000e+00;
nmpcWorkspace.H[528] = 0.0000000000000000e+00;
nmpcWorkspace.H[529] = 0.0000000000000000e+00;
nmpcWorkspace.H[530] = 0.0000000000000000e+00;
nmpcWorkspace.H[531] = 0.0000000000000000e+00;
nmpcWorkspace.H[532] = 0.0000000000000000e+00;
nmpcWorkspace.H[533] = 0.0000000000000000e+00;
nmpcWorkspace.H[534] = 0.0000000000000000e+00;
nmpcWorkspace.H[535] = 0.0000000000000000e+00;
nmpcWorkspace.H[536] = 0.0000000000000000e+00;
nmpcWorkspace.H[537] = 0.0000000000000000e+00;
nmpcWorkspace.H[538] = 0.0000000000000000e+00;
nmpcWorkspace.H[539] = 0.0000000000000000e+00;
nmpcWorkspace.H[660] = 0.0000000000000000e+00;
nmpcWorkspace.H[661] = 0.0000000000000000e+00;
nmpcWorkspace.H[662] = 0.0000000000000000e+00;
nmpcWorkspace.H[663] = 0.0000000000000000e+00;
nmpcWorkspace.H[664] = 0.0000000000000000e+00;
nmpcWorkspace.H[665] = 0.0000000000000000e+00;
nmpcWorkspace.H[666] = 0.0000000000000000e+00;
nmpcWorkspace.H[667] = 0.0000000000000000e+00;
nmpcWorkspace.H[668] = 0.0000000000000000e+00;
nmpcWorkspace.H[669] = 0.0000000000000000e+00;
nmpcWorkspace.H[670] = 0.0000000000000000e+00;
nmpcWorkspace.H[671] = 0.0000000000000000e+00;
nmpcWorkspace.H[792] = 0.0000000000000000e+00;
nmpcWorkspace.H[793] = 0.0000000000000000e+00;
nmpcWorkspace.H[794] = 0.0000000000000000e+00;
nmpcWorkspace.H[795] = 0.0000000000000000e+00;
nmpcWorkspace.H[796] = 0.0000000000000000e+00;
nmpcWorkspace.H[797] = 0.0000000000000000e+00;
nmpcWorkspace.H[798] = 0.0000000000000000e+00;
nmpcWorkspace.H[799] = 0.0000000000000000e+00;
nmpcWorkspace.H[800] = 0.0000000000000000e+00;
nmpcWorkspace.H[801] = 0.0000000000000000e+00;
nmpcWorkspace.H[802] = 0.0000000000000000e+00;
nmpcWorkspace.H[803] = 0.0000000000000000e+00;
nmpcWorkspace.H[924] = 0.0000000000000000e+00;
nmpcWorkspace.H[925] = 0.0000000000000000e+00;
nmpcWorkspace.H[926] = 0.0000000000000000e+00;
nmpcWorkspace.H[927] = 0.0000000000000000e+00;
nmpcWorkspace.H[928] = 0.0000000000000000e+00;
nmpcWorkspace.H[929] = 0.0000000000000000e+00;
nmpcWorkspace.H[930] = 0.0000000000000000e+00;
nmpcWorkspace.H[931] = 0.0000000000000000e+00;
nmpcWorkspace.H[932] = 0.0000000000000000e+00;
nmpcWorkspace.H[933] = 0.0000000000000000e+00;
nmpcWorkspace.H[934] = 0.0000000000000000e+00;
nmpcWorkspace.H[935] = 0.0000000000000000e+00;
nmpcWorkspace.H[1056] = 0.0000000000000000e+00;
nmpcWorkspace.H[1057] = 0.0000000000000000e+00;
nmpcWorkspace.H[1058] = 0.0000000000000000e+00;
nmpcWorkspace.H[1059] = 0.0000000000000000e+00;
nmpcWorkspace.H[1060] = 0.0000000000000000e+00;
nmpcWorkspace.H[1061] = 0.0000000000000000e+00;
nmpcWorkspace.H[1062] = 0.0000000000000000e+00;
nmpcWorkspace.H[1063] = 0.0000000000000000e+00;
nmpcWorkspace.H[1064] = 0.0000000000000000e+00;
nmpcWorkspace.H[1065] = 0.0000000000000000e+00;
nmpcWorkspace.H[1066] = 0.0000000000000000e+00;
nmpcWorkspace.H[1067] = 0.0000000000000000e+00;
nmpcWorkspace.H[1188] = 0.0000000000000000e+00;
nmpcWorkspace.H[1189] = 0.0000000000000000e+00;
nmpcWorkspace.H[1190] = 0.0000000000000000e+00;
nmpcWorkspace.H[1191] = 0.0000000000000000e+00;
nmpcWorkspace.H[1192] = 0.0000000000000000e+00;
nmpcWorkspace.H[1193] = 0.0000000000000000e+00;
nmpcWorkspace.H[1194] = 0.0000000000000000e+00;
nmpcWorkspace.H[1195] = 0.0000000000000000e+00;
nmpcWorkspace.H[1196] = 0.0000000000000000e+00;
nmpcWorkspace.H[1197] = 0.0000000000000000e+00;
nmpcWorkspace.H[1198] = 0.0000000000000000e+00;
nmpcWorkspace.H[1199] = 0.0000000000000000e+00;
nmpcWorkspace.H[1320] = 0.0000000000000000e+00;
nmpcWorkspace.H[1321] = 0.0000000000000000e+00;
nmpcWorkspace.H[1322] = 0.0000000000000000e+00;
nmpcWorkspace.H[1323] = 0.0000000000000000e+00;
nmpcWorkspace.H[1324] = 0.0000000000000000e+00;
nmpcWorkspace.H[1325] = 0.0000000000000000e+00;
nmpcWorkspace.H[1326] = 0.0000000000000000e+00;
nmpcWorkspace.H[1327] = 0.0000000000000000e+00;
nmpcWorkspace.H[1328] = 0.0000000000000000e+00;
nmpcWorkspace.H[1329] = 0.0000000000000000e+00;
nmpcWorkspace.H[1330] = 0.0000000000000000e+00;
nmpcWorkspace.H[1331] = 0.0000000000000000e+00;
nmpcWorkspace.H[1452] = 0.0000000000000000e+00;
nmpcWorkspace.H[1453] = 0.0000000000000000e+00;
nmpcWorkspace.H[1454] = 0.0000000000000000e+00;
nmpcWorkspace.H[1455] = 0.0000000000000000e+00;
nmpcWorkspace.H[1456] = 0.0000000000000000e+00;
nmpcWorkspace.H[1457] = 0.0000000000000000e+00;
nmpcWorkspace.H[1458] = 0.0000000000000000e+00;
nmpcWorkspace.H[1459] = 0.0000000000000000e+00;
nmpcWorkspace.H[1460] = 0.0000000000000000e+00;
nmpcWorkspace.H[1461] = 0.0000000000000000e+00;
nmpcWorkspace.H[1462] = 0.0000000000000000e+00;
nmpcWorkspace.H[1463] = 0.0000000000000000e+00;
}

void nmpc_multCTQC( real_t* const Gx1, real_t* const Gx2 )
{
nmpcWorkspace.H[0] += + Gx1[0]*Gx2[0] + Gx1[12]*Gx2[12] + Gx1[24]*Gx2[24] + Gx1[36]*Gx2[36] + Gx1[48]*Gx2[48] + Gx1[60]*Gx2[60] + Gx1[72]*Gx2[72] + Gx1[84]*Gx2[84] + Gx1[96]*Gx2[96] + Gx1[108]*Gx2[108] + Gx1[120]*Gx2[120] + Gx1[132]*Gx2[132];
nmpcWorkspace.H[1] += + Gx1[0]*Gx2[1] + Gx1[12]*Gx2[13] + Gx1[24]*Gx2[25] + Gx1[36]*Gx2[37] + Gx1[48]*Gx2[49] + Gx1[60]*Gx2[61] + Gx1[72]*Gx2[73] + Gx1[84]*Gx2[85] + Gx1[96]*Gx2[97] + Gx1[108]*Gx2[109] + Gx1[120]*Gx2[121] + Gx1[132]*Gx2[133];
nmpcWorkspace.H[2] += + Gx1[0]*Gx2[2] + Gx1[12]*Gx2[14] + Gx1[24]*Gx2[26] + Gx1[36]*Gx2[38] + Gx1[48]*Gx2[50] + Gx1[60]*Gx2[62] + Gx1[72]*Gx2[74] + Gx1[84]*Gx2[86] + Gx1[96]*Gx2[98] + Gx1[108]*Gx2[110] + Gx1[120]*Gx2[122] + Gx1[132]*Gx2[134];
nmpcWorkspace.H[3] += + Gx1[0]*Gx2[3] + Gx1[12]*Gx2[15] + Gx1[24]*Gx2[27] + Gx1[36]*Gx2[39] + Gx1[48]*Gx2[51] + Gx1[60]*Gx2[63] + Gx1[72]*Gx2[75] + Gx1[84]*Gx2[87] + Gx1[96]*Gx2[99] + Gx1[108]*Gx2[111] + Gx1[120]*Gx2[123] + Gx1[132]*Gx2[135];
nmpcWorkspace.H[4] += + Gx1[0]*Gx2[4] + Gx1[12]*Gx2[16] + Gx1[24]*Gx2[28] + Gx1[36]*Gx2[40] + Gx1[48]*Gx2[52] + Gx1[60]*Gx2[64] + Gx1[72]*Gx2[76] + Gx1[84]*Gx2[88] + Gx1[96]*Gx2[100] + Gx1[108]*Gx2[112] + Gx1[120]*Gx2[124] + Gx1[132]*Gx2[136];
nmpcWorkspace.H[5] += + Gx1[0]*Gx2[5] + Gx1[12]*Gx2[17] + Gx1[24]*Gx2[29] + Gx1[36]*Gx2[41] + Gx1[48]*Gx2[53] + Gx1[60]*Gx2[65] + Gx1[72]*Gx2[77] + Gx1[84]*Gx2[89] + Gx1[96]*Gx2[101] + Gx1[108]*Gx2[113] + Gx1[120]*Gx2[125] + Gx1[132]*Gx2[137];
nmpcWorkspace.H[6] += + Gx1[0]*Gx2[6] + Gx1[12]*Gx2[18] + Gx1[24]*Gx2[30] + Gx1[36]*Gx2[42] + Gx1[48]*Gx2[54] + Gx1[60]*Gx2[66] + Gx1[72]*Gx2[78] + Gx1[84]*Gx2[90] + Gx1[96]*Gx2[102] + Gx1[108]*Gx2[114] + Gx1[120]*Gx2[126] + Gx1[132]*Gx2[138];
nmpcWorkspace.H[7] += + Gx1[0]*Gx2[7] + Gx1[12]*Gx2[19] + Gx1[24]*Gx2[31] + Gx1[36]*Gx2[43] + Gx1[48]*Gx2[55] + Gx1[60]*Gx2[67] + Gx1[72]*Gx2[79] + Gx1[84]*Gx2[91] + Gx1[96]*Gx2[103] + Gx1[108]*Gx2[115] + Gx1[120]*Gx2[127] + Gx1[132]*Gx2[139];
nmpcWorkspace.H[8] += + Gx1[0]*Gx2[8] + Gx1[12]*Gx2[20] + Gx1[24]*Gx2[32] + Gx1[36]*Gx2[44] + Gx1[48]*Gx2[56] + Gx1[60]*Gx2[68] + Gx1[72]*Gx2[80] + Gx1[84]*Gx2[92] + Gx1[96]*Gx2[104] + Gx1[108]*Gx2[116] + Gx1[120]*Gx2[128] + Gx1[132]*Gx2[140];
nmpcWorkspace.H[9] += + Gx1[0]*Gx2[9] + Gx1[12]*Gx2[21] + Gx1[24]*Gx2[33] + Gx1[36]*Gx2[45] + Gx1[48]*Gx2[57] + Gx1[60]*Gx2[69] + Gx1[72]*Gx2[81] + Gx1[84]*Gx2[93] + Gx1[96]*Gx2[105] + Gx1[108]*Gx2[117] + Gx1[120]*Gx2[129] + Gx1[132]*Gx2[141];
nmpcWorkspace.H[10] += + Gx1[0]*Gx2[10] + Gx1[12]*Gx2[22] + Gx1[24]*Gx2[34] + Gx1[36]*Gx2[46] + Gx1[48]*Gx2[58] + Gx1[60]*Gx2[70] + Gx1[72]*Gx2[82] + Gx1[84]*Gx2[94] + Gx1[96]*Gx2[106] + Gx1[108]*Gx2[118] + Gx1[120]*Gx2[130] + Gx1[132]*Gx2[142];
nmpcWorkspace.H[11] += + Gx1[0]*Gx2[11] + Gx1[12]*Gx2[23] + Gx1[24]*Gx2[35] + Gx1[36]*Gx2[47] + Gx1[48]*Gx2[59] + Gx1[60]*Gx2[71] + Gx1[72]*Gx2[83] + Gx1[84]*Gx2[95] + Gx1[96]*Gx2[107] + Gx1[108]*Gx2[119] + Gx1[120]*Gx2[131] + Gx1[132]*Gx2[143];
nmpcWorkspace.H[132] += + Gx1[1]*Gx2[0] + Gx1[13]*Gx2[12] + Gx1[25]*Gx2[24] + Gx1[37]*Gx2[36] + Gx1[49]*Gx2[48] + Gx1[61]*Gx2[60] + Gx1[73]*Gx2[72] + Gx1[85]*Gx2[84] + Gx1[97]*Gx2[96] + Gx1[109]*Gx2[108] + Gx1[121]*Gx2[120] + Gx1[133]*Gx2[132];
nmpcWorkspace.H[133] += + Gx1[1]*Gx2[1] + Gx1[13]*Gx2[13] + Gx1[25]*Gx2[25] + Gx1[37]*Gx2[37] + Gx1[49]*Gx2[49] + Gx1[61]*Gx2[61] + Gx1[73]*Gx2[73] + Gx1[85]*Gx2[85] + Gx1[97]*Gx2[97] + Gx1[109]*Gx2[109] + Gx1[121]*Gx2[121] + Gx1[133]*Gx2[133];
nmpcWorkspace.H[134] += + Gx1[1]*Gx2[2] + Gx1[13]*Gx2[14] + Gx1[25]*Gx2[26] + Gx1[37]*Gx2[38] + Gx1[49]*Gx2[50] + Gx1[61]*Gx2[62] + Gx1[73]*Gx2[74] + Gx1[85]*Gx2[86] + Gx1[97]*Gx2[98] + Gx1[109]*Gx2[110] + Gx1[121]*Gx2[122] + Gx1[133]*Gx2[134];
nmpcWorkspace.H[135] += + Gx1[1]*Gx2[3] + Gx1[13]*Gx2[15] + Gx1[25]*Gx2[27] + Gx1[37]*Gx2[39] + Gx1[49]*Gx2[51] + Gx1[61]*Gx2[63] + Gx1[73]*Gx2[75] + Gx1[85]*Gx2[87] + Gx1[97]*Gx2[99] + Gx1[109]*Gx2[111] + Gx1[121]*Gx2[123] + Gx1[133]*Gx2[135];
nmpcWorkspace.H[136] += + Gx1[1]*Gx2[4] + Gx1[13]*Gx2[16] + Gx1[25]*Gx2[28] + Gx1[37]*Gx2[40] + Gx1[49]*Gx2[52] + Gx1[61]*Gx2[64] + Gx1[73]*Gx2[76] + Gx1[85]*Gx2[88] + Gx1[97]*Gx2[100] + Gx1[109]*Gx2[112] + Gx1[121]*Gx2[124] + Gx1[133]*Gx2[136];
nmpcWorkspace.H[137] += + Gx1[1]*Gx2[5] + Gx1[13]*Gx2[17] + Gx1[25]*Gx2[29] + Gx1[37]*Gx2[41] + Gx1[49]*Gx2[53] + Gx1[61]*Gx2[65] + Gx1[73]*Gx2[77] + Gx1[85]*Gx2[89] + Gx1[97]*Gx2[101] + Gx1[109]*Gx2[113] + Gx1[121]*Gx2[125] + Gx1[133]*Gx2[137];
nmpcWorkspace.H[138] += + Gx1[1]*Gx2[6] + Gx1[13]*Gx2[18] + Gx1[25]*Gx2[30] + Gx1[37]*Gx2[42] + Gx1[49]*Gx2[54] + Gx1[61]*Gx2[66] + Gx1[73]*Gx2[78] + Gx1[85]*Gx2[90] + Gx1[97]*Gx2[102] + Gx1[109]*Gx2[114] + Gx1[121]*Gx2[126] + Gx1[133]*Gx2[138];
nmpcWorkspace.H[139] += + Gx1[1]*Gx2[7] + Gx1[13]*Gx2[19] + Gx1[25]*Gx2[31] + Gx1[37]*Gx2[43] + Gx1[49]*Gx2[55] + Gx1[61]*Gx2[67] + Gx1[73]*Gx2[79] + Gx1[85]*Gx2[91] + Gx1[97]*Gx2[103] + Gx1[109]*Gx2[115] + Gx1[121]*Gx2[127] + Gx1[133]*Gx2[139];
nmpcWorkspace.H[140] += + Gx1[1]*Gx2[8] + Gx1[13]*Gx2[20] + Gx1[25]*Gx2[32] + Gx1[37]*Gx2[44] + Gx1[49]*Gx2[56] + Gx1[61]*Gx2[68] + Gx1[73]*Gx2[80] + Gx1[85]*Gx2[92] + Gx1[97]*Gx2[104] + Gx1[109]*Gx2[116] + Gx1[121]*Gx2[128] + Gx1[133]*Gx2[140];
nmpcWorkspace.H[141] += + Gx1[1]*Gx2[9] + Gx1[13]*Gx2[21] + Gx1[25]*Gx2[33] + Gx1[37]*Gx2[45] + Gx1[49]*Gx2[57] + Gx1[61]*Gx2[69] + Gx1[73]*Gx2[81] + Gx1[85]*Gx2[93] + Gx1[97]*Gx2[105] + Gx1[109]*Gx2[117] + Gx1[121]*Gx2[129] + Gx1[133]*Gx2[141];
nmpcWorkspace.H[142] += + Gx1[1]*Gx2[10] + Gx1[13]*Gx2[22] + Gx1[25]*Gx2[34] + Gx1[37]*Gx2[46] + Gx1[49]*Gx2[58] + Gx1[61]*Gx2[70] + Gx1[73]*Gx2[82] + Gx1[85]*Gx2[94] + Gx1[97]*Gx2[106] + Gx1[109]*Gx2[118] + Gx1[121]*Gx2[130] + Gx1[133]*Gx2[142];
nmpcWorkspace.H[143] += + Gx1[1]*Gx2[11] + Gx1[13]*Gx2[23] + Gx1[25]*Gx2[35] + Gx1[37]*Gx2[47] + Gx1[49]*Gx2[59] + Gx1[61]*Gx2[71] + Gx1[73]*Gx2[83] + Gx1[85]*Gx2[95] + Gx1[97]*Gx2[107] + Gx1[109]*Gx2[119] + Gx1[121]*Gx2[131] + Gx1[133]*Gx2[143];
nmpcWorkspace.H[264] += + Gx1[2]*Gx2[0] + Gx1[14]*Gx2[12] + Gx1[26]*Gx2[24] + Gx1[38]*Gx2[36] + Gx1[50]*Gx2[48] + Gx1[62]*Gx2[60] + Gx1[74]*Gx2[72] + Gx1[86]*Gx2[84] + Gx1[98]*Gx2[96] + Gx1[110]*Gx2[108] + Gx1[122]*Gx2[120] + Gx1[134]*Gx2[132];
nmpcWorkspace.H[265] += + Gx1[2]*Gx2[1] + Gx1[14]*Gx2[13] + Gx1[26]*Gx2[25] + Gx1[38]*Gx2[37] + Gx1[50]*Gx2[49] + Gx1[62]*Gx2[61] + Gx1[74]*Gx2[73] + Gx1[86]*Gx2[85] + Gx1[98]*Gx2[97] + Gx1[110]*Gx2[109] + Gx1[122]*Gx2[121] + Gx1[134]*Gx2[133];
nmpcWorkspace.H[266] += + Gx1[2]*Gx2[2] + Gx1[14]*Gx2[14] + Gx1[26]*Gx2[26] + Gx1[38]*Gx2[38] + Gx1[50]*Gx2[50] + Gx1[62]*Gx2[62] + Gx1[74]*Gx2[74] + Gx1[86]*Gx2[86] + Gx1[98]*Gx2[98] + Gx1[110]*Gx2[110] + Gx1[122]*Gx2[122] + Gx1[134]*Gx2[134];
nmpcWorkspace.H[267] += + Gx1[2]*Gx2[3] + Gx1[14]*Gx2[15] + Gx1[26]*Gx2[27] + Gx1[38]*Gx2[39] + Gx1[50]*Gx2[51] + Gx1[62]*Gx2[63] + Gx1[74]*Gx2[75] + Gx1[86]*Gx2[87] + Gx1[98]*Gx2[99] + Gx1[110]*Gx2[111] + Gx1[122]*Gx2[123] + Gx1[134]*Gx2[135];
nmpcWorkspace.H[268] += + Gx1[2]*Gx2[4] + Gx1[14]*Gx2[16] + Gx1[26]*Gx2[28] + Gx1[38]*Gx2[40] + Gx1[50]*Gx2[52] + Gx1[62]*Gx2[64] + Gx1[74]*Gx2[76] + Gx1[86]*Gx2[88] + Gx1[98]*Gx2[100] + Gx1[110]*Gx2[112] + Gx1[122]*Gx2[124] + Gx1[134]*Gx2[136];
nmpcWorkspace.H[269] += + Gx1[2]*Gx2[5] + Gx1[14]*Gx2[17] + Gx1[26]*Gx2[29] + Gx1[38]*Gx2[41] + Gx1[50]*Gx2[53] + Gx1[62]*Gx2[65] + Gx1[74]*Gx2[77] + Gx1[86]*Gx2[89] + Gx1[98]*Gx2[101] + Gx1[110]*Gx2[113] + Gx1[122]*Gx2[125] + Gx1[134]*Gx2[137];
nmpcWorkspace.H[270] += + Gx1[2]*Gx2[6] + Gx1[14]*Gx2[18] + Gx1[26]*Gx2[30] + Gx1[38]*Gx2[42] + Gx1[50]*Gx2[54] + Gx1[62]*Gx2[66] + Gx1[74]*Gx2[78] + Gx1[86]*Gx2[90] + Gx1[98]*Gx2[102] + Gx1[110]*Gx2[114] + Gx1[122]*Gx2[126] + Gx1[134]*Gx2[138];
nmpcWorkspace.H[271] += + Gx1[2]*Gx2[7] + Gx1[14]*Gx2[19] + Gx1[26]*Gx2[31] + Gx1[38]*Gx2[43] + Gx1[50]*Gx2[55] + Gx1[62]*Gx2[67] + Gx1[74]*Gx2[79] + Gx1[86]*Gx2[91] + Gx1[98]*Gx2[103] + Gx1[110]*Gx2[115] + Gx1[122]*Gx2[127] + Gx1[134]*Gx2[139];
nmpcWorkspace.H[272] += + Gx1[2]*Gx2[8] + Gx1[14]*Gx2[20] + Gx1[26]*Gx2[32] + Gx1[38]*Gx2[44] + Gx1[50]*Gx2[56] + Gx1[62]*Gx2[68] + Gx1[74]*Gx2[80] + Gx1[86]*Gx2[92] + Gx1[98]*Gx2[104] + Gx1[110]*Gx2[116] + Gx1[122]*Gx2[128] + Gx1[134]*Gx2[140];
nmpcWorkspace.H[273] += + Gx1[2]*Gx2[9] + Gx1[14]*Gx2[21] + Gx1[26]*Gx2[33] + Gx1[38]*Gx2[45] + Gx1[50]*Gx2[57] + Gx1[62]*Gx2[69] + Gx1[74]*Gx2[81] + Gx1[86]*Gx2[93] + Gx1[98]*Gx2[105] + Gx1[110]*Gx2[117] + Gx1[122]*Gx2[129] + Gx1[134]*Gx2[141];
nmpcWorkspace.H[274] += + Gx1[2]*Gx2[10] + Gx1[14]*Gx2[22] + Gx1[26]*Gx2[34] + Gx1[38]*Gx2[46] + Gx1[50]*Gx2[58] + Gx1[62]*Gx2[70] + Gx1[74]*Gx2[82] + Gx1[86]*Gx2[94] + Gx1[98]*Gx2[106] + Gx1[110]*Gx2[118] + Gx1[122]*Gx2[130] + Gx1[134]*Gx2[142];
nmpcWorkspace.H[275] += + Gx1[2]*Gx2[11] + Gx1[14]*Gx2[23] + Gx1[26]*Gx2[35] + Gx1[38]*Gx2[47] + Gx1[50]*Gx2[59] + Gx1[62]*Gx2[71] + Gx1[74]*Gx2[83] + Gx1[86]*Gx2[95] + Gx1[98]*Gx2[107] + Gx1[110]*Gx2[119] + Gx1[122]*Gx2[131] + Gx1[134]*Gx2[143];
nmpcWorkspace.H[396] += + Gx1[3]*Gx2[0] + Gx1[15]*Gx2[12] + Gx1[27]*Gx2[24] + Gx1[39]*Gx2[36] + Gx1[51]*Gx2[48] + Gx1[63]*Gx2[60] + Gx1[75]*Gx2[72] + Gx1[87]*Gx2[84] + Gx1[99]*Gx2[96] + Gx1[111]*Gx2[108] + Gx1[123]*Gx2[120] + Gx1[135]*Gx2[132];
nmpcWorkspace.H[397] += + Gx1[3]*Gx2[1] + Gx1[15]*Gx2[13] + Gx1[27]*Gx2[25] + Gx1[39]*Gx2[37] + Gx1[51]*Gx2[49] + Gx1[63]*Gx2[61] + Gx1[75]*Gx2[73] + Gx1[87]*Gx2[85] + Gx1[99]*Gx2[97] + Gx1[111]*Gx2[109] + Gx1[123]*Gx2[121] + Gx1[135]*Gx2[133];
nmpcWorkspace.H[398] += + Gx1[3]*Gx2[2] + Gx1[15]*Gx2[14] + Gx1[27]*Gx2[26] + Gx1[39]*Gx2[38] + Gx1[51]*Gx2[50] + Gx1[63]*Gx2[62] + Gx1[75]*Gx2[74] + Gx1[87]*Gx2[86] + Gx1[99]*Gx2[98] + Gx1[111]*Gx2[110] + Gx1[123]*Gx2[122] + Gx1[135]*Gx2[134];
nmpcWorkspace.H[399] += + Gx1[3]*Gx2[3] + Gx1[15]*Gx2[15] + Gx1[27]*Gx2[27] + Gx1[39]*Gx2[39] + Gx1[51]*Gx2[51] + Gx1[63]*Gx2[63] + Gx1[75]*Gx2[75] + Gx1[87]*Gx2[87] + Gx1[99]*Gx2[99] + Gx1[111]*Gx2[111] + Gx1[123]*Gx2[123] + Gx1[135]*Gx2[135];
nmpcWorkspace.H[400] += + Gx1[3]*Gx2[4] + Gx1[15]*Gx2[16] + Gx1[27]*Gx2[28] + Gx1[39]*Gx2[40] + Gx1[51]*Gx2[52] + Gx1[63]*Gx2[64] + Gx1[75]*Gx2[76] + Gx1[87]*Gx2[88] + Gx1[99]*Gx2[100] + Gx1[111]*Gx2[112] + Gx1[123]*Gx2[124] + Gx1[135]*Gx2[136];
nmpcWorkspace.H[401] += + Gx1[3]*Gx2[5] + Gx1[15]*Gx2[17] + Gx1[27]*Gx2[29] + Gx1[39]*Gx2[41] + Gx1[51]*Gx2[53] + Gx1[63]*Gx2[65] + Gx1[75]*Gx2[77] + Gx1[87]*Gx2[89] + Gx1[99]*Gx2[101] + Gx1[111]*Gx2[113] + Gx1[123]*Gx2[125] + Gx1[135]*Gx2[137];
nmpcWorkspace.H[402] += + Gx1[3]*Gx2[6] + Gx1[15]*Gx2[18] + Gx1[27]*Gx2[30] + Gx1[39]*Gx2[42] + Gx1[51]*Gx2[54] + Gx1[63]*Gx2[66] + Gx1[75]*Gx2[78] + Gx1[87]*Gx2[90] + Gx1[99]*Gx2[102] + Gx1[111]*Gx2[114] + Gx1[123]*Gx2[126] + Gx1[135]*Gx2[138];
nmpcWorkspace.H[403] += + Gx1[3]*Gx2[7] + Gx1[15]*Gx2[19] + Gx1[27]*Gx2[31] + Gx1[39]*Gx2[43] + Gx1[51]*Gx2[55] + Gx1[63]*Gx2[67] + Gx1[75]*Gx2[79] + Gx1[87]*Gx2[91] + Gx1[99]*Gx2[103] + Gx1[111]*Gx2[115] + Gx1[123]*Gx2[127] + Gx1[135]*Gx2[139];
nmpcWorkspace.H[404] += + Gx1[3]*Gx2[8] + Gx1[15]*Gx2[20] + Gx1[27]*Gx2[32] + Gx1[39]*Gx2[44] + Gx1[51]*Gx2[56] + Gx1[63]*Gx2[68] + Gx1[75]*Gx2[80] + Gx1[87]*Gx2[92] + Gx1[99]*Gx2[104] + Gx1[111]*Gx2[116] + Gx1[123]*Gx2[128] + Gx1[135]*Gx2[140];
nmpcWorkspace.H[405] += + Gx1[3]*Gx2[9] + Gx1[15]*Gx2[21] + Gx1[27]*Gx2[33] + Gx1[39]*Gx2[45] + Gx1[51]*Gx2[57] + Gx1[63]*Gx2[69] + Gx1[75]*Gx2[81] + Gx1[87]*Gx2[93] + Gx1[99]*Gx2[105] + Gx1[111]*Gx2[117] + Gx1[123]*Gx2[129] + Gx1[135]*Gx2[141];
nmpcWorkspace.H[406] += + Gx1[3]*Gx2[10] + Gx1[15]*Gx2[22] + Gx1[27]*Gx2[34] + Gx1[39]*Gx2[46] + Gx1[51]*Gx2[58] + Gx1[63]*Gx2[70] + Gx1[75]*Gx2[82] + Gx1[87]*Gx2[94] + Gx1[99]*Gx2[106] + Gx1[111]*Gx2[118] + Gx1[123]*Gx2[130] + Gx1[135]*Gx2[142];
nmpcWorkspace.H[407] += + Gx1[3]*Gx2[11] + Gx1[15]*Gx2[23] + Gx1[27]*Gx2[35] + Gx1[39]*Gx2[47] + Gx1[51]*Gx2[59] + Gx1[63]*Gx2[71] + Gx1[75]*Gx2[83] + Gx1[87]*Gx2[95] + Gx1[99]*Gx2[107] + Gx1[111]*Gx2[119] + Gx1[123]*Gx2[131] + Gx1[135]*Gx2[143];
nmpcWorkspace.H[528] += + Gx1[4]*Gx2[0] + Gx1[16]*Gx2[12] + Gx1[28]*Gx2[24] + Gx1[40]*Gx2[36] + Gx1[52]*Gx2[48] + Gx1[64]*Gx2[60] + Gx1[76]*Gx2[72] + Gx1[88]*Gx2[84] + Gx1[100]*Gx2[96] + Gx1[112]*Gx2[108] + Gx1[124]*Gx2[120] + Gx1[136]*Gx2[132];
nmpcWorkspace.H[529] += + Gx1[4]*Gx2[1] + Gx1[16]*Gx2[13] + Gx1[28]*Gx2[25] + Gx1[40]*Gx2[37] + Gx1[52]*Gx2[49] + Gx1[64]*Gx2[61] + Gx1[76]*Gx2[73] + Gx1[88]*Gx2[85] + Gx1[100]*Gx2[97] + Gx1[112]*Gx2[109] + Gx1[124]*Gx2[121] + Gx1[136]*Gx2[133];
nmpcWorkspace.H[530] += + Gx1[4]*Gx2[2] + Gx1[16]*Gx2[14] + Gx1[28]*Gx2[26] + Gx1[40]*Gx2[38] + Gx1[52]*Gx2[50] + Gx1[64]*Gx2[62] + Gx1[76]*Gx2[74] + Gx1[88]*Gx2[86] + Gx1[100]*Gx2[98] + Gx1[112]*Gx2[110] + Gx1[124]*Gx2[122] + Gx1[136]*Gx2[134];
nmpcWorkspace.H[531] += + Gx1[4]*Gx2[3] + Gx1[16]*Gx2[15] + Gx1[28]*Gx2[27] + Gx1[40]*Gx2[39] + Gx1[52]*Gx2[51] + Gx1[64]*Gx2[63] + Gx1[76]*Gx2[75] + Gx1[88]*Gx2[87] + Gx1[100]*Gx2[99] + Gx1[112]*Gx2[111] + Gx1[124]*Gx2[123] + Gx1[136]*Gx2[135];
nmpcWorkspace.H[532] += + Gx1[4]*Gx2[4] + Gx1[16]*Gx2[16] + Gx1[28]*Gx2[28] + Gx1[40]*Gx2[40] + Gx1[52]*Gx2[52] + Gx1[64]*Gx2[64] + Gx1[76]*Gx2[76] + Gx1[88]*Gx2[88] + Gx1[100]*Gx2[100] + Gx1[112]*Gx2[112] + Gx1[124]*Gx2[124] + Gx1[136]*Gx2[136];
nmpcWorkspace.H[533] += + Gx1[4]*Gx2[5] + Gx1[16]*Gx2[17] + Gx1[28]*Gx2[29] + Gx1[40]*Gx2[41] + Gx1[52]*Gx2[53] + Gx1[64]*Gx2[65] + Gx1[76]*Gx2[77] + Gx1[88]*Gx2[89] + Gx1[100]*Gx2[101] + Gx1[112]*Gx2[113] + Gx1[124]*Gx2[125] + Gx1[136]*Gx2[137];
nmpcWorkspace.H[534] += + Gx1[4]*Gx2[6] + Gx1[16]*Gx2[18] + Gx1[28]*Gx2[30] + Gx1[40]*Gx2[42] + Gx1[52]*Gx2[54] + Gx1[64]*Gx2[66] + Gx1[76]*Gx2[78] + Gx1[88]*Gx2[90] + Gx1[100]*Gx2[102] + Gx1[112]*Gx2[114] + Gx1[124]*Gx2[126] + Gx1[136]*Gx2[138];
nmpcWorkspace.H[535] += + Gx1[4]*Gx2[7] + Gx1[16]*Gx2[19] + Gx1[28]*Gx2[31] + Gx1[40]*Gx2[43] + Gx1[52]*Gx2[55] + Gx1[64]*Gx2[67] + Gx1[76]*Gx2[79] + Gx1[88]*Gx2[91] + Gx1[100]*Gx2[103] + Gx1[112]*Gx2[115] + Gx1[124]*Gx2[127] + Gx1[136]*Gx2[139];
nmpcWorkspace.H[536] += + Gx1[4]*Gx2[8] + Gx1[16]*Gx2[20] + Gx1[28]*Gx2[32] + Gx1[40]*Gx2[44] + Gx1[52]*Gx2[56] + Gx1[64]*Gx2[68] + Gx1[76]*Gx2[80] + Gx1[88]*Gx2[92] + Gx1[100]*Gx2[104] + Gx1[112]*Gx2[116] + Gx1[124]*Gx2[128] + Gx1[136]*Gx2[140];
nmpcWorkspace.H[537] += + Gx1[4]*Gx2[9] + Gx1[16]*Gx2[21] + Gx1[28]*Gx2[33] + Gx1[40]*Gx2[45] + Gx1[52]*Gx2[57] + Gx1[64]*Gx2[69] + Gx1[76]*Gx2[81] + Gx1[88]*Gx2[93] + Gx1[100]*Gx2[105] + Gx1[112]*Gx2[117] + Gx1[124]*Gx2[129] + Gx1[136]*Gx2[141];
nmpcWorkspace.H[538] += + Gx1[4]*Gx2[10] + Gx1[16]*Gx2[22] + Gx1[28]*Gx2[34] + Gx1[40]*Gx2[46] + Gx1[52]*Gx2[58] + Gx1[64]*Gx2[70] + Gx1[76]*Gx2[82] + Gx1[88]*Gx2[94] + Gx1[100]*Gx2[106] + Gx1[112]*Gx2[118] + Gx1[124]*Gx2[130] + Gx1[136]*Gx2[142];
nmpcWorkspace.H[539] += + Gx1[4]*Gx2[11] + Gx1[16]*Gx2[23] + Gx1[28]*Gx2[35] + Gx1[40]*Gx2[47] + Gx1[52]*Gx2[59] + Gx1[64]*Gx2[71] + Gx1[76]*Gx2[83] + Gx1[88]*Gx2[95] + Gx1[100]*Gx2[107] + Gx1[112]*Gx2[119] + Gx1[124]*Gx2[131] + Gx1[136]*Gx2[143];
nmpcWorkspace.H[660] += + Gx1[5]*Gx2[0] + Gx1[17]*Gx2[12] + Gx1[29]*Gx2[24] + Gx1[41]*Gx2[36] + Gx1[53]*Gx2[48] + Gx1[65]*Gx2[60] + Gx1[77]*Gx2[72] + Gx1[89]*Gx2[84] + Gx1[101]*Gx2[96] + Gx1[113]*Gx2[108] + Gx1[125]*Gx2[120] + Gx1[137]*Gx2[132];
nmpcWorkspace.H[661] += + Gx1[5]*Gx2[1] + Gx1[17]*Gx2[13] + Gx1[29]*Gx2[25] + Gx1[41]*Gx2[37] + Gx1[53]*Gx2[49] + Gx1[65]*Gx2[61] + Gx1[77]*Gx2[73] + Gx1[89]*Gx2[85] + Gx1[101]*Gx2[97] + Gx1[113]*Gx2[109] + Gx1[125]*Gx2[121] + Gx1[137]*Gx2[133];
nmpcWorkspace.H[662] += + Gx1[5]*Gx2[2] + Gx1[17]*Gx2[14] + Gx1[29]*Gx2[26] + Gx1[41]*Gx2[38] + Gx1[53]*Gx2[50] + Gx1[65]*Gx2[62] + Gx1[77]*Gx2[74] + Gx1[89]*Gx2[86] + Gx1[101]*Gx2[98] + Gx1[113]*Gx2[110] + Gx1[125]*Gx2[122] + Gx1[137]*Gx2[134];
nmpcWorkspace.H[663] += + Gx1[5]*Gx2[3] + Gx1[17]*Gx2[15] + Gx1[29]*Gx2[27] + Gx1[41]*Gx2[39] + Gx1[53]*Gx2[51] + Gx1[65]*Gx2[63] + Gx1[77]*Gx2[75] + Gx1[89]*Gx2[87] + Gx1[101]*Gx2[99] + Gx1[113]*Gx2[111] + Gx1[125]*Gx2[123] + Gx1[137]*Gx2[135];
nmpcWorkspace.H[664] += + Gx1[5]*Gx2[4] + Gx1[17]*Gx2[16] + Gx1[29]*Gx2[28] + Gx1[41]*Gx2[40] + Gx1[53]*Gx2[52] + Gx1[65]*Gx2[64] + Gx1[77]*Gx2[76] + Gx1[89]*Gx2[88] + Gx1[101]*Gx2[100] + Gx1[113]*Gx2[112] + Gx1[125]*Gx2[124] + Gx1[137]*Gx2[136];
nmpcWorkspace.H[665] += + Gx1[5]*Gx2[5] + Gx1[17]*Gx2[17] + Gx1[29]*Gx2[29] + Gx1[41]*Gx2[41] + Gx1[53]*Gx2[53] + Gx1[65]*Gx2[65] + Gx1[77]*Gx2[77] + Gx1[89]*Gx2[89] + Gx1[101]*Gx2[101] + Gx1[113]*Gx2[113] + Gx1[125]*Gx2[125] + Gx1[137]*Gx2[137];
nmpcWorkspace.H[666] += + Gx1[5]*Gx2[6] + Gx1[17]*Gx2[18] + Gx1[29]*Gx2[30] + Gx1[41]*Gx2[42] + Gx1[53]*Gx2[54] + Gx1[65]*Gx2[66] + Gx1[77]*Gx2[78] + Gx1[89]*Gx2[90] + Gx1[101]*Gx2[102] + Gx1[113]*Gx2[114] + Gx1[125]*Gx2[126] + Gx1[137]*Gx2[138];
nmpcWorkspace.H[667] += + Gx1[5]*Gx2[7] + Gx1[17]*Gx2[19] + Gx1[29]*Gx2[31] + Gx1[41]*Gx2[43] + Gx1[53]*Gx2[55] + Gx1[65]*Gx2[67] + Gx1[77]*Gx2[79] + Gx1[89]*Gx2[91] + Gx1[101]*Gx2[103] + Gx1[113]*Gx2[115] + Gx1[125]*Gx2[127] + Gx1[137]*Gx2[139];
nmpcWorkspace.H[668] += + Gx1[5]*Gx2[8] + Gx1[17]*Gx2[20] + Gx1[29]*Gx2[32] + Gx1[41]*Gx2[44] + Gx1[53]*Gx2[56] + Gx1[65]*Gx2[68] + Gx1[77]*Gx2[80] + Gx1[89]*Gx2[92] + Gx1[101]*Gx2[104] + Gx1[113]*Gx2[116] + Gx1[125]*Gx2[128] + Gx1[137]*Gx2[140];
nmpcWorkspace.H[669] += + Gx1[5]*Gx2[9] + Gx1[17]*Gx2[21] + Gx1[29]*Gx2[33] + Gx1[41]*Gx2[45] + Gx1[53]*Gx2[57] + Gx1[65]*Gx2[69] + Gx1[77]*Gx2[81] + Gx1[89]*Gx2[93] + Gx1[101]*Gx2[105] + Gx1[113]*Gx2[117] + Gx1[125]*Gx2[129] + Gx1[137]*Gx2[141];
nmpcWorkspace.H[670] += + Gx1[5]*Gx2[10] + Gx1[17]*Gx2[22] + Gx1[29]*Gx2[34] + Gx1[41]*Gx2[46] + Gx1[53]*Gx2[58] + Gx1[65]*Gx2[70] + Gx1[77]*Gx2[82] + Gx1[89]*Gx2[94] + Gx1[101]*Gx2[106] + Gx1[113]*Gx2[118] + Gx1[125]*Gx2[130] + Gx1[137]*Gx2[142];
nmpcWorkspace.H[671] += + Gx1[5]*Gx2[11] + Gx1[17]*Gx2[23] + Gx1[29]*Gx2[35] + Gx1[41]*Gx2[47] + Gx1[53]*Gx2[59] + Gx1[65]*Gx2[71] + Gx1[77]*Gx2[83] + Gx1[89]*Gx2[95] + Gx1[101]*Gx2[107] + Gx1[113]*Gx2[119] + Gx1[125]*Gx2[131] + Gx1[137]*Gx2[143];
nmpcWorkspace.H[792] += + Gx1[6]*Gx2[0] + Gx1[18]*Gx2[12] + Gx1[30]*Gx2[24] + Gx1[42]*Gx2[36] + Gx1[54]*Gx2[48] + Gx1[66]*Gx2[60] + Gx1[78]*Gx2[72] + Gx1[90]*Gx2[84] + Gx1[102]*Gx2[96] + Gx1[114]*Gx2[108] + Gx1[126]*Gx2[120] + Gx1[138]*Gx2[132];
nmpcWorkspace.H[793] += + Gx1[6]*Gx2[1] + Gx1[18]*Gx2[13] + Gx1[30]*Gx2[25] + Gx1[42]*Gx2[37] + Gx1[54]*Gx2[49] + Gx1[66]*Gx2[61] + Gx1[78]*Gx2[73] + Gx1[90]*Gx2[85] + Gx1[102]*Gx2[97] + Gx1[114]*Gx2[109] + Gx1[126]*Gx2[121] + Gx1[138]*Gx2[133];
nmpcWorkspace.H[794] += + Gx1[6]*Gx2[2] + Gx1[18]*Gx2[14] + Gx1[30]*Gx2[26] + Gx1[42]*Gx2[38] + Gx1[54]*Gx2[50] + Gx1[66]*Gx2[62] + Gx1[78]*Gx2[74] + Gx1[90]*Gx2[86] + Gx1[102]*Gx2[98] + Gx1[114]*Gx2[110] + Gx1[126]*Gx2[122] + Gx1[138]*Gx2[134];
nmpcWorkspace.H[795] += + Gx1[6]*Gx2[3] + Gx1[18]*Gx2[15] + Gx1[30]*Gx2[27] + Gx1[42]*Gx2[39] + Gx1[54]*Gx2[51] + Gx1[66]*Gx2[63] + Gx1[78]*Gx2[75] + Gx1[90]*Gx2[87] + Gx1[102]*Gx2[99] + Gx1[114]*Gx2[111] + Gx1[126]*Gx2[123] + Gx1[138]*Gx2[135];
nmpcWorkspace.H[796] += + Gx1[6]*Gx2[4] + Gx1[18]*Gx2[16] + Gx1[30]*Gx2[28] + Gx1[42]*Gx2[40] + Gx1[54]*Gx2[52] + Gx1[66]*Gx2[64] + Gx1[78]*Gx2[76] + Gx1[90]*Gx2[88] + Gx1[102]*Gx2[100] + Gx1[114]*Gx2[112] + Gx1[126]*Gx2[124] + Gx1[138]*Gx2[136];
nmpcWorkspace.H[797] += + Gx1[6]*Gx2[5] + Gx1[18]*Gx2[17] + Gx1[30]*Gx2[29] + Gx1[42]*Gx2[41] + Gx1[54]*Gx2[53] + Gx1[66]*Gx2[65] + Gx1[78]*Gx2[77] + Gx1[90]*Gx2[89] + Gx1[102]*Gx2[101] + Gx1[114]*Gx2[113] + Gx1[126]*Gx2[125] + Gx1[138]*Gx2[137];
nmpcWorkspace.H[798] += + Gx1[6]*Gx2[6] + Gx1[18]*Gx2[18] + Gx1[30]*Gx2[30] + Gx1[42]*Gx2[42] + Gx1[54]*Gx2[54] + Gx1[66]*Gx2[66] + Gx1[78]*Gx2[78] + Gx1[90]*Gx2[90] + Gx1[102]*Gx2[102] + Gx1[114]*Gx2[114] + Gx1[126]*Gx2[126] + Gx1[138]*Gx2[138];
nmpcWorkspace.H[799] += + Gx1[6]*Gx2[7] + Gx1[18]*Gx2[19] + Gx1[30]*Gx2[31] + Gx1[42]*Gx2[43] + Gx1[54]*Gx2[55] + Gx1[66]*Gx2[67] + Gx1[78]*Gx2[79] + Gx1[90]*Gx2[91] + Gx1[102]*Gx2[103] + Gx1[114]*Gx2[115] + Gx1[126]*Gx2[127] + Gx1[138]*Gx2[139];
nmpcWorkspace.H[800] += + Gx1[6]*Gx2[8] + Gx1[18]*Gx2[20] + Gx1[30]*Gx2[32] + Gx1[42]*Gx2[44] + Gx1[54]*Gx2[56] + Gx1[66]*Gx2[68] + Gx1[78]*Gx2[80] + Gx1[90]*Gx2[92] + Gx1[102]*Gx2[104] + Gx1[114]*Gx2[116] + Gx1[126]*Gx2[128] + Gx1[138]*Gx2[140];
nmpcWorkspace.H[801] += + Gx1[6]*Gx2[9] + Gx1[18]*Gx2[21] + Gx1[30]*Gx2[33] + Gx1[42]*Gx2[45] + Gx1[54]*Gx2[57] + Gx1[66]*Gx2[69] + Gx1[78]*Gx2[81] + Gx1[90]*Gx2[93] + Gx1[102]*Gx2[105] + Gx1[114]*Gx2[117] + Gx1[126]*Gx2[129] + Gx1[138]*Gx2[141];
nmpcWorkspace.H[802] += + Gx1[6]*Gx2[10] + Gx1[18]*Gx2[22] + Gx1[30]*Gx2[34] + Gx1[42]*Gx2[46] + Gx1[54]*Gx2[58] + Gx1[66]*Gx2[70] + Gx1[78]*Gx2[82] + Gx1[90]*Gx2[94] + Gx1[102]*Gx2[106] + Gx1[114]*Gx2[118] + Gx1[126]*Gx2[130] + Gx1[138]*Gx2[142];
nmpcWorkspace.H[803] += + Gx1[6]*Gx2[11] + Gx1[18]*Gx2[23] + Gx1[30]*Gx2[35] + Gx1[42]*Gx2[47] + Gx1[54]*Gx2[59] + Gx1[66]*Gx2[71] + Gx1[78]*Gx2[83] + Gx1[90]*Gx2[95] + Gx1[102]*Gx2[107] + Gx1[114]*Gx2[119] + Gx1[126]*Gx2[131] + Gx1[138]*Gx2[143];
nmpcWorkspace.H[924] += + Gx1[7]*Gx2[0] + Gx1[19]*Gx2[12] + Gx1[31]*Gx2[24] + Gx1[43]*Gx2[36] + Gx1[55]*Gx2[48] + Gx1[67]*Gx2[60] + Gx1[79]*Gx2[72] + Gx1[91]*Gx2[84] + Gx1[103]*Gx2[96] + Gx1[115]*Gx2[108] + Gx1[127]*Gx2[120] + Gx1[139]*Gx2[132];
nmpcWorkspace.H[925] += + Gx1[7]*Gx2[1] + Gx1[19]*Gx2[13] + Gx1[31]*Gx2[25] + Gx1[43]*Gx2[37] + Gx1[55]*Gx2[49] + Gx1[67]*Gx2[61] + Gx1[79]*Gx2[73] + Gx1[91]*Gx2[85] + Gx1[103]*Gx2[97] + Gx1[115]*Gx2[109] + Gx1[127]*Gx2[121] + Gx1[139]*Gx2[133];
nmpcWorkspace.H[926] += + Gx1[7]*Gx2[2] + Gx1[19]*Gx2[14] + Gx1[31]*Gx2[26] + Gx1[43]*Gx2[38] + Gx1[55]*Gx2[50] + Gx1[67]*Gx2[62] + Gx1[79]*Gx2[74] + Gx1[91]*Gx2[86] + Gx1[103]*Gx2[98] + Gx1[115]*Gx2[110] + Gx1[127]*Gx2[122] + Gx1[139]*Gx2[134];
nmpcWorkspace.H[927] += + Gx1[7]*Gx2[3] + Gx1[19]*Gx2[15] + Gx1[31]*Gx2[27] + Gx1[43]*Gx2[39] + Gx1[55]*Gx2[51] + Gx1[67]*Gx2[63] + Gx1[79]*Gx2[75] + Gx1[91]*Gx2[87] + Gx1[103]*Gx2[99] + Gx1[115]*Gx2[111] + Gx1[127]*Gx2[123] + Gx1[139]*Gx2[135];
nmpcWorkspace.H[928] += + Gx1[7]*Gx2[4] + Gx1[19]*Gx2[16] + Gx1[31]*Gx2[28] + Gx1[43]*Gx2[40] + Gx1[55]*Gx2[52] + Gx1[67]*Gx2[64] + Gx1[79]*Gx2[76] + Gx1[91]*Gx2[88] + Gx1[103]*Gx2[100] + Gx1[115]*Gx2[112] + Gx1[127]*Gx2[124] + Gx1[139]*Gx2[136];
nmpcWorkspace.H[929] += + Gx1[7]*Gx2[5] + Gx1[19]*Gx2[17] + Gx1[31]*Gx2[29] + Gx1[43]*Gx2[41] + Gx1[55]*Gx2[53] + Gx1[67]*Gx2[65] + Gx1[79]*Gx2[77] + Gx1[91]*Gx2[89] + Gx1[103]*Gx2[101] + Gx1[115]*Gx2[113] + Gx1[127]*Gx2[125] + Gx1[139]*Gx2[137];
nmpcWorkspace.H[930] += + Gx1[7]*Gx2[6] + Gx1[19]*Gx2[18] + Gx1[31]*Gx2[30] + Gx1[43]*Gx2[42] + Gx1[55]*Gx2[54] + Gx1[67]*Gx2[66] + Gx1[79]*Gx2[78] + Gx1[91]*Gx2[90] + Gx1[103]*Gx2[102] + Gx1[115]*Gx2[114] + Gx1[127]*Gx2[126] + Gx1[139]*Gx2[138];
nmpcWorkspace.H[931] += + Gx1[7]*Gx2[7] + Gx1[19]*Gx2[19] + Gx1[31]*Gx2[31] + Gx1[43]*Gx2[43] + Gx1[55]*Gx2[55] + Gx1[67]*Gx2[67] + Gx1[79]*Gx2[79] + Gx1[91]*Gx2[91] + Gx1[103]*Gx2[103] + Gx1[115]*Gx2[115] + Gx1[127]*Gx2[127] + Gx1[139]*Gx2[139];
nmpcWorkspace.H[932] += + Gx1[7]*Gx2[8] + Gx1[19]*Gx2[20] + Gx1[31]*Gx2[32] + Gx1[43]*Gx2[44] + Gx1[55]*Gx2[56] + Gx1[67]*Gx2[68] + Gx1[79]*Gx2[80] + Gx1[91]*Gx2[92] + Gx1[103]*Gx2[104] + Gx1[115]*Gx2[116] + Gx1[127]*Gx2[128] + Gx1[139]*Gx2[140];
nmpcWorkspace.H[933] += + Gx1[7]*Gx2[9] + Gx1[19]*Gx2[21] + Gx1[31]*Gx2[33] + Gx1[43]*Gx2[45] + Gx1[55]*Gx2[57] + Gx1[67]*Gx2[69] + Gx1[79]*Gx2[81] + Gx1[91]*Gx2[93] + Gx1[103]*Gx2[105] + Gx1[115]*Gx2[117] + Gx1[127]*Gx2[129] + Gx1[139]*Gx2[141];
nmpcWorkspace.H[934] += + Gx1[7]*Gx2[10] + Gx1[19]*Gx2[22] + Gx1[31]*Gx2[34] + Gx1[43]*Gx2[46] + Gx1[55]*Gx2[58] + Gx1[67]*Gx2[70] + Gx1[79]*Gx2[82] + Gx1[91]*Gx2[94] + Gx1[103]*Gx2[106] + Gx1[115]*Gx2[118] + Gx1[127]*Gx2[130] + Gx1[139]*Gx2[142];
nmpcWorkspace.H[935] += + Gx1[7]*Gx2[11] + Gx1[19]*Gx2[23] + Gx1[31]*Gx2[35] + Gx1[43]*Gx2[47] + Gx1[55]*Gx2[59] + Gx1[67]*Gx2[71] + Gx1[79]*Gx2[83] + Gx1[91]*Gx2[95] + Gx1[103]*Gx2[107] + Gx1[115]*Gx2[119] + Gx1[127]*Gx2[131] + Gx1[139]*Gx2[143];
nmpcWorkspace.H[1056] += + Gx1[8]*Gx2[0] + Gx1[20]*Gx2[12] + Gx1[32]*Gx2[24] + Gx1[44]*Gx2[36] + Gx1[56]*Gx2[48] + Gx1[68]*Gx2[60] + Gx1[80]*Gx2[72] + Gx1[92]*Gx2[84] + Gx1[104]*Gx2[96] + Gx1[116]*Gx2[108] + Gx1[128]*Gx2[120] + Gx1[140]*Gx2[132];
nmpcWorkspace.H[1057] += + Gx1[8]*Gx2[1] + Gx1[20]*Gx2[13] + Gx1[32]*Gx2[25] + Gx1[44]*Gx2[37] + Gx1[56]*Gx2[49] + Gx1[68]*Gx2[61] + Gx1[80]*Gx2[73] + Gx1[92]*Gx2[85] + Gx1[104]*Gx2[97] + Gx1[116]*Gx2[109] + Gx1[128]*Gx2[121] + Gx1[140]*Gx2[133];
nmpcWorkspace.H[1058] += + Gx1[8]*Gx2[2] + Gx1[20]*Gx2[14] + Gx1[32]*Gx2[26] + Gx1[44]*Gx2[38] + Gx1[56]*Gx2[50] + Gx1[68]*Gx2[62] + Gx1[80]*Gx2[74] + Gx1[92]*Gx2[86] + Gx1[104]*Gx2[98] + Gx1[116]*Gx2[110] + Gx1[128]*Gx2[122] + Gx1[140]*Gx2[134];
nmpcWorkspace.H[1059] += + Gx1[8]*Gx2[3] + Gx1[20]*Gx2[15] + Gx1[32]*Gx2[27] + Gx1[44]*Gx2[39] + Gx1[56]*Gx2[51] + Gx1[68]*Gx2[63] + Gx1[80]*Gx2[75] + Gx1[92]*Gx2[87] + Gx1[104]*Gx2[99] + Gx1[116]*Gx2[111] + Gx1[128]*Gx2[123] + Gx1[140]*Gx2[135];
nmpcWorkspace.H[1060] += + Gx1[8]*Gx2[4] + Gx1[20]*Gx2[16] + Gx1[32]*Gx2[28] + Gx1[44]*Gx2[40] + Gx1[56]*Gx2[52] + Gx1[68]*Gx2[64] + Gx1[80]*Gx2[76] + Gx1[92]*Gx2[88] + Gx1[104]*Gx2[100] + Gx1[116]*Gx2[112] + Gx1[128]*Gx2[124] + Gx1[140]*Gx2[136];
nmpcWorkspace.H[1061] += + Gx1[8]*Gx2[5] + Gx1[20]*Gx2[17] + Gx1[32]*Gx2[29] + Gx1[44]*Gx2[41] + Gx1[56]*Gx2[53] + Gx1[68]*Gx2[65] + Gx1[80]*Gx2[77] + Gx1[92]*Gx2[89] + Gx1[104]*Gx2[101] + Gx1[116]*Gx2[113] + Gx1[128]*Gx2[125] + Gx1[140]*Gx2[137];
nmpcWorkspace.H[1062] += + Gx1[8]*Gx2[6] + Gx1[20]*Gx2[18] + Gx1[32]*Gx2[30] + Gx1[44]*Gx2[42] + Gx1[56]*Gx2[54] + Gx1[68]*Gx2[66] + Gx1[80]*Gx2[78] + Gx1[92]*Gx2[90] + Gx1[104]*Gx2[102] + Gx1[116]*Gx2[114] + Gx1[128]*Gx2[126] + Gx1[140]*Gx2[138];
nmpcWorkspace.H[1063] += + Gx1[8]*Gx2[7] + Gx1[20]*Gx2[19] + Gx1[32]*Gx2[31] + Gx1[44]*Gx2[43] + Gx1[56]*Gx2[55] + Gx1[68]*Gx2[67] + Gx1[80]*Gx2[79] + Gx1[92]*Gx2[91] + Gx1[104]*Gx2[103] + Gx1[116]*Gx2[115] + Gx1[128]*Gx2[127] + Gx1[140]*Gx2[139];
nmpcWorkspace.H[1064] += + Gx1[8]*Gx2[8] + Gx1[20]*Gx2[20] + Gx1[32]*Gx2[32] + Gx1[44]*Gx2[44] + Gx1[56]*Gx2[56] + Gx1[68]*Gx2[68] + Gx1[80]*Gx2[80] + Gx1[92]*Gx2[92] + Gx1[104]*Gx2[104] + Gx1[116]*Gx2[116] + Gx1[128]*Gx2[128] + Gx1[140]*Gx2[140];
nmpcWorkspace.H[1065] += + Gx1[8]*Gx2[9] + Gx1[20]*Gx2[21] + Gx1[32]*Gx2[33] + Gx1[44]*Gx2[45] + Gx1[56]*Gx2[57] + Gx1[68]*Gx2[69] + Gx1[80]*Gx2[81] + Gx1[92]*Gx2[93] + Gx1[104]*Gx2[105] + Gx1[116]*Gx2[117] + Gx1[128]*Gx2[129] + Gx1[140]*Gx2[141];
nmpcWorkspace.H[1066] += + Gx1[8]*Gx2[10] + Gx1[20]*Gx2[22] + Gx1[32]*Gx2[34] + Gx1[44]*Gx2[46] + Gx1[56]*Gx2[58] + Gx1[68]*Gx2[70] + Gx1[80]*Gx2[82] + Gx1[92]*Gx2[94] + Gx1[104]*Gx2[106] + Gx1[116]*Gx2[118] + Gx1[128]*Gx2[130] + Gx1[140]*Gx2[142];
nmpcWorkspace.H[1067] += + Gx1[8]*Gx2[11] + Gx1[20]*Gx2[23] + Gx1[32]*Gx2[35] + Gx1[44]*Gx2[47] + Gx1[56]*Gx2[59] + Gx1[68]*Gx2[71] + Gx1[80]*Gx2[83] + Gx1[92]*Gx2[95] + Gx1[104]*Gx2[107] + Gx1[116]*Gx2[119] + Gx1[128]*Gx2[131] + Gx1[140]*Gx2[143];
nmpcWorkspace.H[1188] += + Gx1[9]*Gx2[0] + Gx1[21]*Gx2[12] + Gx1[33]*Gx2[24] + Gx1[45]*Gx2[36] + Gx1[57]*Gx2[48] + Gx1[69]*Gx2[60] + Gx1[81]*Gx2[72] + Gx1[93]*Gx2[84] + Gx1[105]*Gx2[96] + Gx1[117]*Gx2[108] + Gx1[129]*Gx2[120] + Gx1[141]*Gx2[132];
nmpcWorkspace.H[1189] += + Gx1[9]*Gx2[1] + Gx1[21]*Gx2[13] + Gx1[33]*Gx2[25] + Gx1[45]*Gx2[37] + Gx1[57]*Gx2[49] + Gx1[69]*Gx2[61] + Gx1[81]*Gx2[73] + Gx1[93]*Gx2[85] + Gx1[105]*Gx2[97] + Gx1[117]*Gx2[109] + Gx1[129]*Gx2[121] + Gx1[141]*Gx2[133];
nmpcWorkspace.H[1190] += + Gx1[9]*Gx2[2] + Gx1[21]*Gx2[14] + Gx1[33]*Gx2[26] + Gx1[45]*Gx2[38] + Gx1[57]*Gx2[50] + Gx1[69]*Gx2[62] + Gx1[81]*Gx2[74] + Gx1[93]*Gx2[86] + Gx1[105]*Gx2[98] + Gx1[117]*Gx2[110] + Gx1[129]*Gx2[122] + Gx1[141]*Gx2[134];
nmpcWorkspace.H[1191] += + Gx1[9]*Gx2[3] + Gx1[21]*Gx2[15] + Gx1[33]*Gx2[27] + Gx1[45]*Gx2[39] + Gx1[57]*Gx2[51] + Gx1[69]*Gx2[63] + Gx1[81]*Gx2[75] + Gx1[93]*Gx2[87] + Gx1[105]*Gx2[99] + Gx1[117]*Gx2[111] + Gx1[129]*Gx2[123] + Gx1[141]*Gx2[135];
nmpcWorkspace.H[1192] += + Gx1[9]*Gx2[4] + Gx1[21]*Gx2[16] + Gx1[33]*Gx2[28] + Gx1[45]*Gx2[40] + Gx1[57]*Gx2[52] + Gx1[69]*Gx2[64] + Gx1[81]*Gx2[76] + Gx1[93]*Gx2[88] + Gx1[105]*Gx2[100] + Gx1[117]*Gx2[112] + Gx1[129]*Gx2[124] + Gx1[141]*Gx2[136];
nmpcWorkspace.H[1193] += + Gx1[9]*Gx2[5] + Gx1[21]*Gx2[17] + Gx1[33]*Gx2[29] + Gx1[45]*Gx2[41] + Gx1[57]*Gx2[53] + Gx1[69]*Gx2[65] + Gx1[81]*Gx2[77] + Gx1[93]*Gx2[89] + Gx1[105]*Gx2[101] + Gx1[117]*Gx2[113] + Gx1[129]*Gx2[125] + Gx1[141]*Gx2[137];
nmpcWorkspace.H[1194] += + Gx1[9]*Gx2[6] + Gx1[21]*Gx2[18] + Gx1[33]*Gx2[30] + Gx1[45]*Gx2[42] + Gx1[57]*Gx2[54] + Gx1[69]*Gx2[66] + Gx1[81]*Gx2[78] + Gx1[93]*Gx2[90] + Gx1[105]*Gx2[102] + Gx1[117]*Gx2[114] + Gx1[129]*Gx2[126] + Gx1[141]*Gx2[138];
nmpcWorkspace.H[1195] += + Gx1[9]*Gx2[7] + Gx1[21]*Gx2[19] + Gx1[33]*Gx2[31] + Gx1[45]*Gx2[43] + Gx1[57]*Gx2[55] + Gx1[69]*Gx2[67] + Gx1[81]*Gx2[79] + Gx1[93]*Gx2[91] + Gx1[105]*Gx2[103] + Gx1[117]*Gx2[115] + Gx1[129]*Gx2[127] + Gx1[141]*Gx2[139];
nmpcWorkspace.H[1196] += + Gx1[9]*Gx2[8] + Gx1[21]*Gx2[20] + Gx1[33]*Gx2[32] + Gx1[45]*Gx2[44] + Gx1[57]*Gx2[56] + Gx1[69]*Gx2[68] + Gx1[81]*Gx2[80] + Gx1[93]*Gx2[92] + Gx1[105]*Gx2[104] + Gx1[117]*Gx2[116] + Gx1[129]*Gx2[128] + Gx1[141]*Gx2[140];
nmpcWorkspace.H[1197] += + Gx1[9]*Gx2[9] + Gx1[21]*Gx2[21] + Gx1[33]*Gx2[33] + Gx1[45]*Gx2[45] + Gx1[57]*Gx2[57] + Gx1[69]*Gx2[69] + Gx1[81]*Gx2[81] + Gx1[93]*Gx2[93] + Gx1[105]*Gx2[105] + Gx1[117]*Gx2[117] + Gx1[129]*Gx2[129] + Gx1[141]*Gx2[141];
nmpcWorkspace.H[1198] += + Gx1[9]*Gx2[10] + Gx1[21]*Gx2[22] + Gx1[33]*Gx2[34] + Gx1[45]*Gx2[46] + Gx1[57]*Gx2[58] + Gx1[69]*Gx2[70] + Gx1[81]*Gx2[82] + Gx1[93]*Gx2[94] + Gx1[105]*Gx2[106] + Gx1[117]*Gx2[118] + Gx1[129]*Gx2[130] + Gx1[141]*Gx2[142];
nmpcWorkspace.H[1199] += + Gx1[9]*Gx2[11] + Gx1[21]*Gx2[23] + Gx1[33]*Gx2[35] + Gx1[45]*Gx2[47] + Gx1[57]*Gx2[59] + Gx1[69]*Gx2[71] + Gx1[81]*Gx2[83] + Gx1[93]*Gx2[95] + Gx1[105]*Gx2[107] + Gx1[117]*Gx2[119] + Gx1[129]*Gx2[131] + Gx1[141]*Gx2[143];
nmpcWorkspace.H[1320] += + Gx1[10]*Gx2[0] + Gx1[22]*Gx2[12] + Gx1[34]*Gx2[24] + Gx1[46]*Gx2[36] + Gx1[58]*Gx2[48] + Gx1[70]*Gx2[60] + Gx1[82]*Gx2[72] + Gx1[94]*Gx2[84] + Gx1[106]*Gx2[96] + Gx1[118]*Gx2[108] + Gx1[130]*Gx2[120] + Gx1[142]*Gx2[132];
nmpcWorkspace.H[1321] += + Gx1[10]*Gx2[1] + Gx1[22]*Gx2[13] + Gx1[34]*Gx2[25] + Gx1[46]*Gx2[37] + Gx1[58]*Gx2[49] + Gx1[70]*Gx2[61] + Gx1[82]*Gx2[73] + Gx1[94]*Gx2[85] + Gx1[106]*Gx2[97] + Gx1[118]*Gx2[109] + Gx1[130]*Gx2[121] + Gx1[142]*Gx2[133];
nmpcWorkspace.H[1322] += + Gx1[10]*Gx2[2] + Gx1[22]*Gx2[14] + Gx1[34]*Gx2[26] + Gx1[46]*Gx2[38] + Gx1[58]*Gx2[50] + Gx1[70]*Gx2[62] + Gx1[82]*Gx2[74] + Gx1[94]*Gx2[86] + Gx1[106]*Gx2[98] + Gx1[118]*Gx2[110] + Gx1[130]*Gx2[122] + Gx1[142]*Gx2[134];
nmpcWorkspace.H[1323] += + Gx1[10]*Gx2[3] + Gx1[22]*Gx2[15] + Gx1[34]*Gx2[27] + Gx1[46]*Gx2[39] + Gx1[58]*Gx2[51] + Gx1[70]*Gx2[63] + Gx1[82]*Gx2[75] + Gx1[94]*Gx2[87] + Gx1[106]*Gx2[99] + Gx1[118]*Gx2[111] + Gx1[130]*Gx2[123] + Gx1[142]*Gx2[135];
nmpcWorkspace.H[1324] += + Gx1[10]*Gx2[4] + Gx1[22]*Gx2[16] + Gx1[34]*Gx2[28] + Gx1[46]*Gx2[40] + Gx1[58]*Gx2[52] + Gx1[70]*Gx2[64] + Gx1[82]*Gx2[76] + Gx1[94]*Gx2[88] + Gx1[106]*Gx2[100] + Gx1[118]*Gx2[112] + Gx1[130]*Gx2[124] + Gx1[142]*Gx2[136];
nmpcWorkspace.H[1325] += + Gx1[10]*Gx2[5] + Gx1[22]*Gx2[17] + Gx1[34]*Gx2[29] + Gx1[46]*Gx2[41] + Gx1[58]*Gx2[53] + Gx1[70]*Gx2[65] + Gx1[82]*Gx2[77] + Gx1[94]*Gx2[89] + Gx1[106]*Gx2[101] + Gx1[118]*Gx2[113] + Gx1[130]*Gx2[125] + Gx1[142]*Gx2[137];
nmpcWorkspace.H[1326] += + Gx1[10]*Gx2[6] + Gx1[22]*Gx2[18] + Gx1[34]*Gx2[30] + Gx1[46]*Gx2[42] + Gx1[58]*Gx2[54] + Gx1[70]*Gx2[66] + Gx1[82]*Gx2[78] + Gx1[94]*Gx2[90] + Gx1[106]*Gx2[102] + Gx1[118]*Gx2[114] + Gx1[130]*Gx2[126] + Gx1[142]*Gx2[138];
nmpcWorkspace.H[1327] += + Gx1[10]*Gx2[7] + Gx1[22]*Gx2[19] + Gx1[34]*Gx2[31] + Gx1[46]*Gx2[43] + Gx1[58]*Gx2[55] + Gx1[70]*Gx2[67] + Gx1[82]*Gx2[79] + Gx1[94]*Gx2[91] + Gx1[106]*Gx2[103] + Gx1[118]*Gx2[115] + Gx1[130]*Gx2[127] + Gx1[142]*Gx2[139];
nmpcWorkspace.H[1328] += + Gx1[10]*Gx2[8] + Gx1[22]*Gx2[20] + Gx1[34]*Gx2[32] + Gx1[46]*Gx2[44] + Gx1[58]*Gx2[56] + Gx1[70]*Gx2[68] + Gx1[82]*Gx2[80] + Gx1[94]*Gx2[92] + Gx1[106]*Gx2[104] + Gx1[118]*Gx2[116] + Gx1[130]*Gx2[128] + Gx1[142]*Gx2[140];
nmpcWorkspace.H[1329] += + Gx1[10]*Gx2[9] + Gx1[22]*Gx2[21] + Gx1[34]*Gx2[33] + Gx1[46]*Gx2[45] + Gx1[58]*Gx2[57] + Gx1[70]*Gx2[69] + Gx1[82]*Gx2[81] + Gx1[94]*Gx2[93] + Gx1[106]*Gx2[105] + Gx1[118]*Gx2[117] + Gx1[130]*Gx2[129] + Gx1[142]*Gx2[141];
nmpcWorkspace.H[1330] += + Gx1[10]*Gx2[10] + Gx1[22]*Gx2[22] + Gx1[34]*Gx2[34] + Gx1[46]*Gx2[46] + Gx1[58]*Gx2[58] + Gx1[70]*Gx2[70] + Gx1[82]*Gx2[82] + Gx1[94]*Gx2[94] + Gx1[106]*Gx2[106] + Gx1[118]*Gx2[118] + Gx1[130]*Gx2[130] + Gx1[142]*Gx2[142];
nmpcWorkspace.H[1331] += + Gx1[10]*Gx2[11] + Gx1[22]*Gx2[23] + Gx1[34]*Gx2[35] + Gx1[46]*Gx2[47] + Gx1[58]*Gx2[59] + Gx1[70]*Gx2[71] + Gx1[82]*Gx2[83] + Gx1[94]*Gx2[95] + Gx1[106]*Gx2[107] + Gx1[118]*Gx2[119] + Gx1[130]*Gx2[131] + Gx1[142]*Gx2[143];
nmpcWorkspace.H[1452] += + Gx1[11]*Gx2[0] + Gx1[23]*Gx2[12] + Gx1[35]*Gx2[24] + Gx1[47]*Gx2[36] + Gx1[59]*Gx2[48] + Gx1[71]*Gx2[60] + Gx1[83]*Gx2[72] + Gx1[95]*Gx2[84] + Gx1[107]*Gx2[96] + Gx1[119]*Gx2[108] + Gx1[131]*Gx2[120] + Gx1[143]*Gx2[132];
nmpcWorkspace.H[1453] += + Gx1[11]*Gx2[1] + Gx1[23]*Gx2[13] + Gx1[35]*Gx2[25] + Gx1[47]*Gx2[37] + Gx1[59]*Gx2[49] + Gx1[71]*Gx2[61] + Gx1[83]*Gx2[73] + Gx1[95]*Gx2[85] + Gx1[107]*Gx2[97] + Gx1[119]*Gx2[109] + Gx1[131]*Gx2[121] + Gx1[143]*Gx2[133];
nmpcWorkspace.H[1454] += + Gx1[11]*Gx2[2] + Gx1[23]*Gx2[14] + Gx1[35]*Gx2[26] + Gx1[47]*Gx2[38] + Gx1[59]*Gx2[50] + Gx1[71]*Gx2[62] + Gx1[83]*Gx2[74] + Gx1[95]*Gx2[86] + Gx1[107]*Gx2[98] + Gx1[119]*Gx2[110] + Gx1[131]*Gx2[122] + Gx1[143]*Gx2[134];
nmpcWorkspace.H[1455] += + Gx1[11]*Gx2[3] + Gx1[23]*Gx2[15] + Gx1[35]*Gx2[27] + Gx1[47]*Gx2[39] + Gx1[59]*Gx2[51] + Gx1[71]*Gx2[63] + Gx1[83]*Gx2[75] + Gx1[95]*Gx2[87] + Gx1[107]*Gx2[99] + Gx1[119]*Gx2[111] + Gx1[131]*Gx2[123] + Gx1[143]*Gx2[135];
nmpcWorkspace.H[1456] += + Gx1[11]*Gx2[4] + Gx1[23]*Gx2[16] + Gx1[35]*Gx2[28] + Gx1[47]*Gx2[40] + Gx1[59]*Gx2[52] + Gx1[71]*Gx2[64] + Gx1[83]*Gx2[76] + Gx1[95]*Gx2[88] + Gx1[107]*Gx2[100] + Gx1[119]*Gx2[112] + Gx1[131]*Gx2[124] + Gx1[143]*Gx2[136];
nmpcWorkspace.H[1457] += + Gx1[11]*Gx2[5] + Gx1[23]*Gx2[17] + Gx1[35]*Gx2[29] + Gx1[47]*Gx2[41] + Gx1[59]*Gx2[53] + Gx1[71]*Gx2[65] + Gx1[83]*Gx2[77] + Gx1[95]*Gx2[89] + Gx1[107]*Gx2[101] + Gx1[119]*Gx2[113] + Gx1[131]*Gx2[125] + Gx1[143]*Gx2[137];
nmpcWorkspace.H[1458] += + Gx1[11]*Gx2[6] + Gx1[23]*Gx2[18] + Gx1[35]*Gx2[30] + Gx1[47]*Gx2[42] + Gx1[59]*Gx2[54] + Gx1[71]*Gx2[66] + Gx1[83]*Gx2[78] + Gx1[95]*Gx2[90] + Gx1[107]*Gx2[102] + Gx1[119]*Gx2[114] + Gx1[131]*Gx2[126] + Gx1[143]*Gx2[138];
nmpcWorkspace.H[1459] += + Gx1[11]*Gx2[7] + Gx1[23]*Gx2[19] + Gx1[35]*Gx2[31] + Gx1[47]*Gx2[43] + Gx1[59]*Gx2[55] + Gx1[71]*Gx2[67] + Gx1[83]*Gx2[79] + Gx1[95]*Gx2[91] + Gx1[107]*Gx2[103] + Gx1[119]*Gx2[115] + Gx1[131]*Gx2[127] + Gx1[143]*Gx2[139];
nmpcWorkspace.H[1460] += + Gx1[11]*Gx2[8] + Gx1[23]*Gx2[20] + Gx1[35]*Gx2[32] + Gx1[47]*Gx2[44] + Gx1[59]*Gx2[56] + Gx1[71]*Gx2[68] + Gx1[83]*Gx2[80] + Gx1[95]*Gx2[92] + Gx1[107]*Gx2[104] + Gx1[119]*Gx2[116] + Gx1[131]*Gx2[128] + Gx1[143]*Gx2[140];
nmpcWorkspace.H[1461] += + Gx1[11]*Gx2[9] + Gx1[23]*Gx2[21] + Gx1[35]*Gx2[33] + Gx1[47]*Gx2[45] + Gx1[59]*Gx2[57] + Gx1[71]*Gx2[69] + Gx1[83]*Gx2[81] + Gx1[95]*Gx2[93] + Gx1[107]*Gx2[105] + Gx1[119]*Gx2[117] + Gx1[131]*Gx2[129] + Gx1[143]*Gx2[141];
nmpcWorkspace.H[1462] += + Gx1[11]*Gx2[10] + Gx1[23]*Gx2[22] + Gx1[35]*Gx2[34] + Gx1[47]*Gx2[46] + Gx1[59]*Gx2[58] + Gx1[71]*Gx2[70] + Gx1[83]*Gx2[82] + Gx1[95]*Gx2[94] + Gx1[107]*Gx2[106] + Gx1[119]*Gx2[118] + Gx1[131]*Gx2[130] + Gx1[143]*Gx2[142];
nmpcWorkspace.H[1463] += + Gx1[11]*Gx2[11] + Gx1[23]*Gx2[23] + Gx1[35]*Gx2[35] + Gx1[47]*Gx2[47] + Gx1[59]*Gx2[59] + Gx1[71]*Gx2[71] + Gx1[83]*Gx2[83] + Gx1[95]*Gx2[95] + Gx1[107]*Gx2[107] + Gx1[119]*Gx2[119] + Gx1[131]*Gx2[131] + Gx1[143]*Gx2[143];
}

void nmpc_macCTSlx( real_t* const C0, real_t* const g0 )
{
g0[0] += 0.0;
;
g0[1] += 0.0;
;
g0[2] += 0.0;
;
g0[3] += 0.0;
;
g0[4] += 0.0;
;
g0[5] += 0.0;
;
g0[6] += 0.0;
;
g0[7] += 0.0;
;
g0[8] += 0.0;
;
g0[9] += 0.0;
;
g0[10] += 0.0;
;
g0[11] += 0.0;
;
}

void nmpc_macETSlu( real_t* const E0, real_t* const g1 )
{
g1[0] += 0.0;
;
g1[1] += 0.0;
;
g1[2] += 0.0;
;
g1[3] += 0.0;
;
}

void nmpc_condensePrep(  )
{
int lRun1;
int lRun2;
int lRun3;
int lRun4;
int lRun5;
nmpc_moveGuE( nmpcWorkspace.evGu, nmpcWorkspace.E );
for (lRun1 = 1; lRun1 < 30; ++lRun1)
{
nmpc_moveGxT( &(nmpcWorkspace.evGx[ lRun1 * 144 ]), nmpcWorkspace.T );
nmpc_multGxd( &(nmpcWorkspace.d[ lRun1 * 12-12 ]), &(nmpcWorkspace.evGx[ lRun1 * 144 ]), &(nmpcWorkspace.d[ lRun1 * 12 ]) );
nmpc_multGxGx( nmpcWorkspace.T, &(nmpcWorkspace.evGx[ lRun1 * 144-144 ]), &(nmpcWorkspace.evGx[ lRun1 * 144 ]) );
for (lRun2 = 0; lRun2 < lRun1; ++lRun2)
{
lRun4 = (((lRun1) * (lRun1-1)) / (2)) + (lRun2);
lRun3 = (((lRun1 + 1) * (lRun1)) / (2)) + (lRun2);
nmpc_multGxGu( nmpcWorkspace.T, &(nmpcWorkspace.E[ lRun4 * 48 ]), &(nmpcWorkspace.E[ lRun3 * 48 ]) );
}
lRun3 = (((lRun1 + 1) * (lRun1)) / (2)) + (lRun2);
nmpc_moveGuE( &(nmpcWorkspace.evGu[ lRun1 * 48 ]), &(nmpcWorkspace.E[ lRun3 * 48 ]) );
}

nmpc_multGxGx( &(nmpcWorkspace.Q1[ 144 ]), nmpcWorkspace.evGx, nmpcWorkspace.QGx );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 288 ]), &(nmpcWorkspace.evGx[ 144 ]), &(nmpcWorkspace.QGx[ 144 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 432 ]), &(nmpcWorkspace.evGx[ 288 ]), &(nmpcWorkspace.QGx[ 288 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 576 ]), &(nmpcWorkspace.evGx[ 432 ]), &(nmpcWorkspace.QGx[ 432 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 720 ]), &(nmpcWorkspace.evGx[ 576 ]), &(nmpcWorkspace.QGx[ 576 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 864 ]), &(nmpcWorkspace.evGx[ 720 ]), &(nmpcWorkspace.QGx[ 720 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 1008 ]), &(nmpcWorkspace.evGx[ 864 ]), &(nmpcWorkspace.QGx[ 864 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 1152 ]), &(nmpcWorkspace.evGx[ 1008 ]), &(nmpcWorkspace.QGx[ 1008 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 1296 ]), &(nmpcWorkspace.evGx[ 1152 ]), &(nmpcWorkspace.QGx[ 1152 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 1440 ]), &(nmpcWorkspace.evGx[ 1296 ]), &(nmpcWorkspace.QGx[ 1296 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 1584 ]), &(nmpcWorkspace.evGx[ 1440 ]), &(nmpcWorkspace.QGx[ 1440 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 1728 ]), &(nmpcWorkspace.evGx[ 1584 ]), &(nmpcWorkspace.QGx[ 1584 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 1872 ]), &(nmpcWorkspace.evGx[ 1728 ]), &(nmpcWorkspace.QGx[ 1728 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 2016 ]), &(nmpcWorkspace.evGx[ 1872 ]), &(nmpcWorkspace.QGx[ 1872 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 2160 ]), &(nmpcWorkspace.evGx[ 2016 ]), &(nmpcWorkspace.QGx[ 2016 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 2304 ]), &(nmpcWorkspace.evGx[ 2160 ]), &(nmpcWorkspace.QGx[ 2160 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 2448 ]), &(nmpcWorkspace.evGx[ 2304 ]), &(nmpcWorkspace.QGx[ 2304 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 2592 ]), &(nmpcWorkspace.evGx[ 2448 ]), &(nmpcWorkspace.QGx[ 2448 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 2736 ]), &(nmpcWorkspace.evGx[ 2592 ]), &(nmpcWorkspace.QGx[ 2592 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 2880 ]), &(nmpcWorkspace.evGx[ 2736 ]), &(nmpcWorkspace.QGx[ 2736 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 3024 ]), &(nmpcWorkspace.evGx[ 2880 ]), &(nmpcWorkspace.QGx[ 2880 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 3168 ]), &(nmpcWorkspace.evGx[ 3024 ]), &(nmpcWorkspace.QGx[ 3024 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 3312 ]), &(nmpcWorkspace.evGx[ 3168 ]), &(nmpcWorkspace.QGx[ 3168 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 3456 ]), &(nmpcWorkspace.evGx[ 3312 ]), &(nmpcWorkspace.QGx[ 3312 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 3600 ]), &(nmpcWorkspace.evGx[ 3456 ]), &(nmpcWorkspace.QGx[ 3456 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 3744 ]), &(nmpcWorkspace.evGx[ 3600 ]), &(nmpcWorkspace.QGx[ 3600 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 3888 ]), &(nmpcWorkspace.evGx[ 3744 ]), &(nmpcWorkspace.QGx[ 3744 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 4032 ]), &(nmpcWorkspace.evGx[ 3888 ]), &(nmpcWorkspace.QGx[ 3888 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 4176 ]), &(nmpcWorkspace.evGx[ 4032 ]), &(nmpcWorkspace.QGx[ 4032 ]) );
nmpc_multGxGx( nmpcWorkspace.QN1, &(nmpcWorkspace.evGx[ 4176 ]), &(nmpcWorkspace.QGx[ 4176 ]) );

for (lRun1 = 0; lRun1 < 29; ++lRun1)
{
for (lRun2 = 0; lRun2 < lRun1 + 1; ++lRun2)
{
lRun3 = (((lRun1 + 1) * (lRun1)) / (2)) + (lRun2);
nmpc_multGxGu( &(nmpcWorkspace.Q1[ lRun1 * 144 + 144 ]), &(nmpcWorkspace.E[ lRun3 * 48 ]), &(nmpcWorkspace.QE[ lRun3 * 48 ]) );
}
}

for (lRun2 = 0; lRun2 < lRun1 + 1; ++lRun2)
{
lRun3 = (((lRun1 + 1) * (lRun1)) / (2)) + (lRun2);
nmpc_multGxGu( nmpcWorkspace.QN1, &(nmpcWorkspace.E[ lRun3 * 48 ]), &(nmpcWorkspace.QE[ lRun3 * 48 ]) );
}

nmpc_zeroBlockH00(  );
nmpc_multCTQC( nmpcWorkspace.evGx, nmpcWorkspace.QGx );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 144 ]), &(nmpcWorkspace.QGx[ 144 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 288 ]), &(nmpcWorkspace.QGx[ 288 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 432 ]), &(nmpcWorkspace.QGx[ 432 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 576 ]), &(nmpcWorkspace.QGx[ 576 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 720 ]), &(nmpcWorkspace.QGx[ 720 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 864 ]), &(nmpcWorkspace.QGx[ 864 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 1008 ]), &(nmpcWorkspace.QGx[ 1008 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 1152 ]), &(nmpcWorkspace.QGx[ 1152 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 1296 ]), &(nmpcWorkspace.QGx[ 1296 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 1440 ]), &(nmpcWorkspace.QGx[ 1440 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 1584 ]), &(nmpcWorkspace.QGx[ 1584 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 1728 ]), &(nmpcWorkspace.QGx[ 1728 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 1872 ]), &(nmpcWorkspace.QGx[ 1872 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 2016 ]), &(nmpcWorkspace.QGx[ 2016 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 2160 ]), &(nmpcWorkspace.QGx[ 2160 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 2304 ]), &(nmpcWorkspace.QGx[ 2304 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 2448 ]), &(nmpcWorkspace.QGx[ 2448 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 2592 ]), &(nmpcWorkspace.QGx[ 2592 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 2736 ]), &(nmpcWorkspace.QGx[ 2736 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 2880 ]), &(nmpcWorkspace.QGx[ 2880 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 3024 ]), &(nmpcWorkspace.QGx[ 3024 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 3168 ]), &(nmpcWorkspace.QGx[ 3168 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 3312 ]), &(nmpcWorkspace.QGx[ 3312 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 3456 ]), &(nmpcWorkspace.QGx[ 3456 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 3600 ]), &(nmpcWorkspace.QGx[ 3600 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 3744 ]), &(nmpcWorkspace.QGx[ 3744 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 3888 ]), &(nmpcWorkspace.QGx[ 3888 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 4032 ]), &(nmpcWorkspace.QGx[ 4032 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 4176 ]), &(nmpcWorkspace.QGx[ 4176 ]) );

for (lRun1 = 0; lRun1 < 30; ++lRun1)
{
nmpc_zeroBlockH10( &(nmpcWorkspace.H10[ lRun1 * 48 ]) );
for (lRun2 = lRun1; lRun2 < 30; ++lRun2)
{
lRun3 = (((lRun2 + 1) * (lRun2)) / (2)) + (lRun1);
nmpc_multQETGx( &(nmpcWorkspace.QE[ lRun3 * 48 ]), &(nmpcWorkspace.evGx[ lRun2 * 144 ]), &(nmpcWorkspace.H10[ lRun1 * 48 ]) );
}
}

for (lRun1 = 0;lRun1 < 12; ++lRun1)
for (lRun2 = 0;lRun2 < 120; ++lRun2)
nmpcWorkspace.H[(lRun1 * 132) + (lRun2 + 12)] = nmpcWorkspace.H10[(lRun2 * 12) + (lRun1)];

for (lRun1 = 0; lRun1 < 30; ++lRun1)
{
nmpc_setBlockH11_R1( lRun1, lRun1, &(nmpcWorkspace.R1[ lRun1 * 16 ]) );
lRun2 = lRun1;
for (lRun3 = lRun1; lRun3 < 30; ++lRun3)
{
lRun4 = (((lRun3 + 1) * (lRun3)) / (2)) + (lRun1);
lRun5 = (((lRun3 + 1) * (lRun3)) / (2)) + (lRun2);
nmpc_setBlockH11( lRun1, lRun2, &(nmpcWorkspace.E[ lRun4 * 48 ]), &(nmpcWorkspace.QE[ lRun5 * 48 ]) );
}
for (lRun2 = lRun1 + 1; lRun2 < 30; ++lRun2)
{
nmpc_zeroBlockH11( lRun1, lRun2 );
for (lRun3 = lRun2; lRun3 < 30; ++lRun3)
{
lRun4 = (((lRun3 + 1) * (lRun3)) / (2)) + (lRun1);
lRun5 = (((lRun3 + 1) * (lRun3)) / (2)) + (lRun2);
nmpc_setBlockH11( lRun1, lRun2, &(nmpcWorkspace.E[ lRun4 * 48 ]), &(nmpcWorkspace.QE[ lRun5 * 48 ]) );
}
}
}

for (lRun1 = 0; lRun1 < 30; ++lRun1)
{
for (lRun2 = 0; lRun2 < lRun1; ++lRun2)
{
nmpc_copyHTH( lRun1, lRun2 );
}
}

for (lRun1 = 0;lRun1 < 120; ++lRun1)
for (lRun2 = 0;lRun2 < 12; ++lRun2)
nmpcWorkspace.H[(lRun1 * 132 + 1584) + (lRun2)] = nmpcWorkspace.H10[(lRun1 * 12) + (lRun2)];

nmpc_multQ1d( &(nmpcWorkspace.Q1[ 144 ]), nmpcWorkspace.d, nmpcWorkspace.Qd );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 288 ]), &(nmpcWorkspace.d[ 12 ]), &(nmpcWorkspace.Qd[ 12 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 432 ]), &(nmpcWorkspace.d[ 24 ]), &(nmpcWorkspace.Qd[ 24 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 576 ]), &(nmpcWorkspace.d[ 36 ]), &(nmpcWorkspace.Qd[ 36 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 720 ]), &(nmpcWorkspace.d[ 48 ]), &(nmpcWorkspace.Qd[ 48 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 864 ]), &(nmpcWorkspace.d[ 60 ]), &(nmpcWorkspace.Qd[ 60 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 1008 ]), &(nmpcWorkspace.d[ 72 ]), &(nmpcWorkspace.Qd[ 72 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 1152 ]), &(nmpcWorkspace.d[ 84 ]), &(nmpcWorkspace.Qd[ 84 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 1296 ]), &(nmpcWorkspace.d[ 96 ]), &(nmpcWorkspace.Qd[ 96 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 1440 ]), &(nmpcWorkspace.d[ 108 ]), &(nmpcWorkspace.Qd[ 108 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 1584 ]), &(nmpcWorkspace.d[ 120 ]), &(nmpcWorkspace.Qd[ 120 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 1728 ]), &(nmpcWorkspace.d[ 132 ]), &(nmpcWorkspace.Qd[ 132 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 1872 ]), &(nmpcWorkspace.d[ 144 ]), &(nmpcWorkspace.Qd[ 144 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 2016 ]), &(nmpcWorkspace.d[ 156 ]), &(nmpcWorkspace.Qd[ 156 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 2160 ]), &(nmpcWorkspace.d[ 168 ]), &(nmpcWorkspace.Qd[ 168 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 2304 ]), &(nmpcWorkspace.d[ 180 ]), &(nmpcWorkspace.Qd[ 180 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 2448 ]), &(nmpcWorkspace.d[ 192 ]), &(nmpcWorkspace.Qd[ 192 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 2592 ]), &(nmpcWorkspace.d[ 204 ]), &(nmpcWorkspace.Qd[ 204 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 2736 ]), &(nmpcWorkspace.d[ 216 ]), &(nmpcWorkspace.Qd[ 216 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 2880 ]), &(nmpcWorkspace.d[ 228 ]), &(nmpcWorkspace.Qd[ 228 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 3024 ]), &(nmpcWorkspace.d[ 240 ]), &(nmpcWorkspace.Qd[ 240 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 3168 ]), &(nmpcWorkspace.d[ 252 ]), &(nmpcWorkspace.Qd[ 252 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 3312 ]), &(nmpcWorkspace.d[ 264 ]), &(nmpcWorkspace.Qd[ 264 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 3456 ]), &(nmpcWorkspace.d[ 276 ]), &(nmpcWorkspace.Qd[ 276 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 3600 ]), &(nmpcWorkspace.d[ 288 ]), &(nmpcWorkspace.Qd[ 288 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 3744 ]), &(nmpcWorkspace.d[ 300 ]), &(nmpcWorkspace.Qd[ 300 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 3888 ]), &(nmpcWorkspace.d[ 312 ]), &(nmpcWorkspace.Qd[ 312 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 4032 ]), &(nmpcWorkspace.d[ 324 ]), &(nmpcWorkspace.Qd[ 324 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 4176 ]), &(nmpcWorkspace.d[ 336 ]), &(nmpcWorkspace.Qd[ 336 ]) );
nmpc_multQN1d( nmpcWorkspace.QN1, &(nmpcWorkspace.d[ 348 ]), &(nmpcWorkspace.Qd[ 348 ]) );

nmpc_macCTSlx( nmpcWorkspace.evGx, nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 144 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 288 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 432 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 576 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 720 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 864 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 1008 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 1152 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 1296 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 1440 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 1584 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 1728 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 1872 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 2016 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 2160 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 2304 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 2448 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 2592 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 2736 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 2880 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 3024 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 3168 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 3312 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 3456 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 3600 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 3744 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 3888 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 4032 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 4176 ]), nmpcWorkspace.g );
for (lRun1 = 0; lRun1 < 30; ++lRun1)
{
for (lRun2 = lRun1; lRun2 < 30; ++lRun2)
{
lRun3 = (((lRun2 + 1) * (lRun2)) / (2)) + (lRun1);
nmpc_macETSlu( &(nmpcWorkspace.QE[ lRun3 * 48 ]), &(nmpcWorkspace.g[ lRun1 * 4 + 12 ]) );
}
}
nmpcWorkspace.lb[12] = (real_t)-6.9813170079773179e-01 - nmpcVariables.u[0];
nmpcWorkspace.lb[13] = (real_t)-6.9813170079773179e-01 - nmpcVariables.u[1];
nmpcWorkspace.lb[14] = (real_t)-1.0000000000000000e+12 - nmpcVariables.u[2];
nmpcWorkspace.lb[15] = (real_t)1.1183399999999999e+01 - nmpcVariables.u[3];
nmpcWorkspace.lb[16] = (real_t)-6.9813170079773179e-01 - nmpcVariables.u[4];
nmpcWorkspace.lb[17] = (real_t)-6.9813170079773179e-01 - nmpcVariables.u[5];
nmpcWorkspace.lb[18] = (real_t)-1.0000000000000000e+12 - nmpcVariables.u[6];
nmpcWorkspace.lb[19] = (real_t)1.1183399999999999e+01 - nmpcVariables.u[7];
nmpcWorkspace.lb[20] = (real_t)-6.9813170079773179e-01 - nmpcVariables.u[8];
nmpcWorkspace.lb[21] = (real_t)-6.9813170079773179e-01 - nmpcVariables.u[9];
nmpcWorkspace.lb[22] = (real_t)-1.0000000000000000e+12 - nmpcVariables.u[10];
nmpcWorkspace.lb[23] = (real_t)1.1183399999999999e+01 - nmpcVariables.u[11];
nmpcWorkspace.lb[24] = (real_t)-6.9813170079773179e-01 - nmpcVariables.u[12];
nmpcWorkspace.lb[25] = (real_t)-6.9813170079773179e-01 - nmpcVariables.u[13];
nmpcWorkspace.lb[26] = (real_t)-1.0000000000000000e+12 - nmpcVariables.u[14];
nmpcWorkspace.lb[27] = (real_t)1.1183399999999999e+01 - nmpcVariables.u[15];
nmpcWorkspace.lb[28] = (real_t)-6.9813170079773179e-01 - nmpcVariables.u[16];
nmpcWorkspace.lb[29] = (real_t)-6.9813170079773179e-01 - nmpcVariables.u[17];
nmpcWorkspace.lb[30] = (real_t)-1.0000000000000000e+12 - nmpcVariables.u[18];
nmpcWorkspace.lb[31] = (real_t)1.1183399999999999e+01 - nmpcVariables.u[19];
nmpcWorkspace.lb[32] = (real_t)-6.9813170079773179e-01 - nmpcVariables.u[20];
nmpcWorkspace.lb[33] = (real_t)-6.9813170079773179e-01 - nmpcVariables.u[21];
nmpcWorkspace.lb[34] = (real_t)-1.0000000000000000e+12 - nmpcVariables.u[22];
nmpcWorkspace.lb[35] = (real_t)1.1183399999999999e+01 - nmpcVariables.u[23];
nmpcWorkspace.lb[36] = (real_t)-6.9813170079773179e-01 - nmpcVariables.u[24];
nmpcWorkspace.lb[37] = (real_t)-6.9813170079773179e-01 - nmpcVariables.u[25];
nmpcWorkspace.lb[38] = (real_t)-1.0000000000000000e+12 - nmpcVariables.u[26];
nmpcWorkspace.lb[39] = (real_t)1.1183399999999999e+01 - nmpcVariables.u[27];
nmpcWorkspace.lb[40] = (real_t)-6.9813170079773179e-01 - nmpcVariables.u[28];
nmpcWorkspace.lb[41] = (real_t)-6.9813170079773179e-01 - nmpcVariables.u[29];
nmpcWorkspace.lb[42] = (real_t)-1.0000000000000000e+12 - nmpcVariables.u[30];
nmpcWorkspace.lb[43] = (real_t)1.1183399999999999e+01 - nmpcVariables.u[31];
nmpcWorkspace.lb[44] = (real_t)-6.9813170079773179e-01 - nmpcVariables.u[32];
nmpcWorkspace.lb[45] = (real_t)-6.9813170079773179e-01 - nmpcVariables.u[33];
nmpcWorkspace.lb[46] = (real_t)-1.0000000000000000e+12 - nmpcVariables.u[34];
nmpcWorkspace.lb[47] = (real_t)1.1183399999999999e+01 - nmpcVariables.u[35];
nmpcWorkspace.lb[48] = (real_t)-6.9813170079773179e-01 - nmpcVariables.u[36];
nmpcWorkspace.lb[49] = (real_t)-6.9813170079773179e-01 - nmpcVariables.u[37];
nmpcWorkspace.lb[50] = (real_t)-1.0000000000000000e+12 - nmpcVariables.u[38];
nmpcWorkspace.lb[51] = (real_t)1.1183399999999999e+01 - nmpcVariables.u[39];
nmpcWorkspace.lb[52] = (real_t)-6.9813170079773179e-01 - nmpcVariables.u[40];
nmpcWorkspace.lb[53] = (real_t)-6.9813170079773179e-01 - nmpcVariables.u[41];
nmpcWorkspace.lb[54] = (real_t)-1.0000000000000000e+12 - nmpcVariables.u[42];
nmpcWorkspace.lb[55] = (real_t)1.1183399999999999e+01 - nmpcVariables.u[43];
nmpcWorkspace.lb[56] = (real_t)-6.9813170079773179e-01 - nmpcVariables.u[44];
nmpcWorkspace.lb[57] = (real_t)-6.9813170079773179e-01 - nmpcVariables.u[45];
nmpcWorkspace.lb[58] = (real_t)-1.0000000000000000e+12 - nmpcVariables.u[46];
nmpcWorkspace.lb[59] = (real_t)1.1183399999999999e+01 - nmpcVariables.u[47];
nmpcWorkspace.lb[60] = (real_t)-6.9813170079773179e-01 - nmpcVariables.u[48];
nmpcWorkspace.lb[61] = (real_t)-6.9813170079773179e-01 - nmpcVariables.u[49];
nmpcWorkspace.lb[62] = (real_t)-1.0000000000000000e+12 - nmpcVariables.u[50];
nmpcWorkspace.lb[63] = (real_t)1.1183399999999999e+01 - nmpcVariables.u[51];
nmpcWorkspace.lb[64] = (real_t)-6.9813170079773179e-01 - nmpcVariables.u[52];
nmpcWorkspace.lb[65] = (real_t)-6.9813170079773179e-01 - nmpcVariables.u[53];
nmpcWorkspace.lb[66] = (real_t)-1.0000000000000000e+12 - nmpcVariables.u[54];
nmpcWorkspace.lb[67] = (real_t)1.1183399999999999e+01 - nmpcVariables.u[55];
nmpcWorkspace.lb[68] = (real_t)-6.9813170079773179e-01 - nmpcVariables.u[56];
nmpcWorkspace.lb[69] = (real_t)-6.9813170079773179e-01 - nmpcVariables.u[57];
nmpcWorkspace.lb[70] = (real_t)-1.0000000000000000e+12 - nmpcVariables.u[58];
nmpcWorkspace.lb[71] = (real_t)1.1183399999999999e+01 - nmpcVariables.u[59];
nmpcWorkspace.lb[72] = (real_t)-6.9813170079773179e-01 - nmpcVariables.u[60];
nmpcWorkspace.lb[73] = (real_t)-6.9813170079773179e-01 - nmpcVariables.u[61];
nmpcWorkspace.lb[74] = (real_t)-1.0000000000000000e+12 - nmpcVariables.u[62];
nmpcWorkspace.lb[75] = (real_t)1.1183399999999999e+01 - nmpcVariables.u[63];
nmpcWorkspace.lb[76] = (real_t)-6.9813170079773179e-01 - nmpcVariables.u[64];
nmpcWorkspace.lb[77] = (real_t)-6.9813170079773179e-01 - nmpcVariables.u[65];
nmpcWorkspace.lb[78] = (real_t)-1.0000000000000000e+12 - nmpcVariables.u[66];
nmpcWorkspace.lb[79] = (real_t)1.1183399999999999e+01 - nmpcVariables.u[67];
nmpcWorkspace.lb[80] = (real_t)-6.9813170079773179e-01 - nmpcVariables.u[68];
nmpcWorkspace.lb[81] = (real_t)-6.9813170079773179e-01 - nmpcVariables.u[69];
nmpcWorkspace.lb[82] = (real_t)-1.0000000000000000e+12 - nmpcVariables.u[70];
nmpcWorkspace.lb[83] = (real_t)1.1183399999999999e+01 - nmpcVariables.u[71];
nmpcWorkspace.lb[84] = (real_t)-6.9813170079773179e-01 - nmpcVariables.u[72];
nmpcWorkspace.lb[85] = (real_t)-6.9813170079773179e-01 - nmpcVariables.u[73];
nmpcWorkspace.lb[86] = (real_t)-1.0000000000000000e+12 - nmpcVariables.u[74];
nmpcWorkspace.lb[87] = (real_t)1.1183399999999999e+01 - nmpcVariables.u[75];
nmpcWorkspace.lb[88] = (real_t)-6.9813170079773179e-01 - nmpcVariables.u[76];
nmpcWorkspace.lb[89] = (real_t)-6.9813170079773179e-01 - nmpcVariables.u[77];
nmpcWorkspace.lb[90] = (real_t)-1.0000000000000000e+12 - nmpcVariables.u[78];
nmpcWorkspace.lb[91] = (real_t)1.1183399999999999e+01 - nmpcVariables.u[79];
nmpcWorkspace.lb[92] = (real_t)-6.9813170079773179e-01 - nmpcVariables.u[80];
nmpcWorkspace.lb[93] = (real_t)-6.9813170079773179e-01 - nmpcVariables.u[81];
nmpcWorkspace.lb[94] = (real_t)-1.0000000000000000e+12 - nmpcVariables.u[82];
nmpcWorkspace.lb[95] = (real_t)1.1183399999999999e+01 - nmpcVariables.u[83];
nmpcWorkspace.lb[96] = (real_t)-6.9813170079773179e-01 - nmpcVariables.u[84];
nmpcWorkspace.lb[97] = (real_t)-6.9813170079773179e-01 - nmpcVariables.u[85];
nmpcWorkspace.lb[98] = (real_t)-1.0000000000000000e+12 - nmpcVariables.u[86];
nmpcWorkspace.lb[99] = (real_t)1.1183399999999999e+01 - nmpcVariables.u[87];
nmpcWorkspace.lb[100] = (real_t)-6.9813170079773179e-01 - nmpcVariables.u[88];
nmpcWorkspace.lb[101] = (real_t)-6.9813170079773179e-01 - nmpcVariables.u[89];
nmpcWorkspace.lb[102] = (real_t)-1.0000000000000000e+12 - nmpcVariables.u[90];
nmpcWorkspace.lb[103] = (real_t)1.1183399999999999e+01 - nmpcVariables.u[91];
nmpcWorkspace.lb[104] = (real_t)-6.9813170079773179e-01 - nmpcVariables.u[92];
nmpcWorkspace.lb[105] = (real_t)-6.9813170079773179e-01 - nmpcVariables.u[93];
nmpcWorkspace.lb[106] = (real_t)-1.0000000000000000e+12 - nmpcVariables.u[94];
nmpcWorkspace.lb[107] = (real_t)1.1183399999999999e+01 - nmpcVariables.u[95];
nmpcWorkspace.lb[108] = (real_t)-6.9813170079773179e-01 - nmpcVariables.u[96];
nmpcWorkspace.lb[109] = (real_t)-6.9813170079773179e-01 - nmpcVariables.u[97];
nmpcWorkspace.lb[110] = (real_t)-1.0000000000000000e+12 - nmpcVariables.u[98];
nmpcWorkspace.lb[111] = (real_t)1.1183399999999999e+01 - nmpcVariables.u[99];
nmpcWorkspace.lb[112] = (real_t)-6.9813170079773179e-01 - nmpcVariables.u[100];
nmpcWorkspace.lb[113] = (real_t)-6.9813170079773179e-01 - nmpcVariables.u[101];
nmpcWorkspace.lb[114] = (real_t)-1.0000000000000000e+12 - nmpcVariables.u[102];
nmpcWorkspace.lb[115] = (real_t)1.1183399999999999e+01 - nmpcVariables.u[103];
nmpcWorkspace.lb[116] = (real_t)-6.9813170079773179e-01 - nmpcVariables.u[104];
nmpcWorkspace.lb[117] = (real_t)-6.9813170079773179e-01 - nmpcVariables.u[105];
nmpcWorkspace.lb[118] = (real_t)-1.0000000000000000e+12 - nmpcVariables.u[106];
nmpcWorkspace.lb[119] = (real_t)1.1183399999999999e+01 - nmpcVariables.u[107];
nmpcWorkspace.lb[120] = (real_t)-6.9813170079773179e-01 - nmpcVariables.u[108];
nmpcWorkspace.lb[121] = (real_t)-6.9813170079773179e-01 - nmpcVariables.u[109];
nmpcWorkspace.lb[122] = (real_t)-1.0000000000000000e+12 - nmpcVariables.u[110];
nmpcWorkspace.lb[123] = (real_t)1.1183399999999999e+01 - nmpcVariables.u[111];
nmpcWorkspace.lb[124] = (real_t)-6.9813170079773179e-01 - nmpcVariables.u[112];
nmpcWorkspace.lb[125] = (real_t)-6.9813170079773179e-01 - nmpcVariables.u[113];
nmpcWorkspace.lb[126] = (real_t)-1.0000000000000000e+12 - nmpcVariables.u[114];
nmpcWorkspace.lb[127] = (real_t)1.1183399999999999e+01 - nmpcVariables.u[115];
nmpcWorkspace.lb[128] = (real_t)-6.9813170079773179e-01 - nmpcVariables.u[116];
nmpcWorkspace.lb[129] = (real_t)-6.9813170079773179e-01 - nmpcVariables.u[117];
nmpcWorkspace.lb[130] = (real_t)-1.0000000000000000e+12 - nmpcVariables.u[118];
nmpcWorkspace.lb[131] = (real_t)1.1183399999999999e+01 - nmpcVariables.u[119];
nmpcWorkspace.ub[12] = (real_t)6.9813170079773179e-01 - nmpcVariables.u[0];
nmpcWorkspace.ub[13] = (real_t)6.9813170079773179e-01 - nmpcVariables.u[1];
nmpcWorkspace.ub[14] = (real_t)1.0000000000000000e+12 - nmpcVariables.u[2];
nmpcWorkspace.ub[15] = (real_t)7.4555999999999997e+01 - nmpcVariables.u[3];
nmpcWorkspace.ub[16] = (real_t)6.9813170079773179e-01 - nmpcVariables.u[4];
nmpcWorkspace.ub[17] = (real_t)6.9813170079773179e-01 - nmpcVariables.u[5];
nmpcWorkspace.ub[18] = (real_t)1.0000000000000000e+12 - nmpcVariables.u[6];
nmpcWorkspace.ub[19] = (real_t)7.4555999999999997e+01 - nmpcVariables.u[7];
nmpcWorkspace.ub[20] = (real_t)6.9813170079773179e-01 - nmpcVariables.u[8];
nmpcWorkspace.ub[21] = (real_t)6.9813170079773179e-01 - nmpcVariables.u[9];
nmpcWorkspace.ub[22] = (real_t)1.0000000000000000e+12 - nmpcVariables.u[10];
nmpcWorkspace.ub[23] = (real_t)7.4555999999999997e+01 - nmpcVariables.u[11];
nmpcWorkspace.ub[24] = (real_t)6.9813170079773179e-01 - nmpcVariables.u[12];
nmpcWorkspace.ub[25] = (real_t)6.9813170079773179e-01 - nmpcVariables.u[13];
nmpcWorkspace.ub[26] = (real_t)1.0000000000000000e+12 - nmpcVariables.u[14];
nmpcWorkspace.ub[27] = (real_t)7.4555999999999997e+01 - nmpcVariables.u[15];
nmpcWorkspace.ub[28] = (real_t)6.9813170079773179e-01 - nmpcVariables.u[16];
nmpcWorkspace.ub[29] = (real_t)6.9813170079773179e-01 - nmpcVariables.u[17];
nmpcWorkspace.ub[30] = (real_t)1.0000000000000000e+12 - nmpcVariables.u[18];
nmpcWorkspace.ub[31] = (real_t)7.4555999999999997e+01 - nmpcVariables.u[19];
nmpcWorkspace.ub[32] = (real_t)6.9813170079773179e-01 - nmpcVariables.u[20];
nmpcWorkspace.ub[33] = (real_t)6.9813170079773179e-01 - nmpcVariables.u[21];
nmpcWorkspace.ub[34] = (real_t)1.0000000000000000e+12 - nmpcVariables.u[22];
nmpcWorkspace.ub[35] = (real_t)7.4555999999999997e+01 - nmpcVariables.u[23];
nmpcWorkspace.ub[36] = (real_t)6.9813170079773179e-01 - nmpcVariables.u[24];
nmpcWorkspace.ub[37] = (real_t)6.9813170079773179e-01 - nmpcVariables.u[25];
nmpcWorkspace.ub[38] = (real_t)1.0000000000000000e+12 - nmpcVariables.u[26];
nmpcWorkspace.ub[39] = (real_t)7.4555999999999997e+01 - nmpcVariables.u[27];
nmpcWorkspace.ub[40] = (real_t)6.9813170079773179e-01 - nmpcVariables.u[28];
nmpcWorkspace.ub[41] = (real_t)6.9813170079773179e-01 - nmpcVariables.u[29];
nmpcWorkspace.ub[42] = (real_t)1.0000000000000000e+12 - nmpcVariables.u[30];
nmpcWorkspace.ub[43] = (real_t)7.4555999999999997e+01 - nmpcVariables.u[31];
nmpcWorkspace.ub[44] = (real_t)6.9813170079773179e-01 - nmpcVariables.u[32];
nmpcWorkspace.ub[45] = (real_t)6.9813170079773179e-01 - nmpcVariables.u[33];
nmpcWorkspace.ub[46] = (real_t)1.0000000000000000e+12 - nmpcVariables.u[34];
nmpcWorkspace.ub[47] = (real_t)7.4555999999999997e+01 - nmpcVariables.u[35];
nmpcWorkspace.ub[48] = (real_t)6.9813170079773179e-01 - nmpcVariables.u[36];
nmpcWorkspace.ub[49] = (real_t)6.9813170079773179e-01 - nmpcVariables.u[37];
nmpcWorkspace.ub[50] = (real_t)1.0000000000000000e+12 - nmpcVariables.u[38];
nmpcWorkspace.ub[51] = (real_t)7.4555999999999997e+01 - nmpcVariables.u[39];
nmpcWorkspace.ub[52] = (real_t)6.9813170079773179e-01 - nmpcVariables.u[40];
nmpcWorkspace.ub[53] = (real_t)6.9813170079773179e-01 - nmpcVariables.u[41];
nmpcWorkspace.ub[54] = (real_t)1.0000000000000000e+12 - nmpcVariables.u[42];
nmpcWorkspace.ub[55] = (real_t)7.4555999999999997e+01 - nmpcVariables.u[43];
nmpcWorkspace.ub[56] = (real_t)6.9813170079773179e-01 - nmpcVariables.u[44];
nmpcWorkspace.ub[57] = (real_t)6.9813170079773179e-01 - nmpcVariables.u[45];
nmpcWorkspace.ub[58] = (real_t)1.0000000000000000e+12 - nmpcVariables.u[46];
nmpcWorkspace.ub[59] = (real_t)7.4555999999999997e+01 - nmpcVariables.u[47];
nmpcWorkspace.ub[60] = (real_t)6.9813170079773179e-01 - nmpcVariables.u[48];
nmpcWorkspace.ub[61] = (real_t)6.9813170079773179e-01 - nmpcVariables.u[49];
nmpcWorkspace.ub[62] = (real_t)1.0000000000000000e+12 - nmpcVariables.u[50];
nmpcWorkspace.ub[63] = (real_t)7.4555999999999997e+01 - nmpcVariables.u[51];
nmpcWorkspace.ub[64] = (real_t)6.9813170079773179e-01 - nmpcVariables.u[52];
nmpcWorkspace.ub[65] = (real_t)6.9813170079773179e-01 - nmpcVariables.u[53];
nmpcWorkspace.ub[66] = (real_t)1.0000000000000000e+12 - nmpcVariables.u[54];
nmpcWorkspace.ub[67] = (real_t)7.4555999999999997e+01 - nmpcVariables.u[55];
nmpcWorkspace.ub[68] = (real_t)6.9813170079773179e-01 - nmpcVariables.u[56];
nmpcWorkspace.ub[69] = (real_t)6.9813170079773179e-01 - nmpcVariables.u[57];
nmpcWorkspace.ub[70] = (real_t)1.0000000000000000e+12 - nmpcVariables.u[58];
nmpcWorkspace.ub[71] = (real_t)7.4555999999999997e+01 - nmpcVariables.u[59];
nmpcWorkspace.ub[72] = (real_t)6.9813170079773179e-01 - nmpcVariables.u[60];
nmpcWorkspace.ub[73] = (real_t)6.9813170079773179e-01 - nmpcVariables.u[61];
nmpcWorkspace.ub[74] = (real_t)1.0000000000000000e+12 - nmpcVariables.u[62];
nmpcWorkspace.ub[75] = (real_t)7.4555999999999997e+01 - nmpcVariables.u[63];
nmpcWorkspace.ub[76] = (real_t)6.9813170079773179e-01 - nmpcVariables.u[64];
nmpcWorkspace.ub[77] = (real_t)6.9813170079773179e-01 - nmpcVariables.u[65];
nmpcWorkspace.ub[78] = (real_t)1.0000000000000000e+12 - nmpcVariables.u[66];
nmpcWorkspace.ub[79] = (real_t)7.4555999999999997e+01 - nmpcVariables.u[67];
nmpcWorkspace.ub[80] = (real_t)6.9813170079773179e-01 - nmpcVariables.u[68];
nmpcWorkspace.ub[81] = (real_t)6.9813170079773179e-01 - nmpcVariables.u[69];
nmpcWorkspace.ub[82] = (real_t)1.0000000000000000e+12 - nmpcVariables.u[70];
nmpcWorkspace.ub[83] = (real_t)7.4555999999999997e+01 - nmpcVariables.u[71];
nmpcWorkspace.ub[84] = (real_t)6.9813170079773179e-01 - nmpcVariables.u[72];
nmpcWorkspace.ub[85] = (real_t)6.9813170079773179e-01 - nmpcVariables.u[73];
nmpcWorkspace.ub[86] = (real_t)1.0000000000000000e+12 - nmpcVariables.u[74];
nmpcWorkspace.ub[87] = (real_t)7.4555999999999997e+01 - nmpcVariables.u[75];
nmpcWorkspace.ub[88] = (real_t)6.9813170079773179e-01 - nmpcVariables.u[76];
nmpcWorkspace.ub[89] = (real_t)6.9813170079773179e-01 - nmpcVariables.u[77];
nmpcWorkspace.ub[90] = (real_t)1.0000000000000000e+12 - nmpcVariables.u[78];
nmpcWorkspace.ub[91] = (real_t)7.4555999999999997e+01 - nmpcVariables.u[79];
nmpcWorkspace.ub[92] = (real_t)6.9813170079773179e-01 - nmpcVariables.u[80];
nmpcWorkspace.ub[93] = (real_t)6.9813170079773179e-01 - nmpcVariables.u[81];
nmpcWorkspace.ub[94] = (real_t)1.0000000000000000e+12 - nmpcVariables.u[82];
nmpcWorkspace.ub[95] = (real_t)7.4555999999999997e+01 - nmpcVariables.u[83];
nmpcWorkspace.ub[96] = (real_t)6.9813170079773179e-01 - nmpcVariables.u[84];
nmpcWorkspace.ub[97] = (real_t)6.9813170079773179e-01 - nmpcVariables.u[85];
nmpcWorkspace.ub[98] = (real_t)1.0000000000000000e+12 - nmpcVariables.u[86];
nmpcWorkspace.ub[99] = (real_t)7.4555999999999997e+01 - nmpcVariables.u[87];
nmpcWorkspace.ub[100] = (real_t)6.9813170079773179e-01 - nmpcVariables.u[88];
nmpcWorkspace.ub[101] = (real_t)6.9813170079773179e-01 - nmpcVariables.u[89];
nmpcWorkspace.ub[102] = (real_t)1.0000000000000000e+12 - nmpcVariables.u[90];
nmpcWorkspace.ub[103] = (real_t)7.4555999999999997e+01 - nmpcVariables.u[91];
nmpcWorkspace.ub[104] = (real_t)6.9813170079773179e-01 - nmpcVariables.u[92];
nmpcWorkspace.ub[105] = (real_t)6.9813170079773179e-01 - nmpcVariables.u[93];
nmpcWorkspace.ub[106] = (real_t)1.0000000000000000e+12 - nmpcVariables.u[94];
nmpcWorkspace.ub[107] = (real_t)7.4555999999999997e+01 - nmpcVariables.u[95];
nmpcWorkspace.ub[108] = (real_t)6.9813170079773179e-01 - nmpcVariables.u[96];
nmpcWorkspace.ub[109] = (real_t)6.9813170079773179e-01 - nmpcVariables.u[97];
nmpcWorkspace.ub[110] = (real_t)1.0000000000000000e+12 - nmpcVariables.u[98];
nmpcWorkspace.ub[111] = (real_t)7.4555999999999997e+01 - nmpcVariables.u[99];
nmpcWorkspace.ub[112] = (real_t)6.9813170079773179e-01 - nmpcVariables.u[100];
nmpcWorkspace.ub[113] = (real_t)6.9813170079773179e-01 - nmpcVariables.u[101];
nmpcWorkspace.ub[114] = (real_t)1.0000000000000000e+12 - nmpcVariables.u[102];
nmpcWorkspace.ub[115] = (real_t)7.4555999999999997e+01 - nmpcVariables.u[103];
nmpcWorkspace.ub[116] = (real_t)6.9813170079773179e-01 - nmpcVariables.u[104];
nmpcWorkspace.ub[117] = (real_t)6.9813170079773179e-01 - nmpcVariables.u[105];
nmpcWorkspace.ub[118] = (real_t)1.0000000000000000e+12 - nmpcVariables.u[106];
nmpcWorkspace.ub[119] = (real_t)7.4555999999999997e+01 - nmpcVariables.u[107];
nmpcWorkspace.ub[120] = (real_t)6.9813170079773179e-01 - nmpcVariables.u[108];
nmpcWorkspace.ub[121] = (real_t)6.9813170079773179e-01 - nmpcVariables.u[109];
nmpcWorkspace.ub[122] = (real_t)1.0000000000000000e+12 - nmpcVariables.u[110];
nmpcWorkspace.ub[123] = (real_t)7.4555999999999997e+01 - nmpcVariables.u[111];
nmpcWorkspace.ub[124] = (real_t)6.9813170079773179e-01 - nmpcVariables.u[112];
nmpcWorkspace.ub[125] = (real_t)6.9813170079773179e-01 - nmpcVariables.u[113];
nmpcWorkspace.ub[126] = (real_t)1.0000000000000000e+12 - nmpcVariables.u[114];
nmpcWorkspace.ub[127] = (real_t)7.4555999999999997e+01 - nmpcVariables.u[115];
nmpcWorkspace.ub[128] = (real_t)6.9813170079773179e-01 - nmpcVariables.u[116];
nmpcWorkspace.ub[129] = (real_t)6.9813170079773179e-01 - nmpcVariables.u[117];
nmpcWorkspace.ub[130] = (real_t)1.0000000000000000e+12 - nmpcVariables.u[118];
nmpcWorkspace.ub[131] = (real_t)7.4555999999999997e+01 - nmpcVariables.u[119];

}

void nmpc_condenseFdb(  )
{
int lRun1;
int lRun2;
int lRun3;
int lRun4;
int lRun5;
nmpcWorkspace.Dx0[0] = nmpcVariables.x0[0] - nmpcVariables.x[0];
nmpcWorkspace.Dx0[1] = nmpcVariables.x0[1] - nmpcVariables.x[1];
nmpcWorkspace.Dx0[2] = nmpcVariables.x0[2] - nmpcVariables.x[2];
nmpcWorkspace.Dx0[3] = nmpcVariables.x0[3] - nmpcVariables.x[3];
nmpcWorkspace.Dx0[4] = nmpcVariables.x0[4] - nmpcVariables.x[4];
nmpcWorkspace.Dx0[5] = nmpcVariables.x0[5] - nmpcVariables.x[5];
nmpcWorkspace.Dx0[6] = nmpcVariables.x0[6] - nmpcVariables.x[6];
nmpcWorkspace.Dx0[7] = nmpcVariables.x0[7] - nmpcVariables.x[7];
nmpcWorkspace.Dx0[8] = nmpcVariables.x0[8] - nmpcVariables.x[8];
nmpcWorkspace.Dx0[9] = nmpcVariables.x0[9] - nmpcVariables.x[9];
nmpcWorkspace.Dx0[10] = nmpcVariables.x0[10] - nmpcVariables.x[10];
nmpcWorkspace.Dx0[11] = nmpcVariables.x0[11] - nmpcVariables.x[11];

for (lRun2 = 0; lRun2 < 420; ++lRun2)
nmpcWorkspace.Dy[lRun2] -= nmpcVariables.y[lRun2];

nmpcWorkspace.DyN[0] -= nmpcVariables.yN[0];
nmpcWorkspace.DyN[1] -= nmpcVariables.yN[1];
nmpcWorkspace.DyN[2] -= nmpcVariables.yN[2];
nmpcWorkspace.DyN[3] -= nmpcVariables.yN[3];
nmpcWorkspace.DyN[4] -= nmpcVariables.yN[4];
nmpcWorkspace.DyN[5] -= nmpcVariables.yN[5];

nmpc_multRDy( nmpcWorkspace.R2, nmpcWorkspace.Dy, &(nmpcWorkspace.g[ 12 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 56 ]), &(nmpcWorkspace.Dy[ 14 ]), &(nmpcWorkspace.g[ 16 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 112 ]), &(nmpcWorkspace.Dy[ 28 ]), &(nmpcWorkspace.g[ 20 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 168 ]), &(nmpcWorkspace.Dy[ 42 ]), &(nmpcWorkspace.g[ 24 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 224 ]), &(nmpcWorkspace.Dy[ 56 ]), &(nmpcWorkspace.g[ 28 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 280 ]), &(nmpcWorkspace.Dy[ 70 ]), &(nmpcWorkspace.g[ 32 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 336 ]), &(nmpcWorkspace.Dy[ 84 ]), &(nmpcWorkspace.g[ 36 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 392 ]), &(nmpcWorkspace.Dy[ 98 ]), &(nmpcWorkspace.g[ 40 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 448 ]), &(nmpcWorkspace.Dy[ 112 ]), &(nmpcWorkspace.g[ 44 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 504 ]), &(nmpcWorkspace.Dy[ 126 ]), &(nmpcWorkspace.g[ 48 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 560 ]), &(nmpcWorkspace.Dy[ 140 ]), &(nmpcWorkspace.g[ 52 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 616 ]), &(nmpcWorkspace.Dy[ 154 ]), &(nmpcWorkspace.g[ 56 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 672 ]), &(nmpcWorkspace.Dy[ 168 ]), &(nmpcWorkspace.g[ 60 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 728 ]), &(nmpcWorkspace.Dy[ 182 ]), &(nmpcWorkspace.g[ 64 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 784 ]), &(nmpcWorkspace.Dy[ 196 ]), &(nmpcWorkspace.g[ 68 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 840 ]), &(nmpcWorkspace.Dy[ 210 ]), &(nmpcWorkspace.g[ 72 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 896 ]), &(nmpcWorkspace.Dy[ 224 ]), &(nmpcWorkspace.g[ 76 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 952 ]), &(nmpcWorkspace.Dy[ 238 ]), &(nmpcWorkspace.g[ 80 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 1008 ]), &(nmpcWorkspace.Dy[ 252 ]), &(nmpcWorkspace.g[ 84 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 1064 ]), &(nmpcWorkspace.Dy[ 266 ]), &(nmpcWorkspace.g[ 88 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 1120 ]), &(nmpcWorkspace.Dy[ 280 ]), &(nmpcWorkspace.g[ 92 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 1176 ]), &(nmpcWorkspace.Dy[ 294 ]), &(nmpcWorkspace.g[ 96 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 1232 ]), &(nmpcWorkspace.Dy[ 308 ]), &(nmpcWorkspace.g[ 100 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 1288 ]), &(nmpcWorkspace.Dy[ 322 ]), &(nmpcWorkspace.g[ 104 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 1344 ]), &(nmpcWorkspace.Dy[ 336 ]), &(nmpcWorkspace.g[ 108 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 1400 ]), &(nmpcWorkspace.Dy[ 350 ]), &(nmpcWorkspace.g[ 112 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 1456 ]), &(nmpcWorkspace.Dy[ 364 ]), &(nmpcWorkspace.g[ 116 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 1512 ]), &(nmpcWorkspace.Dy[ 378 ]), &(nmpcWorkspace.g[ 120 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 1568 ]), &(nmpcWorkspace.Dy[ 392 ]), &(nmpcWorkspace.g[ 124 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 1624 ]), &(nmpcWorkspace.Dy[ 406 ]), &(nmpcWorkspace.g[ 128 ]) );

nmpc_multQDy( nmpcWorkspace.Q2, nmpcWorkspace.Dy, nmpcWorkspace.QDy );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 168 ]), &(nmpcWorkspace.Dy[ 14 ]), &(nmpcWorkspace.QDy[ 12 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 336 ]), &(nmpcWorkspace.Dy[ 28 ]), &(nmpcWorkspace.QDy[ 24 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 504 ]), &(nmpcWorkspace.Dy[ 42 ]), &(nmpcWorkspace.QDy[ 36 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 672 ]), &(nmpcWorkspace.Dy[ 56 ]), &(nmpcWorkspace.QDy[ 48 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 840 ]), &(nmpcWorkspace.Dy[ 70 ]), &(nmpcWorkspace.QDy[ 60 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 1008 ]), &(nmpcWorkspace.Dy[ 84 ]), &(nmpcWorkspace.QDy[ 72 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 1176 ]), &(nmpcWorkspace.Dy[ 98 ]), &(nmpcWorkspace.QDy[ 84 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 1344 ]), &(nmpcWorkspace.Dy[ 112 ]), &(nmpcWorkspace.QDy[ 96 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 1512 ]), &(nmpcWorkspace.Dy[ 126 ]), &(nmpcWorkspace.QDy[ 108 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 1680 ]), &(nmpcWorkspace.Dy[ 140 ]), &(nmpcWorkspace.QDy[ 120 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 1848 ]), &(nmpcWorkspace.Dy[ 154 ]), &(nmpcWorkspace.QDy[ 132 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 2016 ]), &(nmpcWorkspace.Dy[ 168 ]), &(nmpcWorkspace.QDy[ 144 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 2184 ]), &(nmpcWorkspace.Dy[ 182 ]), &(nmpcWorkspace.QDy[ 156 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 2352 ]), &(nmpcWorkspace.Dy[ 196 ]), &(nmpcWorkspace.QDy[ 168 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 2520 ]), &(nmpcWorkspace.Dy[ 210 ]), &(nmpcWorkspace.QDy[ 180 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 2688 ]), &(nmpcWorkspace.Dy[ 224 ]), &(nmpcWorkspace.QDy[ 192 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 2856 ]), &(nmpcWorkspace.Dy[ 238 ]), &(nmpcWorkspace.QDy[ 204 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 3024 ]), &(nmpcWorkspace.Dy[ 252 ]), &(nmpcWorkspace.QDy[ 216 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 3192 ]), &(nmpcWorkspace.Dy[ 266 ]), &(nmpcWorkspace.QDy[ 228 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 3360 ]), &(nmpcWorkspace.Dy[ 280 ]), &(nmpcWorkspace.QDy[ 240 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 3528 ]), &(nmpcWorkspace.Dy[ 294 ]), &(nmpcWorkspace.QDy[ 252 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 3696 ]), &(nmpcWorkspace.Dy[ 308 ]), &(nmpcWorkspace.QDy[ 264 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 3864 ]), &(nmpcWorkspace.Dy[ 322 ]), &(nmpcWorkspace.QDy[ 276 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 4032 ]), &(nmpcWorkspace.Dy[ 336 ]), &(nmpcWorkspace.QDy[ 288 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 4200 ]), &(nmpcWorkspace.Dy[ 350 ]), &(nmpcWorkspace.QDy[ 300 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 4368 ]), &(nmpcWorkspace.Dy[ 364 ]), &(nmpcWorkspace.QDy[ 312 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 4536 ]), &(nmpcWorkspace.Dy[ 378 ]), &(nmpcWorkspace.QDy[ 324 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 4704 ]), &(nmpcWorkspace.Dy[ 392 ]), &(nmpcWorkspace.QDy[ 336 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 4872 ]), &(nmpcWorkspace.Dy[ 406 ]), &(nmpcWorkspace.QDy[ 348 ]) );

nmpcWorkspace.QDy[360] = + nmpcWorkspace.QN2[0]*nmpcWorkspace.DyN[0] + nmpcWorkspace.QN2[1]*nmpcWorkspace.DyN[1] + nmpcWorkspace.QN2[2]*nmpcWorkspace.DyN[2] + nmpcWorkspace.QN2[3]*nmpcWorkspace.DyN[3] + nmpcWorkspace.QN2[4]*nmpcWorkspace.DyN[4] + nmpcWorkspace.QN2[5]*nmpcWorkspace.DyN[5];
nmpcWorkspace.QDy[361] = + nmpcWorkspace.QN2[6]*nmpcWorkspace.DyN[0] + nmpcWorkspace.QN2[7]*nmpcWorkspace.DyN[1] + nmpcWorkspace.QN2[8]*nmpcWorkspace.DyN[2] + nmpcWorkspace.QN2[9]*nmpcWorkspace.DyN[3] + nmpcWorkspace.QN2[10]*nmpcWorkspace.DyN[4] + nmpcWorkspace.QN2[11]*nmpcWorkspace.DyN[5];
nmpcWorkspace.QDy[362] = + nmpcWorkspace.QN2[12]*nmpcWorkspace.DyN[0] + nmpcWorkspace.QN2[13]*nmpcWorkspace.DyN[1] + nmpcWorkspace.QN2[14]*nmpcWorkspace.DyN[2] + nmpcWorkspace.QN2[15]*nmpcWorkspace.DyN[3] + nmpcWorkspace.QN2[16]*nmpcWorkspace.DyN[4] + nmpcWorkspace.QN2[17]*nmpcWorkspace.DyN[5];
nmpcWorkspace.QDy[363] = + nmpcWorkspace.QN2[18]*nmpcWorkspace.DyN[0] + nmpcWorkspace.QN2[19]*nmpcWorkspace.DyN[1] + nmpcWorkspace.QN2[20]*nmpcWorkspace.DyN[2] + nmpcWorkspace.QN2[21]*nmpcWorkspace.DyN[3] + nmpcWorkspace.QN2[22]*nmpcWorkspace.DyN[4] + nmpcWorkspace.QN2[23]*nmpcWorkspace.DyN[5];
nmpcWorkspace.QDy[364] = + nmpcWorkspace.QN2[24]*nmpcWorkspace.DyN[0] + nmpcWorkspace.QN2[25]*nmpcWorkspace.DyN[1] + nmpcWorkspace.QN2[26]*nmpcWorkspace.DyN[2] + nmpcWorkspace.QN2[27]*nmpcWorkspace.DyN[3] + nmpcWorkspace.QN2[28]*nmpcWorkspace.DyN[4] + nmpcWorkspace.QN2[29]*nmpcWorkspace.DyN[5];
nmpcWorkspace.QDy[365] = + nmpcWorkspace.QN2[30]*nmpcWorkspace.DyN[0] + nmpcWorkspace.QN2[31]*nmpcWorkspace.DyN[1] + nmpcWorkspace.QN2[32]*nmpcWorkspace.DyN[2] + nmpcWorkspace.QN2[33]*nmpcWorkspace.DyN[3] + nmpcWorkspace.QN2[34]*nmpcWorkspace.DyN[4] + nmpcWorkspace.QN2[35]*nmpcWorkspace.DyN[5];
nmpcWorkspace.QDy[366] = + nmpcWorkspace.QN2[36]*nmpcWorkspace.DyN[0] + nmpcWorkspace.QN2[37]*nmpcWorkspace.DyN[1] + nmpcWorkspace.QN2[38]*nmpcWorkspace.DyN[2] + nmpcWorkspace.QN2[39]*nmpcWorkspace.DyN[3] + nmpcWorkspace.QN2[40]*nmpcWorkspace.DyN[4] + nmpcWorkspace.QN2[41]*nmpcWorkspace.DyN[5];
nmpcWorkspace.QDy[367] = + nmpcWorkspace.QN2[42]*nmpcWorkspace.DyN[0] + nmpcWorkspace.QN2[43]*nmpcWorkspace.DyN[1] + nmpcWorkspace.QN2[44]*nmpcWorkspace.DyN[2] + nmpcWorkspace.QN2[45]*nmpcWorkspace.DyN[3] + nmpcWorkspace.QN2[46]*nmpcWorkspace.DyN[4] + nmpcWorkspace.QN2[47]*nmpcWorkspace.DyN[5];
nmpcWorkspace.QDy[368] = + nmpcWorkspace.QN2[48]*nmpcWorkspace.DyN[0] + nmpcWorkspace.QN2[49]*nmpcWorkspace.DyN[1] + nmpcWorkspace.QN2[50]*nmpcWorkspace.DyN[2] + nmpcWorkspace.QN2[51]*nmpcWorkspace.DyN[3] + nmpcWorkspace.QN2[52]*nmpcWorkspace.DyN[4] + nmpcWorkspace.QN2[53]*nmpcWorkspace.DyN[5];
nmpcWorkspace.QDy[369] = + nmpcWorkspace.QN2[54]*nmpcWorkspace.DyN[0] + nmpcWorkspace.QN2[55]*nmpcWorkspace.DyN[1] + nmpcWorkspace.QN2[56]*nmpcWorkspace.DyN[2] + nmpcWorkspace.QN2[57]*nmpcWorkspace.DyN[3] + nmpcWorkspace.QN2[58]*nmpcWorkspace.DyN[4] + nmpcWorkspace.QN2[59]*nmpcWorkspace.DyN[5];
nmpcWorkspace.QDy[370] = + nmpcWorkspace.QN2[60]*nmpcWorkspace.DyN[0] + nmpcWorkspace.QN2[61]*nmpcWorkspace.DyN[1] + nmpcWorkspace.QN2[62]*nmpcWorkspace.DyN[2] + nmpcWorkspace.QN2[63]*nmpcWorkspace.DyN[3] + nmpcWorkspace.QN2[64]*nmpcWorkspace.DyN[4] + nmpcWorkspace.QN2[65]*nmpcWorkspace.DyN[5];
nmpcWorkspace.QDy[371] = + nmpcWorkspace.QN2[66]*nmpcWorkspace.DyN[0] + nmpcWorkspace.QN2[67]*nmpcWorkspace.DyN[1] + nmpcWorkspace.QN2[68]*nmpcWorkspace.DyN[2] + nmpcWorkspace.QN2[69]*nmpcWorkspace.DyN[3] + nmpcWorkspace.QN2[70]*nmpcWorkspace.DyN[4] + nmpcWorkspace.QN2[71]*nmpcWorkspace.DyN[5];

for (lRun2 = 0; lRun2 < 360; ++lRun2)
nmpcWorkspace.QDy[lRun2 + 12] += nmpcWorkspace.Qd[lRun2];


for (lRun2 = 0; lRun2 < 12; ++lRun2)
{
for (lRun4 = 0; lRun4 < 1; ++lRun4)
{
real_t t = 0.0;
for (lRun5 = 0; lRun5 < 360; ++lRun5)
{
t += + nmpcWorkspace.evGx[(lRun5 * 12) + (lRun2)]*nmpcWorkspace.QDy[(lRun5 + 12) + (lRun4)];
}
nmpcWorkspace.g[(lRun2) + (lRun4)] = t;
}
}


for (lRun1 = 0; lRun1 < 30; ++lRun1)
{
for (lRun2 = lRun1; lRun2 < 30; ++lRun2)
{
lRun3 = (((lRun2 + 1) * (lRun2)) / (2)) + (lRun1);
nmpc_multEQDy( &(nmpcWorkspace.E[ lRun3 * 48 ]), &(nmpcWorkspace.QDy[ lRun2 * 12 + 12 ]), &(nmpcWorkspace.g[ lRun1 * 4 + 12 ]) );
}
}

nmpcWorkspace.lb[0] = nmpcWorkspace.Dx0[0];
nmpcWorkspace.lb[1] = nmpcWorkspace.Dx0[1];
nmpcWorkspace.lb[2] = nmpcWorkspace.Dx0[2];
nmpcWorkspace.lb[3] = nmpcWorkspace.Dx0[3];
nmpcWorkspace.lb[4] = nmpcWorkspace.Dx0[4];
nmpcWorkspace.lb[5] = nmpcWorkspace.Dx0[5];
nmpcWorkspace.lb[6] = nmpcWorkspace.Dx0[6];
nmpcWorkspace.lb[7] = nmpcWorkspace.Dx0[7];
nmpcWorkspace.lb[8] = nmpcWorkspace.Dx0[8];
nmpcWorkspace.lb[9] = nmpcWorkspace.Dx0[9];
nmpcWorkspace.lb[10] = nmpcWorkspace.Dx0[10];
nmpcWorkspace.lb[11] = nmpcWorkspace.Dx0[11];
nmpcWorkspace.ub[0] = nmpcWorkspace.Dx0[0];
nmpcWorkspace.ub[1] = nmpcWorkspace.Dx0[1];
nmpcWorkspace.ub[2] = nmpcWorkspace.Dx0[2];
nmpcWorkspace.ub[3] = nmpcWorkspace.Dx0[3];
nmpcWorkspace.ub[4] = nmpcWorkspace.Dx0[4];
nmpcWorkspace.ub[5] = nmpcWorkspace.Dx0[5];
nmpcWorkspace.ub[6] = nmpcWorkspace.Dx0[6];
nmpcWorkspace.ub[7] = nmpcWorkspace.Dx0[7];
nmpcWorkspace.ub[8] = nmpcWorkspace.Dx0[8];
nmpcWorkspace.ub[9] = nmpcWorkspace.Dx0[9];
nmpcWorkspace.ub[10] = nmpcWorkspace.Dx0[10];
nmpcWorkspace.ub[11] = nmpcWorkspace.Dx0[11];
}

void nmpc_expand(  )
{
int lRun1;
int lRun2;
int lRun3;
nmpcVariables.x[0] += nmpcWorkspace.x[0];
nmpcVariables.x[1] += nmpcWorkspace.x[1];
nmpcVariables.x[2] += nmpcWorkspace.x[2];
nmpcVariables.x[3] += nmpcWorkspace.x[3];
nmpcVariables.x[4] += nmpcWorkspace.x[4];
nmpcVariables.x[5] += nmpcWorkspace.x[5];
nmpcVariables.x[6] += nmpcWorkspace.x[6];
nmpcVariables.x[7] += nmpcWorkspace.x[7];
nmpcVariables.x[8] += nmpcWorkspace.x[8];
nmpcVariables.x[9] += nmpcWorkspace.x[9];
nmpcVariables.x[10] += nmpcWorkspace.x[10];
nmpcVariables.x[11] += nmpcWorkspace.x[11];

nmpcVariables.u[0] += nmpcWorkspace.x[12];
nmpcVariables.u[1] += nmpcWorkspace.x[13];
nmpcVariables.u[2] += nmpcWorkspace.x[14];
nmpcVariables.u[3] += nmpcWorkspace.x[15];
nmpcVariables.u[4] += nmpcWorkspace.x[16];
nmpcVariables.u[5] += nmpcWorkspace.x[17];
nmpcVariables.u[6] += nmpcWorkspace.x[18];
nmpcVariables.u[7] += nmpcWorkspace.x[19];
nmpcVariables.u[8] += nmpcWorkspace.x[20];
nmpcVariables.u[9] += nmpcWorkspace.x[21];
nmpcVariables.u[10] += nmpcWorkspace.x[22];
nmpcVariables.u[11] += nmpcWorkspace.x[23];
nmpcVariables.u[12] += nmpcWorkspace.x[24];
nmpcVariables.u[13] += nmpcWorkspace.x[25];
nmpcVariables.u[14] += nmpcWorkspace.x[26];
nmpcVariables.u[15] += nmpcWorkspace.x[27];
nmpcVariables.u[16] += nmpcWorkspace.x[28];
nmpcVariables.u[17] += nmpcWorkspace.x[29];
nmpcVariables.u[18] += nmpcWorkspace.x[30];
nmpcVariables.u[19] += nmpcWorkspace.x[31];
nmpcVariables.u[20] += nmpcWorkspace.x[32];
nmpcVariables.u[21] += nmpcWorkspace.x[33];
nmpcVariables.u[22] += nmpcWorkspace.x[34];
nmpcVariables.u[23] += nmpcWorkspace.x[35];
nmpcVariables.u[24] += nmpcWorkspace.x[36];
nmpcVariables.u[25] += nmpcWorkspace.x[37];
nmpcVariables.u[26] += nmpcWorkspace.x[38];
nmpcVariables.u[27] += nmpcWorkspace.x[39];
nmpcVariables.u[28] += nmpcWorkspace.x[40];
nmpcVariables.u[29] += nmpcWorkspace.x[41];
nmpcVariables.u[30] += nmpcWorkspace.x[42];
nmpcVariables.u[31] += nmpcWorkspace.x[43];
nmpcVariables.u[32] += nmpcWorkspace.x[44];
nmpcVariables.u[33] += nmpcWorkspace.x[45];
nmpcVariables.u[34] += nmpcWorkspace.x[46];
nmpcVariables.u[35] += nmpcWorkspace.x[47];
nmpcVariables.u[36] += nmpcWorkspace.x[48];
nmpcVariables.u[37] += nmpcWorkspace.x[49];
nmpcVariables.u[38] += nmpcWorkspace.x[50];
nmpcVariables.u[39] += nmpcWorkspace.x[51];
nmpcVariables.u[40] += nmpcWorkspace.x[52];
nmpcVariables.u[41] += nmpcWorkspace.x[53];
nmpcVariables.u[42] += nmpcWorkspace.x[54];
nmpcVariables.u[43] += nmpcWorkspace.x[55];
nmpcVariables.u[44] += nmpcWorkspace.x[56];
nmpcVariables.u[45] += nmpcWorkspace.x[57];
nmpcVariables.u[46] += nmpcWorkspace.x[58];
nmpcVariables.u[47] += nmpcWorkspace.x[59];
nmpcVariables.u[48] += nmpcWorkspace.x[60];
nmpcVariables.u[49] += nmpcWorkspace.x[61];
nmpcVariables.u[50] += nmpcWorkspace.x[62];
nmpcVariables.u[51] += nmpcWorkspace.x[63];
nmpcVariables.u[52] += nmpcWorkspace.x[64];
nmpcVariables.u[53] += nmpcWorkspace.x[65];
nmpcVariables.u[54] += nmpcWorkspace.x[66];
nmpcVariables.u[55] += nmpcWorkspace.x[67];
nmpcVariables.u[56] += nmpcWorkspace.x[68];
nmpcVariables.u[57] += nmpcWorkspace.x[69];
nmpcVariables.u[58] += nmpcWorkspace.x[70];
nmpcVariables.u[59] += nmpcWorkspace.x[71];
nmpcVariables.u[60] += nmpcWorkspace.x[72];
nmpcVariables.u[61] += nmpcWorkspace.x[73];
nmpcVariables.u[62] += nmpcWorkspace.x[74];
nmpcVariables.u[63] += nmpcWorkspace.x[75];
nmpcVariables.u[64] += nmpcWorkspace.x[76];
nmpcVariables.u[65] += nmpcWorkspace.x[77];
nmpcVariables.u[66] += nmpcWorkspace.x[78];
nmpcVariables.u[67] += nmpcWorkspace.x[79];
nmpcVariables.u[68] += nmpcWorkspace.x[80];
nmpcVariables.u[69] += nmpcWorkspace.x[81];
nmpcVariables.u[70] += nmpcWorkspace.x[82];
nmpcVariables.u[71] += nmpcWorkspace.x[83];
nmpcVariables.u[72] += nmpcWorkspace.x[84];
nmpcVariables.u[73] += nmpcWorkspace.x[85];
nmpcVariables.u[74] += nmpcWorkspace.x[86];
nmpcVariables.u[75] += nmpcWorkspace.x[87];
nmpcVariables.u[76] += nmpcWorkspace.x[88];
nmpcVariables.u[77] += nmpcWorkspace.x[89];
nmpcVariables.u[78] += nmpcWorkspace.x[90];
nmpcVariables.u[79] += nmpcWorkspace.x[91];
nmpcVariables.u[80] += nmpcWorkspace.x[92];
nmpcVariables.u[81] += nmpcWorkspace.x[93];
nmpcVariables.u[82] += nmpcWorkspace.x[94];
nmpcVariables.u[83] += nmpcWorkspace.x[95];
nmpcVariables.u[84] += nmpcWorkspace.x[96];
nmpcVariables.u[85] += nmpcWorkspace.x[97];
nmpcVariables.u[86] += nmpcWorkspace.x[98];
nmpcVariables.u[87] += nmpcWorkspace.x[99];
nmpcVariables.u[88] += nmpcWorkspace.x[100];
nmpcVariables.u[89] += nmpcWorkspace.x[101];
nmpcVariables.u[90] += nmpcWorkspace.x[102];
nmpcVariables.u[91] += nmpcWorkspace.x[103];
nmpcVariables.u[92] += nmpcWorkspace.x[104];
nmpcVariables.u[93] += nmpcWorkspace.x[105];
nmpcVariables.u[94] += nmpcWorkspace.x[106];
nmpcVariables.u[95] += nmpcWorkspace.x[107];
nmpcVariables.u[96] += nmpcWorkspace.x[108];
nmpcVariables.u[97] += nmpcWorkspace.x[109];
nmpcVariables.u[98] += nmpcWorkspace.x[110];
nmpcVariables.u[99] += nmpcWorkspace.x[111];
nmpcVariables.u[100] += nmpcWorkspace.x[112];
nmpcVariables.u[101] += nmpcWorkspace.x[113];
nmpcVariables.u[102] += nmpcWorkspace.x[114];
nmpcVariables.u[103] += nmpcWorkspace.x[115];
nmpcVariables.u[104] += nmpcWorkspace.x[116];
nmpcVariables.u[105] += nmpcWorkspace.x[117];
nmpcVariables.u[106] += nmpcWorkspace.x[118];
nmpcVariables.u[107] += nmpcWorkspace.x[119];
nmpcVariables.u[108] += nmpcWorkspace.x[120];
nmpcVariables.u[109] += nmpcWorkspace.x[121];
nmpcVariables.u[110] += nmpcWorkspace.x[122];
nmpcVariables.u[111] += nmpcWorkspace.x[123];
nmpcVariables.u[112] += nmpcWorkspace.x[124];
nmpcVariables.u[113] += nmpcWorkspace.x[125];
nmpcVariables.u[114] += nmpcWorkspace.x[126];
nmpcVariables.u[115] += nmpcWorkspace.x[127];
nmpcVariables.u[116] += nmpcWorkspace.x[128];
nmpcVariables.u[117] += nmpcWorkspace.x[129];
nmpcVariables.u[118] += nmpcWorkspace.x[130];
nmpcVariables.u[119] += nmpcWorkspace.x[131];

for (lRun1 = 0; lRun1 < 360; ++lRun1)
{
for (lRun2 = 0; lRun2 < 1; ++lRun2)
{
real_t t = 0.0;
for (lRun3 = 0; lRun3 < 12; ++lRun3)
{
t += + nmpcWorkspace.evGx[(lRun1 * 12) + (lRun3)]*nmpcWorkspace.x[(lRun3) + (lRun2)];
}
nmpcVariables.x[(lRun1 + 12) + (lRun2)] += t + nmpcWorkspace.d[(lRun1) + (lRun2)];
}
}

for (lRun1 = 0; lRun1 < 30; ++lRun1)
{
for (lRun2 = 0; lRun2 < lRun1 + 1; ++lRun2)
{
lRun3 = (((lRun1 + 1) * (lRun1)) / (2)) + (lRun2);
nmpc_multEDu( &(nmpcWorkspace.E[ lRun3 * 48 ]), &(nmpcWorkspace.x[ lRun2 * 4 + 12 ]), &(nmpcVariables.x[ lRun1 * 12 + 12 ]) );
}
}
}

int nmpc_preparationStep(  )
{
int ret;

ret = nmpc_modelSimulation();
nmpc_evaluateObjective(  );
nmpc_condensePrep(  );
return ret;
}

int nmpc_feedbackStep(  )
{
int tmp;

nmpc_condenseFdb(  );

tmp = nmpc_solve( );

nmpc_expand(  );
return tmp;
}

int nmpc_initializeSolver(  )
{
int ret;

/* This is a function which must be called once before any other function call! */


ret = 0;

memset(&nmpcWorkspace, 0, sizeof( nmpcWorkspace ));
return ret;
}

void nmpc_initializeNodesByForwardSimulation(  )
{
int index;
for (index = 0; index < 30; ++index)
{
nmpcWorkspace.state[0] = nmpcVariables.x[index * 12];
nmpcWorkspace.state[1] = nmpcVariables.x[index * 12 + 1];
nmpcWorkspace.state[2] = nmpcVariables.x[index * 12 + 2];
nmpcWorkspace.state[3] = nmpcVariables.x[index * 12 + 3];
nmpcWorkspace.state[4] = nmpcVariables.x[index * 12 + 4];
nmpcWorkspace.state[5] = nmpcVariables.x[index * 12 + 5];
nmpcWorkspace.state[6] = nmpcVariables.x[index * 12 + 6];
nmpcWorkspace.state[7] = nmpcVariables.x[index * 12 + 7];
nmpcWorkspace.state[8] = nmpcVariables.x[index * 12 + 8];
nmpcWorkspace.state[9] = nmpcVariables.x[index * 12 + 9];
nmpcWorkspace.state[10] = nmpcVariables.x[index * 12 + 10];
nmpcWorkspace.state[11] = nmpcVariables.x[index * 12 + 11];
nmpcWorkspace.state[204] = nmpcVariables.u[index * 4];
nmpcWorkspace.state[205] = nmpcVariables.u[index * 4 + 1];
nmpcWorkspace.state[206] = nmpcVariables.u[index * 4 + 2];
nmpcWorkspace.state[207] = nmpcVariables.u[index * 4 + 3];
nmpcWorkspace.state[208] = nmpcVariables.od[index * 12];
nmpcWorkspace.state[209] = nmpcVariables.od[index * 12 + 1];
nmpcWorkspace.state[210] = nmpcVariables.od[index * 12 + 2];
nmpcWorkspace.state[211] = nmpcVariables.od[index * 12 + 3];
nmpcWorkspace.state[212] = nmpcVariables.od[index * 12 + 4];
nmpcWorkspace.state[213] = nmpcVariables.od[index * 12 + 5];
nmpcWorkspace.state[214] = nmpcVariables.od[index * 12 + 6];
nmpcWorkspace.state[215] = nmpcVariables.od[index * 12 + 7];
nmpcWorkspace.state[216] = nmpcVariables.od[index * 12 + 8];
nmpcWorkspace.state[217] = nmpcVariables.od[index * 12 + 9];
nmpcWorkspace.state[218] = nmpcVariables.od[index * 12 + 10];
nmpcWorkspace.state[219] = nmpcVariables.od[index * 12 + 11];

nmpc_integrate(nmpcWorkspace.state, index == 0);

nmpcVariables.x[index * 12 + 12] = nmpcWorkspace.state[0];
nmpcVariables.x[index * 12 + 13] = nmpcWorkspace.state[1];
nmpcVariables.x[index * 12 + 14] = nmpcWorkspace.state[2];
nmpcVariables.x[index * 12 + 15] = nmpcWorkspace.state[3];
nmpcVariables.x[index * 12 + 16] = nmpcWorkspace.state[4];
nmpcVariables.x[index * 12 + 17] = nmpcWorkspace.state[5];
nmpcVariables.x[index * 12 + 18] = nmpcWorkspace.state[6];
nmpcVariables.x[index * 12 + 19] = nmpcWorkspace.state[7];
nmpcVariables.x[index * 12 + 20] = nmpcWorkspace.state[8];
nmpcVariables.x[index * 12 + 21] = nmpcWorkspace.state[9];
nmpcVariables.x[index * 12 + 22] = nmpcWorkspace.state[10];
nmpcVariables.x[index * 12 + 23] = nmpcWorkspace.state[11];
}
}

void nmpc_shiftStates( int strategy, real_t* const xEnd, real_t* const uEnd )
{
int index;
for (index = 0; index < 30; ++index)
{
nmpcVariables.x[index * 12] = nmpcVariables.x[index * 12 + 12];
nmpcVariables.x[index * 12 + 1] = nmpcVariables.x[index * 12 + 13];
nmpcVariables.x[index * 12 + 2] = nmpcVariables.x[index * 12 + 14];
nmpcVariables.x[index * 12 + 3] = nmpcVariables.x[index * 12 + 15];
nmpcVariables.x[index * 12 + 4] = nmpcVariables.x[index * 12 + 16];
nmpcVariables.x[index * 12 + 5] = nmpcVariables.x[index * 12 + 17];
nmpcVariables.x[index * 12 + 6] = nmpcVariables.x[index * 12 + 18];
nmpcVariables.x[index * 12 + 7] = nmpcVariables.x[index * 12 + 19];
nmpcVariables.x[index * 12 + 8] = nmpcVariables.x[index * 12 + 20];
nmpcVariables.x[index * 12 + 9] = nmpcVariables.x[index * 12 + 21];
nmpcVariables.x[index * 12 + 10] = nmpcVariables.x[index * 12 + 22];
nmpcVariables.x[index * 12 + 11] = nmpcVariables.x[index * 12 + 23];
}

if (strategy == 1 && xEnd != 0)
{
nmpcVariables.x[360] = xEnd[0];
nmpcVariables.x[361] = xEnd[1];
nmpcVariables.x[362] = xEnd[2];
nmpcVariables.x[363] = xEnd[3];
nmpcVariables.x[364] = xEnd[4];
nmpcVariables.x[365] = xEnd[5];
nmpcVariables.x[366] = xEnd[6];
nmpcVariables.x[367] = xEnd[7];
nmpcVariables.x[368] = xEnd[8];
nmpcVariables.x[369] = xEnd[9];
nmpcVariables.x[370] = xEnd[10];
nmpcVariables.x[371] = xEnd[11];
}
else if (strategy == 2) 
{
nmpcWorkspace.state[0] = nmpcVariables.x[360];
nmpcWorkspace.state[1] = nmpcVariables.x[361];
nmpcWorkspace.state[2] = nmpcVariables.x[362];
nmpcWorkspace.state[3] = nmpcVariables.x[363];
nmpcWorkspace.state[4] = nmpcVariables.x[364];
nmpcWorkspace.state[5] = nmpcVariables.x[365];
nmpcWorkspace.state[6] = nmpcVariables.x[366];
nmpcWorkspace.state[7] = nmpcVariables.x[367];
nmpcWorkspace.state[8] = nmpcVariables.x[368];
nmpcWorkspace.state[9] = nmpcVariables.x[369];
nmpcWorkspace.state[10] = nmpcVariables.x[370];
nmpcWorkspace.state[11] = nmpcVariables.x[371];
if (uEnd != 0)
{
nmpcWorkspace.state[204] = uEnd[0];
nmpcWorkspace.state[205] = uEnd[1];
nmpcWorkspace.state[206] = uEnd[2];
nmpcWorkspace.state[207] = uEnd[3];
}
else
{
nmpcWorkspace.state[204] = nmpcVariables.u[116];
nmpcWorkspace.state[205] = nmpcVariables.u[117];
nmpcWorkspace.state[206] = nmpcVariables.u[118];
nmpcWorkspace.state[207] = nmpcVariables.u[119];
}
nmpcWorkspace.state[208] = nmpcVariables.od[360];
nmpcWorkspace.state[209] = nmpcVariables.od[361];
nmpcWorkspace.state[210] = nmpcVariables.od[362];
nmpcWorkspace.state[211] = nmpcVariables.od[363];
nmpcWorkspace.state[212] = nmpcVariables.od[364];
nmpcWorkspace.state[213] = nmpcVariables.od[365];
nmpcWorkspace.state[214] = nmpcVariables.od[366];
nmpcWorkspace.state[215] = nmpcVariables.od[367];
nmpcWorkspace.state[216] = nmpcVariables.od[368];
nmpcWorkspace.state[217] = nmpcVariables.od[369];
nmpcWorkspace.state[218] = nmpcVariables.od[370];
nmpcWorkspace.state[219] = nmpcVariables.od[371];

nmpc_integrate(nmpcWorkspace.state, 1);

nmpcVariables.x[360] = nmpcWorkspace.state[0];
nmpcVariables.x[361] = nmpcWorkspace.state[1];
nmpcVariables.x[362] = nmpcWorkspace.state[2];
nmpcVariables.x[363] = nmpcWorkspace.state[3];
nmpcVariables.x[364] = nmpcWorkspace.state[4];
nmpcVariables.x[365] = nmpcWorkspace.state[5];
nmpcVariables.x[366] = nmpcWorkspace.state[6];
nmpcVariables.x[367] = nmpcWorkspace.state[7];
nmpcVariables.x[368] = nmpcWorkspace.state[8];
nmpcVariables.x[369] = nmpcWorkspace.state[9];
nmpcVariables.x[370] = nmpcWorkspace.state[10];
nmpcVariables.x[371] = nmpcWorkspace.state[11];
}
}

void nmpc_shiftControls( real_t* const uEnd )
{
int index;
for (index = 0; index < 29; ++index)
{
nmpcVariables.u[index * 4] = nmpcVariables.u[index * 4 + 4];
nmpcVariables.u[index * 4 + 1] = nmpcVariables.u[index * 4 + 5];
nmpcVariables.u[index * 4 + 2] = nmpcVariables.u[index * 4 + 6];
nmpcVariables.u[index * 4 + 3] = nmpcVariables.u[index * 4 + 7];
}

if (uEnd != 0)
{
nmpcVariables.u[116] = uEnd[0];
nmpcVariables.u[117] = uEnd[1];
nmpcVariables.u[118] = uEnd[2];
nmpcVariables.u[119] = uEnd[3];
}
}

real_t nmpc_getKKT(  )
{
real_t kkt;

int index;
real_t prd;

kkt = + nmpcWorkspace.g[0]*nmpcWorkspace.x[0] + nmpcWorkspace.g[1]*nmpcWorkspace.x[1] + nmpcWorkspace.g[2]*nmpcWorkspace.x[2] + nmpcWorkspace.g[3]*nmpcWorkspace.x[3] + nmpcWorkspace.g[4]*nmpcWorkspace.x[4] + nmpcWorkspace.g[5]*nmpcWorkspace.x[5] + nmpcWorkspace.g[6]*nmpcWorkspace.x[6] + nmpcWorkspace.g[7]*nmpcWorkspace.x[7] + nmpcWorkspace.g[8]*nmpcWorkspace.x[8] + nmpcWorkspace.g[9]*nmpcWorkspace.x[9] + nmpcWorkspace.g[10]*nmpcWorkspace.x[10] + nmpcWorkspace.g[11]*nmpcWorkspace.x[11] + nmpcWorkspace.g[12]*nmpcWorkspace.x[12] + nmpcWorkspace.g[13]*nmpcWorkspace.x[13] + nmpcWorkspace.g[14]*nmpcWorkspace.x[14] + nmpcWorkspace.g[15]*nmpcWorkspace.x[15] + nmpcWorkspace.g[16]*nmpcWorkspace.x[16] + nmpcWorkspace.g[17]*nmpcWorkspace.x[17] + nmpcWorkspace.g[18]*nmpcWorkspace.x[18] + nmpcWorkspace.g[19]*nmpcWorkspace.x[19] + nmpcWorkspace.g[20]*nmpcWorkspace.x[20] + nmpcWorkspace.g[21]*nmpcWorkspace.x[21] + nmpcWorkspace.g[22]*nmpcWorkspace.x[22] + nmpcWorkspace.g[23]*nmpcWorkspace.x[23] + nmpcWorkspace.g[24]*nmpcWorkspace.x[24] + nmpcWorkspace.g[25]*nmpcWorkspace.x[25] + nmpcWorkspace.g[26]*nmpcWorkspace.x[26] + nmpcWorkspace.g[27]*nmpcWorkspace.x[27] + nmpcWorkspace.g[28]*nmpcWorkspace.x[28] + nmpcWorkspace.g[29]*nmpcWorkspace.x[29] + nmpcWorkspace.g[30]*nmpcWorkspace.x[30] + nmpcWorkspace.g[31]*nmpcWorkspace.x[31] + nmpcWorkspace.g[32]*nmpcWorkspace.x[32] + nmpcWorkspace.g[33]*nmpcWorkspace.x[33] + nmpcWorkspace.g[34]*nmpcWorkspace.x[34] + nmpcWorkspace.g[35]*nmpcWorkspace.x[35] + nmpcWorkspace.g[36]*nmpcWorkspace.x[36] + nmpcWorkspace.g[37]*nmpcWorkspace.x[37] + nmpcWorkspace.g[38]*nmpcWorkspace.x[38] + nmpcWorkspace.g[39]*nmpcWorkspace.x[39] + nmpcWorkspace.g[40]*nmpcWorkspace.x[40] + nmpcWorkspace.g[41]*nmpcWorkspace.x[41] + nmpcWorkspace.g[42]*nmpcWorkspace.x[42] + nmpcWorkspace.g[43]*nmpcWorkspace.x[43] + nmpcWorkspace.g[44]*nmpcWorkspace.x[44] + nmpcWorkspace.g[45]*nmpcWorkspace.x[45] + nmpcWorkspace.g[46]*nmpcWorkspace.x[46] + nmpcWorkspace.g[47]*nmpcWorkspace.x[47] + nmpcWorkspace.g[48]*nmpcWorkspace.x[48] + nmpcWorkspace.g[49]*nmpcWorkspace.x[49] + nmpcWorkspace.g[50]*nmpcWorkspace.x[50] + nmpcWorkspace.g[51]*nmpcWorkspace.x[51] + nmpcWorkspace.g[52]*nmpcWorkspace.x[52] + nmpcWorkspace.g[53]*nmpcWorkspace.x[53] + nmpcWorkspace.g[54]*nmpcWorkspace.x[54] + nmpcWorkspace.g[55]*nmpcWorkspace.x[55] + nmpcWorkspace.g[56]*nmpcWorkspace.x[56] + nmpcWorkspace.g[57]*nmpcWorkspace.x[57] + nmpcWorkspace.g[58]*nmpcWorkspace.x[58] + nmpcWorkspace.g[59]*nmpcWorkspace.x[59] + nmpcWorkspace.g[60]*nmpcWorkspace.x[60] + nmpcWorkspace.g[61]*nmpcWorkspace.x[61] + nmpcWorkspace.g[62]*nmpcWorkspace.x[62] + nmpcWorkspace.g[63]*nmpcWorkspace.x[63] + nmpcWorkspace.g[64]*nmpcWorkspace.x[64] + nmpcWorkspace.g[65]*nmpcWorkspace.x[65] + nmpcWorkspace.g[66]*nmpcWorkspace.x[66] + nmpcWorkspace.g[67]*nmpcWorkspace.x[67] + nmpcWorkspace.g[68]*nmpcWorkspace.x[68] + nmpcWorkspace.g[69]*nmpcWorkspace.x[69] + nmpcWorkspace.g[70]*nmpcWorkspace.x[70] + nmpcWorkspace.g[71]*nmpcWorkspace.x[71] + nmpcWorkspace.g[72]*nmpcWorkspace.x[72] + nmpcWorkspace.g[73]*nmpcWorkspace.x[73] + nmpcWorkspace.g[74]*nmpcWorkspace.x[74] + nmpcWorkspace.g[75]*nmpcWorkspace.x[75] + nmpcWorkspace.g[76]*nmpcWorkspace.x[76] + nmpcWorkspace.g[77]*nmpcWorkspace.x[77] + nmpcWorkspace.g[78]*nmpcWorkspace.x[78] + nmpcWorkspace.g[79]*nmpcWorkspace.x[79] + nmpcWorkspace.g[80]*nmpcWorkspace.x[80] + nmpcWorkspace.g[81]*nmpcWorkspace.x[81] + nmpcWorkspace.g[82]*nmpcWorkspace.x[82] + nmpcWorkspace.g[83]*nmpcWorkspace.x[83] + nmpcWorkspace.g[84]*nmpcWorkspace.x[84] + nmpcWorkspace.g[85]*nmpcWorkspace.x[85] + nmpcWorkspace.g[86]*nmpcWorkspace.x[86] + nmpcWorkspace.g[87]*nmpcWorkspace.x[87] + nmpcWorkspace.g[88]*nmpcWorkspace.x[88] + nmpcWorkspace.g[89]*nmpcWorkspace.x[89] + nmpcWorkspace.g[90]*nmpcWorkspace.x[90] + nmpcWorkspace.g[91]*nmpcWorkspace.x[91] + nmpcWorkspace.g[92]*nmpcWorkspace.x[92] + nmpcWorkspace.g[93]*nmpcWorkspace.x[93] + nmpcWorkspace.g[94]*nmpcWorkspace.x[94] + nmpcWorkspace.g[95]*nmpcWorkspace.x[95] + nmpcWorkspace.g[96]*nmpcWorkspace.x[96] + nmpcWorkspace.g[97]*nmpcWorkspace.x[97] + nmpcWorkspace.g[98]*nmpcWorkspace.x[98] + nmpcWorkspace.g[99]*nmpcWorkspace.x[99] + nmpcWorkspace.g[100]*nmpcWorkspace.x[100] + nmpcWorkspace.g[101]*nmpcWorkspace.x[101] + nmpcWorkspace.g[102]*nmpcWorkspace.x[102] + nmpcWorkspace.g[103]*nmpcWorkspace.x[103] + nmpcWorkspace.g[104]*nmpcWorkspace.x[104] + nmpcWorkspace.g[105]*nmpcWorkspace.x[105] + nmpcWorkspace.g[106]*nmpcWorkspace.x[106] + nmpcWorkspace.g[107]*nmpcWorkspace.x[107] + nmpcWorkspace.g[108]*nmpcWorkspace.x[108] + nmpcWorkspace.g[109]*nmpcWorkspace.x[109] + nmpcWorkspace.g[110]*nmpcWorkspace.x[110] + nmpcWorkspace.g[111]*nmpcWorkspace.x[111] + nmpcWorkspace.g[112]*nmpcWorkspace.x[112] + nmpcWorkspace.g[113]*nmpcWorkspace.x[113] + nmpcWorkspace.g[114]*nmpcWorkspace.x[114] + nmpcWorkspace.g[115]*nmpcWorkspace.x[115] + nmpcWorkspace.g[116]*nmpcWorkspace.x[116] + nmpcWorkspace.g[117]*nmpcWorkspace.x[117] + nmpcWorkspace.g[118]*nmpcWorkspace.x[118] + nmpcWorkspace.g[119]*nmpcWorkspace.x[119] + nmpcWorkspace.g[120]*nmpcWorkspace.x[120] + nmpcWorkspace.g[121]*nmpcWorkspace.x[121] + nmpcWorkspace.g[122]*nmpcWorkspace.x[122] + nmpcWorkspace.g[123]*nmpcWorkspace.x[123] + nmpcWorkspace.g[124]*nmpcWorkspace.x[124] + nmpcWorkspace.g[125]*nmpcWorkspace.x[125] + nmpcWorkspace.g[126]*nmpcWorkspace.x[126] + nmpcWorkspace.g[127]*nmpcWorkspace.x[127] + nmpcWorkspace.g[128]*nmpcWorkspace.x[128] + nmpcWorkspace.g[129]*nmpcWorkspace.x[129] + nmpcWorkspace.g[130]*nmpcWorkspace.x[130] + nmpcWorkspace.g[131]*nmpcWorkspace.x[131];
kkt = fabs( kkt );
for (index = 0; index < 132; ++index)
{
prd = nmpcWorkspace.y[index];
if (prd > 1e-12)
kkt += fabs(nmpcWorkspace.lb[index] * prd);
else if (prd < -1e-12)
kkt += fabs(nmpcWorkspace.ub[index] * prd);
}
return kkt;
}

real_t nmpc_getObjective(  )
{
real_t objVal;

int lRun1;
/** Row vector of size: 14 */
real_t tmpDy[ 14 ];

/** Row vector of size: 6 */
real_t tmpDyN[ 6 ];

for (lRun1 = 0; lRun1 < 30; ++lRun1)
{
nmpcWorkspace.objValueIn[0] = nmpcVariables.x[lRun1 * 12];
nmpcWorkspace.objValueIn[1] = nmpcVariables.x[lRun1 * 12 + 1];
nmpcWorkspace.objValueIn[2] = nmpcVariables.x[lRun1 * 12 + 2];
nmpcWorkspace.objValueIn[3] = nmpcVariables.x[lRun1 * 12 + 3];
nmpcWorkspace.objValueIn[4] = nmpcVariables.x[lRun1 * 12 + 4];
nmpcWorkspace.objValueIn[5] = nmpcVariables.x[lRun1 * 12 + 5];
nmpcWorkspace.objValueIn[6] = nmpcVariables.x[lRun1 * 12 + 6];
nmpcWorkspace.objValueIn[7] = nmpcVariables.x[lRun1 * 12 + 7];
nmpcWorkspace.objValueIn[8] = nmpcVariables.x[lRun1 * 12 + 8];
nmpcWorkspace.objValueIn[9] = nmpcVariables.x[lRun1 * 12 + 9];
nmpcWorkspace.objValueIn[10] = nmpcVariables.x[lRun1 * 12 + 10];
nmpcWorkspace.objValueIn[11] = nmpcVariables.x[lRun1 * 12 + 11];
nmpcWorkspace.objValueIn[12] = nmpcVariables.u[lRun1 * 4];
nmpcWorkspace.objValueIn[13] = nmpcVariables.u[lRun1 * 4 + 1];
nmpcWorkspace.objValueIn[14] = nmpcVariables.u[lRun1 * 4 + 2];
nmpcWorkspace.objValueIn[15] = nmpcVariables.u[lRun1 * 4 + 3];
nmpcWorkspace.objValueIn[16] = nmpcVariables.od[lRun1 * 12];
nmpcWorkspace.objValueIn[17] = nmpcVariables.od[lRun1 * 12 + 1];
nmpcWorkspace.objValueIn[18] = nmpcVariables.od[lRun1 * 12 + 2];
nmpcWorkspace.objValueIn[19] = nmpcVariables.od[lRun1 * 12 + 3];
nmpcWorkspace.objValueIn[20] = nmpcVariables.od[lRun1 * 12 + 4];
nmpcWorkspace.objValueIn[21] = nmpcVariables.od[lRun1 * 12 + 5];
nmpcWorkspace.objValueIn[22] = nmpcVariables.od[lRun1 * 12 + 6];
nmpcWorkspace.objValueIn[23] = nmpcVariables.od[lRun1 * 12 + 7];
nmpcWorkspace.objValueIn[24] = nmpcVariables.od[lRun1 * 12 + 8];
nmpcWorkspace.objValueIn[25] = nmpcVariables.od[lRun1 * 12 + 9];
nmpcWorkspace.objValueIn[26] = nmpcVariables.od[lRun1 * 12 + 10];
nmpcWorkspace.objValueIn[27] = nmpcVariables.od[lRun1 * 12 + 11];

nmpc_evaluateLSQ( nmpcWorkspace.objValueIn, nmpcWorkspace.objValueOut );
nmpcWorkspace.Dy[lRun1 * 14] = nmpcWorkspace.objValueOut[0] - nmpcVariables.y[lRun1 * 14];
nmpcWorkspace.Dy[lRun1 * 14 + 1] = nmpcWorkspace.objValueOut[1] - nmpcVariables.y[lRun1 * 14 + 1];
nmpcWorkspace.Dy[lRun1 * 14 + 2] = nmpcWorkspace.objValueOut[2] - nmpcVariables.y[lRun1 * 14 + 2];
nmpcWorkspace.Dy[lRun1 * 14 + 3] = nmpcWorkspace.objValueOut[3] - nmpcVariables.y[lRun1 * 14 + 3];
nmpcWorkspace.Dy[lRun1 * 14 + 4] = nmpcWorkspace.objValueOut[4] - nmpcVariables.y[lRun1 * 14 + 4];
nmpcWorkspace.Dy[lRun1 * 14 + 5] = nmpcWorkspace.objValueOut[5] - nmpcVariables.y[lRun1 * 14 + 5];
nmpcWorkspace.Dy[lRun1 * 14 + 6] = nmpcWorkspace.objValueOut[6] - nmpcVariables.y[lRun1 * 14 + 6];
nmpcWorkspace.Dy[lRun1 * 14 + 7] = nmpcWorkspace.objValueOut[7] - nmpcVariables.y[lRun1 * 14 + 7];
nmpcWorkspace.Dy[lRun1 * 14 + 8] = nmpcWorkspace.objValueOut[8] - nmpcVariables.y[lRun1 * 14 + 8];
nmpcWorkspace.Dy[lRun1 * 14 + 9] = nmpcWorkspace.objValueOut[9] - nmpcVariables.y[lRun1 * 14 + 9];
nmpcWorkspace.Dy[lRun1 * 14 + 10] = nmpcWorkspace.objValueOut[10] - nmpcVariables.y[lRun1 * 14 + 10];
nmpcWorkspace.Dy[lRun1 * 14 + 11] = nmpcWorkspace.objValueOut[11] - nmpcVariables.y[lRun1 * 14 + 11];
nmpcWorkspace.Dy[lRun1 * 14 + 12] = nmpcWorkspace.objValueOut[12] - nmpcVariables.y[lRun1 * 14 + 12];
nmpcWorkspace.Dy[lRun1 * 14 + 13] = nmpcWorkspace.objValueOut[13] - nmpcVariables.y[lRun1 * 14 + 13];
}
nmpcWorkspace.objValueIn[0] = nmpcVariables.x[360];
nmpcWorkspace.objValueIn[1] = nmpcVariables.x[361];
nmpcWorkspace.objValueIn[2] = nmpcVariables.x[362];
nmpcWorkspace.objValueIn[3] = nmpcVariables.x[363];
nmpcWorkspace.objValueIn[4] = nmpcVariables.x[364];
nmpcWorkspace.objValueIn[5] = nmpcVariables.x[365];
nmpcWorkspace.objValueIn[6] = nmpcVariables.x[366];
nmpcWorkspace.objValueIn[7] = nmpcVariables.x[367];
nmpcWorkspace.objValueIn[8] = nmpcVariables.x[368];
nmpcWorkspace.objValueIn[9] = nmpcVariables.x[369];
nmpcWorkspace.objValueIn[10] = nmpcVariables.x[370];
nmpcWorkspace.objValueIn[11] = nmpcVariables.x[371];
nmpcWorkspace.objValueIn[12] = nmpcVariables.od[360];
nmpcWorkspace.objValueIn[13] = nmpcVariables.od[361];
nmpcWorkspace.objValueIn[14] = nmpcVariables.od[362];
nmpcWorkspace.objValueIn[15] = nmpcVariables.od[363];
nmpcWorkspace.objValueIn[16] = nmpcVariables.od[364];
nmpcWorkspace.objValueIn[17] = nmpcVariables.od[365];
nmpcWorkspace.objValueIn[18] = nmpcVariables.od[366];
nmpcWorkspace.objValueIn[19] = nmpcVariables.od[367];
nmpcWorkspace.objValueIn[20] = nmpcVariables.od[368];
nmpcWorkspace.objValueIn[21] = nmpcVariables.od[369];
nmpcWorkspace.objValueIn[22] = nmpcVariables.od[370];
nmpcWorkspace.objValueIn[23] = nmpcVariables.od[371];
nmpc_evaluateLSQEndTerm( nmpcWorkspace.objValueIn, nmpcWorkspace.objValueOut );
nmpcWorkspace.DyN[0] = nmpcWorkspace.objValueOut[0] - nmpcVariables.yN[0];
nmpcWorkspace.DyN[1] = nmpcWorkspace.objValueOut[1] - nmpcVariables.yN[1];
nmpcWorkspace.DyN[2] = nmpcWorkspace.objValueOut[2] - nmpcVariables.yN[2];
nmpcWorkspace.DyN[3] = nmpcWorkspace.objValueOut[3] - nmpcVariables.yN[3];
nmpcWorkspace.DyN[4] = nmpcWorkspace.objValueOut[4] - nmpcVariables.yN[4];
nmpcWorkspace.DyN[5] = nmpcWorkspace.objValueOut[5] - nmpcVariables.yN[5];
objVal = 0.0000000000000000e+00;
for (lRun1 = 0; lRun1 < 30; ++lRun1)
{
tmpDy[0] = + nmpcWorkspace.Dy[lRun1 * 14]*nmpcVariables.W[0];
tmpDy[1] = + nmpcWorkspace.Dy[lRun1 * 14 + 1]*nmpcVariables.W[15];
tmpDy[2] = + nmpcWorkspace.Dy[lRun1 * 14 + 2]*nmpcVariables.W[30];
tmpDy[3] = + nmpcWorkspace.Dy[lRun1 * 14 + 3]*nmpcVariables.W[45];
tmpDy[4] = + nmpcWorkspace.Dy[lRun1 * 14 + 4]*nmpcVariables.W[60];
tmpDy[5] = + nmpcWorkspace.Dy[lRun1 * 14 + 5]*nmpcVariables.W[75];
tmpDy[6] = + nmpcWorkspace.Dy[lRun1 * 14 + 6]*nmpcVariables.W[90];
tmpDy[7] = + nmpcWorkspace.Dy[lRun1 * 14 + 7]*nmpcVariables.W[105];
tmpDy[8] = + nmpcWorkspace.Dy[lRun1 * 14 + 8]*nmpcVariables.W[120];
tmpDy[9] = + nmpcWorkspace.Dy[lRun1 * 14 + 9]*nmpcVariables.W[135];
tmpDy[10] = + nmpcWorkspace.Dy[lRun1 * 14 + 10]*nmpcVariables.W[150];
tmpDy[11] = + nmpcWorkspace.Dy[lRun1 * 14 + 11]*nmpcVariables.W[165];
tmpDy[12] = + nmpcWorkspace.Dy[lRun1 * 14 + 12]*nmpcVariables.W[180];
tmpDy[13] = + nmpcWorkspace.Dy[lRun1 * 14 + 13]*nmpcVariables.W[195];
objVal += + nmpcWorkspace.Dy[lRun1 * 14]*tmpDy[0] + nmpcWorkspace.Dy[lRun1 * 14 + 1]*tmpDy[1] + nmpcWorkspace.Dy[lRun1 * 14 + 2]*tmpDy[2] + nmpcWorkspace.Dy[lRun1 * 14 + 3]*tmpDy[3] + nmpcWorkspace.Dy[lRun1 * 14 + 4]*tmpDy[4] + nmpcWorkspace.Dy[lRun1 * 14 + 5]*tmpDy[5] + nmpcWorkspace.Dy[lRun1 * 14 + 6]*tmpDy[6] + nmpcWorkspace.Dy[lRun1 * 14 + 7]*tmpDy[7] + nmpcWorkspace.Dy[lRun1 * 14 + 8]*tmpDy[8] + nmpcWorkspace.Dy[lRun1 * 14 + 9]*tmpDy[9] + nmpcWorkspace.Dy[lRun1 * 14 + 10]*tmpDy[10] + nmpcWorkspace.Dy[lRun1 * 14 + 11]*tmpDy[11] + nmpcWorkspace.Dy[lRun1 * 14 + 12]*tmpDy[12] + nmpcWorkspace.Dy[lRun1 * 14 + 13]*tmpDy[13];
}

tmpDyN[0] = + nmpcWorkspace.DyN[0]*nmpcVariables.WN[0];
tmpDyN[1] = + nmpcWorkspace.DyN[1]*nmpcVariables.WN[7];
tmpDyN[2] = + nmpcWorkspace.DyN[2]*nmpcVariables.WN[14];
tmpDyN[3] = + nmpcWorkspace.DyN[3]*nmpcVariables.WN[21];
tmpDyN[4] = + nmpcWorkspace.DyN[4]*nmpcVariables.WN[28];
tmpDyN[5] = + nmpcWorkspace.DyN[5]*nmpcVariables.WN[35];
objVal += + nmpcWorkspace.DyN[0]*tmpDyN[0] + nmpcWorkspace.DyN[1]*tmpDyN[1] + nmpcWorkspace.DyN[2]*tmpDyN[2] + nmpcWorkspace.DyN[3]*tmpDyN[3] + nmpcWorkspace.DyN[4]*tmpDyN[4] + nmpcWorkspace.DyN[5]*tmpDyN[5];

objVal *= 0.5;
return objVal;
}

