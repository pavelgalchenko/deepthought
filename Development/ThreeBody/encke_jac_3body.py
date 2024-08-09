#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Jul 31 12:35:26 2024
"""

import sympy as sp
from sympy.utilities.autowrap import autowrap

def EnckeFq(r, delta):
    q = (delta[0] * (delta[0] - 2*r[0]) + delta[1] * (delta[1] - 2*r[1]) + delta[2] * (delta[2] - 2*r[2]))/(r.dot(r))
    
    
    q1 = 1 + q
    return q * (3 + q * (3 + q))/ ( 1 + sp.sqrt(q1**3))

rrel1, rrel2, rrel3 = sp.symbols("rrel1, rrel2, rrel3")
vrel1, vrel2, vrel3 = sp.symbols("vrel1, vrel2, vrel3")
a1, a2, a3 = sp.symbols("a1, a2, a3")
muR13, muR23 = sp.symbols("muR13, muR23")

R1_1, R1_2, R1_3, R2_1, R2_2, R2_3 = sp.symbols("R1_1, R1_2, R1_3, R2_1, R2_2, R2_3")

rrel = sp.Matrix([[rrel1, rrel2, rrel3]]).T
vrel = sp.Matrix([[vrel1, vrel2, vrel3]]).T
a = sp.Matrix([[a1, a2, a3]]).T
states = sp.Matrix(sp.BlockMatrix([[rrel], [vrel]]))

R1 = sp.Matrix([[R1_1, R1_2, R1_3]]).T
R2 = sp.Matrix([[R2_1, R2_2, R2_3]]).T

r1 = R1 + rrel
r2 = R2 + rrel

fq1 = EnckeFq(r1, rrel)
fq2 = EnckeFq(r2, rrel)

eoms = sp.Matrix([[vrel1, vrel2, vrel3,
                   a1 - muR13 * (rrel1 + fq1 * r1[0]) - muR23 * (rrel1 + fq2 * r2[0]),
                   a2 - muR13 * (rrel2 + fq1 * r1[1]) - muR23 * (rrel2 + fq2 * r2[1]),
                   a3 - muR13 * (rrel3 + fq1 * r1[2]) - muR23 * (rrel3 + fq2 * r2[2])]])

jac = eoms.jacobian(states)
autowrap(jac, language="C", backend="Cython", tempdir="../C_functions")
