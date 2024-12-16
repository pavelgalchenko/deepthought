#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Jul 31 13:16:29 2024

"""

import sympy as sp
from sympy.utilities.autowrap import autowrap

r1, r2, r3, v1, v2, v3 = sp.symbols("r1, r2, r3, v1, v2, v3")
p1, p2, p3 = sp.symbols("p1, p2, p3")
mu1, mu2 = sp.symbols("mu1, mu2")

rs = sp.Matrix([[r1, r2, r3]]).T

states = sp.Matrix([[r1, r2, r3, v1, v2, v3]]).T

r13 = sp.sqrt(rs.dot(rs))**3
r2_42 = sp.Matrix([[r1 - p1, r2 - p2, r3 - p3]]).T
r23 = sp.sqrt(r2_42.dot(r2_42))**3
p3_42 = sp.Matrix([[p1, p2, p3]]).T
p33 = sp.sqrt(p3_42.dot(p3_42))**3

c1 = -mu1 / r13
c2 = -mu2 / r23
c3 = mu2 / p33

eoms = sp.Matrix([[v1, v2, v3, 
                   c1*r1 + c2*r2_42[0] + c3*p1,
                   c1*r2 + c2*r2_42[1] + c3*p2,
                   c1*r3 + c2*r2_42[2] + c3*p3,]])

jac = eoms.jacobian()
autowrap(jac, language="C", backend="Cython", tempdir="../C_functions")