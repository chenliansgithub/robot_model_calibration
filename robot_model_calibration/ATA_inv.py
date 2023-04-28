import sympy as sym
n = sym.symbols('n:3')
o = sym.symbols('o:3')
a = sym.symbols('a:3')
p = sym.symbols('p:3')

delta = sym.symbols('delta:3')
d = sym.symbols('d:3')
Delta = sym.Matrix([[0,-delta[2],delta[1],d[0]],[delta[2],0,-delta[0],d[1]],[-delta[1],delta[0],0,d[2]],[0,0,0,0]])
print(sym.latex(sym.simplify(Delta)))
R = sym.Matrix.eye(3)
R[:3,0] = n
R[:3,1] = o
R[:3,2] = a
print(sym.latex(sym.simplify(R)))
T = sym.eye(4)
T[0:3,0:3] = R
T[0:3,3] = p 
print(sym.latex(sym.simplify(T)))
T_inv = sym.Matrix.eye(4)
n_m = sym.Matrix(n)
o_m = sym.Matrix(o)
a_m = sym.Matrix(a)
p_m = sym.Matrix(p)
T_inv[:3,:3] = R.T
T_inv[0,3] = -n_m.T * p_m
T_inv[1,3] = -o_m.T * p_m
T_inv[2,3] = -a_m.T * p_m
res = T_inv * Delta * T
print(sym.latex(sym.simplify(T_inv)))
print(sym.latex(sym.simplify(res)))