import casadi as cs
from horizon.utils import utils
from truth.truth import AssertThat

sym_t = cs.MX

x = sym_t.sym('x', 1)
u = sym_t.sym('u', 1)

f = 2.*cs.sumsqr(x) + cs.sumsqr(u) + u*x # 2x^2 + u^2 + xu

print(f"f: {f}")

Ff = cs.Function('my_func', {'x': x, 'u': u, 'f': f}, ['x', 'u'], ['f'])

print(f"Ff: {Ff}")

DfDx_expected = 5 # DfDx(1,1) = 4x + 0 + u = 4 + 1 = 5
DfDu_expected = 3 # DfDu(1,1) = 0 + 2u + x = 2 + 1 = 3

# Test Jacobian
#jac from casadi
Jac_Ff = Ff.jac()
DfDx_casadi = Jac_Ff(x=[1], u=[1])["DfDx"]
DfDu_casadi = Jac_Ff(x=[1], u=[1])["DfDu"]
print(f"Jac_Ff: {Jac_Ff}")
print(f"Jac_Ff_DfDx casadi: {DfDx_casadi}")
print(f"Jac_Ff_DfDu casadi: {DfDu_casadi} #WRONG!")

AssertThat(DfDx_casadi).IsEqualTo(DfDx_expected)
AssertThat(DfDu_casadi).IsNotEqualTo(DfDu_expected)

#our jac
jacF, jack = utils.jac({'x': x, 'u': u, 'f': f}, ['x', 'u'], 'f')
DfDx = jacF(x=[1], u=[1])["DfDx"]
DfDu = jacF(x=[1], u=[1])["DfDu"]
print(f"Jac_Ff_DfDx: {DfDx}")
print(f"Jac_Ff_DfDu: {DfDu}")
print("jac: ", jack)

AssertThat(DfDx).IsEqualTo(DfDx_expected)
AssertThat(DfDu).IsEqualTo(DfDu_expected)

# Test Hessian
DDfDxDx_expected = 4 # DDfDxDx(1,1) = 4
DDfDxDu_expected = 1 # DDfDxDu(1,1) = 1
DDfDuDx_expected = 1 # DDfDuDx(1,1) = 1
DDfDuDu_expected = 2 # DDfDuDu(1,1) = 2

#jac from casadi
Hes_Ff = Jac_Ff.jac()
DDfDxDx_casadi = Hes_Ff(x=[1], u=[1])["DDfDxDx"]
DDfDuDu_casadi = Hes_Ff(x=[1], u=[1])["DDfDuDu"]
DDfDxDu_casadi = Hes_Ff(x=[1], u=[1])["DDfDxDu"]
DDfDuDx_casadi = Hes_Ff(x=[1], u=[1])["DDfDuDx"]
print(f"Hes_Ff_DDfDxDx casadi: {DDfDxDx_casadi}")
print(f"Hes_Ff_DDfDuDu casadi: {DDfDuDu_casadi} #WRONG!")
print(f"Hes_Ff_DDfDxDu casadi: {DDfDxDu_casadi}")
print(f"Hes_Ff_DDfDuDx casadi: {DDfDuDx_casadi} #WRONG!")

AssertThat(DDfDxDx_casadi).IsEqualTo(DDfDxDx_expected)
AssertThat(DDfDxDu_casadi).IsEqualTo(DDfDxDu_expected)
AssertThat(DDfDuDx_casadi).IsNotEqualTo(DDfDuDx_expected)
AssertThat(DDfDuDu_casadi).IsNotEqualTo(DDfDuDu_expected)

# our hessian
d = dict(list({'x': x, 'u': u}.items()) + list(jack.items()))
print("d: ", d)
print("jac.keys(): ", jack.keys())
hesF, hess = utils.jac(d, ['x', 'u'], jack.keys())
print("hess: ", hess)
DDfDxDx = hesF(x=[1], u=[1])["DDfDxDx"]
DDfDuDu = hesF(x=[1], u=[1])["DDfDuDu"]
DDfDxDu = hesF(x=[1], u=[1])["DDfDxDu"]
DDfDuDx = hesF(x=[1], u=[1])["DDfDuDx"]

print(f"Hes_Ff_DDfDxDx: {DDfDxDx_casadi}")
print(f"Hes_Ff_DDfDuDu: {DDfDuDu_casadi}")
print(f"Hes_Ff_DDfDxDu: {DDfDxDu_casadi}")
print(f"Hes_Ff_DDfDuDx: {DDfDuDx_casadi}")

AssertThat(DDfDxDx).IsEqualTo(DDfDxDx_expected)
AssertThat(DDfDxDu).IsEqualTo(DDfDxDu_expected)
AssertThat(DDfDuDx).IsEqualTo(DDfDuDx_expected)
AssertThat(DDfDuDu).IsEqualTo(DDfDuDu_expected)

### Next derivative
for key in jack.keys(): del d[key]
print("d: ", d)
d = dict(list(d.items()) + list(hess.items()))
pippoF, pippo = utils.jac(d, ['x', 'u'], hess.keys())
print("pippo: ", pippo)


sym_t = cs.MX

X = sym_t.sym('x', 1)

f = X*X*X*X
f_map = {'f': f}
var_map = {'x': X}
var_string_list = ['x']
for i in range(10):
    F, f = utils.jac(dict(list(var_map.items()) + list(f_map.items())), var_string_list, f_map.keys())
    print("f: ", f)
    f_map = f

f = X*X*X*X
f_map = {'f': f}
var_map = {'x': X}
for i in range(10):
    F, f = utils.jac(dict(list(var_map.items()) + list(f_map.items())), var_string_list, f_map.keys())
    print("f: ", f)
    h = F(x=X)["DfDx"]
    f_map = {'f': h}
