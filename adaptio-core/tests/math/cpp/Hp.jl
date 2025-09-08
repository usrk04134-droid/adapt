module Cpp
using SymPy

@syms d rho tau

Rt11 = cos(rho)^2*(1-cos(tau))+cos(tau)
Rt12 = cos(rho)*sin(rho)*(1-cos(tau))
Rt13 = sin(rho)*sin(tau)

Rt21 = cos(rho)*sin(rho)*(1-cos(tau))
Rt22 = sin(rho)^2*(1-cos(tau))+cos(tau)
Rt23 = -cos(rho)*sin(tau)

Rt31 = -sin(rho)*sin(tau)
Rt32 = cos(rho)*sin(tau)
Rt33 = cos(tau)

Rt = Sym[Rt11 Rt12 Rt13 ; Rt21 Rt22 Rt23 ; Rt31 Rt32 Rt33];
R = Rt.T
Ks = Sym[d*Rt[3,3] 0 0 ; 0 d*Rt[3,3] 0; 0 0 1];
Ku = Sym[d 0 0 ; 0 d 0 ; 0 0 1];
T = Sym[1 0 -d*Rt[3,1] ; 0 1 -d*Rt[3,2] ; 0 0 1];


Hp = convert(Matrix{Sym}, T * Ks * R * Ku^-1);

end