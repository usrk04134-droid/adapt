module Python
using SymPy
using LinearAlgebra


@syms m d rho tau

H11 = cos(rho)*cos(rho)*cos(tau) + sin(rho)*sin(rho)
H12 = cos(rho)*sin(rho)*(cos(tau) - 1.0)
H22 = sin(rho)*sin(rho)*cos(tau) + cos(rho)*cos(rho)
H31 = sin(rho)*sin(tau)/(m*d)
H32 = (-1)*cos(rho)*sin(tau)/(m*d)
H33 = cos(tau)

Hp = convert(Matrix{Sym}, Sym[H11 H12 0 ; 0 H22 0 ; H31 H32 H33 ]);

Hinv11 = -H22/(H12*H12 - H11*H22)
Hinv12 = H12/(H12*H12 - H11*H22)
Hinv22 = -H11/(H12*H12 - H11*H22)
Hinv31 = -(H12*H32 - H22*H31)/(H33*H12*H12 - H11*H22*H33)
Hinv32 = (H11*H32 - H12*H31)/(H33*H12*H12 - H11*H22*H33)
Hinv33 = 1.0/H33

Hp_inv = Sym[Hinv11 Hinv12 0.0; Hinv12 Hinv22 0.0; Hinv31 Hinv32 Hinv33]

end