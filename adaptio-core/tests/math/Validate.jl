using SymPy

using Printf
Base.show(io::IO, f::Float64) = @printf(io, "%.16f", f)

# Parameters

f_scaled = 3.744565963745117188e+00
rho = 3.141447305679321289e+00
tau = 1.645262539386749268e-01
d = 5.489320311280183606e-01

R = [9.999974673412257431e-01 2.039705193809659024e-03 9.512696023625968975e-04;
0.000000000000000000e+00 4.226691551490259768e-01 -9.062840533108859065e-01;
-2.250624609754632317e-03 9.062817580026263364e-01 4.226680846722816187e-01]
T = [0.000000000000000000e+00 0.000000000000000000e+00 4.087606157143235386e-01]
w = 0.007093
m = 0.1
K1 = 3.780014812946319580e-03
K2 =         -1.993117621168494225e-03
K3 = 5.228068857832113281e-07
P1 =        -1.876385213108733296e-04
P2 =        -5.847600405104458332e-04

K1c = 0.1
K2c = 0.1
K3c = 0.1
P1c = 0.1
P2c = 0.1

principal_point = [9.869480729103088379e-01 7.230033874511718750e-01]'
pixel_pitch = [2.74e-06 2.74e-06]'

image_cpp = zeros(Float64, 3, 10);
image_python = zeros(Float64, 3, 10);
for i in 1:Int((size(image_cpp, 2)/2))
    image_cpp[1, i] = i;
    image_cpp[2, i] = 500 + i * (20.0 / 3500.0);
end

for i in Int((size(image_cpp, 2)/2)):(size(image_cpp, 2))
    image_cpp[1, i] = 3495 + i;
    image_cpp[2, i] = 500 + (3495 + i) * (20.0 / 3500.0);
end

display(image_cpp)
image_python = image_cpp;

image_python = image_python ./ w;
for i in 1:(size(image_cpp, 2))
    image_cpp[1:2, i] = image_cpp[1:2, i] .* pixel_pitch .+ principal_point;
    image_python[1:2, i] = image_python[1:2, i] .* pixel_pitch .+ principal_point;
end

image_cpp[3, :] = ones(Float64, 1, size(image_cpp, 2));
image_python[3, :] = ones(Float64, 1, size(image_python, 2))/w;

display(image_cpp)
display(image_python)

println("Real coords diff")
display(image_cpp - image_python)

println("--------------")
println("Tilted lens")
println("--------------")

include("cpp/Hp.jl")
include("python/Hp.jl")

println("\n\nC++:")
Hp_cpp = Cpp.Hp.subs(Cpp.rho, rho).subs(Cpp.tau, tau,).subs(Cpp.d, d * m);
Hp_inv_cpp = Hp_cpp^-1;

println("\nHp");
display(Hp_cpp);
println("\nHp^-1");
display(Hp_inv_cpp);

println("\n\nPython");
Hp_python = Python.Hp.subs(Python.rho, rho).subs(Python.tau, tau,).subs(Python.d, d).subs(Python.m, m);
Hp_inv_python = Python.Hp_inv.subs(Python.rho, rho).subs(Python.tau, tau,).subs(Python.d, d).subs(Python.m, m);

println("\nHp");
display(Hp_python);
println("\nHp^-1");
display(Hp_inv_python);

println("\n\nDiff:");
Hp_diff = Hp_cpp - Hp_python;
Hp_inv_diff = Hp_inv_cpp - Hp_inv_python;

println("\nHp");
display(Hp_diff);
println("\nHp^-1");
display(Hp_inv_diff);

# Calculations for "real" image
image_cpp = Hp_inv_cpp * image_cpp;
image_python = (w*Hp_inv_python) * image_python;

image_cpp[1, :] = image_cpp[1, :] .* (image_cpp[3, :].^-1);
image_cpp[2, :] = image_cpp[2, :] .* (image_cpp[3, :].^-1);

image_python[1, :] = image_python[1, :] .* (w*(image_python[3, :].^-1));
image_python[2, :] = image_python[2, :] .* (w*(image_python[3, :].^-1));

println("Real coords diff")
display(image_cpp - image_python)



println("--------------")
println("Undistort")
println("--------------")

include("cpp/Undistort.jl")
include("python/Undistort.jl")

coordinates = [1.0 2.0 3.0 4.0 5.0 ; 0.1 0.2 0.3 0.4 0.5];

undistorted_cpp = Cpp.Undistort(coordinates, K1 * K1c, K2 * K2c, K3 * K3c, P1 * P1c, P2 * P2c);
undistorted_python = Python.Undistort(coordinates, K1, K2, K3, P1, P2, K1c, K2c, K3c, P1c, P2c);

println("\n\nCpp:");
println("");
display(undistorted_cpp);

println("\n\nPython:");
println("");
display(undistorted_python);

println("\n\nDiff:");
println("");
display(undistorted_cpp - undistorted_python);

undistorted_image_cpp = Cpp.Undistort(image_cpp[1:2, :], K1 * K1c, K2 * K2c, K3 * K3c, P1 * P1c, P2 * P2c);
undistorted_image_python = Python.Undistort(image_python[1:2, :], K1, K2, K3, P1, P2, K1c, K2c, K3c, P1c, P2c);

println("Real coords diff")
display(undistorted_image_cpp - undistorted_image_python)

println("--------------");
println("Image -> Laser");
println("--------------");


include("cpp/ImagePlaneToLaserPlane.jl");
include("python/ImagePlaneToLaserPlane.jl");

coordinates = [1.0 2.0 3.0 4.0 5.0 ; 0.1 0.2 0.3 0.4 0.5];

println("\n\nCpp:");

laser_plane_cpp = Cpp.ImagePlaneToLaserPlane(coordinates, R, T, f_scaled);

println("");
display(laser_plane_cpp);


println("\n\nPython:");

laser_plane_python = Python.ImagePlaneToLaserPlane(coordinates, R, T, f_scaled, w, m);

println("");
display(laser_plane_python);

println("\n\nDiff:");
println("");
display(laser_plane_cpp - laser_plane_python);

laser_plane_image_cpp = Cpp.ImagePlaneToLaserPlane(undistorted_image_cpp, R, T, f_scaled);
laser_plane_image_python = Python.ImagePlaneToLaserPlane(undistorted_image_python, R, T, f_scaled, w, m);


println("\n\nCpp:\n");
display(laser_plane_image_cpp);

println("\n\nPython:\n");
display(laser_plane_image_python);

println("\n\nDiff:\n")
display(laser_plane_image_cpp - laser_plane_image_python);
