
module Cpp
using SymPy



function Undistort(coordinates::Matrix, K1, K2, K3, P1, P2)
    distorted = coordinates
    rd2 = distorted[1,:] .* distorted[1,:] .+ distorted[2,:] .* distorted[2,:]
    rd4 = rd2.^2;
    rd6 = rd2.^3;

    x_undistorted = distorted[1,:] .* (K1 * rd2 .+ K2 * rd4 .+ K3 * rd6 .+ 1) + (P1 * (rd2 .+ 2 * distorted[1,:].^2)) + 2 * P2 * distorted[1,:] .* distorted[2,:]
    y_undistorted = distorted[2,:] .* (K1 * rd2 .+ K2 * rd4 .+ K3 * rd6 .+ 1) + (2 * P1 * distorted[1,:] .* distorted[2,:] .+ P2 * (distorted[1,:].^2 .+ 2 * distorted[2,:].^2))

    undistorted = zeros(Float64, 2, length(x_undistorted))
    undistorted[1, :] = x_undistorted
    undistorted[2, :] = y_undistorted

    return undistorted
end
end