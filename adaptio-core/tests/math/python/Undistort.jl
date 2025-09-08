
module Python

function Undistort(coordinates::Matrix, K1, K2, K3, P1, P2, K1c, K2c, K3c, P1c, P2c)

    P_dipcs_scaled = coordinates
    K1_scaled = K1
    K2_scaled = K2
    K3_scaled = K3
    P1_scaled = P1
    P2_scaled = P2

    r2_dipcs_scaled = P_dipcs_scaled[1, :] .* P_dipcs_scaled[1, :] + P_dipcs_scaled[2, :] .* P_dipcs_scaled[2, :]

    a1 = P_dipcs_scaled[1, :] .* (1. .+ K1c*K1_scaled*r2_dipcs_scaled .+ K2c*K2_scaled * (r2_dipcs_scaled.^2) .+ K3c*K3_scaled * (r2_dipcs_scaled.^3))
    a2 = (P1c*P1_scaled .* (r2_dipcs_scaled .+ 2.0 * P_dipcs_scaled[1, :].^2) .+ 2.0 .* P2c*P2_scaled .* (P_dipcs_scaled[1, :]) .* (P_dipcs_scaled[2, :]))
    a = a1 .+ a2

    b1 = P_dipcs_scaled[2, :] .* (1. .+ K1c*K1_scaled*r2_dipcs_scaled .+ K2c*K2_scaled * (r2_dipcs_scaled.^2) .+ K3c*K3_scaled * (r2_dipcs_scaled.^3))
    b2 = (P2c*P2_scaled .* (r2_dipcs_scaled .+ 2.0 * P_dipcs_scaled[2, :].^2) .+ 2.0 .* P1c*P1_scaled .* (P_dipcs_scaled[1, :]) .* (P_dipcs_scaled[2, :]))
    b = b1 .+ b2

    #return reshape([a ; b], 2, convert(Int64, round(Int, length([a ; b])/2)));

    undistorted = zeros(Float64, 2, length(a))
    undistorted[1,:] = a
    undistorted[2,:] = b
    return undistorted
end

end