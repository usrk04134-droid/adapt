module Python

function ImagePlaneToLaserPlane(P_ipcs_scaled::Matrix, R::Matrix, T::Matrix, f_scaled, w, m)

    # Translation vector
    #T_scaled = (T./m).reshape(-1,1)
    T_scaled = transpose(T) ./ m

    # Ow
    # Ow = -m*np.matmul(np.transpose(R), T_scaled)
    Ow = -m .* transpose(R) * T_scaled

    # Dw
    # zv = np.zeros((1,pnts.shape[1])) + w*f_scaled
    zv = zeros(Float64, 1, size(P_ipcs_scaled, 2)) .+ (w * f_scaled)

    # y = np.vstack((w*P_ipcs_scaled,zv))
    y = zeros(Float64, 3, size(P_ipcs_scaled, 2))
    y[1, :] = w .* P_ipcs_scaled[1,:]
    y[2, :] = w .* P_ipcs_scaled[2,:]
    y[3, :] = zv;

    # Dw = np.matmul(np.transpose(R),y)
    Dw = transpose(R) * y

    # WCS
    # Pw = np.zeros((3,pnts.shape[1]))
    Pw = zeros(Float64, 3, size(P_ipcs_scaled, 2))

    Pw[1,:] = Ow[1,:] .- Ow[3,:] .* Dw[1,:] ./ Dw[3,:]
    Pw[2,:] = Ow[2,:] .- Ow[3,:] .* Dw[2,:] ./ Dw[3,:] #.+ origin_offset

    return Pw

end

end