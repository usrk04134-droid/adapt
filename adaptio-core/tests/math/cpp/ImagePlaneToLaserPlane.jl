module Cpp

function ImagePlaneToLaserPlane(P_ipcs_scaled::Matrix, R::Matrix, T::Matrix, focus_distance)
    z_component = zeros(Float64, 1, size(P_ipcs_scaled, 2)) .+ focus_distance
    coordinates = zeros(Float64, 3, size(P_ipcs_scaled, 2))
    coordinates[1, :] = P_ipcs_scaled[1,:]
    coordinates[2, :] = P_ipcs_scaled[2,:]
    coordinates[3, :] = z_component

    for i in 1:size(coordinates, 2)
        coordinates[:,i] = R^-1 * (coordinates[:,i] - T')
    end

    projection_center = -R^-1 * T'

    optical_rays = zeros(Float64, 3, size(P_ipcs_scaled, 2))

    for i in 1:size(coordinates, 2)
        optical_rays[:,i] = coordinates[:,i] - projection_center
    end

    wcs_coordinates = zeros(Float64, 3, size(coordinates, 2))
    wcs_coordinates[1,:] = -projection_center[3] ./ optical_rays[3,:] .* optical_rays[1,:] .+ projection_center[1,:]
    wcs_coordinates[2,:] = -projection_center[3] ./ optical_rays[3,:] .* optical_rays[2,:] .+ projection_center[2,:]

    return wcs_coordinates

end
end