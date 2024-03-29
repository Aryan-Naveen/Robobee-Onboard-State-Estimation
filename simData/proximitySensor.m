function tof = proximitySensor(ptCloud)
    % Define sensor characteristics (e.g., field of view, range)
    fov = 180; % Field of view in degrees
    maxRange = 1.2; % Maximum range of the proximity sensor in meters

    % Extract XYZ coordinates from the point cloud

    XYZ = reshape(ptCloud.Location, [size(ptCloud.Location, 1)*size(ptCloud.Location, 2), 3]);


    % Calculate Euclidean distances
    distances = sqrt(sum(XYZ.^2, 2));

    % Filter points within the sensor's field of view and within range
    validPoints = (abs(atan2d(XYZ(:,3), XYZ(:,2))) <= fov/2) & (distances <= maxRange);

    % Get the closest valid point
    rmsNoise = 1e-3;
    distances = distances +  rmsNoise * randn(size(distances, 1), 1);
    closestPoint = min(distances(validPoints));


    % Output sensor reading
    if isempty(closestPoint)
        tof = 0; % No valid points in range
    else
        tof = closestPoint;
    end
end

