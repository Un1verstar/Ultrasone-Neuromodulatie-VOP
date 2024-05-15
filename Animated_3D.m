% create the computational grid
Nx = 300;           % number of grid points in the x (row) direction 
Ny = 150;           % number of grid points in the y (column) direction 
Nz = 150;           % number of grid points in the z (depth) direction 
dx = 0.5 * 1e-3;    % grid point spacing in the x direction [m]
dy = dx;          % grid point spacing in the y direction [m]
dz = dx;          % grid point spacing in the z direction [m]
kgrid = kWaveGrid(Nx, dx, Ny, dy, Nz, dz);

% define the medium properties
medium.sound_speed = 1500;      % [m/s]
medium.density = 1000;          % [kg/m^3]

% create the time array
kgrid.makeTime(medium.sound_speed);

% define bowl transducer properties
bowl_radius = 0.085/dx;      % [m]
bowl_diameter = 0.064/dx;    % [m]

if rem(bowl_diameter,2) == 0
    bowl_diameter = bowl_diameter + 1;
end

focus_pos = [Nx/2, Ny/2, Nz/2]; % [m]
bowl_pos = [1, Ny/2, Nz/2];

% create the bowl transducer using makeBowl()
bowl = makeBowl([Nx, Ny, Nz], bowl_pos, bowl_radius, bowl_diameter, focus_pos);

% define source properties
source_strength = 10;     % [Pa]
source_freq = 500e3;      % [Hz]
source.p = source_strength * sin(2 * pi * source_freq * kgrid.t_array);
source.p = filterTimeSeries(kgrid, medium, source.p);
source.p_mask = bowl;

% define a binary line sensor
sensor.mask = zeros(Nx, Ny, Nz);
sensor.mask(:, :, :) = 1;

% set the input options
input_args = {'DisplayMask', source.p_mask, 'DataCast', 'single', 'PlotSim', false};

% run the simulation
sensor_data = kspaceFirstOrder3D(kgrid, medium, source, sensor, input_args{:});

%disp(size(sensor_data));

% Choose a time step to visualize
time_step_to_visualize = size(sensor_data, 2); % for last time step

% Frame skip for the animation
skip = 10;

% Global color limits for consistency
min_pressure = min(sensor_data(:));
max_pressure = max(sensor_data(:));

% Preparing video files
vXY = VideoWriter('animation_XY.avi');
vYZ = VideoWriter('animation_YZ.avi');
vXZ = VideoWriter('animation_XZ.avi');
open(vXY); open(vYZ); open(vXZ);

sensor_data = reshape(sensor_data, Nx, Ny, Nz, time_step_to_visualize);


% Loop through selected time steps
for t = 1:skip:size(sensor_data, 4) % Adjust skip as needed
    
    % XY plane at a chosen Z (mid-depth)
    figXY = figure('Visible', 'off');
    imagesc(squeeze(sensor_data(:, :, round(Nz/2), t))');
    clim([min_pressure max_pressure]);
    xlabel('X (mm)');
    ylabel('Y (mm)');

    cbar = colorbar; % Create the color bar
    ylabel(cbar, 'Pressure (Pa)'); % Add a label to the color bar
    title(sprintf('XY Plane at Time Step %d', t));
    frame = getframe(figXY);
    writeVideo(vXY, frame);
    close(figXY);
    
    % YZ plane at a chosen X (mid-width)
    figYZ = figure('Visible', 'off');
    imagesc(squeeze(sensor_data(round(Nx/2), :, :, t))');
    clim([min_pressure max_pressure]);
    xlabel('Y (mm)');
    ylabel('Z (mm)');

    cbar = colorbar; % Create the color bar
    ylabel(cbar, 'Pressure (Pa)'); % Add a label to the color bar
    title(sprintf('YZ Plane at Time Step %d', t));
    frame = getframe(figYZ);
    writeVideo(vYZ, frame);
    close(figYZ);
    
    % XZ plane at a chosen Y (mid-height)
    figXZ = figure('Visible', 'off');
    imagesc(squeeze(sensor_data(:, round(Ny/2), :, t))');
    clim([min_pressure max_pressure]);
    xlabel('X (mm)');
    ylabel('Z (mm)');

    cbar = colorbar; % Create the color bar
    ylabel(cbar, 'Pressure (Pa)'); % Add a label to the color bar
    title(sprintf('XZ Plane at Time Step %d', t));
    frame = getframe(figXZ);
    writeVideo(vXZ, frame);
    close(figXZ);
    
end

% Closing video files
close(vXY); close(vYZ); close(vXZ);
disp('Animations saved for XY, YZ, and XZ planes.');
