Nx = 250;           % number of grid points in the axial direction (r)
Nz = 100;           % number of grid points in the radial direction (z)
dx = 0.5 * 1e-3;          % grid point spacing in the z direction [m]
dz = dx;          % grid point spacing in the r direction [m]
kgrid = kWaveGrid(Nx, dx, Nz, dz);

% define the medium properties
medium.sound_speed = 1500;      % [m/s]
medium.density = 1000;          % [kg/m^3]
t_end = 200 * 1e-6;

% create the time array
kgrid.makeTime(medium.sound_speed, [], t_end);

% define arc transducer properties
arc_radius = 0.085 / dx;      % Adjusted for grid [in grid points]
arc_diameter = 0.064 / dx;    % Adjusted for grid [in grid points]

if rem(arc_diameter,2) == 0
    arc_diameter = arc_diameter + 1;
end

focus_pos = [Nx/2, 1];      % Adjusted for 2D grid
arc = makeArc([Nx, Nz], [1,1], arc_radius, arc_diameter, focus_pos);

% Define source properties
source_strength = 10;     % [Pa]
source_freq = 500e3;      % [Hz]
source.p = source_strength * sin(2 * pi * source_freq * kgrid.t_array);
source.p = filterTimeSeries(kgrid, medium, source.p);
source.p_mask = arc;



% Define a sensor line across the diameter of the tank
sensor.mask = ones(Nx, Nz); % Select a plane through the cylinder

% Set the input options
input_args = {'DisplayMask', source.p_mask, 'DataCast', 'single','PlotSim', false, 'PMLInside', false};

% Run the simulation using the axisymmetric solver
sensor_data = kspaceFirstOrderAS(kgrid, medium, source, sensor, input_args{:});
last_time_step = size(sensor_data, 2);
pressure_distribution = sensor_data;


axial_positions = dx*(0:Nx-1); % Radial distances from the axis of symmetry
radial_distance = dz*(0:Nz-1); % Axial positions from the transducer

% Create a symmetrical radial distance array for plotting
sym_radial_distance = [-flip(radial_distance), radial_distance];



min_pressure = min(sensor_data(:));
max_pressure = max(sensor_data(:));

% Create a figure for the animation
fig = figure;

% Preparing the video file
videoFilename = 'pressure_distribution_animation.avi';
v = VideoWriter(videoFilename);
open(v);

% Loop through selected time steps to create animation frames
for t = 1:5:last_time_step % Adjust the step size as needed for time resolution and speed
    % Reshape sensor data for the current time step
    pressure_at_t = reshape(sensor_data(:, t), [Nx, Nz]);
    
    % Mirror the pressure data for symmetrical plotting
    sym_pressure_at_t = [flip(pressure_at_t, 2), pressure_at_t];
    
    % Plotting
    imagesc(axial_positions, sym_radial_distance, sym_pressure_at_t');
    axis xy; % Corrects the orientation
    xlabel('Axial Position (m)');
    ylabel('Radial Distance (m)');
    cbar = colorbar; % Create the color bar
    ylabel(cbar, 'Pressure (Pa)'); % Add a label to the color bar
    title(sprintf('Symmetrical Pressure Distribution at Time Step %d', t));

    clim([min_pressure max_pressure]);

    drawnow;
    
    % Capture the plot as a frame in the video
    frame = getframe(fig);
    writeVideo(v, frame);
end

% Close the video file
close(v);

% Inform the user
fprintf('Animation saved as %s\n', videoFilename);

