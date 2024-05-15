source_freq = 500e3;      % [Hz]
medium.sound_speed = 1500;      % [m/s]

dx = (medium.sound_speed/source_freq) * (0.5/3);          % grid point spacing in the r direction [m]
dz = dx;          % grid point spacing in the z direction [m]

Nx = round(0.25/(2*dx));           % number of grid points in the radial direction (r)
Nz = round(0.1/(2*dz));           % number of grid points in the axial direction (z)

kgrid = kWaveGrid(Nx, dx, Nz, dz);


medium.density = 1000;          % [kg/m^3]

t_end = 200 * 1e-6;

% create the time array
kgrid.makeTime(medium.sound_speed, [], t_end);


arc_radius = 0.085 / dx;      % Adjusted for grid [in grid points]
arc_diameter = 0.064 / dx;    % Adjusted for grid [in grid points]

if rem(round(arc_diameter),2) == 0
    arc_diameter = arc_diameter + 1;
end

focus_pos = [Nx/2, 1];      % Adjusted for 2D grid
arc = makeArc([Nx, Nz], [1,1], arc_radius, arc_diameter, focus_pos);

% Define source properties
source_strength = 10;     % [Pa]
source.p = source_strength * sin(2 * pi * source_freq * kgrid.t_array);
source.p = filterTimeSeries(kgrid, medium, source.p);
source.p_mask = arc;



% Define a sensor line across the diameter of the tank
sensor.mask = ones(Nx, Nz); % Select a plane through the cylinder

% Set the input options
input_args = {'DisplayMask', source.p_mask, 'DataCast', 'single','PlotSim', false, 'PMLInside', false};

% Run the simulation using the axisymmetric solver
sensor_data = kspaceFirstOrderAS(kgrid, medium, source, sensor, input_args{:});

% Plotting the pressure distribution at the last time step
last_time_step = size(sensor_data, 2); % for the last time step

sensor_data_amplitude = max(sensor_data(:, 1:last_time_step), [], 2);



pressure_distribution = reshape(sensor_data_amplitude, [Nx, Nz]);


axial_positions = dx*(0:Nx-1); % Radial distances from the axis of symmetry
radial_distance = dz*(0:Nz-1); % Axial positions from the transducer

% Create a symmetrical radial distance array for plotting
sym_radial_distance = [-flip(radial_distance), radial_distance];

% Prepare the pressure data for symmetrical plotting
sym_pressure_distribution = [flip(pressure_distribution,2), pressure_distribution];

% Plotting
figure;
imagesc(axial_positions, sym_radial_distance, sym_pressure_distribution');
axis xy; % Corrects the orientation
xlabel('Axial Position (m)');
ylabel('Radial Distance (m)');
cbar = colorbar; % Create the color bar
ylabel(cbar, 'Pressure (Pa)'); % Add a label to the color bar
title('Symmetrical Pressure Distribution');



% Find the maximum amplitude and its index
[max_amplitude, max_index] = max(sensor_data_amplitude(:));

% Convert the index to grid coordinates
[max_idx_r, max_idx_z] = ind2sub(size(sensor_data_amplitude), max_index);

% Convert the grid coordinates to radial and axial distances
max_axial_position = max_idx_r * dx;
max_radial_distance = max_idx_z * dz;

% Display the results
fprintf('Maximum amplitude: %.2f Pa\n', max_amplitude);
fprintf('Position of maximum amplitude - Radial distance: %.4f m, Axial position: %.4f m\n', max_radial_distance, max_axial_position);

% HFWHM

% Find maximum pressure value and its position
[max_pressure, max_index] = max(sensor_data_amplitude);
[max_row, max_col] = ind2sub(size(pressure_distribution), max_index);
max_axial_position = axial_positions(max_row);
max_radial_distance = sym_radial_distance(max_col);

% Calculate FWHM along the axial direction
half_max_pressure = max_pressure / 2;
axial_indices_left = find(pressure_distribution(:, max_col) >= half_max_pressure, 1, 'first');
axial_indices_right = find(pressure_distribution(:, max_col) >= half_max_pressure, 1, 'last');
fwhm_axial = axial_positions(axial_indices_right) - axial_positions(axial_indices_left);

% Calculate FWHM along the radial direction (assuming maximum at radial coordinate = 0)
half_max_pressure_radial = max(sensor_data_amplitude) / 2;
radial_indices_left = find(pressure_distribution(max_row, :) >= half_max_pressure_radial, 1, 'first');
radial_indices_right = find(pressure_distribution(max_row, :) >= half_max_pressure_radial, 1, 'last');
fwhm_radial = sym_radial_distance(radial_indices_right) - sym_radial_distance(radial_indices_left);

% Display results
fprintf('FWHM along axial direction: %.4f m\n', fwhm_axial);
fprintf('FWHM along radial direction: %.4f m\n', fwhm_radial);


