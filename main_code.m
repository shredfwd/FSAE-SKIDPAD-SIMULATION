
clear; clc; close all;

g = 9.81;      
rho = 1.225;   


car = struct();
car.mass = 250;                 
car.wheelbase = 1.6;        
car.frontal_area = 1.0;        
car.Cd = 1.1;                   
car.Cl = -2.5;                 
car.max_brake_g = 2.0;         
car.final_drive_ratio = 3.5;
car.tire_radius = 0.2286;
car.drivetrain_efficiency = 0.92;
car.mu_peak = 1.6;             


setup_files = {'SIM 3.CSV','SIM 3 Positive.CSV', 'SIM 4 Positive.CSV', 'SIM 5 Positive.CSV', 'SIM 6 Positive.CSV', 'SIM 7 Positive.CSV', 'SIM 8 Positive.CSV','SIM 4.CSV', 'SIM 5.CSV', 'SIM 6.CSV', 'SIM 7.CSV', 'SIM 8.CSV'};

track_table = readtable('track_data.csv', 'VariableNamingRule', 'preserve');
torque_curve = readmatrix('torque curve.csv');
gear_ratios_table = readmatrix('gear ratios.csv');

track_points = [track_table.X_m, track_table.Y_m];
car.gears = gear_ratios_table(:, 2)';
car.torque_interp = griddedInterpolant(torque_curve(:,1), torque_curve(:,2), 'linear', 'none');


path_dist_diff = diff(track_points, 1, 1);
segment_lengths = [0; sqrt(sum(path_dist_diff.^2, 2))];
total_distance = cumsum(segment_lengths);
curvature = calculate_curvature(track_points);
num_points = length(total_distance);


fprintf('Starting Lap Time Simulation for %d setups...\n\n', length(setup_files));

results = []; 

for setup_idx = 1:length(setup_files)
    
    current_file = setup_files{setup_idx};
    fprintf('--- Simulating Setup: %s ---\n', current_file);
    
   
    steering_table = readtable(current_file, 'VariableNamingRule', 'preserve');
    
  
    car_sim = car;
    
    
raw_radii = steering_table.('TURNING CIRCLE RADIUS');
raw_ackermann = steering_table.('ACKERMANN');


[sorted_radii, sort_order] = sort(raw_radii);
sorted_ackermann = raw_ackermann(sort_order);


[final_radii, unique_idx] = unique(sorted_radii, 'stable');
final_ackermann = sorted_ackermann(unique_idx);


car_sim.ackermann_lookup = griddedInterpolant(final_radii, final_ackermann, 'linear', 'nearest');
    
   
    v_corner = zeros(num_points, 1);
    v_forward = zeros(num_points, 1);
    v_final = zeros(num_points, 1);

    for i = 1:num_points
        k = abs(curvature(i));
        
        if k < 1e-6 
            v_corner(i) = 500;
        else
            corner_radius_m = 1 / k;
            
            
            effective_mu = get_effective_mu(corner_radius_m, car_sim);
            
            numerator = effective_mu * car_sim.mass * g;
            denominator = car_sim.mass * k - 0.5 * effective_mu * rho * car_sim.frontal_area * car_sim.Cl;
            if denominator <= 0
                v_corner(i) = 500;
            else
                v_corner(i) = sqrt(numerator / denominator);
            end
        end
    end

    
    v_forward(1) = 1;
    for i = 2:num_points
        F_tractive = get_max_tractive_force(v_forward(i-1), car_sim);
        F_drag = 0.5 * rho * car_sim.frontal_area * car_sim.Cd * v_forward(i-1)^2;
        F_net = F_tractive - F_drag;
        accel = F_net / car_sim.mass;
        v_accel = sqrt(v_forward(i-1)^2 + 2 * max(0, accel) * segment_lengths(i));
        v_forward(i) = min(v_accel, v_corner(i));
    end

    v_final = v_forward;
    v_final(end) = v_forward(end);
    for i = (num_points-1):-1:1
        F_drag = 0.5 * rho * car_sim.frontal_area * car_sim.Cd * v_final(i+1)^2;
        F_brake_max = car_sim.max_brake_g * car_sim.mass * g;
        F_net_brake = F_brake_max + F_drag;
        decel = F_net_brake / car_sim.mass;
        v_brake = sqrt(v_final(i+1)^2 + 2 * decel * segment_lengths(i+1));
        v_final(i) = min(v_final(i), v_brake);
    end

    
    segment_times = segment_lengths(2:end) ./ ((v_final(1:end-1) + v_final(2:end)) / 2);
    lap_time = sum(segment_times);
    
    fprintf('    > Approximate Lap Time: %.3f seconds\n\n', lap_time);
  
    results(setup_idx).SetupFile = current_file;
    results(setup_idx).LapTime = lap_time;
    results(setup_idx).FinalVelocityProfile = v_final;
end


[min_lap_time, best_idx] = min([results.LapTime]);
best_setup = results(best_idx);

fprintf('==============================================\n');
fprintf('           SIMULATION SUMMARY\n');
fprintf('==============================================\n');
results_table = struct2table(results);
disp(results_table(:, {'SetupFile', 'LapTime'}));
fprintf('\n🏆 Best Setup is from file ''%s'' with a lap time of %.3f seconds.\n', best_setup.SetupFile, best_setup.LapTime);
fprintf('==============================================\n\n');


figure('Name', 'Best Lap Time Simulation Results', 'NumberTitle', 'off');
v_best = best_setup.FinalVelocityProfile;
subplot(2,1,1);
scatter(track_points(:,1), track_points(:,2), 20, v_best * 3.6, 'filled');
colormap('jet'); c = colorbar; c.Label.String = 'Velocity (kph)';
axis equal; title(sprintf('Best Setup (%s) - Track Velocity Profile', best_setup.SetupFile));
xlabel('X (m)'); ylabel('Y (m)'); grid on;

subplot(2,1,2);
plot(total_distance, v_best * 3.6, 'b-', 'LineWidth', 2);
title(sprintf('Best Setup (%s) - Velocity vs. Distance', best_setup.SetupFile));
xlabel('Distance (m)'); ylabel('Velocity (kph)'); grid on;





function mu_eff = get_effective_mu(corner_radius_m, car_sim)
    
  
    ackermann_penalty_factor = 0.2; 
    

    corner_radius_mm = corner_radius_m * 1000;
    
 
    ideal_ackermann = -100; 
    ackermann_at_radius = car_sim.ackermann_lookup(corner_radius_mm);
    
  
    ackermann_error = abs(1 - (ackermann_at_radius / ideal_ackermann));
    
  
    total_penalty = ackermann_error * ackermann_penalty_factor;

    mu_eff = car_sim.mu_peak * (1 - total_penalty);
end

function F_tractive = get_max_tractive_force(v, car)
    if v < 0.1, v = 0.1; end
    wheel_rpm = (v / car.tire_radius) * (60 / (2 * pi));
    max_wheel_torque = 0;
    for i = 1:length(car.gears)
        gear_ratio = car.gears(i);
        engine_rpm = wheel_rpm * gear_ratio * car.final_drive_ratio;
        engine_torque = car.torque_interp(engine_rpm);
        if isnan(engine_torque), engine_torque = 0; end
        wheel_torque = engine_torque * gear_ratio * car.final_drive_ratio * car.drivetrain_efficiency;
        if wheel_torque > max_wheel_torque
            max_wheel_torque = wheel_torque;
        end
    end
    F_tractive = max_wheel_torque / car.tire_radius;
end

function curvature = calculate_curvature(points)
    x = points(:,1); y = points(:,2); n = length(x);
    curvature = zeros(n,1);
    for i = 2:n-1
        x1=x(i-1); y1=y(i-1); x2=x(i); y2=y(i); x3=x(i+1); y3=y(i+1);
        area = 0.5*abs(x1*(y2-y3) + x2*(y3-y1) + x3*(y1-y2));
        d12=sqrt((x1-x2)^2+(y1-y2)^2); d23=sqrt((x2-x3)^2+(y2-y3)^2); d31=sqrt((x3-x1)^2+(y3-y1)^2);
        denominator = d12*d23*d31;
        if denominator < 1e-9, curvature(i) = 0; else, curvature(i) = (4*area)/denominator; end
    end
    curvature(1) = curvature(2); curvature(n) = curvature(n-1);

end