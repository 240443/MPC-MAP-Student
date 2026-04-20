function [unc] = calc_unc(read_only_vars, plot_en)

persistent lidar_history

% keep accumulating measurements
if isempty(lidar_history)
    lidar_history = [];
end
lidar_history(end+1, :) = read_only_vars.lidar_distances;

N = size(lidar_history, 1);
if N < 10
    unc = [];
    return
end

%% LIDAR STATISTICS
unc.lidar_std = std(lidar_history);
unc.lidar_cov = cov(lidar_history);

%% GNSS STATISTICS
gnss_window   = read_only_vars.gnss_history(1:N, :);
unc.gnss_std  = std(gnss_window);
unc.gnss_cov  = cov(gnss_window);

% disp('Lidar std:'); disp(unc.lidar_std);
% disp('GNSS  std:'); disp(unc.gnss_std);

% --- Plot histograms once, then stop ---
if plot_en && N == 200
    plot_histograms(lidar_history, gnss_window, unc);

    mu = 0;

sigma_lidar = unc.lidar_std(1);
sigma_gnss  = unc.gnss_std(1);

% x ranges centered around 0, wide enough to show the distribution
x_lidar = linspace(-3*sigma_lidar, 3*sigma_lidar, 500);
x_gnss  = linspace(-3*sigma_gnss,  3*sigma_gnss,  500);

figure('Name', 'Sensor Noise PDFs');

subplot(1, 2, 1);
plot(x_lidar, norm_pdf(x_lidar, mu, sigma_lidar), 'b', 'LineWidth', 1.5);
xlabel('Distance error (m)');
ylabel('Probability density');
title(sprintf('LiDAR Ch1  \\sigma = %.4f', sigma_lidar));
grid on;

subplot(1, 2, 2);
plot(x_gnss, norm_pdf(x_gnss, mu, sigma_gnss), 'r', 'LineWidth', 1.5);
xlabel('x error [-]');
ylabel('Probability density');
title(sprintf('GNSS X  \\sigma = %.6f', sigma_gnss));
grid on;

sgtitle('Sensor Noise Characteristics (Normal PDF)');
unc.move_en = 1;
unc.gnss_mean = mean(read_only_vars.gnss_history);

end