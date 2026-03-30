function plot_histograms(lidar_data, gnss_data, unc)

gnss_labels = {'x', 'y'};
n_lidar     = size(lidar_data, 2);

figure('Name', 'LiDAR Histograms');
for ch = 1:n_lidar
    subplot(1, n_lidar, ch);
    histogram(lidar_data(:, ch), 20);
    title(sprintf('Ch %d  \\sigma = %.3f', ch, unc.lidar_std(ch)));
    xlabel('Distance (m)'); ylabel('Count'); grid on;
end

figure('Name', 'GNSS Histograms');
for ch = 1:2
    subplot(1, 2, ch);
    histogram(gnss_data(:, ch), 20);
    title(sprintf('%s  \\sigma = %.6f', gnss_labels{ch}, unc.gnss_std(ch)));
    xlabel(gnss_labels{ch}); ylabel('Count'); grid on;
end