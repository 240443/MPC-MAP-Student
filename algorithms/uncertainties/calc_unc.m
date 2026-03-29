function [unc] = calc_unc(read_only_vars, plot_en)

persistent lidar_history % so we can save it between calls

if isempty(lidar_history)
    lidar_history = []; % init
end

lidar_history(read_only_vars.counter,:) = read_only_vars.lidar_distances;
unc.lidar_history = lidar_history;

% modulo so that we calc every given amount of measurements
if mod(read_only_vars.counter, 200) == 0
    idx = read_only_vars.counter-99 : read_only_vars.counter;
    unc.lidar_std = std(lidar_history(idx,:));
    disp(unc.lidar_std);

if plot_en == 1
    persistent fig_lidar ax_lidar fig_gnss ax_gnss % persistant axis for 
%overplotting graphs

    % --- Lidar Histogram ---
    num_lidar    = size(lidar_history, 2);
    lidar_data   = lidar_history(idx, :);


    % ai generated slop that works
    if isempty(fig_lidar) || ~isvalid(fig_lidar)
        fig_lidar = figure('Name', 'Lidar per-channel Histogram', 'NumberTitle', 'off');
        ax_lidar  = gobjects(1, num_lidar);
        for ch = 1:num_lidar
            ax_lidar(ch) = subplot(1, num_lidar, ch, 'Parent', fig_lidar);
        end
    end

    %overplot it 
    for ch = 1:num_lidar
        cla(ax_lidar(ch));
        histogram(ax_lidar(ch), lidar_data(:, ch), 20);
        xlim(ax_lidar(ch), 'auto');
        xlabel(ax_lidar(ch), 'Distance (m)');
        ylabel(ax_lidar(ch), 'Count');
        title(ax_lidar(ch), sprintf('Ch %d  \\sigma=%.3f', ch, unc.lidar_std(ch)));
        grid(ax_lidar(ch), 'on');
    end

    sgtitle(fig_lidar, sprintf('Lidar - All measurements  [counter = %d]', read_only_vars.counter));
    drawnow;

    % --- GNSS Histogram ---
    gnss_labels = {'Latitude', 'Longitude'};
    gnss_data   = read_only_vars.gnss_history(idx, :);  % already in read_only_vars
    unc.gnss_std = std(gnss_data(max(1,end-99):end, :));   % std over last 100

    if isempty(fig_gnss) || ~isvalid(fig_gnss)
        fig_gnss = figure('Name', 'GNSS Histogram', 'NumberTitle', 'off');
        ax_gnss  = gobjects(1, 2);
        for ch = 1:2
            ax_gnss(ch) = subplot(1, 2, ch, 'Parent', fig_gnss);
        end
    end

    for ch = 1:2
        cla(ax_gnss(ch));
        histogram(ax_gnss(ch), gnss_data(:, ch), 20);
        xlim(ax_gnss(ch), 'auto');
        xlabel(ax_gnss(ch), gnss_labels{ch});
        ylabel(ax_gnss(ch), 'Count');
        title(ax_gnss(ch), sprintf('%s  \\sigma=%.6f', gnss_labels{ch}, unc.gnss_std(ch)));
        grid(ax_gnss(ch), 'on');
    end

    sgtitle(fig_gnss, sprintf('GNSS - All measurements  [counter = %d]', read_only_vars.counter));
    drawnow;
end

%lidar covariance -----
lidar_window     = unc.lidar_history(idx, :);       % 100 x 8
unc.lidar_cov    = cov(lidar_window);               % 8x8 covariance matrix

%gnss covariance -----
gnss_window      = read_only_vars.gnss_history(idx, :);  % 100 x 2
unc.gnss_cov     = cov(gnss_window);     
end 



end