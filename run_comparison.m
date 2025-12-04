% RUN_COMPARISON - Run both v2 and klei implementations and compare results
% This script runs both flywheel analysis implementations, saves all plots,
% and produces a side-by-side comparison.

clear; close all; clc;

fprintf('=================================================================\n');
fprintf('     FLYWHEEL ANALYSIS COMPARISON: V2 vs KLEI (Team 16 Cycle)\n');
fprintf('=================================================================\n\n');

% Add path to EE functions
addpath('Project3_Functions');

%% Run V2 Analysis
fprintf('Running V2 analysis...\n');
fprintf('-----------------------------------------------------------------\n');

cd('v2');
flywheel_analysis_v2;
v2_figs = findall(0, 'Type', 'figure');

% Move v2 plots to comparison folder
for i = 1:length(v2_figs)
    fig = v2_figs(i);
    name = get(fig, 'Name');
    if ~isempty(name)
        % Clean up name for filename
        clean_name = regexprep(name, '[^a-zA-Z0-9]', '_');
        saveas(fig, sprintf('../comparison_plots/v2_%s.png', clean_name));
    end
end
close all;

cd('..');

%% Run Klei Analysis (with team_16_cycle)
fprintf('\n\nRunning Klei analysis (with team_16_cycle)...\n');
fprintf('-----------------------------------------------------------------\n');

cd('klei_repo');

% Run the modified klei script
MagLevFlywheelModel_team16;

klei_figs = findall(0, 'Type', 'figure');

% Save klei plots
for i = 1:length(klei_figs)
    fig = klei_figs(i);
    num = get(fig, 'Number');
    saveas(fig, sprintf('../comparison_plots/klei_fig%d.png', num));
end

cd('..');

fprintf('\n\n=================================================================\n');
fprintf('                    COMPARISON COMPLETE\n');
fprintf('=================================================================\n');
fprintf('Plots saved to: comparison_plots/\n');
