function rerender_hists()
% Re-export the E2 batch histograms with fonts sized for half-column
% display (the 6-page layout places them side by side at 0.49\linewidth).
T = readtable('rst_v3_feasible100.csv');
fs = 22;

f1 = figure('Visible','off','Position',[100 100 560 430],'Color','w');
histogram(T.Iterations, 'FaceColor', [0.35 0.5 0.75]);
xlabel('Iterations to converge','FontSize',fs); ylabel('Poses','FontSize',fs);
set(gca,'FontSize',fs,'Color','w','XColor',[0.1 0.1 0.1],'YColor',[0.1 0.1 0.1]); grid on;
exportgraphics(f1, 'fig_rst_iter_hist_big.pdf', 'ContentType','vector');

f2 = figure('Visible','off','Position',[100 100 560 430],'Color','w');
histogram(log10(T.PosErr_mm), 'FaceColor', [0.8 0.45 0.35]);
xlabel('log_{10} final position error (mm)','FontSize',fs); ylabel('Poses','FontSize',fs);
set(gca,'FontSize',fs,'Color','w','XColor',[0.1 0.1 0.1],'YColor',[0.1 0.1 0.1]); grid on;
exportgraphics(f2, 'fig_rst_residual_hist_big.pdf', 'ContentType','vector');
fprintf('exported big-font histograms\n');
end
