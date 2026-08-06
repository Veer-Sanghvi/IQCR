%% IQCR - Robotics System Toolbox verification, ABB IRB 120
clear; clc; rng(7);
robot = loadrobot('abbIrb120','DataFormat','row','Gravity',[0 0 -9.81]);
ee = 'tool0';

nJ = 6;
lims = zeros(nJ,2);
for i = 1:nJ
    lims(i,:) = robot.Bodies{i}.Joint.PositionLimits;
end
% conservative subset of published limits, away from mechanical stops
qmin = lims(:,1)'*0.7;
qmax = lims(:,2)'*0.7;

iksolver = inverseKinematics('RigidBodyTree', robot);
iksolver.SolverParameters.AllowRandomRestart = true;
weights = [1 1 1 1 1 1];

N = 8;
results = zeros(N,4); % pos_err_mm, ori_err_deg, iterations, converged
for k = 1:N
    q_true = qmin + rand(1,nJ).*(qmax-qmin);
    T_target = getTransform(robot, q_true, ee);

    q_init = q_true + deg2rad(15)*(rand(1,nJ)-0.5);
    q_init = max(min(q_init, qmax), qmin);

    [qSol, solInfo] = iksolver(ee, T_target, weights, q_init);

    T_final = getTransform(robot, qSol, ee);
    posErr = norm(T_target(1:3,4) - T_final(1:3,4)) * 1000; % m -> mm
    Rerr = T_target(1:3,1:3) * T_final(1:3,1:3)';
    cosang = min(1,max(-1,(trace(Rerr)-1)/2));
    oriErr = rad2deg(acos(cosang));

    results(k,:) = [posErr, oriErr, solInfo.Iterations, strcmp(solInfo.Status,'success')];
end

T = array2table(results, 'VariableNames', {'PosErr_mm','OriErr_deg','Iterations','Converged'});
T.Case = (1:N)';
T = T(:, {'Case','PosErr_mm','OriErr_deg','Iterations','Converged'});
disp(T)
writetable(T, 'rst_ik_verification_results.csv');

% --- color figures ---
colors = lines(N);

f1 = figure('Visible','off','Position',[100 100 520 360],'Color','w');
semilogy(1:N, max(results(:,1),1e-9), '-', 'Color',[0.2 0.2 0.7], 'LineWidth',1.5); hold on;
scatter(1:N, max(results(:,1),1e-9), 60, colors, 'filled');
xlabel('Test pose'); ylabel('Final position error (mm)');
title('Robotics Toolbox IK convergence error, ABB IRB 120');
grid on; xlim([0.5 N+0.5]); set(gca,'Color','w');
exportgraphics(f1, 'fig_rst_ik_error.png', 'Resolution', 200);

f2 = figure('Visible','off','Position',[100 100 520 360],'Color','w');
b = bar(1:N, results(:,3), 'FaceColor','flat');
b.CData = colors;
xlabel('Test pose'); ylabel('Solver iterations to converge');
title('Inverse kinematics solver iterations per pose');
grid on; set(gca,'Color','w');
exportgraphics(f2, 'fig_rst_ik_iterations.png', 'Resolution', 200);

% --- real robot visualization figure (color, toolbox-rendered) ---
f3 = figure('Visible','off','Position',[100 100 600 500],'Color','w');
show(robot, q_true, 'PreservePlot', false, 'Frames','off');
title('ABB IRB 120 reference model, randomized inspection pose');
view(135,20);
exportgraphics(f3, 'fig_rst_robot_pose.png', 'Resolution', 200);

fprintf('\nMean position error: %.4e mm\n', mean(results(:,1)));
fprintf('Max position error: %.4e mm\n', max(results(:,1)));
fprintf('Mean iterations: %.1f\n', mean(results(:,3)));
fprintf('Converged: %d / %d\n', sum(results(:,4)), N);
