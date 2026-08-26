function rerender_robot_pose()
% Re-export fig_rst_robot_pose with tight axis limits around the arm.
% Pose comes from the archived E1 case 8 joint vector (rst_v3_eight.csv),
% same pose the paper's Fig. shows; only the framing changes.
T = readtable('rst_v3_eight.csv');
q = table2array(T(8, {'qTrue1','qTrue2','qTrue3','qTrue4','qTrue5','qTrue6'}));
robot = loadrobot('abbIrb120', 'DataFormat', 'row');
fs = 13;
f = figure('Visible','off','Position',[100 100 700 620],'Color','w');
show(robot, q, 'PreservePlot', false, 'Frames','off');
view(135,20); set(gca,'FontSize',fs,'Color','w');
% tight box around the arm's actual extent (IRB 120 reach 0.58 m)
axis([-0.45 0.45 -0.45 0.45 0 0.75]);
axis vis3d;
exportgraphics(f, 'fig_rst_robot_pose_tight.png', 'Resolution', 600);
fprintf('exported fig_rst_robot_pose_tight.png\n');
end
