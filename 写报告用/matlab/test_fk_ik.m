%% 
clear; clc;

%%
clear; clc;

pos_and_eul = [-0.4, 0.1, -0.1, deg2rad(-90), 0, 0];   % [x, y, z, alpha, beta, gamma] (m, rad)

solutions = ik_space_robot(pos_and_eul(1), pos_and_eul(2), pos_and_eul(3), ...
                           pos_and_eul(4), pos_and_eul(5), pos_and_eul(6));

fprintf('关节角 (deg):'); fprintf(' %.2f', rad2deg(solutions(1).theta)); fprintf('\n');

[pos_fk, eul_fk] = fk_space_robot(solutions(1).theta);

fprintf('位置: (%.0f, %.0f, %.0f) mm\n', pos_fk*1000);
fprintf('欧拉角: (%.2f, %.2f, %.2f) deg\n', rad2deg(eul_fk));


