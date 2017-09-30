function [q, v, u, T, V, B, phi] = ball_collision(alpha, r, r_s, m)
% dynamics of a ball of radius r and mass m that can interact with a static
% ball of radius r_s
% zero friction means that the ball will not rotate, so the angle of the
% ball will not be modeled

g = 9.81;

syms qx qy;
syms dqx dqy;

assume([qx qy dqx dqy],'real');

q = [qx qy]';
v = [dqx dqy]';
u = sym([]);

T = (1/2) * (dqx.^2 + dqy^2);
V = m * g * qy;
B = sym(zeros(numel(q),0));

phi = sqrt(qx^2 + qy^2) - r - r_s;

end

