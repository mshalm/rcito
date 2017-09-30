function [q, v, u, T, V, B, phi] = ball_trough(alpha, r, m)
% dynamics of a ball of radius r and mass m that can interact with a trough
% with mass
% zero friction means that the ball will not rotate, so the angle of the
% ball will not be modeled

R = inline('[cos(t) -sin(t); sin(t) cos(t)]');
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

q_perp1 = R(-alpha)*q;
q_perp2 = R(alpha)*q;

phi = [q_perp1(2) - r, q_perp2(2) - r]';
    

end

