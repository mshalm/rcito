function simulate_fall
alpha = pi/4;
r = 1;
r_s = 5;
m = 1;
x0 = [1 10 0 0]';
h = 0.01;
N = 500;

sys1 = ContactImplicitSystem(@() ball_collision(alpha, r, r_s, m), 'ball');
sys1 = sys1.addVisualizer(@(x) ball_collision_visual(x, alpha, r, r_s));

[t, x] = sys1.simulate(0.01, N, x0, true, 4);

end