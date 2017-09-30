function simulate_fall
alpha = pi/4;
r = 1;
m = 1;
x0 = [0 3 0 0]';
h = 0.01;
N = 150;

sys1 = ContactImplicitSystem(@()ball_slope(alpha, r, m), 'ball');
sys1 = sys1.addVisualizer(@(x) ball_slope_visual(x, alpha, r));

[t, x] = sys1.simulate(0.01, N, x0, true, 4);

end