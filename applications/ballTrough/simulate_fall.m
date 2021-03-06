function simulate_fall
alpha = pi/4;
r = 1;
m = 1;
x0 = [1 5 0 0]';
h = 0.01;
N = 150;

sys1 = ContactImplicitSystem(@()ball_trough(alpha, r, m), 'ball');
sys1 = sys1.addVisualizer(@(x) ball_trough_visual(x, alpha, r));

[t, x] = sys1.simulate(0.01, N, x0, true, 4);

%qx = x(1,:);
%qy = x(2,:);
%figure(1);
%plot(qx, qy);
%axis equal

end