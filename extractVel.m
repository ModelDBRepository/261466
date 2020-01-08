function vx = extractVel(simout)

vel = (simout.state(3,:).^2+simout.state(4,:).^2).^0.5;
[~,imin] = min(vel(41:80));
[maxv,~,~,~] = diffmin(.01,simout.state(3,imin+35:end));


[~,imax] = max(vel(imin+40:end));

vx = [maxv(1) simout.state(3,imin+40+imax)];

end

