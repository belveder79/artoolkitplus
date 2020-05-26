%%
[~, loaded_mexes] = inmem('-completenames');
for i=1:length(loaded_mexes)
    clear loaded_mexes{i};
end
%[~, loaded_mexes] = inmem('-completenames');
%assert(isempty(loaded_mexes))
delete robust_pose.mexmaci64
clear robust_pose.mexmaci64

mex -I../ -I../../../include robust_pose.cpp ../rpp.cpp ../rpp_vecmat.cpp ../rpp_quintic.cpp ../rpp_svd.cpp
%%
iprts = [0.14833333333333332, 0.023333333333333345, 1;
    0.1133333333333334, 0.023333333333333345, 1;
  0.1133333333333334, -0.011666666666666659, 1;
  0.14833333333333332, -0.011666666666666659,1]';

model = [77219.265625, -42.853523254394531, 0;
77134.265625, -42.853523254394531, 0;
77134.265625, 42.146476745605469, 0;
77219.265625, 42.146476745605469, 0]';

n = 0;
[R,t,err] = robust_pose(circshift(model,n,2),circshift(iprts,n,2));

K = [1200 0 480; 0 1200 270; 0 0 1];

P = [R t];
ptsInC = P * [model; ones(1,4)];
ptsInC(1:2,:)./repmat(ptsInC(3,:),2,1)
iprts(1:2,:)

q = dcm2quat(R); q(1) = -q(1); q(4) = -q(4);
Rn = quat2dcm(q);
P = [Rn -t];
ptsInC = P * [model; ones(1,4)];
ptsInC(1:2,:)./repmat(ptsInC(3,:),2,1)
iprts(1:2,:)

v21 = model(:,1) - model(:,2); v21 = v21./norm(v21);
v23 = model(:,3) - model(:,2); v23 = v23./norm(v23);
vx = cross(v21,v23);
figure(2); clf; hold on;
plot3([0,v21(1)],[0,v21(2)],[0,v21(3)],'r-');
plot3([0,v23(1)],[0,v23(2)],[0,v23(3)],'g-');
plot3([0,vx(1)],[0,vx(2)],[0,vx(3)],'b-');
xlabel('x'); ylabel('y'); zlabel('z');
hold off;


%%
minErr = 10;
for n = 0:3
    [R,t,err] = robust_pose(circshift(model,n,2),circshift(iprts,n,2))
    if err < minErr
        bestR = R;
        bestt = t;
        minErr = err;
    end
end