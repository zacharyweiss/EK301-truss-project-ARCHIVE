% ITERATION NUMBER
n = 1;
% UPDATE TO LOAD NEW ITERATION

warning OFF BACKTRACE
iname = sprintf('TrussDesign%d_ZachAmyNatalia_A2.mat',n);
load(iname,'C','Sx','Sy','X','Y','L')
[j,m] = size(C); % # of joints and members from connection mat dim.

T = zeros(m+3,1); % Tension; magnitudes of unknown forces
A = zeros(2*j,m); % coeff mat filled by loop, final dims will be [2j,m+3]
r = zeros(1,m); % distances, used for failure & cost later

for i = 1:m
    x = X(logical(C(:,i).')); % x vals
    y = Y(logical(C(:,i).')); % y vals
    f = find(C(:,i)); % indices
    
    r(i) = sqrt(((x(1)-x(2))^2)+((y(1)-y(2))^2)); % distance
    if r(i)>16 % TEST: member length within spec?
        warning('m%d longer than allowed 16cm @ %.1fcm',i,r(i))
    elseif r(i)<10
        warning('m%d shorter than allowed 10cm @ %.1fcm',i,r(i))
    end
    
    A(f(1),i) = (x(2)-x(1))/r(i); % x coeff
    A(f(2),i) = A(f(1),i)*(-1); % opp x
    A(j+f(1),i) = (y(2)-y(1))/r(i); % y coeff
    A(j+f(2),i) = A(j+f(1),i)*(-1); % opp y
end
A = [A vertcat(Sx,Sy)]; % create final A mat by concatenating support coeffs (logical)

% magnitudes of unknown forces [m1:end,Sx1,Sy1,Sy2].'
T = inv(A)*L;

% theoretical cost
cost = 10*j+sum(r); % presumes r in cm, $1 per cm length  
if cost>350
    warning('Cost over $350 @ $%.2f',cost)
end

% scaling ratio by member
sr = 0;
for i = find(T<zeros(m+3,1)) % only iterate through members in compression (negative)
    buk(i) = 1440./(r(i).^2); % buckling load by length from curve fit, assumes r in cm
    int(i) = abs(T(i)); % corrected internal force, cannot inline else mat. dim. err w/ division
    sr = max(sr,int(i)/buk(i));
end
if sr>1 % TEST: internal load greater than buckling?
    warning('Bridge will buckle with applied force of %d',max(L))
end
failL = max(L)/sr % failure load, max(L) used to get applied load

%
% OUTPUT
%
oname = sprintf('out%d.txt',n);
fid = fopen(oname,'wt');
fprintf(fid,'\\%% EK301, Section A2, "Truss Us, We''re Engineers": Zachary W, Natalia R, Amy P-Q, ');
    fprintf(fid,datestr(now,'mm/dd/yyyy'));
    fprintf(fid,'.\n');
fprintf(fid,'Load: %d N\n',max(L));
fprintf(fid,'Member forces in Newtons\n');
for i = 1:m
    fprintf(fid,'m%d: %.3f',i,abs(T(i)));
    if T(i) > 0
        fprintf(fid,' (T)\n');
    elseif T == 0
        fprintf(fid,'\n');
    else
        fprintf(fid,' (C)\n');
    end
end
fprintf(fid,'Reaction forces in Newtons:\n');
fprintf(fid,'Sx1: %.1f\n',T(m+1));
fprintf(fid,'Sy1: %.2f\n',T(m+2));
fprintf(fid,'Sy2: %.2f\n',T(m+3));
fprintf(fid,'Cost of truss: $%.0f\n',cost);
fprintf(fid,'Theoretical max load/cost ratio in N/$: %.4f\n',failL/cost);
if fclose(fid) ~= 0
    error('File close failed')
end