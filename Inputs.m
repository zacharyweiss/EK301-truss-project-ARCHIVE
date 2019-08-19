% ITERATION NUMBER
n = 1;
% UPDATE WITH EACH DESIGN TO SAVE TO NEW FILE


% connection matrix
C = [1,1,0,0,0,0,0;1,0,1,0,1,1,0;0,1,1,1,0,0,0;0,0,0,1,1,0,1;0,0,0,0,0,1,1];
[j,m] = size(C); % joint/member count

% unknown forces at joint
Sx = [1,0,0;zeros(4,3)]; 
Sy = [0,1,0;zeros(3);0,0,1]; 

% location vectors by joint, in cm
X = [0,10,5,15,20];
Y = [10,10,5,0,10];

% load vector, Newtons
L = [zeros(j,1);0;0;0;1;0];


%
% ERROR CHECKING
%

if m~=2*j-3
    error('Member/joint count error, check connection matrix dimensions')
elseif length(X)~=j || length(Y)~=j || length(L')~=m+3 || any(size(Sx)~=[j,3]) || any(size(Sy)~=[j,3])
    error('Dimension error')
elseif any(sum(C,1)~=ones(1,m)*2)
    error('Member connected to more than 2 joints')
elseif sum(sum(Sx))~=1 || sum(sum(Sy))~=2
    error('Improper count of reaction forces')
end

%
% END ERROR CHECKING
%

name = sprintf('TrussDesign%d_ZachAmyNatalia_A2.mat',n)
% save parameters
save(name,'C','Sx','Sy','X','Y','L')