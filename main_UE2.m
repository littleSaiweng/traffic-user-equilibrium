clc;clear;
global TL_cap row_TL col_TL qrs delta K n_OD t0
TL=[0,2,Inf,2,1.50000000000000,Inf,Inf,Inf,Inf;
    2,0,2,Inf,2,Inf,Inf,Inf,Inf;
    Inf,2,0,Inf,1.50000000000000,2,Inf,Inf,Inf;
    2,Inf,Inf,0,1,Inf,2,Inf,Inf;
    1.50000000000000,2,1.50000000000000,1,0,1,1.50000000000000,2,1.50000000000000;
    Inf,Inf,2,Inf,1,0,Inf,Inf,2;
    Inf,Inf,Inf,2,1.50000000000000,Inf,0,2,Inf;
    Inf,Inf,Inf,Inf,2,Inf,2,0,2;
    Inf,Inf,Inf,Inf,1.50000000000000,2,Inf,2,0];% distance between two nodes

[row_TL,col_TL]=size(TL);
% TL_cap=TL*5;TL_cap(TL_cap>100000|TL_cap==0)=NaN;%road capacity
TL_cap=TL*2;TL_cap(TL_cap>1000000)=NaN;
t0=TL/2;
t0(t0>3100000)=NaN;
ODpairs=[1,9,10;2,7,12;3,7,8];
qrs=ODpairs(:,3)';
n_OD=size(ODpairs,1);
K=2;
%K row OD column
Qrs=cell(K,n_OD);%paths or routes
for n=1:n_OD
    [Paths, totalCosts] = kShortestPath(TL, ODpairs(n,1),ODpairs(n,2),K);
    Qrs(:,n)=Paths;%
end
delta=cell(K,n_OD);% =¡¾k*n od¡¿
for nth_od=1:n_OD%n OD
    for route=1:K
        npoint=size(Qrs{route,nth_od},2);
        delta{route,nth_od}=zeros(row_TL,col_TL);
        for q=1:npoint-1
            point1=Qrs{route,nth_od}(1,q);
            point2=Qrs{route,nth_od}(1,q+1);
            delta{route,nth_od}(point1,point2)=1;%the first path
        end
    end
end


Aeq=[];beq=[];
A=[];b=[];
 lb=zeros(K,n_OD);ub=repmat(qrs,K,1);
% lb=[];ub=[];
x0=zeros(K,n_OD);
fun=@obj_fun;
nonlcon=@s_t_ ; %¡ÌNonlinear constraint
 option=optimoptions(@fmincon,'PlotFcn','optimplotfval');%optimplotx
[frsk,fval,exitflag,output] =fmincon(fun,x0,A,b,Aeq,beq,lb,ub,nonlcon, option)


