function [c,ceq] =s_t_(x)
%K row, OD Column
% ¡Æk frs_k=qrs
% frs_k>=0
global TL_cap row_TL col_TL qrs delta K n_OD t0
n=size(x,1);A=ones(1,n);
%x:frs_k
ceq=A*x-qrs;
c =-x;
