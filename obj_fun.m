function [y ] =obj_fun( x )
%Ŀ�꺯���Ļ��ֲ���
% x �ǻ������ޣ�frsq
%y ���ֽ��
% �����޻���
global TL_cap row_TL col_TL qrs delta K n_OD t0
syms t
% x=[1,2,3;4,5,6];
f=t0.*(1+0.15*(t./TL_cap).^4);%pmn

frsk_delta=cell(K,n_OD);
for row=1:K
    for col=1:n_OD
        frsk_delta{row,col}=delta{row,col}.*x(row,col);%��
    end
end
x_mn=zeros( row_TL ,col_TL);%��9*9��
int_xmn_pmn=x_mn;
for row=1:K
    for col=1:n_OD
        x_mn=frsk_delta{row,col}+x_mn;
    end
end
for row=1:row_TL 
    for col=1:col_TL
        if ~isnan(f(row,col))
            int_xmn_pmn(row,col)=int(f(row,col),t,0,x_mn(row,col));%a part of object function,integral
        end
    end
end
y=sum(sum(int_xmn_pmn));%object function �ơ�f(x)dt
% xmn
end

