function n=countless(x,y)
%Usage:  n=countless(x,y)
%n(i)= # elements of x < y(i)
[MX,MY]=ndgrid(x,y);
%each column of MX = x
%each row of MY = y
n=(sum((MX<MY),1))';
