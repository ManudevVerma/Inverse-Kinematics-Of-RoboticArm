function [value] = Obj_FuncN(theta1,theta2,theta3,Xt,Yt,l1,l2,l3)
%This is the objective function of our project on 3-Dof planer robot

x=l1*cos(theta1)+l2*cos(theta1+theta2) +l3*cos(theta1+theta2+theta3);
y=l1*sin(theta1)+l2*sin(theta1+theta2) +l3*sin(theta1+theta2+theta3);

value = (Xt-x)^2 + (Yt-y)^2;

end