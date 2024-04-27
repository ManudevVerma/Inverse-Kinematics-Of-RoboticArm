%Numeric Gradient of the objective function using 2-point central
%difference forula

function [f_theta1, f_theta2, f_theta3] = Func_Gradient(f,theta1,theta2,theta3)

function f_theta1 = F_t1(f,theta1,theta2,theta3)
del_t1 = 0.00001;
f_theta1 = (f(theta1+del_t1,theta2, theta3) - f(theta1-del_t1,theta2,theta3))/(2*del_t1);
end

function f_theta2 = F_t2(f,theta1,theta2,theta3)
del_t2 = 0.00001;
f_theta2 = (f(theta1,theta2+del_t2, theta3) - f(theta1,theta2-del_t2,theta3))/(2*del_t2);
end

function f_theta3 = F_t3(f,theta1,theta2,theta3)
del_t3 = 0.00001;
f_theta3 = (f(theta1,theta2, theta3+del_t3) - f(theta1,theta2,theta3-del_t3))/(2*del_t3);
end

f_theta1 = F_t1(f,theta1,theta2,theta3);
f_theta2 = F_t2(f,theta1,theta2,theta3);
f_theta3 = F_t3(f,theta1,theta2,theta3);

end
