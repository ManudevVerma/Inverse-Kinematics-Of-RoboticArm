
% { This is the main code file for the Course project of subject
% Optimization Methods in Engineering - Topic: 3-DOF Planer robot }

clc, clearvars, close all

M=input('> Please enter maximum iterations you want: ');                 % Max iterations for the optimization program
disp(" ")
x=input('> Enter the initial guess(1x3):');                              % Initial position of the arm
disp(" ")
e1=input('> Please enter the termination parameter for gradient: '); 
disp(" ")
Xt=input("> Enter the target X-coordinate: ");
disp(" ")
Yt=input("> Enter the target Y-coordinate: ");
k=0;

%input parameters
l1=1;
l2=1;
l3=1;


if sqrt(Xt^2+Yt^2) <= l1+l2+l3

    plot(Xt, Yt, "*")
    xlabel('X-axis')
    ylabel('Y-axis')
    title('The motion graph')
    hold on
    
    tic 

    theta1=x(1,1);
    theta2=x(1,2);
    theta3=x(1,3);

    % initial value of objective function before optimization

    disp("> Starting value for the objective function: "+string(Obj_FuncN(theta1,theta2,theta3,Xt,Yt,l1,l2,l3)))


    % anonymous function for gradiant computation

    f = @(theta1,theta2,theta3) ((Xt-(l1*cos(theta1)+l2*cos(theta1+theta2) +l3*cos(theta1+theta2+theta3)))^2 + (Yt-(l1*sin(theta1)+l2*sin(theta1+theta2) +l3*sin(theta1+theta2+theta3)))^2);

    [f_theta1, f_theta2, f_theta3] = Func_Gradient(f,theta1,theta2,theta3);

    grad_magd = sqrt((f_theta1)^2+(f_theta2)^2+(f_theta3)^2);

 
    
    syms t1 t2 t3  %symbolic variables for the hessian matrix calculation

    xf=l1*cos(t1)+l2*cos(t1+t2) +l3*cos(t1+t2+t3);
    yf=l1*sin(t1)+l2*sin(t1+t2) +l3*sin(t1+t2+t3);

    g = (Xt-xf)^2 + (Yt-yf)^2;
   
    v=[t1 t2 t3];


    %checking if gradient is smaller than termination parameters:

    while grad_magd >= e1 && k<M

        S=[f_theta1, f_theta2, f_theta3];             %direction vector of Del*f

        H=hessian(g,v);                               % Hessian matrix of function g wrt vector v containig t1 t2 and t3

        H_sub = double(subs( H,v, [x(1) x(2) x(3)])); % datatype conversion from sym to double and value evaluation

        lambda=-(S*S')/(S*H_sub*S');                  % analylitical updation parameter
 
        x = x+lambda*S;                               % updation rule

        % disp("Optimim point in "+string(k+1)+" run is: "+string(rad2deg(x(1)))+"  "+string(rad2deg(x(2)))+"  "+string(rad2deg(x(3)))+" deg ");
        % disp(" ");

        theta1=x(1);
        theta2=x(2);                                  % updated parameters
        theta3=x(3);

        %anonymous function for gradiant computation
        f = @(theta1,theta2,theta3) ((Xt-(l1*cos(theta1)+l2*cos(theta1+theta2) +l3*cos(theta1+theta2+theta3)))^2 + (Yt-(l1*sin(theta1)+l2*sin(theta1+theta2) +l3*sin(theta1+theta2+theta3)))^2);

        [f_theta1, f_theta2, f_theta3] = Func_Gradient(f,theta1,theta2,theta3);

        grad_magd=sqrt((f_theta1)^2+(f_theta2)^2+(f_theta3)^2);

        S = [f_theta1, f_theta2, f_theta3];           % Updating the direction vector

        k = k+1;                                      % Updation of iteration counter

        %Obj_FuncN(theta1,theta2,theta3,Xt,Yt,l1,l2,l3);

        %plot:

        Xp=l1*cos(theta1)+l2*cos(theta1+theta2) +l3*cos(theta1+theta2+theta3); %previous X and Y coordinate for plotting
        Yp=l1*sin(theta1)+l2*sin(theta1+theta2) +l3*sin(theta1+theta2+theta3);

        plot(Xp,Yp,'o')
        hold on
        grid on

    end

    %final optimised values for the angles are
    disp(" ")
    disp(">>> Final optimum values for Xt = "+string(Xt)+" and Yt = "+string(Yt));
    Optimum_Theta1 = rad2deg(x(1,1))
    Optimum_Theta2 = rad2deg(x(1,2))
    Optimum_Theta3 = rad2deg(x(1,3))

    %final value for objective function

    disp(">>> Final value of the objective function after optimization: "+string(Obj_FuncN(x(1),x(2),x(3),Xt,Yt,l1,l2,l3)));
    disp(" ");

    toc

else
    disp(" ");
    disp(">>> Points out of range of the arm! Please enter correct points!! <<< ");

end