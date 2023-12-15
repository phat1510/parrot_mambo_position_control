clc 
close all

%% Extract data
sim_time = tout;
pos_ref_x = closed_loop_sim.Data(:,1);
pos_ref_y = closed_loop_sim.Data(:,2);
pos_ref_z = closed_loop_sim.Data(:,3);
pos_lin_x = closed_loop_sim.Data(:,4);
pos_lin_y = closed_loop_sim.Data(:,5);
pos_lin_z = closed_loop_sim.Data(:,6);
pos_non_lin_x = closed_loop_sim.Data(:,7);
pos_non_lin_y = closed_loop_sim.Data(:,8);
pos_non_lin_z = closed_loop_sim.Data(:,9);


q1 = false;
q2 = false;
q3 = false;
q31 = false;
q32 = false;
q4 = false;
q5 = true;
q6 = false;

%% Question 3
%% 3.1 Apply LQR for linear discreted model M_LD

% Plot position outputs of linear model vs references
if(q31)
    figure(1)
    plot(sim_time, pos_ref_x, '--', ...
        sim_time, pos_ref_y, '--',...
        sim_time, pos_ref_z, '--');
    hold on
    
    plot(sim_time, pos_lin_x, ...
        sim_time, pos_lin_y, ...
        sim_time, pos_lin_z);
    hold on
    ylim([0 2.2]);
    ylabel('$Position \; (meter)$','interpreter','latex','FontSize',12);
    xlabel('$Time \; (second)$','interpreter','latex','FontSize',12);
    legend("x\_ref","y\_ref", "z\_ref","x\_lin","y\_lin", "z\_lin",'interpreter','latex','FontSize',12)
    grid on;
    grid minor;
    
    %Plot errors
    error_x = pos_ref_x - pos_lin_x;
    error_y = pos_ref_y - pos_lin_y;
    error_z = pos_ref_z - pos_lin_z;
    
    figure(2)
    plot(sim_time, error_x, ...
        sim_time, error_y, ...
        sim_time, error_z);
    ylim([0 2.2]);
    ylabel('$Error \; (meter)$','interpreter','latex','FontSize',12);
    xlabel('$Time \; (second)$','interpreter','latex','FontSize',12);
    legend("error\_x","error\_y", " error\_z",'interpreter','latex','FontSize',12)
    grid on;
    grid minor;
end
  


%% 3.2 Use the propose LQR controller for the non-linear model M_NL
% Plot position outputs of non-linear model vs references
if(q32)
    figure(1)
    plot(sim_time, pos_ref_x, '--', ...
        sim_time, pos_ref_y, '--',...
        sim_time, pos_ref_z, '--');
    hold on
    
    plot(sim_time, pos_non_lin_x, ...
        sim_time, pos_non_lin_y, ...
        sim_time, pos_non_lin_z);
    hold on
    ylim([-0.6 2.2]);
    ylabel('$Position \; (meter)$','interpreter','latex','FontSize',12);
    xlabel('$Time \; (second)$','interpreter','latex','FontSize',12);
    legend("x\_ref","y\_ref", "z\_ref","x\_nonlin","y\_nonlin", "z\_nonlin",'interpreter','latex','FontSize',12)
    grid on;
    grid minor;
    
    %Plot errors
    error_x = pos_ref_x - pos_non_lin_x;
    error_y = pos_ref_y - pos_non_lin_y;
    error_z = pos_ref_z - pos_non_lin_z;
    
    figure(2)
    plot(sim_time, error_x, ...
        sim_time, error_y, ...
        sim_time, error_z);
    
    ylabel('$Error \; (meter)$','interpreter','latex','FontSize',12);
    xlabel('$Time \; (second)$','interpreter','latex','FontSize',12);
    legend("error x","error y", " error z",'interpreter','latex','FontSize',12)
    grid on;
    grid minor;
    % Comparision between reference input and output of linear and non-linear
    % models
    figure(3)
    plot3(pos_ref_x, pos_ref_y, pos_ref_z);
    hold on
    plot3(pos_lin_x, pos_lin_y, pos_lin_z);
    hold on
    plot3(pos_non_lin_x, pos_non_lin_y, pos_non_lin_z);
    zlim([-0.1 2.5]);
    
    ylabel('$x \; (meter)$','interpreter','latex','FontSize',12);
    xlabel('$y \; (meter)$','interpreter','latex','FontSize',12);
    zlabel('$z \; (meter)$','interpreter','latex','FontSize',12);
    legend("ref","pos\_lin", "pos\_nonlin",'interpreter','latex','FontSize',12)
    
    grid on
    grid minor;
end

%% 5.1 to 5.3
if(q5)
    % Dynamic plot
    figure(1)
    view(3);
    for k = 1:length(pos_non_lin_x)
        clf();
        plot3(pos_non_lin_x(k),pos_non_lin_y(k),pos_non_lin_z(k),'pentagram','LineWidth',3);
        hold on
        plot3(pos_ref_x, pos_ref_y, pos_ref_z);
        hold on
        plot3(pos_non_lin_x(1:k),pos_non_lin_y(1:k),pos_non_lin_z(1:k),'LineWidth',1);
        axis([-3 5 -3 5 -0.5 z0+0.1]);
        grid on
         drawnow limitrate
    end
    drawnow

    % Static plot
    figure(2)
    plot(sim_time, pos_ref_x, '--', ...
        sim_time, pos_ref_y, '--',...
        sim_time, pos_ref_z, '--');
    hold on
    
    plot(sim_time, pos_non_lin_x, ...
        sim_time, pos_non_lin_y, ...
        sim_time, pos_non_lin_z);
    hold on
    ylim([-0.1 1.2]);
    ylabel('$Position \; (meter)$','interpreter','latex','FontSize',12);
    xlabel('$Time \; (second)$','interpreter','latex','FontSize',12);
    legend("x\_ref","y\_ref", "z\_ref","x\_mambo","y\_mambo", "z\_mambo",'interpreter','latex','FontSize',12)
    grid on;
    grid minor;
    
    %Plot errors
    error_x = pos_ref_x - pos_non_lin_x;
    error_y = pos_ref_y - pos_non_lin_y;
    error_z = pos_ref_z - pos_non_lin_z;
    
    figure(3)
    plot(sim_time, error_x, ...
        sim_time, error_y, ...
        sim_time, error_z);
    
    ylabel('$Error \; (meter)$','interpreter','latex','FontSize',12);
    xlabel('$Time \; (second)$','interpreter','latex','FontSize',12);
    legend("error x","error y", " error z",'interpreter','latex','FontSize',12)
    grid on;
    grid minor;
    % Comparision between reference input and output of linear and non-linear
    % models
    figure(4)
    % Draw obstacle 1
    obs1_x = 0.8;
    obs1_y = -0.81;
    obs1 = [obs1_x obs1_y 0;
        obs1_x obs1_y 1.3];
    plot3(obs1(:,1),obs1(:,2), obs1(:,3), LineWidth=3);
    hold on

    % Draw obstacle 2
    obs2_x = -1.16;
    obs2_y = 0;
    obs2_z = 0.83;
    off_x = 0.3;
    off_z = 0.1;
    obs2 = [obs2_x-off_x obs2_y obs2_z-off_z;
        obs2_x+off_x obs2_y obs2_z-off_z;
        obs2_x+off_x obs2_y obs2_z+off_z;
        obs2_x-off_x obs2_y obs2_z+off_z;
        obs2_x-off_x obs2_y obs2_z-off_z];

    plot3(obs2(:,1),obs2(:,2), obs2(:,3), LineWidth=3);
    hold on

    % Draw obstacle 3
    obs3_x = 0;
    obs3_y = 1.19;
    obs3_z = 0.13;
    off_y = 0.3;
    off_z = 0.1;
    obs3 = [obs3_x obs3_y-off_y obs3_z-off_z;
        obs3_x obs3_y+off_y obs3_z-off_z;
        obs3_x obs3_y+off_y obs3_z+off_z;
        obs3_x obs3_y-off_y obs3_z+off_z;
        obs3_x obs3_y-off_y obs3_z-off_z];
    plot3(obs3(:,1),obs3(:,2), obs3(:,3), LineWidth=3);
    hold on

    plot3(pos_ref_x, pos_ref_y, pos_ref_z);
    hold on
    plot3(pos_non_lin_x, pos_non_lin_y, pos_non_lin_z);
    axis([x0-0.3 x0+3 y0-0.3 y0+3 -0.1 1]);
    
    xlabel('$x \; (meter)$','interpreter','latex','FontSize',12);
    ylabel('$y \; (meter)$','interpreter','latex','FontSize',12);
    zlabel('$z \; (meter)$','interpreter','latex','FontSize',12);
    legend("obs1","obs2","obs3","ref","pos\_mambo",'interpreter','latex','FontSize',12)
    
    grid on
    grid minor;
end


%% Plot question 2
if(q2)
    sim_time = tout;
    pos_open_x = open_loop_pos.Data(:,1);
    pos_open_y = open_loop_pos.Data(:,2);
    pos_open_z = open_loop_pos.Data(:,3);
    input_roll = open_loop_pos.Data(:,4);
    input_pitch = open_loop_pos.Data(:,5);
    input_thrust = open_loop_pos.Data(:,6);

    figure(1)
    subplot(1,2,2)
    plot(sim_time, pos_open_x, ...
        sim_time, pos_open_y,...
        sim_time, pos_open_z);
    ylabel('$Position \; (meter)$','interpreter','latex','FontSize',12);
    xlabel('$Time \; (second)$','interpreter','latex','FontSize',12);
    legend("x\_open","y\_open", "z\_open",'interpreter','latex','FontSize',12)
    subplot(1,2,1);
    plot(sim_time, input_roll, ...
        sim_time, input_pitch,...
        sim_time, input_thrust);
    legend("input_roll","input_pitch", "input_thrust",'interpreter','latex','FontSize',12);
    ylabel('$ \;$','interpreter','latex','FontSize',12);
    xlabel('$Time \; (second)$','interpreter','latex','FontSize',12);
    

    grid on;
    grid minor;
end
