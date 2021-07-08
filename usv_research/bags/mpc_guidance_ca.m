%declare name of the bag
experimentbag = rosbag('mpc_guidance_exp1/sim_2021-07-07-18-20-16.bag');
desiredheading = select(experimentbag, "Topic", '/guidance/desired_heading');
desiredheadingts = timeseries(desiredheading, 'Data');
start_time = desiredheadingts.get.TimeInfo.Start;
end_time = desiredheadingts.get.TimeInfo.End;

%heading gain plot
heading = select(experimentbag, "Topic", '/usv_control/asmc/heading_gain');
headingts = timeseries(heading, 'Data');
headingts = getsampleusingtime(headingts, start_time, end_time);
t = headingts.get.Time - start_time;
headingdata = headingts.get.Data;
figure
plot(t,headingdata)
legend('$heading gain$', 'Interpreter', 'latex')
xlabel('Time [s]', 'Interpreter', 'latex') 
ylabel('$\psi$ [rad]', 'Interpreter', 'latex')
title('Heading Gain')

%heading sigma plot
heading = select(experimentbag, "Topic", '/usv_control/asmc/heading_sigma');
headingts = timeseries(heading, 'Data');
headingts = getsampleusingtime(headingts, start_time, end_time);
t = headingts.get.Time - start_time;
headingdata = headingts.get.Data;
figure
plot(t,headingdata)
legend( '$\heading sigma$', 'Interpreter', 'latex')
xlabel('Time [s]', 'Interpreter', 'latex') 
ylabel('$\psi$ [rad]', 'Interpreter', 'latex')
title('Heading Sigma')

%control input plot
heading = select(experimentbag, "Topic", '/usv_control/controller/control_input');
headingts = timeseries(heading, 'Theta');
headingts = getsampleusingtime(headingts, start_time, end_time);
t = headingts.get.Time - start_time;
headingdata = headingts.get.Data;
figure
plot(t,headingdata)
hold off
legend('$\control input$', 'Interpreter', 'latex')
xlabel('Time [s]', 'Interpreter', 'latex') 
ylabel('$\psi$ [rad]', 'Interpreter', 'latex')
title('Control input')

%heading plot
%heading = select(experimentbag, "Topic", '/vectornav/ins_2d/local_vel');
%headingts = timeseries(heading, 'Z');
%t = headingts.get.Time - start_time;
%headingdata = headingts.get.Data;
%figure
%plot(t,headingdata)
%hold on
%desired heading plot
%desiredheading = select(experimentbag, "Topic", '/guidance/desired_r');
%desiredheadingts = timeseries(desiredheading, 'Data');
%t = desiredheadingts.get.Time - start_time;
%desiredheadingdata = desiredheadingts.get.Data;
%plot(t,desiredheadingdata)
%hold off
%legend('$\r$','$\r_{d}$', 'Interpreter', 'latex')
%xlabel('Time [s]', 'Interpreter', 'latex') 
%ylabel('$\psi$ [rad]', 'Interpreter', 'latex')
%title(' Angular vel MPC')

%heading plot
heading = select(experimentbag, "Topic", '/vectornav/ins_2d/NED_pose');
headingts = timeseries(heading, 'Theta');
headingts = getsampleusingtime(headingts, start_time, end_time);
t = headingts.get.Time - start_time;
headingdata = headingts.get.Data;
figure
plot(t,headingdata)
hold on
%desired heading plot
desiredheading = select(experimentbag, "Topic", '/guidance/desired_heading');
desiredheadingts = timeseries(desiredheading, 'Data');
headingts = getsampleusingtime(headingts, start_time, end_time);
t = desiredheadingts.get.Time - start_time;
desiredheadingdata = desiredheadingts.get.Data;
plot(t,desiredheadingdata)
hold off
legend('$\psi$','$\psi_{d}$', 'Interpreter', 'latex')
xlabel('Time [s]', 'Interpreter', 'latex') 
ylabel('$\psi$ [rad]', 'Interpreter', 'latex')
title('Heading MPC')

%x plot
figure
desiredheading = select(experimentbag, "Topic", '/vectornav/ins_2d/NED_pose');
desiredheadingts = timeseries(desiredheading, 'X');
desiredheadingts = getsampleusingtime(desiredheadingts, start_time, end_time);
t = desiredheadingts.get.Time - start_time;
desiredheadingdata = desiredheadingts.get.Data;
plot(t,desiredheadingdata)
hold on
%y plot
desiredheading = select(experimentbag, "Topic", '/vectornav/ins_2d/NED_pose');
desiredheadingts = timeseries(desiredheading, 'Y');
desiredheadingts = getsampleusingtime(desiredheadingts, start_time, end_time);
t = desiredheadingts.get.Time - start_time;
desiredheadingdata = desiredheadingts.get.Data;
plot(t,desiredheadingdata)
hold off
legend('$\X$','$\Y$', 'Interpreter', 'latex')
xlabel('Time [s]', 'Interpreter', 'latex') 
ylabel('$\Pose$ [m]', 'Interpreter', 'latex')
title('Pose')

%right thruster plot
right = select(experimentbag, "Topic", '/usv_control/controller/right_thruster');
rightts = timeseries(right, 'Data');
rightts = getsampleusingtime(rightts, start_time, end_time);
t = rightts.get.Time - start_time;
rightdata = rightts.get.Data;
figure
plot(t,rightdata)
hold on
%left thruster plot
left = select(experimentbag, "Topic", '/usv_control/controller/left_thruster');
leftts = timeseries(left, 'Data');
leftts = getsampleusingtime(leftts, start_time, end_time);
t = leftts.get.Time - start_time;
leftdata = leftts.get.Data;
plot(t,leftdata)
hold off
legend('$T_{stbd}$','$T_{port}$', 'Interpreter', 'latex')
xlabel('Time [s]', 'Interpreter', 'latex') 
ylabel('Thrust [N]', 'Interpreter', 'latex') 
title('Thruster MPC')
%speed plot
speed = select(experimentbag, "Topic", '/vectornav/ins_2d/local_vel');
speedts = timeseries(speed, 'X');
speedts = getsampleusingtime(speedts, start_time, end_time);
t = speedts.get.Time - start_time;
speeddata = speedts.get.Data;
figure
plot(t,speeddata)
hold on
%desired speed plot
desiredspeed = select(experimentbag, "Topic", '/guidance/desired_speed');
desiredspeedts = timeseries(desiredspeed, 'Data');
desiredspeedts = getsampleusingtime(desiredspeedts, start_time, end_time);
t = desiredspeedts.get.Time - start_time;
desiredspeeddata = desiredspeedts.get.Data;
plot(t,desiredspeeddata)
hold off
legend('$u$','$u_{d}$', 'Interpreter', 'latex')
xlabel('Time [s]', 'Interpreter', 'latex') 
ylabel('$u$ [m/s]', 'Interpreter', 'latex') 
title('Speed MPC')

%Publish desired Path
%desired_path = select(experimentbag, 'Topic', '/guidance/target');
%msgStructs = readMessages(desired_path,'DataFormat','struct');
%%msgStructs{1};
%figure
%xPoints = cellfun(@(m) double(m.X),msgStructs);
%yPoints = cellfun(@(m) double(m.Y),msgStructs);
%plot(yPoints,xPoints)
%hold on 

desired_path = select(experimentbag, 'Topic', '/guidance/target');
desiredxs = timeseries(desired_path, 'X');
desiredxs = getsampleusingtime(desiredxs, start_time-10, start_time);
desiredxdata = desiredxs.get.Data;

desiredys = timeseries(desired_path, 'Y');
desiredys = getsampleusingtime(desiredys, start_time-10, start_time);
desiredydata = desiredys.get.Data;
figure
plot(desiredydata,desiredxdata)
hold on


%publish actual path
position = select(experimentbag, 'Topic', '/vectornav/ins_2d/NED_pose');
msgStructs = readMessages(position,'DataFormat','struct');
%msgStructs{1};
xlim([-30 0]), ylim([0 10])
xPoints = cellfun(@(m) double(m.X),msgStructs);
yPoints = cellfun(@(m) double(m.Y),msgStructs);
plot(yPoints,xPoints)
xlabel('Y(m)', 'Interpreter', 'latex') 
ylabel('X(m)', 'Interpreter', 'latex')
legend('Desired Path', 'Actual Path', 'Interpreter', 'latex')
title('Path')
hold on

%publish obstacles
position = select(experimentbag, 'Topic', '/nmpc_ca/obstacle_list');
msgStructs = readMessages(position,'DataFormat','struct');
%figure
%xlim([-30 0]), ylim([0 10])
xobs = zeros(8,length(msgStructs));
yobs = zeros(8,length(msgStructs));
zobs = zeros(8,length(msgStructs));
for j=1:length(msgStructs)
    for i=1:8
        %fprintf('length %i index i %i index j %i. \n', length(msgStructs{j}.Obstacles), i ,j)
        if length(msgStructs{j}.Obstacles)>0
            xobs(i, j) = msgStructs{j}.Obstacles(i).X;
            yobs(i, j) = msgStructs{j}.Obstacles(i).Y;
            zobs(i, j) = msgStructs{j}.Obstacles(i).Z;
            if zobs(i,j)>0
                %zobs(i, j) = 0.105;
                %viscircles([yobs(i, j), xobs(i, j)], zobs(i, j));
                rectangle('Position',[yobs(i, j)-zobs(i,j),xobs(i, j)-zobs(i,j),2*zobs(i,j),2*zobs(i,j)],'Curvature',[1,1],'FaceColor','r','EdgeColor', 'r')
            end
        end
    end
end

%xlabel('Y(m)', 'Interpreter', 'latex') 
%ylabel('X(m)', 'Interpreter', 'latex')
title('Obstacles')

%speed gain plot
heading = select(experimentbag, "Topic", '/usv_control/asmc/speed_gain');
headingts = timeseries(heading, 'Data');
headingts = getsampleusingtime(headingts, start_time, end_time);
t = headingts.get.Time - start_time;
headingdata = headingts.get.Data;
figure
plot(t,headingdata)
legend('$speed gain$', 'Interpreter', 'latex')
xlabel('Time [s]', 'Interpreter', 'latex') 
ylabel('$u$ [m/s]', 'Interpreter', 'latex')
title('Speed Gain')

%speed sigma plot
heading = select(experimentbag, "Topic", '/usv_control/asmc/speed_sigma');
headingts = timeseries(heading, 'Data');
headingts = getsampleusingtime(headingts, start_time, end_time);
t = headingts.get.Time - start_time;
headingdata = headingts.get.Data;
figure
plot(t,headingdata)
legend('$speed sigma$', 'Interpreter', 'latex')
xlabel('Time [s]', 'Interpreter', 'latex') 
ylabel('$u$ [m/s]', 'Interpreter', 'latex')
title('Speed Sigma')

heading = select(experimentbag, "Topic", '/usv_control/controller/heading_error');
headingts = timeseries(heading, 'Data');
headingts = getsampleusingtime(headingts, start_time, end_time);
t = headingts.get.Time - start_time;
headingdata = headingts.get.Data;
figure
plot(t,headingdata)
legend( '$\heading error$', 'Interpreter', 'latex')
xlabel('Time [s]', 'Interpreter', 'latex') 
ylabel('$\psi$ [rad]', 'Interpreter', 'latex')
title('Heading Error')

heading = select(experimentbag, "Topic", '/usv_control/controller/speed_error');
headingts = timeseries(heading, 'Data');
headingts = getsampleusingtime(headingts, start_time, end_time);
t = headingts.get.Time - start_time;
headingdata = headingts.get.Data;
figure
plot(t,headingdata)
legend( '$\speed error$', 'Interpreter', 'latex')
xlabel('Time [s]', 'Interpreter', 'latex') 
ylabel('$\psi$ [rad]', 'Interpreter', 'latex')
title('Speed Error')




