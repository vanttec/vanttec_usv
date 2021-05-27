%declare name of the bag
experimentbag = rosbag('mpc_guidance_exp0/sim_2021-05-26-23-35-03.bag');
desiredheading = select(experimentbag, "Topic", '/guidance/desired_heading');
desiredheadingts = timeseries(desiredheading, 'Data');
start_time = desiredheadingts.get.TimeInfo.Start;

%heading gain plot
heading = select(experimentbag, "Topic", '/usv_control/asmc/heading_gain');
headingts = timeseries(heading, 'Data');
t = headingts.get.Time - start_time;
headingdata = headingts.get.Data;
figure
plot(t,headingdata)
legend('$\heading gain$', 'Interpreter', 'latex')
xlabel('Time [s]', 'Interpreter', 'latex') 
ylabel('$\psi$ [rad]', 'Interpreter', 'latex')
title('Heading Gain')

%heading sigma plot
heading = select(experimentbag, "Topic", '/usv_control/asmc/heading_sigma');
headingts = timeseries(heading, 'Data');
t = headingts.get.Time - start_time;
headingdata = headingts.get.Data;
figure
plot(t,headingdata)
hold on
heading = select(experimentbag, "Topic", '/usv_control/controller/heading_error');
headingts = timeseries(heading, 'Data');
t = headingts.get.Time - start_time;
headingdata = headingts.get.Data;
plot(t,headingdata)
hold off
legend('$\heading sigma$', '$\heading error$', 'Interpreter', 'latex')
xlabel('Time [s]', 'Interpreter', 'latex') 
ylabel('$\psi$ [rad]', 'Interpreter', 'latex')
title('Heading Sigma')

%control input plot
heading = select(experimentbag, "Topic", '/usv_control/controller/control_input');
headingts = timeseries(heading, 'Theta');
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
t = headingts.get.Time - start_time;
headingdata = headingts.get.Data;
figure
plot(t,headingdata)
hold on
%desired heading plot
desiredheading = select(experimentbag, "Topic", '/guidance/desired_heading');
desiredheadingts = timeseries(desiredheading, 'Data');
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
%y plot
desiredheading = select(experimentbag, "Topic", '/vectornav/ins_2d/NED_pose');
desiredheadingts = timeseries(desiredheading, 'Y');
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
t = rightts.get.Time - start_time;
rightdata = rightts.get.Data;
figure
plot(t,rightdata)
hold on
%left thruster plot
left = select(experimentbag, "Topic", '/usv_control/controller/left_thruster');
leftts = timeseries(left, 'Data');
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
t = speedts.get.Time - start_time;
speeddata = speedts.get.Data;
figure
plot(t,speeddata)
hold on
%desired speed plot
desiredspeed = select(experimentbag, "Topic", '/guidance/desired_speed');
desiredspeedts = timeseries(desiredspeed, 'Data');
t = desiredspeedts.get.Time - start_time;
desiredspeeddata = desiredspeedts.get.Data;
plot(t,desiredspeeddata)
hold off
legend('$u$','$u_{d}$', 'Interpreter', 'latex')
xlabel('Time [s]', 'Interpreter', 'latex') 
ylabel('$u$ [m/s]', 'Interpreter', 'latex') 
title('Speed MPC')

%Publish desired Path
desired_path = select(experimentbag, 'Topic', '/guidance/target');
msgStructs = readMessages(desired_path,'DataFormat','struct');
msgStructs{1};
figure
xPoints = cellfun(@(m) double(m.X),msgStructs);
yPoints = cellfun(@(m) double(m.Y),msgStructs);
plot(yPoints,xPoints)
xlabel('Y(m)', 'Interpreter', 'latex') 
ylabel('X(m)', 'Interpreter', 'latex')
hold on 

%publish actual path
position = select(experimentbag, 'Topic', '/vectornav/ins_2d/ins_pose');
msgStructs = readMessages(position,'DataFormat','struct');
msgStructs{1};
xPoints = cellfun(@(m) double(m.X),msgStructs);
yPoints = cellfun(@(m) double(m.Y),msgStructs);
plot(yPoints,xPoints)
legend('Desired Path', 'Actual Path', 'Interpreter', 'latex')

%publish obstacles
position = select(experimentbag, 'Topic', '/nmpc_ca/obstacle_list');
msgStructs = readMessages(position,'DataFormat','struct');
figure
xPoints = zeros(8,length(msgStructs));
yPoints = zeros(8,length(msgStructs));
zPoints = zeros(8,length(msgStructs));
for j=1:length(msgStructs)
    for i=1:8
        xPoints(i, j) = msgStructs{200}.Obstacles(i).X;
        yPoints(i, j) = msgStructs{200}.Obstacles(i).Y;
        zPoints(i, j) = msgStructs{200}.Obstacles(i).Z;
        viscircles([xPoints(i, j), yPoints(i, j)], zPoints(i, j));
    end
end
xlabel('Y(m)', 'Interpreter', 'latex') 
ylabel('X(m)', 'Interpreter', 'latex')
