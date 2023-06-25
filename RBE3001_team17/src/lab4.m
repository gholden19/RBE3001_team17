%%
% RBE3001 - Laboratory 1 
% 
% Instructions
% ------------
% Welcome again! This MATLAB script is your starting point for Lab
% 1 of RBE3001. The sample code below demonstrates how to establish
% communication between this script and the Nucleo firmware, send
% setpoint commands and receive sensor data.
% 
% IMPORTANT - understanding the code below requires being familiar
% with the Nucleo firmware. Read that code first.

% Lines 15-37 perform necessary library initializations. You can skip reading
% to line 38
tic
%delete all active timers. This is necessary to stop timers which may have
%been left running if the robot was force-quit without cleanup
listOfTimers = timerfindall;
if ~isempty(listOfTimers)
    delete(listOfTimers(:));
end
clear("Robot")
clear("Model")
clear java
clear classes;

vid = hex2dec('16c0');
pid = hex2dec('0486');

disp (vid);
disp (pid);

javaaddpath ../lib/SimplePacketComsJavaFat-0.6.4.jar;
javaaddpath(pwd);
import edu.wpi.SimplePacketComs.*;
import edu.wpi.SimplePacketComs.device.*;
import edu.wpi.SimplePacketComs.phy.*;
import java.util.*;
import org.hid4java.*;
import java.util.LinkedList;
version -java
myHIDSimplePacketComs=HIDfactory.get();
myHIDSimplePacketComs.setPid(pid);
myHIDSimplePacketComs.setVid(vid);
myHIDSimplePacketComs.connect();
common

% Create a PacketProcessor object to send data to the nucleo firmware
global robot;
robot= Robot(myHIDSimplePacketComs);
%clean=onCleanup(@(robot)cleanup)
%robot.start_profiler();
state = STATE.init_stl_plot;
pose=[0, 0, 0; -63, 45, -10; -90, 86.14, 33.71];


%draw a square
currentX=0;
currentY=-50;
currentZ=14.79;
trajectoryX=robot.cubic_traj(currentX,0,0,2,0,0);
trajectoryY=robot.cubic_traj(currentY,-130,0,2,0,0);
trajectoryZ=robot.cubic_traj(currentZ,14.9,0,2,0,0);
trajectory1=Trajectory([trajectoryX';trajectoryY';trajectoryZ'],2,false);
trajectoryX=robot.cubic_traj(0,105,0,2,0,0);
trajectoryY=robot.cubic_traj(-130,-130,0,2,0,0);
trajectoryZ=robot.cubic_traj(14.79,14.79,0,2,0,0);
trajectory2=Trajectory([trajectoryX';trajectoryY';trajectoryZ'],2,false);
trajectoryX=robot.cubic_traj(105,105,0,2,0,0);
trajectoryY=robot.cubic_traj(-130,-50,0,2,0,0);
trajectoryZ=robot.cubic_traj(14.79,14.79,0,2,0,0);
trajectory3=Trajectory([trajectoryX';trajectoryY';trajectoryZ'],2,false);
trajectoryX=robot.cubic_traj(105,0,0,2,0,0);
trajectoryY=robot.cubic_traj(-50,-50,0,2,0,0);
trajectoryZ=robot.cubic_traj(14.79,14.79,0,2,0,0);
trajectory4=Trajectory([trajectoryX';trajectoryY';trajectoryZ'],2,false);
trajectoryX=robot.cubic_traj(0,0,0,2,0,0);
trajectoryY=robot.cubic_traj(-50,-50,0,2,0,0);
trajectoryZ=robot.cubic_traj(14.79,99,0,2,0,0);
trajectory5=Trajectory([trajectoryX';trajectoryY';trajectoryZ'],2,false);
trajectoryX=robot.cubic_traj(0,0,0,2,0,0);
trajectoryY=robot.cubic_traj(-50,-130,0,2,0,0);
trajectoryZ=robot.cubic_traj(99,99,0,2,0,0);
trajectory6=Trajectory([trajectoryX';trajectoryY';trajectoryZ'],2,false);
trajectoryX=robot.cubic_traj(0,0,0,2,0,0);
trajectoryY=robot.cubic_traj(-130,-130,0,2,0,0);
trajectoryZ=robot.cubic_traj(99,99,14.79,2,0,0);
trajectory7=Trajectory([trajectoryX';trajectoryY';trajectoryZ'],2,false);
trajectoryX=robot.cubic_traj(0,0,0,2,0,0);
trajectoryY=robot.cubic_traj(-130,-130,0,2,0,0);
trajectoryZ=robot.cubic_traj(14.79,99,0,2,0,0);
trajectory8=Trajectory([trajectoryX';trajectoryY';trajectoryZ'],2,false);
trajectoryX=robot.cubic_traj(0,105,0,2,0,0);
trajectoryY=robot.cubic_traj(-130,-130,0,2,0,0);
trajectoryZ=robot.cubic_traj(99,99,0,2,0,0);
trajectory9=Trajectory([trajectoryX';trajectoryY';trajectoryZ'],2,false);
trajectoryX=robot.cubic_traj(105,105,0,2,0,0);
trajectoryY=robot.cubic_traj(-130,-130,0,2,0,0);
trajectoryZ=robot.cubic_traj(99,14.79,0,2,0,0);
trajectory10=Trajectory([trajectoryX';trajectoryY';trajectoryZ'],2,false);
trajectoryX=robot.cubic_traj(105,105,0,2,0,0);
trajectoryY=robot.cubic_traj(-130,-130,0,2,0,0);
trajectoryZ=robot.cubic_traj(14.79,99,0,2,0,0);
trajectory11=Trajectory([trajectoryX';trajectoryY';trajectoryZ'],2,false);
trajectoryX=robot.cubic_traj(105,105,0,2,0,0);
trajectoryY=robot.cubic_traj(-130,-50,0,2,0,0);
trajectoryZ=robot.cubic_traj(99,99,0,2,0,0);
trajectory12=Trajectory([trajectoryX';trajectoryY';trajectoryZ'],2,false);
trajectoryX=robot.cubic_traj(105,105,0,2,0,0);
trajectoryY=robot.cubic_traj(-50,-50,0,2,0,0);
trajectoryZ=robot.cubic_traj(99,14.79,0,2,0,0);
trajectory13=Trajectory([trajectoryX';trajectoryY';trajectoryZ'],2,false);
trajectoryX=robot.cubic_traj(105,105,0,2,0,0);
trajectoryY=robot.cubic_traj(-50,-50,0,2,0,0);
trajectoryZ=robot.cubic_traj(14.79,99,0,2,0,0);
trajectory14=Trajectory([trajectoryX';trajectoryY';trajectoryZ'],2,false);
trajectoryX=robot.cubic_traj(105,0,0,2,0,0);
trajectoryY=robot.cubic_traj(-50,-50,0,2,0,0);
trajectoryZ=robot.cubic_traj(99,99,0,2,0,0);
trajectory15=Trajectory([trajectoryX';trajectoryY';trajectoryZ'],2,false);
trajectoryX=robot.cubic_traj(0,0,0,2,0,0);
trajectoryY=robot.cubic_traj(-50,-50,0,2,0,0);
trajectoryZ=robot.cubic_traj(99,14.79,0,2,0,0);
trajectory16=Trajectory([trajectoryX';trajectoryY';trajectoryZ'],2,false);

trajectory_queue=LinkedList();

trajectoryX=robot.cubic_traj(currentX,0,0,4,0,0);
trajectoryY=robot.cubic_traj(currentY,0,0,4,0,0);
trajectoryZ=robot.cubic_traj(currentZ,295,0,4,0,0);
trajectoryA=Trajectory([trajectoryX';trajectoryY';trajectoryZ'],4,false);
trajectory_queue.add(trajectoryA);

vel_traj_queue=LinkedList();
vel_traj_1=VelocityTrajectory([100,0,195],50);
vel_traj_2=VelocityTrajectory([69,-135,108],50);
vel_traj_3=VelocityTrajectory([0,-50,14.9987],50);
vel_traj_queue.add(vel_traj_1);
vel_traj_queue.add(vel_traj_2);
vel_traj_queue.add(vel_traj_3);

%trajectory_queue.add(trajectory1);
%trajectory_queue.add(trajectory2);
%trajectory_queue.add(trajectory3);
%trajectory_queue.add(trajectory4);
%trajectory_queue.add(trajectory5);
%trajectory_queue.add(trajectory6);
%trajectory_queue.add(trajectory7);
%trajectory_queue.add(trajectory8);
%trajectory_queue.add(trajectory9);
%trajectory_queue.add(trajectory10);
%trajectory_queue.add(trajectory11);
%trajectory_queue.add(trajectory12);
%trajectory_queue.add(trajectory13);
%trajectory_queue.add(trajectory14);
%trajectory_queue.add(trajectory15);
%trajectory_queue.add(trajectory16);

i=1;
should_end=false;
vel_traj_time=0;
trajectory_start_time=tic;
current_trajectory=trajectory_queue.get(0);
current_velocity_trajectory=vel_traj_queue.get(0);
robot.start_profiler();
plot_timer=timer("TimerFcn",@plotter_callback,"Period", 0.1, "ExecutionMode", "fixedRate");
%start(plot_timer);
%plot_future=parfeval(backgroundPool, @plotter_callback, 0)
try
  while(true)
    switch state
        %run once at the beginning of every new trajectory. Move here to
        %start interpolating.
        case STATE.start_trajectory
            trajectory_start_time=tic;
            elapsed_time=0;
            if trajectory_queue.isEmpty()
                should_end=true;
            else
                current_trajectory=trajectory_queue.remove();
                state=STATE.run_trajectory;
            end
        %main running state- should not be called before 
        case STATE.run_trajectory
            elapsed_time=toc(trajectory_start_time);
            if elapsed_time>current_trajectory.getTime()
                state=STATE.start_trajectory;
            else
                robot.run_trajectory(current_trajectory.getCoefficients(), elapsed_time, current_trajectory.getJointspace());
                %state=STATE.disp_3d_plot;
                state=STATE.check_for_singularity;
            end
        case STATE.start_velocity_trajectory
            if vel_traj_queue.isEmpty()
                should_end=true;
            else
                current_velocity_trajectory=vel_traj_queue.remove();
                trajectory_start_time=tic;
                elapsed_time=0;
                dist= dist3d(robot.position_profile(end,5:7),current_velocity_trajectory.getPosition());
                vel_traj_time=dist/current_velocity_trajectory.getVelocity();
                
                state=STATE.run_velocity_trajectory;
            end
        case STATE.run_velocity_trajectory
            elapsed_time=toc(trajectory_start_time);
            if(elapsed_time>=vel_traj_time)
                state=STATE.start_velocity_trajectory;
            else
                robot.run_velocity_trajectory(current_velocity_trajectory.getPosition(), current_velocity_trajectory.getVelocity());
            end
        case STATE.check_for_singularity
            pos=robot.measured_js(true, false);
            jacob=robot.jacob3001(pos(1,1),pos(1,2),pos(1,3));
            state=STATE.run_trajectory;
        case STATE.interpolate
            %robot.interpolate_jp(pose(i,:),2000);
            robot.servo_jp(pose(i,:))
            state=STATE.disp_3d_plot;
        case STATE.disp_setpoint
            robot.setpoint_js()
        case  STATE.disp_pos_vel
            pos=robot.measured_js(true, false);
            robot.fk_3001(pos(1,:))
            %state=STATE.check_for_completion;
        case STATE.disp_goal_js
            robot.goal_js()
        case STATE.init_stl_plot
            robot.model.init_stl_plot();
            state=STATE.disp_stl_plot;
        case STATE.disp_stl_plot
            robot.model.plot_STL();
        case STATE.disp_3d_plot
            robot.model.plot_3d();
            %state=STATE.run_trajectory;
        case STATE.check_for_completion
            state=STATE.interpolate;
            pos=robot.measured_js(true, false);
            x=norm(pos(1,:)-pose(i,:));
            disp(x)
            if(x<10)
                if(i<height(pose))
                    i=i+1;
                    state=STATE.interpolate;
                else
                    should_end=true;
                end
            end

    end
    if(should_end)
        robot.record_results()
        break
    end
  end
  
catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end

%robot.display_results()
%robot.record_results();
% Clear up memory upon termination
robot.shutdown()
stop(plot_timer);
%cancel(plot_future)
toc

function plotter_callback(timer, event)
    global robot;
    %while true
    robot.model.plot_3d();
    %end
end
function dist=dist3d(p1, p2)
    dist=sqrt((p2(1)-p1(1))^2+(p2(2)-p1(2))^2+(p2(3)-p1(3))^2)
end