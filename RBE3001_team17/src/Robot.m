%packet structure is alernating setpoint/value
classdef Robot < handle
    
    properties
        myHIDSimplePacketComs;
        pol; 
        GRIPPER_ID = 1962
        last_command_time=NaN;
        %not used yet
        currently_profiling=false;
        %position history
        position_profile=[];
        t;
        t0 = clock;
        current_goal=[]
        model;
    end
    
    methods
        function record_results(self)
            self.position_profile(1,:)=[];
            hold off
            figure(2)
            plot3(self.position_profile(:,5),self.position_profile(:,6),self.position_profile(:,7))
            title("End effector position")
            xlabel("X (mm)")
            ylabel("Y (mm)")
            zlabel("Z (mm)")
            figure(3)
            subplot(3,1,1)
            hold on
            title("axis velocities")
            xlabel("time(ms)")
            ylabel("velocity(mm/s)")
            xpos=self.position_profile(:,5);
            ypos=self.position_profile(:,6);
            zpos=self.position_profile(:,7);
            ti=self.position_profile(:,4);
            vx= diff(xpos)./diff(ti);
            vy= diff(ypos)./diff(ti);
            vz= diff(zpos)./diff(ti);
            plot(ti(1:end-1),vx)
            plot(ti(1:end-1),vy)
            plot(ti(1:end-1),vz)
            legend("X velovity","Y velocity","Z velocity")
            hold off
       
            subplot(3,1,2)
            hold on
            title("position vs time")
            plot(self.position_profile(:,4),self.position_profile(:,1))
            plot(self.position_profile(:,4),self.position_profile(:,2))
            plot(self.position_profile(:,4),self.position_profile(:,3))
            legend("X position","Y position","Z position")
            xlabel("time(ms)")
            ylabel("position(degrees)")
            hold off
           
            subplot(3,1,3)
            hold on
            title("axis acceleration")
            xlabel("time(ms)")
            ylabel("acceleration(mm/s^2)")
            ti(end)=[];
            ax= diff(vx)./diff(ti);
            ay= diff(vy)./diff(ti);
            az= diff(vz)./diff(ti);
            plot(ti(1:end-1),ax)
            plot(ti(1:end-1),ay)
            plot(ti(1:end-1),az)
            legend("X acceleration","Y acceleration","Z acceleration")
            hold off
            
        end
        %The is a shutdown function to clear the HID hardware connection
        function  shutdown(self)
            %stop the profiling timer
            self.stop_profiler();
	        %Close the device
            self.myHIDSimplePacketComs.disconnect();
        end
        
        % Create a packet processor for an HID device with USB PID 0x007
        function self = Robot(dev)
             self.myHIDSimplePacketComs=dev; 
            self.pol = java.lang.Boolean(false);
            self.model=Model(self);
            self.t=timer("TimerFcn",@self.profiler_callback,"Period", 0.04, "ExecutionMode", "fixedRate");

        end
        
        %Perform a command cycle. This function will take in a command ID
        %and a list of 32 bit floating point numbers and pass them over the
        %HID interface to the device, it will take the response and parse
        %them back into a list of 32 bit floating point numbers as well
        function com = command(self, idOfCommand, values)
                com= zeros(15, 1, 'single');
                try
                    ds = javaArray('java.lang.Double',length(values));
                    for i=1:length(values)
                        ds(i)= java.lang.Double(values(i));
                    end
                    % Default packet size for HID
                    intid = java.lang.Integer(idOfCommand);
                    self.myHIDSimplePacketComs.writeFloats(intid,  ds);
                    ret = 	self.myHIDSimplePacketComs.readFloats(intid) ;
                    for i=1:length(com)
                       com(i)= ret(i).floatValue();
                    end
                catch exception
                    getReport(exception)
                    disp('Command error, reading too fast');
                end
        end
        %activates position profiling- used for more streamlined velocity
        %readings
        function start_profiler(self)
            self.currently_profiling=true;
            start(self.t);
        end
        %deactivates position profiling
        function stop_profiler(self)
            self.currently_profiling=false;
            stop(self.t);
        end
        function profiler_callback(self, timer, event)
            common
            read_packet=self.read(SERVER_ID_READ);
            position = [read_packet(3), read_packet(5), read_packet(7), round(etime(clock,self.t0) * 1000)];
            joints=[read_packet(3),read_packet(5),read_packet(7)];
            T=self.fk_3001(joints);
            position=[position T(1,end) T(2,end) T(3,end)];
            self.position_profile=[self.position_profile ; position];
        end
        %runs position profiling
        function com = read(self, idOfCommand)
                com= zeros(15, 1, 'single');
                try

                    % Default packet size for HID
                    intid = java.lang.Integer(idOfCommand);
                    ret = 	self.myHIDSimplePacketComs.readFloats(intid) ;
                    for i=1:length(com)
                       com(i)= ret(i).floatValue();
                    end
                catch exception
                  getReport(exception)
                    disp('Command error, reading too fast');
                end
        end
        
        function  write(self, idOfCommand, values)
                try
                    ds = javaArray('java.lang.Double',length(values));
                    for i=1:length(values)
                        ds(i)= java.lang.Double(values(i));
                    end
                    % Default packet size for HID
                    intid = java.lang.Integer(idOfCommand);
                    self.myHIDSimplePacketComs.writeFloats(intid,  ds,self.pol);

                catch exception
                    getReport(exception)
                    disp('Command error, reading too fast');
                end
        end
        
        % Specifies a position to the gripper
        function writeGripper(self, value)
            try
                ds = javaArray('java.lang.Byte',length(1));
                ds(1)= java.lang.Byte(value);
                intid = java.lang.Integer(self.GRIPPER_ID);
                self.myHIDSimplePacketComs.writeBytes(intid, ds, self.pol);
            catch exception
                getReport(exception)
                disp('Command error, reading too fast');
            end
        end
        
        % Opens the gripper
        function openGripper(self)
            self.writeGripper(180);
        end
        
        % Closes the gripper
        function closeGripper(self)
            self.writeGripper(0);
        end

        %Moves the arm to a set position based on joint_values
        function servo_jp(self, joint_values)
            common
            packet = zeros(15, 1, 'single'); %The packets sent to the robot
            packet(1) = 0;%interpolation time
            packet(2) = 0;%linear interpolation
            packet(3) = joint_values(1);%first link
            packet(4) = joint_values(2);% Second link
            packet(5) = joint_values(3);% Third link
            self.current_goal = joint_values;

            % Send packet to the server and get the response      
            %robot.write sends a 15 float packet to the micro controller
            self.write(SERV_ID, packet); 
            %robot.read reads a returned 15 float backet from the micro controller.
            returnPacket = self.read(SERVER_ID_READ);
            if DEBUG
                disp('Sent Packet:')
                disp(packet);
                disp('Received Packet:');
                disp(returnPacket);
            end
        end

        %Gets to the arm to a set position in a set amount of time
        function interpolate_jp(self, joint_values, int_time)
            common
            packet = zeros(15, 1, 'single');
            packet(1) = int_time;%interpolation time
            packet(2) = 0;%linear interpolation
            packet(3) = joint_values(1);%first link
            packet(4) = joint_values(2);% Second link
            packet(5) = joint_values(3);% Third link
            self.current_goal = joint_values;

            % Send packet to the server and get the response      
            %robot.write sends a 15 float packet to the micro controller
            self.write(SERV_ID, packet); 
            %robot.read reads a returned 15 float backet from the micro controller.
            returnPacket = self.read(SERVER_ID_READ);
            if DEBUG
                disp('Sent Packet:')
                disp(packet);
                disp('Received Packet:');
                disp(returnPacket);
            end
        end
        
        %Measures velocity on the bottom and position on the top in a 2x3 array
        function m = measured_js(self, GETPOS, GETVEL)
            common

            %Gets the time from the cpu
            current_command_time=cputime;

            %Checks if the time changed from the last call
            if(isnan(self.last_command_time))
            end

            %Initialized the value measured
            measured = zeros(2,3);

            %Reads the current position of the robot
            packet = self.read(SERVER_ID_READ);

            if GETPOS
               measured(1,:) = [packet(3), packet(5), packet(7)]; 
            end 
            
            if GETVEL && height(self.position_profile)>1
                measured(2,:) = [(packet(3)-self.position_profile(height(self.position_profile)-1,1))/(round(etime(clock,self.t0) * 1000)-self.position_profile(height(self.position_profile),4)) (packet(5)-self.position_profile(height(self.position_profile)-1,2))/(round(etime(clock,self.t0) * 1000)-self.position_profile(height(self.position_profile),4)) (packet(7)-self.position_profile(height(self.position_profile)-1,3))/(round(etime(clock,self.t0) * 1000)-self.position_profile(height(self.position_profile),4))];
            end
            m = measured;

            %Updates the last_command_time
            self.last_command_time=current_command_time;
        end

        %returns a 1x3 array containing current joint set positions in degrees
        function m = setpoint_js(self)
            common
            packet=self.read(SERVER_ID_READ);
            m=[packet(2),packet(4),packet(6)];
        end

        %returns the end of motion joint set point positions
        function m = goal_js(self)
            m = self.current_goal;
        end
        %DH transformation Code
        function T = dh2mat(self,theta, d, a, alpha) 
            %% your code here 
            %Finds transformation matrix of the first section
        
            T = [cosd(theta) -sind(theta)*cosd(alpha) sind(theta)*sind(alpha) a*cosd(theta); sind(theta) cosd(theta)*cosd(alpha) -cosd(theta)*sind(alpha) a*sind(theta); 0 sind(alpha) cosd(alpha) d; 0 0 0 1];
        end 
        
        %Receives DH parameters, outputs transformation matrix
        function matrix = dh2fk(self,input)
            theta = input(1,1);
            d = input(1,2);
            a = input(1,3);
            alpha = input(1,4);
            matrix = self.dh2mat(theta, d, a, alpha);
            if size(input)>1
                for i = 2:size(input,1)
                    theta = input(i,1);
                    d = input(i,2);
                    a = input(i,3);
                    alpha = input(i,4);
                    matrix = matrix*self.dh2mat(theta, d, a, alpha);
                end
            end
        end
        %takes n joint configurations as inputs in the form of an nx1 vector, returns a 4x4
        %homogeneous transformation matrix representing the position and orientation of the
        %tip frame with respect to the base frame
        function T=fk_3001(self,joints)
            d0=55;
            d1=40;
            d2=100;
            d3=100;
            dht=[0,d0,0,0;joints(1),d1,0,-90;joints(2)-90,0,d2,0;joints(3)+90,0,d3,0];
            T=self.dh2fk(dht);
        
        end
        function T=fk_3001_joint0_1(self)
            d0=55;
            dht=[0,d0,0,0];
            T=self.dh2fk(dht);
        
        end
        function T=fk_3001_joint1_2(self,joints)
            d1=40;
            dht=[joints(1),d1,0,-90];
            T=self.dh2fk(dht);
        
        end
        function T=fk_3001_joint2_3(self,joints)
            d2=100;
            dht=[joints(2)-90,0,d2,0];
            T=self.dh2fk(dht);
        
        end
        function T=fk_3001_joint3_4(self,joints)
            d3=100;
            dht=[joints(3)+90,0,d3,0];
            T=self.dh2fk(dht);
        
        end
        function matrix = measured_cp(self)
            measurement = measured_js(true,false);
            theta1 = measurement(1,1);
            theta2 = measurement(1,2);
            theta3 = measurement(1,3);
            matrix = self.fk_3001([theta1; theta2; theta3]);
        end
        
        function matrix = setpoint_cp(self)
            measurement = setpoint_js(true,false);
            theta1 = measurement(1);
            theta2 = measurement(2);
            theta3 = measurement(3);
            matrix = self.fk_3001([theta1; theta2; theta3]);
        end
        
        function matrix = goal_cp(self)
            measurement = goal_js(true,false);
            theta1 = measurement(1);
            theta2 = measurement(2);
            theta3 = measurement(3);
            matrix = self.fk_3001([theta1; theta2; theta3]);
        end

        %%Calculates the Inverse kinematics
        function jointVal = ik_3001(self,coord)
            jointVal = zeros(2,3);

            %Size of the robot's links
            L1 = 95;
            L2 = 100;
            L3 = 100;
            %Initial X, Y and Z coordinates
            x = coord(1);
            y = coord(2);
            z = coord(3);
            
            r=sqrt(x^2+y^2);
            s=z-L1;
            c=sqrt(r^2+s^2);

            alpha0=atand(s/r);

            %calculate theta 1

            D1=x/r;
            if coord(2)>0
                jointVal(1,1)=atan2d(sqrt(1-D1^2),D1);
                jointVal(2,1)=atan2d(-sqrt(1-D1^2),D1);
            else
                jointVal(1,1)=atan2d(-sqrt(1-D1^2),D1);
                jointVal(2,1)=atan2d(sqrt(1-D1^2),D1);
            end

            %calculate theta 2
            D4=(L2^2+c^2-L3^2)/(2*L2*c);
            alpha4_1=atan2d(sqrt(1-D4^2),D4);
            alpha4_2=atan2d(-sqrt(1-D4^2),D4);
            jointVal(1,2)=90-alpha0-alpha4_1;
            jointVal(2,2)=90-alpha0-alpha4_2;

            %calculate theta 3
            D3=(L2^2+L3^2-c^2)/(2*L2*L3);
            alpha3_1=atan2d(sqrt(1-D3^2),D3);
            alpha3_2=atan2d(-sqrt(1-D3^2),D3);
            jointVal(1,3)=90-alpha3_1;
            jointVal(2,3)=90-alpha3_2;
           
        end
        function a_vals=cubic_traj(self, q0,qf,t0,tf,v0,vf)
            a_vals = zeros(4,1);
        
            %Calculates m and m^-1
            m = [1 t0 t0^2 t0^3; 0 1 2&t0 3*t0^2; 1 tf tf^2 tf^3; 0 1 2*tf 3*tf^2];
            mInverse = inv(m);
        
            q = [q0; v0; qf; vf];
            
            a_vals= mInverse*q;
            
        end
        
        function a_vals = quintic_traj(self, q0, qf,t0,tf,v0,vf,a0,af)
            a_vals = zeros(6,1);
        
            m = [1 t0 t0^2 t0^3 t0^4 t0^5; 
                0 1 2*t0 3*t0^2 4*t0^3 5*t0^4;
                0 0 2 6*t0 12*t0^2 20*t0^3;
                1 tf tf^2 tf^3 tf^4 tf^5; 
                0 1 2*tf 3*tf^2 4*tf^3 5*tf^4; 
                0 0 2 6*tf 12*tf^2 20*tf^3];
        
            mInverse = inv(m);
        
            q = [q0; v0; a0; qf; vf; af];
        
            a_vals=mInverse*q;
        end
        %INCOMPLETE
        %Creates and runs through a trajectory for a movement
        function run_trajectory(self, traj, time, jointspace)
            servo1=0;
            servo2=0;
            servo3=0;
            if jointspace && width(traj)<5
                %Calculates the position it should be at the time
                %specified
                servo1 = traj(1,1)+traj(1,2)*time+traj(1,3)*time^2+traj(1,4)*time^3;
                servo2 = traj(2,1)+traj(2,2)*time+traj(2,3)*time^2+traj(2,4)*time^3;
                servo3 = traj(3,1)+traj(3,2)*time+traj(3,3)*time^2+traj(3,4)*time^3;
            %It is a cubic workspace
            elseif jointspace
                servo1 = traj(1,1)+traj(1,2)*time+traj(1,3)*time^2+traj(1,4)*time^3+traj(1,5)*time^4+traj(1,6)*time^5;
                servo2 = traj(2,1)+traj(2,2)*time+traj(2,3)*time^2+traj(2,4)*time^3+traj(2,5)*time^4+traj(2,6)*time^5;
                servo3 = traj(3,1)+traj(3,2)*time+traj(3,3)*time^2+traj(3,4)*time^3+traj(3,5)*time^4+traj(3,6)*time^5;
            elseif ~jointspace && width(traj) < 5
                %Calculates the position it should be at the time
                %specified
                xpos = traj(1,1)+traj(1,2)*time+traj(1,3)*time^2+traj(1,4)*time^3;
                ypos = traj(2,1)+traj(2,2)*time+traj(2,3)*time^2+traj(2,4)*time^3;
                zpos = traj(3,1)+traj(3,2)*time+traj(3,3)*time^2+traj(3,4)*time^3;
                pos=[xpos,ypos,zpos];
                setpos=self.ik_3001(pos);
                servo1=setpos(1,1);
                servo2=setpos(1,2);
                servo3=setpos(1,3);
            %it is quintic workspace
            else
                %Calculates the position it should be at the time
                %specified
                xpos=traj(1,1)+traj(1,2)*time+traj(1,3)*time^2+traj(1,4)*time^3+traj(1,5)*time^4+traj(1,6)*time^5;
                ypos=traj(2,1)+traj(2,2)*time+traj(2,3)*time^2+traj(2,4)*time^3+traj(2,5)*time^4+traj(2,6)*time^5;
                zpos=traj(3,1)+traj(3,2)*time+traj(3,3)*time^2+traj(3,4)*time^3+traj(3,5)*time^4+traj(3,6)*time^5;
                pos=[xpos,ypos,zpos];
                setpos=self.ik_3001(pos);
                servo1=setpos(1,1);
                servo2=setpos(1,2);
                servo3=setpos(1,3);
            end
            self.servo_jp([servo1,servo2,servo3]);
        end
        %run a velocity trajectory
        function run_velocity_trajectory(self, target, velocity)
            target=target'
            current_pos=self.position_profile(end,5:7);
            direction = target - current_pos;
            direction_scaled = direction/norm(direction)*velocity; % vector proportional to velocity
            current_joints=self.position_profile(end,1:3);
            jacob=self.jacob3001(current_joints(1),current_joints(2), current_joints(3));
            jacob=inv(jacob(1:3,:));
            desired_joint_vels=jacob*direction_scaled';
            self.servo_jp(current_joints+12*desired_joint_vels');
        end
        %Calculates the Jacobian Matrix
        %
        function matrix = jacob3001(self,s1,s2,s3)
            %Gets all Linear x values
            x1 = 100*sind(s2 - 90)*sind(s3 + 90)*sind(s1) - 100*cosd(s2 - 90)*cosd(s3 + 90)*sind(s1) - 100*cosd(s2 - 90)*sind(s1);
            x2 = - 100*sind(s2 - 90)*cosd(s1) - 100*cosd(s2 - 90)*sind(s3 + 90)*cosd(s1) - 100*cosd(s3 + 90)*sind(s2 - 90)*cosd(s1);
            x3 = - 100*cosd(s2 - 90)*sind(s3 + 90)*cosd(s1) - 100*cosd(s3 + 90)*sind(s2 - 90)*cosd(s1);

            %Gets all Linear y values
            y1 = 100*cosd(s2 - 90)*cosd(s1) + 100*cosd(s2 - 90)*cosd(s3 + 90)*cosd(s1) - 100*sind(s2 - 90)*sind(s3 + 90)*cosd(s1);
            y2 = - 100*sind(s2 - 90)*sind(s1) - 100*cosd(s2 - 90)*sind(s3 + 90)*sind(s1) - 100*cosd(s3 + 90)*sind(s2 - 90)*sind(s1);
            y3 = - 100*cosd(s2 - 90)*sind(s3 + 90)*sind(s1) - 100*cosd(s3 + 90)*sind(s2 - 90)*sind(s1);

            %Gets all Linear z values
            z1 = 0;
            z2 = 100*sind(s2 - 90)*sind(s3 + 90) - 100*cosd(s2 - 90)*cosd(s3 + 90) - 100*cosd(s2 - 90);
            z3 = 100*sind(s2 - 90)*sind(s3 + 90) - 100*cosd(s2 - 90)*cosd(s3 + 90);

            %Gets all Rotational s1 values
            oX1 =  0;
            oX2 = 0;
            oX3 = 1;
            
            %Gets all Rotational s2 values
            oY1 = -sind(s1);
            oY2 = cosd(s1);
            oY3 = 0;

            %Gets all Rotational s3 values
            oZ1 = -sind(s1);
            oZ2 = cosd(s1);
            oZ3 = 0;

            matrixCheck = [x1 x2 x3;y1 y2 y3; z1 z2 z3; oX1 oY1 oZ1; oX2 oY2 oZ2; oX3 oY3 oZ3];
            matrixP = [x1 x2 x3;y1 y2 y3; z1 z2 z3];
            %This section of the code determines if the arm is approaching
            %a singularity
            checkSingularity = abs(det(matrixP));

            if checkSingularity > 1000
                matrix = matrixCheck;
            else
                singularity = MException('jacob3001:Error', 'Approaching Singularity');
                throw(singularity);            end
        end

        %Calculates the Velocity Kinematics
        %q is the current joint position
        %qPrime is vector of instantaneous joint velocities
        function velocityVector = fdk3001(self, q,qPrime)
            jacobian = self.jacob3001(q(1),q(2),q(3));
            velocityVector = jacobian*qPrime';
        end
        function velocityVector = get_tip_velocity(self)
            pos=self.measured_js(true, true);
            velocityVector=self.fdk3001(pos(1,:), pos(2,:));
        end

    end
end