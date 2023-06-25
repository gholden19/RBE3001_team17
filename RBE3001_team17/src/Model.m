classdef Model < handle
    properties
        robot;
        plot_intermediate_frames=false
        h=animatedline;
        b1=animatedline;
        b2=animatedline;
        b3=animatedline;
        a1=animatedline;
        a2=animatedline;
        a3=animatedline;
        a4=animatedline;
        a5=animatedline;
        a6=animatedline;
        a7=animatedline;
        a8=animatedline;
        a9=animatedline;
        a10=animatedline;
        a11=animatedline;
        a12=animatedline;
        v1=animatedline;
        data1=stlread('../stl/10-BaseMount.stl');
        data2=stlread('../stl/2-BaseCone.stl')
        data3=stlread('../stl/7-MiddleLinkMainBracket.stl');
        data4=stlread('../stl/6-LastLinkMainBracket.stl')
        data1b;
        data2b;
        data3b;
        data4b;
        base;
        cone;
        middle;
        top;
        stl_fig;
        current_j1_pos=0;
        current_j2_pos=0;
        current_j3_pos=0;
    end
    methods
        function self = Model(robot)
            self.robot=robot;
            %these started breaking matlab with "invalid or deleted object- something sometimes doesn't get freed"
            self.h.Marker='o';
            self.b1.Color="r";
            self.b2.Color="g";
            self.b3.Color="b";
            self.a1.Color="r";
            self.a2.Color="g";
            self.a3.Color="b";
            self.a4.Color="r";
            self.a5.Color="g";
            self.a6.Color="b";
            self.a7.Color="r";
            self.a8.Color="g";
            self.a9.Color="b";
            self.a10.Color="r";
            self.a11.Color="g";
            self.a12.Color="b";
            self.v1.Color="m";
        end
        function init_stl_plot(self)
             %calculate joint positions
            self.stl_fig=gcf;
            j1=self.robot.position_profile(end,1);
            j2=self.robot.position_profile(end,2);
            j3=self.robot.position_profile(end,3);
            T_0_1=self.robot.fk_3001_joint0_1();
            T_1_2=self.robot.fk_3001_joint1_2([j1,j2,j3]);
            T_2_3=self.robot.fk_3001_joint2_3([j1,j2,j3]);
            T_3_4=self.robot.fk_3001_joint3_4([j1,j2,j3]);
            T=T_0_1;
            pos_1=[T(1,end),T(2,end),T(3,end)];
            T=T*T_1_2;
            pos_2=[T(1,end),T(2,end),T(3,end)];
            T=T*T_2_3;
            pos_3=[T(1,end),T(2,end),T(3,end)];


            hold on
            %zero configuration
            points=[self.data2.Points(:,1:2),self.data2.Points(:,3)+55];
            self.data2b=triangulation(self.data2.ConnectivityList, points);
            self.cone=trisurf(self.data2b, 'FaceColor', 'red', 'EdgeColor', 'k');
            rotate(self.cone, [1 0 0], 180);
            
            
            %zero configuration
            points=[self.data3.Points(:,1)+60,self.data3.Points(:,2),self.data3.Points(:,3)+135];
            self.data3b=triangulation(self.data3.ConnectivityList, points);
            self.middle=trisurf(self.data3b,'FaceColor', 'green', 'EdgeColor', 'k');
            center=[0,0,135];
            rotate(self.middle, [1,0,0], -90, center)
            rotate(self.middle, [0,1,0],-90,center)
            
            %zero configuration
            points=[self.data4.Points(:,1)+100,self.data4.Points(:,2),self.data4.Points(:,3)+195];
            self.data4b=triangulation(self.data4.ConnectivityList, points);
            self.top=trisurf(self.data4b,'FaceColor', 'green', 'EdgeColor', 'k');
            center=[0,0,195];
            rotate(self.top, [1,0,0], -90, center)

            %this has to be last for some reason???
            self.base=trisurf(self.data1,'FaceColor', 'green', 'EdgeColor', 'k');
            xlabel("X") 
            ylabel("Y")
            zlabel("Z")
            
            aH = ancestor(self.base,'axes');
            set(aH,'zLim',[-20 300])
        end
        function plot_STL(self)
            figure(self.stl_fig);
            delete(self.middle)
            j1=self.robot.position_profile(end,1);
            j2=self.robot.position_profile(end,2);
            j3=self.robot.position_profile(end,3);
            T_0_1=self.robot.fk_3001_joint0_1();
            T_1_2=self.robot.fk_3001_joint1_2([j1,j2,j3]);
            T_2_3=self.robot.fk_3001_joint2_3([j1,j2,j3]);
            T_3_4=self.robot.fk_3001_joint3_4([j1,j2,j3]);
            T=T_0_1;
            pos_1=[T(1,end),T(2,end),T(3,end)];
            T0_2=T*T_1_2;
            pos_2=[T0_2(1,end),T0_2(2,end),T0_2(3,end)];
            T0_3=T*T_2_3;
            pos_3=[T0_3(1,end),T0_3(2,end),T0_3(3,end)];
            update_j1=false;
            update_j2=false
            if(self.current_j1_pos~=j1)
                update_j1=true;
            end
            if(self.current_j2_pos~=j2)
                update_j2=true;
            end

            %zero configuration of cone
            points=[self.data2.Points(:,1)+pos_1(1),self.data2.Points(:,2)+pos_1(2),self.data2.Points(:,3)+pos_1(3)];
            self.data2b=triangulation(self.data2.ConnectivityList, points);
            if(update_j1)
                rotate(self.cone,[0,0,1],j1-self.current_j1_pos,pos_1);
            end

            %zero configuration of middle
            points=double([self.data3.Points(:,1)+60+pos_2(1),self.data3.Points(:,2)+pos_2(2),self.data3.Points(:,3)+pos_2(3)]);
            for i=1:height(points)
                points_transform=[points(i,:), 1]*T0_3;
                points_transform=points_transform(1:3);
                Rx = [1 0 0; 0 cos(-90) -sind(-90); 0 sind(-90) cosd(-90)];
                points_transform=points_transform*Rx;
                Ry = [cosd(-90) 0 sind(-90); 0 1 0; -sind(-90) 0 cosd(-90)];
                points_transform=points_transform*Ry;
                points(i,:)=points_transform;
            end
            self.data3b=triangulation(self.data3.ConnectivityList, points);
            self.middle=trisurf(self.data3b,'FaceColor', 'green', 'EdgeColor', 'k');
            %if(update_j1)
            %    rotate(self.middle,[0,0,1],j1-self.current_j1_pos,pos_2);
            %end
            %if(update_j2)
            %    rot_axis=[1,0,0,1]*T0_2;
            %    rotate(self.middle,rot_axis(1:3),j2-self.current_j2_pos,pos_1);
            %end

            %zero configuration of top
            %points=[self.data4.Points(:,1)+100,self.data4.Points(:,2),self.data4.Points(:,3)+195];
            %self.data4b=triangulation(self.data4.ConnectivityList, points);
            %if(update_j1)
            %    rotate(self.top,[0,0,1],j1-self.current_j1_pos,pos_2);
            %end

            if(update_j1)
                self.current_j1_pos=j1;
            end
            if(update_j2)
                self.current_j2_pos=j2;
            end

            refreshdata(self.stl_fig);
            drawnow
           
        end
        function plot_3d(self)
           

            figure(1)
            if(self.plot_intermediate_frames)
                clearpoints(self.a1)
                clearpoints(self.a2)
                clearpoints(self.a3)
                clearpoints(self.a4)
                clearpoints(self.a5)
                clearpoints(self.a6)
                clearpoints(self.a7)
                clearpoints(self.a8)
                clearpoints(self.a9)
            end
            clearpoints(self.a10)
            clearpoints(self.a11)
            clearpoints(self.a12)
            clearpoints(self.v1)
            j1=self.robot.position_profile(end,1);
            j2=self.robot.position_profile(end,2);
            j3=self.robot.position_profile(end,3);
            base_x_vector=[30,0,0,1];
            base_y_vector=[0,30,0,1];
            base_z_vector=[0,0,30,1];
          
            T_0_1=self.robot.fk_3001_joint0_1();
            T_1_2=self.robot.fk_3001_joint1_2([j1,j2,j3]);
            T_2_3=self.robot.fk_3001_joint2_3([j1,j2,j3]);
            T_3_4=self.robot.fk_3001_joint3_4([j1,j2,j3]);
            T=T_0_1;
            pos_1=[T(1,end),T(2,end),T(3,end)];
            T=T*T_1_2;
            pos_2=[T(1,end),T(2,end),T(3,end)];
            T=T*T_2_3;
            pos_3=[T(1,end),T(2,end),T(3,end)];
            T=T*T_3_4;
            pos_4=[T(1,end),T(2,end),T(3,end)];
            if(self.plot_intermediate_frames)
                pos1_x=T*(base_x_vector)';
                pos1_y=T*(base_y_vector)';
                pos1_z=T*(base_z_vector)';
                addpoints(self.a1,pos_1(1),pos_1(2),pos_1(3))
                addpoints(self.a1,pos1_x(1),pos1_x(2),pos1_x(3))
                addpoints(self.a2,pos_1(1),pos_1(2),pos_1(3))
                addpoints(self.a2,pos1_y(1),pos1_y(2),pos1_y(3))
                addpoints(self.a3,pos_1(1),pos_1(2),pos_1(3))
                addpoints(self.a3,pos1_z(1),pos1_z(2),pos1_z(3))
                
                pos2_x=T*(base_x_vector)';
                pos2_y=T*(base_y_vector)';
                pos2_z=T*(base_z_vector)';
                addpoints(self.a4,double(pos_2(1)),double(pos_2(2)),double(pos_2(3)))
                addpoints(self.a4,double(pos2_x(1)),double(pos2_x(2)),double(pos2_x(3)))
                addpoints(self.a5,double(pos_2(1)),double(pos_2(2)),double(pos_2(3)))
                addpoints(self.a5,double(pos2_y(1)),double(pos2_y(2)),double(pos2_y(3)))
                addpoints(self.a6,double(pos_2(1)),double(pos_2(2)),double(pos_2(3)))
                addpoints(self.a6,double(pos2_z(1)),double(pos2_z(2)),double(pos2_z(3)))
    
                pos3_x=T*(base_x_vector)';
                pos3_y=T*(base_y_vector)';
                pos3_z=T*(base_z_vector)';
                addpoints(self.a7,double(pos_3(1)),double(pos_3(2)),double(pos_3(3)))
                addpoints(self.a7,double(pos3_x(1)),double(pos3_x(2)),double(pos3_x(3)))
                addpoints(self.a8,double(pos_3(1)),double(pos_3(2)),double(pos_3(3)))
                addpoints(self.a8,double(pos3_y(1)),double(pos3_y(2)),double(pos3_y(3)))
                addpoints(self.a9,double(pos_3(1)),double(pos_3(2)),double(pos_3(3)))
                addpoints(self.a9,double(pos3_z(1)),double(pos3_z(2)),double(pos3_z(3)))
    
            end
            pos4_x=T*(base_x_vector)';
            pos4_y=T*(base_y_vector)';
            pos4_z=T*(base_z_vector)';
            addpoints(self.a10,double(pos_4(1)),double(pos_4(2)),double(pos_4(3)))
            addpoints(self.a10,double(pos4_x(1)),double(pos4_x(2)),double(pos4_x(3)))
            addpoints(self.a11,double(pos_4(1)),double(pos_4(2)),double(pos_4(3)))
            addpoints(self.a11,double(pos4_y(1)),double(pos4_y(2)),double(pos4_y(3)))
            addpoints(self.a12,double(pos_4(1)),double(pos_4(2)),double(pos_4(3)))
            addpoints(self.a12,double(pos4_z(1)),double(pos4_z(2)),double(pos4_z(3)))
     
            clearpoints(self.h)
            hold on;
            title("3d Robot Plot")
            xlabel("X (mm)")
            ylabel("Y (mm)")
            zlabel("Z (mm)")
            %legend("x axis")
            addpoints(self.h,0,0,0)
          
            addpoints(self.b1,0,0,0)
            addpoints(self.b1,30,0,0)
            addpoints(self.b2,0,0,0)
            addpoints(self.b2,0,30,0)
            addpoints(self.b3,0,0,0)
            addpoints(self.b3,0,0,30)

            addpoints(self.h, double(pos_1(1)), double(pos_1(2)), double(pos_1(3)));

            addpoints(self.h, double(pos_2(1)), double(pos_2(2)), double(pos_2(3)));

            addpoints(self.h, double(pos_3(1)), double(pos_3(2)), double(pos_3(3)));

            addpoints(self.h, double(pos_4(1)), double(pos_4(2)), double(pos_4(3)));

            %velocity
            addpoints(self.v1, double(pos_4(1)), double(pos_4(2)), double(pos_4(3)));
            velocity_vector = self.robot.get_tip_velocity();
            addpoints(self.v1, double(pos_4(1)+velocity_vector(1)*5), double(pos_4(2)+velocity_vector(2)*5), double(pos_4(3)+velocity_vector(3)*5));

            axis([-300 300 -300 300 -300 300])
            drawnow
        end
    end
end