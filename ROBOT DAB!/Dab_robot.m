clear all
close all
clf
handle_axes= axes('XLim', [-0.4,0.4], 'YLim', [-0.2,0.4], 'ZLim', [0,0.4]);

xlabel('e_1'); 
ylabel('e_2');
zlabel('e_3');

view(-130, 26);
grid on;
axis equal
camlight
axis_length= 0.05;


dim = [.2 .2 .3 .3];
str = 'ROBOT DAB!';
annotation('textbox',dim,'String',str,'FitBoxToText','on');

%% Root frame E
trf_E_axes= hgtransform('Parent', handle_axes); 
% The root-link transform should be created as a child of the axes from the
% beginning to avoid the error "Cannot set property to a deleted object".
% E is synonymous with the axes, so there is no need for plot_axes(trf_E_axes, 'E');

%% Link-0: Base-link

trf_link0_E= make_transform([0, 0, 0], 0, 0, pi/2, trf_E_axes);
plot_axes(trf_link0_E, 'L_0', false, axis_length); 

trf_viz_link0= make_transform([0, 0, 0.0425], 0, 0, 0, trf_link0_E);
h(1)= link_box([0.32, 0.26, 0.085], trf_viz_link0, [0 0.9 0.9]); 
plot_axes(trf_viz_link0, ' ', true, axis_length); 
%% Link-laser: base laser
trf_viz_linklas=make_transform([0,0,0],0,0,0);

%% Link-cpu
trf_viz_linkcpu= make_transform([-0.028,0,0.035],0,0,0);
h(1)= link_box([0.32, 0.26, 0.085], trf_viz_linkcpu, [0 0.9 0.9]); 

%% Link-1 : upper base link
trf_viz_linkup= make_transform([0, 0, 0.05], 0, 0, 0); % Do not specify parent yet: It will be done in the joint
h(4)=link_cylinder(0.085, 0.10, trf_viz_linkup, [0 0.5 0.5 ]); 


%% Link torso
trf_viz_linktorso=make_transform([0,0,0.12],0,0,0);
h(5)=link_cylinder(0.05, 0.24, trf_viz_linktorso, [0.8 0.8 0 ]);

%% Link headpan link
trf_viz_linkhdpan= make_transform([0,0,0.0225], 0,0,0);
h(7)=link_box([0.05,0.045,0.045], trf_viz_linkhdpan,  [0 1 0 ]);

%% Link headtilt
trf_viz_linkhdtilt= make_transform([0,0,0.02], 0,0,0);
h(7)=link_box([0.03,0.038,0.04], trf_viz_linkhdtilt,  [0.1 0.8 0]);

%% Link neck
trf_viz_linkneck= make_transform([0,0,0.021], 0,0,0);
h(8)=link_box([0.03, 0.05, 0.042],trf_viz_linkneck, [0.9 0.9 0.9 ]);

%% Link head
trf_viz_linkhead= make_transform([0,0,0.025],0,0,0);
h(9)=link_box([0.03, 0.07, 0.11], trf_viz_linkhead, [0.9 0.9 0.9 ]);

%% link antenna
trf_viz_linkant=make_transform([0,0,0.035],0,0,0);

%% link left shoulder
trf_viz_lshoulder=make_transform([0,0,0],0,0,0);
h(11)=link_box([0.025, 0.015, 0.05], trf_viz_lshoulder,[0 1 0 ]);

%% link right shoulder
trf_viz_rshoulder=make_transform([0,0,0],0,0,0);
h(12)=link_box([0.025, 0.015, 0.05], trf_viz_rshoulder,[0 1 0 ]);

%% link left shoulder forward link
trf_viz_lfshoulder=make_transform([0,0,0],0,0,0);
h(13)=link_box([0.03, 0.05, 0.03], trf_viz_lfshoulder,[0 0 0.9]);

%% link right shoulder forward link
trf_viz_rfshoulder=make_transform([0,0,0],0,0,0);
h(14)=link_box([0.03, 0.05, 0.03], trf_viz_rfshoulder,[0 0 0.9]);

%% link left shoulder up link
trf_viz_linkleftupsh=make_transform([0,0,0],1.57,0,0);
h(15)=link_box([0.03, 0.05, 0.03], trf_viz_linkleftupsh,[0 0 0.7 ]);

%% link right shoulder up link
trf_viz_linkrightupsh=make_transform([0,0,0],1.57,0,0);
h(16)=link_box([0.03, 0.05, 0.03], trf_viz_linkrightupsh,[0 0 0.7 ]);

%% link left upper arm
trf_viz_linkluparm=make_transform([0,0,0],0,0,0);
h(17)=link_cylinder(0.0075,0.05, trf_viz_linkluparm, [0.9 0.9 0.9 ]);

%% link right upper arm
trf_viz_linkruparm=make_transform([0,0,0],0,0,0);
h(18)=link_cylinder(0.0075, 0.05, trf_viz_linkruparm, [0.9 0.9 0.9]);

%% link left elbow
trf_viz_lelbow=make_transform([0,0,0],0,0,1.57);
h(19)=link_box([0.035, 0.035, 0.05], trf_viz_lelbow,[0 0 0.7 ]);

%% link right elbow
trf_viz_relbow=make_transform([0,0,0],0,0,1.57);
h(20)=link_box([0.035, 0.035, 0.05], trf_viz_relbow,[0 0 0.7 ]);

%% link left lower arm
trf_viz_linkllowarm=make_transform([0,0,0],0,0,0);
h(21)=link_cylinder(0.0075, 0.11, trf_viz_linkllowarm, [0.9 0.9 0.9 ]);

%% link right lower arm
trf_viz_linkrlowarm=make_transform([0,0,0],0,0,0);
h(22)=link_cylinder(0.0075,0.11, trf_viz_linkrlowarm, [0.9 0.9 0.9 ]);

%% link left wrist
trf_viz_lwrist=make_transform([0,0,0],1.57,0,0);
h(23)=link_box([0.03, 0.05, 0.03], trf_viz_lwrist,[0 0 0.7 ]);

%% link right wrist
trf_viz_rwrist=make_transform([0,0,0],1.57,0,0);
h(24)=link_box([0.03, 0.05, 0.03], trf_viz_rwrist,[0 0 0.7 ]);

%% link left hand
trf_viz_lhand=make_transform([0,0,0],0,0,0);
h(25)=link_box([0.03, 0.01, 0.06], trf_viz_lhand,[0.7 0.7 0.7 ]);

%% link right hand
trf_viz_rhand=make_transform([0,0,0],0,0,0);
h(26)=link_box([0.03, 0.01, 0.06], trf_viz_rhand,[0.9 0.9 0.9]);

%% Now define all the joints

%% Joint 1 : - Base link, CPU : Fixed
trf_linkCPU_linkBase = make_transform ([0.025,0,0.085],0,0,0, trf_link0_E);
make_child(trf_linkCPU_linkBase, trf_viz_linkcpu);

%% Joint 2: Base laser, Base link :Fixed
trf_linklaser_linkBase = make_transform ([0.18,0,0.07],0,0,0, trf_link0_E);
make_child(trf_linklaser_linkBase, trf_viz_linklas);

%% Joint 3: CPU link, Upper base link : Fixed
trf_linkup_cpu= make_transform([0,0,0.07],0,0,0, trf_linkCPU_linkBase);
make_child(trf_linkup_cpu,trf_viz_linkup);

%% Joint 4: Upper base link, Torso : Revolute
j1_rot_axis_j1= [0,0,1]';
j1_rot_angle= 0; % [-pi/2, pi/2]

trf_joint4_linkup= make_transform([0, 0, 0.1], 0, 0, 0, trf_linkup_cpu); 
trf_linktorso_joint4= make_transform_revolute(j1_rot_axis_j1, j1_rot_angle, trf_joint4_linkup); 
plot_axes(trf_linktorso_joint4, 'L_1', false, axis_length); 
make_child(trf_linktorso_joint4, trf_viz_linktorso);

%% Joint 5 : Torso, Head Pan Servo : fixed
trf_linktorso_hps= make_transform([0,0,0.225],0,0,0, trf_linktorso_joint4);
make_child(trf_linktorso_hps,trf_viz_linkhdpan);

%% Joint 6: head pan link, head tilt link : Revolute
j2_rot_axis_j1= [0,0,1]';
j1_rot_angle= 0; % [-pi/2, pi/2]

trf_joint6_linkhdpan= make_transform([0, 0, 0.045], 0, 0, 0, trf_linktorso_hps); 
trf_linkhdtilt_joint4= make_transform_revolute(j2_rot_axis_j1, j1_rot_angle, trf_joint6_linkhdpan); 
plot_axes(trf_linkhdtilt_joint4, 'L_3', false, axis_length); 
make_child(trf_linkhdtilt_joint4, trf_viz_linkhdtilt);

%% Joint 7 : head tilt link, neck link : Revolute
j3_rot_axis_j1= [0,1,0]';
j1_rot_angle= 0; % [-pi/2, pi/2]

trf_joint7_linkhdtilt= make_transform([0, 0, 0.04], 0, 0, 0, trf_linkhdtilt_joint4); 
trf_linkneck_joint6= make_transform_revolute(j3_rot_axis_j1, j1_rot_angle, trf_joint7_linkhdtilt); 
plot_axes(trf_linkneck_joint6, 'L_4', false, axis_length); 
make_child(trf_linkneck_joint6, trf_viz_linkneck);

%% Joint 8 : neck link, head link: fixed
trf_linkneck_head= make_transform([0.05,0,0.015],0,0,0, trf_linkneck_joint6);
make_child(trf_linkneck_head,trf_viz_linkhead);

%% Joint 9 : head link, antenna link: fixed
trf_linkhead_ant= make_transform([0.0,-0.025,0.065],0,0,0, trf_linkneck_head);
make_child(trf_linkhead_ant,trf_viz_linkant);

%% Joint 10 : torso link, left shoulder link: fixed
trf_linktorso_lsh= make_transform([0,0.055,0.165],0,0,0, trf_linktorso_joint4);
make_child(trf_linktorso_lsh,trf_viz_lshoulder);

%% Joint 11 : torso link, right shoulder link: fixed
trf_linktorso_rsh= make_transform([0,-0.055,0.165],0,0,0, trf_linktorso_joint4);
make_child(trf_linktorso_rsh,trf_viz_rshoulder);

%% Joint 12 : left shoulder link, left shoulder forward : Revolute
j4_rot_axis_j1= [0,0,1]';
j1_rot_angle= 0; % [-pi/2, pi/2]

trf_joint12_linklshoulder= make_transform([0, 0.025, 0], 0, 0, 0, trf_linktorso_lsh); 
trf_linklshf_joint12= make_transform_revolute(j4_rot_axis_j1, j1_rot_angle, trf_joint12_linklshoulder); 
plot_axes(trf_linklshf_joint12, 'L_5', false, axis_length); 
make_child(trf_linklshf_joint12, trf_viz_lfshoulder);

%% Joint 13 : right shoulder link, right shoulder forward : Revolute
j5_rot_axis_j1= [0,0,1]';
j1_rot_angle= 0; % [-pi/2, pi/2]

trf_joint13_linkrshoulder= make_transform([0, -0.025, 0], 0, 0, 0, trf_linktorso_rsh); 
trf_linkrshf_joint13= make_transform_revolute(j5_rot_axis_j1, j1_rot_angle, trf_joint13_linkrshoulder); 
plot_axes(trf_linkrshf_joint13, 'L_6', false, axis_length); 
make_child(trf_linkrshf_joint13, trf_viz_rfshoulder);

%% Joint 14 : left shoulder forward link, left shoulder up : Revolute
j6_rot_axis_j1= [0,1,1]';
j1_rot_angle= 180; % [-pi/2, pi/2]

trf_joint14_linklfshoulder= make_transform([0, 0.04, -0.01], 0, -0.707, 0, trf_linklshf_joint12); 
trf_linklshu_joint14= make_transform_revolute(j6_rot_axis_j1, j1_rot_angle, trf_joint14_linklfshoulder); 
plot_axes(trf_linklshu_joint14, '', false, axis_length); 
make_child(trf_linklshu_joint14, trf_viz_linkleftupsh);

%% Joint 15 : right shoulder forward link, right shoulder up : Revolute
j7_rot_axis_j1= [0,1,1]';
j1_rot_angle= 60; % [-pi/2, pi/2]

trf_joint14_linkrfshoulder= make_transform([0, -0.04, -0.01], 0, -0.707, 0, trf_linkrshf_joint13); 
trf_linkrshu_joint15= make_transform_revolute(j7_rot_axis_j1, j1_rot_angle, trf_joint14_linkrfshoulder); 
plot_axes(trf_linkrshu_joint15, '', false, axis_length); 
make_child(trf_linkrshu_joint15, trf_viz_linkrightupsh);

%% Joint 16 : left shoulder up link, left upper arm link: fixed
trf_linklshup_lual= make_transform([0,0,-0.05],0,0,0, trf_linklshu_joint14);
make_child(trf_linklshup_lual,trf_viz_linkluparm);

%% Joint 17 : right shoulder up link, right upper arm link: fixed
trf_linkrshup_rual= make_transform([0,0,-0.05],0,0,0, trf_linkrshu_joint15);
make_child(trf_linkrshup_rual,trf_viz_linkruparm);

%% Joint 18 : left upper arm lnk, left elbow : Revolute
j8_rot_axis_j1= [0,1,1]';
j1_rot_angle= 0; % [-pi/2, pi/2]

trf_joint18_linkual= make_transform([-0.005, 0, -0.05], 0, 0, 0, trf_linklshup_lual); 
trf_linkle_joint18= make_transform_revolute(j8_rot_axis_j1, j1_rot_angle, trf_joint18_linkual); 
plot_axes(trf_linkle_joint18, 'L_7', false, axis_length); 
make_child(trf_linkle_joint18, trf_viz_lelbow);

%% Joint 19 : right upper arm lnk, right elbow : Revolute
j9_rot_axis_j1= [0,1,1]';
j1_rot_angle= 0; % [-pi/2, pi/2]

trf_joint19_linkuar= make_transform([-0.005, 0, -0.05], 0, 0, 0, trf_linkrshup_rual); 
trf_linkre_joint19= make_transform_revolute(j9_rot_axis_j1, j1_rot_angle, trf_joint19_linkuar); 
plot_axes(trf_linkre_joint19, 'L_8', false, axis_length); 
make_child(trf_linkre_joint19, trf_viz_relbow);

%% Joint 20 : left elbow link, left lower arm link: fixed
trf_linkle_llal= make_transform([0,0,-0.08],0,0,0, trf_linkle_joint18);
make_child(trf_linkle_llal,trf_viz_linkllowarm);

%% Joint 21 : right elbow link, right lower arm link: fixed
trf_linkre_rlal= make_transform([0,0,-0.08],0,0,0, trf_linkre_joint19);
make_child(trf_linkre_rlal,trf_viz_linkrlowarm);

%% Joint 22 : left lower arm, left wrist : Revolute
j10_rot_axis_j1= [0,1,1]';
j1_rot_angle= 0; % [-pi/2, pi/2]

trf_joint22_linkla= make_transform([0, 0, -0.05], 0, 0, 0, trf_linkle_llal ); 
trf_linklw_joint22= make_transform_revolute(j10_rot_axis_j1, j1_rot_angle, trf_joint22_linkla); 
plot_axes(trf_linklw_joint22, 'L_9', false, axis_length); 
make_child(trf_linklw_joint22, trf_viz_lwrist);

%% Joint 23 : right lower arm, right wrist : Revolute
j11_rot_axis_j1= [1,0,0]';
j1_rot_angle= 0; % [-pi/2, pi/2]

trf_joint23_linkra= make_transform([0, 0, -0.05], 0, 0, 0, trf_linkre_rlal ); 
trf_linkrw_joint23= make_transform_revolute(j11_rot_axis_j1, j1_rot_angle, trf_joint23_linkra); 
plot_axes(trf_linkrw_joint23, 'L_10', false, axis_length); 
make_child(trf_linkrw_joint23, trf_viz_rwrist);

%% Joint 24 : left wrist link, left hand link : fixed
trf_linklw_lhl= make_transform([0,0,-0.055],0,0,0, trf_linklw_joint22);
make_child(trf_linklw_lhl,trf_viz_lhand);

%% Joint 25 : right wrist link, right hand link : fixed
trf_linkrw_rhl= make_transform([0,0,-0.055],0,0,0, trf_linkrw_joint23);
make_child(trf_linkrw_rhl,trf_viz_rhand);

%% Animation: One joint at a time
for q1=[linspace(0, -pi/2, 30), linspace(-pi/2, pi/2, 30), linspace(pi/2, 0, 30)]
    set(handle_axes, 'XLim', [-0.50,0.50], 'YLim', [-0.50,0.50], 'ZLim', [0,0.8]);
    trf_q1= makehgtform('axisrotate', j1_rot_axis_j1, q1);
    set(trf_linktorso_joint4, 'Matrix', trf_q1);
    drawnow;
    pause(0.002);
end

for q1=[linspace(0, -pi/2, 30), linspace(-pi/2, pi/2, 30), linspace(pi/2, 0, 30)]
    set(handle_axes, 'XLim', [-0.50,0.50], 'YLim', [-0.50,0.50], 'ZLim', [0,0.8]);
    trf_q1= makehgtform('axisrotate', j2_rot_axis_j1, q1);
    set(trf_linkhdtilt_joint4, 'Matrix', trf_q1);
    drawnow;
    pause(0.002);
end

for q1=[linspace(0, -pi/4, 30), linspace(-pi/4, pi/4, 30), linspace(pi/4, 0, 30)]
    set(handle_axes, 'XLim', [-0.50,0.50], 'YLim', [-0.50,0.50], 'ZLim', [0,0.8]);
    trf_q1= makehgtform('axisrotate', j3_rot_axis_j1, q1);
    set(trf_linkneck_joint6, 'Matrix', trf_q1);
    drawnow;
    pause(0.002);
end

for q1=[linspace(0, -pi/4, 30), linspace(-pi/4, pi/4, 30), linspace(pi/4, 0, 30)]
    set(handle_axes, 'XLim', [-0.50,0.50], 'YLim', [-0.50,0.50], 'ZLim', [0,0.8]);
    trf_q1= makehgtform('axisrotate', j4_rot_axis_j1, q1);
    set(trf_linklshf_joint12, 'Matrix', trf_q1);
    drawnow;
    pause(0.002);
end

for q1=[linspace(0, -pi/4, 30), linspace(-pi/4, pi/4, 30), linspace(pi/4, 0, 30)]
    set(handle_axes, 'XLim', [-0.50,0.50], 'YLim', [-0.50,0.50], 'ZLim', [0,0.8]);
    trf_q1= makehgtform('axisrotate', j5_rot_axis_j1, q1);
    set(trf_linkrshf_joint13, 'Matrix', trf_q1);
    drawnow;
    pause(0.002);
end

for q1=[linspace(0, -pi/4, 30), linspace(-pi/4, pi/4, 30), linspace(pi/4, 0, 30)]
    set(handle_axes, 'XLim', [-0.50,0.50], 'YLim', [-0.50,0.50], 'ZLim', [0,0.8]);
    trf_q1= makehgtform('axisrotate', j6_rot_axis_j1, q1);
    set(trf_linklshu_joint14, 'Matrix', trf_q1);
    drawnow;
    pause(0.002);
end

for q1=[linspace(0, -pi/4, 30), linspace(-pi/4, pi/4, 30), linspace(pi/4, 0, 30)]
    set(handle_axes, 'XLim', [-0.50,0.50], 'YLim', [-0.50,0.50], 'ZLim', [0,0.8]);
    trf_q1= makehgtform('axisrotate', j7_rot_axis_j1, q1);
    set(trf_linkrshu_joint15, 'Matrix', trf_q1);
    drawnow;
    pause(0.002);
end

for q1=[linspace(0, -pi/2, 30), linspace(-pi/2, 0, 30), 0]
    set(handle_axes, 'XLim', [-0.50,0.50], 'YLim', [-0.50,0.50], 'ZLim', [0,0.8]);
    trf_q1= makehgtform('axisrotate', j8_rot_axis_j1, q1);
    set(trf_linkle_joint18, 'Matrix', trf_q1);
    drawnow;
    pause(0.002);
end

for q1=[linspace(0, -pi/2, 30), linspace(-pi/2, 0, 30), 0]
    set(handle_axes, 'XLim', [-0.50,0.50], 'YLim', [-0.50,0.50], 'ZLim', [0,0.8]);
    trf_q1= makehgtform('axisrotate', j9_rot_axis_j1, q1);
    set(trf_linkre_joint19, 'Matrix', trf_q1);
    drawnow;
    pause(0.002);
end

for q1=[linspace(0, -pi/4, 30), linspace(-pi/4, pi/4, 30), linspace(pi/4, 0, 30)]
    set(handle_axes, 'XLim', [-0.5,0.5], 'YLim', [-0.5,0.5], 'ZLim', [0,0.8]);
    trf_q1= makehgtform('axisrotate', j10_rot_axis_j1, q1);
    set(trf_linklw_joint22, 'Matrix', trf_q1);
    drawnow;
    pause(0.002);
end

for q1=[linspace(0, -pi/4, 30), linspace(-pi/4, pi/4, 30), linspace(pi/4, 0, 30)]
    set(handle_axes, 'XLim', [-0.50,0.50], 'YLim', [-0.50,0.50], 'ZLim', [0,0.8]);
    trf_q1= makehgtform('axisrotate', j11_rot_axis_j1, q1);
    set(trf_linkrw_joint23, 'Matrix', trf_q1);
    drawnow;
    pause(0.002);
end
%% Animation: All joints together.
q_init= 0.5*ones(10,1); % This leads to all joints being at 0.

while 1
    % infinite loop
    q_next= rand(10,1); 
    % rand() gives uniformly distributed random numbers in the interval [0,1]
    
    for t=0:0.02:1
        q= q_init + t*(q_next - q_init);
        q1= (pi/2)*(2*q(1) - 1);
        q2= (pi/2)*(2*q(2) - 1);
        q3= (pi/4)*(2*q(3) - 1);
        q4= (pi/4)*(2*q(4) - 1);
        q5= (pi/4)*(2*q(5) - 1);
        q6= (pi/4)*(2*q(6) - 1);
        q7= (pi/2)*(2*q(7) - 1);
        if q7 > 0
            q7=-q7;
        end
        q8= (pi/2)*(2*q(8) - 1);
        if q8 > 0
            q8=-q8;
        end
 
        q9= (pi/4)*(2*q(9) - 1);
        q10= (pi/4)*(2*q(10) - 1);
        
        set(handle_axes, 'XLim', [-0.50,0.50], 'YLim', [-0.50,0.50], 'ZLim', [0,0.8]);
        trf_q1= makehgtform('axisrotate', j1_rot_axis_j1, q1);
        set(trf_linktorso_joint4, 'Matrix', trf_q1);
        
        set(handle_axes, 'XLim', [-0.50,0.50], 'YLim', [-0.50,0.50], 'ZLim', [0,0.8]);
        trf_q2= makehgtform('axisrotate', j2_rot_axis_j1, q2);
        set(trf_linkhdtilt_joint4, 'Matrix', trf_q2);
        
        set(handle_axes, 'XLim', [-0.50,0.50], 'YLim', [-0.50,0.50], 'ZLim', [0,0.8]);
        trf_q3= makehgtform('axisrotate', j3_rot_axis_j1, q3);
        set(trf_linkneck_joint6, 'Matrix', trf_q3);
        
        set(handle_axes, 'XLim', [-0.50,0.50], 'YLim', [-0.50,0.50], 'ZLim', [0,0.8]);
        trf_q4= makehgtform('axisrotate', j4_rot_axis_j1, q4);
        set(trf_linklshf_joint12, 'Matrix', trf_q4);
        
         set(handle_axes, 'XLim', [-0.50,0.50], 'YLim', [-0.50,0.50], 'ZLim', [0,0.8]);
        trf_q5= makehgtform('axisrotate', j5_rot_axis_j1, q5);
        set(trf_linkrshf_joint13, 'Matrix', trf_q5);
    
        set(handle_axes, 'XLim', [-0.50,0.50], 'YLim', [-0.50,0.50], 'ZLim', [0,0.8]);
        trf_q6= makehgtform('axisrotate', j7_rot_axis_j1, q6);
        set(trf_linkrshu_joint15, 'Matrix', trf_q6);
        
        set(handle_axes, 'XLim', [-0.50,0.50], 'YLim', [-0.50,0.50], 'ZLim', [0,0.8]);
        trf_q7= makehgtform('axisrotate', j8_rot_axis_j1, q7);
        set(trf_linkle_joint18, 'Matrix', trf_q7);
        
        set(handle_axes, 'XLim', [-0.50,0.50], 'YLim', [-0.50,0.50], 'ZLim', [0,0.8]);
        trf_q8= makehgtform('axisrotate', j9_rot_axis_j1, q8);
        set(trf_linkre_joint19, 'Matrix', trf_q8);
        
        set(handle_axes, 'XLim', [-0.5,0.5], 'YLim', [-0.5,0.5], 'ZLim', [0,0.8]);
        trf_q9= makehgtform('axisrotate', j10_rot_axis_j1, q9);
        set(trf_linklw_joint22, 'Matrix', trf_q9);
        
        set(handle_axes, 'XLim', [-0.50,0.50], 'YLim', [-0.50,0.50], 'ZLim', [0,0.8]);
        trf_q10= makehgtform('axisrotate', j11_rot_axis_j1, q10);
        set(trf_linkrw_joint23, 'Matrix', trf_q10);
        drawnow;
        pause(0.005);
    end
    
    q_init= q_next;1
    
end



