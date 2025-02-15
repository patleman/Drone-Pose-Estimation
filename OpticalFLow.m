%% PROJECT 2 VELOCITY ESTIMATION
close all;
clear all;
clc;
addpath('../data')

%Change this for both dataset 1 and dataset 4. Do not use dataset 9.
datasetNum = 4;

[sampledData, sampledVicon, sampledTime] = init(datasetNum);

%% INITIALIZE CAMERA MATRIX AND OTHER NEEDED INFORMATION
intrinsic_matrix=[311.0520        0        201.8724;
 
  0         311.3885    113.6210;
 
  0            0           1   ];

first_time=0;

for n = 2:length(sampledData)

    %% Initalize Loop load images
    prev_image=sampledData(n-1).img;
    cur_image=sampledData(n).img;


    %% Detect good points
    % from previous image
    GPoints = detectFASTFeatures(prev_image,'MinQuality',0.7);


    %% Initalize the tracker to the last frame.
    pointTracker = vision.PointTracker('MaxBidirectionalError',2);
    initialize(pointTracker,GPoints.Location,prev_image);

    %% Find the location of the next points;
    [points,point_validity] = pointTracker(cur_image);%here points are in pixels

       
    %% Calculate velocity
    % Use a for loop
    %if  (first_time == 0)
    dt=sampledData(n).t-sampledData(n-1).t;% computing dt time interval
        % prev_dt=dt;
        % first_time=1;
    % else  
    %     dt=0.4*(sampledData(n).t-sampledData(n-1).t)+0.6*prev_dt;
    %     prev_dt=dt;
    % end    

    
    current_image_points  =   [transpose(points);ones(1,length(points))];% ma
    previous_image_points =  [transpose(GPoints.Location);ones(1,length(GPoints.Location))];
    % defining A B V matrices
    A=[];
    B=[];
    V=[];
    Zps=[];
    %Getting 
    [position, orientation, R_c2w] = estimatePose(sampledData, n);
    
    for i=1:length(points)

        % first transforming the points(pixels) in points to camera frame 
        aux_cip=inv(intrinsic_matrix)*current_image_points(:,i);
        aux_pip=inv(intrinsic_matrix)*previous_image_points(:,i);

        x=aux_cip(1);
        y=aux_cip(2);
        
        %% Calculate Height 
        % height is different for each feature point
        point_vector=[x;y;1];
        % transforming this vector in world frame using R_c2w
         point_vector_world=R_c2w*point_vector;
         z_vector_world=[0;0;-position(3)];
         costheta=dot(point_vector_world,z_vector_world)/(norm(point_vector_world)*norm(z_vector_world));
         Z=position(3);
         zi=Z/costheta;
         Zps=[Zps;zi];
        a=[-1/zi 0 x/zi;
            0 -1/zi y/zi];

        b=[x*y -(1+x^2) y;
            1+y^2 -x*y -x];

        A=[A;a];
        B=[B;b];

        pdot=(aux_cip-aux_pip)/dt;

        V=[V;pdot(1);pdot(2)];

    end
    h=[A B];
    %Vel=pinv(h)*V;
    
       % Ensure the data is in double format
    h = double(h);
    V = double(V);
    Vel_initial = double(zeros(size(h,2), 1));  % Make sure the initial guess is also double
    
    % Define the objective function (residual error)
    objective_function = @(Vel) norm(h * Vel - V)^2;
    
    % Solve the optimization problem using lsqnonlin
    options = optimoptions('lsqnonlin', 'Display', 'off'); % Suppress display
    
    % Run the solver
    Vel_optimized = lsqnonlin(objective_function, Vel_initial, [], [], options);
    
    Vel=Vel_optimized;
    %% RANSAC    
    % Write your own RANSAC implementation in the file velocityRANSAC
    e=0.1;
    
    [Vel] = velocityRANSAC(V,h,Zps,R_c2w,e);

    %% Thereshold outputs into a range.
    % Not necessary
    
    %% Fix the linear velocity
    % Change the frame of the computed velocity to world frame
    nv=R_c2w*Vel(1:3);
    nvw=R_c2w*Vel(4:6);
    Vel=[nv;nvw];
    
    %% ADD SOME LOW PASS FILTER CODE
   
    % Not neceessary but recommended 
    estimatedV(:,n) = Vel;
    % 
    % if  (first_time == 0)
    %     estimatedV(:,n) = Vel;
    %     prev_vel=Vel;
    %     first_time=1;
    % else  
    %     estimatedV(:,n) = 0.6*Vel+0.4*prev_vel;
    %     prev_vel=0.6*Vel+0.4*prev_vel;
    % end   

    
    %% STORE THE COMPUTED VELOCITY IN THE VARIABLE estimatedV AS BELOW
    estimatedV(:,n) = Vel; % Feel free to change the variable Vel to anything that you used.
    % Structure of the Vector Vel should be as follows:
    % Vel(1) = Linear Velocity in X
    % Vel(2) = Linear Velocity in Y
    % Vel(3) = Linear Velocity in Z
    % Vel(4) = Angular Velocity in X
    % Vel(5) = Angular Velocity in Y
    % Vel(6) = Angular Velocity in Z
end

plotData(estimatedV, sampledData, sampledVicon, sampledTime, datasetNum)
