function [position, orientation, R_c2w] = estimatePose(data, t)  
    %% CHANGE THE NAME OF THE FUNCTION TO estimatePose
    % Please note that the coordinates for each corner of each AprilTag are
    % defined in the world frame, as per the information provided in the
    % handout. Ideally a call to the function getCorner with ids of all the
    % detected AprilTags should be made. This function should return the X and
    % Y coordinate of each corner, or each corner and the centre, of all the
    % detected AprilTags in the image. You can implement that anyway you want
    % as long as the correct output is received. A call to that function
    % should made from this function.
    %% Input Parameter Defination
    % data = the entire data loaded in the current dataset
    % t = index of the current data in the dataset
    
    %% Output Parameter Defination
    
    % position = translation vector representing the position of the
    % drone(body) in the world frame in the current time, in the order ZYX
    
    % orientation = euler angles representing the orientation of the
    % drone(body) in the world frame in the current time, in the order ZYX

    %R_c2w = Rotation which defines camera to world frame

  intrinsic_matrix = [311.0520        0        201.8724;
 
  0         311.3885    113.6210;
 
  0            0           1   ];


 %width of square=0.152 b/w column 3 and 4  , 6 and 7 0.178
 
  % getting 4 points of any april tag in the given image at any instant
  % this is in pixels or camera frame

  res = getCorner(data(t).id);% output will be 4x2xlength

   
  % forming A matrix
  
 
  
  for i=1:length(data(t).id)
      % each april tag id is associated with four points 
      % using 4 points make a matrix of type Ah=0
         % xi world coordinate is from res
        % xip camera coordinate is from data

        % for point 1
        xip=data(t).p1(1,i);
        yip=data(t).p1(2,i);
        xi=res(1,1,i);
        yi=res(1,2,i);
        if(i==1)
                 aux=[xi yi 1 0 0 0 -xi*xip -xip*yi -xip;
                        0 0 0 xi yi 1 -yip*xi -yip*yi  -yip];
        else 
            aux_0=[xi yi 1 0 0 0 -xi*xip -xip*yi -xip;
                        0 0 0 xi yi 1 -yip*xi -yip*yi  -yip];
            aux=[aux;aux_0];

        end
        
         % for point 2
        xip=data(t).p2(1,i);
        yip=data(t).p2(2,i);
        xi=res(2,1,i);
        yi=res(2,2,i);
        aux_1=[xi yi 1 0 0 0 -xi*xip -xip*yi -xip;
            0 0 0 xi yi 1 -yip*xi -yip*yi  -yip];
        aux=[aux;aux_1];
         % for point 3
        xip=data(t).p3(1,i);
        yip=data(t).p3(2,i);
        xi=res(3,1,i);
        yi=res(3,2,i);
        aux_2=[xi yi 1 0 0 0 -xi*xip -xip*yi -xip;
            0 0 0 xi yi 1 -yip*xi -yip*yi  -yip];
        aux=[aux;aux_2];
         % for point 4
        xip=data(t).p4(1,i);
        yip=data(t).p4(2,i);
        xi=res(4,1,i);
        yi=res(4,2,i);
        aux_3=[xi yi 1 0 0 0 -xi*xip -xip*yi -xip;
            0 0 0 xi yi 1 -yip*xi -yip*yi  -yip];
        aux=[aux;aux_3];

  end

  A=aux;

  [U, S, V] = svd(A);

% making homography matrix from 9th column of V
h_formed=sign(V(9,9))*[V(1,9) V(2,9) V(3,9);
          V(4,9) V(5,9) V(6,9);
          V(7,9) V(8,9) V(9,9)];

r_t=inv(intrinsic_matrix)*h_formed;

r1_cap=[r_t(1,1);r_t(2,1);r_t(3,1)];
r2_cap=[r_t(1,2);r_t(2,2);r_t(3,2)];
t_cap=[r_t(1,3);r_t(2,3);r_t(3,3)];

aux_mat=[r1_cap r2_cap cross(r1_cap,r2_cap)];

[U, S, V] = svd(aux_mat);

Final_R=U*[1 0 0;0 1 0;0 0 det(U*(V'))]*transpose(V);

R_c2w=transpose(Final_R);

t_final=t_cap/norm(r1_cap);

T_cw=[Final_R t_final;0 0 0 1];% world to camera transformation matrix
% ZYX is the convention
% from the image, base with respect to camera

R_cr = [1/sqrt(2) -1/sqrt(2) 0;
    -1/sqrt(2) -1/sqrt(2) 0;
    0 0 -1];


o_cr = [-0.04;
    0;
    -0.03];

T_cr = [R_cr o_cr;
    0 0 0 1];

T_wr = inv(T_cw)*T_cr;

R_wr = T_wr(1:3,1:3);
position = T_wr(1:3,4);

orientation = rotm2eul(R_wr);% convention is ZYX default

end