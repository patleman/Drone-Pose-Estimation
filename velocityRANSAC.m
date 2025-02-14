function [Vel] = velocityRANSAC(V, h, Zps, R_c2w, e)
    % Input:
    % V          - Velocity vector
    % h          - Matrix to be used to solve for velocity
    % Zps        - Heights of the feature points
    % R_c2w      - Rotation matrix from camera to world frame
    % e          - Threshold for determining inliers
    
    % Output:
    % Vel        - Estimated velocity (after RANSAC)
    
    % Number of iterations for RANSAC
    maxIter = 1000;
    bestInlierCount = 0;
    bestVel = zeros(size(h, 2), 1);  % Best velocity (initially zero)
    
    % Set the number of data points (rows in h)
    numDataPoints = size(h, 1);
    
    for iter = 1:maxIter
        % Step 1: Randomly sample a minimal set of points to estimate velocity
        idx = randperm(numDataPoints, 4);  % Randomly select 4 points (change if necessary)
        
        % Step 2: Extract subset of h and V for the random points
        h_subset = h(idx, :);
        V_subset = V(idx);
        
        % Step 3: Solve for velocity using the selected subset
        Vel_candidate = pinv(h_subset) * V_subset;  % Use the pseudoinverse to estimate the velocity
        
        % Step 4: Calculate errors for all data points
        residuals = h * Vel_candidate - V;
        
        % Step 5: Count the number of inliers (points with error less than threshold)
        inliers = abs(residuals) < e;  % Find the points where residuals are smaller than threshold
        inlierCount = sum(inliers);
        
        % Step 6: If this model has more inliers, update the best model
        if inlierCount > bestInlierCount
            bestInlierCount = inlierCount;
            bestVel = Vel_candidate;
        end
    end
    
    % Return the best estimated velocity (Vel)
    Vel = bestVel;
end

%function [Vel] = velocityRANSAC(optV,optPos,Z,R_c2w,e)
% %% CHANGE THE NAME OF THE FUNCTION TO velocityRANSAC
%     %% Input Parameter Description
%     % optV = The optical Flow
%     % optPos = Position of the features in the camera frame 
%     % Z = Height of the drone
%     % R_c2w = Rotation defining camera to world frame
%     % e = RANSAC hyper parameter    
% 
%     %% Output Parameter Description
%     % Vel = Linear velocity and angualr velocity vector
%     Vel=pinv(optPos)*optV;
%     p_inliers_count=0;
% 
%     for i=1:20
%         inliers_count=0;
%         newoptPos=[];
%         newoptV=[];
%         for j=1:2:length(optPos)
%            if (norm(optPos(j:j+1,:)*Vel-optV(j:j+1)))^2<e
%              newoptPos=[newoptPos ;optPos(j:j+1,:)];
%              newoptV=[newoptV;optV(j:j+1,:)];
%              inliers_count=inliers_count+1;
%            end
%         end
% 
%         if(inliers_count>p_inliers_count)
%             Vel=pinv(newoptPos)*newoptV;
%         end
%         p_inliers_count=inliers_count;
%     end
% end
