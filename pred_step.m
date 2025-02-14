function [covarEst,uEst] = pred_step(uPrev,covarPrev,angVel,acc,dt)
%% BEFORE RUNNING THE CODE CHANGE NAME TO pred_step
    %% Parameter Definition
    % uPrev - is the mean of the prev state
    %covarPrev - covar of the prev state
    %angVel - angular velocity input at the time step
    %acc - acceleration at the timestep
    %dt - difference in time 
    

    
    n = 15+12;

    alpha = 0.001;
    beta = 2;
    k = 1;

    lemda = ((alpha^2)*(n+k)) - n;

    Q = 0.1*eye(12);


    sig_aug = [covarPrev zeros(15,12);
                  zeros(12,15) Q];
    chol_paug = chol(sig_aug,"lower");
    u_aug = [uPrev;
                       zeros(12,1)];
    %calculating sigma points
    X_sigma = zeros(27,55);
    
    X_sigma(:,1) = u_aug;
    
    for i = 2:28
        
            X_sigma(:,i) = u_aug + (sqrt(n + lemda)*chol_paug(:,i-1));

    end
    for  i=29:55
       
            X_sigma(:,i) = u_aug - (sqrt(n + lemda)*chol_paug(:,(i-28)));
     
    end

    %prpopagating sigma points
    prop_x = zeros(15,55);
     for i = 1:55
                x3 = [X_sigma(7,i);X_sigma(8,i);X_sigma(9,i)];
                x4 = [X_sigma(10,i);X_sigma(11,i);X_sigma(12,i)];
                x5 = [X_sigma(13,i);X_sigma(14,i);X_sigma(15,i)];
            
                roll = X_sigma(4,i); %Phi
                pitch = X_sigma(5,i); %Theta
                yaw = X_sigma(6,i); %Gamma
                
                %Rotation matrix defines the body in the world frame
                R = eul2rotm([yaw pitch roll]); 
            
                %Inverse of G Matrix
                Gq = [cos(pitch)*cos(yaw), -sin(yaw), 0;
                    cos(pitch)*sin(yaw), cos(yaw), 0;       
                    -sin(pitch), 0, 1];
        
                 p_model = [x3
                           inv(Gq)*R*(angVel-x4-X_sigma(16 : 18, i)) ;
                           R*(acc-x5-X_sigma(19 : 21, i))-[0;0;9.81] ;
                           X_sigma(22 : 24, i) ;
                           X_sigma(25 : 27, i)]*dt;   
        
                 prop_x(:,i) = X_sigma(1:15,i) + p_model; 
        
    end
   
     
    % calculating mean from prop_x 
    uEst = zeros(15,1);
     W = lemda/(lemda+n);

    uEst = uEst+(W*prop_x(:,1)); 
    for i = 2:55
        
         W = 1/(2*(lemda+n));
         uEst = uEst+(W*prop_x(:,i));
      
    end
    


    % calculating covariance matrix
   
    Wc = (lemda / (n + lemda)) + (1 - alpha^2 + beta);
    covarEst = Wc * (prop_x(:, 1) - uEst) * transpose((prop_x(:, 1) - uEst));
    for i = 2 : 55
       
            %Calculating the final predicted Covariance 
            Wc = 1/(2 * (n + lemda));
            covarEst = covarEst + (Wc * (prop_x(:, i) - uEst) * transpose((prop_x(:, i) - uEst)));
       
    end

 end

 