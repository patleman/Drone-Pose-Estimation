function [uCurr,covar_curr] = upd_step(z_t,covarEst,uEst)
%% BEFORE RUNNING THE CODE CHANGE NAME TO upd_step
    %% Parameter Definition
    %z_t - is the sensor data at the time step
    %covarEst - estimated covar of the  state
    %uEst - estimated mean of the state
    
    
   
   

  
    
    

    %Number of variables in n
    n = 15;

    alpha = 0.001;
    beta = 2;
    k = 1;

    lemda = ((alpha^2)*(n+k)) - n;
    
    chol_paug = chol(covarEst,"lower");
    

    %calculating sigma Points
    X_sigma = zeros(15,31);
    X_sigma(:,1) = uEst;
    for i = 2:16
        
            X_sigma(:,i) = uEst + (sqrt(n + lemda)*chol_paug(:,i-1));
    end
    for i=17:31
            X_sigma(:,i) = uEst - (sqrt(n + lemda)*chol_paug(:,(i-16)));
        
    end
    
      %rotation matrix of the body wrt Camera frame.
    rot = [1/sqrt(2) -1/sqrt(2) 0
          -1/sqrt(2) -1/sqrt(2) 0  
                   0 0 -1];
    
    % putting X_sigma points in measurement model
    g = zeros(3,31);
    tvecc = [-0.04
                      0        
                      -0.03];
    
    
    tvec = -rot'*tvecc; 
    skew_vec = [0 -tvec(3) tvec(2);
             tvec(3) 0 -tvec(1);
             -tvec(2) tvec(1) 0]; 
    %Generating the Sigma Points
    for i = 1:31
        g(:,i) = rot*eul2rotm([X_sigma(6,i) X_sigma(5,i) X_sigma(4,i)])'*[X_sigma(7,i);X_sigma(8,i);X_sigma(9,i)] - rot*skew_vec*rot'*z_t(4:6);
    end
    
    Zm_u = zeros(3,1);

    %computing z mean
     W = lemda/(lemda+n);
     Zm_u = Zm_u+(W*g(:,1));
     for i = 2:31
       
        
            W = 1/(2*(lemda+n));
            Zm_u = Zm_u+(W*g(:,i));
     
    end
   % computing C matrix
     W = lemda/(lemda+n);
     C = W*(X_sigma(:,1) - uEst)*(g(:,1) - Zm_u)';

    for i = 2:31
        
         W = 1/(2*(lemda+n));
         C = C + W*(X_sigma(:,i) - uEst)*(g(:,i) - Zm_u)';
       
    end

     R = 0.01*eye(3);
% computing S matrix
     W = (lemda / (n + lemda)) + (1 - alpha^2 + beta);
     S = W*(g(:,1) - Zm_u)*(g(:,1) - Zm_u)' +R;
  



    for i = 2:31
        
            W = 1/(2*(lemda+n));
            S = S + W*(g(:,i) - Zm_u)*(g(:,i) - Zm_u)' +R;
        
    end

    %kalman gain
    Kt = C/S; 
    
    uCurr = uEst + Kt*(z_t(1:3)-Zm_u);
    
   
    covar_curr = covarEst - Kt*S*Kt';

end
