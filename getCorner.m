function res = getCorner(id)
%% CHANGE THE NAME OF THE FUNCTION TO getCorner
    %% Input Parameter Description
    % id = List of all the AprilTag ids detected in the current image(data)
    
    %% Output Parameter Description
    % res = List of the coordinates of the 4 corners (or 4 corners and the
    % centre) of each detected AprilTag in the image in a systematic method



 Tag_ids =[0, 12, 24, 36, 48, 60, 72, 84,  96;
 
 1, 13, 25, 37, 49, 61, 73, 85,  97;

 2, 14, 26, 38, 50, 62, 74, 86,  98;
 
 3, 15, 27, 39, 51, 63, 75, 87,  99;
 
 4, 16, 28, 40, 52, 64, 76, 88, 100;
 
 5, 17, 29, 41, 53, 65, 77, 89, 101;
 
 6, 18, 30, 42, 54, 66, 78, 90, 102;
 
 7, 19, 31, 43, 55, 67, 79, 91, 103;
 
 8, 20, 32, 44, 56, 68, 80, 92, 104;
 
 9, 21, 33, 45, 57, 69, 81, 93, 105;

 10, 22, 34, 46, 58, 70, 82, 94, 106;

 11, 23, 35, 47, 59, 71, 83, 95, 107];


res_is=zeros(4,2,length(id));
id_count=1;

for i=1:length(id)
    for x=1:12
        for y=1:9
           if(Tag_ids(x,y)==id(i)) 

               aux_x=x;
               aux_y=y;

               [p1_x,p1_y]=getp1(aux_x,aux_y);

               p2_x = p1_x;p2_y=p1_y+0.152;

               p3_x = p1_x-0.152 ; p3_y=p2_y;

               p4_x=p3_x; p4_y=p1_y;

               res_is(1,1,id_count)=p1_x;
               res_is(1,2,id_count)=p1_y;
               res_is(2,1,id_count)=p2_x;
               res_is(2,2,id_count)=p2_y;
               res_is(3,1,id_count)=p3_x;
               res_is(3,2,id_count)=p3_y;
               res_is(4,1,id_count)=p4_x;
               res_is(4,2,id_count)=p4_y;
               id_count=id_count+1;
           end
       end
   end

end
res=res_is;
end


function [x1,y1]=getp1(x,y)
 x1=(2*x-1)*0.152;
 y_aux=2*(y-1)*0.152;
if y>=4 && y<7
    y_aux=y_aux-0.152+0.178;
elseif y>=7
    y_aux=y_aux-0.152-0.152+0.178+0.178;
else
    y_aux=1*y_aux;
end

y1=y_aux;
end
