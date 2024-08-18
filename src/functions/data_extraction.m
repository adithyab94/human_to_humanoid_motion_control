%%% Data extraction
% https://fr.mathworks.com/matlabcentral/answers/493775-importing-motion-capture-data-drf-as-a-text-file

% clear all

% % GUIDE : to get the position of the body i in the timestep k :
% timestep = 1;
% body = 8;
% test = body_poses(body,timestep,:);
% test = test(:);

line_jump = find(data==newline);

%% Metadata
% Correspondance between body names and index in the vectors
body_ids = ["right foot", "left foot", "right upper leg", "left upper leg", ...
            "right lower leg", "left lower arm", "left lower leg", "left upper arm", ...
            "hip", "right hand", "right upper arm", "right lower arm", "left shoulder", "right shoulder",...
            "left hand", "chest", "head"];
 
                
        
body_links = [2 7;7 4;4 9;1 5;5 3;3 9;9 16;16 14;14 11;11 12;12 10;16 13;13 8;8 6;6 15;16 17];
body_joints = [18 20;17 19;14 16;13 15;...
               16 18;9 7;15 17;7 5;...
               2 1;12 10;8 6;10 8;5 3;6 3;...
               11 9;3 2;4 3];

n_body_links = numel(body_links(:,1));

joint_links = [4 3;3 5;5 7;7 9;9 11;3 6;6 8;8 10;10 12;3 2;2 1;1 13;13 15;...
         15 17;17 19;1 14;14 16;16 18;18 20];
n_joint_links = numel(joint_links(:,1));


hanavan_body_ids = ["right foot", "left foot", "right upper leg", "left upper leg", ...
                    "right lower leg", "left lower arm", "left lower leg", "left upper arm", ...
                    "hip", "right hand", "right upper arm", "right lower arm", ...
                    "left hand", "chest", "head"];   
hanavan_body_joints = [18; 17; 14; 13; 16; 9; 15; 7; 2; 12; 8; 10; 11; 3; 4]; 

%% ts : the time serie
ts_locate = strfind(convertCharsToStrings(data),"ts");
tss = 0;
for i = ts_locate
    jump_locate = find(line_jump>i);
    i_end = line_jump(jump_locate);
    tss = [tss, str2num(data(i+2:i_end(1)))];
end
tss = tss(2:end);
tss = tss-tss(1);

%% bodies : the data from the 17 bodies
d17_locate = strfind(convertCharsToStrings(data),"6d ");
N  = numel(d17_locate);
body_poses = zeros(17,N,6);
body_rot_matrices = zeros(17,N,9);
for k = 1:N
    i = d17_locate(k);
    jump_locate = find(line_jump>i);
    i_end = line_jump(jump_locate);
    sub_string = data(i+6:i_end(1));
    
    % Sometimes, a body is lost. In the data file, it will be written as
    % "6d 16" instead of "6d 17" for example
    n_body_ok = str2num(data(i+3:i+4)); 

    index_before = 1;
    for body = 1:n_body_ok-1
        str_to_find = "[" + num2str(body) + " 1.000]";
        index_after = strfind(convertCharsToStrings(sub_string),str_to_find)-1;
        if(index_after > 0)
    %         disp(sub_string(index_before:index_after))
            sub_sub_string = sub_string(index_before:index_after);
            subsubsplit = split(convertCharsToStrings(sub_sub_string),"[");
            almost_vec = convertStringsToChars(subsubsplit(3));
            almost_vec = almost_vec(1:end-1);
            pose = split(convertCharsToStrings(almost_vec)," ");
            pose = str2double(convertStringsToChars(pose));
            body_poses(body,k,:) = pose;

            almost_rot_mat = convertStringsToChars(subsubsplit(4));
            almost_rot_mat = almost_rot_mat(1:end-2);
            rot_mat = split(convertCharsToStrings(almost_rot_mat)," ");
            rot_mat = str2double(convertStringsToChars(rot_mat));
            body_rot_matrices(body,k,:) = rot_mat;

            index_before = index_after;
        end
    end
    
    body = n_body_ok;
    sub_sub_string = sub_string(index_before:end);
    subsubsplit = split(convertCharsToStrings(sub_sub_string),"[");
    almost_vec = convertStringsToChars(subsubsplit(3));
    almost_vec = almost_vec(1:end-1);
    pose = split(convertCharsToStrings(almost_vec)," ");
    pose = str2double(convertStringsToChars(pose));
    body_poses(body,k,:) = pose;
    
    almost_rot_mat = convertStringsToChars(subsubsplit(4));
    almost_rot_mat = almost_rot_mat(1:end-2);
    rot_mat = split(convertCharsToStrings(almost_rot_mat)," ");
    rot_mat = str2double(convertStringsToChars(rot_mat));
    body_rot_matrices(body,k,:) = rot_mat;
end
body_poses = body_poses*1e-3;

%% joints : the data from the 20 joints (not all the groups have those, comment this section if it raises an error
dj11_locate = strfind(convertCharsToStrings(data),"6dj 1 1");
joint_poses = zeros(20,N,6);
joint_rot_matrices = zeros(20,N,9);
for k = 1:N
    i = dj11_locate(k);
    jump_locate = find(line_jump>i);
    i_end = line_jump(jump_locate);
    sub_string = data(i+14:i_end(1));
    index_before = 1;
    for body = 1:19
        str_to_find = "[" + num2str(body) + " 1.000]";
        index_after = strfind(convertCharsToStrings(sub_string),str_to_find)-1;
%         disp(sub_string(index_before:index_after))
        sub_sub_string = sub_string(index_before:index_after);
        subsubsplit = split(convertCharsToStrings(sub_sub_string),"[");
        almost_vec = convertStringsToChars(subsubsplit(3));
        almost_vec = almost_vec(1:end-1);
        pose = split(convertCharsToStrings(almost_vec)," ");
        pose = str2double(convertStringsToChars(pose));
        joint_poses(body,k,:) = pose;
        
        almost_rot_mat = convertStringsToChars(subsubsplit(4));
        almost_rot_mat = almost_rot_mat(1:end-2);
        rot_mat = split(convertCharsToStrings(almost_rot_mat)," ");
        rot_mat = str2double(convertStringsToChars(rot_mat));
        joint_rot_matrices(body,k,:) = rot_mat;
        
        index_before = index_after;
    end
    
    body = 20;
    sub_sub_string = sub_string(index_before:end);
    subsubsplit = split(convertCharsToStrings(sub_sub_string),"[");
    almost_vec = convertStringsToChars(subsubsplit(3));
    almost_vec = almost_vec(1:end-1);
    pose = split(convertCharsToStrings(almost_vec)," ");
    pose = str2double(convertStringsToChars(pose));
    joint_poses(body,k,:) = pose;
    
    almost_rot_mat = convertStringsToChars(subsubsplit(4));
    almost_rot_mat = almost_rot_mat(1:end-2);
    rot_mat = split(convertCharsToStrings(almost_rot_mat)," ");
    rot_mat = str2double(convertStringsToChars(rot_mat));
    joint_rot_matrices(body,k,:) = rot_mat;
end
joint_poses = joint_poses*1e-3;

%% Body frames 
body_xi = zeros(17,N,3);
body_yi = zeros(17,N,3);
body_zi = zeros(17,N,3);
for timestep = 1:N
    for body = 1:17
        if(body ~= 9)
            i = body_joints(body,1);
            j = body_joints(body,2);
            posei = joint_poses(i,timestep,:);
            Xi = posei(:,:,1);
            Yi = posei(:,:,2);
            Zi = posei(:,:,3);
            Pi = [Xi;Yi;Zi];
            rot_mat = joint_rot_matrices(i,timestep,:);
            r11i = rot_mat(:,:,1);
            r12i = rot_mat(:,:,2);
            r13i = rot_mat(:,:,3);
            r21i = rot_mat(:,:,4);
            r22i = rot_mat(:,:,5);
            r23i = rot_mat(:,:,6);
            r31i = rot_mat(:,:,7);
            r32i = rot_mat(:,:,8);
            r33i = rot_mat(:,:,9);
            R = [r11i r12i r13i;r21i r22i r23i;r31i r32i r33i];
            xi = R*[1 0 0]';
            yi = R*[0 1 0]';
            zi = R*[0 0 1]';

            posej = joint_poses(j,timestep,:);
            Xj = posej(:,:,1);
            Yj = posej(:,:,2);
            Zj = posej(:,:,3);
            Pj = [Xj;Yj;Zj];
    %         rot_mat = joint_rot_matrices(j,timestep,:);
    %         r11j = rot_mat(:,:,1);
    %         r12j = rot_mat(:,:,2);
    %         r13j = rot_mat(:,:,3);
    %         r21j = rot_mat(:,:,4);
    %         r22j = rot_mat(:,:,5);
    %         r23j = rot_mat(:,:,6);
    %         r31j = rot_mat(:,:,7);
    %         r32j = rot_mat(:,:,8);
    %         r33j = rot_mat(:,:,9);
    %         R = [r11j r12j r13j;r21j r22j r23j;r31j r32j r33j];
    %         xj = R*[1 0 0]';
    %         yj = R*[0 1 0]';
    %         zj = R*[0 0 1]';

            vec_xi = Pj-Pi;
            vec_xi = vec_xi/norm(vec_xi);
            vec_yi = cross(vec_xi,yi);
            vec_yi = vec_yi/norm(vec_yi);
            vec_zi = cross(vec_xi,vec_yi);

            body_xi(body,timestep,:) = vec_xi;
            body_yi(body,timestep,:) = vec_yi;
            body_zi(body,timestep,:) = vec_zi;
        else
            i_A = body_joints(9,1);
            i_B = body_joints(9,2);
            
            poseA = joint_poses(i_A,timestep,:);
            XA = poseA(:,:,1);
            YA = poseA(:,:,2);
            ZA = poseA(:,:,3);
            PA = [XA;YA;ZA];
            
            poseB = joint_poses(i_B,timestep,:);
            XB = poseB(:,:,1);
            YB = poseB(:,:,2);
            ZB = poseB(:,:,3);
            PB = [XB;YB;ZB];
            
            vec_xi = PB-PA;
            vec_xi = vec_xi/norm(vec_xi);
            
            poseAl = joint_poses(13,timestep,:);
            XAl = poseAl(:,:,1);
            YAl = poseAl(:,:,2);
            ZAl = poseAl(:,:,3);
            PAl = [XAl;YAl;ZAl];
            
            poseAr = joint_poses(14,timestep,:);
            XAr = poseAr(:,:,1);
            YAr = poseAr(:,:,2);
            ZAr = poseAr(:,:,3);
            PAr = [XAr;YAr;ZAr];
            
            vec_yi = PAr-PAl;
            vec_yi = vec_yi/norm(vec_yi);
            
            vec_zi = cross(vec_xi,vec_yi);

            body_xi(body,timestep,:) = vec_xi;
            body_yi(body,timestep,:) = vec_yi;
            body_zi(body,timestep,:) = vec_zi;
            
        end
        
    end
end


