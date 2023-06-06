for id=1:2000
    %% readfile
    obsfile = [pwd, '\dataset-obs\dataset\obs\', num2str(id), '.csv'];
    taskfile = [pwd, '\dataset-obs\dataset\task\', num2str(id), '.csv'];
    trajfile = [pwd, '\dataset-obs\dataset\traj\', num2str(id), '.csv'];
    obs = readmatrix(obsfile);
    task = readmatrix(taskfile);
    traj = readmatrix(trajfile);
%     obs =  round(obs,1);
%     task = round(task,1);
%     traj = round(traj,1);
    
    obs = obs./60; %障碍物归一化
    task(1:2,:) =  task(1:2,:)./60;
    task(3,:) =  task(3,:)./pi;
    traj(:,1:2) = traj(:,1:2)./60;
    %角度先转化到-pi到pi中，再归一化至0至1中
    traj(traj(:,3)<-pi,:)=traj(traj(:,3)<-pi,:)+pi;
    traj(traj(:,3)>pi,:)=traj(traj(:,3)>pi,:)-pi;
    traj(:,3) = (traj(:,3)+pi)/(2*pi);
    traj(:,4) = (traj(:,4)+2.5)/5;
    %角度先转化到-pi到pi中，再归一化至0至1中
    traj(traj(:,5)<-pi,:)=traj(traj(:,5)<-pi,:)+pi;
    traj(traj(:,5)>pi,:)=traj(traj(:,5)>pi,:)-pi;
    traj(:,5) = (traj(:,5)+pi)/(2*pi);
    
    %% writefile
    obsfile = [pwd, '\dataset-obs-guiyi\obs\', num2str(id), '.csv'];
    taskfile = [pwd, '\dataset-obs-guiyi\task\', num2str(id), '.csv'];
    trajfile = [pwd, '\dataset-obs-guiyi\traj\', num2str(id), '.csv'];
    writematrix(obs,obsfile);
    writematrix(task,taskfile);
    writematrix(traj(:,1:5),trajfile);
end