function [A, b, d] = FindCFS(xr, obstacle)
    %% 针对障碍物所有obj，为当前参考点xr寻找凸可行集CFS：F(xr)
    % xr的大小2*1，obj的大小2*4（每一列是一个顶点的坐标）
    %% 首先确定函数phi：因为实例中的障碍物都是凸的，因此phi取为文献中的公式(23)
    % 1. 需要注意的是，公式（23）所示的是phi是凸函数，并且光滑，因此次梯度集合为单点集，只包含一个元素——梯度
    % 2. 此外我们需要寻找障碍物边界上距离xr最近的点：依次考虑各个相邻顶点
    ncorner = size(obstacle, 2); %障碍物顶点个数
    nobj = size(obstacle, 1) / 2;
    d = inf * ones(nobj, 1); % xr到障碍物的最小距离初始值设为无穷大
    % 考虑所有的障碍物，得到的凸可行集(aTx<=b)保存在pre_A和pre_b中
    pre_A = zeros(nobj, 2);
    pre_b = zeros(nobj, 1);
    counter = 0;
    index_true = ones(nobj, 1); % index_true(j)=0表示排除了第j条直线
    %% 确定了函数phi之后，F(xr)为：phi(xr) + delta_phi(xr)*(x-xr)>=0
    for j = 1:nobj
        counter = counter + 1;
        obj = obstacle(2 * j - 1:2 * j, :);

        for i = 1:ncorner
            corner1 = obj(:, i);
            corner2 = obj(:, mod(i, ncorner) + 1);
            % xr,corner1以及corner2三点构成三角形，求三边的长度
            dist_r1 = norm(xr - corner1);
            dist_r2 = norm(xr - corner2);
            dist_12 = norm(corner1 - corner2);
            % 若角r12为钝角，则此时xr距离corner1更近
            if (dist_r1^2 + dist_12^2 - dist_r2^2) < -1e-4 %余弦定理
                temp_d = dist_r1;
                temp_A = xr' - corner1';
                temp_b = temp_A * corner1;
            elseif (dist_r2^2 + dist_12^2 - dist_r1^2) < -1e-4 % 若角r21为钝角，则此时xr距离corner2更近
                temp_d = dist_r2;
                temp_A = xr' - corner2';
                temp_b = temp_A * corner2;
            else % 若角r12以及角r21均为锐角，则xr到由corner1和corner2构成的直线段的垂线最短
                project_length = (xr - corner1)' * (corner2 - corner1) / dist_12;
                temp_d = sqrt(dist_r1^2 - project_length^2);
                temp_A = [corner1(2) - corner2(2), corner2(1) - corner1(1)];
                temp_b = corner2(1) * corner1(2) - corner1(1) * corner2(2);
            end

            if temp_d < d(j)
                d(j) = temp_d;
                single_A = temp_A;
                single_b = temp_b;
            end

        end

        length_A = norm(single_A);
        single_A = single_A / length_A;
        single_b = single_b / length_A;

        %     %% 离xr最近的顶点的对角点，与xr应该分居直线Ax=b的两侧
        for kkk = 1:ncorner

            if single_A * obj(:, kkk) < single_b
                single_A = -single_A;
                single_b = -single_b;
                break;
            end

        end

        pre_A(counter, :) = single_A;
        pre_b(counter, :) = single_b;

    end

    %% 看看是否pre_A和pre_b中是否有冗余约束
    %% 在上面，每一个障碍物都对应找了一条直线分割xr和障碍物本身。如果排除这条直线，剩下的直线仍然可以分割这个障碍物和xr,则这条直线确实可以从约束中排除
    for j = 1:nobj
        temp_index = index_true;
        temp_index(j) = 0;
        cur_index = find(temp_index > 0);
        cur_A = pre_A(cur_index, :);
        cur_b = pre_b(cur_index);
        obj = obstacle(2 * j - 1:2 * j, :);

        result = cur_A * obj - repmat(cur_b, 1, 4);
        flag = sum(result >= 0, 2);

        if ~isempty(find(flag >= 4, 1))
            index_true(j) = 0;
        end

    end

    final_index = find(index_true > 0);
    A = pre_A(final_index, :);
    b = pre_b(final_index);
end