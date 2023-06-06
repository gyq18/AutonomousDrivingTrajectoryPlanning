%% Find CFS for all obstacles
function [A, b, d] = FindCFS(xr, obstacle)
    % the point xr:2*1, obstacle 2n*4
    ncorner = size(obstacle, 2); %the number of vertices in the obstacles (4)
    nobj = size(obstacle, 1) / 2;
    d = inf * ones(nobj, 1); 
    % (aTx<=b):pre_A,pre_b
    pre_A = zeros(nobj, 2);
    pre_b = zeros(nobj, 1);
    counter = 0;
    index_true = ones(nobj, 1); % index_true(j)=0 means the jth line is excluded
    % After determining the function phi, F(xr) is: phi(xr) + delta_phi(xr)*(x-xr)>=0
    for j = 1:nobj
        counter = counter + 1;
        obj = obstacle(2 * j - 1:2 * j, :);

        for i = 1:ncorner
            corner1 = obj(:, i);
            corner2 = obj(:, mod(i, ncorner) + 1);
             % xr,corner1 and corner2 form a triangle and find the lengths of the three sides
            dist_r1 = norm(xr - corner1);
            dist_r2 = norm(xr - corner2);
            dist_12 = norm(corner1 - corner2);
            % If angle r12 is obtuse, then xr is closer to corner1 at this point
            if (dist_r1^2 + dist_12^2 - dist_r2^2) < -1e-4 % Cosine Theorem
                temp_d = dist_r1;
                temp_A = xr' - corner1';
                temp_b = temp_A * corner1;
            elseif (dist_r2^2 + dist_12^2 - dist_r1^2) < -1e-4 % If angle r21 is obtuse, then xr is closer to corner2 at this point
                temp_d = dist_r2;
                temp_A = xr' - corner2';
                temp_b = temp_A * corner2;
            else
                % If angle r12 and angle r21 are both acute, then the vertical line from xr to the segment of the line formed by corner1 and corner2 is the shortest
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

        % the diagonal point of the vertex nearest to xr, which should be on either side of the line Ax = b with xr
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

    % See if there are redundant constraints in pre_A and pre_b
    % Above, each obstacle corresponds to finding a straight line dividing xr and the obstacle itself. 
    % If excluding this line leaves a line that still splits this obstacle and xr, then this line can indeed be excluded from the constraint
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