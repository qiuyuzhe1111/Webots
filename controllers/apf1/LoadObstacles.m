function [obs,number] = LoadObstacles()
%读障碍物，返回障碍物对象及数量
    obs = {};
    index = 1;
    while true
        node_name = ['obs' int2str(index)];
        obstacle_temp = wb_supervisor_node_get_from_def(node_name);
        if isNull(obstacle_temp)
            break
        end
        obs{index} = obstacle_temp;
        index = index + 1;
    end
    number = index-1;
end