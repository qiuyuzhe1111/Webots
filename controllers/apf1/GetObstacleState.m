function states = GetObstacleState(Obstacles)
    obs_num = size(Obstacles,2);
    states = zeros([obs_num,2]);
    for i = 1:obs_num
        cur_field = wb_supervisor_node_get_field(Obstacles{i},'translation');
        cur_state = wb_supervisor_field_get_sf_vec3f(cur_field);
        states(i,:) = cur_state(1:2);
    end
end