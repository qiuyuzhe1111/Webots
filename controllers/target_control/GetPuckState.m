function state = GetPuckState(Pucks)
    if strcmp(class(Pucks),'cell')
        puck_num = size(Pucks,2);
        state = zeros(puck_num,2);
        for i = 1:puck_num
            cur_field = wb_supervisor_node_get_field(Pucks{i},'translation');
            cur_state = wb_supervisor_field_get_sf_vec3f(cur_field);
            state(i,:) = cur_state(1:2);
        end
    else
        state = zeros([1,2]);
        cur_field = wb_supervisor_node_get_field(Pucks,'translation');
        cur_state = wb_supervisor_field_get_sf_vec3f(cur_field);
        state(1,:) = cur_state(1:2);
    end
end