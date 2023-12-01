function [pucks,number] = LoadPucks()
%读小车，返回小车对象及数量
    pucks = {};
    index = 1;
    while true
        node_name = ['E-puck' int2str(index)];
        puck_temp = wb_supervisor_node_get_from_def(node_name);
        if isNull(puck_temp)
            break
        end
        pucks{index} = puck_temp;
        index = index + 1;
    end
    number = index-1;
end