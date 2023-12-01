function index = GetSelfPuck()
%返回自身序号
    name = wb_robot_get_name();
    name_len = size(name,2);
    index = str2double(name(name_len));
    index = int16(index);
end