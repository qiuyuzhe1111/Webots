function angle = CalculateAngle(force,cur_position,goal,obstacle,zeta,Apoint,lambda1,lambda2,Po)
    puck_num = size(force,1);
    obstacle_num = size(obstacle,1);
    angle = zeros([puck_num,1]);
    for i = 1:puck_num
        if norm(force(i,:)) < zeta
            rei = zeros([obstacle_num,1]);
            for j = 1:obstacle_num
                rei(j) = norm(cur_position(i,:)-obstacle(j,:));
            end
            probs = min(min(rei));
            fesc = Apoint/(probs^lambda1+Po^lambda2);
            angle_temp = atan2(goal(i,2)-cur_position(i,2),goal(i,1)-cur_position(i,1));
            force(i,:) = force(i,:) + [fesc*cos(angle_temp) fesc*sin(angle_temp)];
        end
        angle(i) = atan2(force(i,2),force(i,1));
    end
end