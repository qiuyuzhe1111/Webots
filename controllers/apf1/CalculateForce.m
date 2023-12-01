function force = CalculateForce(cur_position,goal,obstacle,k,beta,Po,a,r,d0,d1,a1,rer_limit,Pg,target_p,apg)
    puck_num = size(cur_position, 1);
    obstacle_num = size(obstacle, 1);
    force_att = zeros([puck_num, 2]);%吸引力
    force_ata = zeros([puck_num, 2]);%排斥力中的斥力
    force_rer = zeros([puck_num, 2]);%排斥力中的引力
    force_agent = zeros([puck_num, 2]);
    force_goal = zeros([puck_num, 2]);
    for i = 1:puck_num
        dist = norm(goal(i,:)-cur_position(i,:));
        angle = atan2(goal(i,2)-cur_position(i,2),goal(i,1)-cur_position(i,1));
        f_att = k*exp(-dist)*dist;
        force_att(i,:) = [f_att*cos(angle) f_att*sin(angle)];
        force_ata_single = zeros([obstacle_num, 2]);
        force_rer_single = zeros([obstacle_num, 2]);
        dist2 = norm(target_p - cur_position(i,:));
        angle2 = atan2(cur_position(i,2)-target_p(2),cur_position(i,1)-target_p(1));
        if dist2 < Pg
             force_goal(i,:) = [apg*cos(angle2) apg*sin(angle2)];
        else
            force_goal(i,:) = [0 0];
        end
        for j = 1:obstacle_num
            rre = norm(cur_position(i,:)-obstacle(j,:))-r;
            if rre < 0.01
                rre = 0.01;
            end
            if rre > Po
                force_rer_single(j,:) = [0 0];
                force_ata_single(j,:) = [0 0];
            else
                rer_mag = beta * (1/rre-1/Po)/(rre^2)*(dist^0.5);
                if rer_mag < 0
                    rer_mag = 0;
                end
                rer_angle = atan2(obstacle(j,2)-cur_position(i,2),obstacle(j,1)-cur_position(i,1));
                ddist = dist;
                if ddist < 1
                    ddist = 1;
                end
                ata_mag = 0.5*a*beta*((1/rre-1/Po)^2)*(ddist^(-0.5));
		        if ata_mag > rer_limit
                    ata_mag = rer_limit;
                end
                ata_angle = angle;
                force_rer_single(j,:) = -[rer_mag*cos(rer_angle) rer_mag*sin(rer_angle)];
                force_ata_single(j,:) = [ata_mag*cos(ata_angle) ata_mag*sin(ata_angle)];
            end
        end
        force_ata(i,:) = sum(force_ata_single,1);
        force_rer(i,:) = sum(force_rer_single,1);
        for j = 1:puck_num
            if i == j
                continue
            end
            r0 = norm(cur_position(i,:)-cur_position(j,:));
            if (r0<d1)
                F=a1*(1/r0-d0/(r0^2));
            else
                F=0;
            end
            theta = atan2(cur_position(i,2)-cur_position(j,2),cur_position(i,1)-cur_position(j,1));
            force_agent(i,:) = force_agent(i,:) + [abs(F)*cos(theta) abs(F)*sin(theta)];
        end
    end
    force = force_att + force_rer + force_ata + force_agent;
end