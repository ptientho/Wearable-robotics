function [Xp,Xh,LhipAvg,RhipAvg,Sp,Sh] = normalize_data(Lhip_pos,Lhip_vel,Rhip_pos,Rhip_vel)

    %calculate the average of Lhip state
    Lhip_posAvg = mean(Lhip_pos);
    Lhip_velAvg = mean(Lhip_vel);

    LhipAvg = [Lhip_posAvg; Lhip_velAvg];
    
    %diff_avg of Lhip_pos & Lhip_vel
    diff_avgLhip_pos = Lhip_pos - Lhip_posAvg;
    diff_avgLhip_vel = Lhip_vel - Lhip_velAvg;

    diff_avgLhip = [diff_avgLhip_pos, diff_avgLhip_vel];
 
    %calculate the average of Rhip state
    Rhip_posAvg = mean(Rhip_pos);
    Rhip_velAvg = mean(Rhip_vel);

    RhipAvg = [Rhip_posAvg; Rhip_velAvg];

    %diff_avg of Rhip_pos & Rhip_vel
    diff_avgRhip_pos = Rhip_pos - Rhip_posAvg;
    diff_avgRhip_vel = Rhip_vel - Rhip_velAvg;
    
    diff_avgRhip = [diff_avgRhip_pos, diff_avgRhip_vel];
     
    %state normalization
    %calculate Xp
    sp1 = zeros(1,length(Lhip_pos)); 
    sp2 = zeros(1,length(Lhip_vel));
    Xp = zeros(length(diff_avgLhip),2);
    
    sd_Lhip_pos = std(Lhip_pos);
    sd_Lhip_vel = std(Lhip_vel);
    
    for i = 1:length(sp1)
        sp1(i) = sd_Lhip_pos;
    end
    
    for j = 1:length(sp2)
        sp2(j) = sd_Lhip_vel;
    end

    Sp1 = diag(sp1); Sp2 = diag(sp2);
    Sp = [Sp1(1,1) 0;0 Sp2(1,1)];
    Xp(:,1) = Sp1 \ diff_avgLhip(:,1);
    Xp(:,2) = Sp2 \ diff_avgLhip(:,2);
    
    %calculate Xh
    sh1 = zeros(1,length(Rhip_pos));
    sh2 = zeros(1,length(Rhip_vel));
    Xh = zeros(length(diff_avgRhip),2);
    
    sd_Rhip_pos = std(Rhip_pos);
    sd_Rhip_vel = std(Rhip_vel);
    
    for m = 1:length(sh1)
        sh1(m) = sd_Rhip_pos;
    end

    for n = 1:length(sh2)
        sh2(n) = sd_Rhip_vel;
    end

    Sh1 = diag(sh1); Sh2 = diag(sh2);
    Sh = [Sh1(1,1) 0;0 Sh2(1,1)];
    Xh(:,1) = Sh1 \ diff_avgRhip(:,1);
    Xh(:,2) = Sh2 \ diff_avgRhip(:,2);
    
end
