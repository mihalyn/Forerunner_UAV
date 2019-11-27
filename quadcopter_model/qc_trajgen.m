function [ varargout ] = qc_trajgen( xyz, xyz0, tmax,t_trajsamp )
%QC_TRAJGEN Builds trajectory from samples
%   position
%   velocity function

    q = csape(0:(length(xyz)-1), [xyz0 xyz xyz0], 'clamped');
    qd = fnder(q);
    qdd = fnder(qd);

    % Spline length calculation - r(s) function

    len = 100; %10 for test
    clear parts
    diff=zeros(length(q.breaks)-1,len);

    
    for i = 1:(length(q.breaks)-1)
        parts(i) = fnbrk(q,i);
        part1 = parts(i).breaks(1);
        part2 = parts(i).breaks(end);
        for j = 1:len
            pt1 = fnval(parts(i), ((j-1)/len)*(part2-part1)+part1);
            pt2 = fnval(parts(i), (j/len)*(part2-part1)+part1);
            diff(i,j) = sqrt((pt1-pt2)'*(pt1-pt2));
        end
    end
    sums = sum(diff,2);
    sums = [0;sums];
    j = 0;
    
    arclen=zeros(1,length(sums));
    for i = 1:length(sums)
        j = j + sums(i);
        arclen(i) = j;
    end

    r = csape(arclen, q.breaks, 'variational');
    rd = fnder(r);
    rdd = fnder(rd);

    % Velocity profile - s(t) function

    s = csape([0 tmax/2 tmax], [0 0 arclen(end)/2 arclen(end) 0], 'clamped');
    sd = fnder(s);
    sdd = fnder(sd);

    clearvars pts xyz xyz0 i j pt1 pt2 diff len part1 part2 sums parts arclen

    % Sampling - q(r(s(t))) -> q(t) function

    t = 0:t_trajsamp:tmax; %%FIXME! WHY 50 samples?
    sval = fnval(s,t);      % containing velocities
    rval = fnval(r,sval);	% containing section lengths
    qval = fnval(q,rval);	% containing points

    s = csape(t,[qval(1,:); qval(2,:); qval(3,:)], 'variational');
    sd = fnder(s);
    sdd = fnder(sd);

    varargout{1} = s;
    varargout{2} = sd;
    varargout{3} = sdd;
end

