function occ = inflate_obstacles(map, r)
% INFLATE_OBSTACLES  Toolbox-free equivalent of imdilate(map, strel('disk',r)).
%
% Each occupied cell paints a filled disk of radius r into the output grid.
% Used to enforce a clearance margin around walls before path planning.

occ = map > 0;
[rows, cols] = find(occ);
[nR, nC] = size(occ);

for k = 1 : numel(rows)
    rr = rows(k);
    cc = cols(k);
    for dr = -r : r
        for dc = -r : r
            if dr*dr + dc*dc <= r*r
                nr = rr + dr;
                nc = cc + dc;
                if nr >= 1 && nr <= nR && nc >= 1 && nc <= nC
                    occ(nr, nc) = true;
                end
            end
        end
    end
end
end
