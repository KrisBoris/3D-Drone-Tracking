function dist = pointToEpipolarDistance(pt, line_abc)
    if any(isnan(pt)) || any(isnan(line_abc))
        dist = NaN;
        return;
    end
    A = line_abc(1); B = line_abc(2); C = line_abc(3);
    dist = abs(A*pt(1) + B*pt(2) + C) / sqrt(A^2 + B^2 + eps);
end