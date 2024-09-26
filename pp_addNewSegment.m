function path = pp_addNewSegment(path, outerSegment, segmentStart, segmentLength)
    
    % This function inserts a new segment between two points of the path.
    % outerSegment: segment inside which to insert the new segment
    % segmentStart: the distance of the inner segment from the first point
    % of the outerSegment
    % segmentLength: the length of the inner segment

    % Identifica gli estremi del segmento P1 e P2
    segment = 0;
    while segment~=outerSegment
        segment = segment+1;
        P1 = path(segment,:);
    end

    P2 = path(segment+1,:);

    % Compute the direction vector of the original segment
    dir_vector = (P2 - P1) / norm(P2 - P1); % Normalized direction vector
    segmentStart;

    % Compute the start point of the new segment
    Pnew_start = P1 + dir_vector * segmentStart;
    
    % Compute the endpoint of the new segment
    Pnew_end = Pnew_start + dir_vector * segmentLength;
   
    % Add the new segment to the path
    % If the segment start coincides with the start of the outer segment
    % Only add the end of the segment to the outer segment
    if segmentStart~=0
        path = [path(1:outerSegment,:); Pnew_start; Pnew_end; path(outerSegment+1:end,:)];
    else
        path = [path(1:outerSegment,:); Pnew_end; path(outerSegment+1:end,:)];
    end
end