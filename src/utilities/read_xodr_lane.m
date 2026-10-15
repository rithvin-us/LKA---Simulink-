function road = read_xodr_lane(xodrFile, laneId, ds)
% READ_XODR_LANE Reads one driving lane of an ASAM OpenDRIVE road.
%
% Syntax:
%   road = read_xodr_lane(xodrFile, laneId, ds)
%
% Inputs:
%   xodrFile - path to the .xodr file (first <road> is used)
%   laneId   - OpenDRIVE lane id of the ego lane (e.g. 1 for the aBaja LKA track)
%   ds       - sampling step [m] (default 0.25)
%
% Output:
%   road - ego-lane description (see lka_make_lane_road), driven in the legal
%          direction: with rule="LHT" left lanes (id > 0) follow the reference
%          line, with right-hand traffic right lanes (id < 0) do.
%
% Supported: <line>, <arc>, <spiral> plan-view primitives, constant-polynomial
% lane widths (first <width> record of each lane), <laneOffset> a-coefficient.

    arguments
        xodrFile (1,:) char
        laneId (1,1) double {mustBeInteger, mustBeNonzero}
        ds (1,1) double {mustBePositive} = 0.25
    end

    doc = xmlread(xodrFile);
    roadNode = doc.getElementsByTagName('road').item(0);
    rule = char(roadNode.getAttribute('rule'));

    % ---------------- plan view
    gNodes = roadNode.getElementsByTagName('geometry');
    geom = struct('s0', {}, 'x0', {}, 'y0', {}, 'hdg0', {}, 'length', {}, 'k0', {}, 'k1', {});
    for k = 0:gNodes.getLength - 1
        g = gNodes.item(k);
        G.s0 = str2double(g.getAttribute('s'));
        G.x0 = str2double(g.getAttribute('x'));
        G.y0 = str2double(g.getAttribute('y'));
        G.hdg0 = str2double(g.getAttribute('hdg'));
        G.length = str2double(g.getAttribute('length'));
        if g.getElementsByTagName('line').getLength > 0
            G.k0 = 0; G.k1 = 0;
        elseif g.getElementsByTagName('arc').getLength > 0
            a = g.getElementsByTagName('arc').item(0);
            G.k0 = str2double(a.getAttribute('curvature')); G.k1 = G.k0;
        elseif g.getElementsByTagName('spiral').getLength > 0
            sp = g.getElementsByTagName('spiral').item(0);
            G.k0 = str2double(sp.getAttribute('curvStart'));
            G.k1 = str2double(sp.getAttribute('curvEnd'));
        else
            error('read_xodr_lane:unsupported', 'Unsupported plan-view primitive at s = %g', G.s0);
        end
        geom(end+1) = G; %#ok<AGROW>
    end
    ref = lka_road_from_geometry(geom, ds);

    % ---------------- lanes
    laneOffset0 = 0;
    lo = roadNode.getElementsByTagName('laneOffset');
    if lo.getLength > 0
        laneOffset0 = str2double(lo.item(0).getAttribute('a'));
    end

    if laneId > 0
        sideNode = roadNode.getElementsByTagName('left').item(0);
    else
        sideNode = roadNode.getElementsByTagName('right').item(0);
    end
    lanes = sideNode.getElementsByTagName('lane');
    widths = containers.Map('KeyType', 'double', 'ValueType', 'double');
    for k = 0:lanes.getLength - 1
        ln = lanes.item(k);
        id = str2double(ln.getAttribute('id'));
        wn = ln.getElementsByTagName('width').item(0);
        widths(id) = str2double(wn.getAttribute('a'));
    end

    sgn = sign(laneId);
    inner = 0;
    for id = sgn:sgn:laneId - sgn
        inner = inner + widths(id);
    end
    w = widths(laneId);
    centreOffset = laneOffset0 + sgn * (inner + 0.5 * w);

    lineWidth = 0.125;
    rm = roadNode.getElementsByTagName('roadMark');
    for k = 0:rm.getLength - 1
        lw = str2double(rm.item(k).getAttribute('width'));
        if lw > 0
            lineWidth = lw;
            break
        end
    end

    lht = strcmpi(rule, 'LHT');
    forward = (lht && laneId > 0) || (~lht && laneId < 0);
    [~, fname] = fileparts(xodrFile);
    road = lka_make_lane_road(ref, centreOffset, w, lineWidth, ~forward, ...
        sprintf('%s lane %d', fname, laneId));
end
