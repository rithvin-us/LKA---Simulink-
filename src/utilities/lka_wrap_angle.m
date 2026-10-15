function a = lka_wrap_angle(a)
% LKA_WRAP_ANGLE Wraps an angle to (-pi, pi]. Code-generation compatible.
    a = atan2(sin(a), cos(a));
end
