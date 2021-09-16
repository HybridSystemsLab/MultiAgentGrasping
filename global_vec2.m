% pos, rot  - position/direction of the local frame's origin in world frame
% variables - (to fit the base frame : origin(0,0), (1,0)-direction)
function vec2_ = global_vec2(pos_vec2, rot_vec2, vec2)

vec2_ = zeros(2,1);
R = zeros(2,2);

R = [rot_vec2(1) -rot_vec2(2);
    rot_vec2(2) rot_vec2(1)];
q = pos_vec2;

T = [R q;
    0 0 1];
    
vec3 = [vec2; 1];
vec3 = T * vec3;

vec2_(1) = vec3(1);
vec2_(2) = vec3(2);

end
